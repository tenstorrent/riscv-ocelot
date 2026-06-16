Frontend
========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


The Frontend Stages
-------------------

The frontend includes 2 stages the Instruction Fetch and Decoder. Nominally the IF should consume
5 cycles and the Decoder 1 cycle, with support for super-scalar issue.

Instruction Fetch
-------------------

The Instruction fetch stage is unchanged from BOOMv4. The reader may review the BOOMv4 IFU here: 
`BOOM IFU docs <https://docs.boom-core.org/en/latest/sections/instruction-fetch-stage.html>`_.

.. _vector-rvv-decode:
Vector (RVV) Decode
-------------------

The Decoder is extended to support RVV1.0 instruction opcodes. The decoder itself is single cycle 
combinational module which produces the decoded uOP packet bundle. For |caracal|, parameterizable 
super-scalar decoder support is also maintained.

.. note::

   A uOP packet in BOOMv4 is simply a decoded instruction. The decoder does not perform any cracking 
   or micro-op expansion; all instructions, including vector instructions, are decoded into a single 
   uOP.

.. list-table::
   :header-rows: 1
   :widths: 1 2

   * - Parameter
     - Value
   * - RVV spec version
     - RVV1.0
   * - VLEN
     - 256
   * - ELEN
     - 64
   * - Supported SEW(s)
     - 8, 16, 32, 64
   * - Supported LMUL(s)
     - ALL
   * - Vector regfile depth
     - 128

CII Shared Instruction Decoding
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Certain instructions may require shared resources between the vector load store units and
the vector co-processor. These instructions are marked as *is_shared* by the Decoder and
require special handling. 

Currently the only instruction type that will be marked *is_shared* is the segmented load store.
This is because segmented load stores must access memory using the LSU, and be transposed
using the co-processor transpose datapath.

The primary differentiator for shared instructions is their use of temporary vector registers
to hand off partial results between the the LSU and Co-processor via the bypass network. 
Shared instructions are only applicable to RVV1.0 instructions as only the CII attached co-processor
may want to read temporary results from the LSU and vice-versa.


VSET Special Handling
~~~~~~~~~~~~~~~~~~~~~~

Because we delay vector uOP cracking until the execution stage of the pipeline, 
the vector mapper must in advance allocate enough vector PRNs for a specific 
instruction based on LMUL/EMUL. Thus all uOP's emitted by the decoder must contain
the calculated EMUL of the instruction. This EMUL information is used by the 
Mapper to allocate PRN vector register groups.

We introduce a new unit called **Vector Config Unit**. The VCFG keeps a local copy 
of the VTYPE and VL values set by a vset instruction, so that dependent, subsequent 
instructions can know the most recent value of vset and be included in the uOP. 
The VCFG will also store performance counters and other related CSRs for vector
instructions. The VCFG must also handle NxWide instruction decoder width. When more than
one vset instruction appears in a single decode bundle, the selection is **per lane**: each
vector uOP uses the **nearest preceding** vset in program order within the bundle (a prefix
select across the lanes), not simply the globally newest one. For example in
``[vsetvli, vadd, vsetivli, vadd]`` the first ``vadd`` uses the ``vsetvli`` config and the
second uses the ``vsetivli`` config.

**vsetivli** is the best case instruction as the VTYPE and VL is an immediate value 
and can be immediately decoded and stored in the VCFG for decoding of subsequent vector instructions.

**vsetvli** is the most common case where VTYPE is an immediate but VL is supplied 
by a integer source register. In this case the decoder does not need to stall or wait 
for VL instead the uOP VL type is marked as a register, and the scalar value is
resolved at the issue stage by the VL Broadcast Unit (VLBU). Once vl is resolved
it is broadcast to the vectro config unit to be used by later vector instructions
that rely on it.

**vsetvl** is a special case as both the VTYPE and VL is a scalar source operand instead
of an immediate encoded in the instruction. In this case the newer vector uOP cannot
continue to be issued as the VTYPE value is needed by the next pipeline stage. Concretely, the
vector mapper needs VTYPE to derive EMUL and allocate the correct number of vector PRNs per group;
this is why **vsetvli** (VTYPE immediate, EMUL known at decode) need not serialize, while
**vsetvl** (VTYPE from a register) must.

To solve this issue we re-use the |boom| ``is_unique`` feature, and mark vsetvl as a unique
instruction. This forces all instructions in the BOOM pipeline to complete before vsetvl 
can be issued by the decoder. This allows the scalar instructions that calculate the vtype 
and VL value to complete, and the VCSRU reads this value and updates its local copy of VTYPE, 
VL and use it for subsequent vector uOP decoding.

This does result in poorer performance for this instruction, but this is acceptable as this vsetvl instruction type is not common.

VCFG Mirror Recovery (speculative vtype/vl)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The VCFG mirror is **speculative decode-time state**: it is updated in program order as
``vset*`` uop's decode, and every younger vector uop snapshots it into its ``vconfig``.
Because the mapper derives ``EMUL`` from the mirror's ``vtype`` to size the PRN group it
allocates, a ``vset*`` on a **mispredicted path** that updates the mirror would otherwise
corrupt the ``EMUL`` of every surviving younger vector uop — a silent miss-size of the
vector register group. The mirror therefore needs the **same recovery machinery as the
RMT**, not just the RMT itself. |caracal| mirrors |boom|'s speculative/committed RMT pair:

1. **Speculative VCFG mirror** — the working ``vtype``/``vl``/``vstart`` copy updated at
   decode. This is the structure described above. The per-lane nearest-preceding-``vset``
   prefix select already resolves the in-bundle program order, so the value each branch
   "sees" is well defined.

2. **Committed VCFG shadow** — updated **only** by the ROB when a ``vset*`` uop commits
   (the same retire event that writes the architectural ``vtype``/``vl`` CSRs). It holds the
   known-good architectural vector config.

3. **Per-``br_tag`` snapshot** — on the same ``ren_br_tags`` allocation event that snapshots
   the scalar/vector RMTs, the speculative mirror is snapshotted into a
   ``maxBrCount``-deep array (≈24 bits/entry — ``vtype`` + ``vl`` + ``vstart`` — small).

Recovery is then identical in shape to the RMT (see :ref:`snapshots`):

.. list-table::
   :header-rows: 1
   :widths: 1 1 2

   * - Event
     - Recovery source
     - Cost
   * - Branch mispredict
     - ``vcfg_snapshots(br_tag)``
     - 1 cycle, restores the speculative mirror in lockstep with the RMT restore
   * - Exception / pipeline flush
     - committed VCFG shadow
     - 1 cycle, parallel to ``map_table := com_map_table``

Because the mirror physically lives a stage ahead of ``br_tag`` allocation (decode vs.
rename), the snapshot is written from the **delayed** ``br_tag`` that reaches the mapper
stage — the same delayed-``br_tag`` path used for the vector RMT snapshot (see
:ref:`rename-twostage`) — so the snapshotted value reflects all ``vset*`` updates older than
the branch in program order.

VL delivery — VL is just an integer register
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There is no separate VL physical-register namespace. The VL value lives in an **ordinary integer
physical register** — the integer destination of the VL-producing instruction — written back by the
integer EU like any scalar result. Two pieces make this usable downstream:

- **Current-VL-PRN tracker.** A 1-entry tracker in rename records the integer destination PRN of the
  most recent VL-producing instruction. Every younger vector uOP carries that PRN as an **implicit
  integer source operand** (``pvl``); it is read/woken on the **integer** wakeup network like any
  other scalar feeder (see the Vector Issue Slot). The tracker is **branch-snapshotted per ``br_tag``
  and restored on mispredict**, exactly like the integer map table, so a vset on a squashed path does
  not corrupt the VL mapping of the surviving path.
- **VLBU capture.** When the integer EU writes back that PRN, the VL Broadcast Unit (VLBU) snoops the
  integer writeback **data** lane and captures the VL value into every waiting vector issue slot whose
  ``pvl`` matches, clearing ``pvl_busy``. No separate broadcast bus is added.

Because VL is an integer destination, a destination PRN is **always allocated** for the VL value —
**even when ``rd == x0``** — so the value remains tappable by the VLBU. The four ``vsetvli`` sub-cases
follow directly:

- ``rs1 != x0`` → VL = min(rs1, VLMAX); register-sourced, delivered by the VLBU.
- ``rs1 == x0, rd != x0`` → VL = VLMAX; statically known, but still written to the integer dest PRN
  (the slot may pre-load it from the ``VConfig`` snapshot rather than wait on the VLBU).
- ``rs1 == x0, rd == x0`` → **keep VL unchanged**; this is *not* a VL producer, so the current-VL-PRN
  tracker is left untouched and younger uOPs keep the existing ``pvl``.
- Otherwise (``rd == x0`` with a VL change) a destination PRN is still allocated to hold VL.
