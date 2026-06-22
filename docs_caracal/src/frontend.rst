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

The primary differentiator for shared instructions is their use of an intermediate temp vector group
(``pvtmp``, allocated in the VRF by the vector mapper) to hand off partial results between the LSU
and Co-processor. Shared instructions are only applicable to RVV1.0 instructions as only the CII
attached co-processor may want to read intermediate results from the LSU and vice-versa.


VSET Special Handling
~~~~~~~~~~~~~~~~~~~~~~

Because we delay vector uOP cracking until the execution stage of the pipeline, 
the vector mapper must in advance allocate enough vector PRNs for a specific 
instruction based on LMUL/EMUL. Thus all uOP's emitted by the decoder must contain
the calculated EMUL of the instruction. This EMUL information is used by the 
Mapper to allocate PRN vector register groups.

We introduce a new unit called **Vector Config Unit**. The VCFG keeps a local copy
of the **VTYPE** value set by a vset instruction, so that dependent, subsequent
instructions can derive ``EMUL`` and snapshot ``vtype`` into the uOP at decode. The VCFG
mirrors only ``vtype`` (plus vector performance counters) — it does **not** mirror ``vl``: VL is
renamed into the VL register file and delivered via ``pvl`` (see VL delivery below), so there is no
decode-time VL value at all. The VCFG is **not** the architectural CSR store; the architectural
vector CSRs — ``vstart``, ``vxrm``, ``vxsat``, ``vcsr``, ``vl``, ``vtype``, and read-only
``vlenb`` — live in the normal CSR file and are updated precisely at commit (``vstart``/``vxrm``/
``vxsat`` are read from the CSR file at execute, not snapshotted in the uOP).
 
**Where each vset executes.** Only ``vsetivli`` is **front-end only**: both VTYPE and AVL are
immediate, so the VCFG resolves it at decode in a single cycle — updating the ``vtype`` mirror and
writing the computed VL (``min(uimm, VLMAX)``) into the renamed VL RF — with **no back-end issue
slot or EU**.

``vsetvli`` (register AVL) and ``vsetvl`` (register VTYPE and AVL) **execute on an integer ALU EU**,
because their VL (and ``vsetvl``'s VTYPE) depends on a runtime register value. They are dispatched to
a scalar integer issue queue and woken by ``rs1`` (and ``rs2`` for ``vsetvl``) on the **integer**
wakeup network like any integer op; the ALU reads the source(s) from the integer RF/bypass, computes
``VL = min(rs1, VLMAX)`` (and VTYPE for ``vsetvl``), and on writeback **targets the VL RF** and drives
the **VL wakeup network** (``pvl``). For ``vsetvl`` the EU also updates the VCFG ``vtype`` mirror,
which is safe because ``vsetvl`` is ``is_unique`` (the pipeline is drained, so the mirror update lands
before any younger vector uOP decodes). When ``rd != x0`` the ALU additionally writes ``rd`` in the
integer RF as normal. So a register-sourced vset is an ordinary integer-ALU uop whose result fans out
to the VL RF (+ ``rd``); no dedicated VCFG wakeup network is added — it consumes the integer network
and produces on the VL network. This integer-ALU extension (VL/VTYPE compute + VL-RF writeback) is
**gated by ``usingRVV``**: with the switch off, the ALU is bit-identical to |boom| v4.

The VCFG must also handle NxWide instruction decoder width. When more than
one vset instruction appears in a single decode bundle, the selection is **per lane**: each
vector uOP uses the **nearest preceding** vset in program order within the bundle (a prefix
select across the lanes), not simply the globally newest one. For example in
``[vsetvli, vadd, vsetivli, vadd]`` the first ``vadd`` uses the ``vsetvli`` config and the
second uses the ``vsetivli`` config.

**vsetivli** has an immediate VTYPE and AVL: VTYPE updates the VCFG mirror at decode, and the
computed VL (``min(AVL, VLMAX)``) is written to the VL register file and renamed (``pvl``) like any
VL producer — there is no decode-time VL fast path.

**vsetvli** is the most common case where VTYPE is an immediate but VL is supplied
by a integer source register. VTYPE updates the VCFG mirror at decode, but VL needs ``rs1``, so the
instruction **executes on an integer ALU EU**: it is woken by ``rs1`` on the integer network, the ALU
computes ``min(rs1, VLMAX)``, and its writeback targets the VL RF — waking ``pvl`` for dependent
vector uOPs on the VL wakeup network. VL is **not** broadcast back to the VCFG; younger vector uOPs
carry ``pvl`` and read it from the VL RF at execute.

**vsetvl** is a special case as both the VTYPE and VL is a scalar source operand instead
of an immediate encoded in the instruction. In this case the newer vector uOP cannot
continue to be issued as the VTYPE value is needed by the next pipeline stage. Concretely, the
vector mapper needs VTYPE to derive EMUL and allocate the correct number of vector PRNs per group;
this is why **vsetvli** (VTYPE immediate, EMUL known at decode) need not serialize, while
**vsetvl** (VTYPE from a register) must.

To solve this issue we re-use the |boom| ``is_unique`` feature, and mark vsetvl as a unique
instruction. This forces all instructions in the BOOM pipeline to complete before vsetvl 
can be issued by the decoder. This allows the scalar instructions that calculate the vtype 
and VL value to complete, and the VCFG reads this value and updates its local copy of VTYPE, 
VL and use it for subsequent vector uOP decoding.

This does result in poorer performance for this instruction, but this is acceptable as this vsetvl instruction type is not common.

Explicit vector-CSR accesses
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Every **explicit** vector-CSR access — any ``csrr``/``csrw``/``csrrw``-family instruction targeting
``vstart``, ``vxrm``, ``vxsat``, ``vcsr``, ``vl``, ``vtype``, or ``vlenb`` — is decoded as
``is_unique`` and serialized, reusing the same drain mechanism as ``vsetvl``. This guarantees
correctness with no speculative vector-CSR path: a read returns the committed value and a write
takes effect before any younger uOP observes it. These accesses are rare, so the serialization cost
is acceptable. Implicit updates are unaffected and use their normal paths: ``vtype``/``vl`` from
``vset``, ``vstart`` set on a trap, and the sticky ``vxsat`` accumulated by the coprocessor — all
committed precisely.

VCFG Mirror Recovery (speculative vtype)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The VCFG mirror is **speculative decode-time state**: it is updated in program order as
``vset*`` uop's decode, and every younger vector uop snapshots its ``vtype`` into its
``vconfig``. Because the mapper derives ``EMUL`` from the mirror's ``vtype`` to size the PRN
group it allocates, a ``vset*`` on a **mispredicted path** that updates the mirror would
otherwise corrupt the ``EMUL`` of every surviving younger vector uop — a silent miss-size of the
vector register group. The mirror therefore needs the **same recovery machinery as the
RMT**, not just the RMT itself. |caracal| mirrors |boom|'s speculative/committed RMT pair:

1. **Speculative VCFG mirror** — the working ``vtype`` copy updated at decode. The per-lane
   nearest-preceding-``vset`` prefix select already resolves the in-bundle program order, so the
   value each branch "sees" is well defined. (``vl`` is **not** mirrored — it is renamed into the VL
   register file, which has its own snapshot/recovery, see :ref:`vl-vtype-rename`; ``vstart`` lives
   in the CSR file.)

2. **Committed VCFG shadow** — updated **only** by the ROB when a ``vset*`` uop commits
   (the same retire event that writes the architectural ``vtype`` CSR; ``vl`` is committed via the
   VL register file's commit path). It holds the known-good architectural ``vtype``.

3. **Per-``br_tag`` snapshot** — on the same ``ren_br_tags`` allocation event that snapshots
   the scalar/vector RMTs, the speculative mirror is snapshotted into a
   ``maxBrCount``-deep array (≈9 bits/entry — ``vtype`` only — small).

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

VL delivery — VL lives in its own register file
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

VL is renamed into a dedicated **VL register file**, not the integer RF (see
:ref:`vl-vtype-rename`). A VL-producing instruction allocates a fresh ``VL`` PRN, writes the new VL
value into the VL RF, and broadcasts ``pvl`` on the VL wakeup network. Every younger vector uOP
carries the current ``pvl`` (from the VL map table at rename) as an implicit operand, woken on the
**VL** wakeup network and read from the VL RF by the vector EU at execute. The VL map table is
branch-snapshotted per ``br_tag`` and restored on mispredict, so a vset on a squashed path does not
corrupt the VL mapping of the surviving path.

The four ``vsetvli`` sub-cases:

- ``rs1 != x0`` → VL = min(rs1, VLMAX); the producer reads ``rs1`` from the integer RF, computes VL,
  and writes the VL RF.
- ``rs1 == x0, rd != x0`` → VL = VLMAX; the producer computes VLMAX from ``vtype`` and writes the
  VL RF (no decode-time fast path).
- ``rs1 == x0, rd == x0`` → **keep VL unchanged**; *not* a VL producer, so the VL map table is left
  untouched and younger uOPs keep the existing ``pvl``.
- Otherwise (``rd == x0`` with a VL change) a VL PRN is still allocated to hold VL.

In every producing case VL is written to the VL RF and read back via ``pvl`` — there is no
statically-known-VL path that bypasses the VL RF.

In all producing cases, when ``rd != x0`` the ``vset`` also writes ``rd`` as an ordinary integer
destination (normal, unmodified integer rename); vector consumers read VL only from the VL RF.
