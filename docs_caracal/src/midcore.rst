Midcore
=======

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC

The Rename Stage
----------------

The Rename Stage is extended to support vector register renaming. For vector instructions the rename stage is split into two pipeline stages **Scalar Mapping/Rename** and **Vector Mapping/Rename**.

In the scalar mapping the original |boom| implementation is unchanged, there still exists a integer rename_stage and fp_rename_stage. Scalar instructions may enter the rename stage and be dispatched normally to the scalar instruction queues.

Vector instructions will also enter the rename_stage and fp_rename_stage and have any scalar destination or source registers assigned a int or fp PRN. For vsetivl the VL register will require a integer PRN to be allocated for the VL value, which will be resolved at the issue stage by the VL Broadcast Unit.

.. _rename-twostage:

Dispatch-group atomicity across the two stages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Because vector mapping takes an extra cycle, a vector ``OP.v`` would otherwise reach dispatch one
cycle after a scalar uop in the same dispatch group. |caracal| does **not** insert a 1-cycle bubble
on the scalar path to re-align them. Instead the ROB allocation is split from the vector metadata
fill:

- **Scalar Mapping (cycle 1)** allocates the ROB entry in program order — for both scalar and vector
  uops — and assigns the ``br_tag`` and any scalar PRNs. The dispatch group is therefore allocated
  atomically and in order exactly as in |boom|; branch-tag allocation is unchanged.
- **Vector Mapping (cycle 2)** fills the vector-specific metadata (``pvdest``, the ``EMUL`` stale
  group, ``pvs*``/``pvm``, ``v_emul``, etc.) into the **already-allocated** ROB entry the next cycle.

The ROB entry is thus reserved by the in-order scalar stage and completed a cycle later by the vector
stage; no bubble is needed and dispatch-group integrity is preserved. The only constraint is that the
vector metadata write lands before the entry can complete/issue, which it always does since vector
mapping is exactly one cycle behind.

Rename Map Table (RMT)
~~~~~~~~~~~~~~~~~~~~~~~

|boom| maintains two sets of register rename map tables **Speculative RMT** and **Committed RMT**. The Committed RMT is an optional feature, which we we will always keep enabled in |caracal|.

1. Speculative RMT — ``map_table``

   This is the working table updated at rename time as physical destinations are allocated. It reflects in-flight, not-yet-committed instructions. Source operands are read from it (``map_resps``).

2. Architectural / Committed RMT — ``com_map_table``

   Updated only by committed uop's (driven from the ROB via ``com_remap_reqs``). It holds the known-good architectural state.

When an exception occurs, the ``com_map_table`` is copied to the speculative ``map_table``. Because the ``com_map_table`` only contains the latest committed ARN to PRN mappings it holds the most recent correct state before the trapping instruction, thus instead of a ROB 1 entry/cycle walk back a single cycle copy is all that is needed to roll back the ``map_table``.

The vector mapper extends the existing RenameStage implementation to support vector renaming. The vector mapper implements atomic LMUL/EMUL based vector register mapping. The mapper may rename and allocate up to LMUL=8 PRNs for a single vdest. This removes the need for vector uOP cracking after the decoder and reduces the number of ROB entries required for vector instructions.

.. note::

   The Vector Mapper should also get the old stale vdest group and update the OP.v ``stale_pvdest`` field — a ``Vec`` of up to ``EMUL`` stale PRNs, not a single reg, since a vector dest renames a whole group. This will assist in handling tail undisturbed instructions and lets commit free the entire stale group. This stale-group capture happens in the cycle-2 vector-metadata fill into the ROB entry (see :ref:`rename-twostage`).

The vector mapper adds 1 feature:

**1. LMUL TAG Whole Vector Group Checker**

The vector map table stores one PRN per architectural vreg, so an EMUL-wide read returns 
the group's current mappings directly and is correct under arbitrary fragmentation. On top 
of this we add an additional structure to the RMT for vector remapping. The LMUL tag checker
adds a 2 bit tracking table for each of the ARN indexes to track which vector register group 
the ARN index belongs to. The tag checker table will consume 32x2 bits, and is used to validate 
that a read group is whole.

.. list-table::
   :header-rows: 1
   :widths: 1 1

   * - LMUL
     - Encoding
   * - LMUL=1
     - 2'b00
   * - LMUL=2
     - 2'b01
   * - LMUL=4
     - 2'b10
   * - LMUL=8
     - 2'b11

We can exploit the fact that vector register groups must be begin on ARN indexes that must be a multiple of the effective LMUL. So we build combinational checkers that check whether or not each valid ARN base index represents a whole vector group or not.

For example to check vector group Tags for LMUL = 8:

.. code-block:: python

   LMUL = 8
   vdest = [0, 8, 16, 24]              # valid vector-group base ARNs for LMUL=8
   is_whole_vg_8 = [0] * (32 // LMUL)  # one flag per group

   for i, vd in enumerate(vdest):
       is_whole_vg_8[i] = int(
           tag_table[vd]     == tag_table[vd + 1] == tag_table[vd + 2] ==
           tag_table[vd + 3] == tag_table[vd + 4] == tag_table[vd + 5] ==
           tag_table[vd + 6] == tag_table[vd + 7] == 0b11
       )

The same combinational check is performed for other LMULs and their valid vdest indexes. For LMUL=1 all ARN indexes would always return true.

If the check for a is_whole_vg_[1,2,4,8] returns TRUE, then the vector mapper can immediately 
read the RMT and get the most recent vsrc PRN mappings for an entire vgroup and update the OP.v 
with the PRN indexes.

If the check for a is_whole_vg_[1,2,4,8] returns FALSE, the group has been fragmented by an
intervening narrower write. Because the map table stores one PRN per architectural vreg, the
EMUL-wide read is still correct, so no exception is raised and no recovery is needed — the read
simply proceeds. The checker is retained purely as an **observability counter**: a performance
counter increments each time a non-whole group is read. This is expected to be extremely rare for
compiler-generated code, and the counter lets perf analysis quantify how often it happens.



CII Shared Instruction Mapping
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Up to this point both the scalar and vector mapper treat shared instructions as any 
other instruction. The differences in handling begin at the Issue Stage (see the CII Shared
Instruction Scheduling section).

Shared vector instructions are still allocated PRN's and ROB entries as normal instructions.


.. _free-list:

Free List
~~~~~~~~~


The Free List tracks the physical registers that are currently un-used and 
is used to allocate new physical registers to instructions passing through the Rename stage

Modification is required for the Vector Free List. Because we rename whole vector groups
atomically, the vector free list allocates an entire EMUL group per OP.v. It reuses |boom|'s
``SelectFirstN`` selector with ``allocWidth = coreWidth*8``; the selected pregs are
**non-contiguous** (no contiguous-run allocator is needed) and are grouped per OP.v. Branch and
commit reclaim are unchanged from the scalar free list — allocations are bit-vector ORs, so
setting 8 bits per OP.v needs no new logic, only wider ports (``deallocWidth = commitWidth*8`` to
free a whole stale group at commit).


Busy Table
~~~~~~~~~~

The Busy Table tracks the readiness status of each physical register. If all physical operands are
ready, the instruction will be ready to be issued. The scalar Busy Table does three things: it
**sets** one busy bit per allocated ``pdst``, **clears** a bit per writeback port, and is **read**
for the source operands (``prs1``/``prs2``/``prs3``) of each rename lane.

The Vector Busy Table operates on the same principle over ``numVecPhysRegisters`` bits, but each of
the three sides scales differently — and only the *set* side scales like the :ref:`free-list`:

**1. Set-busy (allocation) **
   A vector dest is a whole ``EMUL`` group, so up to 8 busy bits are set per vector ``OP.v``. Because
   the free list allocates **non-contiguous** PRNs, the set mask is the OR of ``UIntToOH(pvdest_j)``
   over the group's PRNs, up to ``coreWidth*8`` bits/cycle.


**2. Source reads **
   The Busy Table is read for every source. A vector source is a
   group, so each rename lane reads the busy status of an ``EMUL``-wide group per vector source:
   ``pvs1[8]``, ``pvs2[8]``, ``pvs3[8]`` (the old ``vd`` for tail/mask-undisturbed) and the mask
   ``pvm[1]`` — on the order of 25 bit-reads per lane, times ``plWidth``. The in-flight wakeup bypass
   is replicated per group member. This is the dominant cost of the Vector Busy Table.

**3. Clear-busy (writeback)**
   Each vector writeback completes ONE PRN, so the clear side is ``numVecWbPorts`` wide,
   the same shape as the scalar table — not a group-wide clear.

**Group-readiness aggregation .**
   Because the mapper is atomic-group — no cracking, one ROB entry, **one issue slot** per ``OP.v`` —
   a single slot operand (e.g. ``pvs1``) stands for an 8-PRN group. The Busy Table reads 8 per-PRN
   bits, but the slot tracks one readiness signal per operand, so the per-PRN bits must be **AND-ed
   into a single group-ready bit** and the operand woken only when the **last** group member writes
   back. This is the busy-table-side counterpart of the ROB's group completion counter.

Accordingly the Vector Busy Table interface widens: ``busy_resps`` carries per-group source readiness
(``pvs1``/``pvs2``/``pvs3``/``pvm``, aggregated to group-ready bits); ``ren_uops`` carries the group
bases plus ``EMUL``; ``rebusy_reqs`` set up to 8 bits per ``OP.v``; and ``wakeups`` is
``numVecWbPorts`` wide.



Reorder Buffer (ROB)
~~~~~~~~~~~~~~~~~~~~~

|caracal| reuses |boom| v4's ROB structurally unchanged. The only field change is
widening the entry's ``dst_rtype`` to 3 bits (``exu/rob.scala``) to encode ``RT_VEC``,
so vector uop's allocate, commit, and roll back through the same
head-pointer/exception machinery as scalar ops.

Because the vector mapper allocates whole ``LMUL``/``EMUL`` register groups atomically
(see :ref:`free-list`), a vector instruction is **not** cracked into one uop per
destination register. It therefore occupies **a single ROB entry**, regardless of
``EMUL``. This keeps the ROB small under wide vectors and lets vector instructions
retire in one commit slot.

Completion tracking
^^^^^^^^^^^^^^^^^^^^

A single ROB entry that owns an ``EMUL``-wide destination group needs to know when
**all** of the group's writes are done before it can clear its busy bit and commit.
How that is detected differs between the two vector paths:

Vector arithmetic (tt_CII)
   The vector ALU path executes **in program order on the in-order tt_CII
   coprocessor** (see :ref:`vector-execution`). An in-order unit knows when an entire
   ``op.v`` has retired, so it signals completion **once per instruction**. This maps
   directly onto the existing single-writeback busy-clear in ``rob.scala`` — **no
   per-destination counter is required** for CII ops. The ROB treats a CII completion
   exactly like a scalar writeback: one wakeup clears ``rob_bsy`` for the entry.

Out-of-order vector load/store (VLS)
   The vector memory path generates element/segment sub-accesses that complete
   **independently and out of order** through the LSU. Here a single writeback is not
   sufficient — the entry must remain busy until every sub-access has reported back.
   For this path the ROB entry carries a small **completion counter** (4 bits;
   ``EMUL ≤ 8``, and segment ``NF·EMUL ≤ 8``). The counter is initialized at dispatch to
   the number of expected destination writes — derived from ``v_emul`` and ``v_seg_nf``,
   **not** raw ``EMUL`` (widening/narrowing change the destination group size, and
   mask/reduction results are a single register). Each writeback (or ``lsu_clr_bsy``
   for stores) increments the counter by the number of ports matching the entry that
   cycle; ``rob_bsy`` is cleared only when the counter reaches its target. Scalar ops
   are the ``target = 1`` case and behave exactly as today.

Shared instruction (segmented LS)
   A shared instruction occupies **two issue slots** — one in the CII IQ (coprocessor half) and
   one in the vector load/store IQ (LSU half) — but **a single ROB entry**. The entry stays busy
   until **both** halves report. This reuses the VLS completion counter above: it is initialized to
   expect the LSU sub-access completions **and** the coprocessor completion, and ``rob_bsy`` clears
   only when the counter reaches that combined target. The two halves are sequenced by the TVRB
   bypass — the producer (LSU for a segmented load, coprocessor for a segmented store) writes the
   TVRB temp, whose available-broadcast wakes the consumer's IQ slot (see the CII Shared Instruction
   Scheduling section). For a segmented **store** the LSU half's actual D$ writes are still
   post-commit, exactly as for any store.

The ``rob_unsafe`` (speculation-hazard) bit is gated the same way for VLS: a
multi-access vector load is not memory-safe until **all** of its sub-accesses have
disambiguated, so ``rob_unsafe`` is cleared only once the whole group reports safe,
not on the first sub-access.


Commit
~~~~~~

At retirement a vector entry frees its entire **stale destination group** — ``EMUL``
stale vector pregs, not one. Because allocation is non-contiguous, the ROB carries the ``EMUL``
stale PRNs captured at rename (an explicit stale group, not a base+count) alongside the new
``pvdest``. This couples to the free-list deallocation path described in :ref:`free-list`.


.. _snapshots:

Checkpoint RMT Snapshots
~~~~~~~~~~~~~~~~~~~~~~~~

BOOM implements a branch snaphot mechnism to roll back the RMT upon branch mispredicts in a 
single cycle. A copy of the speculative scalar RMTs is taken per outstanding branch 
(upto maxBrCount entries).

We will reuse this mecahnism for the vector mapper.

The vector mapper takes a snapshot of the speculative vector RMT on the same event the scalar
RMT is branch-snapshotted (per outstanding branch, up to maxBrCount). There is no periodic
snapshot — the per-ARN map table needs no fragmentation-recovery walk.

.. list-table::
   :header-rows: 1
   :widths: 1 1 2

   * - Event
     - Recovery source
     - Cost
   * - Branch mispredict
     - ``br_snapshots(br_tag)``
     - 1 cycle, only flushes younger-than-branch
   * - Exception / pipeline flush
     - ``com_map_table``
     - 1 cycle, flushes everything in flight
   * - Normal
     - ``remap_table``
     - advance




Precise exceptions for vector ops
---------------------------------

A vector load/store can fault partway through its element stream (e.g. element *k* raises a page fault). 
Because the whole instruction is one ROB entry, the trap must be made restartable by recording ``vstart = k`` 
for the **oldest** faulting element. The single ``rob_exception`` bit is therefore augmented on the VLS path 
with the faulting element index (carried in on the LSU exception port).

On trap, ``vstart`` is written from this index so the instruction resumes mid-stream per the RVV 1.0 spec. 
CII arithmetic ops, which complete atomically, raise exceptions at instruction granularity and need no element index.

On vector loads if we load upto *k* elements and an exception occurs, we do not need to 
restore the remaining VL - *k* elements and can treat it as tail agnostic. 
This is thanks to the relaxation of rules for precise exception handling in section 17.1 RVV Specification.


Segmented Load/Store (Shared Instruction)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In addition, to the above procedures segmented load store instructions must also invalidate any TVRB entries which are
allocated for its execution. Segmented load stores should still commit data up until the point of exception. For segmented loads
this means the vsrc temporary register should be written to up until the point of execution and tail values may hold junk.
The Coprocessor should still read and free the vsrc temporary register and write the result to the VPRF. A segmented store will
always read and free the vsrc temporary register, but may only write data to memory up until the point of exception.


The Register Files and Bypass Network
-------------------------------------

|caracal| shall be configured to support an performant number of physical registers via
parameterization. 

.. list-table::
   :header-rows: 1
   :widths: 1 1 1

   * - Register file
     - Architectural regs
     - Physical regs (default)
   * - Integer (INT)
     - 32
     - 128
   * - Floating-point (FP)
     - 32
     - 128
   * - Vector (VEC)
     - 32
     - 128

.. note::

   |caracal|/|boom| has no dedicated mask **register file**; Vector mask is treated 
   like any other vector register, with masking semantics handled in the execution units.



Vector Bypass Network and Temporary Register Buffer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|boom| supports a full operand forwarding bypass network for the integer and FP pipeline.
We also must implement the bypass network for the vector pipeline. However, the vector bypass 
network extends the scalar bypass networks with a **Temporary Vector Register Buffer** (TVRB).
The Temporary Vector Register Buffer serves an important role to support temporary vector registers
and shared instructions. The TVRB will be a relatively small register file parameterized to support a
default 16 temporary vector registers.

A shared instruction is assigned a **TVRB tag** — an index in a namespace **separate from the main
vector PRNs**, drawn from the TVRB's own free list. The tag is carried in the ``OP.v``'s ``vsrc``
field so the producing and consuming EUs rendezvous on the same TVRB entry. Because the tag is
**never installed in the vector RMT and never drawn from the main vector free list**, no
architectural vector read can ever alias a TVRB temp — the two namespaces are disjoint by
construction. (Where the segmented-LS steps below say "vsrc PRN," they mean this TVRB tag, not a main
vector PRN.) The TVRB maps the tag to a ``TVRB_RF`` index; free indexes are tracked with the TVRB
free list.

A ``TVRB_RF`` entry is **deallocated on consume** — when the consuming EU reads it using the tag, the
index is marked free. Since the temp is never an RMT mapping there is **no commit-time stale-free
path** for it; free-on-consume (plus the branch-kill reclaim below) is the only deallocation.

**Branch-mispredict kill.** The read-based deallocation above only frees an entry once its consumer
reads it. If the shared instruction that allocated the entry is squashed by a branch mispredict
*before* the consumer reads — or the producing EU is itself squashed before it ever writes — that
entry would leak, slowly draining the 16-entry pool until it deadlocks. To prevent this, each TVRB
entry carries the ``br_mask`` of the ``OP.v`` that allocated it, and the TVRB participates in the
same branch-kill machinery |boom| already uses for the LDQ/STQ:

- On every ``brupdate``, each entry's ``br_mask`` is updated by clearing the resolved-branch bits
  (``br_mask & ~brupdate.b1.resolve_mask``).
- On a mispredict, any entry whose ``br_mask`` intersects ``brupdate.b1.mispredict_mask`` is younger
  than the mispredicted branch; it is **invalidated and its TVRB_RF index returned to the free list**
  the same cycle (the ``IsKilledByBranch`` predicate).

This is identical in shape to how a squashed scalar store releases its SQ entry, and it guarantees no
TVRB entry outlives the speculative path that created it, whether or not its consumer ever ran.


Segmented Load
~~~~~~~~~~~~~~

1. The LSU will begin executing the segmented load when vsrc operand are avalible in the vPRF.
2. The LSU reads from memory.
3. When executing a segmented load the LSU uses the vsrc PRN as the destination temporary register and will store the value in the TVRB.
4. The TVRB will notify the CII IQ that the temp register is available, and the Coprocessor can begin execution.
5. The Coprocessor will regard the vsrc as a temp register and only access the vector bypass network.
6. The Coprocessor will perform the necessary transform of the data and store the final result to the vdest PRN in the VPRF.  


Segmented Store
~~~~~~~~~~~~~~~

1. The Coprocessor will execute the segmented store when all source operands become available.
2. The Coprocessor will perform the necessary transforms on the data and write the result to the TVRB using the vsrc PRN.
3. The TVRB will notify the vec store IQ that the temp register is available, and the LSU can begin execution.
4. When executing a segmented store the LSU uses the vsrc PRN as the source temporary register and will read the value in the TVRB.
5. The LSU will regard the vsrc as a temp register and only access the vector bypass network.
6. The LSU will store the temp vsrc data to memory once its ROB entry is committed.



Speculative Wakeups
~~~~~~~~~~~~~~~~~~~

The vector bypass network also supports speculative load wakeup, however speculative wakeup's are
only sent to the vector operands of the ``IQ_V_*`` queues (via the vector wakeup network).
Speculative bypass wakeup's to the scalar queues would not improve performance as it would require
more than 1 cycle for the OP.v to reach a vector execution unit.



The Vector Register File
~~~~~~~~~~~~~~~~~~~~~~~~~

The vector physical register file will be parameterizable with default support for
128 PRNs, 8 Read Ports, and 4 Write Ports. The vector register file (VRF) will be
implemented using standard cell flip flops, with a banked architecture. A vector register
will split across 4 banks of 64 (VLEN/4) bits. This each port will have its own
decoder and support forwarding for single cycle reads, if a write port writes
to the same PRN.  

.. note::
  The VRF is only initialized by the usingRVV switch.

.. list-table::
   :header-rows: 1
   :widths: 1 1 1

   * - Functional Unit
     - PRF Read Ports
     - PRF Write Ports
   * - CoProcessor
     - 4
     - 2
   * - Store Unit
     - 2
     - 0
   * - Load Unit
     - 2
     - 2

Write-port breakdown: **4 total** = CoProcessor 2 (arithmetic results) + Load Unit 2 (the Load
Coalescing Buffer's one-write-per-destination-PRN, dual-lane on Mega). The **Store Unit has 0 write
ports** — a store *reads* vector data from the VRF and writes it to memory; it never writes the VRF.
Read-port breakdown: **8 total** = CoProcessor 4 (``pvs1``/``pvs2``/old-``vd``/``pvm``) + Store Unit 2
(store data + mask/index) + Load Unit 2 (index + mask). Segmented-LS temporaries use the separate
TVRB, not these VRF ports.
