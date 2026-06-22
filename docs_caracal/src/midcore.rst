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

Vector instructions will also enter the rename_stage and fp_rename_stage and have any scalar destination or source registers assigned a int or fp PRN. For vsetivl the VL register will require a integer PRN to be allocated for the VL value; younger vector uOPs carry it as an ordinary integer source operand (``pvl``) and read it from the integer RF/bypass at execute.



.. figure:: ../figures/vector_mapper.png
   :align: center

   Overview of Vector Mapper Stage.





.. _rename-twostage:

Dispatch-group atomicity across the two stages
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Because vector mapping takes an extra cycle, a vector ``OP.v`` reaches dispatch one cycle after a
scalar uop in the same dispatch group. |caracal| does **not** insert a 1-cycle bubble on the scalar
path to re-align them. Instead it decouples the three things that the rename→dispatch boundary
normally does in one cycle — ROB allocation, IQ-slot write, and branch snapshot — and lets the
vector-specific two of them slip a cycle, while the in-order one stays put:

- **Scalar Mapping (cycle 1)** allocates the ROB entry in program order — for both scalar and vector
  uops — and assigns the ``br_tag`` and any scalar PRNs. ROB allocation is the only thing that
  *must* be in program order, and it stays exactly as in |boom|: the dispatch group reserves its ROB
  entries atomically and in order. Scalar uops in the group also write their issue-queue slots this
  cycle and may begin issuing immediately.
- **Vector Mapping (cycle 2)** fills the vector-specific metadata (``pvdest``, the ``EMUL`` stale
  group, ``pvs*``/``pvm``, ``v_emul``, etc.) into the **already-allocated** ROB entry **and** writes
  the vector ``OP.v``'s issue-queue slot. A vector ``OP.v`` therefore enters its ``IQ_V_*`` queue one
  cycle behind its scalar group-mates. This is harmless: its ROB entry already exists (program order
  preserved), and a vector slot cannot be granted before its operands are renamed anyway, so the
  one-cycle-later slot write is never on the critical issue path. Scalar dispatch throughput is
  unaffected — only the vector lane is delayed, and only by one cycle.

**Branch-snapshot alignment (the subtle part).** The scalar/integer/FP RMTs are snapshotted on the
``ren_br_tags`` event in cycle 1. The **vector** RMT and the :ref:`VCFG mirror <vector-rvv-decode>`
update a cycle later, so snapshotting them off the *same* cycle-1 event would capture state from the
wrong cycle. |caracal| instead snapshots the vector RMT and the VCFG mirror off a **1-cycle-delayed
``br_tag``** — the ``ren_br_tags`` valid/tag pipelined into the vector-mapping stage. Concretely:

- When branch *B* in a dispatch group is assigned ``br_tag = t`` in cycle 1, any vector ``OP.v`` older
  than *B* in that group performs its vector remap in cycle 2.
- The vector RMT snapshot for tag *t* is written in cycle 2 from the delayed ``br_tag``, **after**
  those older vector remaps have landed in the vector ``map_table``. The snapshot therefore captures
  the correct "state as of *B*" — younger-than-*B* vector remaps have not yet been applied (they are
  in the next group, a later cycle).
- On ``brupdate.b2.mispredict`` the vector RMT and VCFG mirror restore from
  ``br_snapshots(br_tag)`` / ``vcfg_snapshots(br_tag)`` exactly as the scalar RMT does — the
  one-cycle-delayed *write* does not change the *restore* path; both reads are indexed by the same
  ``br_tag`` the mispredict carries.

The only ordering constraint is that the vector metadata write and the delayed snapshot both land
before the entry can issue, which they always do since vector mapping is exactly one cycle behind ROB
allocation. No scalar bubble is inserted and dispatch-group integrity is preserved.

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

Shared vector instructions are allocated PRNs and a single ROB entry as normal instructions.
**In addition**, when the vector mapper sees ``is_shared`` it allocates a **second vector group —
the intermediate temp group — from the main vector free list** and writes its PRNs into the
``OP.v``'s ``pvtmp`` field (up to ``EMUL`` members). The temp group is an ordinary VRF group: it
has a real busy lifetime, is reclaimed on branch-mispredict through ``br_alloc_lists``, and is
**freed at commit** alongside the stale vdest group. It is not installed in the vector RMT, so no
architectural read can alias it. The two halves rendezvous entirely through this temp group in the
VRF — one half writes it as a destination, the other reads it as a source — using the same
busy-table / group-done machinery as any vector operand (see :ref:`group-done wakeup <group-done>`).

.. note::

   A segmented load/store consumes **two** vector register groups (``pvdest`` + ``pvtmp``, up to
   ``EMUL`` PRNs each — up to 16 PRNs total), so it is a heavy consumer of vector-PRN resources.
   Several in-flight segmented ops can pressure the vector free list and back-pressure dispatch; the
   free list must keep enough headroom that the oldest segmented op can always allocate both groups
   to guarantee forward progress.


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

.. _group-done:

Readiness is tracked **per member PRN**, not by a group base. A consumer may read a source group
that is a sub-range of a larger destination group (an ``LMUL=8`` write to ``v0..v7`` followed by an
``LMUL=2`` read at ``v4`` sources ``{p4, p5}``, not the producer base ``p0``), and a group's members
may even come from different producers, so a base-only busy bit would wake the consumer early on a
stale read. The Vector Busy Table is therefore a per-PRN bit vector over ``numVecPhysRegisters``,
and each side scales like the :ref:`free-list` on set and source-read:

**1. Set-busy (allocation).**
   Set **all** member bits of the destination group — up to 8 per ``OP.v`` — as the ``OR`` of
   ``UIntToOH(pvdest_j)`` over the group's (non-contiguous) PRNs, up to ``coreWidth*8`` bits/cycle.

**2. Source reads.**
   Read the busy bit of **each member** of each source group (``pvs1``/``pvs2``/``pvs3`` plus the
   mask ``pvm``) — on the order of ~25 bit-reads per lane, times ``plWidth``, plus the integer
   ``pvl`` read. The per-member bits are **AND-ed into one group-ready bit** per operand; the
   operand wakes only when its **last** member is ready. This per-member match is the dominant
   cost of the vector Busy Table.

**3. Clear-busy (group-done).**
   Each group-done **carries the completing group's full member-PRN vector** (up to 8 PRNs) and
   clears all of those bits; the clear side is ``numVecWbPorts`` × up-to-8 bits wide.

**Single completion event, per-member readiness.** A producer still emits **one** group-done per
``OP.v`` (CII once per instruction; the LCB once after the last destination PRN lands —
:ref:`group-done-wb`), which is what keeps the **ROB busy-clear single-shot** with no per-entry
counter. That single event simply carries the member-PRN vector, so the Busy-Table clear and the
issue-slot wakeup remain per-member. A consumer reading a sub-range of an in-flight group wakes when
that producer's group-done fires (conservative but correct).

Accordingly ``busy_resps`` carries per-group source readiness (aggregated to group-ready bits);
``ren_uops`` carries the group member PRNs plus ``EMUL``; ``rebusy_reqs`` set up to 8 bits per
``OP.v``; and ``wakeups`` is ``numVecWbPorts`` wide, each carrying the completing group's member-PRN
vector.


.. _vl-vtype-rename:

VL Rename
~~~~~~~~~

``VL`` is renamed by the vector mapper into **its own register space** (64 entries), separate from
the integer/FP/vector PRFs. **Integer rename is not modified** — the VL value no longer lives in the
integer RF. It is a one-architectural-register rename with the same structures the scalar rename
already provides, just one ARN wide. (``VTYPE`` is **not** renamed — it rides the VCFG ``vtype``
mirror and the per-uOP ``VConfig`` snapshot; only ``VL`` gets a register file.)

- **Map table** — a current-PRN pointer (the renamed ``VL``), branch-snapshotted per ``br_tag`` and
  restored on mispredict; restored from a committed pointer on exception/flush. This *replaces* the
  old current-VL-PRN tracker. Because VL is renamed in the cycle-2 vector-map stage, its snapshot is
  written off the **1-cycle-delayed ``br_tag``** (the same delayed path as the vector RMT and VCFG
  mirror, see :ref:`rename-twostage`); a ``vset``→dependent pair in one dispatch group uses the
  in-bundle prefix bypass so the dependent picks up the just-renamed ``pvl``.
- **Free list** — 64-bit free vector; a producer allocates a fresh PRN.
- **Busy table** — one bit per PRN; set on allocation, cleared by the producer's VL writeback.
- **Wakeup network** — a dedicated ``VL`` network; vector issue slots match ``pvl`` on it (see the
  Vector Issue Slot).
- **Commit logic** — at commit of a VL producer the **outgoing committed pointer is freed** (no
  per-uop stale field is needed: the single-entry committed map table already holds the PRN this
  producer displaces), the committed pointer is advanced to the new PRN, and the architectural
  ``vl`` CSR is written (precise). Wrong-path producers are reclaimed by the free list's branch
  machinery, as usual.
- **EU read ports** — every vector EU reads ``VL`` from ``VL_RF[pvl]`` (and ``vtype`` from its
  ``VConfig`` snapshot).

**Producers.** Every producer allocates a VL PRN at rename and writes the VL RF, broadcasting ``pvl``
on the VL wakeup network. Who computes the value differs (see :ref:`vector-rvv-decode`):

- ``vsetivli`` — front-end only: the **VCFG** writes the VL RF at decode (VL is immediate).
- ``vsetvli`` / ``vsetvl`` — executed on an **integer ALU EU**: woken by ``rs1`` (and ``rs2`` for
  ``vsetvl``) on the integer network, the ALU computes VL and its writeback targets the VL RF.
- ``vleff`` — the LSU writes the (possibly trimmed) VL on completion.

Reading the integer ``AVL`` source is an ordinary integer RF **read** — it does not touch integer
rename. When ``rd != x0`` the vset also writes ``rd`` as a normal integer destination (unchanged),
but vector consumers read VL only from ``VL_RF``.

**Consumers.** Every younger vector ``OP.v`` carries the current ``pvl`` (read from the VL map table
at rename) as an implicit operand, and the ``vtype`` snapshot for its config. The mapper derives
``EMUL`` at decode from the :ref:`VCFG mirror <vector-rvv-decode>` (``vtype`` known there for the
immediate vset forms); ``vsetvl`` (register ``vtype``) still serializes via ``is_unique`` because the
mapper needs ``vtype`` at decode.


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

.. _group-done-wb:

Completion tracking
^^^^^^^^^^^^^^^^^^^^

A single ROB entry that owns an ``EMUL``-wide destination group needs to know when **all** of
the group's writes are done before it can clear its busy bit and commit. |caracal| keeps the ROB
side **identical to scalar** — a single-shot ``rob_bsy`` clear per entry — by requiring every
vector producer to **aggregate its group into one group-done event** rather than streaming
per-PRN writebacks into the ROB. The per-PRN counting lives in the producer, not in a new ROB
counter:

Vector arithmetic (tt_CII)
   The vector ALU path executes **in program order on the in-order tt_CII coprocessor** (see
   :ref:`vector-execution`). An in-order unit knows when an entire ``OP.v`` has retired, so it
   signals completion **once per instruction**. This maps directly onto the existing
   single-writeback busy-clear in ``rob.scala`` — one wakeup clears ``rob_bsy``.

Out-of-order vector load (VLS)
   The vector memory path generates element/segment sub-accesses that complete **out of order**
   through the LSU, but they do **not** report to the ROB individually. The **Load Coalescing
   Buffer** (:ref:`load-coalesce`) owns the per-PRN assembly *and* the per-group PRN-done count
   (target derived from ``v_emul``/``v_seg_nf`` — the true destination-group size, accounting for
   widening/narrowing and single-register mask/reduction results). When the **last** destination
   PRN of the group is assembled, the LCB emits **one** group-done carrying the group's member-PRN vector.
   The ROB clears ``rob_bsy`` on that single event — no per-entry completion counter is added.
   The same group-done drives the Busy-Table clear and the vector wakeup (see :ref:`group-done wakeup <group-done>`),
   so the three structures stay consistent by construction. Vector **stores** clear via a single
   ``lsu_clr_bsy`` once the whole active element set has translated/disambiguated (they write no
   VRF); scalar ops are unchanged.

Shared instruction (segmented LS)
   A shared instruction occupies **two issue slots** — one in the CII IQ (coprocessor half) and
   one in the vector load/store IQ (LSU half) — but **a single ROB entry**. This is the **only**
   case that waits for more than one completion, and it needs just a **1-bit "other half pending"
   flag**, not a counter: the entry clears ``rob_bsy`` only when **both** the LSU group-done and
   the coprocessor group-done have arrived. The two halves are sequenced through the ``pvtmp`` group
   in the VRF — the producer (LSU for a segmented load, coprocessor for a segmented store) writes
   ``pvtmp``, whose group-done wakes the consumer's IQ slot (see the CII Shared Instruction
   Scheduling section). For a segmented **store** the LSU half's actual D$ writes are still
   post-commit, exactly as for any store.

The ``rob_unsafe`` (speculation-hazard) bit is gated the same way: a multi-access vector load is
not memory-safe until **all** of its element addresses have disambiguated, so the LSU reports a
single **group-safe** event (when the last element address has been LCAM-checked) that clears
``rob_unsafe`` — not a per-sub-access clear.


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

Segmented load/stores commit data up to the point of exception. Because the ``pvtmp`` group is an
ordinary VRF group freed at commit, a faulting (non-committing) shared instruction needs **no special
temp cleanup** — its ``pvtmp`` group is reclaimed by the standard free-list flush/rollback path like
any uncommitted allocation. For a segmented **load** the loaded elements before the fault are written
into ``pvtmp`` and the coprocessor transposes them into ``pvdest`` (tail elements may hold junk). For
a segmented **store** the coprocessor writes ``pvtmp`` and the LSU writes memory only up to the
faulting element.


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
   * - VL
     - 1
     - 64

``VL`` is **renamed into its own register file** (default 64 entries), not stored in the integer RF.
It is a one-architectural-register rename space with its own map table (a current-PRN pointer +
``maxBrCount`` snapshots), free list, busy table, wakeup network, commit logic, and read ports to
every vector EU — see :ref:`vl-vtype-rename`. The VL value is therefore **not** held in the integer
RF; ``vset``'s ``rd`` GPR write (when ``rd != x0``) is a separate, ordinary integer destination and
is the only integer-RF interaction, leaving integer rename unchanged. ``VTYPE`` is **not** renamed —
it is held in the VCFG ``vtype`` mirror and carried to the EU in the per-uOP ``VConfig`` snapshot
(no VTYPE register file).

.. note::

   |caracal|/|boom| has no dedicated mask **register file**; Vector mask is treated
   like any other vector register, with masking semantics handled in the execution units.



Vector Bypass Network
~~~~~~~~~~~~~~~~~~~~~

|boom| supports a full operand forwarding bypass network for the integer and FP pipeline. |caracal|
implements the equivalent bypass network for the vector pipeline. There is **no separate temporary
register file**: intermediate results for shared instructions live in the main VRF as the
``pvtmp`` group (see :ref:`group-done wakeup <group-done>` and CII Shared Instruction Mapping), and every vector EU
addresses ``pvtmp`` exactly as it addresses any other vector PRN — reads, writes, busy/wakeup, and
branch/commit reclaim all go through the existing vector register machinery.


Segmented Load
~~~~~~~~~~~~~~

The ``pvtmp`` group is the **destination** of the LSU half and the **source** of the coprocessor half.

1. The LSU issues when its address operands are ready, reads memory, and writes the loaded data into
   the ``pvtmp`` group in the VRF (treating ``pvtmp`` as an ordinary vector destination).
2. ``pvtmp``'s group-done on the vector wakeup network wakes the coprocessor's IQ slot.
3. The coprocessor reads ``pvtmp`` from the VRF, performs the transpose, and writes the final result
   to the ``pvdest`` group.


Segmented Store
~~~~~~~~~~~~~~~

Roles reverse: the ``pvtmp`` group is the **destination** of the coprocessor half and the **source**
of the LSU half.

1. The coprocessor issues when its source operands are ready, transposes the data, and writes it into
   the ``pvtmp`` group in the VRF.
2. ``pvtmp``'s group-done wakes the store IQ slot.
3. The LSU reads ``pvtmp`` from the VRF as its store data and writes it to memory once the ROB entry
   is committed.



Speculative Wakeups
~~~~~~~~~~~~~~~~~~~

With the single unified issue stage there is no separate scalar/vector scheduling stage to steer
speculative wakeups to, so the policy is split by **operand class** instead:

- **Scalar feeders of a vector slot** (base/stride/VL on the integer network, the ``.vf`` scalar on
  the FP network) participate in |boom|'s existing **speculative load-hit wakeup** unchanged. A vector
  uOP waiting on a scalar-load result is woken speculatively just like any integer/FP consumer, and is
  re-busied through the same machinery if the load later misses.
- **Vector operands** (``pvs*``, ``pvm`` on the vector wakeup network) wake only on **actual
  completion**, not speculatively. A vector-load producer completes via the Load Coalescing Buffer
  after a long, variable, streaming latency, and a vector-arithmetic producer completes in program
  order over the CII — neither has the fixed short load-use latency that makes speculation profitable.
  Waking vector operands on real writeback also avoids adding re-busy / replay machinery to the
  vector network and the vector issue slots.



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
(store data + mask/index) + Load Unit 2 (index + mask). Segmented-LS temporaries (``pvtmp``) are
ordinary VRF groups and use these same ports — the LSU/coprocessor read or write ``pvtmp`` on the
ports already counted (it is a destination for one half and a source for the other), adding no new
ports.

**Tail/mask-undisturbed reads its budget from these same ports — no extra ports are added:**

- **Arithmetic** tail/mask-undisturbed merges are done **inside the CII coprocessor**, which reads
  the old ``vd`` on the read port already counted above (``old-vd`` of the CoProcessor's 4 reads) and
  merges the inactive lanes before writing back. No additional port.
- **Load** tail/mask fill is done **inside the LCB** before its single ``W0`` write
  (:ref:`load-coalesce`); the inactive-lane source comes from the assembly entry, not a new VRF read.
- **The standalone ``VL = 0`` / fully-inactive ``vta = 0`` group copy** ``pvdest ← stale_pvdest``
  (see the VL == 0 case study) **reuses the Load Unit's 2R/2W ports**, which are idle when no load is
  draining. It therefore fits in the 8R/4W budget with no new ports; under contention it arbitrates
  behind active load drains (the copy is rare and not latency-critical).

So the 8R/4W budget above is the **complete** Goal-1 VRF port requirement; tail-undisturbed handling
does not widen it.
