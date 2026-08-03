Midcore
=======

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC

.. _midcore-rename:

The Rename Stage
----------------

The Rename Stage is extended to support vector register renaming in a **single pipeline stage**.
For a vector instruction the scalar (INT/FP) rename and the vector group rename run **in parallel
in the same cycle** — they target independent register spaces, free lists, and busy tables, so
neither depends on the other for a given ``OP.v``. There is **no** separate vector-mapping pipeline
stage and **no** 1-cycle-delayed pipeline register: the whole dispatch group (scalar and vector)
emerges from rename together, in one cycle.

On the scalar side the original |boom| implementation is unchanged — the integer ``rename_stage`` and
``fp_rename_stage`` still rename scalar destinations/sources to INT/FP PRNs for both scalar and
vector uops (e.g. a vector load's base/stride, a ``.vx``/``.vf`` operand). In parallel, the vector
mapper renames the vector group (``lvd``/``lvs*``/``lvm`` → ``pvdest``/``pvs*``/``pvm``) and the VL
mapper renames VL into the VL register file. The combined rename cycle is therefore the *max* of the
scalar and vector-group rename latencies, not their sum — the cost of folding the vector group
mapper (EMUL-wide group read, up to-8-PRN allocation, per-member bypass) onto the rename critical
path, in exchange for dropping the second stage and all of its alignment machinery.

For ``vset`` the VL value is renamed into the **VL register file** (its own space); younger vector
uOPs carry ``pvl`` (a VL-RF index) and read VL from the VL RF at execute (see :ref:`vl-vtype-rename`).



.. figure:: ../figures/vector_mapper.png
   :align: center

   Overview of Vector Mapper Stage.





.. _rename-stage:

Single-stage rename: reservation and dispatch-group atomicity
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Because scalar and vector rename run **in parallel in one cycle**, the whole dispatch group reaches
dispatch together — there is no cross-group skew to reconcile, no 1-cycle bubble, and no delayed
pipeline register. Everything the rename→dispatch boundary does happens atomically, in program
order, in that single cycle:

- **ROB allocation** — one entry per uop (one per ``OP.v``), reserved in program order for the whole
  group, exactly as in |boom|.
- **LDQ/STQ slot reservation** — every memory uop, including each vector load/store ``OP.v`` (a
  **single** LDQ or STQ entry), claims its slot at the in-order ``ldq_tail`` / ``stq_tail`` this
  cycle, with ``ldq_idx``/``stq_idx`` written into the ``OP.v`` as the program-age stamp for
  cross-queue disambiguation. The per-element addresses/data are produced later at the Vector LS
  AGEN/DGEN (they live in the SSI/US queues, not the LDQ/STQ entry); only the *slot* is reserved here.
- **``br_tag`` allocation and branch snapshots** — taken on the ``ren_br_tags`` event this cycle.
- **Register rename (parallel)** — scalar INT/FP PRNs from the unchanged ``rename_stage`` /
  ``fp_rename_stage``, the vector group (``pvdest``/``pvs*``/``pvm`` + the ``EMUL`` ``stale_pvdest``
  group + ``v_emul``) from the vector mapper, and ``pvl`` from the VL mapper — all in the same cycle.
- **Issue-queue slot write** — every uop (scalar and vector) writes its IQ slot this cycle; no uop
  lags its group-mates.

**Branch snapshots are simple again.** Since the vector RMT, the VL map table, and the
:ref:`VCFG mirror <vector-rvv-decode>` all update in the *same* cycle as the scalar RMT, they are
snapshotted on the **same ``ren_br_tags`` event** as the scalar RMT — there is **no** delayed-``br_tag``
path. On ``brupdate.b2.mispredict`` the vector RMT / VL map table / VCFG mirror restore from
``br_snapshots(br_tag)`` / ``vcfg_snapshots(br_tag)`` in lockstep with the scalar RMT, all indexed by
the same ``br_tag``. (The VCFG mirror is decode-stage state, so its per-``br_tag`` snapshot is sourced
from the branch's carried ``vconfig`` — the nearest-preceding-``vset`` value from the per-lane prefix
select — so it reflects the state as-of the branch.)

This combining is the source of the simplification: ROB allocation, LDQ/STQ reservation, and branch
snapshotting are all done once, in order, in the single rename cycle — no split, no re-alignment, no
delayed snapshot. The trade-off is purely timing: the vector group mapper now sits on the rename
critical path (in parallel with scalar rename, so cost = the slower of the two, not the sum).

.. warning::

   **This is the design's principal timing risk and it is not yet bounded.** One cycle now carries,
   in parallel with scalar rename: the ``EMUL``-wide vector map-table group read, up-to-8-PRN
   non-contiguous free-list allocation (``allocWidth = coreWidth*8``), and on the order of **25
   per-member busy-bit reads per lane × ``plWidth``** AND-reduced into group-ready bits (see
   :ref:`group-done wakeup <group-done>`) — plus the VL rename and the ``pvtmp`` allocation for shared ops. "Cost = the
   slower of the two" is the right *shape* of the argument but it is not a bound, and if the vector
   side loses, the whole core's rename stage pays.

   Recommended mitigation: run a **timing spike on the vector mapper + busy table before Milestone 1
   Step 4 lands**, so the fallback (splitting rename back into two stages, with all the alignment
   machinery this design deleted) is a known-cost decision rather than a late surprise. Deleting the
   LMUL tag checker (see the note under Rename Map Table) removes one comparator tree from this path.

.. _rmt:

Rename Map Table (RMT)
~~~~~~~~~~~~~~~~~~~~~~~

|boom| maintains two sets of register rename map tables **Speculative RMT** and **Committed RMT**. The Committed RMT is an optional feature, which we will always keep enabled in |caracal|.

1. Speculative RMT — ``map_table``

   This is the working table updated at rename time as physical destinations are allocated. It reflects in-flight, not-yet-committed instructions. Source operands are read from it (``map_resps``).

2. Architectural / Committed RMT — ``com_map_table``

   Updated only by committed uop's (driven from the ROB via ``com_remap_reqs``). It holds the known-good architectural state.

When an exception occurs, the ``com_map_table`` is copied to the speculative ``map_table``. Because the ``com_map_table`` only contains the latest committed ARN to PRN mappings it holds the most recent correct state before the trapping instruction, thus instead of a ROB 1 entry/cycle walk back a single cycle copy is all that is needed to roll back the ``map_table``.

The vector mapper extends the existing RenameStage implementation to support vector renaming. The vector mapper implements atomic LMUL/EMUL based vector register mapping. The mapper may rename and allocate up to LMUL=8 PRNs for a single vdest. This removes the need for vector uOP cracking after the decoder and reduces the number of ROB entries required for vector instructions.

.. note::

   The Vector Mapper should also get the old stale vdest group and update the OP.v ``stale_pvdest`` field — a ``Vec`` of up to ``EMUL`` stale PRNs, not a single reg, since a vector dest renames a whole group. This will assist in handling tail undisturbed instructions and lets commit free the entire stale group. This stale-group capture happens in the single rename cycle, in parallel with scalar rename (see :ref:`rename-stage`).

The vector map table stores **one PRN per architectural vreg**, so an ``EMUL``-wide read returns the
group's current mappings directly and is correct under **arbitrary fragmentation**. That single
property is what makes the mapper simple: no contiguous-run allocator, no whole-group validity
check, and no fragmentation-recovery walk.

.. note::

   **Removed: the "LMUL TAG Whole Vector Group Checker."** Earlier drafts added a 32×2-bit
   per-ARN tag table plus combinational whole-group comparators for every valid base ARN at each
   LMUL. It has been deleted, for two reasons:

   1. **It gated nothing.** The draft itself concluded that a FALSE result needs no exception and no
      recovery — "the EMUL-wide read is still correct, so the read simply proceeds" — leaving the
      structure driving only a performance counter. A per-ARN map table is *always* whole-group
      correct, so there is nothing to validate.
   2. **Its test was wrong anyway.** Requiring all eight tags to equal ``2'b11`` classifies a group
      written by eight independent ``LMUL=1`` instructions as "fragmented," even though that group
      reads back perfectly.

   Deleting it also removes 32×2 bits of state and a wide comparator tree from the **single-cycle
   rename critical path**, which is the design's principal timing risk (see :ref:`rename-stage`).
   If the observability is still wanted, a per-ARN "last-write ``EMUL``" performance counter gives
   the same signal far more cheaply.



.. _cii-shared-mapping:

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
   Several in-flight segmented ops can pressure the vector free list and back-pressure dispatch.

   **No free-list headroom reservation is required for forward progress.** Rename is in program
   order, so an ``OP.v`` that cannot allocate simply stalls at rename while every older instruction
   continues to commit and free its stale group — the oldest op is never the one that starves. The
   actual requirement is narrower and must be stated explicitly: allocation of ``pvdest`` + ``pvtmp``
   is **all-or-nothing**. A segmented op must never take a partial group and wait for the remainder,
   because those held PRNs cannot be freed until it commits, which produces exactly the deadlock the
   headroom argument was reaching for.

   **A failed allocation stalls the whole dispatch group**, not just the failing ``OP.v``: no uop in
   the bundle renames that cycle, and the bundle retries intact on the next. This is the coarse rule
   deliberately — it is trivially order-preserving and needs no per-lane stall mask. The cost is
   dispatch bandwidth, since older group-mates that could have renamed re-do the attempt each retry
   cycle; the benefit is that the atomic rename→dispatch actions of :ref:`rename-stage` never have
   to be applied to a partially renamed bundle.


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


.. _busy-table:

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
   mask ``pvm``) — on the order of ~25 bit-reads per lane, times ``plWidth``, plus the ``pvl``
   read. **The ``pvl`` bit is read from the VL busy table** (:ref:`vl-vtype-rename`), not the
   integer one: ``VL`` is renamed into its own register space, so ``pvl`` indexes ``VL_RF`` and
   has exactly one busy bit, in that table.
   The per-member bits are **AND-ed into one group-ready bit** per operand; the
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

``VL`` is renamed by the vl mapper into **its own register space** (64 entries), separate from
the integer/FP/vector PRFs. **Integer rename is not modified** — the VL value no longer lives in the
integer RF. It is a one-architectural-register rename with the same structures the scalar rename
already provides, just one ARN wide. (``VTYPE`` is **not** renamed — it rides the VCFG ``vtype``
mirror and the per-uOP ``VConfig`` snapshot; only ``VL`` gets a register file.)

- **Map table** — a current-PRN pointer (the renamed ``VL``), branch-snapshotted per ``br_tag`` and
  restored on mispredict; restored from a committed pointer on exception/flush. This *replaces* the
  old current-VL-PRN tracker. VL is renamed in the **single rename cycle** (in parallel with scalar
  and vector-group rename, see :ref:`rename-stage`), so its snapshot is taken on the same
  ``ren_br_tags`` event as the other RMTs — no delayed-``br_tag`` path; a ``vset``→dependent pair in
  one dispatch group uses the in-bundle prefix bypass so the dependent picks up the just-renamed
  ``pvl``.
- **Free list** — 64-bit free vector; a producer allocates a fresh PRN. **A producer that finds no
  free VL PRN stalls at rename**, exactly as an ``OP.v`` short of vector PRNs does
  (:ref:`cii-shared-mapping`), and is deadlock-free for the same reason: rename is in program order,
  so older producers keep committing and freeing the pointer each displaces, and the oldest producer
  is never the one that starves. The 64 entries can genuinely be exhausted — the live count is the
  one committed pointer plus one per in-flight producer — so any ROB deeper than 63 can reach this
  stall.
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

- ``vsetivli`` — front-end only (no EU): the **VCFG** computes VL at decode and the value is written
  to ``VL_RF[pvl]`` in the **rename** cycle, where ``pvl`` is allocated. No busy bit is set — ``pvl``
  is born ready. (Earlier drafts said "writes the VL RF at decode"; there is no VL-RF index to write
  at decode.)
- ``vsetvli`` / ``vsetvl`` — executed on an **integer ALU EU**: woken by ``rs1`` (and ``rs2`` for
  ``vsetvl``) on the integer network, the ALU computes VL and its writeback targets the VL RF.
- ``vleff`` — the LSU writes the (possibly trimmed) VL on completion.

Every producer sets the ``is_vl_producer`` bit
(:ref:`dual-destination rule <vset-dual-dest>`), which is **orthogonal to
``dst_rtype``**: a ``vset`` with ``rd != x0`` writes *both* the integer RF (``pdst``) and the VL RF
(``pvl``) from one result bus, and with ``rd == x0`` it writes only the VL RF while ``dst_rtype`` reads
``RT_ZERO``. VL-producing is therefore never inferable from ``dst_rtype``.

Reading the integer ``AVL`` source is an ordinary integer RF **read** — it does not touch integer
rename. When ``rd != x0`` the vset also writes ``rd`` as a normal integer destination (unchanged),
but vector consumers read VL only from ``VL_RF``.

**Consumers.** Every younger vector ``OP.v`` carries the current ``pvl`` (read from the VL map table
at rename) as an implicit operand, and the ``vtype`` snapshot for its config. The mapper derives
``EMUL`` at decode from the :ref:`VCFG mirror <vector-rvv-decode>` (``vtype`` known there for the
immediate vset forms); ``vsetvl`` (register ``vtype``) still serializes via ``is_unique`` because the
mapper needs ``vtype`` at decode.


.. _rob-vec:

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
   so the three structures stay consistent by construction. A **non-shared** vector store clears via
   a single ``lsu_clr_bsy`` once its whole active element set has translated/disambiguated (it writes
   no VRF); scalar ops are unchanged. A **shared** store's LSU half is the exception — see below.

Shared instruction (segmented LS)
   A shared instruction occupies **two issue slots** — one in the CII IQ (coprocessor half) and
   one in the vector load/store IQ (LSU half) — but **a single ROB entry**. This is the **only**
   case that waits for more than one completion, and it needs just a **1-bit "other half pending"
   flag**, not a counter: the entry clears ``rob_bsy`` only when **both** halves' completion events
   have arrived. Which event each half emits depends on whether it writes the VRF: a segmented
   **load**'s LSU half writes ``pvtmp`` and so emits a real **group-done**, while a segmented
   **store**'s LSU half writes no VRF and so signals **``lsu_clr_bsy``**. The coprocessor half always
   emits a group-done.

   **A shared store's LSU half defers its completion until after DGEN has read ``pvtmp``**, not to
   the translate/disambiguate point that governs a non-shared store. Signalling at translate would
   claim the half is done while it still has to obtain its store data from the coprocessor, and it
   would invert the fixed order below.

   **The flag is set at dispatch**, when the Decoder's ``is_shared`` is seen, and cleared when the
   **second** completion arrives. Both completions can never land in the same cycle: the consumer
   half is woken *by* the producer's ``pvtmp`` group-done, so it issues and executes strictly
   afterwards. No same-cycle case needs handling.

   .. note::

      The two halves always complete in a **fixed order** — producer half, then consumer half —
      because the consumer is woken by the producer's ``pvtmp`` group-done: LSU-then-CII for a
      segmented load, and (per the dependency chain in :ref:`shared-store-chain`) CII-then-LSU for a
      segmented store. The flag is therefore equivalent to "wait for the **consumer** half";
      the producer's completion is implied. It is kept as an explicit flag for clarity and as a
      cross-check, not because either order is possible.

   The two halves are sequenced through the ``pvtmp`` group
   in the VRF — the producer (LSU for a segmented load, coprocessor for a segmented store) writes
   ``pvtmp``, whose group-done wakes the consumer's IQ slot (see the CII Shared Instruction
   Scheduling section). For a segmented **store** the LSU half's actual D$ writes are still
   post-commit, exactly as for any store.

The ``rob_unsafe`` (speculation-hazard) bit is gated the same way: a multi-access vector load is
not memory-safe until **all** of its element addresses have disambiguated, so the LSU reports a
single **group-safe** event (when the last element address has been LCAM-checked) that clears
``rob_unsafe`` — not a per-sub-access clear.

A vector **store** clears ``rob_unsafe`` on the same shape of event: one group-safe when the last of
its element addresses has been LCAM-checked. A **shared** instruction needs **only** the LSU half's
group-safe — the coprocessor half performs no memory access, so it carries no speculation hazard, and
requiring a second group-safe from it would deadlock: that half cannot issue until the PNR has passed
the entry, which needs ``rob_unsafe`` already cleared (:ref:`shared-store-chain`).


.. _vec-commit:

Commit
~~~~~~

At retirement a vector entry frees its entire **stale destination group** — ``EMUL``
stale vector pregs, not one. Because allocation is non-contiguous, the ROB carries the ``EMUL``
stale PRNs captured at rename (an explicit stale group, not a base+count) alongside the new
``pvdest``. This couples to the free-list deallocation path described in :ref:`free-list`.

**Every ROB entry provides ``MAX_MEMBERS`` = 8 stale-PRN slots.** A vector entry uses ``EMUL`` of
them and leaves the remainder don't-care; a scalar entry uses one, for its stale ``pdst``. The width
is therefore ``8 × log2(numVecPhysRegisters)`` bits per entry — about 56 bits at the default 96 PRNs
— paid on every entry including scalar ones. Sizing the slots to the worst-case ``EMUL`` is what lets
commit free the group with no second structure and no per-entry indirection.


.. _snapshots:

Checkpoint RMT Snapshots
~~~~~~~~~~~~~~~~~~~~~~~~

BOOM implements a branch snapshot mechanism to roll back the RMT upon branch mispredicts in a 
single cycle. A copy of the speculative scalar RMTs is taken per outstanding branch 
(upto maxBrCount entries).

We will reuse this mechanism for the vector mapper.

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
     - ``map_table``
     - the vector RMT advances through its remap requests




.. _precise-vec-exc:

Precise exceptions for vector ops
---------------------------------

A vector load/store can fault partway through its element stream (e.g. element *k* raises a page
fault). Because the whole instruction is one ROB entry, the trap must be made restartable. |caracal|
does this the **simplest correct way: it traps with ``vstart = 0`` and restarts the whole
instruction.**

.. warning::

   **Do not resume at ``vstart = k``.** Earlier drafts latched the oldest faulting element index and
   trapped with ``vstart = k`` so the instruction could resume mid-stream. That is **not
   implementable in this microarchitecture and silently corrupts data.** Elements ``0..k-1`` were
   written into ``pvdest``, a *freshly allocated* physical group. The faulting instruction never
   commits, so ``pvdest`` is never installed in ``com_map_table``; the exception path restores
   ``map_table := com_map_table`` and returns the whole group to the free list. Architectural ``vd``
   still maps to ``stale_pvdest``, so **those k elements are gone.** Resuming at ``vstart = k`` then
   re-executes only ``k..VL-1``, leaving ``0..k-1`` holding pre-instruction values — while RVV
   requires them to hold the loaded data. The result is silent wrong data on any page-crossing
   gather.

Because **nothing was architecturally written**, ``vstart = 0`` is a legal trap value and a full
restart is correct: loads are idempotent, and stores have not touched memory (they drain only
post-commit). The consequences for the rest of the design are simplifications:

- The faulting-element index is **not** carried to the ROB and **not** written to ``vstart``. The
  LSU exception port reports a plain precise exception, like a scalar load.
- ``fault_elem`` survives only as (a) the *stop* signal for the element cursor and (b) a
  debug/performance counter. See :ref:`elem-progress`.
- The cost is re-draining elements ``0..k-1`` after the handler returns. Forward progress is
  unaffected.

CII arithmetic ops complete atomically and raise exceptions at instruction granularity, so they need
no element index either.

.. note::

   The RVV §17.1 justification that appeared here has been **removed**. That relaxation concerns
   elements *after* the faulting element (letting the un-loaded tail be treated as agnostic). It says
   nothing about elements *before* it, which is the case that actually mattered.

``vleff`` is unaffected: it does not trap for a fault at element ``i > 0``, it trims VL — a genuinely
different mechanism (see :ref:`elem-progress`).


Segmented Load/Store (Shared Instruction)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Because the ``pvtmp`` group is an ordinary VRF group freed at commit, a faulting (non-committing)
shared instruction needs **no special temp cleanup** — its ``pvtmp`` group is reclaimed by the
standard free-list flush/rollback path like any uncommitted allocation.

Consistent with ``vstart = 0`` above, a faulting segmented load/store **does not commit partial
data**: both ``pvdest`` and ``pvtmp`` are uncommitted physical groups, and a segmented store's memory
writes are post-commit, so the fault leaves no architectural trace and the whole instruction restarts.
(Earlier drafts stated that segmented load/stores "commit data up to the point of exception"; that is
not achievable for the same reason ``vstart = k`` is not.)


.. _regfiles-bypass:

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
     - 96
   * - VL
     - 1
     - 64

.. note::

   **Vector-PRN capacity is a published architectural limit, not just an area knob.** 32 of the 96
   vector PRNs are always held by the committed map table — the RMT maps all 32 architectural vregs
   at all times, regardless of the current ``LMUL`` — and an ``LMUL=8`` ``OP.v`` takes 8 at once. So
   at most ``(96 − 32) / 8 = 8`` ``LMUL=8`` groups can be in flight, and a segmented op takes two
   groups (``pvdest`` + ``pvtmp``, 16 PRNs), so at most 4 of those. Size this against a target
   memory-level parallelism and state the derived in-flight limit wherever the parameter is
   documented.

   **Nothing enforces the bound explicitly** — it is a consequence of free-list capacity, not a
   credit. At the bound an ``OP.v`` simply finds too few free PRNs and **stalls at rename**, under the
   ordinary all-or-nothing allocation rule (see :ref:`cii-shared-mapping`). There is **no**
   group-credit counter, and none should be added: it would duplicate state the free list already
   holds.

The VRF as specified is ``96 × 256 b`` = 24 kbit of standard-cell flops carrying **12 ports**
(9R/3W), banked 4×64 b. That is a large structure to build from flops, and it should be given an
area/timing estimate before the implementation commits to a flop-based file rather than a
latch/SRAM-based banked one. Note that dropping 128 → 96 PRNs cut the array by 8 kbit while the
port count went 11 → 12, so the port-driven area term grew relative to the storage term.

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


.. _midcore-segmented-load:

Segmented Load
~~~~~~~~~~~~~~

The ``pvtmp`` group is the **destination** of the LSU half and the **source** of the coprocessor half.

1. The LSU issues when its address operands are ready, reads memory, and writes the loaded data into
   the ``pvtmp`` group in the VRF (treating ``pvtmp`` as an ordinary vector destination).
2. ``pvtmp``'s group-done on the vector wakeup network wakes the coprocessor's IQ slot.
3. The coprocessor reads ``pvtmp`` from the VRF, performs the transpose, and writes the final result
   to the ``pvdest`` group.


.. _midcore-segmented-store:

Segmented Store
~~~~~~~~~~~~~~~

Roles reverse: the ``pvtmp`` group is the **destination** of the coprocessor half and the **source**
of the LSU half.

1. The coprocessor issues when its source operands are ready, transposes the data, and writes it into
   the ``pvtmp`` group in the VRF.
2. ``pvtmp``'s group-done wakes the store IQ slot.
3. The LSU reads ``pvtmp`` from the VRF as its store data and writes it to memory once the ROB entry
   is committed.



.. _spec-wakeups:

Speculative Wakeups
~~~~~~~~~~~~~~~~~~~

With the single unified issue stage there is no separate scalar/vector scheduling stage to steer
speculative wakeups to, so the policy is split by **operand class** instead:

- **Scalar feeders of a vector slot** (base/stride on the integer network, the ``.vf`` scalar on
  the FP network) participate in |boom|'s existing **speculative load-hit wakeup** unchanged. A vector
  uOP waiting on a scalar-load result is woken speculatively just like any integer/FP consumer, and is
  re-busied through the same machinery if the load later misses.
- **``pvl``** wakes on **actual completion**, on its own **VL** wakeup network — *not* on the integer
  network and *not* speculatively. No VL producer has the fixed short load-use latency that makes
  speculation profitable: ``vsetivli``'s ``pvl`` is born ready, ``vsetvli``/``vsetvl`` have
  deterministic integer-ALU latency, and ``vleff`` publishes its trimmed VL only after a long,
  variable vector-load stream. So VL needs no re-busy or replay machinery either.
- **Vector operands** (``pvs*``, ``pvm`` on the vector wakeup network) wake only on **actual
  completion**, not speculatively. A vector-load producer completes via the Load Coalescing Buffer
  after a long, variable, streaming latency, and a vector-arithmetic producer completes in program
  order over the CII — neither has the fixed short load-use latency that makes speculation profitable.
  Waking vector operands on real writeback also avoids adding re-busy / replay machinery to the
  vector network and the vector issue slots.



.. _vector-regfile:

The Vector Register File
~~~~~~~~~~~~~~~~~~~~~~~~~

The vector physical register file will be parameterizable with default support for
96 PRNs, **9 Read Ports, and 3 Write Ports** (see :ref:`vrf-ports` for the canonical
port assignment). The vector register file (VRF) will be
implemented using standard cell flip flops, with a banked architecture. A vector register
will split across 4 banks of 64 (VLEN/4) bits. This each port will have its own
decoder and support forwarding for single cycle reads, if a write port writes
to the same PRN.

**Bank count is fixed at 4 and bank width is ``VLEN/4``**; the array is
``numVecPhysRegisters × VLEN``. The ``64 b``, ``96 × 256 b`` and 24 kbit figures quoted here and in
:ref:`regfiles-bypass` are the ``VLEN = 256`` instances of those formulas, not independent constants.

**Neither of the two conflicts a 12-port file invites can arise, for two different reasons.**

- **Two write ports never target the same PRN.** Every PRN is the destination of exactly one
  producer, and rename always allocates a *fresh* group, so ``W0``/``W1`` (the LCB's lanes, one write
  per destination PRN) and ``W2`` (the coprocessor) can never name the same register — including for a
  shared instruction, where one half writes ``pvtmp`` and the other ``pvdest``, two distinct groups.
  No write arbitration or priority is required.
- **Read ports never conflict over a bank.** Banking splits the register's **width**, not the port
  count: a VRF read is a full ``VLEN``, so *every* port spans all four banks, and each bank carries
  all 12 ports at ``VLEN/4`` bits. "Several read ports addressing one bank" is the normal case, not a
  hazard, which is why each port has its own per-bank decoder.

.. note::
  The VRF is only initialized by the usingRVV switch.

.. _vrf-ports:

Port assignment (canonical, 0-based)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Ports are **statically partitioned**, not arbitrated. This table is the single source of truth;
:doc:`cii` and :doc:`loadstore` refer to it rather than restating port numbers.

.. list-table::
   :header-rows: 1
   :widths: 22 44 34

   * - Functional Unit
     - Read Ports
     - Write Ports
   * - Load Unit / LCB
     - ``R0`` index, ``R1`` mask, ``R2`` ``stale_pvdest``
     - ``W0``–``W1``, tracking ``lsuWidth`` (``W0`` only at ``lsuWidth = 1``)
   * - Store Unit
     - ``R3`` store data, ``R4`` mask/index
     - — (a store *reads* the VRF and writes memory; it never writes the VRF)
   * - CoProcessor (CII)
     - ``R5``–``R8`` (the four Src-Request pull lanes)
     - ``W2``

**The write-port count follows ``lsuWidth``**, so the total is **9R / 2W** at ``lsuWidth = 1`` and
**9R / 3W** at ``lsuWidth = 2``. Every figure below is the ``lsuWidth = 2`` case. Binding the count
to the parameter rather than to configuration names covers any configuration, including those below
Medium, without this table having to track a config list.

``R1`` and ``R4`` each have **exactly one reader**. On the load path that reader is ``ld_vAGEN_1``,
which performs the mask read for **every** access class including unit-stride, and carries the mask
with the ``nOP.v`` to the stage 2 Packer rather than letting the Packer read the VRF itself — see
:ref:`vector-agen`. Without that, a US ``OP.v`` in stage 2 and an SSI ``OP.v`` in stage 1 would be
two concurrent readers of ``R1``, which a statically partitioned file cannot serve.

``R4`` serves both the store mask and the store index because **the two reads never need the same
cycle**: one ``VLEN``-wide read of ``v0`` yields the mask bits for every element of the access, so the
mask is read **once per ``OP.v``** and latched, while index members are read as the walk advances.
An indexed masked store therefore needs no second port.

Both coprocessor figures are **derived from the frozen SV
contract**, not chosen here — ``tt_cii_caracal_pkg.svh`` is the authority and the host must match it:

.. list-table::
   :header-rows: 1
   :widths: 30 18 52

   * - Frozen constant
     - Value
     - Consequence for the VRF
   * - ``CII_NUM_SRC_REQ`` / ``CII_NUM_SRC_DAT_RSP``
     - 4
     - **Four** coprocessor read ports (``R5``–``R8``). The VPU wrapper returns "member ``k`` of
       ``{VS1, VS2, VS3, VM}`` in one dat beat", so all four lanes can be live in the same cycle.
   * - ``CII_NUM_DST_WB``
     - 1
     - **One** coprocessor write port (``W2``). The constant is flagged *must match the*
       ``tt_cii_interface`` *default* — the relay's ``type(wb_data)`` resolves against it, so an
       override that differs is a port-connection mismatch.

.. warning::

   Earlier drafts said **7R/4W**, on the reasoning that "the CoProcessor needs 2 read ports, not 4"
   because a *pull* interface "can consume at most two VRF reads per cycle". That argument was wrong
   in both directions once the SV froze: the pull interface has **four** lanes, and it has **one**
   write port, not two. Note that ``tt_cii_caracal_pkg.svh`` also contradicts itself here — the
   inline comment on ``CII_NUM_SRC_REQ = 4`` still reads "2 VRF read ports for CII (5,6)". The
   **value** is authoritative, not the comment.

``R2`` is likewise not in the oldest drafts: it is the LCB's read of ``stale_pvdest``, required for
tail-/mask-undisturbed loads — see below.

Segmented-LS temporaries (``pvtmp``) are ordinary VRF groups and use these same ports — the
LSU/coprocessor read or write ``pvtmp`` on the ports already listed (it is a destination for one half
and a source for the other), adding no new ports.

Because the partition is static, **the CII can never stall on a VRF write port**: it owns ``W2``
outright. That invariant is load-bearing, because the CII Writeback channel is credit-metered and has
no back-pressure line — a port conflict would have nowhere to go. One port suffices precisely because
``CII_NUM_DST_WB = 1``: the coprocessor cannot present two result beats in one cycle, so a second
port would never be driven.

.. _old-vd:

Undisturbed lanes always come from ``stale_pvdest``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Under renaming, ``pvdest`` is a **fresh** group; the destination's previous contents live in
``stale_pvdest``. One rule therefore governs every undisturbed case in the machine:

.. important::

   ``stale_pvdest`` is the architectural old-``vd`` group and is the **only** source of undisturbed
   lanes (``vta = 0`` tail, ``vma = 0`` masked-off, and any ``vstart > 0`` prefix). A consumer needing
   old-``vd`` reads ``stale_pvdest`` — **never** ``pvs3``.

``pvs3`` and ``stale_pvdest`` are **two independent ``MicroOp`` fields naming two independent physical
groups**, and both are exposed to the coprocessor as **separate CII source slots**:

- ``pvs3`` is the group holding an **explicitly encoded third source operand** — the store-data
  register of ``vse.v vs3, (rs1)``, for instance.
- ``stale_pvdest`` is the group that held the destination architectural vreg **before this ``OP.v``
  renamed it**. It exists for *every* vector op with a destination, whether or not that op encodes a
  third source.

They **coincide** for read-modify-write arithmetic, and that is the common case: ``vfmacc.vv vd, vs1,
vs2`` computes ``vd += vs1 * vs2``, so its third source *is* the old destination and both fields name
the same group. They **diverge** whenever an instruction needs old-``vd`` for merging but its third
source is something else, or has no third source at all — a masked ``vadd.vv`` under ``vma = 0``, a
``vslideup`` whose untouched prefix comes from old-``vd``, a ``vcompress`` tail.

.. important::

   Because the two fields are separate, the **coprocessor decides what to pull**. It has four
   Src-Request lanes (``CII_NUM_SRC_REQ = 4``) and issues a request per slot it actually needs. When
   ``pvs3`` and ``stale_pvdest`` are the same group it pulls **one**; when they differ it pulls
   **both**, spending an additional lane. The host never collapses the two or guesses which was
   meant — it simply serves whichever slot is requested from the per-tag side-table. See
   :ref:`cii-operands`.

- **Arithmetic** merges happen **inside the CII coprocessor**, which pulls old-``vd`` as the
  ``STALE_VD`` operand slot (and ``pvs3`` separately, on the ``VS3`` slot, if the instruction encodes
  a third source).
- **Load** tail/mask fill happens **inside the LCB** before its ``W0`` write
  (:ref:`load-coalesce`): the LCB **pre-loads each assembly entry from the corresponding
  ``stale_pvdest`` member on ``R2``**, then overlays arriving elements. Only members that actually
  contain inactive lanes are pre-loaded — with ``VL`` known at execute that is typically the single
  partially-covered register plus any fully-inactive tail registers, not all 8. The pre-load overlaps
  the load's memory latency, so it is not latency-critical, and it preserves one ``W0`` write and one
  group-done per PRN.

  .. note::

     Earlier drafts said the inactive-lane source "comes from the assembly entry, not a new VRF read"
     and gave the Load Unit only ``index`` + ``mask`` read ports. Nothing initialized the assembly
     entry, so a ``vta = 0`` load with ``VL`` short of a full register (e.g. ``vle32.v v8, (a0)`` with
     ``vl=3``, ``SEW=32``, ``VLEN=256``) would have written junk into the undisturbed tail. ``R2``
     closes that hole.

- **The standalone ``VL = 0`` / fully-inactive ``vta = 0`` group copy** ``pvdest ← stale_pvdest``
  (see the VL == 0 case study) uses the Load Unit's ports, which are idle when no load is draining;
  under contention it arbitrates behind active load drains (the copy is rare and not
  latency-critical).
