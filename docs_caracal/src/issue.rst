.. _issue-chapter:

Issue
=====

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC

.. figure:: ../figures/dispatch_issue.png
   :align: center

   Overview of Dispatch and Issue Stages.

.. _dispatch-stage:

Dispatch Stage
--------------

In |boom|, dispatch is the pipeline stage that sits between rename and the issue queues —
it is the last in-order stage before instructions go out-of-order. This stage routes the
instruction into the appropriate IQ and reserves the LDQ/STQ slot in program order, which
only the in-order dispatch stage can do. The dispatch stage is essentially unchanged from
|boom| except modified to support vector op-codes and routing into the new ``IQ_V_*`` queues.

.. _vec-queue-reservation:

Vector element-queue reservation (in program order)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Dispatch acquires one more thing for a vector memory ``OP.v``: **capacity in the vector element
queues**, reserved in program order alongside the LDQ/STQ slot. The amount reserved differs by
direction, and the reason is structural rather than a tuning choice:

- **A vector store** may not dispatch unless its address queue AND its data queue have room for its
  **worst-case active element count**, computed from ``EMUL`` and ``EEW`` — both known at decode
  (e.g. ``SEW=8, LMUL=8`` at ``VLEN=256`` → ``8 × 256/8 = 256`` elements). The four-step deadlock
  argument in :ref:`ssi-queues` depends on the oldest store already holding its full capacity.
- **A vector load** reserves ``min(worstCase, ldResvMembers * VLEN/EEW)`` entries, with
  ``ldResvMembers`` defaulting to **4**, and streams the remainder in waves. A load reserves for
  **squashability**, not deadlock avoidance — it completes out of the Load Coalescing Buffer without
  gating on commit — so it has no deadlock exposure to protect against.

At execute, once ``VL`` is read from the VL RF, the **unused portion of the reservation is
released**.

.. note:: **Corrected (decision D9/D10).** An earlier revision of this paragraph applied the
   worst-case rule to *both* directions — "a vector load/store may not dispatch unless its target
   address queue … has room for its worst-case active element count". That made the load-streaming
   clause in :ref:`ssi-queues` **unreachable**, because
   ``worstCase = EMUL * VLEN/EEW = LMUL * VLEN/SEW = VLMAX >= VL >= active count``, so a load's
   active count can never exceed a worst-case reservation. The split above is what brings that
   clause into force. See :ref:`ssi-queues` for why under-reserving a load is deadlock-free.

**The release is tail-only.** The Vector AGEN — the stage that reads ``VL`` — may return the unused
portion **only while the reserving ``OP.v``'s region is still the youngest in that queue**, in which
case the release simply moves the tail pointer back. If any younger ``OP.v`` has already reserved
past it, the unused entries stay held and are freed in order with the rest of the region. A
mid-queue release is **not** permitted: it would punch a hole in the occupied region and break the
program-ordered-tail invariant that property 2 below depends on. The honest cost of the restriction
is that under a stream of vector memory ops the release usually cannot fire, so the worst-case
reservation, not ``VL``, is what bounds in-flight vector memory (:ref:`ssi-queues`).

This is the same discipline :ref:`rename-stage` already applies to the LDQ/STQ slot, extended to the
element queues, and it buys two unrelated properties that both turn out to be structural:

1. **Deadlock freedom** for stores — see :ref:`ssi-queues`.
2. **Squashability by pointer rollback** — because reservations are handed out in program order, each
   queue's occupied region is program-ordered, so a mispredict rolls the tail back to the branch's
   reservation index exactly as |boom| rolls ``stq_tail``. No per-entry ``br_mask`` is needed on
   queues that hold hundreds of entries. See :ref:`vec-squash`.

A special case exists for shared vector instructions.

.. _cii-shared-sched:

CII Shared Instruction Scheduling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Shared instructions (currently only segmented load/store, marked ``is_shared`` by the
Decoder) require more than one execution resource. In the single-stage scheme this is
handled at **dispatch time**: when a segmented load or store is renamed by the vector mapper,
it is dispatched to BOTH the CII IQ/coprocessor and its own Load/Store path. Its ``pvdest`` and
``pvtmp`` groups are allocated **all-or-nothing** (see the note under CII Shared Instruction Mapping
in :doc:`midcore`) — a partially allocated shared op would hold PRNs it cannot free until commit. A shared vector
*arithmetic* instruction does not apply — the coprocessor fully manages its own
resources — so shared handling applies only to vector load/store.

The two halves occupy two issue slots but share **one ROB entry**, which stays busy until both
halves report — tracked by the ROB's **1-bit "other half pending" flag** (see the ROB
Completion-tracking section), not a counter. The halves rendezvous on the ``pvtmp`` group in the
VRF (allocated by the vector mapper, see CII Shared Instruction Mapping): the producer writes it as
a destination and the consumer reads it as a source, woken by ``pvtmp``'s group-done on the vector
wakeup network like any vector operand.

The wakeup conditions differ for the two halves:

Segmented Load
^^^^^^^^^^^^^^

1. The LSU treats ``pvtmp`` as its **destination** group and writes the loaded data into it.
2. The LSU is selected for issue when its **address operands** are ready — the scalar base and
   stride, the index vector for indexed forms, ``pvm`` when the op is masked, and ``pvl``. This is
   the same condition midcore.rst §"Segmented Load" states, named explicitly here. It is
   deliberately narrower than "all vector source operands": the LSU half of a segmented load never
   reads ``pvs3``, so gating on it would couple the half to an operand it does not use.
3. ``pvtmp``'s group-done wakes the coprocessor, which reads ``pvtmp``, transposes, and writes
   ``pvdest``.


Segmented Store
^^^^^^^^^^^^^^^

1. The coprocessor treats ``pvtmp`` as its **destination** group and writes the transposed data
   into it.
2. ``pvtmp``'s group-done wakes the store IQ slot's **DGEN** path (not AGEN — see below).
3. The LSU treats ``pvtmp`` as its **source** group, reads it from the VRF, and writes memory once
   committed.

.. _shared-store-chain:

The segmented-store dependency chain
""""""""""""""""""""""""""""""""""""""

The two halves of a segmented store are not independent: the coprocessor half cannot start until the
LSU half has *already* run its address path. The full chain is six steps, and it must be respected by
the implementation:

.. code-block:: text

   1. LSU half AGEN                (needs base GPR, + index vector, + mask — all from older producers)
   2. → group-safe / clr_unsafe    (lsu.scala:1443, on do_st_search)
   3. → PNR advances past this ROB entry
   4. → coprocessor half becomes PNR-eligible in IQ_V_ALU and is granted
   5. → coprocessor transposes, writes pvtmp, emits group-done
   6. → LSU half DGEN reads pvtmp as store data

Two requirements follow:

- **AGEN and DGEN of the same issue slot must be independently grantable, in that order, separated by
  a long and variable delay** (steps 1 and 6 can be hundreds of cycles apart). The slot cannot treat
  AGEN+DGEN as a single grant.
- **There is no circular wait**, so the chain cannot deadlock: every operand step 1 depends on comes
  from an instruction *older* than the segmented store, and the coprocessor half's PNR eligibility
  depends only on its own store's address translation. It is a long serial chain, not a cycle.

**What makes the shared ROB entry pass the PNR.** The entry carries **one** ``rob_unsafe`` bit, and
the LSU half's first address translation clearing it is **sufficient** — nothing waits on the
coprocessor half. The PNR tracks the resolution of *speculation*, not completion, so a ROB entry with
a half still un-issued is treated no differently from any other safe entry; completion is what the
ROB's "other half pending" flag tracks, separately. Requiring *both* halves safe is what would close
the cycle above: the coprocessor half cannot issue until the PNR passes the entry, so gating the PNR
on that half issuing would deadlock.

This chain is also why ``IQ_V_ALU`` must not be a head-only FIFO: step 4 can stall for the duration of
a 256-element translation pass, and in a head-only queue that stall would block every younger vector
arithmetic op (see the Issue/Scheduling Stage).

.. _issue-sched-stage:

The Issue/Scheduling Stage
--------------------------

Like |boom|, |caracal| uses split issue queues. The scalar queues — ``IQ_MEM``,
``IQ_UNQ``, ``IQ_ALU``, ``IQ_FP`` — are **unchanged** from |boom|. |caracal| adds three
vector queues: ``IQ_V_LOAD``, ``IQ_V_STORE``, and ``IQ_V_ALU``. Each vector IQ may hold
any datatype.

All queues issue in a **single** scheduling stage; |caracal| does **not** add a second issue
stage, and a non-shared vector ``OP.v`` occupies exactly one issue slot in one queue and is granted **once**,
when all of its operands (scalar feeders and vector registers) are ready. The *selection policy*,
however, differs by queue:

.. note:: **Read "granted once" as once per EXECUTION RESOURCE, not as a literal grant count.**
   A vector **store** slot is granted **twice** — AGEN, then DGEN — and must be
   (:ref:`shared-store-chain`); the ``squash_grant`` / re-busy replay is likewise a second grant
   of the same select. A generator that asserted ``PopCount(grants per slot) == 1`` over an
   entry's lifetime would break every vector store. The obligation the "once" is protecting is
   that an ``OP.v`` is *allocated and selected* once — there is no second issue stage — not that
   a slot fires exactly one grant.

- **``IQ_MEM``/``IQ_UNQ``/``IQ_ALU``/``IQ_FP`` and ``IQ_V_LOAD``/``IQ_V_STORE``** reuse |boom|'s
  **age-ordered collapsing** Issue Queue and its priority-encoder select unchanged — they grant the
  **oldest *ready*** entry and may skip a not-ready older entry (out-of-order issue among ready ops).
  This is correct for vector loads/stores because the V-LSU is out-of-order.
- **``IQ_V_ALU`` is age-ordered collapsing with a *per-entry* past-PNR gate.** Like the other queues
  it grants the **oldest *ready*** entry, but an entry is only *eligible* once it is **past the PNR** —
  its ROB entry is older than ``rob.io.rob_pnr_idx`` (see the ROB Point-of-No-Return logic) — so every
  op handed to the CII is individually non-speculative, RoCC-style. The cost is latency: a vector
  arithmetic op cannot start on the CII until older branches have resolved and older loads have
  disambiguated (the PNR has swept past it).

  .. note::

     **Why this is not a head-only FIFO.** Earlier drafts made ``IQ_V_ALU`` a strict in-order FIFO,
     presenting only its oldest entry, justified as "it feeds the **in-order** CII in program order."
     That conflates two different things. The VPU being in-order means it processes **one instruction
     at a time**; it does *not* require **program-order issue**, because rename has already resolved
     every register dependence before an op crosses the CII. The three things that could have required
     program order do not:

     - **vtype/vl delivery** — would require it only if configuration arrived out-of-band via a
       serializing ``VCONFIG`` write. It does not: vtype/vl/vstart/vxrm ride the **per-instruction
       issue packet** (:ref:`cii-issue-packet`).
     - **``vxsat``/``fflags`` precision** — accumulated at commit in ROB order, so unaffected.
     - **tag/credit model** — tags are opaque and results already "may return out of order."

     The head-only variant had a concrete cost: a segmented store's coprocessor half sits at the head
     unable to issue until its *own* LSU half has translated its entire element set
     (:ref:`shared-store-chain`), blocking **every** younger vector arithmetic op behind it for
     potentially hundreds of cycles of dead VPU time. Age-ordered issue with a per-entry PNR gate
     removes that head-of-line block while preserving the non-speculative property exactly.

  Squashed ``IQ_V_ALU`` entries are dropped from the queue before they ever issue. Ops **already
  accepted by the CII** are a separate matter: past-PNR is *not* a commit guarantee, and a ROB-head
  flush can squash them — see :ref:`cii-flush`.


Wakeup Networks
~~~~~~~~~~~~~~~

|boom| wires each issue queue only to the wakeup network of its own register space (the
integer queues listen to integer writebacks, the FP queue to FP writebacks). Because each
register space has its own physical-register numbering and its own wakeup ports, operand
matches across spaces never collide. |caracal| preserves this partitioning:

- The scalar queues (``IQ_MEM``/``IQ_UNQ``/``IQ_ALU``/``IQ_FP``) are **bit-identical** to
  |boom| — they connect only to the existing integer/FP wakeup networks.
- **Only the three ``IQ_V_*`` queues connect to the vector wakeup network** (driven by
  **group-done** events — one per completed destination group, each carrying the completing group's
  full member-PRN vector; see :ref:`group-done wakeup <group-done>`). A vector slot matches its
  source members against that vector. Each queue also connects to whichever scalar networks supply
  its scalar feeders:

  - The **integer** network — for the base address, stride, and ``.vx`` scalar operand
    (GPR-sourced). Needed by **all three** vector queues.
  - The **VL** network — for ``pvl`` (its own register space, see :ref:`vl-vtype-rename`).
    Needed by **all three** vector queues. (``vtype`` is not woken — it rides the ``VConfig``
    snapshot.)
  - The **FP** network — for the scalar-FP source of ``.vf`` vector floating-point ops and
    ``vfmv.*.f``. Needed by **``IQ_V_ALU`` only**; ``IQ_V_LOAD``/``IQ_V_STORE`` need no FP
    network because vector memory addressing uses only GPRs.

Vector and scalar operand matches therefore never collide — the same separation |boom|
already relies on for int vs FP — and no pure-scalar slot ever pays for vector match ports.


The Vector Issue Slot
~~~~~~~~~~~~~~~~~~~~~

Only the ``IQ_V_*`` slots are extended; scalar slots are unchanged. A vector slot is a
**superset slot** that tracks both operand classes:

- **Scalar feeders**: the base address, stride, and any ``.vx`` integer operand (reusing the
  existing ``prs1``/``prs2`` operand slots) matched against the **integer** wakeup network, plus
  ``pvl`` matched on the **VL** network (its own register space; ``vtype`` is not an operand —
  it rides the ``VConfig`` snapshot). Vector floating-point ALU ops (``.vf`` forms and
  ``vfmv.*.f``) additionally source one scalar **FP** register, matched against the **FP** wakeup
  network; this applies to ``IQ_V_ALU`` only.
- **Vector operands**, matched against the **vector** wakeup network: ``pvs1``/``pvs2``/
  ``pvs3`` and the mask ``pvm`` (V0). Each source group holds its **member PRNs** (up to ``EMUL``),
  each with its own busy bit; the slot matches every member against the group-done's member-PRN
  vector and **AND**\ s them into a per-operand group-ready bit, woken only when the last member is
  ready (see :ref:`group-done wakeup <group-done>`). This per-member match — not a single
  base comparator — is the area/timing cost of the vector slot, and is unavoidable because a source
  group may be a sub-range of, or fragmented across, larger destination groups.

**The match-port budget.** A vector slot matches each of its up-to-``EMUL`` member PRNs per source
group against **all ``numVecWbPorts`` group-done ports** each cycle, every port carrying up to
``MAX_MEMBERS`` PRNs, plus **one** VL wakeup port for ``pvl``. The width is not a free parameter: the
vector wakeup network is ``numVecWbPorts`` wide (midcore.rst §"Busy Table"), so a slot that matched
fewer ports could miss a group-done. The comparator count per source group is therefore
``EMUL × numVecWbPorts × MAX_MEMBERS`` — at ``EMUL = MAX_MEMBERS = 8`` that is 64 comparators per
writeback port per group, which is why the per-member match dominates the slot's area.

An ``OP.v`` asserts ``request`` only when **all** of its operands — scalar and vector — are
ready:

.. code-block::

   request := slot_valid && !iw_issued && scalar_operands_ready && vector_operands_ready

For **``IQ_V_ALU``** each entry additionally gates on being non-speculative (RoCC-style), so its
eligibility is ``request && is_older(rob_idx, rob_pnr_idx)`` — a **per-entry** gate, applied to every
slot rather than only to the head, see the Issue/Scheduling Stage.
``IQ_V_LOAD``/``IQ_V_STORE`` do not gate on the PNR (vector memory may
issue speculatively; the LSU handles ordering/replay and stores write memory only post-commit).

For vector **stores** the existing mem-slot AGEN/DGEN split is extended: the
data-generation (DGEN) path is gated on the vector store-data operand (matched on
the vector network) rather than on ``prs2`` as in the scalar mem slot. **The gated operand is
selected by ``is_shared``:**

.. code-block::

   dgen_operand := Mux(uop.is_shared, uop.pvtmp, uop.pvs3)

.. warning::

   Gating DGEN unconditionally on ``pvs3`` is **incorrect for a segmented store**, and earlier drafts
   did exactly that. For a segmented store ``pvs3`` is the **coprocessor** half's source group, and it
   is ready long before the transpose has run; the LSU half's data source is ``pvtmp``, written *by*
   the coprocessor. Waking DGEN on ``pvs3`` therefore reads ``pvtmp`` before it is written and stores
   garbage. DGEN must wake on ``pvtmp``'s group-done whenever ``is_shared`` is set.

**The deselected operand's busy bit does not participate.** When ``is_shared`` is set, ``pvs3``'s
busy bit is **excluded** from ``vector_operands_ready`` for the LSU half — that half never reads
``pvs3``, which is the *coprocessor* half's source group. This is the same shape as the ``pvm`` rule
below: the mux selects the operand, and only the selected one is waited on. Keeping ``pvs3`` in the
readiness term would happen to work today, because it is ready long before the transpose runs, but
it would turn that timing accident into a correctness dependence.

The mask ``pvm`` is read conditionally — its busy bit only participates in ``request`` when
the OP.v is masked (encoded ``vm`` bit clear). Unmasked ops leave ``pvm`` don't-care so a
stale mask preg is never waited on.

**``stale_pvdest`` is an implicit source operand, and it must be waited on.** A vector ``OP.v``
with a vector destination reads its own ``stale_pvdest`` group — the group that held the
destination architectural vreg before this ``OP.v`` renamed it (:ref:`old-vd`) — even though no
source field of the instruction names it. An ``IQ_V_LOAD`` entry reads it when the Load Coalescing
Buffer pre-loads undisturbed lanes on ``R2`` under ``vta = 0``/``vma = 0``
(:ref:`load-coalesce`), and an ``IQ_V_ALU`` entry reads it whenever the coprocessor pulls the
``STALE_VD`` source slot (:ref:`cii-operands`). An ``IQ_V_LOAD`` or ``IQ_V_ALU`` entry must
therefore not be granted until its ``stale_pvdest`` group is ready. It is matched on the **vector**
wakeup network exactly as ``pvs1``/``pvs2``/``pvs3``/``pvm`` are. ``IQ_V_STORE`` carries no such
matcher: a store has no vector destination, so it has no stale group.

.. danger::

   **Age-ordered issue does not make this redundant.** ``stale_pvdest`` names the *previous*
   mapping of the destination arch vregs, so its producer is always an older instruction — but the
   vector issue queues grant the oldest **ready** entry, which is not the oldest **complete** one.
   An older producer that has not yet emitted its group-done leaves the stale group busy while a
   younger consumer is granted, and both the ``R2`` pre-load and the ``STALE_VD`` pull then read an
   unwritten PRN. The failure is silent: the lanes that should have been preserved instead contain
   whatever the free list last left in that register.

**The ``stale_pvdest`` match must be per member, not an aggregate bit.** ``stale_pvdest`` may span
up to ``MAX_MEMBERS`` producers: if an ``LMUL = 1`` ``OP.v`` writes ``v0`` and a later
``LMUL = 8`` ``OP.v`` renames ``v0``–``v7``, the latter's stale group is the current mapping of
eight architectural vregs, installed by up to eight different instructions. A single aggregate busy
bit cannot express "waiting on the third of eight", and one group-done cannot clear it correctly.
This is the same argument that forces per-member matching on ``pvs*`` above.

**The ``IQ_V_ALU`` gate is deliberately conservative.** Whether the coprocessor actually pulls
``STALE_VD`` for a given op is the VPU decoder's decision and is not visible to the host — the
issue packet carries no hint of it, and no back-channel exists. Every ``IQ_V_ALU`` entry with a
vector destination must therefore wait on ``stale_pvdest`` readiness, including the ops that will
never pull it.


.. _issue-vl-delivery:

VL delivery
-----------

VL is a source operand in its **own register space** (see :ref:`vl-vtype-rename`), always renamed
into the VL RF — there is no decode-time VL value. ``pvl`` is woken on the **VL** wakeup network like
any operand — a plain readiness wakeup, no value capture in the slot — and the value is read from the
VL RF at execute, when the Vector AGEN cracks the ``OP.v`` into element accesses. (``vtype`` is not
an issue operand — it rides the ``VConfig`` snapshot from decode.)

- **vsetivli**: VL is immediate — the VCFG writes the VL RF in the front-end (no EU); ``pvl`` may
  already be ready when the consumer dispatches.
- **vsetvli / vsetvl**: executed on an **integer ALU EU** (woken by ``rs1``/``rs2`` on the integer
  network); the ALU's VL writeback targets the VL RF and wakes ``pvl``. ``vsetvl`` is additionally
  ``is_unique`` (see :ref:`vector-rvv-decode`).





.. _issue-key-features:

Single-Stage Scheduling Key Features
------------------------------------

The single-stage approach with extended vector slots offers the following benefits:

1. **The scalar datapath is untouched.** Only the ``IQ_V_*`` queues are extended and connect
   to the vector wakeup network; the scalar queues are bit-identical to |boom|.
2. **VL is an operand in its own register space** — ``pvl`` is woken on the VL network and the value
   is read from the VL RF by the vector EU at execute (see :ref:`vl-vtype-rename`).
3. **A vector ``OP.v`` is allocated and selected once.** There is no second issue stage, so
   there is no double allocation, no second priority-encoder select, and no cross-queue
   kill/replay to keep consistent.
4. **Enables a detached in-order CII co-processor.** ``IQ_V_ALU`` is **age-ordered with a per-entry
   past-PNR gate**: like RoCC it issues only instructions that are individually **past the PNR**
   (non-speculative), so the CII needs no *branch* kill — the PNR cannot sweep past an unresolved
   branch. It does need a **drain-on-flush** path for ROB-head flushes, because past-PNR is not a
   commit guarantee (:ref:`cii-flush`). Segmented-LS shared instructions rendezvous via the ``pvtmp``
   group.
5. **Cracking of vector instructions in the frontend is unnecessary**, made possible by the
   atomic LMUL vector mapper and AGEN-time element cracking.
