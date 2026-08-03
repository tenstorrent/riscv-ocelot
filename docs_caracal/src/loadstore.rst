Loadstore
=========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC

.. _lsu-unified:

The Unified Load/Store Unit (LSU)
---------------------------------

A major feature of |caracal| is that it extends the |boom| Load Store Unit to support
scalar and vector load stores. We call this the Unified Load Store Unit.

If the ``usingRVV`` parameter is disabled the underlying |boom| LSU is essentially untouched.
When enabled, several vector address and data queues are initialized. Vector load/store
``OP.v``'s are still allocated to the existing STQ and LDQ as a **single entry**, using largely
the same dispatch-stage logic.

As vector memory accesses often produce many D$ accesses to read or write a whole vector,
we allocate separate vector address and data queues to buffer those effective ``nOP.v`` memory
operations. This prevents the primary STQ and LDQ from being polluted by many vector memory
operations and impacting scalar load/store performance. When a vector LDQ or STQ entry becomes
ready for execution, it reads the effective address or store data from the separate address and
data queues, rather than from a uOP as is the case with scalar load stores.

The conditions that allow a vector LDQ or STQ entry to become eligible to drain are the same as
scalar. Here **atomic** means an entry drains *all* of the element accesses it owns in program /
element order without another op interleaving at that entry. It **is** all-or-nothing with respect to
architectural state: a faulting load/store commits **no** elements, traps with ``vstart = 0``, and
restarts the whole instruction from element 0 (see :ref:`elem-progress`). The element accesses
themselves contend for the shared D$ port through the arbiter in :ref:`dcache-arbiter`, so a vector
drain does **not** monopolize the cache.

.. warning::

   Earlier drafts said the opposite here — that atomic "does **not** mean all-or-nothing: a faulting
   load/store still commits the elements before the fault and records ``vstart``." That contradicts
   :ref:`elem-progress`, and the mid-stream-resume model it described is not implementable in this
   machine: the faulting ``OP.v``'s ``pvdest`` group is a *fresh* group that a non-committing
   instruction never installs, and it is reclaimed to the free list on the exception, so elements
   ``0..k-1`` were never architecturally visible to commit in the first place.

For ``LargeBoomV4Config`` or ``MegaBoomV4Config`` we take advantage of the dual-port L1 D$
interface and allow 2 memory operations to be issued per cycle; the vector address and data queues
are ``2 x nOP.v`` wide to allow 2 concurrent element accesses.

Queue-naming convention. Vector queues are named ``{ld,st}_{SSI,US}_{ADDR,DATA}_Q`` —
``ld_SSI_ADDR_Q``, ``st_SSI_ADDR_Q``, ``st_SSI_DATA_Q``, ``ld_US_ADDR_Q``, ``st_US_ADDR_Q``,
``st_US_DATA_Q``.


.. _ssi-queues:

Strided, Segmented, Indexed Queue
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This class of vector (SSI) queues holds effective addresses calculated for strided, indexed, and
segmented load stores. There exists a ``ld_SSI_ADDR_Q`` and a ``st_SSI_ADDR_Q``.

Because the SSI queues hold essentially element-wise memory addresses, their default parameterized
size is quite big at 512 entries. The ``ld_SSI_ADDR_Q`` and ``st_SSI_ADDR_Q`` receive calculated
effective addresses from the stage 1 vAGENs.

**Capacity is reserved in program order at dispatch** (:ref:`vec-queue-reservation`), not claimed
opportunistically at execute. That is a correctness requirement, not a tuning choice:

- **Stores must hold their full active address set pre-commit.** A store cannot write memory until
  it commits, and every element's fault must be detected *before* commit (precise exceptions). So a
  store's translated addresses are **retained in ``st_SSI_ADDR_Q`` from execute until commit-drain**
  — they cannot be streamed/freed mid-instruction. The worst-case single-instruction element count
  with ``VLEN=256`` is ~256 element-accesses (``SEW=8`` × ``LMUL=8`` = 256; segmented
  ``NF*EMUL ≤ 8`` also caps at 256).
- **Loads may stream.** A load completes out of the Load Coalescing Buffer (:ref:`load-coalesce`)
  and does not gate on commit, so the ``ld_SSI_ADDR_Q`` may be drained in waves and a load whose
  active element count exceeds its reservation is streamed through it.

.. danger::

   **Why opportunistic allocation deadlocks the machine.** Earlier drafts sized the queue for one
   worst-case store and let "additional in-flight stores **back-pressure the store vAGEN** when the
   queue is full." Combined with age-ordered-*ready* issue from ``IQ_V_STORE`` — which "may skip a
   not-ready older entry" — that deadlocks:

   1. Older store **A**'s operands aren't ready, so younger **B** and **C** are granted first and
      fill the queue.
   2. **A** becomes ready, issues, and back-pressures at the vAGEN — no room.
   3. **A** cannot finish translating its element set, so **A** cannot commit.
   4. Commit is in-order, so **B** and **C** cannot commit — so they never reach commit-drain and
      never free their entries. Nothing moves.

   It is a partial-fill deadlock too: **A** can occupy part of the queue and wedge against **B**.
   Liveness under that scheme would require
   ``depth ≥ max_elements_per_store × max_in_flight_vector_stores`` — with
   ``numVecStoreQueueEntries = 64`` that is ``64 × 256 = 16384`` entries. **Sizing cannot fix this;
   the reservation is what fixes it,** because the oldest store always already holds its capacity.

.. note::

   **Loads do not have this failure mode** and the reservation is not what protects them: a load
   completes out of the LCB without waiting for commit, so a younger load always drains and frees its
   entries, and an older load merely stalls. Loads are reserved in program order for the *other*
   reason — it is what makes the queue squashable by pointer rollback (:ref:`vec-squash`).

**Depth is now a published architectural limit.** Because reservation happens at dispatch against the
worst-case element count, queue depth directly bounds in-flight vector memory ops: 512 entries ÷ 256
worst-case elements = **2** worst-case stores in flight. Depth should therefore be chosen from a
target memory-level parallelism, not from "fits one max store with headroom", and the derived
in-flight limit should be stated wherever the parameter is documented.

**Minimum depth is a liveness constraint, not a tuning choice.** Each element-granular queue —
``ld_SSI_ADDR_Q``, ``st_SSI_ADDR_Q`` and ``st_SSI_DATA_Q`` — must be at least the worst-case
single-instruction active element count, which is ``VLEN`` elements (``SEW=8`` × ``LMUL=8``;
segmented ``NF*EMUL ≤ 8`` reaches the same bound). This is checked at elaboration. Below the floor a
single store can never complete its dispatch reservation, so it never issues, never commits, and
nothing behind it ever frees — the same wedge the danger note above describes, reached by
undersizing rather than by opportunistic allocation. The unit-stride queues are exempt: they hold one
``nOP.v`` per instruction rather than one per element (:ref:`us-queue`), so an element-count floor
does not apply to them.


.. _us-queue:

Unit-Strided Queue
~~~~~~~~~~~~~~~~~~

We specially optimize unit-strided load/store performance. Unit-strided load/stores have very
simple address generation and only require a base address, VL, and EEW to generate all addresses.
Thus we have a separate ``ld_US_ADDR_Q`` and ``st_US_ADDR_Q`` to hold these transactions. Unlike
the SSI queues, a unit-stride load/store need only generate a **single** ``nOP.v`` from the stage 1
vAGENs that describes the whole contiguous range ``[base, base + VL*EEW)``.

A second Packer-based AGEN unit fires when a unit-strided STQ/LDQ entry is activated and expands
that single ``nOP.v`` into per-element D$ accesses *just-in-time*.

Because a unit-stride access is a single contiguous byte range, its disambiguation and forwarding
are done as **one range-overlap check** rather than per element — see :ref:`mem-order`.


.. _store-data-queue:

Store Data Queue
~~~~~~~~~~~~~~~~

Instead of holding the store data in the STQ entry, which would not be feasible for vector data, a
separate ``st_SSI_DATA_Q`` and ``st_US_DATA_Q`` buffer the store data. The ``st_SSI_DATA_Q`` entry
width is one element (``ELEN``, 64 bits); the ``st_US_DATA_Q`` entry width is a full ``VLEN`` (256
bits), so a unit-stride store reads its entire source ``vPRN`` in one access and the Packer slices
out per-element data at drain.

The store data is **read from the source ``vPRN`` at DGEN (execute time)** and held in the data queue
until the post-commit drain. Because the data is captured this early, the source ``vPRN`` needs **no
pin** — it frees with the rest of the stale group at commit (a later writer to the same architectural
vreg cannot clobber an in-flight store's data, since that data already lives in the queue). The cost
is that, like the SSI address queue, the ``st_SSI_DATA_Q`` must hold a full store's active element
data pre-commit. It is therefore **reserved in program order at dispatch on exactly the same terms as
the address queue** (:ref:`vec-queue-reservation`) and for the same reason — opportunistic allocation
with age-ordered-ready issue deadlocks (see the danger note in :ref:`ssi-queues`). A vector store
reserves capacity in *both* queues or does not dispatch.


.. _load-coalesce:

Load Coalescing Buffer
~~~~~~~~~~~~~~~~~~~~~~

A vector load returns at most one element (``≤ ELEN``) per D$ response, and those responses may
arrive **out of order** across MSHRs, while a destination physical register is a full ``VLEN``.
Writing the VRF once per element would need an impractically wide, byte-masked write port and would
break the one-write-per-PRN (and one-**group-done**-per-instruction) assumption that the ROB
single-shot busy-clear and the vector Busy Table rely on (see :ref:`group-done-wb`).

To bridge this, |caracal| adds a **Load Coalescing Buffer (LCB)** in front of the VRF write port:

1. The LCB holds a small number of in-flight ``VLEN``-wide assembly entries, one per destination
   PRN currently being filled (parameterizable; default a few entries per in-flight vector load).
2. Each returning element is written into its byte offset within the assembly entry for its
   destination PRN, using the element index carried on the ``nOP.v``.
3. When **all active bytes** of a destination PRN are present (tracked by a per-entry byte-valid
   bitmap, bounded by the active-element set the load vAGEN selected), the LCB issues a **single**
   VRF write on ``W0`` for that PRN. The LCB **must not re-derive which elements are active**: it
   consumes the **same mask-derived element cursor** as ``ld_vAGEN_1`` (see :ref:`vector-agen`), so
   the bytes it waits for are exactly the bytes the AGEN generated accesses for and a masked-off
   element is never counted as outstanding.
4. Inactive (masked-off or tail) byte lanes — the complement of that cursor — are filled per
   ``vta``/``vma`` policy before the write,
   so the single VRF write leaves no stale bytes. **For undisturbed policy (``vta = 0`` / ``vma = 0``)
   the inactive-lane data is pre-loaded from ``stale_pvdest`` on VRF read port ``R2``** before the
   arriving elements are overlaid — see :ref:`old-vd`. Only members that actually contain inactive
   lanes are pre-loaded, and the pre-load overlaps the load's memory latency, so it costs no extra
   latency and preserves one ``W0`` write per PRN.
5. The LCB also owns the **per-group PRN-done count** (target = the destination-group size from
   ``v_emul``/``v_seg_nf``). When the **last** destination PRN of the group is written, it emits
   **one group-done** carrying the group's member-PRN vector. That single event is what the ROB, the vector
   Busy Table, and the vector wakeup network all consume (see :ref:`group-done-wb` and
   :ref:`group-done wakeup <group-done>`) — there is no per-PRN ROB writeback or per-PRN vector wakeup. The
   intermediate per-PRN VRF writes are visible only to the regfile.

**Entries are allocated before the accesses that fill them.** An assembly entry is allocated for a
destination PRN *before* any element access for that PRN is issued to the D$; if no entry is free the
load drain stalls at the arbiter (:ref:`dcache-arbiter`). A returning element therefore always has an
entry waiting, and the LCB never has to back-pressure a D$ response — which it could not do safely,
since a blocked response holds an MSHR against the drain that would free the entry. LCB depth is
consequently a memory-level-parallelism knob, not a correctness parameter.

The LCB is what makes "one writeback per destination PRN" true for the regfile — and **one
group-done per instruction** true for the ROB / Busy Table / wakeup network — even though the cache
returns data element-by-element and out of order.


.. _elem-progress:

Per-Element Progress Tracking and Precise Exceptions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A vector LDQ/STQ entry owns up to ``VLMAX`` element accesses but is a single entry, so it carries an
**element cursor** in addition to the scalar fields:

- ``elem_next`` — the index of the next element to drain.
- ``elem_done`` — count (or bitmap) of elements that have completed.
- ``fault_elem`` — the index of the **oldest** faulting element, latched on the first fault. It is a
  **stop signal and a debug/perf counter only** — it is *not* carried to the ROB and *not* written to
  ``vstart``.

Element accesses **fire** (TLB + LCAM) in element order, so a fault at element ``k`` is always
detected before any element ``> k`` fires. Element **responses**, however, return **out of order**
across MSHRs (:ref:`load-coalesce`), so "nothing beyond ``k`` has been written" is *not* an invariant
this design maintains — and it does not need to be.

**On a fault the entry stops advancing and the instruction traps with ``vstart = 0``, restarting from
the beginning.** This is legal because nothing was written architecturally: ``pvdest`` is a *fresh*
physical group that a non-committing instruction never installs in ``com_map_table``, and stores drain
to the cache only post-commit. Loads are idempotent, so the restart is free of side effects; the cost
is re-draining elements ``0..k-1`` after the handler returns.

**Element accesses still outstanding when the fault is latched need no cancellation.** Because
accesses fire in element order, no access beyond ``fault_elem`` was ever issued, so the only
outstanding accesses are for elements below it. Those are left to return and are dropped exactly as
on a branch squash: the trap invalidates the load's LCB assembly entry by its owning ``ldq_idx``, so
a late response has nowhere to land (:ref:`vec-squash`). The PRN-recycling hazard in that section's
danger note applies here unchanged — the faulting instruction's ``pvdest`` group is reclaimed to the
free list, so a late LCB write must be made **impossible**, not merely harmless.

.. warning::

   **Do not resume at ``vstart = k``.** Earlier drafts trapped with ``vstart = fault_elem`` so the
   instruction could "resume mid-stream per RVV 1.0". Because the faulting instruction's ``pvdest``
   group is reclaimed to the free list on the exception, elements ``0..k-1`` are **not** architecturally
   visible — so resuming at ``k`` leaves them holding pre-instruction values while RVV requires them to
   hold loaded data. That is silent data corruption on any page-crossing gather. The full argument is
   in the Precise-exceptions section of :doc:`midcore`.

Fault-only-first (``vleff.v``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The fault-only-first unit-stride load has special semantics: a fault on **element 0** is reported
as a normal precise trap (``vstart = 0``), but a fault on **element ``i > 0``** must **not** trap —
instead VL is trimmed to ``i`` and the instruction completes with the elements it did load.

``vleff`` is **not** serialized — it is an ordinary speculative vector load. It is a
**VL producer**: on completion it writes the final element count to its VL register-file destination
— the full VL if no fault occurred, or ``i`` if element ``i > 0`` faulted — exactly as a vset writes
VL. That write wakes ``pvl`` in dependent vector slots on the VL wakeup network like any vset, so they
pick up the (possibly trimmed) VL through the normal ``pvl`` path, and the architectural ``vl`` CSR is
updated at commit. Only the element-0 fault raises ``rob_exception``; the ``i > 0`` case clears no
architectural state beyond trimming VL.

.. note::

   **``is_unique`` was removed from ``vleff``.** Earlier drafts marked it unique "so it is the only
   in-flight op when it resolves and it can update VL without racing younger uOPs." Both halves of
   that were wrong. ``is_unique`` does not make an instruction the only one in flight — it stalls only
   the unique uop's *own* dispatch until the ROB drains, after which younger uops dispatch one cycle
   later (see the warning in :ref:`vector-rvv-decode`). And the serialization was never needed: VL is
   renamed into the VL register file, so ``vleff``'s trimmed VL reaches consumers through ``pvl`` and
   the VL wakeup network like any other VL producer, with correct ordering by construction. Since
   ``vleff`` is hot in ``strlen``/``memchr``-style loops, serializing it would have been a significant
   and gratuitous cost.


.. _mem-order:

Memory Ordering and Disambiguation
----------------------------------

Vector load/store element addresses live in the SSI/US address queues, **not** in the scalar STQ.
The scalar LCAM (which searches the LDQ/STQ entries) therefore does not see them by default, so
|caracal| explicitly routes vector addresses through the disambiguation machinery so RVWMO ordering
between scalar and vector memory ops is maintained in **both** directions:

- **ST→LD ordering.** Each vector store address presented for execute
  is searched against the LDQ — both scalar LDQ entries and in-flight vector loads — exactly as a
  scalar store addr-gen drives the LCAM. A match on a younger, already-executed load sets that
  load's ``order_fail`` and replays it.
- **ST→LD forwarding.** Forwarding is **load-initiated**: each load address presented for execute is
  searched against both the scalar STQ and the vector store address queues, exactly as a scalar load
  drives the LCAM today. A match on an older vector store forwards that store's data, read from the
  ``st_*_DATA_Q`` (which already holds the store data, captured at DGEN). A store's own search
  (above) cannot forward, because a load that has not yet executed has no address to match against.
- **A vector store address is withheld from disambiguation until its data is captured.** An address
  queue entry is not presented to the LCAM until its corresponding ``st_*_DATA_Q`` entry is valid, so
  a forwarding match can never hit a store whose data has not yet been captured.
- **Scalar LD/ST vs. scalar ST/LD.** Unchanged from |boom|.

Which pairs forward:

- **Scalar load from a vector store.** Permitted for both US and SSI stores. Where several
  already-generated elements of the same SSI store alias the load, the **youngest matching element**
  supplies the data. A store element generated *later* that aliases the load is caught by that
  element's own ST→LD ordering search, which sets the load's ``order_fail`` — the replay path
  (:ref:`order-fail-replay`) is the correctness floor for this speculation.
- **Vector load from a vector store.** Permitted **only when both are unit-stride.** In the three
  other combinations — US store → SSI load, SSI store → US load, SSI store → SSI load — forwarding is
  not attempted and the younger vector load is **held** until the older store drains (below). Holding
  rather than replaying is what stops a multi-element vector load re-failing on every replay until
  the store commits; a scalar load costs at most one replay, which is why it forwards instead.

Granularity of the search differs by access class:

- **Unit-stride (US): one range check, not per element.** A US access is the contiguous byte range
  ``[base, base + VL*EEW)``. The LCAM comparison for US loads/stores is a **range-overlap** test of
  that whole range against each queue entry's address, performed once when the US entry executes —
  rather than expanding to per-element searches. This is both cheaper and sufficient: any overlap
  anywhere in the range triggers the ordering/forwarding action. For forwarding, an overlap that
  only partially covers the load is handled like any partial-forward case in |boom| — replay if the
  covering store data cannot fully satisfy the load. The range test is deliberately
  **mask-oblivious for ordering**: it over-approximates, and a false positive costs only a replay.
  **Forwarding is mask-qualified** — bytes belonging to inactive (masked-off or tail) elements of the
  store are never forwarded, and if the store's active bytes cannot fully cover the load the
  partial-forward rule above applies.
- **Strided / Indexed / Segmented (SSI): per element.** Scattered addresses cannot be range-folded,
  so SSI accesses search the LCAM per element ``nOP.v`` as they drain. These per-element searches
  contend for the shared LCAM through the same arbiter that gates the D$ port
  (:ref:`dcache-arbiter`), so they cannot starve scalar disambiguation.

**Edge case — an older in-flight vector store holds a younger overlapping vector load.**
Vector-to-vector store-to-load forwarding is attempted **only** for a US store to a US load. The
other three combinations do not forward, for two different reasons:

- **The store is SSI** (SSI store → SSI load, SSI store → US load). An SSI store generates its
  element addresses incrementally (one ``nOP.v`` at a time through the arbiter), so until the store
  has resolved *all* its active elements a younger load cannot know whether a not-yet-generated store
  element aliases it — nor which store element holds the youngest byte for an aliased address, since
  a later element of the same store may overwrite it (ordered-indexed or duplicate indices).
  Forwarding a whole vector load from a partially-resolved scatter is unsafe.
- **US store → SSI load.** The forward is safe in principle, but a per-element load would have to
  range-check and slice every element against the store's US range; |caracal| restricts
  vector-to-vector forwarding to the US/US case rather than build that path.

In all three, |caracal| instead **holds** the younger vector load that overlaps the older in-flight
vector store until that store completes (all its elements drain), then lets the load read the updated
cache line. Where both are SSI the two element streams also share the LCAM / D$ port through the
arbiter, so the pair executes **effectively serially**. The ordering-violation replay path
(:ref:`order-fail-replay`) remains the correctness floor if a load slips through before the
dependence is detected; the hold converts a multi-element replay storm into one clean serialization
when the overlap is known or predicted.

The hold reuses existing |boom| machinery rather than adding a structure:

- **Known** — an LCAM match between an already-generated store element address (or the store's US
  range) and the load's address.
- **Predicted** — |boom|'s existing memory-dependence predictor, the same one reused for
  memory-dependency speculation below.
- **Held by** — the younger load's existing per-load store-dependency block on the older vector store
  entry; the held load is simply not granted the LCAM / D$ port by the arbiter
  (:ref:`dcache-arbiter`). No new queue or state is added.
- **Released by** — that store entry's element cursor completing its active element set, i.e. the
  store's drain, after which the load reads the updated cache line.

Memory-dependency speculation (a load issuing past a store whose address is not yet known) reuses
|boom|'s existing predictor and ordering-violation replay path, now extended to fire on the
cross-queue matches above.

.. _order-fail-replay:

Ordering-Violation Replay
~~~~~~~~~~~~~~~~~~~~~~~~~~~

When a store's address search (scalar addr-gen, or a vector store's US range / SSI per-element
search) matches a **younger, already-executed** load, that load has speculated past the store and
may have read stale data. |caracal| recovers with |boom|'s existing ordering-violation path — the
same squash-and-refetch machinery a branch mispredict uses — **not** a selective replay of just the
load and its dependents. The mechanism, and why it is correct even though the load has already
written a physical register and possibly woken dependents:

1. **Mark, don't act.** The matching search sets the load's LDQ ``order_fail`` bit. The LSU
   broadcasts the *oldest* failing load to the ROB as a ``lxcpt`` with cause
   ``MINI_EXCEPTION_MEM_ORDERING``; the ROB records it as an exception on that load's row but takes
   no action yet.

2. **Deferred to the ROB head.** The flush fires only when the failing load reaches the **head** of
   the ROB. This is the load-bearing invariant: because commit is in-order and the load is then the
   oldest instruction in the machine, **every remaining in-flight instruction is younger than the
   load** — so all of its dependents are still un-committed and squashable. No consumer of the bad
   value can have escaped to architectural state.

3. **Mini-exception ⇒ refetch, not trap.** ``MINI_EXCEPTION_MEM_ORDERING`` is not an architectural
   exception: it does not go to the CSR/trap vector. It produces a pipeline flush with
   ``flush_typ = refetch`` — the frontend redirects to the failing load's **own PC** (from its
   ``ftq_idx``/``pc_lob``), so the load re-fetches and re-executes.

4. **The RF is not rolled back — renaming makes that unnecessary.** The load already wrote its result
   into a *physical* register ``Pd``, and dependents may have consumed it. On the flush, the physical
   RF is left untouched; instead the ROB rollback **restores the rename map to the committed
   architectural state and returns every speculatively-allocated physical register — including
   ``Pd`` — to the free list.** The stale value is simply orphaned: after rollback nothing maps the
   load's logical destination to ``Pd``. On refetch the load renames to a *fresh* physical
   destination, executes correctly (forwarding the store data from ``st_*_DATA_Q`` or reading the
   drained cache line), and writes that register.

5. **Dependents are handled by the coarse squash.** Because dependents are by definition younger than
   the load, the flush discards the load and everything younger — ROB rows, issue queues, and the
   pipeline — and frees their speculative physical registers in the same rollback. |caracal| does not
   track which specific instructions consumed the poisoned result; like branch recovery, it
   conservatively squashes all younger work, differing only in that the redirect PC is the load
   itself (``refetch``) rather than a branch target.

The window where the load writes back early and wakes dependents on bad data is therefore harmless:
all of that work is younger than the load and still un-committed when the flush fires at the ROB
head, so in-order commit guarantees it never reaches architectural state. The cost is purely
performance (a full refill from the load), which is what the memory-dependency predictor exists to
avoid by holding back loads likely to alias.

.. important::

   **This path is what breaks the "past-PNR ⇒ will commit" assumption for the CII.** A load's
   ``rob_unsafe`` bit is cleared on its **first address translation** (``lsu.scala:1443``), but
   ``order_fail`` is only discovered later — so the PNR sweeps past the load, vector arithmetic ops
   issue to the coprocessor, and the flush above then squashes them. |boom| documents the same thing at
   ``rob.scala:438-441``: *"In the case of a mem-ordering failure, the failing load will have been
   marked safe already."* Extending ``order_fail`` to cross-queue vector/scalar matches (above) makes
   this **more** frequent than in baseline |boom|, not less. The coprocessor-side contract that handles
   it is :ref:`cii-flush`.

For a **vector** load that order-fails, the same refetch/re-rename path applies at the granularity of
the whole vector instruction: the single LDQ placeholder entry drives one ``lxcpt``, and on replay
the vector op re-renames its whole destination group and re-drains its element accesses through the
LCB (:ref:`load-coalesce`) — there is no partial-group rewind, consistent with the one-group-done
completion model.


.. _dcache-arbiter:

D$ Interface Arbiter
--------------------

The unified LSU shares the D$ request lane(s) (``dmem.req``, ``lsuWidth`` wide — 1 on Medium, 2 on
Large/Mega) among several requestors:

- scalar LSU fire (load/store),
- vector **load** drain (US Packer or SSI per-element),
- vector **store** drain (post-commit, from ``stq_execute_queue``).

A **priority round-robin arbiter** grants the lane(s) each cycle:

- **Priority floor.** Scalar memory ops carry a higher base priority than vector drains, so a burst
  of vector element accesses never indefinitely blocks a scalar load/store. This keeps scalar memory
  latency close to baseline.
- **Round-robin anti-starvation.** Among requestors of equal priority, and to bound how long a
  lower-priority requestor can wait, a round-robin pointer guarantees each contending requestor at
  least one grant every *4* cycles. A long vector drain therefore yields the lane periodically so
  scalar traffic makes progress, and a steady scalar stream still lets the vector drain advance.
- **Dual lane (Large/Mega, ``lsuWidth=2``).** The arbiter grants up to 2 of the pending requests per
  cycle and is work-conserving: an all-scalar workload uses both lanes for scalar, an all-vector
  workload uses both for vector, and a mix splits them under the priority-floor + round-robin policy.

The same policy gates the **shared LCAM** and **TLB** ports, since a vector element stream consumes
those resources at the same rate it consumes D$ bandwidth. All three (D$ lane, LCAM port, TLB port)
are released back to scalar on the round-robin boundary so no single vector ``OP.v`` can monopolize
disambiguation or translation.


.. _fences:

Fences
------

``fence`` (RVWMO ``fence rw,rw`` and friends), ``fence.i`` and ``sfence.vma`` must order vector memory
ops alongside scalar ones. |caracal| gets this **without adding any mechanism**, by extending the
signal |boom| already gates fences on.

All three are decoded ``is_unique`` (``decode.scala:157,166-167``; ``FENCE`` and ``SFENCE_VMA`` are
also ``flush_on_commit``), and an ``is_unique`` uop cannot **dispatch** until the ROB is empty *and*
``fencei_rdy`` (``core.scala:739-740``). So |caracal| simply folds the vector path into that signal:

.. code-block:: scala

   // lsu.scala:439, extended
   io.core.fencei_rdy := !stq_nonempty && io.dmem.ordered && vec_lsu_empty

``vec_lsu_empty`` asserts when **all** vector LSU state is empty: the four address queues
(``ld_SSI_ADDR_Q``, ``st_SSI_ADDR_Q``, ``ld_US_ADDR_Q``, ``st_US_ADDR_Q``), both store data queues
(``st_SSI_DATA_Q``, ``st_US_DATA_Q``), and the Load Coalescing Buffer (:ref:`load-coalesce`).
In-flight D$ responses remain covered by the ``io.dmem.ordered`` term in the same expression. The
conservative definition costs nothing: the wait happens at **dispatch** with the ROB already empty,
so every older vector memory op has committed and drains unconditionally — the deadlock in the
danger note below came from waiting at the ROB *head*, not from the breadth of the condition.

Ordering is therefore established **before the fence ever enters the machine**, and liveness is
structural rather than a sizing argument: at dispatch the ROB is empty, so every older vector store has
already **committed** and will drain unconditionally, and nothing younger exists yet because dispatch is
in program order.

.. danger::

   **The previous head-side handshake deadlocks.** Earlier drafts had the ROB raise ``fence_pending``
   when a fence reaches the head, with the scalar and vector LSUs driving ``drain_done`` when their
   queues are empty, and described it as draining "*all* in-flight memory work, not just pre-fence
   work … unconditionally correct." That coarseness *is* the deadlock: a **younger** vector store
   (dispatched after the fence) fills ``st_SSI_*_Q``, and those entries can only be freed at
   commit-drain — which cannot happen, because the fence is at the ROB head. The fence cannot retire
   because the queues aren't empty. Neither side moves.

.. note::

   Folding ``vec_lsu_empty`` into ``fencei_rdy`` benefits **every** ``is_unique`` instruction, not just
   fences — ``vsetvl`` and the explicit vector-CSR accesses (``csrw vstart``, ``csrr vl``) now also wait
   for in-flight vector memory to settle before dispatching. That is required anyway for the vector-CSR
   serialization in :ref:`vector-csr-ownership`, and it cannot deadlock for the same reason as above:
   the wait happens at dispatch, with the ROB empty.

.. _vec-squash:

Squashing the in-flight vector LSU
----------------------------------

Unlike ``IQ_V_ALU``, ``IQ_V_LOAD``/``IQ_V_STORE`` issue **speculatively** — they do not gate on the PNR
— so a vector memory ``OP.v`` can be mid-drain when a branch mispredicts. |boom| rolls ``ldq_tail`` /
``stq_tail`` back to the branch's ``ldq_idx`` / ``stq_idx`` and kills entries via
``IsKilledByBranch``, which covers the **single** LDQ/STQ entry per ``OP.v`` and nothing else. The
element-level state needs its own answer:

- **Element queues** (``ld_SSI_ADDR_Q``, ``ld_US_ADDR_Q``, ``st_SSI_*_Q``, ``st_US_*_Q``) — because
  capacity is reserved in **program order** at dispatch (:ref:`vec-queue-reservation`), each queue's
  occupied region is program-ordered, so a mispredict **rolls the tail back to the branch's reservation
  index**, exactly as |boom| rolls ``stq_tail``. Killed entries vanish with no per-entry compare, and
  the reservations are released in the same event. This is the second reason loads are reserved even
  though they don't need it for deadlock freedom.
- **Stage-1 vAGEN** — the in-progress crack is killed on ``brupdate``/``exception`` like any pipeline
  stage.
- **Load Coalescing Buffer** — assembly entries are invalidated by owning ``ldq_idx``; a rolled-back
  ``ldq_idx`` is dead, so a late element response has nowhere to land and is dropped, the same shape as
  |boom| dropping a response for an invalidated LDQ entry.

.. danger::

   **Drain-and-discard is not an option here**, even though it is exactly what the CII does
   (:ref:`cii-flush`). On a mispredict, rename returns the killed load's ``pvdest`` group to the free
   list through ``br_alloc_lists``, and those PRNs are **reallocated to a different instruction within
   a few cycles**. A late LCB write would then land a full ``VLEN`` register into a live, unrelated
   destination — silently corrupting a correctly-executing instruction. The CII can drain harmlessly
   only because its *tags* are not recycled during the drain; vector PRNs are.

   Per-entry ``br_mask`` + ``IsKilledByBranch`` on every element-queue and LCB entry is the alternative
   to pointer rollback. It preserves free-running load streaming and full MLP, but it puts
   ``maxBrCount`` bits and kill logic on ~1000 entries — a large area cost for the common case where
   nothing is squashed. |caracal| chooses pointer rollback; the price is that reservation depth now
   bounds vector-load MLP (:ref:`ssi-queues`).


.. _vec-load-algo:

Vector Loads Algorithm
-----------------------
1. **Dispatch** — LDQ slot reserved in-order by the dispatch stage, together with worst-case capacity
   in ``ld_SSI_ADDR_Q`` / ``ld_US_ADDR_Q`` (:ref:`vec-queue-reservation`) — needed here for
   squashability rather than deadlock freedom (:ref:`vec-squash`).
2. **Issue / AGen** — when the load issues out of the issue queue it drains addresses from the
   ``ld_SSI_ADDR_Q`` (per element, precomputed by stage 1 vAGEN) or, for unit-stride, runs the
   stage 2 Packer over the single ``ld_US_ADDR_Q`` ``nOP.v`` to generate element addresses
   just-in-time.
3. **Fire** — for each element access the arbiter (:ref:`dcache-arbiter`) grants a D$ lane; the fire
   does TLB + D$ + LCAM together (``will_fire_load_agen_exec``). US loads do a single range-overlap
   LCAM check; SSI loads search per element.
4. **Coalesce / Writeback** — element responses land in the Load Coalescing Buffer
   (:ref:`load-coalesce`); when a destination PRN is fully assembled the LCB issues one VRF write and
   one completion. ``elem_done`` advances; a fault latches ``fault_elem`` (:ref:`elem-progress`).

Identical to scalar **loads** other than draining addresses from the dedicated vector queues,
coalescing in the LCB, and per-element progress tracking.

.. _vec-store-algo:

Vector Stores Algorithm
-----------------------
1. **Dispatch** — STQ slot reserved in-order by the dispatch stage, **together with worst-case capacity
   in ``st_SSI_ADDR_Q`` / ``st_SSI_DATA_Q``** (:ref:`vec-queue-reservation`); the store does not
   dispatch until both reservations are available. The surplus is released at execute once ``VL`` is
   read from the VL RF.
2. **Execute (translate + order-check)** — drains addresses from ``st_SSI_ADDR_Q`` /
   ``st_US_ADDR_Q`` and does **TLB + LCAM** only. **All active element addresses are translated
   pre-commit** so that any page/access fault is detected and reported precisely **before** the store
   commits (the trap itself sets ``vstart = 0`` and restarts — :ref:`elem-progress`); the translated
   addresses are **retained** in the address queue until commit-drain (a store cannot free them
   mid-instruction), which is why the capacity had to be reserved in program order at step 1.
   LCAM searches the LDQ for ordering violations (US: one range check; SSI: per element).
3. **Commit** — when the ROB retires the store its committed flag is set; only then is it eligible to
   drain to the ``stq_execute_queue``.
4. **Fire** — the ``stq_execute_queue`` drains and the actual D$ writes occur, one element per
   granted lane via the arbiter. Store data is taken from ``st_US_DATA_Q`` (whole-``vPRN``, sliced by
   the Packer) or ``st_SSI_DATA_Q`` (per element). The store data was **read from the source ``vPRN``
   at DGEN (execute time)** and has lived in the data queue ever since, so the source ``vPRN`` is
   **not pinned** — it frees normally with the rest of the stale group at commit, since its value is
   already captured in the data queue.

Identical to scalar stores other than draining address and data from the dedicated vector queues and
the pre-commit translation of the whole active element range.


.. _mem-subsystem:

Memory SubSystem
----------------

The memory subsystem remains unchanged from BOOMv4 — the D$ Interface Arbiter (:ref:`dcache-arbiter`)
sits *outside* the cache and presents the same ``dmem.req`` interface.

.. _vector-bw-ceiling:

Vector memory bandwidth ceiling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Because the cache and its request interface are unchanged, **vector memory bandwidth is bounded by
the scalar D$ port width, not by ``VLEN``.** Each ``dmem.req`` lane carries at most one element
(``≤ ELEN`` = 64 bits) per cycle, and there are ``lsuWidth`` lanes — **1 on Medium, 2 on
Large/Mega**. The consequences are first-order for vector performance and must be understood:

- A single ``VLEN = 256`` destination register is **≥ 4 D$ beats** (``VLEN/ELEN``), even for the
  densest unit-stride load — so peak vector load/store throughput is **64 bits/cycle on Medium,
  128 bits/cycle on Mega**, regardless of ``LMUL``.
- **SSI** (strided / indexed / segmented) accesses drain **one element per granted lane**, and that
  lane is shared with scalar memory ops through the priority round-robin arbiter
  (:ref:`dcache-arbiter`). A pathological scatter/gather is therefore element-serial.
- **Unit-stride** is the optimized case (the Packer coalesces contiguous bytes up to the lane width
  and the LCAM does one range check), but it is still capped at the same ``lsuWidth × ELEN``
  bandwidth — the Packer reduces *address-generation* and *disambiguation* cost, not cache-port
  width.

This is a deliberate area/complexity trade-off: |caracal| reuses the scalar cache port rather than
building a ``VLEN``-wide vector cache interface. For memory-bound vector kernels it is the dominant
performance limiter, so ``LargeBoomV4Config`` / ``MegaBoomV4Config`` (dual-port L1 D$, two grants
per cycle) is **required, not merely recommended,** for acceptable vector throughput. A wider /
line-granular vector cache port is explicitly out of scope here and would be the highest-leverage
follow-on if vector memory bandwidth proves limiting.
