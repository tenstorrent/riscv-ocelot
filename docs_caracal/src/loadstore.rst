Loadstore
=========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC

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
element order without another op interleaving at that entry — it does **not** mean all-or-nothing:
a faulting load/store still commits the elements before the fault and records ``vstart`` (see
:ref:`elem-progress`). The element accesses themselves contend for the shared D$ port through the
arbiter in :ref:`dcache-arbiter`, so a vector drain does **not** monopolize the cache.

For ``LargeBoomV4Config`` or ``MegaBoomV4Config`` we take advantage of the dual-port L1 D$
interface and allow 2 memory operations to be issued per cycle; the vector address and data queues
are ``2 x nOP.v`` wide to allow 2 concurrent element accesses.

Queue-naming convention. Vector queues are named ``{ld,st}_{SSI,US}_{ADDR,DATA}_Q`` —
``ld_SSI_ADDR_Q``, ``st_SSI_ADDR_Q``, ``st_SSI_DATA_Q``, ``ld_US_ADDR_Q``, ``st_US_ADDR_Q``,
``st_US_DATA_Q``.


Strided, Segmented, Indexed Queue
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This class of vector (SSI) queues holds effective addresses calculated for strided, indexed, and
segmented load stores. There exists a ``ld_SSI_ADDR_Q`` and a ``st_SSI_ADDR_Q``.

Because the SSI queues hold essentially element-wise memory addresses, their default parameterized
size is quite big at 512 entries. The ``ld_SSI_ADDR_Q`` and ``st_SSI_ADDR_Q`` receive calculated
effective addresses from the stage 1 vAGENs.

Sizing is set by the **store** requirement, which is stricter than the load one:

- **Stores must hold their full active address set pre-commit.** A store cannot write memory until
  it commits, and every element's fault must be detected *before* commit (precise exceptions). So a
  store's translated addresses are **retained in ``st_SSI_ADDR_Q`` from execute until commit-drain**
  — they cannot be streamed/freed mid-instruction. The queue is therefore sized to hold the
  **worst-case single-instruction element count**: with ``VLEN=256`` the maximum is ~256
  element-accesses (``SEW=8`` × ``LMUL=8`` = 256; segmented ``NF*EMUL ≤ 8`` also caps at 256), so the
  512-entry default fits one max store with headroom. Additional in-flight stores **back-pressure
  the store vAGEN** when the queue is full rather than overflowing it.
- **Loads may stream.** A load completes out of the Load Coalescing Buffer (:ref:`load-coalesce`)
  and does not gate on commit, so the ``ld_SSI_ADDR_Q`` may be drained in waves and a load whose
  active element count exceeds the queue depth is streamed through it.


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


Store Data Queue
~~~~~~~~~~~~~~~~

Instead of holding the store data in the STQ entry, which would not be feasible for vector data, a
separate ``st_SSI_DATA_Q`` and ``st_US_DATA_Q`` buffer the store data. The ``st_SSI_DATA_Q`` entry
width is one element (``ELEN``, 64 bits); the ``st_US_DATA_Q`` entry width is a full ``VLEN`` (256
bits), so a unit-stride store reads its entire source ``vPRN`` in one access and the Packer slices
out per-element data at drain.


.. _load-coalesce:

Load Coalescing Buffer
~~~~~~~~~~~~~~~~~~~~~~

A vector load returns at most one element (``≤ ELEN``) per D$ response, and those responses may
arrive **out of order** across MSHRs, while a destination physical register is a full ``VLEN``.
Writing the VRF once per element would need an impractically wide, byte-masked write port and would
break the one-write-per-PRN assumption the ROB completion counter and the vector Busy Table rely
on.

To bridge this, |caracal| adds a **Load Coalescing Buffer (LCB)** in front of the VRF write port:

1. The LCB holds a small number of in-flight ``VLEN``-wide assembly entries, one per destination
   PRN currently being filled (parameterizable; default a few entries per in-flight vector load).
2. Each returning element is written into its byte offset within the assembly entry for its
   destination PRN, using the element index carried on the ``nOP.v``.
3. When **all active bytes** of a destination PRN are present (tracked by a per-entry byte-valid
   bitmap, bounded by the active-element mask), the LCB issues a **single** VRF write on ``W0`` and
   signals one completion — which decrements the ROB ``EMUL`` completion counter and clears that
   PRN's busy bit (waking the group-readiness aggregate in the Busy Table).
4. Inactive (masked-off or tail) byte lanes are filled per ``vta``/``vma`` policy before the write,
   so the single VRF write leaves no stale bytes.

The LCB is what makes "one writeback per destination PRN" true even though the cache returns data
element-by-element and out of order.


.. _elem-progress:

Per-Element Progress Tracking and Precise Exceptions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A vector LDQ/STQ entry owns up to ``VLMAX`` element accesses but is a single entry, so it carries an
**element cursor** in addition to the scalar fields:

- ``elem_next`` — the index of the next element to drain.
- ``elem_done`` — count (or bitmap) of elements that have completed.
- ``fault_elem`` — the index of the **oldest** faulting element, latched on the first fault.

Because the issue stage grants in age order and the V-LSU pipeline consumes element accesses **in
program/element order**, no element ``> k`` has been written to the cache (stores) or committed to
the LCB→VRF (loads) when element ``k`` faults. On a fault the entry stops advancing, the oldest
``fault_elem`` is written to architectural ``vstart`` on trap, and the instruction resumes
mid-stream per RVV 1.0. This gives precise element-level exceptions without any partial-write
roll-back.

Fault-only-first (``vleff.v``)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The fault-only-first unit-stride load has special semantics: a fault on **element 0** is reported
as a normal precise trap (``vstart = 0``), but a fault on **element ``i > 0``** must **not** trap —
instead VL is trimmed to ``i`` and the instruction completes with the elements it did load.

``vleff`` is decoded as a **unique instruction** (``is_unique``, like ``vsetvl``) so it is the only
in-flight op when it resolves and it can update VL without racing younger uOPs. It is also a
**VL producer**: on completion it writes the final element count to its integer VL destination PRN
— the full VL if no fault occurred, or ``i`` if element ``i > 0`` faulted — exactly as a vset writes
VL. That value is **broadcast to the VLBU** and updates the VCFG mirror / architectural ``vl`` CSR,
so the current-VL-PRN tracker and any dependent vector uOPs pick up the (possibly trimmed) VL through
the same path as a vset. Only the element-0 fault raises ``rob_exception``; the ``i > 0`` case clears
no architectural state beyond trimming VL.


.. _mem-order:

Memory Ordering and Disambiguation
----------------------------------

Vector load/store element addresses live in the SSI/US address queues, **not** in the scalar STQ.
The scalar LCAM (which searches the LDQ/STQ entries) therefore does not see them by default, so
|caracal| explicitly routes vector addresses through the disambiguation machinery so RVWMO ordering
between scalar and vector memory ops is maintained in **both** directions:

- **ST→LD ordering (vector store vs. any load).** Each vector store address presented for execute
  is searched against the LDQ — both scalar LDQ entries and in-flight vector loads — exactly as a
  scalar store addr-gen drives the LCAM. A match on a younger, already-executed load sets that
  load's ``order_fail`` and replays it.
- **LD→ST forwarding (any load vs. vector store).** A load address is searched against both the
  scalar STQ and the vector store address queues. The vector store's data, when forwarded, is read
  from the ``st_*_DATA_Q`` (or the pinned source ``vPRN``), not from a scalar STQ data field.
- **Scalar LD/ST vs. scalar ST/LD.** Unchanged from |boom|.

Granularity of the search differs by access class:

- **Unit-stride (US): one range check, not per element.** A US access is the contiguous byte range
  ``[base, base + VL*EEW)``. The LCAM comparison for US loads/stores is a **range-overlap** test of
  that whole range against each queue entry's address, performed once when the US entry executes —
  rather than expanding to per-element searches. This is both cheaper and sufficient: any overlap
  anywhere in the range triggers the ordering/forwarding action. For forwarding, an overlap that
  only partially covers the load is handled like any partial-forward case in |boom| — replay if the
  covering store data cannot fully satisfy the load.
- **Strided / Indexed / Segmented (SSI): per element.** Scattered addresses cannot be range-folded,
  so SSI accesses search the LCAM per element ``nOP.v`` as they drain. These per-element searches
  contend for the shared LCAM through the same arbiter that gates the D$ port
  (:ref:`dcache-arbiter`), so they cannot starve scalar disambiguation.

Memory-dependency speculation (a load issuing past a store whose address is not yet known) reuses
|boom|'s existing predictor and ordering-violation replay path, now extended to fire on the
cross-queue matches above.


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
  least one grant every *N* cycles. A long vector drain therefore yields the lane periodically so
  scalar traffic makes progress, and a steady scalar stream still lets the vector drain advance.
- **Dual lane (Large/Mega, ``lsuWidth=2``).** The arbiter grants up to 2 of the pending requests per
  cycle and is work-conserving: an all-scalar workload uses both lanes for scalar, an all-vector
  workload uses both for vector, and a mix splits them under the priority-floor + round-robin policy.

The same policy gates the **shared LCAM** and **TLB** ports, since a vector element stream consumes
those resources at the same rate it consumes D$ bandwidth. All three (D$ lane, LCAM port, TLB port)
are released back to scalar on the round-robin boundary so no single vector ``OP.v`` can monopolize
disambiguation or translation.


Fences
------

``fence`` (RVWMO ``fence rw,rw`` and friends) and ``fence.i`` must order vector memory ops alongside
scalar ones. |caracal| takes the simplest correct approach: a fence at the head of the ROB **does not
retire until both LSUs are fully drained** — the scalar LDQ/STQ are empty *and* the vector load/store
path is empty (every vector store drained to the cache and every vector load completed), not merely
committed. A store is only globally visible once drained, so "committed" is not sufficient; the fence
waits for the actual cache writes.

Implementation: the ROB raises a ``fence_pending`` signal when a fence reaches the head; the scalar
LSU and the vector LSU each drive a ``drain_done`` back when their queues are empty, and the ROB
retires the fence only when both are asserted. This is coarser than a per-address or acquire/release
scheme (it drains *all* in-flight memory work, not just pre-fence work), but it is unconditionally
correct.


Vector Loads Algorithm
-----------------------
1. **Dispatch** — LDQ slot reserved in-order by the dispatch stage.
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

Vector Stores Algorithm
-----------------------
1. **Dispatch** — STQ slot reserved in-order by the dispatch stage.
2. **Execute (translate + order-check)** — drains addresses from ``st_SSI_ADDR_Q`` /
   ``st_US_ADDR_Q`` and does **TLB + LCAM** only. **All active element addresses are translated
   pre-commit** so that any page/access fault is detected and reported precisely (with
   ``fault_elem``) **before** the store commits; the translated addresses are **retained** in the
   address queue until commit-drain (a store cannot free them mid-instruction), so the queue is sized
   to the worst-case single-store element count and additional in-flight stores back-pressure the
   store vAGEN. LCAM searches the LDQ for ordering violations (US: one range check; SSI: per element).
3. **Commit** — when the ROB retires the store its committed flag is set; only then is it eligible to
   drain to the ``stq_execute_queue``.
4. **Fire** — the ``stq_execute_queue`` drains and the actual D$ writes occur, one element per
   granted lane via the arbiter. Store data is taken from ``st_US_DATA_Q`` (whole-``vPRN``, sliced by
   the Packer) or ``st_SSI_DATA_Q`` (per element); the source ``vPRN`` is pinned (``store_pending``,
   see the Free List) until the last element drains.

Identical to scalar stores other than draining address and data from the dedicated vector queues and
the pre-commit translation of the whole active element range.


Memory SubSystem
----------------

The memory subsystem remains unchanged from BOOMv4 — the D$ Interface Arbiter (:ref:`dcache-arbiter`)
sits *outside* the cache and presents the same ``dmem.req`` interface. We recommend
``LargeBoomV4Config`` or ``MegaBoomV4Config`` to enable the dual-port L1 D$, which lets the arbiter
grant two requests per cycle for best scalar+vector throughput.
