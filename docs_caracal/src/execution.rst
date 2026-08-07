Execution
=========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


.. _execution-pipelines:

The Execution Pipelines
-----------------------

|caracal| does not modify any |boom| execution pipeline, scalar execution is untouched.
The execution of vector instructions is split into 3 streams:

1. Vector Arithmetic, Reduction, Permutation
2. Vector Loads
3. Vector Stores
   
Vector instruction OP.v's may execute out-of-order relative to program order, but must always execute atomically
and in element order. Shared instructions require more than 1 EU to activate and hand off intermediate results
through a temp vector group (``pvtmp``) in the VRF — one half writes it, the other reads it.

**Atomically means architecturally atomic**, not that an ``OP.v``'s element accesses are
indivisible. No partial result of an ``OP.v`` is ever architecturally visible. An ``OP.v`` whose
element stream faults mid-group satisfies this even though accesses below the fault point have
already fired and their responses land in the LCB: ``pvdest`` is a *fresh* physical group that a
non-committing instruction never installs in ``com_map_table``, so the trap discards those responses
and restarts the whole instruction with ``vstart = 0`` (see :ref:`elem-progress`).


.. figure:: ../figures/execution_stage.png
   :align: center

   Overview of Vector Mapper Stage.



.. _vector-ls-agen:

Vector LS AGEN stage
--------------------

This stages sits between the ``IQ_V_LOAD``/``IQ_V_STORE`` issue queues and the Unified Load Store Unit.
Its primary purpose is to perform cracking of L/S OP.v instructions into nOP.v which access memory at the element
granularity. It also cracks OP.v instructions based on EMUL, so nOP.v also tracks to which PRN and offset within the PRN
the memory access should read or write to.

The vector LS AGEN stage outputs these nOP.v bundles which may be very large to dedicated queues with in the LSU.

.. _vector-agen:

Vector AGEN
~~~~~~~~~~~

Once vector load store instructions are issued from the ``IQ_V_LOAD``/``IQ_V_STORE`` queues, they enter the vector AGEN stage.
The vector AGEN stage contains the load vAGEN, store vAGEN/vDGEN. Vector OP.v's issued by 
the CII IQ are directly forwarded to the co-processor via the CII interface.

.. warning:: **Superseded (Caracal v2).** This section still describes the inherited bobtail
   Load/Store Packer, Skipper and Walker units. v2 **deletes all six** (2110 lines, whose
   direction x class duplication had already produced divergent mask support between the load and
   store Packers, forcing masked stores onto the slower Skipper as a workaround) and replaces them
   with three units cut **by pipeline position, not by direction**:

   * ``VecElemAgen`` — fill side, SSI (strided / indexed / segmented): one element address per
     *active* element. Absorbs both the Skipper and the Walker, which differ only in where the
     address comes from, not in what they emit.
   * ``VecRangeAgen`` — fill side, unit-stride: exactly ONE range entry per instruction.
   * ``VecBeatExpander`` — **drain** side: coalesces contiguous elements into D$-width accesses
     just-in-time. The previous attempt put the Packer on the *fill* side, so nothing coalesced
     late and every access was one 64-bit beat per element.

   Direction is a **parameter**, so each agen is instantiated twice and the two directions never
   arbitrate — the load-priority mux is what silently dropped store grants. Every *behavioural*
   obligation below survives and is allocated to whichever unit now performs it; only the two
   requirements that mandate the superseded module **identity** are retired. The skip is
   additionally **class-conditional** — strided/segmented only, never indexed — because
   ``VecIdxGen``'s per-element handshake pulses once per element *including masked-off ones*, so
   a power-of-2 skip and a one-element-at-a-time indexed walk cannot both hold on that path.

We reuse the Load/StorePacker, Load/StoreSkipper, Load/StoreWalker AGEN units from bobtail.

**The generator is selected by access class, not by direction.** The three rules below hold
identically on the load and store paths — the subsections say "accesses" for that reason. The only
asymmetries between the two paths are that they read the mask and index through different VRF ports
(``R0`` index / ``R1`` mask on the load path, ``R4`` on the store path — see :ref:`vrf-ports`), and
that the store path additionally runs ``st_vdgen`` alongside ``st_vagen_1`` (:ref:`vector-dgen`).
Selection itself is:

- **indexed**, masked or not → **Walker**;
- **non-indexed and not unit-stride** (strided, segmented), masked or not → **Skipper**;
- **unit-stride** → **Packer**, in stage 2.

**Both AGEN stages must read the vector mask and apply it during address generation.** The mask is
read through the unit's VRF mask read port — ``R1`` on the load path, ``R4`` on the store path (see
:ref:`vrf-ports`) — and both stages **suppress address generation for masked-off elements**: a
masked-off
element produces no ``nOP.v``, and therefore no D$ access, no TLB translation and no LCAM
search. The resulting cursor is the **single owner** of which elements survive: both the
``st_vdgen`` on the store path (see :ref:`vector-dgen`) and the Load Coalescing Buffer on the load
path (see :ref:`load-coalesce`) consume that cursor rather than evaluating the mask again.

**Stage 1 performs the mask read for every access class, including unit-stride.** The stage 2 Packer
does **not** read the VRF: a US ``OP.v`` already passes through stage 1, where it is encoded into a
single ``nOP.v``, so ``ld_vAGEN_1`` reads ``v0`` there and the mask travels with that ``nOP.v`` to
the Packer. This is what keeps the mask port to **one reader** — were stage 2 to read it directly, a
US ``OP.v`` in stage 2 and an SSI ``OP.v`` in stage 1 would be two concurrent readers of ``R1``,
which :ref:`vrf-ports` cannot serve because ports are statically partitioned and never arbitrated.
The cost is carrying up to ``VLMAX`` mask bits on the US ``nOP.v`` — wide, but one bundle per
``OP.v``.

The mask read happens **once per ``OP.v``** and is latched: a single ``VLEN``-wide read of ``v0``
yields the mask bits for every element of the access. Index members, by contrast, are read as the
walk advances. That cadence is why the store path's single ``R4`` suffices for both jobs.

.. important::

   Applying the mask in AGEN rather than downstream is what makes masked vector memory
   operations architecturally correct in both directions. A masked-off **store** element must
   leave memory unmodified, and the source ``vPRN``'s inactive lanes hold coprocessor data
   rather than the memory's prior contents, so an access that reached the D$ would clobber
   bytes the instruction never wrote. A masked-off **load** element must not raise a memory
   exception, so an access that reached the TLB could fault on an address the instruction never
   architecturally touches. Suppressing the access itself satisfies both.

Packer
^^^^^^
Handles the fastest, densest case: contiguous/unit-stride loads with no 
indexing. It packs multiple segments and multiple elements into each DMEM-width packet, 
advancing a single EEW_CTR by whichever constraint (VL, vreg boundary, DMEM boundary, 
segment, or mask) is hit first. Supports masked loads: the bobtail unit fetches masked-off
lanes and merely flags them, so the reused Packer is extended to **suppress** those accesses
instead, per the mask rule above. Does not support indexed loads.

Skipper
^^^^^^^
Handles **non-indexed, non-unit-stride accesses** — strided and segmented — whether or not a mask
is present, and where masked-off elements should be skipped rather than fetched.
It packs whole segments per element but uses the mask bits
(via a priority encoder) to jump past runs of disabled elements in power-of-2 strides,
emitting "fake" packets for skipped regions. **Masking is an optimization here, not a selection
criterion**: with no mask active the Skipper skips nothing and simply walks the elements at
``base + i*stride``. Use this instead of the packer whenever the access is not unit-stride, and
instead of the walker whenever there is no index.

Walker
^^^^^^
Handles **indexed accesses**
by walking through elements one at a time, taking a per-element byte offset and mask bit
from the index interface. Segments are still packed per element, but each element gets
its own computed address and direction; to avoid stall/wait states it won't release the
final segment or start until the next index arrives. Use this for any indexed or
indexed-masked access — it does not support non-indexed accesses, which the skipper handles.
The Walker is **not** the fallback for a non-indexed access: its per-element offset comes from the
index interface, and a strided access has no index vector to drive it.

``ld_vAGEN_1`` performs both vector reads that feed these generators: the **index vector** on ``R0``
and the **mask** on ``R1`` (:ref:`vrf-ports` is the authority for the port numbers). The index read
drives the Walker's index interface; the mask read drives the Skipper's priority encoder and the
mask rule above. On the store path ``st_vagen_1`` reads both through ``R4``.

|caracal| splits the vAGEN into two stages. The first stage is after the issue stage (the
``IQ_V_LOAD``/``IQ_V_STORE`` queues) where load and store OP.v's are issued to ld_vAGEN_1 and
st_vagen_1, respectively.
The stage 1 vAGENs will only contain the Skipper and Walker generators. This choice is taken to
optimize the common unit-stride load/store case, and reduces the amount of pressure on the load and store
queues. Thus in stage 1 AGEN only strided, indexed, and segmented load stores will have effective addresses
calculated and expanded into nOP.v bundles to be sent to the load store unit. Unit strided OP.v load stores
will be encoded in a single nOP.v containing the effective base address, effective stride, and is_unit_stride 
flag set high. The stage 2 AGEN's Packer will see that this nOP.v is a unit stride operation and generate 
all of the effective nOP.v *just-in-time* at the load/store queues in the Unified Load Store Unit.


.. _vector-dgen:

Vector DGEN
~~~~~~~~~~~

st_vdgen for vector stores data generation works along side st_vagen_1. The st_vdgen reads store data
from the vector register file only: RVV store data is always the vector operand ``vs3``.

.. note:: **Corrected (decision D4).** An earlier revision of this paragraph said st_vdgen may
   also read "the FP/INT register file for scalar source operands, for instance ``vfmul.vf``".
   That is wrong on both counts. ``vfmul.vf`` is a vector-scalar *floating-point multiply* —
   arithmetic, dispatched to the CII — and is not a store at all. **No RVV store form takes an
   FP scalar operand**: store data is always ``vs3``, and the scalar operands of a store are
   ``rs1`` (base address) and ``rs2`` (stride), both integer, and both read by
   st_vagen_1 rather than by st_vdgen. The store-side FP read port has therefore been deleted.



For strided, indexed, and segmented (SSI) load/stores, st_vdgen may read an entire vPRN and alongside
the calculated effective address from the st_vagen_1 be dispatched to LSU SSI_Q's. SSI_Q's are ELEN
wide so the st_vdgen reads and buffers the entire vPRN read value, and generates a address and 
64 bit data bundle that will be issued to the LSU SSI address and data queues.

These nOP.v bundles must be dispatched to the LSU store queues in-order and atomically. Each nOP.v
must be processed in program and memory order, to maintain precise exception tracking via 
VSTART. As data within vPRN's is already packed each effective address is paired with the data of
the element that ``st_vagen_1`` selected. The ``st_vdgen`` **must not re-derive which elements
survive masking**: it consumes the **same mask-derived element cursor** as ``st_vagen_1``, so the
address and data bundles stay paired element-for-element and a masked-off element is skipped in
both streams. With no mask active the cursor advances consecutively, one bundle per consecutive
element of the vPRN.

For unit-stride load/stores, the st_vdgen may read an entire vPRN and pass the entire VLEN
read data alongside the effective base address from st_vagen_1 to be dispatched to LSU.
The LSU will have seperate dedicated queues for the unit-stride load/store nOP.v's, and
generate per-element memeory accesses sequentially.



.. _vector-execution:

CII Vector Co-processor
-----------------------

The vector Coprocessor (VPU) will be issued instructions from the CII IQ (``IQ_V_ALU``). These
instructions include all vector arithmetic, reduction, permutation, and shared instructions such as
segmented LS. The VPU will reuse the Baby RISCV Vector Unit from bobtail, with modifications to
implement the CII interface. Improvements from the previous generation include a data transpose unit
to support segmented LS.

Like |boom|'s **RoCC** interface, the CII IQ issues to the VPU **only instructions that are
non-speculative** — an entry is granted only once it is **past the PNR** (older than
``rob_pnr_idx``) **and** its operands are ready. The gate is **per entry**, and ``IQ_V_ALU`` is
age-ordered rather than head-only (see the Issue/Scheduling Stage). Squashed
``IQ_V_ALU`` entries are dropped from the queue before they ever issue.

The VPU therefore needs no **branch**-kill path: the PNR cannot sweep past an unresolved branch, so
nothing it accepts is ever younger than one. It *does* need a **drain-on-flush** path, because
past-PNR is not a commit guarantee — a ROB-head flush (exception,
``MINI_EXCEPTION_MEM_ORDERING``, CSR replay, ERET) can squash work the CII has already accepted. See
:ref:`cii-flush` for the contract.

Performance: in-order vector arithmetic is a deliberate trade-off
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|caracal| is **out-of-order for vector memory and in-order for vector arithmetic.** Vector
load/store ``OP.v``'s issue out-of-order from ``IQ_V_LOAD``/``IQ_V_STORE`` and disambiguate against
scalar memory in both directions; vector **arithmetic / reduction / permutation** ``OP.v``'s are
issued from ``IQ_V_ALU`` **in age order** over the CII to the in-order VPU.

- **What it costs.** A long-latency vector arithmetic ``OP.v`` (e.g. a vector multiply or a
  reduction) blocks *younger* vector arithmetic behind it, even when the younger op is independent —
  there is no out-of-order vector-ALU window. Compute-bound vector kernels see the VPU's in-order
  latency directly, and vector-arithmetic ILP is bounded by the VPU's own internal pipelining, not
  by BOOM's OoO scheduler.
- **What it buys.** BOOM remains the sole OoO scheduler and VRF owner; the VPU is a simple in-order
  client behind the CII pull interface. This removes vector-side rename/replay/wakeup machinery, lets
  vector arithmetic complete with a single **group-done** (see :ref:`group-done-wb`), and keeps the
  vector register-read and bypass network tractable. Memory-bound and memory-latency-bound vector
  code — the common case for the targeted workloads — still benefits from OoO vector loads/stores
  overlapping with scalar and with each other.

In short: the OoO win is spent where it pays off most (the memory pipeline, given the
:ref:`bandwidth ceiling <vector-bw-ceiling>`), and the vector ALU is kept in-order to keep the
coprocessor attach simple. Workloads that are vector-arithmetic-throughput-bound rather than
memory-bound are the ones this trade-off disadvantages.


Tenstorrent Custom Instruction Interface (tt_CII)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Design goals:

1. A **PULL** model of execution: the coprocessor requests the register operands it needs,
   rather than the host pushing a fixed operand set.
2. **In-order** execution of dispatched instructions (see the trade-off above).
3. Usage of intermediate temp vector groups (``pvtmp`` in the VRF) for shared-instruction handoff.
4. Extensible interface for custom instructions.
5. The coprocessor drives all register-file reads and writes **indirectly**, through the host,
   using abstract operand slots and member offsets — never register numbers (see
   :ref:`cii-prn-arn`).

Channel overview
^^^^^^^^^^^^^^^^

The interface is four independent, **credit-metered, unidirectional** channels (default depth
16 credits each). The relay between host and coprocessor is pure latency pipes with no buffering;
each channel's **receiver** owns the FIFO and returns one credit per pop. Parameters (Caracal
sizing): ``VLEN = 256``, ``XLEN = ELEN = 64``, ``MAX_MEMBERS = 8`` (EMUL ≤ m8), 1 issue lane,
4 source-request / source-data lanes, 1 writeback lane, 16 in-flight ``tag``\ s.

.. list-table::
   :header-rows: 1
   :widths: 22 14 64

   * - Channel
     - Direction
     - Payload
   * - **Issue**
     - host → cop
     - ``{tag, instr}`` — the offloaded op (see :ref:`cii-issue-packet`).
   * - **Source-Request**
     - cop → host
     - ``{tag, op_id, op_offset}`` — a *pull* request for one operand beat.
   * - **Source-Data**
     - host → cop
     - ``{data[VLEN]}`` — one operand beat, returned **in request order**.
   * - **Writeback**
     - cop → host
     - ``{tag, wb_data[VLEN], wb_dst_offset, wb_wr_en, wb_status}`` — one result beat.

The ``tag`` is an **opaque 4-bit handle** (parameter ``ciiTagBits``, 16 in flight) allocated by the
host at issue and echoed by the
coprocessor on every request/writeback beat. It indexes the host's per-tag **side-table**, whose
membership is defined once in :ref:`cii-issue` and not restated here. The coprocessor treats the
tag as an identifier only; it never inspects its contents. The ``killed`` bit carries the
drain-on-flush state of :ref:`cii-flush`, and ``stale_pvdest_grp`` is what the ``STALE_VD`` slot
resolves to (:ref:`old-vd`).

What the host provides
^^^^^^^^^^^^^^^^^^^^^^

**Issue channel** — for each granted ``IQ_V_ALU`` head the host emits an issue packet (below) and
records a side-table entry keyed by the freshly-allocated ``tag``.

**Source-Data channel** — the host answers each Source-Request beat with a ``VLEN`` data beat, **in
the same order the requests arrived** — a single global order across all four lanes, defined in
:ref:`cii-operands`. It resolves the request's ``op_id`` + ``op_offset`` to a source and reads it:

- a **vector** slot (``VS1``/``VS2``/``VS3``/``VM``/``STALE_VD``) → a registered read of the host VRF
  (CII read ports ``R5``-``R8``, see :ref:`vrf-ports`) at the **physical** register of that member.
  The host serves whichever slot is requested and applies no instruction-dependent reinterpretation
  (:ref:`cii-operands`);
- the **scalar** slot (``SCALAR``) → the ``.vx``/``.vf`` integer/FP scalar **value**, captured from
  the INT/FP RF at issue and served from the side-table (no VRF read).

The host applies **no** ``vta``/``vma`` masking on the operand read — the coprocessor pulls the
``v0`` mask (``VM``) and the old-destination group (``STALE_VD``) itself and applies tail/mask
internally.

What the coprocessor provides
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Source-Request channel** — the coprocessor decodes the issued ``instr`` and, for every operand
member it needs, pulls ``{tag, op_id, op_offset}``:

- ``op_id`` (``cii_caracal_srcid_e``) names the *abstract* source slot, not a register:
  ``NONE=0`` (reserved), ``VS1=1``, ``VS2=2``, ``VS3=3`` (the **explicitly encoded** third source,
  nothing else), ``VM=4`` (the ``v0`` mask), ``SCALAR=5`` (the ``.vx``/``.vf`` scalar),
  ``STALE_VD=6`` (the old-``vd`` group, for merging — see :ref:`cii-operands`);
- ``op_offset`` is the **LMUL member index** (0..MAX_MEMBERS-1) within the register group. For a
  register group of ``NM = EMUL`` members the coprocessor walks ``op_offset = 0..NM-1``; widening
  doubles the destination/relevant-source member count.

**Writeback channel** — the coprocessor returns one result beat per destination member:

- ``wb_data`` — the fully-formed ``VLEN`` result (``vta``/``vma`` already applied by the
  coprocessor; the host writes it **verbatim**). A scalar result (``vmv.x.s``/``vfmv.f.s``/…)
  occupies the low ``XLEN`` bits.
- ``wb_dst_offset`` — the destination **member index** (again a group offset, not a register).
- ``wb_wr_en`` — per-beat write enable.
- ``wb_status`` = ``{last, dst_kind, vxsat, fflags}``: ``last`` marks the final beat of the ``tag``
  (→ completion); ``dst_kind`` routes the write to ``VEC`` (VRF), ``INT`` RF, or ``FP`` RF;
  ``vxsat`` is the sticky fixed-point saturation bit; ``fflags`` are the ``{NV,DZ,OF,UF,NX}`` FP
  exception flags. There is no expected-count — the host frees the ``tag`` and completes the ROB
  entry on the beat with ``last`` set.

.. _cii-prn-arn:

Physical vs. architectural registers: the coprocessor sees neither
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Rename is fully resolved on the host before an op crosses the CII, and the coprocessor never
addresses a register file by number — architectural (ARN) or physical (PRN).** The division is:

- The **issue packet** carries the raw 32-bit RVV instruction word, whose ``vs1``/``vs2``/``vd``
  fields are **architectural** specifiers. The coprocessor decodes these only to derive its operand
  *set* (which ``op_id``\ s to pull) and the op semantics — **not** to address any register file.
- Every **Source-Request** and **Writeback** beat identifies operands by the **abstract**
  ``{op_id, op_offset}`` / ``{tag, wb_dst_offset}`` pair — a slot and a member index, with no
  register number at all.
- The **host** owns the entire ARN→PRN mapping. At issue it snapshots the renamed **physical**
  groups (``pvs1/2/3_grp``, ``pvm``, ``pvdest_grp`` — vectors of PRNs, one per LMUL member) into the
  side-table. On a pull it resolves ``op_id + op_offset → pvs*_grp(op_offset)`` (a **PRN**) and
  reads the physical VRF; on a writeback it resolves ``tag + wb_dst_offset →
  pvdest_grp(wb_dst_offset)`` (a **PRN**) and writes the physical VRF.
- Scalar ``.vx``/``.vf`` sources are delivered **by value**, not by address: the host reads the INT/FP
  RF at the renamed physical scalar reg (``prs1``) at grant and stores the *value* in the side-table.
  Scalar destinations are completed to the renamed physical ``pdst`` recorded at issue.

This keeps the coprocessor free of rename, wakeup, and replay logic (consistent with the past-PNR,
non-speculative issue model): it manipulates opaque tags, slots, and member offsets, while the host
alone translates them to physical registers of the machine.

.. _cii-issue-packet:

Instruction issue packet
^^^^^^^^^^^^^^^^^^^^^^^^^

Because the interface has no separate CSR channel and a 32-bit RVV instruction does not encode
``vtype``, the host folds the full dynamic vector context into the issue packet
(``cii_caracal_instr_t``):

.. list-table::
   :header-rows: 1
   :widths: 20 12 68

   * - Field
     - Width
     - Meaning / source
   * - ``insn``
     - 32 b
     - Raw RVV instruction word (architectural ``vs1``/``vs2``/``vd`` fields) — from ``uop.debug_inst``.
   * - ``vtype``
     - 8 b
     - ``{vsew, vlmul, vta, vma}`` snapshot — from ``uop.vconfig``.
   * - ``vl``
     - 9 b
     - Active vector length — from the VL-RF read of ``uop.pvl``.
   * - ``vstart``
     - 9 b
     - Start element — read from the ``vstart`` CSR at issue and **honoured by the VPU** (elements
       below it are left untouched). It is **not** forced to ``0``: software may set ``vstart``
       directly with ``csrw vstart``, and RVV requires elements below it to be unmodified — visible
       whenever ``vd`` overlaps a source. Hardware never *produces* a non-zero ``vstart``, because
       vector LS faults trap with ``vstart = 0`` and restart (see :ref:`elem-progress`).
   * - ``vxrm``
     - 2 b
     - Fixed-point rounding mode — from ``csr.io.vector``.
   * - ``frm``
     - 3 b
     - FP rounding mode — from ``fcsr.frm``.

The packet is prefixed with the 4-bit ``tag``. A per-source ``src_reuse`` hint field exists in the
type but is **ignored in M2** (the host drives 0 and the coprocessor re-pulls every operand);
honoring it to elide redundant pulls is a future optimization.

Because this packet carries the full dynamic context per instruction, the CII needs **no
cross-instruction configuration state and no serializing configuration write** — which is what
permits ``IQ_V_ALU`` to issue age-ordered rather than in program order (see :doc:`issue`).