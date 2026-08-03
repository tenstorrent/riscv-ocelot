Frontend
========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


.. _frontend-stages:

The Frontend Stages
-------------------

The frontend includes 2 stages the Instruction Fetch and Decoder. Nominally the IF should consume
5 cycles and the Decoder 1 cycle, with support for super-scalar issue.

.. _frontend-fetch:

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
     - ``1/8, 1/4, 1/2, 1, 2, 4, 8`` — subject to ``VLMAX ≥ 1``. A ``vtype`` whose
       ``SEW``/``LMUL`` combination gives ``VLMAX = VLEN*LMUL/SEW < 1`` is **reserved** and sets
       ``vill`` (e.g. ``LMUL=1/8, SEW=64`` at ``VLEN=256`` → ``VLMAX = 0.5``). ``SEW > ELEN`` and
       the reserved ``vlmul=3'b100`` encoding likewise set ``vill``.
   * - Vector regfile depth
     - 96 (``numVecPhysRegisters``) — 32 always held by the committed map table, so
       ``(96 − 32) / 8 = 8`` ``LMUL=8`` groups in flight

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
mirrors only ``vtype`` — it does **not** mirror ``vl``: VL is
renamed into the VL register file and delivered via ``pvl`` (see VL delivery below), so there is no
decode-time VL value at all. The VCFG is **not** the architectural CSR store; the architectural
vector CSRs — ``vstart``, ``vxrm``, ``vxsat``, ``vcsr``, ``vl``, ``vtype``, and read-only
``vlenb`` — live in the normal CSR file and are updated precisely at commit (``vstart``/``vxrm``/
``vxsat`` are read from the CSR file at execute, not snapshotted in the uOP).
 
**Where each vset executes.** Only ``vsetivli`` is **front-end only**: both VTYPE and AVL are
immediate, so the VCFG resolves it at decode in a single cycle — updating the ``vtype`` mirror and
computing ``VL = min(uimm, VLMAX)`` — with **no back-end issue slot or EU**.

.. important::

   The computed VL is written to ``VL_RF[pvl]`` in the **rename** cycle, not at decode: ``pvl`` is
   allocated by the VL mapper at rename, so there is no VL-RF index to write to at decode. The
   write sets **no** busy bit — a ``vsetivli``'s ``pvl`` is *born ready*, so a dependent vector
   uOP in the same or the next dispatch group never waits on it. Because the uop has no writeback,
   its ROB entry is dispatched **non-busy** and commits as soon as it reaches the head (where it
   writes the architectural ``vtype``/``vl`` and the committed VCFG shadow).

**VL-RF write ports are statically partitioned, not arbitrated**, so this rename-cycle write never
contends with a writeback. There is one write port per producer class: the **rename-side** port,
replicated per rename lane because one dispatch bundle may hold several ``vsetivli``; the
**integer-ALU writeback** port for ``vsetvli``/``vsetvl``; and the **LSU writeback** port for
``vleff``'s trimmed VL. This is the same discipline the VRF uses (:ref:`vrf-ports`) and it is nearly
free here — the VL RF is 64 entries of about 9 bits — so no producer can ever stall on a VL-RF port,
which matters because the ALU and LSU writeback paths have no back-pressure line.

``vsetvli`` (register AVL) and ``vsetvl`` (register VTYPE and AVL) **execute on an integer ALU EU**,
because their VL (and ``vsetvl``'s VTYPE) depends on a runtime register value. They are dispatched to
a scalar integer issue queue and woken by ``rs1`` (and ``rs2`` for ``vsetvl``) on the **integer**
wakeup network like any integer op; the ALU reads the source(s) from the integer RF/bypass, computes
``VL = min(rs1, VLMAX)`` (and VTYPE for ``vsetvl``), and on writeback **targets the VL RF** and drives
the **VL wakeup network** (``pvl``). When ``rd != x0`` the ALU additionally writes ``rd`` in the
integer RF as normal. No dedicated VCFG wakeup network is added — a register-sourced vset consumes
the integer network and produces on the VL network. This integer-ALU extension (VL/VTYPE compute +
VL-RF writeback) is **gated by ``usingRVV``**: with the switch off, the ALU is bit-identical to
|boom| v4.

.. _vset-dual-dest:

A register-sourced ``vset`` is **not** an ordinary integer-ALU uop: it has **two destinations in two
independent rename spaces** — ``pdst`` in the integer RF and ``pvl`` in the VL RF. What makes this
cheap is that ``rd`` receives the *new* ``vl``, so both destinations take the **same value** off one
result bus. What it costs is one new field, because ``dst_rtype`` is single-valued and cannot encode
the combination (with ``rd == x0`` it is ``RT_ZERO``, yet the VL RF is still written, so VL-producing
is not inferable from ``dst_rtype`` at all):

- a **new ``is_vl_producer`` bit** on the ``MicroOp``, orthogonal to ``dst_rtype``;
- writeback fans out to the integer RF when ``dst_rtype === RT_FIX`` **and** to the VL RF when
  ``is_vl_producer``;
- rename sets **both** busy bits — integer busy for ``pdst``, VL busy for ``pvl`` — and both wakeup
  networks fire;
- commit runs **both** free paths — the stale integer ``pdst`` through the normal path, and the
  outgoing committed VL pointer through the VL path.

``dst_rtype`` therefore keeps its unmodified integer meaning end to end, which is what "integer
rename is not modified" actually requires.

.. note::

   A ``vset``'s **own** ``vconfig`` field holds its **new** ``vtype``, because that is what the ROB
   writes to the architectural ``vtype`` CSR and the committed VCFG shadow at commit. The per-lane
   prefix select is therefore **self-inclusive for ``vset``\ s and self-exclusive for consumers**.
   (The ``[vsetvli, vadd, vsetivli, vadd]`` example below only illustrates the consumer case.)

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

.. warning::

   **``is_unique`` alone does not solve this, and earlier drafts of this chapter claimed it did.**
   ``is_unique`` gates only the unique uop's **own dispatch** (``core.scala:739-740``):

   .. code-block:: scala

      val wait_for_empty_pipeline = (0 until coreWidth).map(w =>
        (dis_uops(w).is_unique || !custom_csrs.enableOOO) &&
        (!rob.io.empty || !io.lsu.fencei_rdy || dis_prior_slot_valid(w)))

   It stalls ``vsetvl`` until all **older** instructions have retired. It says nothing about
   **younger** ones, which dispatch on the very next cycle while ``vsetvl`` is still in the ALU. A
   younger vector uOP would therefore decode against a **stale** ``vtype`` mirror, derive the wrong
   ``EMUL``, and cause the mapper to allocate the **wrong number of vector PRNs** — silent group
   mis-sizing, with no misprediction involved, so the ``vcfg_snapshots(br_tag)`` recovery below
   never fires.

|caracal| therefore marks ``vsetvl`` **both ``is_unique`` and ``flush_on_commit``**, following
|boom|'s own convention for "younger code must observe my effect" (``core.scala:420``: *"All
flush_on_commit instructions are also is_unique"*; ``FENCE`` and ``SFENCE_VMA`` set both, and real
CSR writes pick it up at ``decode.scala:533`` via ``io.csr_decode.write_flush``).

With ``flush_on_commit`` set, the execute-time mirror write **disappears entirely** and no new
recovery path is needed:

1. The ALU EU **does not write the speculative ``vtype`` mirror**.
2. The ROB updates the **committed VCFG shadow** when ``vsetvl`` commits, as it already does for
   every ``vset``.
3. ``flush_on_commit`` refetches everything younger, and that flush reloads the speculative mirror
   from the committed shadow — which is already row 2 of the recovery table in
   :ref:`vcfg-recovery` ("Exception / pipeline flush → committed VCFG shadow → 1 cycle").

This does result in poorer performance for this instruction (a drain *and* a refetch), but this is
acceptable as this ``vsetvl`` instruction type is not common.

.. _vector-csr-explicit:

Explicit vector-CSR accesses
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Every **explicit** vector-CSR access — any ``csrr``/``csrw``/``csrrw``-family instruction targeting
``vstart``, ``vxrm``, ``vxsat``, ``vcsr``, ``vl``, ``vtype``, or ``vlenb`` — is decoded as
``is_unique``, and every such **write** is additionally ``flush_on_commit``. Both are required, for
the reason given in the warning above: ``is_unique`` makes a *read* return the committed value
(nothing older is in flight), but only ``flush_on_commit`` makes a *write* take effect before a
younger uOP observes it, since younger uops dispatch one cycle behind the unique one. In practice
rocket already supplies this for CSRs it knows about (``decode.scala:533``:
``flush_on_commit := cs.flush_on_commit || (csr_en && !csr_ren && io.csr_decode.write_flush)``), so
the requirement is that the vector CSRs assert ``io.csr_decode.write_flush`` in the ``CSRFile`` —
see :ref:`vector-csr-ownership`.

These accesses are rare, so the serialization cost is acceptable. Implicit updates are unaffected
and use their normal paths: ``vtype``/``vl`` from ``vset``, ``vstart`` set on a trap, and the sticky
``vxsat`` accumulated by the coprocessor — all committed precisely.

.. _vector-csr-ownership:

Vector architectural state: ownership boundary
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|caracal| does **not** implement vector CSR state itself. It is delegated to rocket-chip's
``CSRFile`` under ``usingVector``, reached through ``csr.io.vector`` — the same interface
:ref:`vector-execution` already uses to source ``vxrm``. The split is:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Owner
     - State
   * - **rocket ``CSRFile``**
     - ``vtype`` including **``vill``**; ``vl``; ``vstart``; ``vxrm``/``vxsat``/``vcsr``; read-only
       ``vlenb``; and **``mstatus.VS``/``sstatus.VS``** — dirty tracking on any write to vector
       state, plus the **``VS=Off`` illegal-instruction gate**. Also asserts
       ``io.csr_decode.write_flush`` for vector-CSR writes.
   * - **|caracal|**
     - The **speculative VCFG ``vtype`` mirror** (decode-stage, branch-snapshotted) and the
       **VL register file** (:ref:`vl-vtype-rename`). Neither is architectural state; both are
       recovered from the committed shadow / committed VL pointer.

Three consequences are mandatory and easy to miss:

- **``vill`` must poison the mirror.** Because ``EMUL`` is derived from the VCFG mirror *at decode*,
  a ``vill``-setting ``vset`` must mark the mirror invalid so every younger vtype-dependent vector
  uOP raises illegal-instruction **at decode**, rather than allocating a garbage-sized PRN group.
  For ``vsetivli``/``vsetvli`` the vtype is immediate, so ``vill`` is known at decode and poisons the
  mirror directly; for ``vsetvl`` it is only known at execute, but ``flush_on_commit`` refetches
  younger uops against the committed shadow, so no extra mechanism is needed.

  The poison gates **vtype-dependent** uOPs only. A **whole-register** move, load or store
  (``vmv<n>r.v``, ``vl<n>r.v``, ``vs<n>r.v``) decodes and executes normally while the mirror is
  poisoned, because it takes its ``EMUL`` from the ``NREG`` field of its own encoding rather than
  from the mirror — so the garbage-sized-PRN-group hazard the poison exists to prevent cannot arise
  for it. This is also what keeps vector state restorable after a ``vill``, which is what those
  instructions are for.
- **``mstatus.VS`` is not optional.** Without the ``VS=Off`` trap and Dirty tracking the vector
  context cannot be saved or restored, so the core cannot context-switch.
- **``vstart`` is cleared on completion.** RVV requires every vector instruction that completes
  without trapping to reset ``vstart`` to 0. This is load-bearing here, because the CII issue packet
  carries the real ``vstart`` (see :ref:`cii-issue-packet`); if it were not cleared, the *next*
  vector instruction would silently skip its prefix.

.. _vcfg-recovery:

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
   ``maxBrCount``-deep array (≈9 bits/entry — ``vtype`` only — small). Those nine bits are
   ``vlmul[2:0]``, ``vsew[2:0]``, ``vta``, ``vma`` and **``vill``**: the poison flag is a *field of
   the mirrored ``vtype``*, not a bit beside it, so it is snapshotted and restored with the rest and
   needs no recovery path of its own.

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

The mirror lives at decode, a stage ahead of ``br_tag`` allocation (which happens at the
single-cycle rename, see :ref:`rename-stage`). The per-``br_tag`` snapshot is therefore sourced from
the **branch's carried ``vconfig``** — the nearest-preceding-``vset`` value from the per-lane prefix
select — captured on the same ``ren_br_tags`` event as the RMT snapshots, so it reflects all
``vset*`` updates older than the branch in program order without any delayed-``br_tag`` path.

.. _vl-delivery:

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
  untouched and younger uOPs keep the existing ``pvl``. **Reserved case:** using this form with a
  ``vtype`` whose ``SEW``/``LMUL`` ratio changes ``VLMAX`` is a reserved encoding per RVV 1.0 §6.2;
  |caracal| sets ``vill`` (and therefore poisons the VCFG mirror, see
  :ref:`vector-csr-ownership`).
- Otherwise (``rd == x0`` with a VL change) a VL PRN is still allocated to hold VL.

Every producing case sets the ``is_vl_producer`` bit described in
:ref:`the dual-destination rule <vset-dual-dest>`; the
non-producing case above leaves it clear.

In every producing case VL is written to the VL RF and read back via ``pvl`` — there is no
statically-known-VL path that bypasses the VL RF.

In all producing cases, when ``rd != x0`` the ``vset`` also writes ``rd`` as an ordinary integer
destination (normal, unmodified integer rename); vector consumers read VL only from the VL RF.
