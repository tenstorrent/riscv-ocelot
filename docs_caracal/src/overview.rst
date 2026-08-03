.. _caracal-overview:

Overview
========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


This document provides an overview of the |caracal| micro-architecture, its relationship to |boom| and the Chipyard ecosystem, and details on its implementation of the RISC-V ISA and Vector Extension (RVV).

.. note::

   Unless otherwise specifically mentioned the reader should assume that |caracal| does not modify the existing |boom| micro-architecture.

.. figure:: ../figures/caracal.png
   :align: center

   Overview of Caracal Microarchitecture.


.. _boom-relationship:

Relationship to BOOMv4 and Chipyard
------------------------------------

|caracal| is a fork of BOOMv4 [5/20/2026] with a clean-slate re-architecture to support RVV 1.0 and OOO vector load/store. The scalar pipeline is modified from BOOMv4, with the **decode/rename** and back-end stages extended to support vector instructions and state. The fetch front-end and branch predictor are **unchanged** — see the table below and :ref:`caracal-pipeline`. |caracal| remains compatible with the Chipyard ecosystem, allowing it to be used as a drop-in core for Chipyard SoC designs and simulations.


.. figure:: ../figures/boom_overlay.png
   :align: center

   Block diagram of Caracal Microarchitecture extended from BOOM.

The following table shows which modules have been modified to support RVV1.0 instructions and OOO vector load/store support, with in-order CII issued vector arithmetic operations.

.. list-table::
   :header-rows: 1
   :widths: 1 3

   * - Subsystem
     - |caracal| status (unchanged / extended / new)
   * - Front-end (fetch, BPU)
     - Unchanged
   * - Decode / Rename
     - Extended
   * - Integer execution
     - Extended (``usingRVV``): the integer ALU EU also executes ``vsetvli``/``vsetvl``, computing
       VL (and VTYPE for ``vsetvl``) and writing the VL RF + VL wakeup network. Such a uop has
       **two destinations in two rename spaces** (``pdst`` in the INT RF, ``pvl`` in the VL RF),
       marked by the new ``is_vl_producer`` bit — see :ref:`the dual-destination rule
       <vset-dual-dest>`. Bit-identical to |boom| when ``usingRVV`` is off.
   * - FP execution
     - Unchanged
   * - Vector (RVV) execution
     - New
   * - Load/Store Unit
     - Extended
   * - Memory system
     - Unchanged
   * - Vector architectural CSR state
     - Delegated. ``vtype`` (incl. ``vill``), ``vl``, ``vstart``, ``vxrm``, ``vxsat``, ``vcsr``,
       ``vlenb`` and ``mstatus.VS`` (dirty tracking + the ``VS=Off`` illegal-instruction gate)
       come from rocket-chip's ``CSRFile`` under ``usingVector`` (``csr.io.vector``).
       |caracal| owns only the speculative VCFG ``vtype`` mirror and the VL register file —
       see :ref:`vector-csr-ownership`.

.. _caracal-pipeline:

The Caracal Pipeline
--------------------

.. figure:: ../figures/caracal_pipeline.png
   :align: center

   The Caracal pipeline, note greyed area are identical to BOOMv4.

|caracal| keeps |boom| v4's out-of-order pipeline intact and threads
a vector path through it. Everything is gated on the ``usingRVV`` core parameter
(``common/parameters.scala``), so with vectors disabled the core elaborates as
stock |boom| v4. **The stage list below is authoritative**: each entry follows an instruction
from fetch to commit and opens with that stage's status — *Identical*, *Extended*, *New*, or
*Reuses … unchanged* — so which stages are bit-identical to |boom| v4 is checkable from this text
alone. The greying in the figure above illustrates the same partition and is not a separate claim.
Each description then calls out only where the vector path attaches.

Fetch (F0–F5)
   Identical to |boom| v4. The six-stage front-end (next-PC select, I$ access,
   I$ response, fetch-buffer enqueue, BPD redirect, deliver-to-core) and the
   TAGE-based branch predictor are untouched — vector instructions are ordinary
   32-bit (or RVC-expanded) words and are not distinguished until decode. No
   vector state is read or written here.

Decode
   Extended. The scalar decoders are unchanged; a parallel vector decoder
   (``VDecode``) recognizes RVV opcodes (``v_legal`` gate in ``exu/decode.scala``) and
   populates the new vector ``MicroOp`` fields — ``is_vec``, the logical vector
   specifiers (``lvs1/lvs2/lvs3/lvd/lvm``), and the ``vconfig`` snapshot
   (``vtype``) in ``common/micro-op.scala``. The **Vector Config
   Unit (VCFG)** keeps the running ``vtype`` mirror so younger vector uop's
   snapshot it (and derive ``EMUL``) without a CSR read; it does **not** mirror ``vl``.
   VL is renamed into the VL register file and delivered to consumers via ``pvl`` (read at
   execute). ``vsetivli`` is **front-end only** (both vtype and AVL are immediate): the VCFG
   computes VL at decode and the value is written to ``VL_RF[pvl]`` in the **rename** cycle,
   where ``pvl`` is allocated — there is no decode-stage VL-RF write and no back-end EU.
   ``vsetvli``/``vsetvl`` execute on an integer ALU EU. ``vsetvl``
   (both vtype and VL from registers) is marked **both** ``is_unique`` **and**
   ``flush_on_commit``: ``is_unique`` alone does *not* order the vtype mirror against younger
   decode (see :ref:`vector-rvv-decode`), so the mirror is recovered from the committed VCFG
   shadow on the post-commit flush. Decode also sets the ``iq_type`` routing bits for the
   new vector issue queues (``IQ_V_LOAD/IQ_V_STORE/IQ_V_ALU``, widened in
   ``common/consts.scala``).

Rename
   Extended. The scalar integer/FP map tables, free lists, and busy tables are
   the |boom| v4 design **unchanged** — integer rename is not modified. |caracal| adds a
   third register class — ``RT_VEC`` (``common/consts.scala``) — and a vector physical
   register file (``numVecPhysRegs``, default 96) renamed alongside INT and FP, mapping the
   logical ``lvs*/lvd/lvm`` specifiers to physical ``pvs*/pvdest/pvm``. ``VL`` is renamed into
   **its own register file** (default 64) with its own map table, free list, busy table, wakeup
   network, and commit logic (see the VL Rename section); the VL value is not held in the integer
   RF. ``VTYPE`` is **not** renamed — it rides the VCFG ``vtype`` mirror / ``VConfig`` snapshot.
   Vector ops allocate ROB entries through the same path as scalar ops.

Dispatch (Rename2)
   Reuses |boom| v4's dispatcher unchanged. Dispatch routes purely on the
   ``iq_type`` bits set in decode, so vector uop's fall into the vector issue queues
   with no vector-specific dispatch logic.

Issue
   Extended. The age-ordered issue-unit logic is reused; |caracal| instantiates
   additional vector issue queues (``IQ_V_LOAD``, ``IQ_V_STORE``, ``IQ_V_ALU``)
   alongside the scalar ``IQ_MEM/IQ_UNQ/IQ_ALU/IQ_FP``. All three vector queues
   are **age-ordered collapsing**; ``IQ_V_ALU`` adds a **per-entry past-PNR eligibility
   gate** — like RoCC it issues only instructions that are individually non-speculative. That
   removes **branch**-kill from the CII (the PNR cannot sweep past an unresolved branch) but not
   flush recovery: past-PNR is *not* a commit guarantee, so the CII needs the drain-on-flush
   contract in :ref:`cii-flush`. Only the queue set and operand-readiness tracking are widened to
   cover vector physical registers.

Register Read
   Extended. Integer and FP read out of their existing regfiles unchanged.
   Vector-ALU and vector-load/store uop's read their operands (``pvs1/pvs2/pvs3``,
   mask ``pvm``) from the vector register file through dedicated read ports. Every
   vector EU also reads ``VL`` from the **VL register file** (``pvl``), takes ``vtype`` from
   the uop's ``VConfig`` snapshot, reads ``vstart``/``vxrm`` from the CSR file, and has a read
   port into the **integer register file and integer bypass network** for its remaining scalar
   feeders — base address, stride, and the ``.vx`` operand. The scalar bypass network is
   otherwise unchanged.

Execute
   New (vector) / mostly-unchanged (scalar). The mul/div, branch unit, and FPU are the
   |boom| v4 functional units, untouched. The integer ALU EU is **extended (gated by
   ``usingRVV``)** to execute ``vsetvli``/``vsetvl``: it computes VL (and VTYPE for ``vsetvl``)
   from its integer source(s) and, for these uops, its writeback targets the **VL RF** and drives
   the VL wakeup network (and updates the VCFG ``vtype`` mirror for ``vsetvl``). With ``usingRVV``
   off the ALU is bit-identical to |boom|. Vector arithmetic is **not**
   executed on a BOOM EU: vector-ALU uop's are issued in program order over the
   **Tenstorrent Custom Instruction Interface (tt_CII)** to an in-order vector
   unit (VPU) that owns the vector lanes/ALU datapath. The vector **register file lives in
   BOOM** (banked, see the Register Files section); the coprocessor reads and writes it
   through the CII as a client — it does not own it. This is the core architectural
   divergence — BOOM remains the OoO scalar host, scheduler, and VRF owner, while RVV
   arithmetic executes on the attached in-order coprocessor.

Memory (LSU)
   Extended. The scalar LDQ/STQ, store-to-load forwarding, and 3-stage D$ pipeline
   (``s0/s1/s2``) are reused. |caracal| adds **out-of-order vector load/store**: vector
   memory uop's (``IQ_V_LOAD/IQ_V_STORE``) carry the statically-decoded access descriptor
   (``v_eew``, ``mop``, ``nf``, unit-stride/strided/indexed/segment flags from ``VLSDecode``)
   and generate element/segment accesses that are buffered in dedicated vector address/data
   queues. Those accesses share the D$ with scalar memory ops via a **priority round-robin
   arbiter** (scalar-priority floor + anti-starvation, also gating the LCAM and TLB ports),
   and are ordered against scalar loads/stores by **bidirectional cross-queue disambiguation**
   — vector element addresses are routed through the LCAM in both directions. A Load Coalescing
   Buffer assembles per-element responses into one VRF write per destination register. Element
   faults trap with ``vstart = 0`` and restart the whole instruction — legal because the
   destination group is a *fresh* physical group that is never installed architecturally, so no
   partial result is visible (see :ref:`elem-progress`). Element-queue capacity is reserved in
   **program order at dispatch**, which is what makes the queues both deadlock-free and
   squashable by pointer rollback. See the Loadstore chapter.

Writeback
   Extended. Scalar results write back to the INT/FP regfiles exactly as in
   |boom| v4. Vector results (``RT_VEC`` destinations) write back to the vector
   register file; the writeback arbitration is the |boom| v4 design widened for
   the additional vector write ports.

Commit
   Reuses |boom| v4's ROB unchanged in structure. The ROB entry's ``dst_rtype`` was
   widened to 3 bits (``exu/rob.scala``; ``RT_VEC = 4`` fits in 3 bits alongside the
   existing ``RT_FIX/RT_FLT/RT_X/RT_ZERO``) to encode ``RT_VEC``, so vector uop's commit
   in program order through the same head-pointer/exception machinery as scalar
   ops. Precise vector state (``vtype``/``vl``) is recovered on redirect/exception via
   the per-uop ``vconfig`` snapshot rather than a separate rollback path.

   A ROB-head flush (exception, ``MINI_EXCEPTION_MEM_ORDERING``, CSR replay, ERET) **can**
   squash work the CII has already accepted, because the PNR is not a commit guarantee — see
   :ref:`cii-flush`. Branch mispredicts cannot, because ``is_br``/``is_jalr`` set
   ``starts_unsafe`` and the PNR never sweeps past an unresolved branch.

.. =====================================================================
