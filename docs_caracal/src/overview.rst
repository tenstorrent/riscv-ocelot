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


Relationship to BOOMv4 and Chipyard
------------------------------------

|caracal| is a fork of BOOMv4 [5/20/2026] with a clean-slate re-architecture to support RVV 1.0 and OOO vector load/store. The scalar pipeline is largely unchanged from BOOMv4, with the front-end and back-end stages extended to support vector instructions and state. |caracal| remains compatible with the Chipyard ecosystem, allowing it to be used as a drop-in core for Chipyard SoC designs and simulations.

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
     - Unchanged
   * - FP execution
     - Unchanged
   * - Vector (RVV) execution
     - New
   * - Load/Store Unit
     - Extended
   * - Memory system
     - Unchanged

The Caracal Pipeline
--------------------

.. figure:: ../figures/caracal_pipeline.png
   :align: center

   The Caracal pipeline, note greyed area are identical to BOOMv4.

|caracal| keeps |boom| v4's 13-stage out-of-order pipeline intact and threads
a vector path through it. Everything is gated on the ``usingRVV`` core parameter
(``common/parameters.scala``), so with vectors disabled the core elaborates as
stock |boom| v4. The stages below follow an instruction from fetch to commit;
greyed stages in the figure are bit-identical to |boom| v4, and each
description calls out only where the vector path attaches.

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
   (``vstart/vl/vtype`` mirror) in ``common/micro-op.scala``. ``vset{i}vl{i}`` is
   decoded as a **scalar** ALU uop (not ``is_vec``) so it updates ``vtype``/``vl``
   in-line on the existing integer datapath; younger vector uop's carry the
   resulting ``vconfig`` so they need no separate CSR read. Decode also sets the
   ``iq_type`` routing bits for the new vector issue queues
   (``IQ_V_LOAD/IQ_V_STORE/IQ_V_ALU``, widened in ``common/consts.scala``).

Rename
   Extended. The scalar integer/FP map tables, free lists, and busy tables are
   the |boom| v4 design unchanged. |caracal| adds a third register class — ``RT_VEC``
   (``common/consts.scala``) — and a vector physical register file
   (``numVecPhysRegs``, default 128) renamed alongside INT and FP, mapping the
   logical ``lvs*/lvd/lvm`` specifiers to physical ``pvs*/pvdest/pvm``. Vector ops
   allocate ROB entries through the same path as scalar ops.

Dispatch (Rename2)
   Reuses |boom| v4's dispatcher unchanged. Dispatch routes purely on the
   ``iq_type`` bits set in decode, so vector uop's fall into the vector issue queues
   with no vector-specific dispatch logic.

Issue
   Extended. The age-ordered issue-unit logic is reused; |caracal| instantiates
   additional vector issue queues (``IQ_V_LOAD``, ``IQ_V_STORE``, ``IQ_V_ALU``)
   alongside the scalar ``IQ_MEM/IQ_UNQ/IQ_ALU/IQ_FP``. The wakeup/select policy is
   the same; only the queue set and operand-readiness tracking are widened to
   cover vector physical registers.

Register Read
   Extended. Integer and FP read out of their existing regfiles unchanged.
   Vector-ALU and vector-load/store uop's read their operands (``pvs1/pvs2/pvs3``,
   mask ``pvm``) from the vector register file through dedicated read ports; the
   bypass network for scalar results is unchanged.

Execute
   New (vector) / unchanged (scalar). The integer ALUs, mul/div, branch unit, and
   FPU are the |boom| v4 functional units, untouched. Vector arithmetic is **not**
   executed on a BOOM EU: vector-ALU uop's are issued in program order over the
   **Tenstorrent Custom Instruction Interface (tt_CII)** to an in-order vector
   unit that owns the vector lanes and vector register file datapath. This is the
   core architectural divergence — BOOM remains the OoO scalar host and scheduler,
   while RVV arithmetic executes on the attached in-order coprocessor.

Memory (LSU)
   Extended. The scalar LDQ/STQ, store-to-load forwarding, and 3-stage D$ pipeline
   (``s0/s1/s2``) are reused. |caracal| adds **out-of-order vector load/store**:
   vector memory uop's (``IQ_V_LOAD/IQ_V_STORE``) carry the statically-decoded access
   descriptor (``v_eew``, ``mop``, ``nf``, unit-stride/strided/indexed/segment flags
   from ``VLSDecode``) and generate element/segment accesses against the same D$
   port, ordered through the existing LSU disambiguation machinery.

Writeback
   Extended. Scalar results write back to the INT/FP regfiles exactly as in
   |boom| v4. Vector results (``RT_VEC`` destinations) write back to the vector
   register file; the writeback arbitration is the |boom| v4 design widened for
   the additional vector write ports.

Commit
   Reuses |boom| v4's ROB unchanged in structure. The ROB entry's ``dst_rtype`` was
   widened to 3 bits (``exu/rob.scala``) to encode ``RT_VEC``, so vector uop's commit
   in program order through the same head-pointer/exception machinery as scalar
   ops. Precise vector state (``vtype``/``vl``) is recovered on redirect/exception via
   the per-uop ``vconfig`` snapshot rather than a separate rollback path.

.. =====================================================================
