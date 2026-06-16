Issue
=====

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


Dispatch Stage
--------------

In |boom|, dispatch is the pipeline stage that sits between rename and the issue queues —
it is the last in-order stage before instructions go out-of-order. This stage routes the
instruction into the appropriate IQ and reserves the LDQ/STQ slot in program order, which
only the in-order dispatch stage can do. The dispatch stage is essentially unchanged from
|boom| except modified to support vector op-codes and routing into the new ``IQ_V_*`` queues.

A special case exists for shared vector instructions.

CII Shared Instruction Scheduling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Shared instructions (currently only segmented load/store, marked ``is_shared`` by the
Decoder) require more than one execution resource. In the single-stage scheme this is
handled at **dispatch time**: when a segmented load or store is renamed by the vector mapper,
it is dispatched to BOTH the CII IQ/coprocessor and its own Load/Store path. A shared vector
*arithmetic* instruction does not apply — the coprocessor fully manages its own
resources — so shared handling applies only to vector load/store.

The two halves occupy two issue slots but share **one ROB entry**, which stays busy until both
halves report via the ROB completion counter (see the ROB Completion-tracking section). The halves
rendezvous on a **TVRB tag** — a temp identifier from a namespace separate from the main vector PRNs
(see the Temporary Register Buffer section) — carried in the ``OP.v``'s ``vsrc`` field, so the temp
can never alias an architectural vector register.

The wakeup conditions differ for the two halves:

Segmented Load
^^^^^^^^^^^^^^

1. A segmented load is a shared instruction where the load unit must use the ``vsrc`` PRN as
   the destination *temporary* register, broadcast and buffered only within the bypass
   network (TVRB).
2. For a segmented load the LSU does **not** update the VPRF.
3. A segmented load is selected for issue to the LSU when all of its vector source operands
   are available.


Segmented Store
^^^^^^^^^^^^^^^

1. A segmented store is a shared instruction where the store unit must use the ``vsrc`` PRN
   as the source *temporary* register, broadcast and buffered only within the bypass network
   (TVRB).
2. For a segmented store the LSU does **not** read from the VPRF.
3. A segmented store is selected for issue to the LSU when the bypass network wakes the
   instruction in the vector store IQ and broadcasts that the ``vsrc`` is available in the
   Temporary Register Buffer.

The Issue/Scheduling Stage
--------------------------

Like |boom|, |caracal| uses split issue queues. The scalar queues — ``IQ_MEM``,
``IQ_UNQ``, ``IQ_ALU``, ``IQ_FP`` — are **unchanged** from |boom|. |caracal| adds three
vector queues: ``IQ_V_LOAD``, ``IQ_V_STORE``, and ``IQ_V_ALU``. Each vector IQ may hold
any datatype.

All queues — scalar and vector — issue in a **single** age-ordered scheduling stage.
|caracal| does **not** add a second issue stage. A vector ``OP.v`` occupies exactly one
issue slot in one queue and is granted **once**, when all of its operands (scalar feeders
and vector registers) are ready. This reuses |boom|'s age-ordered collapsing Issue Queue
and its priority-encoder select unchanged.


Wakeup Networks
~~~~~~~~~~~~~~~

|boom| wires each issue queue only to the wakeup network of its own register space (the
integer queues listen to integer writebacks, the FP queue to FP writebacks). Because each
register space has its own physical-register numbering and its own wakeup ports, operand
matches across spaces never collide. |caracal| preserves this partitioning:

- The scalar queues (``IQ_MEM``/``IQ_UNQ``/``IQ_ALU``/``IQ_FP``) are **bit-identical** to
  |boom| — they connect only to the existing integer/FP wakeup networks.
- **Only the three ``IQ_V_*`` queues connect to the vector wakeup network** (driven by
  vector register writebacks). Each also connects to whichever scalar networks supply its
  scalar feeders:

  - The **integer** network — for the base address, stride, ``.vx`` scalar operand, and the
    VL physical register (all GPR-sourced). Needed by **all three** vector queues.
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
  existing ``prs1``/``prs2`` operand slots) plus the VL physical register ``pvl`` — all matched
  against the **integer** wakeup network. Vector floating-point ALU ops (``.vf`` forms and
  ``vfmv.*.f``) additionally source one scalar **FP** register, matched against the **FP**
  wakeup network; this applies to ``IQ_V_ALU`` only.
- **Vector operands**, matched against the **vector** wakeup network: ``pvs1``/``pvs2``/
  ``pvs3`` and the mask ``pvm`` (V0), each with its own busy bit.

An ``OP.v`` asserts ``request`` only when **all** of its operands — scalar and vector — are
ready:

.. code-block::

   request := slot_valid && !iw_issued && scalar_operands_ready && vector_operands_ready

For vector **stores** the existing mem-slot AGEN/DGEN split is extended: the
data-generation (DGEN) path is gated on the vector store-data operand ``pvs3`` (matched on
the vector network) rather than on ``prs2`` as in the scalar mem slot.

The mask ``pvm`` is read conditionally — its busy bit only participates in ``request`` when
the OP.v is masked (encoded ``vm`` bit clear). Unmasked ops leave ``pvm`` don't-care so a
stale mask preg is never waited on.


VL Broadcast Unit
-----------------

VL must be resolved before a vector load/store can crack into element accesses in the
Vector AGEN stage (see :ref:`vector-agen`). VL is delivered at the issue stage by a new
special unit, the **VL Broadcast Unit (VLBU)**.

An ordinary wakeup only flips a readiness bit. The VLBU is different: it must deliver the VL
**value** into the slot, because the AGEN needs the element count, not just readiness. The
VLBU taps the **integer writeback data lane** and, for every waiting vector slot whose VL
physical register ``pvl`` matches the writeback's ``pdst``, it writes the VL value into the
slot's captured-VL field and clears ``pvl_busy`` in the same cycle.

Three delivery cases cover all of the vset variants:

1. **Statically known VL** (``vsetivli``, and ``vsetvli`` with ``rs1 = x0``): VL is known at
   decode. The slot enters with ``pvl_busy = false`` and its captured VL pre-loaded from the
   ``OP.v`` ``VConfig`` snapshot taken by the Vector Config Unit (see
   :ref:`vector-rvv-decode`).
2. **Register-sourced VL** (``vsetvli`` with ``rs1 != x0``): the producing scalar
   instruction writes the VL value through the normal integer writeback; the VLBU captures
   that value into the slot and clears ``pvl_busy`` the same cycle.
3. **vsetvl** (VTYPE and VL both from registers): the instruction is serialized via
   |boom|'s ``is_unique`` mechanism (see :ref:`vector-rvv-decode`), so VL is resolved before
   any dependent vector ``OP.v`` dispatches; the slot is pre-loaded as in case 1.

Once VL is captured, the ``OP.v`` carries it into the Vector AGEN stage.





Single-Stage Scheduling Key Features
------------------------------------

The single-stage approach with extended vector slots offers the following benefits:

1. **The scalar datapath is untouched.** Only the ``IQ_V_*`` queues are extended and connect
   to the vector wakeup network; the scalar queues are bit-identical to |boom|.
2. **The VL scalar value is resolved at issue** by the VL Broadcast Unit and carried into the
   AGEN stage.
3. **A vector ``OP.v`` is allocated and selected once.** There is no second issue stage, so
   there is no double allocation, no second priority-encoder select, and no cross-queue
   kill/replay to keep consistent.
4. **Enables a detached CII co-processor**, with temporary registers and shared instructions
   dispatched at grant time.
5. **Cracking of vector instructions in the frontend is unnecessary**, made possible by the
   atomic LMUL vector mapper and AGEN-time element cracking.
