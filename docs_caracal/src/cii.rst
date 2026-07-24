Vector Arithmetic and the CII Coprocessor
=========================================

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC

.. _cii:

|caracal| executes vector *arithmetic* on an in-order vector unit attached through the
**Tenstorrent Custom Instruction Interface (TT-CII)** — a loosely-coupled coprocessor
interface. Vector *loads and stores* stay in the unified LSU (see :doc:`loadstore`); only
the arithmetic ``OP.v`` stream is offloaded to the CII.

.. note::

   The authoritative CII protocol spec and RTL live in the source tree at
   ``src/main/sv/v4/tt-cii`` (``docs/specs/protocol/{interface,interface_details,
   hazard_analysis}.adoc`` and ``src/tt_cii*.sv``). This chapter describes only how
   |caracal| *attaches* to the CII — the issue queue, register ports, and completion
   path. Where the two disagree, the tt-cii spec is authoritative.

The interface
-------------

The CII is a loosely-coupled coprocessor reached over **four unidirectional,
credit-metered channels** (each a ``tt_cii_channel`` credit FIFO, default depth 16).
Handshake is credit-based: a sender holds a free-running credit counter and stalls at
zero — there is no ``ready`` line.

.. list-table::
   :header-rows: 1
   :widths: 16 12 44 28

   * - Channel
     - Direction
     - Payload (per lane)
     - Notes
   * - **Issue**
     - host → CII
     - ``{tag[8], instr[32], vtype{sew,lmul,vta,vma}, vl, vxrm, src_reuse_hint[3]}``
     - in program order; ``NUM_INST_ISSUE=1``
   * - **Src-Request**
     - CII → host
     - ``{tag[8], op_id[8], op_offset[8]}``
     - *pull* model; ``NUM_SRC_REQ=2``
   * - **Src-Data**
     - host → CII
     - ``{data[VLEN]}``
     - host answers **in request order**; ``NUM_SRC_DAT_RSP=2``
   * - **Writeback**
     - CII → host
     - ``{tag[8], wb_data[VLEN], wb_dst_offset[8], wb_wr_en, wb_status[6], last}``
     - tagged, may return out-of-order; ``NUM_DST_WB=2``

.. note::

   |caracal| extends two of the generic ``tt_cii_interface.sv`` packet structs: the **Issue**
   packet carries ``vtype``/``vl``/``vxrm`` (the SV has no CSR channel, and a 32-bit RVV
   instruction does not encode vtype), and the **Writeback** packet carries a ``last`` bit
   marking the final beat of a ``tag`` (the generic struct has no end-of-transaction marker).
   Both are coordinated changes shared by the host adapter and the VPU coprocessor.

Operand and result buses are ``VLEN`` wide (256 b default). An ``LMUL`` register group is
**one transaction** identified by its ``tag``; individual group members are selected by
``op_offset`` (operand pull) and ``wb_dst_offset`` (writeback). The CII is variable-latency
and may complete internally out of order — results are correlated back to the issuing
instruction by ``tag``. ``wb_status`` carries the FP condition flags. (The verification
tree references CV-X-IF, but TT-CII is its own four-channel protocol, not a CVXIF variant.)

Host bridge: Chisel ↔ SystemVerilog
-----------------------------------

|caracal|'s core is Chisel; the CII interface, its credit relay, and the VPU are
SystemVerilog. A Chisel ``BlackBox`` can only bind **flat** ``Bits`` ports — not a SV
``interface``/``modport`` port, and not ports typed by ``parameter type`` packed structs
(exactly what ``tt_cii_interface`` and the VPU use). So the two are joined by a thin SV
flatten wrapper: the Chisel host adapter (``VecCiiHost``) drives a **flat** four-channel
bundle to a ``BlackBox``, and the wrapper repacks flat ⇄ structs and hides the interface,
the credit relay, and the VPU coprocessor inside.

.. code-block:: text

   CHISEL (BOOM)                              |  SV (inside the BlackBox)
   valu_iss_unit (IQ_V_ALU)                   |
     .io.iss_uops(0): Valid[MicroOp] ─┐       |
     .io.fu_types(0)  ◄────────────┐  │       |
                      ┌────────────▼──▼────┐  |
                      │     VecCiiHost      │  |  Chisel adapter (wired like VecLSU)
                      │  tag alloc +        │  |
                      │  tag side-table +   │  |
                      │  per-channel credit │  |
                      └─────────┬───────────┘  |
                       flat CiiHostIO bundle    |
                       (iss_*, req_*, dat_*,    |
                        wb_* + *_credit;         |
                        plain UInt fields)       |
                      ┌─────────▼───────────┐    |
                      │   BlackBox TTCii     │───┼─►  tt_cii_host_wrap.sv:
                      └─────────────────────┘   |      • repack flat ⇄ cii_caracal_* structs
                                                |      • tt_cii_interface cii_h(), cii_c()
                                                |      • tt_cii u_relay(.cii_host (cii_h),
                                                |                       .cii_coproc(cii_c)) — FIFOs
                                                |      • tt_vpu_cii_wrapper_top u_vpu(.cii_intf(cii_c))

Everything SystemVerilog — the ``tt_cii`` credit relay and the VPU — lives **inside** the
``BlackBox``; the Chisel side sees only the flat host-channel wires. The wrapper packs the
flat ``iss_*`` inputs into ``cii_h.iss_data[0]`` (a ``cii_caracal_issue_req_t``) and unpacks
``req_data``/``wb_data`` back to flat outputs. The ``tag`` threads end to end: it labels
every ``req`` and ``wb`` beat, and ``VecCiiHost`` keys its side-table on it (operand-pull →
PRN, result placement, and ``clr_rob`` on the ``last`` beat).

Because ``IQ_V_ALU``'s grant is fire-and-forget (a ``Valid`` with no ``ready``), the adapter
must advertise ``fu_types`` **only** when a real issue credit is available: it mirrors the
issue FIFO's occupancy in a local credit counter (``+1`` per returned ``iss_credit``, ``−1``
per issued beat) and gates ``fu_types`` on it. The gate is registered — a combinational
``fu_types → grant → iss_valid`` path would form a loop — so the credit accounting must be
exact and never advertise at zero credits.

Issue: from ``IQ_V_ALU`` to the CII
-----------------------------------

Vector arithmetic dispatches to ``IQ_V_ALU`` (:doc:`issue`), an **in-order,
non-speculative FIFO**: it selects only the head, and grants only once the head is **past
the PNR** (guaranteed to commit), RoCC-style. Because in-flight CII operations can never be
squashed, |caracal| does **not** implement the speculative model
(``interface_details.adoc``'s shadow table, ``cv0–cv31`` copy registers, and
drain-REQ/drop-WB-on-redirect): the past-PNR discipline removes the need for any
branch-kill or replay on the coprocessor side.

On grant the host adapter (``VecCiiHost``):

- allocates an 8-bit ``tag`` and emits ``{tag, instr, src_reuse_hint}`` on the Issue
  channel, and
- records a **tag side-table** entry — ``{rob_idx, pvdest_grp, pvdest_grp_mask, vl,
  vconfig, vxrm, is_shared, scalar_operands}`` — used later to service operand pulls and to
  place the result.

Vector configuration (``vtype``/``vl``) is delivered to the CII with a serializing VCONFIG
CSR write whenever it changes (the Issue packet carries no vcfg field); ``vxrm``/``vxsat``
are read from the CSR vector interface and travel the same way.

Register operand delivery (VRF read ports)
------------------------------------------

The CII **pulls** operands. On a Src-Request the host adapter decodes ``op_id`` to a source
class — ``SRC1`` → ``pvs1_grp``, ``SRC2`` → ``pvs2_grp``, ``SRC3`` → ``pvs3_grp``,
``MASK`` → ``pvm`` — and ``op_offset`` to the group member index. It reads that member from
the VRF and returns 256 b on Src-Data **in the exact order the requests arrived** (a small
ordering FIFO covers the registered, one-cycle VRF read). Scalar ``.vx``/``.vf`` operands
are captured from the INT/FP bypass at issue into the side-table and served on request; a
reserved ``NONE`` ``op_id`` encodes "no source needed."

|caracal| sizes the VRF at 8R/4W (:doc:`midcore`); the LSU uses read ports 1–4 and write
port 0, leaving **read ports 5–7 and write ports 1–3** for the CII. Operand pulls use VRF
read ports 5 and 6; the VL register file is read to resolve ``pvl`` → ``vl`` at issue.

Result writeback and completion (VRF write port + group-done)
-------------------------------------------------------------

For a **vector-destination** op, a Writeback beat ``{tag, wb_data, wb_dst_offset, wb_wr_en,
last}`` is placed into ``pvdest_grp(wb_dst_offset)`` via VRF **write port 1**. The VPU has
already applied tail-/mask-undisturbed (``vta``/``vma``) policy, so the host writes the beat
**verbatim** — it does not re-apply vl/vtype. ``wb_status`` FP flags accrue toward
``fflags`` at commit.

A handful of vector instructions write a **scalar** register instead (``vmv.x.s`` /
``vcpop.m`` / ``vfirst.m`` → an integer register, ``vfmv.f.s`` → an FP register). The
adapter routes their writeback to the INT/FP register file with a scalar wakeup, rather than
to the VRF, selected by the instruction's destination register type.

Completion uses the writeback **``last`` bit**: the beat marked ``last`` for a ``tag`` is the
final one, and on it the adapter emits — for a vector-destination op — **one**
``VecGroupDone`` (member-PRN vector) plus a single ROB busy-clear (``clr_rob``) for
``rob_idx``, then frees the ``tag``. That one event drives the ROB single-shot busy-clear,
the vector Busy-Table clear, and the VECTOR wakeup network (:doc:`midcore`) — there is no
per-member ROB completion, and the host does not infer completion by counting beats.

The VPU (coprocessor) side
--------------------------

The vector unit that sits behind the CII is Tenstorrent's VPU (``src/main/sv/v4/vpu``). Its
top-level register plumbing is refactored to the CII protocol while its execution datapaths
(the integer and floating-point units) are left largely unchanged.

**The VPU's register file is a staging buffer, not architectural state.** The architectural
vector RF lives in |caracal| (the host). The VPU keeps its own vector register file but uses
it purely as a per-instruction CII staging buffer: the wrapper writes each ``Src-Data`` beat
into it at the member address, the datapath reads it via its existing iterate addresses
(unchanged), and results drain from the datapath's result ports to CII Writeback. Nothing in
that staging RF persists across instructions — the host owns the architectural values.

Instruction, operands, and results map onto the CII channels as follows:

- **Issue** — the CII wrapper unpacks ``{tag, instr[31:0]}`` and feeds the VPU decoder; the
  ``tag`` is correlated with the VPU's internal instruction id so results can be matched
  back. The ``src_reuse_hint`` is **ignored** — the VPU always re-pulls its source operands
  (eliding redundant pulls is a later optimization).
- **Operand pull** — from the decoded source enables the wrapper issues a Src-Request for
  each needed member, **including the ``v0`` mask and, for tail-/mask-undisturbed
  operations, the old destination group**. Returned Src-Data is presented to the datapath in
  member (``op_offset``) order.
- **Tail/mask policy is applied in the VPU.** Because the VPU pulls ``v0`` and the old
  destination, it applies ``vta``/``vma`` (from the vtype it is configured with) internally
  and returns **fully-formed** ``VLEN`` results. The host writes those back to the VRF
  verbatim — it does not re-apply tail/mask.
- **Vector configuration** (``vtype``/``vl``/``vxrm``) reaches the VPU through the CII
  configuration-write path and is held in a small shadow that drives the datapath.
- **Writeback** — each result member is emitted on the Writeback channel tagged with its
  ``tag`` and destination ``wb_dst_offset``; the host places it and, on the last member,
  the completion (group-done) fires as above.

Segmented load/store
--------------------

A segment load/store is a two-half operation: the LSU writes an intermediate ``pvtmp``
group and the CII transposes ``pvtmp`` → ``pvdest``. Such ``is_shared`` instructions wait
for a group-done from **both** halves (the ROB's "other half pending" flag). The CII's
transpose group-done is the second half; with it wired, full segmented LS — deferred in
Milestone 1 — completes.

Ordering with memory operations
--------------------------------

The CII moves no memory traffic of its own; every vector load/store is a host LSU operation
(:doc:`loadstore`). Arithmetic offloaded to the CII is ordered relative to memory purely by
the ROB and the register dependences resolved through rename, so no additional cross-unit
memory-ordering machinery is required beyond the LSU's (see :ref:`mem-order`).
