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
   hazard_analysis}.adoc``). The RTL that **exists today**:

   - ``tt-cii/src/`` — ``tt_cii.sv`` (the credit relay), ``tt_cii_channel.sv``,
     ``tt_cii_interface.sv``, ``tt_cii_fifo.sv``, ``rv_async_rst_dff*.sv``, and
     ``tt_cii_caracal_pkg.svh`` (the |caracal| parameterization — **the contract both
     sides bind to**).
   - ``src/main/sv/v4/vpu/`` — ``tt_vpu_cii_wrapper_top.sv``, the refactored coprocessor
     top level that speaks the protocol, **verified** by ``vpu/tb/cii_fv_tb.sv`` (which
     plays the host through the real relay and checks per-member writeback, including
     widening/narrowing where dest ``EMUL`` ≠ source ``EMUL``).

   Still **to be created**: the flattening shim ``tt_cii_host_wrap.sv``. A Chisel
   ``BlackBox`` can bind flat ``logic`` ports only — not an SV ``interface``/``modport``,
   and not ``parameter type`` packed-struct ports — and the VPU wrapper's CII port is
   ``tt_cii_interface.coprocessor cii_intf``, so it cannot be a ``BlackBox`` target
   directly.

   This chapter describes only how |caracal| *attaches* to the CII — the issue queue,
   register ports, and completion path.

.. _cii-interface:

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
     - ``{tag[4], instr[32], vtype{sew,lmul,vta,vma}, vl, vstart, vxrm, frm, src_reuse_hint[3]}``
     - ``NUM_INST_ISSUE=1``
   * - **Src-Request**
     - CII → host
     - ``{tag[4], op_id[3], op_offset[3]}``
     - *pull* model; ``NUM_SRC_REQ=4``
   * - **Src-Data**
     - host → CII
     - ``{data[VLEN]}``
     - host answers **in request order**; ``NUM_SRC_DAT_RSP=4``
   * - **Writeback**
     - CII → host
     - ``{tag[4], wb_data[VLEN], wb_dst_offset[3], wb_wr_en, wb_status[9]}``
     - tagged, may return out-of-order; ``NUM_DST_WB=1``

.. note::

   **Tag width is 4 bits (16 in-flight), one parameter ``ciiTagBits``.** ``TAG_T`` is a
   ``parameter type`` in ``tt_cii_interface.sv:27``, so the width is Caracal's to choose, and 4 bits
   matches the 16-tag side-table.



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

.. _cii-host-bridge:

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
                      │   BlackBox TTCii     │───┼─►  tt_vpu_cii_wrapper_top.sv:
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

.. _cii-issue:

Issue: from ``IQ_V_ALU`` to the CII
-----------------------------------

Vector arithmetic dispatches to ``IQ_V_ALU`` (:doc:`issue`), which is **age-ordered collapsing with
a per-entry past-PNR gate**: every entry becomes eligible only once it is **past the PNR**, so each
op handed to the CII is individually non-speculative, RoCC-style. |caracal| therefore does **not**
implement the speculative *rename* model from ``interface_details.adoc`` (its shadow table and
``cv0–cv31`` copy registers): the past-PNR discipline removes any need for rename shadowing or
re-execution on the coprocessor side.

It does **not** remove the need for a flush path — see :ref:`cii-flush`.

On grant the host adapter (``VecCiiHost``):

- allocates a 4-bit ``tag`` and emits the **extended Issue packet** (:ref:`cii-issue-packet`) on the
  Issue channel, and
- records a **tag side-table** entry, used later to service operand pulls and to place the result.

**This list is the single source of truth for side-table membership**; :ref:`vector-execution`
cites it rather than restating it. Every field is retained because a host job needs it *after*
issue:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Field
     - Why the host retains it
   * - ``rob_idx``
     - completes the ROB entry on the ``last`` beat
   * - ``killed``
     - routes the drain-on-flush of :ref:`cii-kill-contract`
   * - ``is_shared``
     - marks one half of a shared instruction, for the ROB's "other half pending" flag
   * - ``pvdest_grp``, ``pvdest_grp_mask``
     - resolve ``wb_dst_offset`` to a physical destination member and its write enable
   * - ``pvs1_grp``, ``pvs2_grp``, ``pvs3_grp``, ``pvm``
     - resolve ``VS1``/``VS2``/``VS3``/``VM`` pulls to a physical member (:ref:`cii-operands`)
   * - ``stale_pvdest_grp``
     - resolves the ``STALE_VD`` pull (:ref:`old-vd`)
   * - ``pdst``
     - completes a scalar result to the renamed physical scalar destination
   * - ``scalar_operands``
     - serves the ``SCALAR`` slot by value, with no VRF read

``vl``, ``vconfig``/``vtype``, ``vstart``, ``vxrm`` and ``frm`` are **not** side-table fields. They
ride in the per-instruction issue packet (:ref:`cii-issue-packet`) and the host never re-reads
them, because the coprocessor applies ``vta``/``vma`` and rounding itself. Writeback routing is not
retained either — it arrives per beat on ``wb_status.dst_kind``.

Vector configuration travels **in the issue packet, per instruction** — ``vtype``, ``vl``,
``vstart`` and ``vxrm`` are fields of ``cii_caracal_instr_t`` (:ref:`cii-issue-packet`).

.. note::

   **There is no serializing VCONFIG CSR write.** An earlier version of this section said vtype/vl
   were "delivered to the CII with a serializing VCONFIG CSR write whenever it changes (the Issue
   packet carries no vcfg field)", directly contradicting this chapter's own channel table. The
   per-instruction packet is authoritative. This matters beyond tidiness: out-of-band configuration
   would impose program-order issue on the CII, which is precisely the constraint
   :doc:`issue` relies on *not* existing in order to drop the head-only FIFO.

.. _cii-flush:

Flush recovery: past-PNR is not a commit guarantee
--------------------------------------------------

.. warning::

   **Past the PNR does not mean "will commit."** |boom| says so itself, in ``rob.scala:436-442``:

   .. code-block:: scala

      when (io.lxcpt.valid && MatchBank(GetBankIdx(io.lxcpt.bits.uop.rob_idx))) {
        rob_exception(GetRowIdx(io.lxcpt.bits.uop.rob_idx)) := true.B
        when (io.lxcpt.bits.cause =/= MINI_EXCEPTION_MEM_ORDERING) {
          // In the case of a mem-ordering failure, the failing load will have been marked safe already.
          assert(rob_unsafe(GetRowIdx(io.lxcpt.bits.uop.rob_idx)),
            "An instruction marked as safe is causing an exception")
        }
      }

   ``clr_unsafe`` fires on a load's **first address translation** (``lsu.scala:1443``), but
   ``order_fail`` is only discovered later, when a store's LCAM search hits that load. So a load goes
   safe → the PNR sweeps past it → CII ops issue → *then* the load order-fails → the ROB-head flush
   squashes everything younger, **including in-flight CII work**. ``MINI_EXCEPTION_CSR_REPLAY``, ERET
   and ordinary exceptions have the same shape. |caracal| makes this *more* frequent than baseline
   |boom|, because :ref:`mem-order` deliberately extends ``order_fail`` to fire on cross-queue
   vector/scalar matches.

**Branch mispredicts, by contrast, genuinely cannot reach the CII.** ``is_br``/``is_jalr`` set
``starts_unsafe`` (``micro-op.scala:164``), so the PNR can never sweep past an unresolved branch, so
no in-flight tag is ever younger than one. Branch recovery costs the coprocessor nothing.


.. _cii-kill-contract:

The kill contract
~~~~~~~~~~~~~~~~~~

**Trigger — ``rob.io.flush.valid`` only.** Never ``brupdate.b2.mispredict`` (unreachable by
construction, above; assert it).

**Scope — every live tag, with no age comparison.** ``io.flush.bits`` carries no ``rob_idx``
(``rob.scala:613-622``) and does not need to: ``flush_val = exception_thrown || flush_commit`` and both
fire at the **ROB head**, while an in-flight tag can never be *older* than the head (commit requires
``rob_bsy`` cleared, which requires the CII's ``last`` beat). So on a flush the adapter simply sets
``killed`` on all live side-table entries. Assert that no in-flight CII tag sets ``flush_on_commit``,
which is what would make the head itself a survivor.

**Behaviour — drain and discard, never ignore.** The VPU is in-order and the SV has **no kill line**,
so a killed instruction must be allowed to *finish* on junk:

.. list-table::
   :header-rows: 1
   :widths: 20 12 68

   * - Channel
     - FIFO owner
     - Action for a killed ``tag``
   * - **Src-Request**
     - host
     - Pop, return ``req_credit``, **and return a don't-care Src-Data beat.** No VRF read (which is
       the real saving — the read ports stay free).
   * - **Writeback**
     - host
     - Pop, return ``wb_credit``, and suppress **all four** effects: the VRF/INT/FP write, ``clr_rob``,
       ``VecGroupDone``, and ``fflags``/``vxsat`` accrual.

.. danger::

   The Src-Data beat is **mandatory**. Credit ownership is asymmetric — the *receiver* owns each
   channel's FIFO and returns credits — so on Src-Data the host is the **sender** and has no credit to
   return. "Ignore the request and hand back a credit" therefore does not apply to that channel: the
   VPU would wait forever for a beat that never arrives, never emit ``last``, never free its tag, and
   **every surviving instruction behind it would be stuck too.** A squash would become a permanent
   hang.

**Tag lifetime.** A killed tag is freed on its (dropped) ``last`` beat, exactly like a live one. The
``killed`` bit and enough side-table state to route the drain must survive the flush, and the tag must
not be reallocated before its ``last`` arrives.

**``killed`` is idempotent.** A second flush arriving while a tag is still draining sets ``killed``
on every live entry again, which changes nothing for an entry that already carries it. The bit and
the side-table state that routes the drain are never cleared while the tag is live — ``killed`` is
cleared only when the tag is allocated — so the drain continues uninterrupted and the tag is freed
on its ``last`` beat however many flushes intervened. There is no re-initialization on the second
flush and no flush counter per tag.

Note that this contract cannot be reused for the vector **LSU** path, even though the shape looks
similar: CII tags are not recycled during the drain, whereas vector destination PRNs *are* returned to
the free list on the flush and promptly reallocated. That path needs real squashing — see
:ref:`vec-squash`.

.. _cii-operands:

Register operand delivery (VRF read ports)
------------------------------------------

The CII **pulls** operands. On a Src-Request the host adapter decodes ``op_id``
(``cii_caracal_srcid_e``) to a source class and ``op_offset`` to the group member index:

.. list-table::
   :header-rows: 1
   :widths: 18 14 68

   * - ``op_id``
     - Value
     - Host resolves to
   * - ``NONE``
     - 0
     - reserved — "no source needed"
   * - ``VS1``
     - 1
     - ``pvs1_grp(op_offset)``
   * - ``VS2``
     - 2
     - ``pvs2_grp(op_offset)``
   * - ``VS3``
     - 3
     - ``pvs3_grp(op_offset)`` — the **explicitly encoded** third source, nothing else
   * - ``VM``
     - 4
     - ``pvm`` (the ``v0`` mask)
   * - ``SCALAR``
     - 5
     - the ``.vx``/``.vf`` **value** from the side-table (no VRF read)
   * - ``STALE_VD``
     - 6
     - ``stale_pvdest_grp(op_offset)`` — the old-``vd`` group, for merging

.. important::

   **``VS3`` and ``STALE_VD`` are two distinct slots naming two distinct groups, and the host never
   conflates them.** ``pvs3`` is an explicitly encoded third source operand; ``stale_pvdest`` is the
   group that held the destination architectural vreg before this ``OP.v`` renamed it (:ref:`old-vd`).
   The host serves whichever slot is requested, straight from the per-tag side-table, and applies no
   instruction-dependent reinterpretation.

   **The coprocessor decides what to pull.** With four Src-Request lanes it issues one request per
   slot it actually needs:

   - For read-modify-write arithmetic — ``vfmacc.vv vd, vs1, vs2`` computing ``vd += vs1 * vs2`` —
     the third source *is* the old destination, so ``pvs3`` and ``stale_pvdest`` name the **same**
     group and the VPU pulls **one** of them.
   - When they differ — a masked ``vadd.vv`` under ``vma = 0`` (old-``vd`` needed for merging, no
     third source encoded), a ``vslideup`` whose untouched prefix comes from old-``vd``, a
     ``vcompress`` tail — the VPU pulls **both**, spending an additional lane.

   .. warning::

      Earlier drafts had a single ``VS3`` slot that the **host** resolved to ``stale_pvdest``
      "when the instruction encodes no third source". That put instruction decoding in the host
      adapter and, worse, made the two-different-groups case unrepresentable: there was no way for
      the coprocessor to obtain ``pvs3`` *and* old-``vd`` for the same instruction. Two slots make
      it representable and move the choice to the side that knows — the VPU decoder.

   Adding slot 6 is a **one-value addition** to ``cii_caracal_srcid_e``: the wire type is already
   ``logic [2:0]``, so values 6 and 7 were free and no payload width changes. It does require the VPU
   decoder to be able to emit it, which is a small SV-side delta rather than a host-only change.

The host reads that member from the VRF and returns 256 b on Src-Data **in the exact order the
requests arrived**. That order is **global across all four lanes**, not per lane: Source-Requests
arriving in the same cycle are ordered by **ascending lane index**, and the Source-Data beats
returned in a cycle occupy ascending lane indices in that same order, so the global sequence is
``(cycle, lane)`` lexicographic. This is what lets the Source-Data payload stay a bare
``{data[VLEN]}`` carrying no ``tag`` and no ``op_id`` — the coprocessor identifies each beat purely
by its position in that sequence. A small ordering FIFO covers the registered, one-cycle VRF read;
because all four reads share the same latency, preserving the order costs no reordering logic.
Scalar ``.vx``/``.vf`` operands are captured from the INT/FP bypass at issue into the side-table and
served on request without touching the VRF.

**Two lanes may name the same member in the same cycle, and the host simply reads it twice.**
Nothing coalesces the duplicate. Each Src-Request lane owns its own read port, so ``vadd.vv v1, v2,
v2`` — whose two sources rename to a single PRN — reads that PRN on two ports in the same cycle at
no cost. The static partition below is what makes the duplicate free, and it is the same property
that guarantees the host never stalls on a VRF port.

Per the canonical port assignment in :ref:`vrf-ports`, the CII owns **read ports ``R5``–``R8``** (one
per Src-Request lane, ``CII_NUM_SRC_REQ = 4``) and **write port ``W2``** (``CII_NUM_DST_WB = 1``);
the VL register file is read to resolve
``pvl`` → ``vl`` at issue. Earlier drafts said the CII had "read ports 5–7 and write ports 1–3" with
the LSU on "read ports 1–4 and write port 0" — a 1-based numbering that also disagreed with
:doc:`midcore`'s own table. Ports are 0-based and statically partitioned; because the partition is
static, the CII **never stalls on a VRF port**, which the credit-metered Writeback channel requires
since it has no back-pressure line.

.. _cii-writeback:

Result writeback and completion (VRF write port + group-done)
-------------------------------------------------------------

For a **vector-destination** op, a Writeback beat ``{tag, wb_data, wb_dst_offset, wb_wr_en,
last}`` is placed into ``pvdest_grp(wb_dst_offset)`` via VRF **write port ``W2``** — one port, because
``CII_NUM_DST_WB = 1`` means the coprocessor cannot present two result beats in a cycle. The VPU has
already applied tail-/mask-undisturbed (``vta``/``vma``) policy, so the host writes the beat
**verbatim** — it does not re-apply vl/vtype. ``wb_status`` FP flags accrue toward
``fflags`` at commit.

A handful of vector instructions write a **scalar** register instead (``vmv.x.s`` /
``vcpop.m`` / ``vfirst.m`` → an integer register, ``vfmv.f.s`` → an FP register). The
adapter routes their writeback to the INT/FP register file with a scalar wakeup, rather than
to the VRF, selected by the instruction's destination register type.

Completion uses the writeback **``last`` bit**: the beat marked ``last`` for a ``tag`` is the
final one, and on it the adapter emits — for a vector-destination op, **and only if the tag is not
``killed``** (:ref:`cii-flush`) — **one**
``VecGroupDone`` (member-PRN vector) plus a single ROB busy-clear (``clr_rob``) for
``rob_idx``, then frees the ``tag``. A killed tag is freed on ``last`` with all four effects
suppressed. That one event drives the ROB single-shot busy-clear,
the vector Busy-Table clear, and the VECTOR wakeup network (:doc:`midcore`) — there is no
per-member ROB completion, and the host does not infer completion by counting beats.

.. _cii-vpu-side:

The VPU (coprocessor) side
--------------------------

The vector unit that sits behind the CII is Tenstorrent's VPU (``src/main/sv/v4/vpu``). Its
top-level register plumbing is refactored to the CII protocol while its execution datapaths
(the integer and floating-point units) are left largely unchanged.

**The VPU's register file is a staging buffer, not architectural state.** The architectural
vector RF lives in |caracal| (the host). The VPU maintains its own vector register per-instruction 
CII staging buffer: the wrapper writes each ``Src-Data`` beat
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
- **Vector configuration** (``vtype``/``vl``/``vstart``/``vxrm``/``frm``) reaches the VPU **in the
  per-instruction Issue packet** (:ref:`cii-issue-packet`) and is latched per ``tag``; there is no
  separate configuration-write path and no cross-instruction configuration state. The VPU must
  **honour a non-zero ``vstart``**, leaving elements below it untouched — software can set ``vstart``
  directly with ``csrw vstart``, and RVV requires those elements to be unmodified.
- **Writeback** — each result member is emitted on the Writeback channel tagged with its
  ``tag`` and destination ``wb_dst_offset``; the host places it and, on the last member,
  the completion (group-done) fires as above.

.. _cii-segmented:

Segmented load/store
--------------------

A segment load/store is a two-half operation: the LSU writes an intermediate ``pvtmp``
group and the CII transposes ``pvtmp`` → ``pvdest``. Such ``is_shared`` instructions wait
for a group-done from **both** halves (the ROB's "other half pending" flag). The CII's
transpose group-done is the second half; with it wired, full segmented LS — deferred in
Milestone 1 — completes.

For a segmented **store** the roles reverse (the CII writes ``pvtmp``, the LSU reads it as store
data), and the coprocessor half cannot be granted until the LSU half has already translated its
address set — a six-step serial chain documented in :ref:`shared-store-chain`. Two consequences land
on this interface: the store's DGEN wakeup must be muxed onto ``pvtmp`` rather than ``pvs3``, and
``IQ_V_ALU`` must not be a head-only FIFO, or that chain would stall every younger vector arithmetic
op behind it.

The coprocessor half learns that the translation is done through **no new mechanism**. The LSU
half's AGEN clears ``unsafe`` on the shared ROB entry, the PNR advances past it, and the half
sitting in ``IQ_V_ALU`` becomes eligible under the same per-entry past-PNR gate every other CII op
passes — steps 2 to 4 of that chain. There is no translation-complete signal, and the handoff is in
particular **not** a ``pvtmp`` group-done: for a store the CII *writes* ``pvtmp`` and the LSU reads
it as store data, so that group-done is produced by the very half that would be waiting on it.

.. _cii-mem-order:

Ordering with memory operations
--------------------------------

The CII moves no memory traffic of its own; every vector load/store is a host LSU operation
(:doc:`loadstore`). Arithmetic offloaded to the CII is ordered relative to memory purely by
the ROB and the register dependences resolved through rename, so no additional cross-unit
memory-ordering machinery is required beyond the LSU's (see :ref:`mem-order`).
