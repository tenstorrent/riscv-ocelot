/*
Tenstorrent CONFIDENTIAL
__________________
 Tenstorrent Inc.
 All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Tenstorrent Inc.  The intellectual
and technical concepts contained
herein are proprietary to Tenstorrent Inc.
and may be covered by U.S., Canadian and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Tenstorrent Inc.
*/

/*
  tt_cii_caracal_pkg — DELTA SPEC. Adds ONE enumeration label,
  `CII_SRC_STALE_VD = 3'd6`, to `cii_caracal_srcid_e` in the pre-existing,
  hand-written, already-verified SystemVerilog package
  src/main/sv/v4/tt-cii/src/tt_cii_caracal_pkg.svh. Nothing else in that file
  changes semantically.

  hierarchy.yaml: kind: package, mode: edit_existing, target_hdl: sv,
  target src/main/sv/v4/tt-cii/src/tt_cii_caracal_pkg.svh, group: coproc.
  No `output:` — this node edits its target in place. depends_on: none.
  Added-line budget (plan v2 §3 and §11): "one enum value". One added
  declaration line, plus a comma on the line above it, plus comment lines.
  No line is deleted and no localparam value changes.

  PACKAGE NODE CONVENTION. This node is a shared declaration unit — an SV
  `package` inside an include guard, emitting localparams, typedefs and enums —
  not an instantiable module. It has no I/O, no clock and no reset. Read the
  three sections inside the module block as: the parameters section = the sizing
  knobs (here: what the delta adds, which is nothing, and which existing ones it
  is forbidden to disturb); the ports section = explicitly "None."; the logic
  section = the declarations themselves.

  ===> THIS FILE IS THE FROZEN CONTRACT BOTH SIDES BIND TO. The BOOM host
       adapter (Chisel: VecCiiHost, VecCiiOperandServer, VecCiiWriteback) and the
       VPU coprocessor wrapper (SV: tt_vpu_cii_wrapper_top) are parameterized
       from the same declarations. Every host-side constant is DERIVED from this
       file and must never be independently redeclared with its own literal.

  ===> IT IS ALSO THE ONE SystemVerilog FILE THIS PROJECT MAY EDIT AT ALL, and
       the change is deliberately the smallest one that closes the gap. The wire
       type is ALREADY `logic [2:0]` (`cii_caracal_srcid_t`), so values 6 and 7
       were free: NO payload width change, NO struct change, NO change to
       tt_cii_interface or to the tt_cii_channel credit relay, and the verified
       testbench src/main/sv/v4/vpu/tb/cii_fv_tb.sv stays valid as written.

  ===> CROSS-TEAM DEPENDENCY, not resolvable here. Slot 6 is only useful once the
       VPU DECODER can emit it. tt_vpu_cii_wrapper_top is a blackbox in this map
       and is not touched; that work must be coordinated with the VPU owner. The
       staging is safe in this order: the host serves slot 6 correctly from day
       one and simply never sees a request for it, so this delta can land, be
       simulated and be cosim-validated before the VPU side exists.

  Governing spec anchors: cii.rst `cii-interface`, cii.rst `cii-operands`,
  midcore.rst `vrf-ports`, midcore.rst `old-vd`;
  docs_caracal/caracal-milestone-plan-v2.md §3 (the two SV nodes) and §11.
*/

<|begin_module|>

  <|begin_parameters|>
  The delta adds NO parameter and NO localparam. `CII_SRC_STALE_VD` is a label in
  an existing enumeration whose base type is already `logic [2:0]`, so the value
  6 is a bare literal that needs no new sizing constant.

  Three tempting additions are all forbidden, because each one would convert a
  free label into a payload-width change and break the frozen contract:

  // Do NOT raise CII_NUM_SRC_SLOTS from 4 to 5 to "make room" for the new slot.
  // CII_NUM_SRC_SLOTS sizes cii_caracal_frwd_hint_t, which is a field of
  // cii_caracal_issue_req_t. Widening it widens the Issue payload, which
  // re-resolves the relay's `type(...)`-parameterized FIFOs and breaks both the
  // interface instantiation and the tb's `instr_src_valid <= '0`. The reuse hint
  // is a bitmap over the four HINTABLE slots {VS1,VS2,VS3,VM} and is IGNORED in
  // M2 (host drives 0); it is not a count of srcid encodings.
  // Do NOT add a CII_NUM_SRCID / CII_SRC_LAST count localparam. Nothing in the
  // design iterates over source-slot encodings; a count would be dead and would
  // invite someone to size a bus from it.
  // Do NOT widen cii_caracal_srcid_t. Three bits hold 0..6 with 7 still spare.

  Every existing localparam keeps its current value bit-for-bit. They are
  enumerated in the logic section because their PRESERVATION is what discharges
  most of this node's requirements, not because the generator should re-emit them.
  <|end_parameters|>

  <|begin_ports|>
  None. A SystemVerilog `package` is a declaration unit: it has no I/O, no clock
  and no reset, so there is no clock edge or reset polarity to state. The delta
  adds no port anywhere — in particular it adds no signal to
  `tt_cii_interface` and no lane to any of the four channels.
  <|end_ports|>

  <|begin_logic|>
  ---- 1. THE DELTA: one enumeration label ----

  //@req-spec-vrf.j3
  //@req-spec-cii.f19
  Add exactly one label to the existing `cii_caracal_srcid_e` enumeration,
  immediately after `CII_SRC_SCALAR = 3'd5`:

    CII_SRC_STALE_VD = 3'd6   // -> stale_pvdest_grp(offset)  (old-vd, merging)

  The label lets the coprocessor request the old-`vd` group (`stale_pvdest`)
  INDEPENDENTLY of the explicitly encoded third source (`pvs3`). `pvs3` and
  `stale_pvdest` are two independent MicroOp fields naming two independent
  physical groups (midcore.rst `old-vd`). They coincide for read-modify-write
  arithmetic — `vfmacc.vv vd,vs1,vs2` computing `vd += vs1*vs2` has its third
  source BE the old destination, so the VPU pulls ONE of them — and they diverge
  for a masked `vadd.vv` under `vma = 0`, for `vslideup`'s untouched prefix and
  for a `vcompress` tail, where the VPU pulls BOTH and spends a second of its
  four Src-Request lanes.

  ===> WHY ONE SLOT CANNOT SERVE BOTH ROLES, which is the whole reason this SV
       file is edited at all. An earlier draft had a single VS3 slot that the
       HOST resolved to `stale_pvdest` "when the instruction encodes no third
       source". That is wrong twice over: it puts RVV instruction decoding inside
       the host adapter, and it makes the two-different-groups case
       UNREPRESENTABLE — there is no way to obtain `pvs3` AND old-`vd` for the
       same instruction over one slot. Two slots move the choice to the side that
       already decoded the instruction (the VPU decoder) and reduce the host to a
       side-table lookup. See VecCiiOperandServer, which performs NO
       instruction-dependent reinterpretation: it serves whichever slot is
       requested, straight from the per-tag side-table.

  // The host resolution table, for reference only — it is implemented in the
  // Chisel VecCiiOperandServer, not here:
  //   0 NONE     reserved, "no source needed"
  //   1 VS1      pvs1_grp(offset)
  //   2 VS2      pvs2_grp(offset)
  //   3 VS3      pvs3_grp(offset)            explicitly encoded 3rd source only
  //   4 VM       pvm (the v0 mask)
  //   5 SCALAR   the .vx/.vf value captured at issue (no VRF read)
  //   6 STALE_VD stale_pvdest_grp(offset)    old-vd, for merging   <== NEW
  //   7          reserved, unused

  ---- 2. Why adding a label is structurally inert ----

  //@req-spec-cii.f19
  `cii_caracal_srcid_e` is NOT the wire type. The wire type is the plain
  `typedef logic [2:0] cii_caracal_srcid_t`, and the existing NOTE in the target
  file explains why: `tt_cii_channel` parameterizes its FIFO via `type(...)`
  across two interface instances, and VCS treats enum-bearing payloads as
  distinct types there, producing a port-connection mismatch. The enumeration
  therefore exists only to name values for readability. Adding a seventh label
  changes no type identity, no struct, no packed width and no `type(...)`
  resolution anywhere in the relay. That is exactly what makes this a one-line
  change rather than a contract renegotiation.

  Mechanically, only two lines of the enumeration body are touched: a trailing
  comma is appended to the `CII_SRC_SCALAR = 3'd5` line, and the new label line
  is inserted after it. `3'd6` fits the declared 3-bit base type, so no base-type
  widening and no implicit-width inference is involved.

  ---- 3. Slot 3 keeps its name; the meaning is narrowed in a comment ----

  DECISION, stated explicitly because a silent rename would violate
  `edit_existing` discipline: slot 3 KEEPS its existing identifier
  `CII_SRC_VS3_VD`. Its comment is narrowed from "3rd src / old dest for RMW" to
  "the EXPLICITLY ENCODED third source only — old-vd is CII_SRC_STALE_VD". The
  rename to `CII_SRC_VS3` that the plan and the spec tables suggest is REJECTED
  here, on evidence rather than on principle:

  // `CII_SRC_VS3_VD` has two live references outside this package:
  //   src/main/sv/v4/vpu/tb/cii_fv_tb.sv:207        — the VERIFIED testbench,
  //     which this project is required to keep valid, and
  //   src/main/sv/v4/vpu/tt_vpu_cii_wrapper_top.sv:524 — a BLACKBOX in this map,
  //     which this project may not edit at all.
  // A rename would therefore either break the verified tb or force an edit to a
  // file that is out of scope. Renaming is a VPU-owner change, to be bundled
  // with the decoder work that emits slot 6, not smuggled in here.

  ===> NAMING SKEW ACROSS LANGUAGES, deliberate and benign: the Chisel host and
       the .rst tables call slot 3 `VS3`, while the SV keeps `CII_SRC_VS3_VD`.
       THE CONTRACT IS THE VALUE (3), NOT THE IDENTIFIER. Both sides encode 3 on
       a 3-bit wire and agree on the meaning "explicitly encoded third source".

  ---- 4. Frozen facts the delta must not disturb ----

  Everything below already exists verbatim in the target and is left byte-for-byte
  alone. It is enumerated here only because its PRESERVATION is what discharges
  these obligations, and because the Chisel host DERIVES each of these rather
  than redeclaring a literal of its own — a divergence would be a silent
  port-count or payload mismatch, not a compile error.

  //@req-spec-cii.a3
  //@req-spec-vrf.h2
  This package is the single authority for the coprocessor figures, and the host
  must match it — the direction of that dependency is one-way. `CII_VLEN = 256`,
  `CII_ELEN = 64`, `CII_XLEN = 64`, `CII_MAX_MEMBERS = 8` (EMUL <= 8) and
  `CII_MEMBER_W = 3` mirror boom.v4.vec VectorParams; `CII_VLMAX = 256` and
  `CII_VL_W = 9` are derived here. Cross-checked and consistent: VectorParams'
  corrected `vecVLSz` is also 9 bits, so the Issue packet's `vl` and `vstart`
  fields and the host's VL register file agree on width. `CII_N_TAGS = 16` and
  `CII_TAG_W = 4` size the host's per-tag side-table; the tag is an opaque index
  into that table and is NOT the `rob_idx` and NOT an architectural register.

  //@req-spec-cii.a7
  `CII_NUM_INST_ISSUE = 1`. One instruction-issue lane, because IQ_V_ALU is a
  single in-order issue head. Unchanged.

  //@req-spec-cii.a9
  //@req-spec-cii.a11
  //@req-spec-vrf.h6
  `CII_NUM_SRC_REQ = 4` and `CII_NUM_SRC_DAT_RSP = 4` — four Src-Request pull
  lanes and four matching Src-Data response lanes, giving the coprocessor VRF
  read ports `R5`-`R8` in the canonical static partition (midcore.rst
  `vrf-ports`). Both values are unchanged.

  // ===> THE VALUE 4 IS AUTHORITATIVE; THE INLINE COMMENT IS STALE. The comment
  //      currently on CII_NUM_SRC_REQ still reads "2 VRF read ports for CII
  //      (5,6)", a leftover from a draft that reasoned a pull interface "can
  //      consume at most two VRF reads per cycle". That was wrong in both
  //      directions once the SV froze: there are four request lanes and one
  //      write port. Anyone sizing the VRF from that comment builds a 7R file
  //      and the CII stalls on a read port — which the credit-metered protocol
  //      cannot absorb, since it has no back-pressure line. Read the value.

  //@req-spec-cii.a13
  //@req-spec-vrf.h5
  `CII_NUM_DST_WB = 1`. One writeback lane, hence exactly one coprocessor VRF
  write port (`W2`), owned outright so the CII can never stall on it. This
  constant MUST match the `tt_cii_interface` default: the relay resolves
  `type(wb_data)` against it, so an override that differs is a port-connection
  mismatch rather than a performance choice. The host uses wb lane 0 only.
  Unchanged, and this delta must not create a reason to revisit it.

  //@req-spec-cii.b3
  `CII_N_ISS_CREDITS`, `CII_N_REQ_CREDITS`, `CII_N_DAT_CREDITS` and
  `CII_N_WB_CREDITS` all stay 16. These four depths ARE the handshake: each
  `tt_cii_channel` is credit-metered, a sender holds a free-running credit
  counter and stalls at zero, and NO CHANNEL HAS A `ready` LINE. Consistent with
  that, none of the four packet typedefs declared in this package carries a ready
  or a back-pressure field, and the delta adds none. A consumer that needs to
  refuse a beat has no way to say so, which is precisely why the static VRF
  partition above is load-bearing.

  Every typedef is untouched and is deliberately NOT restated here; the
  must-not-regress list in the edit_scope section enumerates them once, as the
  reject list a reviewer checks. Two layout facts the host derives and must not
  re-invent: the Issue payload `cii_caracal_instr_t` carries
  vtype/vl/vstart/vxrm/frm (the SV has no CSR channel and a 32-bit RVV
  instruction does not encode vtype), and `cii_caracal_wb_status_t` carries
  {last, dst_kind, vxsat, fflags}. Anything not named in this file is unchanged
  by definition.

  ---- 5. No usingRVV gate exists in this language, and none is needed ----

  The design-wide rule that every vector feature is absent (not tied off) in a
  vectors-off build is a Scala `usingRVV` gate, and an SV package has no
  configuration to read. The gate is discharged one level up: the Chisel host
  instantiates the CII BlackBox only under `usingRVV`, so a non-vector build
  elaborates no reference to this package and emits no CII logic at all. Adding
  a label to an enumeration in a package that is not elaborated cannot perturb
  that build. Do not attempt to condition anything here on a parameter.

  No trace statement, no assertion and no `require`-equivalent is added: the
  guarded VecTrace instrumentation lives on the Chisel side, and a declaration
  unit has no event to trace.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
No behaviour, no timing, no area. The one performance-relevant constraint is
NEGATIVE: every declaration here must remain an elaboration-time constant so the
downstream widths continue to constant-fold exactly as they do today, and the
delta must not turn any of them into a signal.

The delta's performance value is indirect and belongs to the coprocessor: with
`pvs3` and `stale_pvdest` requestable as separate slots, a masked or merging op
fetches both in the SAME transaction over two of the four Src-Request lanes,
instead of forcing the host to serialize a separate old-`vd` copy. Four lanes
means two live source pulls plus a mask plus one more still fit in one cycle.
<|end_perf|>

<|begin_dependencies|>
None. `tt_cii_caracal_pkg` sits at the bottom of the SV declaration graph and
must stay that way — it supplies the `type` parameters that instantiate
`tt_cii_interface`, so any dependency here would be a cycle. It includes nothing
and imports nothing.

Its dependents, none of which this delta may modify:
  - src/main/sv/v4/tt-cii/src/tt_cii_interface.sv and the tt_cii_channel credit
    relay — instantiated with these typedefs as `type` parameters. FROZEN.
  - src/main/sv/v4/vpu/tt_vpu_cii_wrapper_top.sv — the coprocessor wrapper.
    BLACKBOX in this map. It is the file that must later be taught to emit
    `CII_SRC_STALE_VD` on `rsp_src_id`; that is the cross-team item.
  - src/main/sv/v4/vpu/tb/cii_fv_tb.sv — the verified testbench, which plays the
    host through the real relay. It must keep compiling and passing unchanged.
  - src/main/sv/v4/generated/tt_cii_host_wrap.sv (node `tt_cii_host_wrap`, mode
    new) — the flattening shim the Chisel BlackBox binds to.
  - Chisel: VecCiiHost, VecCiiOperandServer, VecCiiWriteback, VecCiiFlush and
    VecRegFile, which DERIVE their lane counts, tag width, port numbers and
    payload layout from this file.
<|end_dependencies|>

<|begin_edit_scope|>
  Target:
    File     src/main/sv/v4/tt-cii/src/tt_cii_caracal_pkg.svh
    Package  `package tt_cii_caracal_pkg` (inside the TT_CII_CARACAL_PKG_SVH
             include guard). Hand-written, vendored-adjacent, already verified by
             src/main/sv/v4/vpu/tb/cii_fv_tb.sv through the real credit relay.

  In scope — the enumeration body of `cii_caracal_srcid_e`, and nothing else:
    - Append the label `CII_SRC_STALE_VD = 3'd6` after `CII_SRC_SCALAR = 3'd5`.
    - Append the trailing comma the previous last label now needs.
    - Narrow the trailing comment on `CII_SRC_VS3_VD = 3'd3` from
      "3rd src / old dest for RMW" to "explicitly encoded 3rd source only;
      old-vd is CII_SRC_STALE_VD".
    - PERMITTED BUT OPTIONAL, comment-only, declared here so it is not a silent
      drive-by: correcting the stale inline comment on `CII_NUM_SRC_REQ` ("2 VRF
      read ports for CII (5,6)") to read 4 lanes / R5-R8. A reviewer may reject
      this with no loss; the localparam VALUE must not change either way.
    - Requirement-tag comments may be attached beside the declarations they
      annotate. Comment-only; they change no elaborated value.

  Must not regress — bit-identical, and each is named because it is at risk:
    - VALUES OF THE SIX EXISTING LABELS: CII_SRC_NONE=0, CII_SRC_VS1=1,
      CII_SRC_VS2=2, CII_SRC_VS3_VD=3, CII_SRC_VM=4, CII_SRC_SCALAR=5. Append
      only; never insert and never renumber. These are decoded by
      tt_vpu_cii_wrapper_top and by the tb.
    - IDENTIFIER `CII_SRC_VS3_VD` IS NOT RENAMED. It is referenced by
      cii_fv_tb.sv:207 and tt_vpu_cii_wrapper_top.sv:524, and the latter is a
      blackbox this project may not edit.
    - The enumeration's base type stays `enum logic [2:0]`, and the wire typedef
      stays `typedef logic [2:0] cii_caracal_srcid_t`. The existing NOTE
      explaining why the wire type is plain logic (VCS treats enum-bearing
      payloads as distinct types across two `type(...)`-parameterized interface
      instances) stays, and its reasoning must stay true.
    - EVERY localparam keeps its value: CII_VLEN=256, CII_ELEN=64, CII_XLEN=64,
      CII_MAX_MEMBERS=8, CII_MEMBER_W=3, CII_VLMAX=256, CII_VL_W=9,
      CII_NUM_INST_ISSUE=1, CII_NUM_SRC_REQ=4, CII_NUM_SRC_DAT_RSP=4,
      CII_NUM_DST_WB=1, the four CII_N_*_CREDITS=16, CII_N_TAGS=16, CII_TAG_W=4,
      CII_NUM_SRC_SLOTS=4, CII_MISA=0x201028.
    - EVERY typedef keeps its exact packed layout and width: cii_caracal_vtype_t
      (8 b), cii_caracal_instr_t (63 b), cii_caracal_offset_t, cii_caracal_data_t
      (256 b), cii_caracal_dst_kind_e, cii_caracal_wb_status_t (9 b),
      cii_caracal_frwd_hint_t (4 b), cii_caracal_tag_t (4 b), and the four packet
      typedefs cii_caracal_issue_req_t / _src_req_t / _dat_rsp_t / _result_t.
      Field names and field ORDER included — packed structs are position-typed.
    - No channel gains a ready or back-pressure field; the handshake stays purely
      credit-based.
    - tt_cii_interface.sv, tt_cii_channel and tt_vpu_cii_wrapper_top are NOT
      edited. cii_fv_tb.sv is NOT edited: it must still compile and pass. Its
      `case (rsp_src_id)` has a `default:` arm and selects on the plain
      `cii_caracal_srcid_t`, so the new label needs no tb change and slot 6 need
      not be modelled there.
    - The include guard, the file header comment, the section banner comments and
      the declaration order are preserved.

  Interface delta:
    NEW:      one enumeration label, `CII_SRC_STALE_VD = 3'd6`, in
              `cii_caracal_srcid_e`.
    WIDENED:  nothing.
    RENAMED:  nothing. The `CII_SRC_VS3_VD` -> `CII_SRC_VS3` rename is
              explicitly DECLINED by this spec (see Must not regress); slot 3's
              MEANING is narrowed by comment instead.
    REMOVED:  nothing.
    Reject list — a reviewer must reject the edit if it contains any of:
      - a changed value on any existing srcid label, or an inserted label;
      - `CII_NUM_SRC_SLOTS` raised to 5 (widens cii_caracal_frwd_hint_t, hence
        cii_caracal_issue_req_t, hence the relay's resolved FIFO payload);
      - any change to `CII_NUM_SRC_REQ`, `CII_NUM_SRC_DAT_RSP`,
        `CII_NUM_DST_WB` or `CII_NUM_INST_ISSUE`;
      - a new count/last localparam over the srcid space;
      - a widening of `cii_caracal_srcid_t` beyond 3 bits;
      - any new field in any struct, or any reordered field;
      - a new typedef, a new `import`, or a new `include`;
      - a rename of any existing identifier;
      - an edit to any other file under src/main/sv/.
<|end_edit_scope|>
