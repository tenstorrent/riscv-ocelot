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
  tt_cii_host_wrap — the flatten shim joining the Chisel BOOM host to the
  SystemVerilog TT-CII stack, and the only new SystemVerilog module in v2.
*/

  hierarchy.yaml: kind: module, mode: new, target_hdl: sv, group: coproc,
  output src/main/sv/v4/generated/tt_cii_host_wrap.sv,
  depends_on tt_cii_caracal_pkg,
  instantiates tt_cii (instance `relay`) and tt_vpu_cii_wrapper_top (instance
  `vpu`), joining two tt_cii_interface instances `ifh` (host <-> relay) and
  `ifc` (relay <-> coproc).

  WHY THIS MODULE EXISTS. A Chisel `BlackBox` can bind FLAT `logic` ports only:
  not an SV `interface`/`modport` port, and not a port whose type comes from a
  `parameter type` packed struct. The TT-CII stack is built out of both —
  `tt_vpu_cii_wrapper_top`'s CII port is `tt_cii_interface.coprocessor cii_intf`
  and `tt_cii_interface` derives every beat from nine `parameter type` arguments
  — so the VPU cannot be a BlackBox target directly. Hence this adapter:

      flat host ports  <->  ifh(host)  --tt_cii relay--  ifc  <->  vpu

  ===> THE RELAY MODPORT CONNECTION IS CROSSED, AND IT LOOKS WRONG. `tt_cii`
       presents the *coprocessor* modport to the real host and the *host*
       modport to the real coprocessor. So `ifh` — the interface this wrapper
       drives as the host — connects to `relay.cii_coproc`, and `ifc` connects
       to `relay.cii_host`. Wiring them "the obvious way" elaborates cleanly and
       silently reverses all four channels. tt_cii.sv's own header says the same.

  ===> THE FLAT PORT LIST IS A HARD CONTRACT. Names, directions, widths and the
       field order inside each packed lane must match the `TTCii` BlackBox io in
       VecCiiHost exactly. A BlackBox port list is type-checked against nothing,
       so a mismatch is a link failure at best and a silently mis-wired channel
       at worst.

  ===> THIS IS THE DESIGN'S SINGLE RESET-POLARITY CROSSING. Chisel's implicit
       reset is active-HIGH; every module in the SV coprocessor stack takes
       active-LOW `rst_n`. hierarchy.yaml declares two named resets
       (`core_reset`, `core_rst_n`) in one clock domain precisely so the
       crossing is a visible design fact and not an undocumented `~reset` buried
       in a shim. The inversion happens HERE, once. The Chisel side must NOT
       invert before driving.

  Governing spec anchors: cii.rst `cii-host-bridge` (all of this module),
  cii.rst `cii-interface` (the four channels and their credit direction),
  execution.rst `vector-execution` "Channel overview" (relay = pure latency
  pipes; each channel's receiver owns the FIFO).

`include "tt_cii_caracal_pkg.svh"

<|begin_module|>

  <|begin_parameters|>
  This module declares NO parameters, deliberately. Every width and lane count
  is imported from `tt_cii_caracal_pkg` via `import tt_cii_caracal_pkg::*` in
  the module header (before the port list, so the ports may use the constants).
  A local parameter would be a second source of truth for a number the Chisel
  host, the interface, the relay and the VPU all already bind to the package
  for, and a drifted copy truncates a channel silently instead of erroring.

  Constants used, with current values: `CII_VLEN` = 256 (one VLEN member per
  beat lane), `CII_TAG_W` = 4, `CII_MEMBER_W` = 3, `CII_VL_W` = 9 (both `vl`
  and `vstart`), `CII_NUM_SRC_SLOTS` = 4, `CII_NUM_INST_ISSUE` = 1,
  `CII_NUM_SRC_REQ` = 4, `CII_NUM_SRC_DAT_RSP` = 4, `CII_NUM_DST_WB` = 1, the
  four depths `CII_N_{ISS,REQ,DAT,WB}_CREDITS` = 16, and `CII_MISA`. `CII_VL_W`
  is 9 bits, not 6: VLMAX at LMUL=8/SEW=8 is 256 elements, so a 6-bit `vl`
  truncates a real architectural VL to zero.

  Widths derived from a struct use `$bits(...)` of that struct, never a literal:
  `$bits(cii_caracal_srcid_t)` = 3 for the op-id and
  `$bits(cii_caracal_wb_status_t)` = 9 for the writeback status. Not style — it
  makes a future field addition a compile error at this boundary rather than a
  silent truncation, which is the one failure this shim cannot detect.

  The include above needs `src/main/sv/v4/tt-cii/src` on the include path. The
  generated output lands in the sibling `src/main/sv/v4/generated/`, so the
  incdir must come from the build (it already does for the rest of the stack,
  via the gen-collateral include directory).
  <|end_parameters|>

  <|begin_ports|>
  CLOCK AND RESET, stated explicitly because this is where the convention
  changes. One clock, `clk`; everything in and below this module is posedge-`clk`
  synchronous — the relay's forward and credit-return pipes and the whole VPU —
  and there is no CDC anywhere in this file. Reset arrives as the ACTIVE-HIGH
  synchronous Chisel reset on `core_reset` and is inverted once, here, into the
  ACTIVE-LOW `core_rst_n` handed to `relay.rst_n`, `vpu.reset_n` and the two
  interface instances' DV reference `rst_n`.

  //@req-spec-cii.c6
  `clk`        input, 1 bit.
  `core_reset` input, 1 bit. Chisel's implicit reset, ACTIVE-HIGH, synchronous.
               ===> The Chisel `TTCii` BlackBox io must declare this port under
               this name and drive it with `reset.asBool` UN-inverted. The
               previous attempt named the port `rst_n` and inverted on the
               Chisel side, putting the crossing where the map says it must not
               be. If the Chisel side is written with `rst_n`, one of the two
               files is wrong and it must be resolved before generation — never
               by adding a second inversion.

  MULTI-LANE PACKING, one rule for all four channels: lane-major, LANE 0 IN THE
  LOW BITS, so lane `i` of a port with per-lane width `W` occupies
  `[i*W +: W]`. That matches an SV packed struct array `[N-1:0]`, whose index 0
  is the low element, and Chisel's `UInt` slicing on the host side. Fields
  within a lane are assigned individually, never by whole-lane bit-cast, so
  field order is explicit here instead of an accident of declaration order.

  Instruction issue — host is the SENDER, so payload in, credit out.
  `CII_NUM_INST_ISSUE` = 1, so the fields are unreplicated:
  `iss_valid` in 1b; `iss_tag` in `CII_TAG_W`=4b (host-allocated, opaque);
  `iss_insn` in 32b (raw RVV word); `iss_vtype` in 8b packed
  `{vsew[2:0], vlmul[2:0], vta, vma}` with vsew in the HIGH bits, per
  `cii_caracal_vtype_t`'s declaration order; `iss_vl` and `iss_vstart` in
  `CII_VL_W`=9b each; `iss_vxrm` in 2b; `iss_frm` in 3b; `iss_hint` in
  `CII_NUM_SRC_SLOTS`=4b (the per-source reuse hint — ignored by the VPU in this
  milestone and driven to zero by the host, carried because it is a field of the
  frozen issue struct); `iss_credit` out 1b.

  Source-operand request — host is the RECEIVER, so payload out, credit in.
  4 lanes: `req_valid` out 1b (qualifies the whole beat); `req_tag` out
  `CII_NUM_SRC_REQ*CII_TAG_W`=16b; `req_op_id` out
  `CII_NUM_SRC_REQ*$bits(cii_caracal_srcid_t)`=12b; `req_op_offset` out
  `CII_NUM_SRC_REQ*CII_MEMBER_W`=12b; `req_credit` in 1b.

  Source-operand data — host is the SENDER. 4 lanes: `dat_valid` in 1b;
  `dat_data` in `CII_NUM_SRC_DAT_RSP*CII_VLEN`=1024b (a scalar `.vx`/`.vf`
  operand rides in the low XLEN bits of its lane); `dat_credit` out 1b.

  Result writeback — host is the RECEIVER. 1 lane: `wb_valid` out 1b; `wb_tag`
  out `CII_NUM_DST_WB*CII_TAG_W`=4b; `wb_data` out
  `CII_NUM_DST_WB*CII_VLEN`=256b; `wb_dst_offset` out
  `CII_NUM_DST_WB*CII_MEMBER_W`=3b; `wb_wr_en` out `CII_NUM_DST_WB`=1b;
  `wb_status` out `CII_NUM_DST_WB*$bits(cii_caracal_wb_status_t)`=9b packed
  `{last, dst_kind[1:0], vxsat, fflags[4:0]}`; `wb_credit` in 1b.

  There are no other ports: no flush port, no busy, no ready on any channel
  (credit is the CII's only backward signal), and no debug port — the VPU's
  `debug_wb_vec_*` outputs stay inside.
  <|end_ports|>

  <|begin_logic|>
  //@req-spec-cii.c2
  This is a THIN, PURELY COMBINATIONAL shim: no register, no FIFO, no credit
  counter, no FSM, no state of any kind. Its whole body is two interface
  instances, two submodule instances and continuous assigns that pack and
  unpack. Everything sequential in the coprocessor path lives inside the relay's
  channel pipes or inside the VPU. Statelessness is what makes the flat-port
  contract checkable by inspection, so "thin" in the spec text is a structural
  claim, not an adjective.

  ===> DELTA FROM THE PREVIOUS ATTEMPT — IT MOVES WORK ACROSS THE SEAM. The
       `addvector` version instantiated two `tt_cii_fifo`s here as host-side
       RECEIVE buffers for the src-request and writeback channels, plus a
       registered valid to regenerate "assert credit -> registered valid/data"
       timing for the flat consumer. v2 does not. `tt_cii` is pure latency pipes
       with no buffering, and the CII contract is that each channel's RECEIVER
       owns the FIFO and returns one credit per pop; on req and wb the receiver
       is the Chisel host, so those FIFOs belong to VecCiiHost. `req_credit` and
       `wb_credit` arriving on these ports are therefore true credit returns,
       forwarded unaltered. That is why this node's `instantiates:` list is
       exactly `tt_cii` and `tt_vpu_cii_wrapper_top`. Do not add a FIFO here.

  ---- Reset inversion (the one crossing) ----

  A single internal net `core_rst_n` is driven as the inverse of `core_reset`
  and is the only reset used below this point. `core_reset` is a synchronous
  active-high reset in one clock domain, so its inverse is a synchronous
  active-low reset in the same domain: no synchronizer, no reset sequencing and
  no assertion-width shaping is needed or permitted here.

  The design's only reset-polarity conversion. See hierarchy.yaml `resets:`.
  core_rst_n = ~core_reset

  ---- The two interface instances ----

  //@req-spec-cii.c5
  Two `tt_cii_interface` instances are declared inside this module and are
  visible nowhere outside it: `ifh`, which this wrapper drives as the host, and
  `ifc`, which the VPU drives as the coprocessor. Both take identical parameters
  — the nine types from the package (`INSTR_T = cii_caracal_instr_t`,
  `SRCID_T = cii_caracal_srcid_t`, `SRC_OFFSET_T` and
  `DST_OFFSET_T = cii_caracal_offset_t`, `TAG_T = cii_caracal_tag_t`,
  `SRC_DATA_T` and `DST_DATA_T = cii_caracal_data_t`,
  `WB_STATUS_T = cii_caracal_wb_status_t`,
  `SRC_FRWD_HINT_T = cii_caracal_frwd_hint_t`) plus the five value parameters
  `CII_NUM_INST_ISSUE`, `CII_NUM_SRC_REQ`, `CII_NUM_SRC_DAT_RSP`,
  `CII_NUM_DST_WB`, `CII_MISA`. Identical parameterization is mandatory: the
  relay cannot bridge two differently-typed instances.

  The overridden lane counts must EQUAL tt_cii_interface's own defaults
  (1/4/4/1). tt_cii sizes each tt_cii_channel as type(cii_host.<sig>), which
  VCS resolves against the interface DEFAULTS and not against these instance
  overrides; a disagreement gives PCWM-L port-width errors rather than a
  clean elaboration failure. Both the interface and the package carry this
  warning. Change a default and its override together.

  Each instance's DV timing reference ports are connected, `.clk(clk)` and
  `.rst_n(core_rst_n)`. They are in no modport, so they are invisible to the RTL
  (every submodule takes its own clk/rst_n on module ports); they exist so a
  monitor or waveform view of the interface has a clock. The previous attempt
  left them unconnected, which is functionally free but makes the `monitor`
  modport useless.

  ---- The credit relay ----

  //@req-spec-cii.c5
  One `tt_cii` instance, `relay`, joins the two interfaces: `.clk(clk)`,
  `.rst_n(core_rst_n)`, and — CROSSED, per the file header — `.cii_host(ifc)`
  and `.cii_coproc(ifh)`. The four credit depths are overridden from the package
  (all 16) and so are the four lane counts, because `tt_cii`'s own defaults are
  1/2/2/2 and disagree with the package's 1/4/4/1.

  tt_cii's lane-count parameters currently only feed tt_cii_channel's WIDTH,
  which that module never references (the payload type T carries the full
  beat width), so overriding them is cosmetic TODAY. Do it anyway: relying on
  an unused parameter staying unused is how a 4-lane channel silently
  becomes a 2-lane one.

  ---- Issue channel: flat inputs -> ifh.iss_data[0] ----

  //@req-spec-cii.c7
  //@req-spec-cii.c4
  The flat `iss_*` inputs are packed field by field into `ifh.iss_data[0]`, a
  `cii_issue_req_t` structurally identical to the package's
  `cii_caracal_issue_req_t`: `.tag` from `iss_tag`; `.instr.insn` from
  `iss_insn`; `.instr.vtype` from `iss_vtype` cast to `cii_caracal_vtype_t`;
  `.instr.vl`, `.instr.vstart`, `.instr.vxrm`, `.instr.frm` from their like-named
  ports; `.instr_src_valid` from `iss_hint`. `ifh.iss_valid` follows `iss_valid`
  and the flat output `iss_credit` follows `ifh.iss_credit`. Index 0 is written
  explicitly rather than by whole-array assignment, so that a widened
  `CII_NUM_INST_ISSUE` becomes a compile error here instead of silent aliasing.

  The vtype cast reinterprets a flat 8-bit port as
  {vsew[2:0], vlmul[2:0], vta, vma}. If the Chisel side ever assembles that
  byte in a different field order, this is where the corruption enters and
  nothing downstream can detect it.

  ---- Source-operand request: ifh.req_data -> flat outputs ----

  //@req-spec-cii.c7
  //@req-spec-cii.c4
  `req_valid` follows `ifh.req_valid` directly — unregistered, because this
  module holds no state and the receive buffer is the host's. For each lane `i`
  in `0 .. CII_NUM_SRC_REQ-1`, in a `for genvar` loop:
  `req_tag[i*CII_TAG_W +: CII_TAG_W]` from `ifh.req_data[i].tag`,
  `req_op_id[...]` from `ifh.req_data[i].rsp_src_id`, `req_op_offset[...]` from
  `ifh.req_data[i].rsp_src_offset`. `ifh.req_credit` is driven from the flat
  `req_credit` input, unaltered and uncounted. The tag is copied field-for-field
  on every beat of this channel and of the writeback channel, which is what
  makes it usable as the host's side-table key end to end.

  //@req-spec-cii.c6
  The op-id is forwarded as opaque bits. This module does not decode it, does
  not know which source slot it names, and specifically needs no change when
  `CII_SRC_STALE_VD = 3'd6` is added to `cii_caracal_srcid_e`: the wire type is
  already `logic [2:0]`, so slots 6 and 7 were free and no width moves. Mapping
  a slot to a physical register number is `VecCiiTagTable`'s and
  `VecCiiOperandServer`'s job, on the Chisel side. Any payload interpretation
  here would be the start of a second, hidden host adapter.

  ---- Source-operand data: flat inputs -> ifh.dat_data ----

  //@req-spec-cii.c7
  //@req-spec-cii.c4
  `ifh.dat_valid` follows `dat_valid`. For each lane `j` in
  `0 .. CII_NUM_SRC_DAT_RSP-1`, `ifh.dat_data[j].rsp_dat` takes
  `dat_data[j*CII_VLEN +: CII_VLEN]`. The flat output `dat_credit` follows
  `ifh.dat_credit`. One field per lane on this channel, so the loop is a slice
  per lane and nothing more.

  ---- Result writeback: ifh.wb_data -> flat outputs ----

  //@req-spec-cii.c7
  //@req-spec-cii.c4
  `wb_valid` follows `ifh.wb_valid`. For each lane `k` in
  `0 .. CII_NUM_DST_WB-1`: `wb_tag` from `ifh.wb_data[k].inst_tag`, `wb_data`
  from `.wb_data`, `wb_dst_offset` from `.wb_dst_offset`, `wb_wr_en[k]` from
  `.wb_wr_en`, `wb_status` from the appended-status field. `ifh.wb_credit` is
  driven from the flat `wb_credit` input, unaltered.

  NAMING TRAP, and it costs an hour every time. tt_cii_interface's own
  `cii_result_t` calls the appended-status field `wb_fp_flags`, a legacy name
  from when it carried only FP flags, so through an interface instance it is
  `ifh.wb_data[k].wb_fp_flags`. The package's convenience typedef
  `cii_caracal_result_t` calls the same field `wb_status`, and it now carries
  {last, dst_kind, vxsat, fflags}. Same bits, two names, depending on which
  declaration you reached them through. Use the interface's name here.

  //@req-spec-cii.c6
  The `last` bit inside that status field is forwarded as data and is not
  interpreted here. `VecCiiComplete` uses it to fire the single group-done
  completion event, so this shim must not consume, retime or regenerate it: a
  beat delayed relative to its `last` marker breaks the one-completion-per-group
  invariant on the Chisel side.

  ---- The VPU coprocessor ----

  //@req-spec-cii.c5
  One `tt_vpu_cii_wrapper_top` instance, `vpu`, parameterized `#(.VLEN(CII_VLEN))`
  and connected `.clk(clk)`, `.reset_n(core_rst_n)`, `.cii_intf(ifc)`. Its
  `debug_wb_vec_valid` / `debug_wb_vec_wdata` / `debug_wb_vec_wmask` outputs are
  left UNCONNECTED on purpose: the cosim commit trace is driven from the BOOM
  side, not from this bridge, and exporting them would add three ports to the
  flat contract for signals no consumer reads. A lint waiver for the dangling
  outputs is expected and is preferable to widening the seam. The VPU subtree is
  a blackbox to this flow; nothing here may assume anything about its internals
  beyond the four channels.

  ---- What this module deliberately does not do ----

  //@req-spec-cii.c6
  //@req-spec-cii.c5
  No credit counting: the issue-credit mirror that gates `fu_types` lives in
  `VecCiiIssue` and must be registered there to break the
  `fu_types -> grant -> iss_valid` loop. No flush or kill handling: `kill`
  quiesces the adapter on the Chisel side, and a killed tag's Src-Data beat is
  still driven through these ports because the CII has no way to decline it. No
  tag allocation, no side-table, no arbitration, no reordering, no dropped beat,
  and no back-pressure invented on any channel. The Chisel side sees the four
  host-channel wire groups and nothing else, which is the property that lets the
  SV stack stay frozen and separately verified.

  This module emits no trace: the shared `VecTrace` helper is Chisel and keys
  every line on a `MicroOp`/`rob_idx`, neither of which exists here, so a
  `$display` would emit lines that cannot be correlated to a ROB entry. The
  channels are traced from `VecCiiIssue`, `VecCiiOperandServer` and
  `VecCiiWriteback`, which do hold the uop.

  With vectors off there is nothing to gate: `usingRVV` false means the `TTCii`
  BlackBox is never elaborated, so this file never reaches the filelist and the
  vectors-off build is bit-identical to pre-Caracal BOOM v4 by construction
  rather than by a tie-off.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
  Zero added latency, zero added area. Every path from a flat port to an
  interface member and back is a continuous assign; the module contributes no
  flop and no logic beyond the single reset inverter, so channel latency is
  entirely the relay's `CRD_REQ_DELAY`/`CRD_RETURN_DELAY` (both 0 here) plus the
  VPU's own pipeline. Throughput is one beat per cycle per channel, set by the
  lane counts (1 issue, 4 src-request, 4 src-data, 1 writeback), not by anything
  in this module — which must never be the reason a beat is not presented in the
  cycle it arrives.

  A constraint on the implementation, not commentary: `dat_data` is 1024 bits
  and `wb_data` 256 bits, both combinational straight through. If this boundary
  ever fails timing the fix is a registered stage on the CHISEL side, or a larger
  relay pipe depth — not a register added here, which would desynchronize a beat
  from its credit and from its `last` marker.
<|end_perf|>

<|begin_dependencies|>
  `tt_cii_caracal_pkg` (src/main/sv/v4/tt-cii/src/tt_cii_caracal_pkg.svh) —
  `depends_on`; included and imported. Supplies every type and constant used
  above. It is `mode: edit_existing` in the map for one added enum value
  (`CII_SRC_STALE_VD = 3'd6`), which changes no width and so requires no change
  in this file.

  `tt_cii_interface` (src/main/sv/v4/tt-cii/src/tt_cii_interface.sv) — frozen,
  verified, blackbox to this flow; instantiated twice as `ifh` and `ifc`. It is
  AUTHORITATIVE for signal and field names: where its names differ from the
  package's convenience typedefs (`wb_fp_flags` vs `wb_status`), the interface
  wins for anything reached through an interface instance.

  `tt_cii` (src/main/sv/v4/tt-cii/src/tt_cii.sv) — instance `relay`. Frozen.
  Instantiates four `tt_cii_channel`s, which are registered forward and
  credit-return pipes with NO buffering FIFO; credit accounting belongs to the
  endpoints, which is why this wrapper does none.

  `tt_vpu_cii_wrapper_top` (src/main/sv/v4/vpu/tt_vpu_cii_wrapper_top.sv) —
  instance `vpu`. Frozen blackbox, verified by cii_fv_tb.sv; v2 writes no VPU
  RTL. Depends transitively on `tt_briscv_pkg` and the whole VPU subtree.

  `VecCiiHost` (src/main/scala/v4/vec/generated/cii/VecCiiHost.scala) — the
  other side of the flat contract. It instantiates this module as its `TTCii`
  BlackBox and owns the src-request and writeback receive FIFOs, the
  issue-credit mirror, tag allocation and the tag side-table.

  BINDING — a v2 ground rule, not a build detail: the BlackBox binds via
  `HasBlackBoxPath` against `src/main/sv/v4/**` DIRECTLY. Nothing is copied into
  `src/main/resources/vsrc/`. The previous attempt copied seven SV files there,
  one of them submodule content, which silently falsifies v2's central premise —
  you verify one file and simulate another, with nothing to tell you. If
  `addPath` fights the Chipyard flow, the fallback is a checksum-guarded sync
  step that FAILS the build on drift. Never a silent copy.
<|end_dependencies|>
