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
  VConfigUnit — the Vector Config Unit (VCFG): the speculative decode-stage
  `vtype` mirror, and the committed shadow it is recovered from.
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/decode/VConfigUnit.scala,
  package boom.v4.vec.generated.decode. depends_on MicroOp, VecTrace,
  VtypeTable. Instantiated once, as `vcfg` inside VecDecode.

  This unit owns EXACTLY TWO pieces of state and nothing else: the speculative
  mirror and the committed shadow. It is NOT architectural state and NOT a CSR
  file — rocket-chip's `CSRFile` under `usingVector` owns architectural `vtype`
  (including `vill`), `vl`, `vstart`, `vxrm`, `vxsat`, `vcsr`, `vlenb` and
  `mstatus.VS`. And it mirrors `vtype` ONLY: no `vl` here, no `vstart`, no
  `vxrm`.

  ===> THERE IS NO EXECUTE-TIME MIRROR WRITE. No port here is driven from an
       execution unit, and a reviewer should reject one if it appears. `vsetvl`
       (vtype from a register) is marked both `is_unique` and `flush_on_commit`
       by the DecodeUnit delta; recovery of the mirror for it is
       committed-shadow + refetch. `is_unique` alone would NOT do: it stalls
       only the unique uop's own dispatch and says nothing about younger uops,
       which dispatch on the next cycle while `vsetvl` is still in the ALU.

  ===> THE POISON FLAG IS `vtype.vill` ITSELF, not a bit beside it. The mirror
       is one `VType`; poisoning it means setting the `vill` field of that
       value, so it is snapshotted, restored and carried into the uop's
       `vconfig` with the rest and needs no recovery path of its own. A separate
       `mirror_valid` register would be a second piece of speculative state with
       its own (missing) recovery — precisely the bug this shape avoids.

  Governing spec anchors: frontend.rst `vector-rvv-decode` (VSET Special
  Handling), `vector-csr-explicit`, `vector-csr-ownership`, `vcfg-recovery`;
  overview.rst `boom-relationship`, `caracal-pipeline`; midcore.rst
  `regfiles-bypass`, `vl-vtype-rename`. Plan v2 section 5 ground rule 9 and the
  Phase B notes.

<|begin_module|>

  <|begin_parameters|>
  All parameters come from BOOM's existing `Parameters`; this module introduces
  no knob of its own.

  `coreWidth` — the decode/rename/retire width (BOOM ties all three together).
  Default 3 (MediumBoom); legal 1..4. It sets the number of decode lanes, the
  number of `ren_br_tags` entries (`coreWidth + 1`, see the ports section) and
  the number of commit lanes.

  `maxBrCount` — BOOM's branch-tag count, from `BoomCoreParams`. Default 12 at
  MediumBoom sizing; legal range 2..32. It sets the depth of the snapshot array,
  and `brTagSz = log2Ceil(maxBrCount)` the width of a `br_tag`.

  `enableSuperscalarSnapshots` — BOOM's existing knob for whether rename may
  allocate more than one `br_tag` per cycle. True at LargeBoom and MegaBoom
  sizing, false at MediumBoom and the tapeout configs. It selects the snapshot
  write path in part 6; no other part of this module reads it.

  `vLen` / `eLen` / `xLen` — used only indirectly. `vLen` and `eLen` reach this
  module through VtypeTable (hence through rocket's `VType.max_vsew` / `vlMax`);
  `xLen` fixes the width of the `reserved` field of rocket's `VType`. Nothing
  here recomputes VLMAX or the `vill` rule from `vLen` directly.

  `usingRVV` — a Scala `Boolean` from `BoomCoreParams`, NOT a hardware `Bool`.
  With it false this module is not elaborated at all: VecDecode does not
  instantiate it, no register here exists, no port appears anywhere. Absent, not
  tied off — a vectors-off build must emit RTL bit-identical to pre-Caracal BOOM
  v4. Do not gate on rocket's `usingVector`; that switch enables the
  architectural CSRs in `CSRFile` and is a different gate.

  STORAGE SIZING. The mirror, the shadow and every snapshot entry hold a `vtype`
  whose only significant bits are `vlmul_sign`, `vlmul_mag[1:0]`, `vsew[2:0]`,
  `vta`, `vma` and `vill` — nine bits. Rocket's `VType` bundle is nominally
  `xLen` wide because of its `reserved` field, which `VType.fromUInt` forces to
  zero. Store the nine significant bits and reconstruct `reserved = 0` on read,
  so the `maxBrCount`-deep array costs ~9 bits per entry rather than ~64. Do not
  store a raw 64-bit `vtype` per branch tag.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel default and the hierarchy.yaml default for
  this design: single `core_clk` domain, posedge-triggered, with an ACTIVE-HIGH
  SYNCHRONOUS `core_reset`. All state below is `RegInit`-style with the reset
  value given in the logic section.

  ---- Decode-stage inputs, one entry per lane (Vec(coreWidth, ...)) ----

  `dec_fire`        — the lane's instruction ADVANCES out of decode this cycle
                      (BOOM's `dec_fire(w)`). The mirror updates on this, never on
                      a bare `dec_valid`: a bundle can partially fire, so a mirror
                      keyed on validity would absorb a `vset` from a lane that did
                      not advance and then see it re-presented next cycle.
  `dec_is_vset`     — this lane is a `vset*` of any of the three shapes.
  `dec_vtype_imm`   — the encoded `vtype` bits when this lane's `vset` has an
                      IMMEDIATE vtype (`vsetivli`, `vsetvli`). Raw encoding, not
                      yet legality-checked.
  `dec_vtype_is_imm`— this lane's `vset` carries its vtype as an immediate. False
                      for `vsetvl`, whose vtype comes from `rs2` at execute.
  `dec_is_vsetivli` — this lane is `vsetivli` specifically.
  `dec_avl_imm`     — the 5-bit `uimm` AVL of a `vsetivli`.
  `dec_uses_vtype`  — this lane's uOP DERIVES its behaviour (EMUL, element width)
                      from the mirrored vtype. False for a non-vector uOP and,
                      critically, false for a whole-register move/load/store,
                      which takes EMUL from the `NREG` field of its own encoding.
  `dec_ftq_idx`     — this lane's `MicroOp.ftq_idx`.
  `dec_pc_lob`      — this lane's `MicroOp.pc_lob`.
                      These two exist ONLY to tag the trace lines of part 7,
                      and no logic may read them. They are here because
                      `VecTrace.traceDecode(module, event, ftq_idx, pc_lob,
                      extra)` requires them and this unit had no port
                      carrying either, which made part 7 unimplementable as
                      written — a decode-stage unit has no `rob_idx`,
                      because the ROB entry does not exist until dispatch,
                      and VecTrace's own comment says a line claiming
                      `rob=0` is worse than one admitting it does not know.
                      Wired by VecDecode from `dec_uops_in(w)`.
  All but `dec_fire` come from the sibling decoders (VsetDecode for the `vset`
  shapes, VDecode/VLSDecode for `dec_uses_vtype`) or from the incoming uOP
  (`dec_ftq_idx`, `dec_pc_lob`), wired by VecDecode.

  ===> `keep_vl_illegal` IS DELIBERATELY NOT AN INPUT HERE; REJECT IT IF IT
  APPEARS. VsetDecode detects the reserved keep-VL case (`vsetvli rd=x0,
  rs1=x0` where the new VLMAX differs from the current VL) and it is a real
  illegal instruction, but it must NOT poison the mirror. The vtype such a
  `vsetvli` carries is itself perfectly legal — `vill` is clear and
  `VtypeTable.decode` accepts it — so the mirror absorbing it through the
  ordinary path of part 4 leaves the mirror holding a VALID configuration,
  not a corrupt one. The instruction traps, and the trap's commit-time
  flush restores the mirror from the committed shadow by part 6, squashing
  every younger uOP that decoded against it. That is precisely the argument
  part 4 already relies on for `vsetvl`, whose vtype is likewise not
  knowable at decode. Adding the port would make VConfigUnit's poison state
  depend on a legality rule computed in a sibling, for a case the existing
  recovery path already covers.
  
  The keep-VL illegality still reaches the OUTGOING uOP: VecDecode ORs
  `keep_vl_illegal` into that lane's `dec_vec_illegal`, which is where an
  illegal-instruction decision belongs. Mirror state and trap reporting are
  different obligations and only the second one is VsetDecode's to trigger.

  ---- Decode-stage outputs, one entry per lane ----

  `dec_vconfig`     — the selected `vtype` for this lane, as rocket's `VType`;
                      the value VecDecode writes into `MicroOp.vconfig`.
  `dec_prev_vconfig`— the `vtype` in effect IMMEDIATELY BEFORE this lane, i.e.
                      the strictly-EXCLUSIVE prefix: the `scanLeft`'s running
                      value ENTERING lane `w`, which for lane 0 is `vcfg_mirror`
                      itself. Consumed by VsetDecode's `prev_vtype`, whose
                      reserved keep-VL check compares the NEW vtype's VLMAX
                      against the VL that the PREVIOUS vtype was established
                      with.
                      ===> THIS IS NOT `dec_vconfig` SHIFTED BY ONE LANE, AND
                      A CONSUMER MUST NOT RECONSTRUCT IT THAT WAY. The
                      identity `dec_prev_vconfig(w) == dec_vconfig(w-1)` does
                      hold for `w >= 1` — a `vset` at `w-1` publishes its new
                      vtype, a non-`vset` publishes the prefix before itself,
                      and both are the value in effect at `w` — but it is an
                      identity a reader has to re-derive, and it has NO
                      w = 0 case at all: the pre-bundle mirror value is not
                      otherwise observable outside this unit. VecDecode was
                      generated against the shifted form and had to leave
                      `prev_vtype(0)` tied to `DontCare`, feeding an undriven
                      value straight into a legality comparison. Exporting
                      the exclusive prefix directly costs no logic — the
                      `scanLeft` of part 3 already computes exactly these
                      `coreWidth` values — and removes both the DontCare and
                      the proof obligation.
  `dec_vl_imm`      — width `vecVLSz`; the decode-computed VL of a `vsetivli`,
                      with `dec_vl_imm_valid` qualifying it. Output only: it
                      leaves for the rename-cycle VL-RF write and never
                      re-enters this unit.
  `dec_vtype_illegal` — raise illegal-instruction on this lane at decode. Feeds
                      VecDecode's contribution to `dec_vec_illegal` in
                      `vec_pipeline_io`.

  ---- Rename-stage inputs: the per-br_tag snapshot ----

  `ren_br_tags`     — `Vec(coreWidth + 1, Valid(UInt(brTagSz.W)))`, the SAME
                      signal the scalar `MapTable` receives
                      (`rename-stage.scala`'s `ren2_br_tags`) with the same
                      indexing: entry 0 tied invalid, entry `w + 1` for rename
                      lane `w`.
  `ren_br_vconfig`  — `Vec(coreWidth + 1, VType)`, index-aligned with
                      `ren_br_tags`: entry `w + 1` is `ren2_uops(w).vconfig`, the
                      branch's CARRIED vconfig. Sharing one index space makes a
                      misalignment visible at the connection site.

  ---- Recovery inputs ----

  `brupdate`        — BOOM's `BrUpdateInfo`; only `b2.mispredict` and
                      `b2.uop.br_tag` are read.
  `rollback`        — the same single-cycle flush/rollback condition the scalar
                      `MapTable` uses for `map_table := com_map_table`
                      (exception, `flush_on_commit`, CSR replay, ERET,
                      `MINI_EXCEPTION_MEM_ORDERING`).

  ---- Commit inputs: the only writer of the committed shadow ----

  `com_valids`      — `Vec(coreWidth, Bool)`, the ROB's commit valids.
  `com_is_vset`     — this committing uop is a `vset*`.
  `com_vtype`       — the `vtype` that uop installs architecturally, as `VType`.
                      For `vsetivli`/`vsetvli` this is the uop's own `vconfig`
                      snapshot; for `vsetvl` it is the EXECUTED vtype, which is
                      not known at decode. It must be the SAME value BoomCore
                      drives onto `csr.io.vector.set_vconfig.bits.vtype` — one
                      value, two consumers, so the shadow and the architectural
                      CSR cannot disagree.

  ---- Check-only inputs ----

  `csr_vtype`, `rob_empty` — rocket's `csr.io.vector.vconfig.vtype` and
  `rob.io.empty`. These drive NOTHING functional; they exist solely for the
  quiescent-state assertion in the logic section. The generator must not let
  either reach the mirror, the shadow or any output.

  There is deliberately NO port for: an execute-time vtype write, a VL value
  input, a `vstart`/`vxrm`/`vxsat` input or output, a separate `mirror_valid`
  output, or a stall/`busy` output. A `vset` never back-pressures decode from
  here.
  <|end_ports|>

  <|begin_logic|>

  ---- 1. What this unit is, and what it is deliberately not ----

  //@req-spec-decode.d2
  //@req-spec-decode.d3
  This module IS the Vector Config Unit. It keeps a local, speculative copy of the
  `vtype` value most recently set by a `vset`, so younger dependent instructions
  can derive EMUL and snapshot `vtype` at decode without a CSR read or a back-end
  round trip. Because uOP cracking is deferred to the vector LS AGEN, the vector
  mapper must know EMUL at rename to allocate the right number of PRNs — that is
  why this copy has to exist at decode at all.

  //@req-spec-core.b8
  //@req-spec-decode.g6
  //@req-spec-decode.g7
  Vector architectural CSR state is DELEGATED, not reimplemented here.
  Architectural `vtype` (with `vill`), `vl`, `vstart`, `vxrm`, `vxsat`, `vcsr`,
  `vlenb` and `mstatus.VS` dirty tracking / the `VS=Off` gate all live in
  rocket-chip's `CSRFile` under `usingVector`, reached through `csr.io.vector`.
  Caracal owns only the speculative mirror below and the VL register file;
  neither is architectural state, and each is recovered from a committed source
  (this unit's committed shadow, and the VL space's committed pointer).

  //@req-spec-core.d1
  //@req-spec-rename.h6
  //@req-spec-vrf.c5
  //@req-spec-vrf.c8
  VTYPE IS NOT RENAMED. No VTYPE map table, no free list, no busy table, no
  wakeup network and NO VTYPE REGISTER FILE anywhere in the design — only VL gets
  a register file. VTYPE reaches the execution units by riding the mirror into a
  per-uOP snapshot: nine bits in the uop instead of a whole rename space for a
  value that changes rarely and is read by every vector instruction.

  //@req-spec-vrf.c6
  //@req-spec-decode.d4
  That per-uOP delivery is the `dec_vconfig` output: each lane is handed the
  `vtype` that applies to it and VecDecode writes it into that uop's
  `MicroOp.vconfig`, so the EU reads VTYPE off the instruction it is executing —
  never from a CSR at execute, never from this module.

  //@req-spec-decode.d5
  //@req-spec-decode.d13
  THE MIRROR DOES NOT MIRROR `vl`, and VL is never broadcast back into it. There
  is no decode-time VL value at all: VL is renamed into the VL register file and a
  younger vector uOP carries `pvl` and reads the value at execute. The one VL
  quantity computed here (`dec_vl_imm`) is an OUTPUT; nothing brings it back. A
  `vl` input would reintroduce exactly the decode-stage VL tracker that the VL
  rename space replaces.

  ---- 2. State: three elements, and only three ----

  //@req-spec-decode.h3
  `vcfg_mirror` — one `VType` register: the working, speculative copy, updated
  at decode. This is the value read by the per-lane select in part 3.

  //@req-spec-decode.h4
  //@req-spec-decode.f4
  `vcfg_shadow` — one `VType` register: the committed shadow, holding the
  known-good architectural `vtype`. It is written ONLY from the commit inputs,
  when a `vset*` uop commits — the same retire event that writes the
  architectural `vtype` CSR. No other writer exists: not decode, not the ALU, not
  a branch resolution. That is what makes the implicit `vtype` update precise
  (`vl`'s counterpart is made precise the same way, by the VL register file's
  commit path, not by anything here). When several `vset*` uops commit in one
  cycle the shadow takes the NEWEST committing one (highest lane index among
  `com_valids && com_is_vset`), since commit is in program order. Commit rollback
  does not write the shadow — rollback retires nothing.

  //@req-spec-decode.h6
  //@req-spec-decode.h14
  `vcfg_snapshots` — a `maxBrCount`-deep array of `vtype` snapshots, one per
  branch tag, holding the nine bits `vlmul[2:0]` (as rocket's
  `vlmul_sign`+`vlmul_mag`), `vsew[2:0]`, `vta`, `vma` and `vill`. Nine bits
  times `maxBrCount` is a few tens of flops — which is why a per-tag snapshot is
  affordable here where a per-tag copy of a map table is a real cost.

  RESET VALUE. All three reset to all-zero `VType` with `vill` SET — i.e. the
  mirror comes out of reset POISONED.
  This value is not a choice: rocket's CSRFile resets reg_vconfig to
  {vl = 0, vtype = 0} with vtype.vill = 1 (CSR.scala, the `when (reset)` in
  the io.vector block). The mirror must reset to the same value, or the very
  first vector instruction after reset would be judged legal here and illegal
  by the architectural CSR. Being poisoned at reset is also architecturally
  right: `vtype` is invalid until the first `vset`.

  ---- 3. The per-lane select: a prefix, not a broadcast ----

  //@req-spec-decode.h1
  //@req-spec-decode.d11
  The mirror is updated in program order as `vset*` uops decode, and a decode
  bundle is `coreWidth` instructions of program order presented at once. So the
  value each lane sees is a PREFIX SELECT across the bundle, not the bundle's
  newest `vset`: lane `w` takes the nearest PRECEDING `vset` in program order
  within the bundle, falling back to `vcfg_mirror` if no earlier lane in the
  bundle holds one. Implement it as a `scanLeft` over the lanes starting from
  `vcfg_mirror`, in the same shape as the scalar MapTable's `remap_table`
  `scanLeft` — a running value per lane, one mux stage per lane.

  //@req-spec-decode.d9
  THE SELECT IS SELF-INCLUSIVE FOR A `vset` AND SELF-EXCLUSIVE FOR A CONSUMER.
  A `vset`'s own `vconfig` must hold its NEW `vtype`, because that is the value
  the ROB installs in the architectural CSR and in the committed shadow at
  commit; a consumer must see the state BEFORE itself. Concretely, for
  `[vsetvli, vadd, vsetivli, vadd]` in one bundle: each `vset` gets its own new
  vtype, the first `vadd` gets the `vsetvli` value and the second gets the
  `vsetivli` value — not the bundle's newest value in both `vadd` cases, which is
  the mistake this prefix exists to prevent.

  Both outputs fall out of the one `scanLeft` and neither adds a mux stage:
  `dec_prev_vconfig(w)` is the running value ENTERING lane `w` (the `scanLeft`'s
  seed, `vcfg_mirror`, at `w = 0`), and `dec_vconfig(w)` is that same value with
  lane `w`'s own update applied when lane `w` is a `vset` — which is exactly the
  self-inclusive/self-exclusive rule above, restated as the two taps of a single
  prefix network. Export both; do not compute the exclusive one twice.

  LEGALITY IS NOT DECIDED HERE. Every raw `dec_vtype_imm` passes through
  `VtypeTable.decode` (which delegates to rocket's `VType.fromUInt`) before it
  enters the running value, so the `vill` the mirror carries is rocket's own
  disjunction of `!lmul_ok` (the VLMAX >= 1 constraint), `vsew > max_vsew` and
  `reserved =/= 0`. Do not add a local legality rule, a local VLMAX computation
  or a separate comparison against the reserved `vlmul = 3'b100` encoding: a
  second rule that disagrees with the CSR's is worse than no rule, because the
  mirror would then admit a configuration the architectural `vtype` calls
  illegal.

  ---- 4. Updating the mirror ----

  //@req-spec-decode.d12
  On the last lane of the bundle that both fires and holds an immediate-vtype
  `vset`, the mirror takes that lane's post-`VtypeTable.decode` value; that is
  the running value out of the end of the `scanLeft`. This covers `vsetvli` —
  whose VTYPE is immediate and so updates the mirror at decode even though its
  VL needs `rs1` and is produced by the integer ALU EU — and `vsetivli`.

  A `vsetvl` (`dec_vtype_is_imm` false) does NOT update the mirror. It cannot:
  its vtype is a register value unknown at decode. It leaves the mirror holding
  the older value, which is safe only because of part 6.

  //@req-spec-decode.c1
  `vsetivli` IS RESOLVED HERE IN A SINGLE CYCLE, front-end only, with no
  back-end issue slot and no EU: both its VTYPE and its AVL are immediate, so
  the mirror update and `VL = min(uimm, VLMAX)` are both available in the decode
  cycle. Drive `dec_vl_imm` from `VtypeTable.computeVL` applied to that lane's
  own decoded vtype (self-inclusive — VLMAX must come from the NEW vtype, not
  the previous one) and `dec_avl_imm`, and raise `dec_vl_imm_valid`.
  The VL RF write itself is NOT done here and must not be: `pvl` is allocated
  by the VL mapper in the RENAME cycle, so at decode there is no index to
  write to. This output is the value only; the write port is rename's.

  ---- 5. Poison ----

  //@req-spec-decode.g8
  //@req-spec-decode.h15
  A `vill`-setting `vset` POISONS the mirror: the illegal `vtype` is written into
  the mirror with its `vill` field set, marking the mirror invalid. No separate
  register is added — `vill` is a field of the `VType` value already being
  stored, so the poison snapshots and restores with the mirror and needs no
  recovery path of its own.

  //@req-spec-decode.g10
  For `vsetivli` and `vsetvli` the vtype is immediate, so `vill` is known at
  decode and poisons the mirror DIRECTLY, in the same cycle, through the ordinary
  path of part 4 — no special case, no separate poison write. For `vsetvl` the
  vtype is known only at execute and no mechanism is added: it is
  `flush_on_commit`, so younger uops are refetched and decode again against the
  restored mirror.

  //@req-spec-decode.g9
  //@req-spec-decode.g12
  //@req-spec-decode.g13
  While a lane's selected `vconfig` has `vill` set, that lane raises
  `dec_vtype_illegal` if and only if `dec_uses_vtype` is also set. So every
  younger VTYPE-DEPENDENT vector uOP takes illegal-instruction AT DECODE rather
  than reaching rename and allocating a garbage-sized PRN group — which is the
  whole point, since EMUL is derived from the mirror at decode and a mis-sized
  group is silent corruption with no misprediction to recover from.
  The gate is `dec_uses_vtype` and nothing else. A whole-register move, load or
  store (`vmv<n>r.v`, `vl<n>r.v`, `vs<n>r.v`) has `dec_uses_vtype` false, so it
  decodes and executes NORMALLY while the mirror is poisoned: it takes its EMUL
  from the `NREG` field of its own encoding, so the hazard cannot arise for it.
  This is not a nicety. Those instructions are how vector state is saved and
  restored, so trapping them under poison would make the machine unable to
  recover from a vill at all.
  A poisoned mirror still propagates into `dec_vconfig` for every lane,
  including the exempt ones; poison suppresses no output.

  ---- 6. Recovery ----

  //@req-spec-decode.h2
  //@req-spec-core.d2
  //@req-spec-core.d3
  The mirror is speculative, so a `vset*` on a mispredicted path that updated it
  would corrupt the EMUL of every surviving younger vector uop. It therefore gets
  THE SAME RECOVERY MACHINERY AS THE RENAME MAP TABLE — speculative copy, committed
  copy, per-`br_tag` snapshot array — and NOT a mechanism of its own. Precise
  vector state on a redirect or exception comes from that pair plus the per-uOP
  `vconfig` snapshot each uop already carries: no separate vector rollback path,
  no reverse walk, no undo log.

  //@req-spec-decode.h12
  //@req-spec-decode.h13
  THE SNAPSHOT SOURCE IS THE BRANCH'S CARRIED `vconfig`, NOT THE MIRROR
  REGISTER. The mirror lives at decode, one stage ahead of `br_tag` allocation,
  which happens at rename; by the time a tag is allocated the mirror may already
  have absorbed `vset*`s YOUNGER than that branch. Taking `ren_br_vconfig(i)` —
  the nearest-preceding-`vset` value the branch was handed by the part-3 prefix
  select — gives exactly the state as of the branch, so no delayed-`br_tag` path
  and no decode-to-rename shadow pipeline is needed. Do not add one.

  Capture on the SAME `ren_br_tags` allocation event that snapshots the scalar
  and vector RMTs, and split on the SAME `enableSuperscalarSnapshots` parameter
  the scalar MapTable and VecMapTable split on — one structure cannot assume a
  narrower event than the RMTs it must stay in lockstep with.

  When `enableSuperscalarSnapshots` is false a cycle allocates at most one tag,
  so use the scalar MapTable's one-hot reduction: assert
  `PopCount(ren_br_tags.map(_.valid)) <= 1`, then `Mux1H` the tag and the value
  and perform ONE write into `vcfg_snapshots`. One write port, not
  `coreWidth + 1`.

  When it is true — LargeBoom and MegaBoom set it — rename allocates a tag per
  branch in the group, so up to `coreWidth + 1` tags are valid in the same
  cycle. Then write per entry, `when (ren_br_tags(i).valid) { vcfg_snapshots(
  ren_br_tags(i).bits) := compress(ren_br_vconfig(i)) }`, exactly as
  VecMapTable writes `br_snapshots`. The array is registers, not a RAM, so the
  `coreWidth + 1` write ports cost muxing per entry and no arbitration. Neither
  the `PopCount` assertion nor a `Mux1H` may appear on this path: with two tags
  valid the assertion is a false failure and the `Mux1H` would OR the two tags
  into a third, writing one nonexistent snapshot and leaving both branches with
  a stale mirror to recover from.

  The trace line is per write, so under the multi-snapshot path emit one per
  valid entry rather than one per cycle.

  //@req-spec-decode.h8
  //@req-spec-decode.h9
  On `brupdate.b2.mispredict` the mirror is restored from
  `vcfg_snapshots(brupdate.b2.uop.br_tag)` in ONE CYCLE, in lockstep with the
  RMT restore — same trigger signal, same cycle, no handshake between them.

  //@req-spec-decode.h10
  //@req-spec-decode.h11
  On an exception or pipeline flush (`rollback`) the mirror is restored from the
  committed shadow in ONE CYCLE, in parallel with `map_table := com_map_table`.

  Write the three cases as one priority chain with exactly the scalar MapTable's
  ordering, so the two structures cannot diverge on a cycle where both a
  mispredict and a flush are asserted: mispredict first, then flush/rollback,
  then the ordinary decode update of part 4 in the `otherwise`.

  //@req-spec-decode.e2
  ===> THE FLUSH RESTORE MUST READ THE SHADOW'S NEXT VALUE, NOT THE REGISTER'S
       CURRENT OUTPUT. `flush_on_commit` means the flush is raised as the
       `vsetvl` commits — the same cycle the shadow is being written with that
       `vsetvl`'s new `vtype`. Sourcing the restore from the shadow REGISTER
       would reload the value from BEFORE the `vsetvl`, and the refetched
       younger code would decode against a stale `vtype`, derive the wrong EMUL,
       and mis-size a PRN group — the exact silent failure that `is_unique`
       alone already fails to prevent and that `flush_on_commit` was chosen to
       fix. So the restore source is the combinational next-shadow value:
       the newest committing `vset*`'s `vtype` when one is committing this
       cycle, else the shadow register. That expression is also correct if the
       ROB raises the flush a cycle later, so it covers both timings.

  A snapshot write and a mispredict restore never collide for the same tag: a
  branch cannot resolve in the cycle it renames, so `b2` is at least one cycle
  behind the allocation. This matches the assumption BOOM's MapTable already
  makes; do not add a bypass for it.

  ---- 7. Trace and checks ----

  This project has no unit tests — validation is end-to-end VCS plus Whisper
  cosim — so every state change here emits one guarded `VecTrace` line, tagged
  with this module's name and the `rob_idx` of the uop responsible, gated on the
  `vecTrace` plusarg and off by default. Four events, and no more: a mirror update
  at decode (new `vtype`, poisoned or not), a snapshot write (with `br_tag`), a
  restore (with its source, snapshot or shadow), and a shadow update at commit.
  A vtype divergence from Whisper surfaces as a wrong EMUL several stages
  later, in the mapper. These four lines are what make it attributable to the
  cycle the mirror went wrong.

  Assert that `vcfg_shadow === csr_vtype` whenever `rob_empty` holds: with
  nothing speculative in flight the shadow and the architectural CSR — written
  from the same commit event — must agree, and a divergence is otherwise
  invisible until a flush restores a wrong `vtype`. This assertion is the only
  consumer of `csr_vtype` and `rob_empty`.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Single cycle, no pipeline stage, no back-pressure: this unit never stalls decode
and exports no ready or busy signal. `dec_vconfig`, `dec_vl_imm` and
`dec_vtype_illegal` are valid in the same cycle as the instructions that produced
them.

It sits directly in the DECODE critical path, and the depth it adds is the part-3
prefix chain — `coreWidth` mux stages after one `VtypeTable.decode` (rocket's
`VType.fromUInt`, a handful of comparators), with `VtypeTable.computeVL` in
parallel rather than in series. Nothing here may become multi-cycle: the mapper
needs EMUL, derived from `dec_vconfig`, in the following rename cycle, and rename
is already the design's top timing risk.

Both restores are 1 cycle and must remain so, because they run in lockstep with
the RMT restores; an extra cycle would leave the mirror and the map table
disagreeing about which path they are on.

Area is tens of flops (three `vtype` values plus a `maxBrCount`-deep 9-bit
array), so cost is not a design consideration here — correctness of the recovery
ordering is.
<|end_perf|>

<|begin_dependencies|>
VtypeTable — for `decode` (legality and the `vill` rule) and `computeVL`
(`vsetivli`'s VL). Binding here rather than reimplementing is what keeps the
mirror and rocket's architectural `vtype` from disagreeing about legality.

MicroOp — for the `vconfig` field type carried by `ren_br_vconfig` and written
from `dec_vconfig`, and for `rob_idx` in trace lines.

VecTrace — for the guarded trace lines in part 7 of the logic section.

Binds by name to `freechips.rocketchip.rocket.VType`, not a node in this map
because it is upstream and unmodified; the mirror deliberately holds the same
bundle the `CSRFile` holds architectural `vtype` in.

Instantiates nothing; it is instantiated once, as `vcfg`, by VecDecode, which
also wires the sibling decoders into its decode-side inputs. Downstream: VecDecode
(writes `MicroOp.vconfig`, collects `dec_vec_illegal`), VecRenameSpace's
`vl_rename` instance (takes `dec_vl_imm` for the rename-cycle VL-RF write) and
Rob (drives the commit inputs and `rollback`).
<|end_dependencies|>
