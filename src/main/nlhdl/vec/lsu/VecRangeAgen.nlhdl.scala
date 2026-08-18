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
  VecRangeAgen — the FILL-side address generator for UNIT-STRIDE vector
  accesses: it encodes one whole vector load or store as EXACTLY ONE range
  entry describing the contiguous byte range [base, base + VL*EEW).
*/

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecRangeAgen.scala,
  package boom.v4.vec.generated.lsu.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace. Instantiates nothing.
  Instantiated TWICE by VecLsu — `ld_range_agen` (isStore = false) and
  `st_range_agen` (isStore = true) — so the two directions never arbitrate.

  ===> IT EMITS ONE ENTRY PER INSTRUCTION AND EXPANDS NOTHING. Element expansion
       into per-element D$ accesses happens LATER, just-in-time on the DRAIN side
       in VecBeatExpander. Putting the expansion here is precisely the mistake v2
       exists to fix: `origin/Caracal/addvector` placed the OVI Packer on the FILL
       side, so nothing could coalesce late and every access degenerated to one
       64-bit beat per element — 8x the access-count floor at SEW=8 (plan section
       2 bug table, "Packer on the fill side"; target P1). One range entry here is
       what leaves the coalescing decision to the cycle the D$ port is granted.

  ===> IT HOLDS NO PER-INSTRUCTION STATE AND EXPORTS NO `busy`. There is no cursor
       to retire because there is nothing to walk: the entry is pushed in the
       cycle the request fires and the module forgets the instruction. It never
       waits on a D$ response, a TLB response or a completion (the vector-LSU
       invariant, plan section 5 rule 6). Grep gate H4 must find nothing here.

  ===> IT OWNS THE FAULT-ONLY-FIRST POLICY, AND THAT IS A CORRECTION TO AN EARLIER
       ALLOCATION. `vle<eew>ff.v` IS ARCHITECTURALLY UNIT-STRIDE — RVV 1.0 defines
       the fault-only-first form only for unit-stride loads — so its OP.v
       self-selects into this module by the class rule in logic section 1 and NEVER
       REACHES `VecElemAgen`. The six obligations that state the policy
       (element-0 trap versus trim at element `i > 0`) were allocated to the
       element agen by the architect pass; that was a routing mistake, not a
       disagreement about the mechanism, and they now live here, in logic section
       9, on the only agen a `vleff` can actually arrive at.
       This does NOT reintroduce state. The policy is a COMBINATIONAL
       CLASSIFICATION of a fault report the drain side raises against the RETAINED
       RANGE ENTRY — the entry in `ld_US_ADDR_Q` is what remembers the
       instruction, exactly as it does for every other unit-stride fault — so this
       module still holds no cursor, no "current op" register and no completion
       tracking. See the ports section for the three-signal interface that carries
       it.

  Governing spec anchors: execution.rst `vector-agen` (the Walker/Vector-AGEN
  paragraphs), loadstore.rst `us-queue`, `store-data-queue`, `mem-order` and
  `elem-progress` (its fault-only-first subsection), midcore.rst `vrf-ports` (R1
  and R4 have exactly one reader each) and `precise-vec-exc`,
  caracal-milestone-plan-v2.md section 2 (the five structural changes) and
  section 5 rules 6 and 11.

<|begin_module|>

  <|begin_parameters|>
  `isStore`, a Scala `Boolean`, default false. DIRECTION IS A PARAMETER, NEVER A
  SEPARATE MODULE (plan section 2, structural change 3). It selects the target
  queue (`ld_US_ADDR_Q` when false, `st_US_ADDR_Q` when true), whether the
  store-data command port exists at all, and which VRF port the incoming latched
  mask was read on (`R1` on the load path, `R4` on the store path — see the
  ports section).

  Two instances are why the load-priority mux is gone: in `addvector` one
  shared generator dropped store grants, worked around by not advertising
  FC_AGEN on the store path whenever the load was granting.

  Elaborated only when `usingRVV` is true — a Scala `Boolean` from
  `BoomCoreParams`, not a hardware `Bool` and not rocket's `usingVector`. With
  vectors off this module is ABSENT, not tied off, so a non-vector build stays
  bit-identical to pre-Caracal BOOM v4.

  Every other figure comes from VectorParams (`HasVectorParams`) and
  `HasBoomCoreParameters` — `vLen`, `eLen`, `maxMembers`, `vecPregSz`, `xLen`,
  `robAddrSz`, `ldqAddrSz`, `stqAddrSz` — and no width below is a literal.

  There is deliberately NO depth, FIFO-size or state-machine parameter: such a
  knob would only be meaningful if this module buffered instructions.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the hierarchy default and Chisel's implicit convention:
  posedge `clock`, ACTIVE-HIGH SYNCHRONOUS `reset` (`core_clk` / `core_reset`).
  The module is combinational from request to push, so reset reaches only the
  optional output register described in the logic section.

  ---- `io.req` — the issued OP.v (input, VALID-ONLY) ----

  A `Valid`, NOT a `DecoupledIO`, carrying the vector load or store that issue
  granted: the full `MicroOp` (`rob_idx`, `ldq_idx`/`stq_idx`, `v_eew`, `v_emul`,
  `v_seg_nf`, `pvdest`, `pvm`, `pvs3`, `pvtmp`, `is_shared`) plus the STATIC ACCESS
  DESCRIPTOR VLSDecode produced: the access-class flags `is_unit_stride`,
  `is_whole_reg`, `is_mask_op`, `is_indexed`, `is_strided`, `is_ff` and the segment
  field count. That descriptor — never a re-decode of `uop.inst` here — is what
  selects this module over VecElemAgen, and `is_ff` is what the fault-only-first
  policy of logic section 9 keys on. `is_ff` implies `is_unit_stride`; assert it.

  ===> THERE IS NO `ready` ON THIS PORT, AND ADDING ONE BACK IS A REVIEW
  FAILURE. The producer cannot consume a ready: `VecScalarOperandRead.out` has
  none, it overwrites its stage register every cycle and publishes no readiness,
  so a ready declared here would be a wire nobody reads — or worse, a wire a
  generator decides to honour by inventing back-pressure that must not exist.
  The accept can never need one: a unit-stride instruction takes EXACTLY ONE
  entry in a region reserved at dispatch, in program order, and that region is
  never pre-filled by anyone else, so the push cannot be refused for lack of
  room. `Valid` states that fact in the type instead of leaving an
  always-true `ready` for a later reader to mistake for a real handshake. The
  mid-walk descriptor hazard that motivates back-pressure at the ELEMENT agen
  does not exist here either, because this module has no walk: the instruction
  is finished with it in the cycle it fires (logic section 10).

  ---- `io.scalar` — from this direction's VecScalarOperandRead (input) ----

  `base` (`xLen`), the rs1 effective base address; `stride` (`xLen`), present in
  the shared operand bundle and IGNORED here; `vl` (VLMAX-wide, see below), read
  from the VL register file through `uop.pvl`; `vm`, high when the instruction is
  unmasked; and `mask`, the LATCHED single `vLen`-wide read of `v0` reduced to
  VLMAX element-granular bits.

  ===> THIS MODULE ADDS NO VRF PORT AND NOWHERE READS THE VRF. The mask
  arrives already read, because midcore.rst `vrf-ports` requires R1 and R4 to
  have EXACTLY ONE READER each and that reader is the stage-1 operand read for
  the direction. A US OP.v reading v0 here while an SSI OP.v read it in
  VecElemAgen would be two concurrent readers of a statically partitioned,
  never-arbitrated port. The port by number: R1 (load) / R4 (store), add none.

  ===> VLMAX IS `maxMembers * vLen / 8` ELEMENTS — 256 at the defaults, not
  32 — the worst case being SEW=8 with LMUL=8, one element per BYTE of the
  group. Size the `vl` input and the mask vector from that, and take the width
  from VectorParams rather than open-coding it: `maxVecVL` IS THAT FIGURE
  (`maxVecVL = vLen` = 256 at the defaults, the two coinciding numerically
  because `maxMembers = 8` and `vLen` is in bits), and `vecVLSz` is its 9-bit
  width. An earlier revision of this file complained that `maxVecVL` was
  `vLen / 8` and 8x too small to hold a VL; THAT COMPLAINT IS STALE and has been
  deleted — VectorParams was corrected long ago, so `maxVecVL`/`vecVLSz` are the
  right things to use and there is nothing to work around.

  ---- Reservation lanes, to `resv` (VecQueueReservation) — LANES 2 AND 3 ----

  ===> THESE PORTS WERE MISSING ENTIRELY, AND THAT WAS THE SECOND DANGLING END OF
       ONE SEAM. `VecQueueReservation` and `VecLsu` both fix the four execute-side
       lookup/release lanes as 0 `ld_elem_agen`, 1 `st_elem_agen`, 2
       `ld_range_agen`, 3 `st_range_agen` — and an earlier revision of this file
       mentioned the reservation nowhere at all, so TWO NAMED LANES HAD NO
       CONSUMER. That is the same shape of defect as A52 (`q_free` with no driver
       anywhere in the subtree): a port pair specified from one side only, which
       elaborates and then does nothing. Two dangling ends on one seam is the
       thing to take away — a lane list agreed by two files is not evidence that
       the third file implements it, and neither end was found by reading either
       of the two that agreed.

  `resv_lookup` — Output(Valid({ `is_store`, `q_idx` })): the combinational read of
  the region this OP.v owns, keyed on `uop.ldq_idx` on the load instance and
  `uop.stq_idx` on the store instance, fired in the same cycle as the request. On
  lane 2 when `isStore` is false and lane 3 when it is true; VecLsu binds the lane
  and nothing here selects it.

  `resv_resp` — Input(`Vec(2, { base, count })`): the granted region, PER QUEUE
  SLOT. Slot 0 is the ADDRESS queue (`ld_US_ADDR_Q` / `st_US_ADDR_Q`), slot 1 the
  DATA queue (`st_US_DATA_Q`, valid on the store instance only).

  ===> AND THE TWO SLOTS DO NOT AGREE ON THIS PATH. THAT IS THE WHOLE POINT.
  `st_US_ADDR_Q` holds ONE range entry per store while `st_US_DATA_Q` holds one
  full `vLen` entry PER GROUP MEMBER, so a US store claims 1 address entry and
  `v_emul * v_seg_nf` data entries: DIFFERENT COUNTS, DIFFERENT BASES,
  DIFFERENT POINTERS. The equal-slot assertion that is correct in `VecElemAgen`
  (the SSI queues have equal depth and one claim event) MUST NOT be written
  here, and `us_data_base` on the range entry is SLOT 1's BASE — never slot 0's
  plus an offset, which is the tempting derivation and is simply not a fact
  about these two queues.

  `release` — Output(Valid({ `is_store`, `q_idx`, `used_count`: `Vec(2, UInt)` }))
  with `release_ok` — Input(Bool): the surplus return, PER QUEUE SLOT, fired once
  per OP.v. A denied release is NEVER retried.

  ===> ONLY THIS MODULE CAN RELEASE A US STORE'S DATA-SIDE SURPLUS, which is why
  these lanes are load-bearing rather than symmetric filler. The reservation
  claimed `v_emul * v_seg_nf` data entries from EMUL at dispatch, and the number
  of members ACTUALLY written depends on VL — read from the VL register file at
  execute, in this module, and nowhere earlier. Nothing else on the unit-stride
  path has that information: the reservation is a dispatch-time structure, the
  queues count entries and not members, and VecDgen learns `members_used` only
  from the same VL this module already has. Omitting the release does not
  under-perform, it LEAKS the difference between the EMUL worst case and the
  real member count on every short-VL unit-stride store until reclamation.

  ---- `io.range` — the range entry (output, ready/valid) ----

  A `DecoupledIO` of `VecRangeEntry` (declared in VecBundles) pushed into
  `ld_US_ADDR_Q` or `st_US_ADDR_Q` per `isStore`. AT MOST ONE FIRE PER
  INSTRUCTION, ever.

  ===> VecRangeEntry MUST CARRY THREE FIELDS ITS VecBundles SUMMARY DOES NOT
  YET ENUMERATE, and they are requirements, not conveniences: the effective
  `stride`, the `is_unit_stride` flag, and the element-granular `mask`
  (spec-agen.b7, spec-agen.e13). Its `len` field must additionally be sized
  for a whole LMUL=8 group of BYTES — `maxMembers * vLen / 8` = 256, i.e. 9
  bits — and never from `vecVLSz`, which is the same WIDTH but a different
  QUANTITY: `len` counts bytes, `vecVLSz` sizes an element count, and the two
  coincide only because the narrowest SEW is one byte. TWO MORE FIELDS join them:
  `is_ff`, now that the fault-only-first policy lives here — the entry is what
  the drain side raises a fault against, and `is_ff` is how that report knows to
  come back as a trim question rather than as a trap — and `us_data_base`, the
  store side's DATA-QUEUE base, taken from reservation SLOT 1 (see the
  reservation lanes above). Reported as a seam item for cross-file review; this
  module drives all of them.
  On `us_data_base` there is one open redundancy to settle rather than to
  duplicate: `VecQueueReservation` and `VecLsu` both say the range ENTRY carries
  it, while `VecDgen`'s own file still says it takes the data-queue base from
  `VecQueueReservation` directly. Either is workable; BOTH is two sources of
  truth for one pointer. This module drives the field as the two amended files
  require, and the duplicate read is flagged for VecDgen's side to drop.

  ---- `io.st_data` — the whole-register store-data command, KEPT AS AN
       ASSERTION-ONLY CROSS-CHECK (output) ----

  Present only when `isStore`. A `DecoupledIO` to VecDgen carrying, per group
  member, the member index, that member's source PRN, `rob_idx`, `stq_idx` and a
  `last` marker. Absent on the load path — a load has no source group to read.

  ===> RECONCILED, AND THE DECISION IS "KEEP IT, BUT ONLY AS A CHECK". This port
  is REDUNDANT as a mechanism: VecDgen derives the whole US member sequence
  ITSELF, from its own `io.req` and its own `members_used = ceil(total_bytes /
  vLenBytes)` bound, in ascending member order with the partial final member
  carrying its valid-byte count — and it must, because that total-byte
  derivation is the fix for the M1 phantom-member store-data corruption and
  cannot be delegated to a command stream from here. VecDgen owns VRF port `R3`
  and the push into `st_US_DATA_Q`; nothing about the data path depends on this
  port.
  So the two candidate mechanisms are settled down to ONE, VecDgen's, and this
  port survives as the AGREEMENT ASSERTION between the address side and the data
  side: VecLsu ties its `ready` HIGH — permanently, so it is not a handshake and
  cannot back-pressure anything — and compares the member index and `last` this
  module publishes against the member VecDgen is streaming for the same
  `stq_idx`, failing loudly on a disagreement. That check is worth its wires
  precisely because the 1:N address-to-data shape below is easy to mis-review
  and its failure mode is a neighbour store's data corrupted rather than a hang.
  It is a CHECK AND NOT A COMMAND, and a reader who wires it into VecDgen's data
  path has reintroduced the second mechanism this note exists to remove.
  ONE UNRESOLVED SHAPE ITEM, RECORDED RATHER THAN QUIETLY FIXED: by the same
  argument that makes `io.req` a `Valid`, a port whose `ready` is permanently
  tied high should not be a `DecoupledIO` either. It is left as declared here
  because VecLsu's spec is the side that states the tie-off, and the two files
  must agree; narrowing it to a `Valid` is a seam item for VecLsu and VecDgen to
  settle together, not a unilateral change from this file.

  ---- The fault-only-first interface (three signals, no state) ----

  `io.fault` — Input(Valid({ `elem_idx` of `vecVLSz`, `is_ff`, `rob_idx`,
  `ldq_idx` })): the drain side's fault report for a unit-stride LOAD, routed and
  qualified by VecLsu. It is SELF-DESCRIBING on purpose — the element index and the
  owning identifiers come from the RETAINED RANGE ENTRY the fault was raised
  against, not from any latch here — so this module needs no record of the
  instruction to classify it, and the statelessness claim in the header survives.
  Present on the load instance only; a store has no fault-only-first form.

  `io.fault_trap` — Output(Bool): raise this fault to the ROB as a plain precise
  exception. `io.ff_trim` — Output(Valid(UInt(vecVLSz.W))): the trimmed element
  count. Both are COMBINATIONAL FUNCTIONS OF `io.fault` — one comparison against
  zero, per logic section 9 — with no register between input and output.

  ===> `io.ff_trim` GOES TO `lcb.io.trim`, NOT TO VlRegFile's `W_lsu`. VecLsu
  converts the trim element index into the LCB's {member, `keep_bytes`} form
  using `v_eew`, and the VL register file's `W_lsu` port keeps EXACTLY ONE
  producer — `lcb.io.vl_wb`, driven on the group-done. This module therefore
  REPORTS the trim and never writes the VL RF itself. Two reasons, both
  load-bearing: `W_lsu` is a statically partitioned, never-arbitrated write
  port, so a second producer is a structural error rather than a contention
  problem; and publishing a trimmed VL before the group it describes has been
  assembled would wake every `pvl` dependent early, handing them a VL whose
  data is not yet in the VRF.

  ---- `io.brupdate`, `io.rob_flush` (inputs) ----

  BOOM's existing `BrUpdateInfo` and the ROB flush pulse, used only to qualify
  the push valid for a uop killed in the push cycle, via the existing
  `IsKilledByBranch` helper. No new kill mechanism: recovery of an entry that
  did get pushed is VecSquashUnit's pointer rollback.

  ---- What is deliberately NOT a port ----

  No `busy`, `active`, `ready_for_next` or any other output that could reach an
  issue unit. No `ready` on `io.req` (see above). No D$, TLB or LCAM port — the
  drain side owns memory. No group-done and no VRF write port — completion belongs
  to the LCB (loads) and to commit-drain (stores). No VL register-file write port:
  the trim is reported, not written. No queue-credit or allocation port: capacity
  was reserved at dispatch, and the reservation lanes above are NOT that — they
  READ a region already granted and RETURN its surplus, and neither direction can
  refuse an instruction or make it wait.

  And one careful non-port: NO `badvaddr` AND NO `vstart` OUTPUT. The fault-only-
  first pair above raises or suppresses a trap; it does not describe one. The
  faulting address is the drain side's — it is the side that formed the beat — and
  a unit-stride fault traps with `vstart = 0` regardless, so there is no element
  index for this module to publish and no partial-restart value it could compute.
  An earlier revision of this file said this module has no exception output AT ALL;
  that was written before the `vleff` policy was allocated here, and the exception
  output it now has is exactly one Bool plus one trim count, both derived from a
  report it is handed.
  <|end_ports|>

  <|begin_logic|>
  The module is a single-cycle, stateless encode. There are no registers other
  than an optional output pipeline register on `io.range`; if that register is
  used it is cleared by `io.rob_flush` and by a branch kill. It may not gate the
  accept at all — `io.req` carries no `ready` — so if it is used it must be sized
  to a path that never needs to hold, which for a reserved one-entry-per-
  instruction region it is. The fault-only-first classification in section 9 adds
  no register either.

  ---- 1. What this module accepts, and what it refuses ----

  It fires only for the three CONTIGUOUS access classes: plain unit-stride
  (`vle{8,16,32,64}` / `vse{8,16,32,64}`, INCLUDING `vle<eew>ff.v`), whole-register
  (`vl1re*`..`vl8re*` / `vs1r`..`vs8r`) and mask (`vlm.v` / `vsm.v`). Everything
  else — strided, indexed, and ANY segmented access with `nf > 1`, unit-stride
  segmented included — is VecElemAgen's. A unit-stride segmented access is
  contiguous in memory but its elements interleave across the destination
  register groups, so one range entry could not say where the bytes land; that
  is a routing rule, not an optimization.

  `resv_lookup` fires in that same cycle, on this instance's fixed lane, keyed on
  `ldq_idx` or `stq_idx`, and `resv_resp` is USED COMBINATIONALLY AND LATCHED
  NOWHERE: slot 0's base becomes the entry's own absolute index and slot 1's base
  becomes `us_data_base` on the entry, both inside the push cycle. That is what
  keeps the "no per-instruction state" claim true with a reservation seam attached —
  the region is read, spent and forgotten in one cycle, and the entry sitting in the
  queue is what remembers it afterwards.

  `vleff` IS IN THE FIRST OF THOSE CLASSES AS A MATTER OF ARCHITECTURE, not of
  convenience: RVV 1.0 defines the fault-only-first form for unit-stride loads and
  for nothing else, so a `vleff` OP.v SELF-SELECTS INTO THIS MODULE on
  `is_unit_stride` and CANNOT ARRIVE AT `VecElemAgen`. That is the whole reason the
  trap-versus-trim policy of section 9 is stated here rather than there — an
  earlier allocation put it on the element agen, where the class rule guaranteed it
  could never execute. Assert `is_unit_stride` on every request carrying `is_ff`,
  so a decode regression that mis-classes a `vleff` fails here instead of losing
  the trim silently.

  ---- 2. The single entry ----

  //@req-spec-agen.b6
  //@req-spec-lsu.c3
  A unit-stride OP.v is encoded in a SINGLE nOP.v — one `VecRangeEntry` — and no
  second entry is emitted for it under any VL, EMUL, mask pattern or alignment.
  One entry stands for what would otherwise be up to `vLen` element accesses (256
  at the defaults, SEW=8 with LMUL=8). That is what lets the unit-stride queues
  be sized in INSTRUCTIONS rather than elements — `usQueueEntries` 16 against
  `ssiQueueEntries` 512 — and why the SSI element-count floor does not apply to
  them (loadstore.rst `ssi-queues`, last paragraph).

  //@req-spec-lsu.c4
  The entry describes the WHOLE CONTIGUOUS RANGE `[base, base + VL*EEW)`.
  Compute it as a base and a total byte length, normalizing all three accepted
  classes onto (base, element count, effective element size in bytes):

    - unit-stride:   count = `vl`,                    bytes/elem = `1 << v_eew`
    - whole-register: count = `nfields * vLen / 8`,   bytes/elem = 1
    - mask:          count = `ceil(vl / 8)`,          bytes/elem = 1

    total_bytes = count * bytes_per_element

  //@req-spec-agen.b7
  The entry carries the EFFECTIVE BASE ADDRESS, the EFFECTIVE STRIDE and the
  `is_unit_stride` flag SET HIGH. The effective stride of a contiguous access is
  the element size in bytes — `1 << v_eew` for unit-stride, 1 for the
  whole-register and mask forms — carried explicitly even though it is derivable
  from `eew`, so the drain side decodes one self-describing entry format instead
  of special-casing three classes. `is_unit_stride` is what tells VecBeatExpander
  to coalesce rather than issue one access per entry, and what tells
  VecCrossLsuSnoop and VecStoreForward that this entry is a range.

  It also carries the destination group's base PRN and member count from
  `pvdest`/`v_emul`, the ownership fields `rob_idx` and `ldq_idx`/`stq_idx`, and
  `is_ff`, so the drain side, the LCB and the squash unit can each name the
  instruction from the entry alone — and so a fault raised against the entry
  arrives back here already knowing whether the fault-only-first split applies.

  On the store instance it additionally carries `us_data_base` — `resv_resp` SLOT
  1's base, verbatim. It is a SEPARATE POINTER and not an alias of the entry's own
  index: one range entry pairs with up to 8 data entries, so the two regions have
  different bases and different counts, and deriving the data base from the address
  base is the mis-generation this field exists to prevent.

  `eew` travels on the entry even though the ADDRESS arithmetic no longer needs it
  once `total_bytes` is known: the mask is ELEMENT-granular, so the drain side
  needs `eew` to scale a mask bit into a byte enable, and the LCB needs it to
  index elements for tracing and fault counting.

  ---- 3. Whole-register and mask accesses are ranges, not a second mechanism ----

  //@req-spec-agen.b6
  A whole-register access transfers `nfields * vLen / 8` bytes, fixed and
  INDEPENDENT OF VL; a mask access transfers `ceil(vl / 8)` bytes. Both are
  single contiguous ranges, so both are ordinary range entries under the length
  rule above — one nOP.v each, no separate generator, no separate queue, no
  separate drain path. Two consequences, stated because this is where a special
  case would otherwise creep in: a whole-register op transfers its full length
  even when `vl` is 0, and both classes are ARCHITECTURALLY UNMASKED, so their
  entry's mask is forced to all-ones and the byte-normalized `eew` cannot
  mis-scale a mask bit.

  ---- 4. The mask travels with the entry ----

  //@req-spec-agen.e13
  The mask is read ONCE per OP.v in stage 1, for every access class INCLUDING
  unit-stride, and it TRAVELS WITH THE RANGE ENTRY to the drain side rather than
  being re-read there. Mask handling is NOT skipped for unit-stride: this module
  copies the latched element-granular mask (all-ones when `vm` is set) into the
  entry, and VecBeatExpander applies it when forming each beat's byte enable.

  ===> WHOLE-REGISTER AND MASK ACCESSES MUST CARRY AN ALL-ONES MASK, not the latched
       one. Both are unmasked by definition, and both describe their transfer in
       BYTES (`eew` = 0, `len` = `emul * vLenBytes` or `ceil(vl/8)`), while the
       latched mask is indexed by vtype ELEMENTS. The drain side scales mask bits by
       the entry's `eew`, so at `eew` = 0 a vl-shaped mask gates byte b on element
       bit b and silently truncates the access to `vl` bytes. Measured on
       `ms11d_vl1r`: `vs1r.v` with `vl` = 4 emitted a single 4-byte beat against a
       32-byte range and then mask-skipped to the end, leaving 28 bytes of the
       destination unwritten. `vlm.v` escapes only by arithmetic accident, since
       `ceil(vl/8)` never exceeds `vl`.

  The
  cost the spec accepts is up to VLMAX mask bits on one bundle per OP.v — wide,
  but one bundle. The alternative, a drain-side `v0` read, would give `R1` a
  second concurrent reader (see the ports section): a port-table consequence, not
  a stylistic choice.

  The mask is NOT applied to the address here: a unit-stride range is
  contiguous whether or not lanes are masked off, and narrowing the range to
  the active lanes would break the range-overlap check below, which must be
  mask-OBLIVIOUS to stay conservative (memord.b20, VecCrossLsuSnoop).
  Suppression of masked-off BYTES happens where the access is formed.

  ---- 5. The store reads its entire source vPRN in one access ----

  //@req-spec-lsu.d5
  A unit-stride store reads its ENTIRE SOURCE vPRN IN ONE ACCESS — one whole-
  register capture per group member (`v_emul` captures, `nfields` for
  `vs1r`..`vs8r`, `last` on the final one), NEVER a per-element data request. Each
  capture lands as one full `vLen`-wide `st_US_DATA_Q` entry, and VecBeatExpander
  slices per-element data out of it at drain. The source group is `pvtmp` for an
  `is_shared` op and `pvs3` otherwise.

  THE CAPTURE SEQUENCE IS VecDgen's, and this module's `io.st_data` is the
  assertion-only cross-check on it (see the ports section). VecDgen derives the
  member sequence from its own request and its own `members_used` total-byte bound,
  owns the `pvtmp`/`pvs3` mux and VRF port `R3`, and pushes the entries; the
  requirement is discharged there, by that sequence, and here by the one range
  entry it pairs with plus the check that the two agree. There is exactly ONE
  mechanism, and it is not this one — what this module publishes is a member index
  and a `last` for comparison, and it chooses no port.

  Note the deliberate 1:N shape at this seam, which is easy to mis-review: ONE
  address entry in `st_US_ADDR_Q` pairs with `v_emul` data entries in
  `st_US_DATA_Q` (up to 8 at LMUL=8 — which is why `usQueueEntries` is 16, not 1).
  Both reservations were claimed at dispatch by VecQueueReservation, in both
  queues or not at all, so neither push can fail for lack of room. THAT 1:N SHAPE
  IS THE SAME FACT AS THE PER-SLOT RESERVATION: the address side got 1 entry at one
  base, the data side got `v_emul * v_seg_nf` entries at ANOTHER base, and slot 1's
  base is what this module copies into `us_data_base`. Read the two together — the
  1:N ratio is where a single shared base stops being merely inelegant and starts
  naming the wrong entries.

  ---- 6. The surplus release: one entry on the address side, MEMBERS on the data
          side ----

  The release fires ONCE per OP.v, in the push cycle — the cycle VL is known,
  because `VecScalarOperandRead` read it from the VL register file — with
  `used_count` PER QUEUE SLOT and the two slots carrying DIFFERENT NUMBERS:

    - SLOT 0 (address queue) = 1. A range entry is one entry by construction, so
      the address side has no surplus to return, for a store or a load alike.
    - SLOT 1 (data queue, store instance only) = `members_used`, the number of
      group members actually written: `ceil(total_bytes / vLenBytes)` from section
      2's `total_bytes`. The reservation claimed `v_emul * v_seg_nf` from EMUL at
      dispatch; EMUL is a rename-time worst case and `members_used` is the live
      figure, so the difference is genuine surplus.
    - Both slots ZERO for the zero-length case of section 7, whose whole region is
      surplus.

  `members_used` MUST BE THE SAME FORMULA VecDgen USES, off the same `total_bytes`,
  and for the same reason its own file gives: the hardcoded 8-member count was the
  M1 back-to-back store-data corruption, and a second derivation here that rounded
  differently would release entries VecDgen is still about to write. Derive it from
  total bytes, never from `v_emul`.

  ===> A SINGLE `used_count` HERE WOULD BE A SILENT DATA CORRUPTION, not a
  performance loss, and it is the reason the port is a `Vec(2, UInt)`. Trimming
  the DATA queue by the ADDRESS queue's count means releasing all but ONE entry
  of a region whose `v_emul` members are live store data — the freed entries are
  handed to the next store's reservation and overwritten, and the corruption
  surfaces on a LATER store than the one that caused it. That is the identical
  failure signature as the M1 phantom-member bug, reached from the opposite
  direction, and neither shows up in a test that stores and loads back one
  vector.
  A denied release is NOT retried: the surplus frees in order with the region at
  reclamation. `release_ok` is false whenever a younger OP.v has already reserved
  past this region, since the reservation permits tail-only trimming, and under a
  stream of vector stores that is the common case rather than the exception.

  ---- 7. Zero-length accesses push nothing ----

  When `total_bytes` computes to zero — `vl == 0`, a mask op with `vl == 0`, or a
  fully-masked-off access under `vta = 0` — this module pushes NO entry and drives
  no `io.st_data`, while still consuming `io.req` that cycle (it has no choice: the
  port carries no `ready`), so issue is never held. Such an op's destination group
  is completed without memory traffic
  by VecGroupCopy. A zero-length range pushed instead would leave the drain side
  to invent a completion for an access with no beats. (The whole-register classes
  are the exception noted above: their length does not depend on VL.)

  ---- 8. Base address, faults and `vstart` ----

  The base is used exactly as VecScalarOperandRead delivered it, with NO `vstart`
  offset applied, because a vector memory op that traps traps with `vstart = 0` and
  restarts WHOLE (plan section 5 rule 8). There is no partial-restart input and
  no resume path into this module. That covers every fault on this path with ONE
  exception, and the exception does not weaken the rule: a fault-only-first load
  faulting at element `i > 0` does not trap at all, so it never restarts and never
  needs a non-zero `vstart` either — section 9. No path in this module can produce
  a non-zero `vstart`.

  Alignment is likewise not this module's business: the entry states the true
  byte range whatever the base's alignment, and beat formation is the drain
  side's. Misaligned unit-stride is a v2 non-goal, so it must be detected
  downstream where beats are formed rather than by quietly rounding the base here
  — a rounded base would corrupt data instead of reporting a gap.

  ---- 9. Fault-only-first (`vleff`): a TRIM, not a trap ----

  //@req-spec-lsu.g1
  //@req-spec-lsu.g2
  //@req-spec-lsu.g3
  //@req-spec-lsu.g4
  //@req-spec-lsu.g9
  //@req-spec-lsu.g10
  A `vle<eew>ff.v` is a plain unit-stride load until it faults, and its fault is
  the one place a vector memory op does NOT simply trap. When `io.fault` arrives
  with `is_ff` set, the FAULTING ELEMENT INDEX SELECTS BETWEEN TWO ENTIRELY
  DIFFERENT MECHANISMS, and the whole of the policy is that split on
  `io.fault.bits.elem_idx === 0`:

  - ELEMENT 0: `io.fault_trap` ASSERTS. This is an ORDINARY PRECISE TRAP WITH
    `vstart = 0`, reported by VecLsu exactly as a scalar load fault would be —
    nothing about `vleff` softens the first element, because an instruction that
    loaded no element at all has nothing to report but the fault. `io.ff_trim`
    stays invalid.

  - ELEMENT `i > 0`: `io.fault_trap` STAYS LOW. THE INSTRUCTION MUST NOT TRAP —
    this is architectural, not a policy choice, and a trap here would be a wrong
    result for `strlen`-shaped code that deliberately reads past the end of a
    mapped page. Instead `io.ff_trim` is driven VALID WITH `i`: VL IS TRIMMED TO
    `i`, and the instruction COMPLETES WITH THE ELEMENTS IT DID LOAD. Those
    elements need no rescue — they were already pushed as beats off the retained
    range entry and land normally through the LCB — so "completes" here means the
    ordinary completion path runs, not a special one.

  ONLY THE ELEMENT-0 FAULT RAISES `rob_exception`. The `i > 0` case CLEARS NO
  ARCHITECTURAL STATE BEYOND TRIMMING VL: no `vstart` write, no `pvdest`
  reclamation, no replay, no squash, no pipeline flush, and nothing else on this
  interface moves. That is a strict statement and worth reading as one — the
  temptation is to "clean up" the elements at or above `i`, and there is nothing to
  clean up, because the tail policy the destination group already carries decides
  what those lanes hold and the trimmed VL is what tells every later reader where
  the data stops.

  `vleff` is A VL PRODUCER, so the trimmed count is not a side effect but the
  instruction's second result: it reaches the VL register file's `W_lsu` port
  through `lcb.io.vl_wb` on the group-done, VecLsu having converted this module's
  element index into the LCB's {member, `keep_bytes`} form on the way in. This
  module REPORTS the trim to the LCB and DOES NOT WRITE THE VL RF ITSELF — the two
  candidate producers for `W_lsu` were settled down to one at VecLsu level, for the
  reasons in the ports-section note. Dependents then pick the (possibly trimmed) VL
  up through the normal `pvl` wakeup, exactly as they would from a vset, and the
  architectural `vl` CSR follows at commit.

  `vleff` IS NOT SERIALIZED and carries no `is_unique`. It is an ordinary
  speculative vector load, and it is HOT — it is the load at the centre of every
  `strlen`/`memchr`-shaped loop, which is the entire reason the fault-only-first
  form exists — so serializing it would tax the one loop the feature was added to
  make fast. Accept nothing here that behaves like a fence.

  On a non-`vleff` unit-stride op (`is_ff` low) `io.ff_trim` is NEVER valid and
  `io.fault_trap` follows `io.fault.valid` unconditionally, with no element-index
  split — the plain precise trap of section 8. Assert that: a trim escaping onto
  a non-`vleff` load would silently shorten an architecturally full-length
  vector load, which no test that does not check VL would catch.

  ---- 10. Retirement, and why there is nothing to retire ----

  There is no `io.req.ready` to compute. `io.range.ready` is expected HIGH on every
  fire, because the single entry this instruction pushes sits in a region reserved
  at dispatch, in program order, that nobody else fills — so assert it on a fire
  rather than back-pressuring the accept, and let a violated reservation contract
  fail loudly instead of turning into an invented handshake. On the store path
  `io.st_data.ready` is tied high by VecLsu and is likewise not a gate. The
  instruction is finished with this module in the cycle it fires: no cursor, no
  member counter, no "current op" register, no completion tracking, no output an
  issue unit can see. Back-to-back unit-stride ops of one direction proceed one
  per cycle, and the two instances are fully independent because they share no
  port and no arbiter.

  ---- 11. Seam: one range-overlap check, not a per-element search ----

  Because the entry is a single contiguous byte range, disambiguation and
  forwarding for a unit-stride access are ONE RANGE-OVERLAP CHECK against each
  queue entry rather than a search per element (loadstore.rst `us-queue` and
  `mem-order`). VecCrossLsuSnoop owns the check and VecStoreForward the forward;
  what this module owes them is a range whose base and length are exact and whose
  `is_unit_stride` flag is set, so the range path is selected over the SSI
  per-element path. Those requirements are theirs (the memord family) and are
  deliberately not tagged here.

  ---- 12. Tracing ----

  One guarded `VecTrace` line per pushed entry — module, event, `rob_idx`, base,
  total byte length, `eew`, `v_emul`, active-mask population count — gated on the
  `vecTrace` plusarg and `!reset`, off by default. A second line records a
  refused zero-length request: "no entry appeared" is otherwise
  indistinguishable from a lost push, and with no unit tests anywhere in this
  project (plan section 5 rule 11) that distinction is only ever made from a
  trace. A third records a fault-only-first report — `rob_idx`, the faulting
  element index, and which arm was taken (trap at element 0, or trim to `i`) —
  because a `vleff` that trims is INDISTINGUISHABLE FROM ONE THAT DID NOT FAULT in
  every observable except the VL a later instruction reads, which is precisely the
  divergence a cosim run reports several instructions too late to explain.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: one range entry — one instruction — per cycle per instance, since an
instruction occupies the module for exactly the cycle it fires. Each instance
sustains that independently and neither direction can stall the other, which is
what target P4 (a `vle` and a `vse` overlapping, neither gating the other's
issue) measures.

Latency: combinational from `io.req` to the queue push, with at most one optional
output register. It must not add a pipeline stage — there is no multi-cycle work
to hide, and a stage would only delay the cycle in which the drain side can start
coalescing. The fault-only-first path is combinational too, and deliberately: it
is one comparison against zero and two output drives, off the address path
entirely, so it costs nothing on the push timing and adds no cycle between the
drain side's fault report and the trap-or-trim answer.

Structural constraint, not commentary: the entry count per instruction is ONE.
Access-count target P1 (`ceil(active_bytes / 8)` D$ accesses per unit-stride op,
the floor) is reachable only if the coalescing decision is made at drain, so an
implementation that emits more than one entry per unit-stride instruction has
failed this file regardless of what it measures.

Area: the mask field dominates the range entry — up to VLMAX bits (256 at the
defaults) per queue entry, times `usQueueEntries`. That is the price the spec
accepts for keeping the mask port to one reader, and it is affordable only
because the unit-stride queues are sized in instructions (16). Do not let this
entry format leak into the 512-entry SSI queues.
<|end_perf|>

<|begin_dependencies|>
MicroOp — the request carries `new MicroOp()`; this module reads `rob_idx`,
`ldq_idx`/`stq_idx`, `v_eew`, `v_emul`, `v_seg_nf`, `pvdest`, `pvm`, `pvs3`,
`pvtmp`, `is_shared`, `pvl`, the `v_is_*` class flags including `v_is_ff`, and
writes NONE of the nOP.v-scoped cursor fields
(`v_split_*`, the element cursor): a range entry is not an element access, so
those fields stay inert on this path.

VecBundles — `VecRangeEntry` is this module's OUTPUT CONTRACT, and the one
declaration whose field list must be checked from both sides (see the ports-
section note on `stride`, `is_unit_stride`, `mask` and the `len` width).

VectorParams — `vLen`, `eLen`, `maxMembers`, `vecPregSz`, and the VLMAX figure the
`vl` and mask widths derive from. VecTrace — the guarded trace helpers.

Instantiates NOTHING, and one absence is a decision: it does NOT instantiate
VecMaskStream. That module serializes `v0` at element granularity with the
one-entry lookahead VecElemAgen's hazard rule needs; a unit-stride access
consumes the whole latched mask at once and walks nothing, so streaming it here
would add a per-instruction cursor — exactly the state the invariant forbids.

Consumed by `ld_US_ADDR_Q` / `st_US_ADDR_Q` (VecElemQueue instances) directly;
VecBeatExpander, VecLoadCoalescingBuffer, VecCrossLsuSnoop, VecStoreForward and
VecSquashUnit all read the entry out of the queue rather than from this module.

VecDgen is a CHECKING counterparty, not a commanded one: it derives its own US
member sequence from its own request and `members_used`, and `io.st_data` exists
only so the two sides can be asserted to agree (ports section, logic section 5).
Confirm from that side that nothing in its data path reads this port.

VecLsu owns the fault-only-first routing and must be checked from that side too:
it qualifies the drain side's report onto `io.fault`, ties `io.st_data.ready`
high, turns `io.fault_trap` into `vec_xcpt` with `vstart = 0`, and converts
`io.ff_trim` into `lcb.io.trim`. VecLoadCoalescingBuffer then owns the single
`W_lsu` write via its `vl_wb` on the group-done — this module never touches
VlRegFile.
<|end_dependencies|>
