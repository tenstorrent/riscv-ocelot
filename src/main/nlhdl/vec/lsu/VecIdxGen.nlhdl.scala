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
  VecIdxGen — serializes one index vector register group into per-element UNSIGNED
  (zero-extended) byte offsets for an indexed (gather/scatter) access.
*/

hierarchy.yaml: kind: module, mode: new,
output src/main/scala/v4/vec/generated/lsu/VecIdxGen.scala,
package boom.v4.vec.generated.lsu, group vec_lsu.
depends_on VecBundles, VectorParams, VecTrace. Instantiates nothing.
Instantiated as `idx` inside VecElemAgen — twice in the machine, once in
`ld_elem_agen` and once in `st_elem_agen`, never shared between them. It is the
index half of what execution.rst calls the Walker; the element walk itself,
the segment packing and the address add all live in the parent.

===> ONE-ENTRY LOOKAHEAD IS A CORRECTNESS RULE, NOT AN OPTIMIZATION. This
     module presents the NEXT element's offset ready-ahead and names the case
     where it cannot. The parent agen must neither start nor emit an element
     access whose index entry is not yet staged — it stalls instead. The
     inherited Walker's rule was "to avoid stall/wait states it won't release
     the final segment or start until the next index arrives", and that rule is
     carried over from bring-up rather than derived from the architecture: an
     agen that walked ahead of the index stream emitted an address computed
     from a stale or undriven staging register, which is a wrong address rather
     than a slow one.

===> OFFSETS ARE UNSIGNED, AND THE INDEX EEW IS `uop.v_idx_eew`. Two
     corrections, both against earlier revisions of this file, the map and the
     plan, and both are the kind that produce plausible addresses rather than
     obvious breakage:

     (1) AN INDEX ELEMENT IS AN UNSIGNED BYTE OFFSET FROM THE SCALAR BASE, so
     widening it from its EEW to the address width is a ZERO-extension. RVV 1.0
     indexed offsets are unsigned — spike's `VI_LDST_GET_INDEX` reads them as
     `uint8_t`/`uint16_t`/`uint32_t` — and Whisper, the cosim reference this
     design is validated against, agrees. A sign-extension diverges on EVERY
     index whose top EEW bit is set, and the divergence presents as an LSU
     ADDRESSING bug rather than as an extension bug, which is the mis-diagnosis
     that costs days. Earlier text here said "signed"; that was a spec defect,
     corrected in `hierarchy.yaml` and the plan as well. The extension stays in
     ONE named function (logic section) so a future correction is a one-line
     change.

     (2) THE EEW THAT GOVERNS THE INDEX STREAM IS `uop.v_idx_eew`, NOT
     `uop.v_eew` and not `vtype.vsew`. For an indexed access the two uop fields
     are INDEPENDENT: `v_eew` is the DATA element width (how wide each loaded or
     stored element is, and therefore how destination bytes are placed) while
     `v_idx_eew` is the INDEX element width (how wide each entry of the index
     vector is). This file predates that split and said `v_eew`; sourcing the
     descriptor from `v_eew` walks the index vector at the WRONG STRIDE, reading
     the wrong offsets from the right register group. Everything derived below —
     elements per register, how many group members are read, where an element
     sits inside a member, the extension arm — is derived from the INDEX EEW for
     that reason, and `v_eew` appears nowhere in this module.

Governing spec anchors: execution.rst `vector-agen` ("Walker", and the
paragraph naming the two vector reads that feed the generators),
midcore.rst `vrf-ports` (the canonical, 0-based port assignment),
caracal-milestone-plan-v2.md Phase E (step E3) and ground rules 6 and 11.

<|begin_module|>

  <|begin_parameters|>
  `isStore` is a Scala `Boolean`, default `false`, forwarded by the parent
  VecElemAgen from its own `isStore`. It selects nothing about the walk — the
  serialization is byte-for-byte identical in both directions, which is
  execution.rst's "the generator is selected by access class, not by direction" —
  and it selects exactly one thing: which VRF read port this instance's index
  read is wired to.

  //@req-spec-vrf.g2
  //@req-spec-agen.c27
  `vrfReadPort` is DERIVED, not chosen: `if (isStore) 4 else 0`. On the load path
  `ld_vAGEN_1` reads the index vector on `R0`, which is the Load Unit / LCB's
  index port in the canonical 0-based table in midcore.rst `vrf-ports`; the
  mask it reads separately on `R1`, which belongs to `VecMaskStream` — instantiated
  at VecLsu level as `ld_msk`, not a sibling inside this agen — and is not touched
  here. The number is an elaboration-time Scala `Int` so that
  it appears as a constant port index at the connection site in VecLsu and can be
  grepped; nothing in this module may add a VRF port, and a second index port for
  either direction would be an amendment to that table and to hierarchy.yaml
  before it is a line of RTL.

  //@req-spec-agen.c29
  On the store path `st_vagen_1` reads BOTH the mask and the index through `R4`,
  so `vrfReadPort` is 4 for the store instance and that one port has two jobs.
  (The store MASK read itself is VecMaskStream's obligation and its requirement
  ID is deliberately not cited here — this module owns only the index half of
  `R4`.) This works, and the reason is a cadence rather than an
  arbitration: the mask is read once per OP.v as a single VLEN-wide read of `v0`
  and latched, while index members are read as the walk advances, so the two
  reads never need the same cycle. The consequence for this module is only that
  its read request must be able to WAIT a cycle — see the ports section's grant
  line — and never that it may assume the port is free.

  ===> THE 2:1 `R4` MUX IS `VecLsu`'S, NOT THIS MODULE'S AND NOT THE PARENT
       AGEN'S. `VecRegFile` publishes NO read `ready`, so the hold-off cannot
       live there, and `R4` must reach it as exactly one request. VecLsu
       resolves it with static priority — MASK WINS, INDEX WAITS — using
       `VecMaskStream.owns_port` as the hold-off; the grant arrives here as the
       `ready` on `io.vrf_read`, routed through VecElemAgen. On the load path
       `R0` has exactly one reader and that `ready` is a constant true.

  Sizes are taken from VectorParams (`vLen`, `eLen`, `maxMembers`, `vecPregSz`,
  `vecVLSz`) and from `HasBoomCoreParameters` (`xLen`, `robAddrSz`). No width in
  this file is a literal.

  `stageDepth` is fixed at 2 and is not a knob: two VLEN-wide member staging
  buffers, ping-ponged. One buffer would insert a wait state at every member
  boundary, which is precisely the "stall/wait state" the lookahead rule exists
  to avoid; more than two buys nothing, because the narrowest member still holds
  `vLen / eLen` = 4 elements at the widest index EEW and one VRF read has a
  single cycle of latency to hide.

  There is no `usingRVV` parameter. This module is elaborated only from inside
  VecPipeline, which exists only when `usingRVV` is set, so a vectors-off build
  contains no instance of it at all — absent, not tied off (plan ground rule 1).

  Elaboration-time requires: `eLen <= xLen`, so that widening an index element to
  the address width is always a widening (a zero-extension) and never a
  truncation; and `vLen % eLen == 0`, so that elements-per-member is a power of two
  for every legal index EEW and the member/offset split below is a shift rather
  than a divide.

  THAT REQUIRE BOUNDS THE LEGAL EEWs, NOT THE ELABORATED ONES, so the extension must
  narrow to `xLen` EXPLICITLY rather than trusting it. `idx_eew` is a runtime 2-bit
  field, so the 4-way mux structurally elaborates a 64-bit arm on every config,
  including one where `xLen < 64` — and Chisel's `pad` is a zero-extend that never
  truncates, so that arm would hand the `UInt(xLen.W)` offset port a wider value and
  `:=` would cut it silently. Zero-extend AND narrow: `x.pad(xLen)(xLen-1, 0)`.
  Reaching that arm requires an index EEW above ELEN, which RVV reserves, so its
  value is a don't-care and the narrowing costs nothing; making the width provable
  for every config is the point. If decode does not already raise illegal-instruction
  on an index EEW above ELEN that is `VLSDecode`'s gap to close, not this module's —
  this module must not be the place that silently defines a reserved encoding.
  (Not live today: `xLen == 64` in every config here.)
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset are Chisel's implicit pair, matching the hierarchy.yaml
  defaults: posedge `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`. No other
  clock, no asynchronous reset, no latch.

  ---- Descriptor input: `io.start`, a `DecoupledIO` input ----

  One hand-off per indexed OP.v, carrying only what the walk needs:
    - `pvs2`  — `Vec(maxMembers, UInt(vecPregSz.W))`, the renamed index group.
                A group's members need not be contiguous, so this is the member
                PRN vector and never a base-plus-count.
    - `idx_eew` — 2 bits, the INDEX element width (0..3 for 8/16/32/64), taken
                from the uop's `v_idx_eew` — NOT from `v_eew`, which is the DATA
                width, and not from `vconfig.vsew`. The parent asserts
                `v_is_indexed` on every `start`, so a class that has no index
                vector can never present a descriptor this field cannot describe.
    - `vl`    — `vecVLSz` bits, the element count to serialize, read from the VL
                register file at execute by the parent.
    - `rob_idx` — `robAddrSz` bits, carried for tracing only.

  `io.start.ready` is high whenever the cursor is idle, and also in the cycle the
  cursor retires, so back-to-back indexed ops cost no idle cycle. This ready is a
  FILL-SIDE HAND-OFF INSIDE VecElemAgen AND MUST NOT LEAVE IT. It is not a
  `busy`, it does not reach an issue queue, and routing it to one would be a
  failed review under the vector-LSU invariant regardless of what it measured.

  No `MicroOp` crosses this boundary, deliberately: `rob_idx` is passed as a bare
  field so that this node needs no `depends_on: MicroOp`, matching its
  hierarchy.yaml entry.

  ---- VRF index read: `io.vrf_read` ----

  Request out, statically bound to port `vrfReadPort`: `valid`, `prn`
  (`UInt(vecPregSz.W)`), and a `ready` input. On the load path `ready` is a
  constant true — `R0` has exactly one reader. On the store path it is `VecLsu`'s
  2:1 `R4` mux (routed in through VecElemAgen), which gives `R4` to the
  once-per-OP.v mask read in preference to an index read; `VecRegFile` itself
  provides no read `ready`, which is precisely why the mux and this line exist.

  Response in: `valid` plus `data` (`UInt(vLen.W)`), one whole member. THE READ IS
  A REGISTERED ONE-CYCLE PORT: the flop is in `VecRegFile`, one per read port, so
  the response arrives the cycle after a GRANTED request. `VecRegFileBank`'s
  "0-cycle, unpipelined" array read is BANK-INTERNAL and sits inside that one
  cycle — this module must not register the payload a second time, or the port's
  observable latency becomes 2 and the double-buffer's coverage argument (one
  cycle of latency to hide) is wrong. The response is nevertheless carried as a
  separate `valid` rather than as a fixed-latency assumption baked into a shift
  counter, so a denied request cannot be mistaken for a returning one and the
  latency stays a property of VecRegFile instead of being duplicated here where
  the two could drift.

  ---- The index interface: `io.idx`, output ----

  //@req-spec-agen.c17
  This is the "index interface" from which execution.rst's Walker takes a
  per-element byte offset. It presents, for the element the parent is about to
  work on:
    - `valid`    — this element's offset is staged and may be used;
    - `offset`   — `UInt(xLen.W)`, the UNSIGNED byte offset, already
                   ZERO-extended from the index EEW (see the logic section). It
                   is a `UInt` and not an `SInt` deliberately: the type is the
                   documentation of the unsigned rule, and an `SInt` here invites
                   a consumer to re-extend or to compare signed. `VecElemAgen`
                   forms `base + offset` at `xLen` and must not re-extend.
    - `elem_idx` — `vecVLSz` bits, WHICH element this offset belongs to;
    - `last`     — this is element `vl - 1`;
    - `next_valid` — the FOLLOWING element's offset is staged too (the lookahead);
    - `stall`    — the single named term the parent gates on.
  `elem_idx` is exported so the parent compares it against its own element cursor
  rather than assuming the two advance in lockstep; a mismatch is the assertion
  that catches a lookahead bug in one cycle instead of as a wrong address.
  The other half of that spec sentence — the per-element MASK BIT — is served by
  `VecMaskStream` (`ld_msk`/`st_msk`, at VecLsu level) and paired with this offset
  in VecElemAgen. It is
  deliberately not re-exported here: two sources for one mask bit is how the
  inherited load and store Packers came to disagree about masking.

  `io.idx.taken`, input: a one-cycle pulse from the parent in the cycle it
  consumes the presented element (emits its access, or retires it as masked-off).
  It advances the cursor. Consuming a masked-off element still pulses `taken` —
  the index stream advances per ELEMENT, not per emitted access, because the
  index vector has an entry for a masked-off element too.

  ---- `io.kill`, input ----

  A single Bool from the parent, the OR of the walking uop's branch kill and
  `rob_flush`. It drops the cursor, both staging buffers and any outstanding
  read, returning to idle in one cycle. Nothing here participates in squash
  bookkeeping beyond that: pointer rollback on the queues is VecSquashUnit's.
  <|end_ports|>

  <|begin_logic|>
  ---- State: the whole of it ----

  A latched descriptor (`pvs2`, `idx_eew`, `vl`, `rob_idx`); an element cursor
  `elem_ptr` (`vecVLSz` bits, the next element to present); two member staging
  buffers `mbr_data(0..1)` of `vLen` bits each with per-buffer `valid` and
  `mbr_num` tags; a `rd_mbr` pointer naming the next member to fetch; and one
  `rd_outstanding` bit. That is the complete state list, and it is bounded by the
  serialization cursor plus its staging.

  ---- Where an element lives, and the total-bytes rule ----

  //@req-spec-agen.c17
  `elemsPerMbr = vLen / (8 << idx_eew)` — 32, 16, 8 or 4 at `vLen = 256`. Every
  value is a power of two, so the split of an element index into
  {member number, position within the member} is a right shift and a mask, with
  the shift amount selected by a 4-way mux on `idx_eew`. No divider.

  The number of members actually read is derived from TOTAL BYTES,
  `ceil((vl << idx_eew) / (vLen/8))`, and NOT from a member count handed down
  from rename. Two things follow, and both are deliberate:
  (1) A partial final member is the normal case, not a special case: the walk
      simply stops at element vl-1 wherever inside a member that falls.
  (2) Only the members that contain a live index are ever read. A hardcoded
      8-member walk is the back-to-back vector-store corruption bug from the
      M1 log — it streamed phantom members after a 1-member op and then
      stalled. Deriving the count from total bytes is what makes that
      unrepresentable rather than merely unlikely.

  ---- Zero extension: ONE named function, used once ----

  `extendIndex(member_bits, idx_eew): UInt(xLen.W)` selects the element's
  `8 << idx_eew` bits out of the staged member and ZERO-extends them to `xLen`.
  It is a 4-way mux, one arm per legal EEW, and at `idx_eew = 3` with
  `eLen == xLen` the extension is the identity.

  The extension is UNSIGNED, and this is the ONE place in the design that
  decides it. RVV 1.0 indexed offsets are unsigned: spike's
  `VI_LDST_GET_INDEX` reads them as `uint8_t`/`uint16_t`/`uint32_t`, and
  Whisper — the cosim reference — matches spike. A SIGN-extension would
  diverge on every index whose top `idx_eew` bit is set (any `uint8_t` index
  >= 0x80, which a stride-16 gather over a 4 KiB buffer hits immediately), and
  it would present as a wrong ADDRESS, i.e. as an LSU bug, not as an extension
  bug. Earlier revisions of this file, of hierarchy.yaml and of the plan said
  "signed"; all three are corrected, and this comment is the record so the
  next reader does not "fix" it back.
  
  Keeping it in this one named function is what makes the choice reviewable
  and ONE-LINE-REVISABLE: a Whisper divergence on an indexed access localises
  to exactly this mux. Do NOT open-code the extension at the address adder in
  the parent — two extension sites is two chances to get the EEW arm wrong,
  and the narrow EEWs are the ones tests exercise least. `VecElemAgen`'s own
  spec states the matching obligation from its side: the offset arrives
  already zero-extended and must not be re-extended.
  
  The arms are selected by `idx_eew` (= `uop.v_idx_eew`), never by `v_eew`. A
  4-way mux driven from the DATA width extends the wrong number of bits out of
  the right member, which is the same class of silent-wrong-address bug as
  walking at the wrong stride.

  The sum with the scalar base is formed in the parent, at `xLen` width, as an
  UNSIGNED add, so that wraparound matches the ISA before the result is narrowed
  to `vaddrBitsExtended` for translation. This module never sees the base and never
  produces an address.

  ---- Fetch: read ahead, one outstanding ----

  Issue a read for member `rd_mbr` when a staging buffer is free, `rd_mbr` is
  within the derived member count, and no read is outstanding. `prn` is
  `pvs2(rd_mbr)`. Hold `valid` until `ready`, because on the store path `R4` may
  be busy with the mask read; a request that dropped itself when not granted
  would lose a member and the walk would present a stale offset for every element
  of it. One read outstanding at a time — there is one port, and the double
  buffer is what covers its latency, not multiple concurrent reads.

  A response is written into the buffer that requested it and clears
  `rd_outstanding`. A response arriving with no read outstanding, or in the cycle
  after `io.kill`, is DROPPED — the kill path must not leave a buffer holding a
  member of a squashed op's index group, since those PRNs are recycled by the
  free list immediately.

  Prefetch is what makes the lookahead hold in steady state: the read of member
  m+1 is issued as soon as its buffer is free, which is the cycle the last
  element of member m-1 is taken, so the member boundary costs no wait state at
  any legal EEW.

  ---- Present, and the lookahead the parent depends on ----

  //@req-spec-agen.c17
  `io.idx.valid` is high when the buffer tagged with `elem_ptr`'s member is
  valid; `io.idx.offset` is `extendIndex` applied to that buffer at `elem_ptr`'s
  position; `io.idx.elem_idx` is `elem_ptr`; `io.idx.last` is
  `elem_ptr === vl - 1`.

  `io.idx.next_valid` is high when the buffer holding element `elem_ptr + 1` is
  also valid — and is FORCED HIGH when `io.idx.last` is set. That forcing is not
  a corner-case tidy-up: without it the final element of every indexed access
  waits forever for an index entry that does not exist, and the instruction never
  completes. It is the exact place the inherited rule "won't release the final
  segment ... until the next index arrives" must stop applying.

  `io.idx.stall` is `!valid || !next_valid`, computed here and exported as one
  signal so the parent gates on a name instead of re-deriving the hazard. The
  parent's obligation, stated from this side so both sides are reviewable: with
  `io.idx.stall` high it must neither START an element access nor RELEASE the
  final segment of the element in flight. It stalls; it does not emit an address
  built from an invalid staging register.

  ---- Advance and retire ----

  On `io.idx.taken` (which the parent may only assert while `io.idx.valid` is
  high — assert this), `elem_ptr` increments and a buffer whose last element has
  been taken is invalidated, freeing it for the next prefetch. When the taken
  element was `vl - 1` the cursor RETIRES IN THAT SAME CYCLE: the descriptor is
  dropped, both buffers are invalidated and `io.start.ready` is high again
  combinationally, so the next indexed OP.v hands off with no dead cycle.

  A descriptor with `vl = 0` retires in its accepting cycle and issues NO VRF
  read at all — not one member, on either port. A zero-length walk that read a
  member would occupy `R4` for a store that has no elements, and a VL = 0 op does
  not come here anyway: it is VecGroupCopy's path. The rule exists so a spurious
  start cannot hang the port.

  ---- What this module deliberately does not have ----

  No `busy` output of any kind, and no state scoped to the instruction beyond the
  cursor and its staging, which retire together at element `vl - 1`. It never
  waits on a D$ response, a translation, a queue credit or a completion, so it
  cannot gate the next instruction — the vector-LSU invariant, restated where it
  is enforced. Its only back-pressure output is `io.start.ready`, consumed inside
  VecElemAgen. It also holds no VRF port beyond the cycles it has a read in
  flight, which is what lets one `R4` serve the store mask and the store index.

  ---- Tracing ----

  Guarded `VecTrace` lines, one per key event, tagged with this module's name and
  the descriptor's `rob_idx`, gated on the `vecTrace` plusarg and off by default
  (plan ground rule 11 — there are no unit tests, so this is the only observation
  surface): `start` (`idx_eew`, vl, derived member count), `read` (member, prn, port
  number), `stage` (member, buffer), `present` (elem_idx, offset), `stall`
  (elem_idx, and which of valid/next_valid was low — that distinction is the whole
  diagnosis), `retire` and `kill`. The stall line is emitted only on the first
  cycle of a stall run, so a long stall does not bury the trace.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Throughput: ONE element offset presented per cycle, sustained, with no wait
state at a member boundary at any legal index EEW. This is a constraint on the
implementation, and it is what forces the two staging buffers and the
prefetch-as-soon-as-free policy rather than a single buffer refilled on demand.
The parent cannot exceed one element per cycle, and the drain side's target is
one D$ access per cycle while the arbiter grants (plan target P2), so a
serialization rate below one element per cycle would cap the whole indexed path
here.

Latency: a descriptor accepted in cycle N can present element 0 in cycle N+2 at
the earliest — one cycle to issue the member read, one for the VRF response. That
start-up cost is paid once per OP.v and is off the steady-state path.

Area: two `vLen`-wide staging registers (512 bits at `vLen = 256`) plus the
cursor and tags, per instance. No divider and no multiplier — every
EEW-dependent quantity is a shift or a 4-way mux, because `elemsPerMbr` is a
power of two for every legal EEW.

Ports: exactly one VRF read port per instance, `R0` for the load instance and
`R4` for the store instance, and on the store path that port is shared with the
mask read and must be released whenever a read is not in flight.
<|end_perf|>

<|begin_dependencies|>
VectorParams — `vLen`, `eLen`, `maxMembers`, `vecPregSz`, `vecVLSz`. Every width
derives from it; none is a literal.

VecBundles — the shared bundle conventions this seam is written in, and the
`VecElemAccess` the parent ultimately fills with the address derived from
`io.idx.offset`. The descriptor and index-interface bundles above are local to
the VecElemAgen-to-VecIdxGen seam; if the design-wide review decides every
cross-node bundle belongs in one place, they move to VecBundles unchanged.

VecTrace — the guarded trace helpers. This module passes a bare `rob_idx` rather
than a `MicroOp`, since it deliberately does not depend on MicroOp.

Instantiates nothing. Its parent is VecElemAgen (instance `idx`), which owns the
element walk, the segment packing, the address add, and the mask pairing — the
mask itself now comes from `VecMaskStream` at VecLsu level, not from a sibling
inside the agen — and which forwards its own `isStore` to this module's parameter
and its uop's `v_idx_eew` (never `v_eew`) into `io.start.idx_eew`.
The VRF read request/response is wired out through VecElemAgen and VecLsu to
VecRegFile's read port `vrfReadPort`; VecRegFile is not a `depends_on:` of this
node because the connection is made by the container, not by a type binding here.
Two obligations that live on the far side of this seam and are checked from both:
VecLsu owns the 2:1 `R4` mux that drives this module's read `ready` (mask wins,
index waits), and VecElemAgen consumes `io.idx.offset` as an ALREADY
ZERO-EXTENDED `UInt` and must not re-extend it.
<|end_dependencies|>
