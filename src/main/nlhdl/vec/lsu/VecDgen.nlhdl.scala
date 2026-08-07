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
  VecDgen — the vector store DATA generator: reads store data out of the VRF at
  EXECUTE time and writes it into the store data queues.

  hierarchy.yaml: kind: module, mode: new,
  output src/main/scala/v4/vec/generated/lsu/VecDgen.scala,
  package boom.v4.vec.generated.lsu, group vec_lsu.
  depends_on MicroOp, VecBundles, VectorParams, VecTrace.

  ONE instance, `dgen`, inside VecLsu. There is no load counterpart and no
  `isStore` parameter — a load has no data to generate. This is the v2 name for
  `st_vdgen`; it runs alongside the store-side stage-1 AGENs `st_elem_agen`
  (VecElemAgen, isStore) and `st_range_agen` (VecRangeAgen, isStore), which are
  jointly what `execution.rst` calls `st_vagen_1`.

  ===> BUG 1 — BACK-TO-BACK STORE DATA CORRUPTION. THIS MODULE COMPLETES BY TOTAL
       BYTES, `vl << eew`, HANDLING A PARTIAL FINAL MEMBER. M1 hardcoded
       `num_members = 8` and streamed phantom members after a 1-member store.
       Logic paragraph 4 is the most load-bearing paragraph in this file.

  ===> BUG 2 — THE SHARED-OP OPERAND is `pvtmp`, not `pvs3`, when `is_shared`:
       `dgen_operand := Mux(uop.is_shared, uop.pvtmp, uop.pvs3)`. See paragraph 2.

  ===> NO `busy`. Beyond a streaming cursor that retires on its own last push, this
       module holds no state scoped to "the current instruction" and exports no
       `busy` of any kind (the vec_lsu invariant, plan rule 6). `addvector`'s
       `agen_active` had to OR in `vec_dgen.active`; nothing here may reproduce it.

  Governing spec anchors: execution.rst `vector-dgen`, loadstore.rst
  `store-data-queue` and `vec-queue-reservation`, midcore.rst `vrf-ports`,
  issue.rst `shared-store-chain`.
*/

<|begin_module|>

  <|begin_parameters|>
  No constructor parameters of its own. Sizes come from VectorParams via
  `HasVectorParams` and no width below is a literal: `vLen` (256) for the VRF read
  and the unit-stride entry width, `eLen` (64) for the SSI entry width, `vecPregSz`
  for a member PRN, `maxMembers` (8) for the member index range, `vecVLSz` for a VL
  value, and `vLenBytes = vLen / 8` as the member size in bytes (derive it, never
  write 32). The module is elaborated only when `usingRVV` is true — VecLsu
  instantiates it inside that gate, so a vectors-off build emits no instance and no
  ports rather than a tied-off one (invariant 1), and the gate is `usingRVV`, not
  rocket's `usingVector`.
  <|end_parameters|>

  <|begin_ports|>
  Clock and reset follow the Chisel default and the map's `defaults:` block:
  posedge `core_clk`, ACTIVE-HIGH SYNCHRONOUS `core_reset`, both implicit. No
  second clock and no `reset_n` — the active-low inversion for the SV coprocessor
  happens once, in `tt_cii_host_wrap`, never here.

  `io.req` — Flipped Decoupled, the DGEN grant for one vector store: the OP.v's
  `MicroOp` (for `is_shared`, `pvs3`, `pvtmp`, `v_eew`, `v_seg_nf`, `v_emul`,
  `rob_idx`, `stq_idx`, `br_mask`), the resolved `vl` as `UInt(vecVLSz.W)`, and the
  base index of this store's reservation in its data queue. VL and the scalar
  feeders are read once per direction by `st_opnd` (VecScalarOperandRead) and arrive
  as values; DGEN issues no VL-RF, INT-RF or FP-RF read of its own.

  `io.cursor` — Flipped Decoupled, the mask-derived element cursor the store AGEN
  produces: one beat per surviving element (per element/field for a segment),
  carrying the element index, the segment field index, `eew`, the ORDINAL of that
  element within the store's reservation region, and `last`. DGEN consumes this
  stream and never generates one.

  `io.vrf_r3` — the VRF read on port `R3`: a request carrying one member PRN, and a
  `UInt(vLen.W)` response one registered cycle later, with the read-during-write
  forwarding VecRegFile guarantees. No second VRF port: `R4` (store mask/index)
  belongs to the store AGEN, and DGEN reading it is exactly the mistake the shared
  cursor exists to prevent.

  `io.ssi_data_enq` — Decoupled write into `st_SSI_DATA_Q`: `eLen` of data, a byte
  enable, the ordinal, `rob_idx`/`stq_idx`, `last`.
  `io.us_data_enq` — Decoupled write into `st_US_DATA_Q`: a full `vLen` member
  value, its member index, a valid-byte count for a partial final member, the same
  ownership fields, `last`. Exactly one of the two is active for a given store,
  chosen by access class.

  `io.brupdate`, `io.rob_flush` — from `vec_pipeline_io`, to abandon an in-flight
  stream. Queue recovery is VecSquashUnit's pointer rollback, not this module's.

  ===> THERE IS NO `io.busy`, NO `io.active`, NO `io.grp_active`. If any of those
       appears in the generated port list the generation is wrong, whatever it
       measures. `io.req.ready` is a stream handshake on the store slot's DGEN
       path only — see logic paragraph 10.
  <|end_ports|>

  <|begin_logic|>

  ---- 1. Where it sits ----

  //@req-spec-agen.d1
  DGEN runs ALONGSIDE the store-side stage-1 AGEN, not before or after it: the
  address and data paths of one vector store are two independently granted streams
  over the same element cursor, running together for a plain store and separated by
  the whole coprocessor round trip for a segmented one. Nothing here may assume they
  share a cycle, or that the AGEN is still running when DGEN starts.

  ---- 2. The data operand — the shared-op mux ----

  //@req-spec-issue.d9
  The group DGEN reads is selected by `is_shared`:

  // dgen_operand := Mux(uop.is_shared, uop.pvtmp, uop.pvs3)   // M1 BUG 2

  For a SHARED (segmented) store the operand is the `pvtmp` rendezvous group,
  because the coprocessor half transposed the segment fields into `pvtmp` and step
  6 of the six-step chain in `issue.rst` `shared-store-chain` is exactly "LSU half
  DGEN reads pvtmp as store data" — the LAST step. `pvs3` there names the
  untransposed architectural source, and gating on it deadlocks the chain because
  the group-done that would release it never arrives. VecStoreDgenPath carries the
  identical mux to gate the DGEN grant and to exclude `pvs3`'s busy bit when
  `is_shared` is set; emit both from one shared helper rather than writing it twice.

  DGEN is otherwise indifferent to which group it got: it treats the operand as a
  flat byte array, member k supplying bytes `[k*vLenBytes, (k+1)*vLenBytes)`, and
  whether that byte order came from a plain register group or from the
  coprocessor's transpose is invisible. That indifference is why segmented stores
  need no separate data path.

  ---- 3. The VRF read ----

  //@req-spec-agen.d2
  //@req-spec-vrf.g7
  Source vector operands are read from the vector register file on read port `R3`,
  the Store Unit's store-data port in the canonical `vrf-ports` partition of
  `midcore.rst`. R3 is the only VRF port this module touches and it adds none — the
  partition is canonical and nothing adds a port without amending `midcore.rst` and
  the map. The read address for member k is `dgen_operand(k)`, a member PRN out of
  the selected group vector, never a base plus offset, because a group's members
  need not be contiguous. Scalar source operands come from the FP/INT register
  file, read by `st_opnd` and delivered as values — DGEN adds no scalar read port,
  so the store direction keeps exactly one INT/FP reader.

  //@req-spec-lsu.d7
  The read happens at DGEN, i.e. at EXECUTE time when the DGEN path is granted —
  not at commit and not at drain — and the data then sits in the data queue until
  the post-commit drain, leaving the store's memory write a pure queue drain with
  no register-file access left to do.

  //@req-spec-agen.d4
  For an SSI store the queue entry is `eLen` wide while the register is `vLen` wide,
  so DGEN reads and BUFFERS THE ENTIRE vPRN read value in a member staging register
  and slices elements out of it, one per enqueue; one VRF read serves up to
  `vLenBytes >> eew` elements. Double-buffer that register (a `cur`/`next` pair,
  prefetching member k+1 while streaming member k) so a member boundary costs no
  bubble — with one buffer a SEW=8 stream stalls on R3 every 32 elements.

  ---- 4. COMPLETION BY TOTAL BYTES — the M1 corruption bug ----

  //@req-spec-agen.d15
  The stream is complete when the BYTES PUSHED EQUAL THE TOTAL BYTES OF THE
  ACCESS, `total_bytes = (vl * nf) << eew` — `vl << eew` non-segmented, with `nf`
  the segment field count (1 when not segmented). Members actually touched are
  `members_used = ceil(total_bytes / vLenBytes)`, and the LAST member is PARTIAL
  whenever `total_bytes` is not a multiple of `vLenBytes`: it contributes
  `total_bytes - (members_used - 1) * vLenBytes` bytes, and its enqueue carries
  that count (as the byte enable on the SSI path, as the valid-byte count on the
  unit-stride path) so the drain writes no byte the instruction did not write.

  // ===> M1 BUG 1, DO NOT REINTRODUCE. num_members was hardcoded 8, so after a
  //      1-member store DGEN kept streaming PHANTOM members: entries with no
  //      matching address ordinal, which (a) consumed the next store's reserved
  //      capacity and shifted every later ordinal, corrupting a back-to-back
  //      store's data, and (b) never drained, because the address side had no
  //      counterpart — so the store never completed and the machine stalled.
  //      Found by accident via a scalar load-back, because Whisper does not deeply
  //      compare vector store DATA (plan section 9). `v_emul` is ALSO wrong: it is
  //      the EMUL-derived worst-case group size fixed at rename, while the live
  //      data extent depends on VL, known only at execute.

  With no mask active the cursor advances CONSECUTIVELY — one bundle per
  consecutive element of the vPRN, ordinal j holding element j — so byte count and
  ordinal count agree trivially; with a mask active they still agree, because both
  come from the cursor. Assert at end of stream that pushed bytes equal
  `total_bytes` and that the final push was the cursor's `last`. A mismatch IS the
  phantom-member failure and must fail loudly rather than corrupt a neighbour's
  queue region silently.

  ---- 5. It must NOT re-derive the mask ----

  //@req-spec-agen.d11
  //@req-spec-agen.d12
  DGEN MUST NOT RE-DERIVE WHICH ELEMENTS SURVIVE MASKING. It consumes the SAME
  mask-derived element cursor as the store AGEN, which is the single owner of which
  elements survive and which already read `v0` once on `R4` and latched it. DGEN
  therefore has no mask input, no `v0` read and no mask comparator at all: the
  absence of the port is the enforcement mechanism, not a convention a reviewer has
  to check.

  //@req-spec-agen.d13
  //@req-spec-agen.d14
  So a masked-off element is skipped in BOTH streams and the address and data
  bundles stay paired element-for-element. A second evaluation of the mask — even a
  correct-looking one, even the same expression — is how the two streams drift: any
  disagreement about one element's survival shifts every later ordinal on one side
  only, and every subsequent element then writes the wrong address. Consuming the
  cursor makes the pairing structural.

  ---- 6. Pairing, ordering and atomicity ----

  //@req-spec-agen.d5
  //@req-spec-agen.d10
  Each effective address is paired with the data of the element the store AGEN
  selected, BY ORDINAL WITHIN THE RESERVATION rather than by same-cycle arrival:
  ordinal j of the store's region in `st_SSI_ADDR_Q` and ordinal j of its region
  in `st_SSI_DATA_Q` are the same nOP.v. DGEN writes `reservation_base + ordinal`,
  taking the ordinal from the cursor beat, so the data half may be written
  arbitrarily long after the address half. Data within a vPRN is already packed,
  so there is no repacking — slice and go.

  // INTERPRETATION, deliberate, and a reviewer should know it. execution.rst reads
  // as though st_vdgen emits ONE bundle holding both the address and the 64-bit
  // data. Taken literally that is not implementable together with issue.rst step
  // 6: for a segmented store the address exists hundreds of cycles before the
  // data, so a joint same-cycle push would force DGEN to buffer the whole address
  // stream — unbounded per-instruction state, banned by the vec_lsu invariant. The
  // pair is therefore formed POSITIONALLY: the AGEN writes the address half at
  // ordinal j, DGEN writes the data half at ordinal j, the drain reads them as one
  // nOP.v. DGEN never computes an address.

  //@req-spec-agen.d6
  //@req-spec-agen.d8
  Bundles reach the store queues IN ORDER in two senses that must not be conflated.
  Within one store DGEN emits ordinals strictly ascending, never out of order and
  never sparsely. Across stores, program order is a property of the RESERVATION —
  capacity was claimed at dispatch in program order in BOTH queues, or the store did
  not dispatch — so an older segmented store writing its data long after a younger
  store wrote its own cannot invert queue order. Each nOP.v is therefore processed
  in program and memory order at the drain, which is what precise exception tracking
  via VSTART rests on. That is also what the shared case buys: between the AGEN
  grant and the DGEN grant DGEN holds NOTHING for the store, so a delay of hundreds
  of cycles costs no state here. Precise exceptions need no mid-stream state either
  — a faulting vector store traps with `vstart = 0` and restarts whole (invariant
  5), so DGEN writes no `vstart` and carries no `fault_elem`.

  //@req-spec-agen.d7
  Enqueues are ATOMIC at two granularities. An entry is written whole in one cycle —
  data, byte enable and ownership fields together — so the drain never sees a
  half-written entry; being a single-cycle positional write, that is structural
  rather than a protocol rule. At instruction granularity DGEN marks the final data
  bundle `last` (derived from the total-byte count of paragraph 4, never from a
  member count), and a store's drain may not begin until that marker has landed, so
  no store is drained with a partial element set. DGEN keeps no record of the store
  after that push.

  ---- 7. Element extraction ----

  For an SSI element the slice is taken from the staged member by BYTE position:

  // byte_pos   = (el_idx * nf + field_idx) << eew   // ELEMENTS -> BYTES
  // member     = byte_pos >> log2(vLenBytes)
  // off_in_mbr = byte_pos & (vLenBytes - 1)
  // data       = staged(member)[off_in_mbr*8 +: (8 << eew)]

  // ===> THE `<< eew` IS LOAD-BEARING. An M1 bug used the cursor's element offset
  //      directly as a byte offset (it is in ELEMENTS, not bytes), silently
  //      corrupting element 0 of every access with a non-zero element offset.
  //      Convert once, here, and assert `member < members_used`.

  The value is right-justified in the `eLen` entry with a byte enable of `1 << eew`
  bytes. Extraction is one combinational `vLen`-to-`eLen` byte-granular select off a
  registered buffer; keep it combinational so the stream sustains one element/cycle.

  ---- 8. The unit-stride path ----

  For a unit-stride store there is one range entry on the address side and the data
  entry is a full `vLen`, so DGEN pushes ONE WHOLE MEMBER VALUE PER ENQUEUE, in
  ascending member order, for `members_used` members — the same total-byte bound,
  the partial final member carrying its valid-byte count. It slices no elements at
  all: the stage-2 Packer (VecBeatExpander, store side) slices per-element data out
  of the `vLen` entry at drain, which is what makes just-in-time coalescing
  possible. The ordinal here indexes members, and the single range entry pairs with
  the whole member sequence.

  ---- 9. Operand lifetime — why the source vPRN needs NO PIN ----

  //@req-spec-lsu.d9
  //@req-spec-lsu.d10
  Because the data is captured this early — at execute, into the queue — the source
  vPRN needs NO PIN, and DGEN exports no pin request, reference count or hold
  signal toward the free list. The group frees with the rest of the stale group at
  commit, on the ordinary path. The lifetime argument is non-obvious and worth
  stating plainly: a later writer to the same architectural vector register cannot
  clobber this store's data, because that data no longer lives in the register file
  — it already lives in the data queue, and the queue entry is what the drain
  reads. A pinning mechanism here would hold PRNs out of a 96-entry file for the
  whole pre-commit lifetime of every vector store, for no correctness gain.

  ---- 10. Readiness, kill, and what must not exist ----

  `io.req.ready` falls only while a data stream is in flight, and it is a handshake
  on the store slot's DGEN path, not a machine-wide gate. Its duration is bounded
  by this store's own remaining ordinals and nothing else: it may NEVER depend on a
  D$ or TLB response, on commit, on the drain, on the load direction, or on another
  instruction. That bound is what distinguishes it from M1's
  `io.busy := grp_active || (state =/= sIdle)`, which gated `fu_ready` for every
  vector memory op in both directions. The store slot must HOLD its DGEN grant
  until accepted rather than dropping it — a silently dropped store grant is the
  other M1 failure this file's neighbours were split up to prevent.

  On `brupdate` kill of the streaming uop or on `rob_flush`, ABANDON the stream
  immediately: stop reading R3, push nothing further, be ready next cycle. Leave no
  residue — VecSquashUnit recovers the queue region by rolling the tail pointer
  back to the youngest surviving reservation, and a partially written region is
  harmless because it is never drained. When a stream retires, every piece of DGEN
  state (staging registers, byte counter, member index, ordinal) is dead; nothing
  outlives the last push and no signal derived from it leaves the module.

  ---- 11. Tracing ----

  There are no unit tests in this project (invariant 9), so the only debug surface is
  the shared VecTrace convention: `vecTrace`-plusarg gated, off by default, one line
  per key event, tagged with the module name and `rob_idx`. Emit on request accept
  (`is_shared`, the selected operand's member 0, `vl`, `eew`, `total_bytes`,
  `members_used`), each R3 member read, each enqueue (ordinal, byte enable, `last`),
  and stream abandon. The `total_bytes`/`members_used` line is what makes the
  phantom-member bug visible in a trace rather than only in a corrupted neighbour,
  so it is not optional.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Constraints on the implementation, not aspirations:

- SSI stores: ONE element data bundle per cycle, sustained across member boundaries,
  matching the store AGEN's one address per cycle so the data path is not the store
  path's bottleneck. This forces the double-buffered member staging of logic
  paragraph 3 — one buffer inserts an R3 bubble every `vLenBytes >> eew` elements,
  i.e. one stall per 32 elements at SEW=8.
- Unit-stride stores: one `vLen` member per cycle, so an EMUL=8 group is queued in 8.
- R3 latency is one registered cycle and the first enqueue is 2 cycles after the
  grant, but nothing downstream may assume a fixed TOTAL latency: stream length is
  data-dependent through VL and the mask.
- The element slice is the only wide mux and is not expected to be critical at
  1 GHz; if it becomes so, pipeline the enqueue rather than lowering the rate.

There is no throughput coupling to the load direction at all — DGEN shares no port,
no arbiter and no state with the load path, which is the point of cutting the vector
LSU by pipeline position instead of by direction.
<|end_perf|>

<|begin_dependencies|>
Declared `depends_on:` — MicroOp (the request's uop fields), VecBundles (the
cursor beat, the data-queue write bundles, and the queue-set enumeration naming
`st_SSI_DATA_Q` and `st_US_DATA_Q`), VectorParams (`vLen`, `eLen`, `vecPregSz`,
`maxMembers`, `vecVLSz`), VecTrace (the guarded trace helpers).

Instantiates nothing; it is a leaf of the vec_lsu tree. Its seams, each of which
Phase R must check from the other side:
- VecElemAgen (`st_elem_agen`) — produces the element cursor DGEN consumes and owns
  the `R4` mask read. The cursor must carry the ORDINAL, not only the element index,
  or the two sides must agree on a count instead of sharing one.
- VecRangeAgen (`st_range_agen`) — US counterpart: one range entry against DGEN's
  member sequence.
- VecElemQueue instances `st_SSI_DATA_Q` (`eLen` wide) and `st_US_DATA_Q` (`vLen`
  wide) — the enqueue targets. DGEN writes at `reservation_base + ordinal`, so the
  queue must support a reservation-relative positional write, not only append.
- VecQueueReservation — supplies the data-queue base; a store reserves in both
  queues or does not dispatch. VecRegFile — read port `R3` only.
- VecStoreDgenPath — grants DGEN separately from AGEN and must use the SAME
  `is_shared` operand mux; one shared helper, not two copies.
- VecScalarOperandRead (`st_opnd`) — supplies `vl` and the scalar feeders.
- VecBeatExpander (store side) — slices per-element data out of a US `vLen` entry.
- VecSquashUnit — recovers queue regions after a kill; DGEN only abandons.
<|end_dependencies|>
