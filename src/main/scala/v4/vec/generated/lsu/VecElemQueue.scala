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

package boom.v4.vec.generated.lsu

import chisel3._
import chisel3.util._
import chisel3.layer

import org.chipsalliance.cde.config.Parameters

import boom.v4.common.BoomModule
import boom.v4.vec.generated.VecTrace
import boom.v4.vec.formal.{BoomSvaLayer, VecElemQueueChecks}

// GENERATED from src/main/nlhdl/vec/lsu/VecElemQueue.nlhdl.scala. Do not
// hand-edit; regenerate via the nlhdl gen-rtl flow instead.
class VecElemQueue(
  val queueName:      String,
  val entries:        Int,
  val width:          Int,
  val isStore:        Boolean,
  val hasXlatePass:   Boolean = false,
  val reserved:       Boolean = true,
  val ports:          Int = 1
)(implicit p: Parameters) extends BoomModule
{
  val readPorts: Int = ports + 1


  //@req-spec-lsu.b1
  //@req-spec-lsu.c2
  require(isPow2(entries), s"$queueName: entries ($entries) must be a power of two")

  //@req-spec-lsu.d1
  //@req-spec-lsu.d2
  //@req-spec-lsu.d3
  //@req-spec-lsu.d4
  require(width > 0, s"$queueName: width ($width) must be > 0")

  require(ports >= 1, s"$queueName: ports ($ports) must be >= 1")
  require(readPorts >= ports, s"$queueName: readPorts ($readPorts) must be >= ports ($ports)")
  require(!hasXlatePass || isStore, s"$queueName: hasXlatePass requires isStore")
  require(reserved || !isStore, s"$queueName: an unreserved store queue is the ssi-queues partial-fill deadlock configuration")

  val idxW = log2Ceil(entries)
  val ptrW = idxW + 1

  def phys(x: UInt): UInt = x(idxW - 1, 0)
  def wrapAdd(a: UInt, b: UInt): UInt = (a + b)(ptrW - 1, 0)
  def wrapSub(a: UInt, b: UInt): UInt = (a - b)(ptrW - 1, 0)

  class EnqBits extends Bundle {
    val idx  = UInt(ptrW.W)
    val data = UInt(width.W)
  }
  class UpdateBits extends Bundle {
    val idx  = UInt(ptrW.W)
    val data = UInt(width.W)
  }
  class RdPort extends Bundle {
    val req  = Input(Valid(UInt(ptrW.W)))
    val resp = Output(new Bundle {
      val data   = UInt(width.W)
      val filled = Bool()
    })
  }
  class FreeBits extends Bundle {
    val base    = UInt(ptrW.W)
    val entries = UInt(ptrW.W)
  }

  val io = IO(new Bundle {
    //@req-spec-lsu.b3
    val enq = Vec(ports, Flipped(Decoupled(new EnqBits)))

    val rd = Vec(readPorts, new RdPort)

    //@req-spec-lsu.k7
    val consume = Vec(ports, Input(Valid(UInt(ptrW.W))))

    val update = if (hasXlatePass) Some(Vec(ports, Input(Valid(new UpdateBits)))) else None

    val resv = new Bundle {
      val avail        = Output(UInt(ptrW.W))
      val tail         = Output(UInt(ptrW.W))
      val claim        = Input(Valid(UInt(ptrW.W)))
      val release_tail = Input(Valid(UInt(ptrW.W)))
      val free         = Input(Valid(new FreeBits))
    }

    val squash = Input(Valid(UInt(ptrW.W)))
    val empty  = Output(Bool())

    //@req-spec-memord.a22
    // Read-only view of the existing per-entry filled register, indexed by
    // phys(). The data gate the snoop owes VecStoreForward has to be answered
    // COMBINATIONALLY at presentation, which the registered rd port cannot do.
    val filled_vec = Output(UInt(entries.W))
  })

  //@req-spec-lsu.a5
  //@req-spec-lsu.j10
  val useMem = entries > 32
  val mem  = if (useMem)  Some(SyncReadMem(entries, UInt(width.W))) else None
  val regs = if (!useMem) Some(Reg(Vec(entries, UInt(width.W))))    else None

  val filled = RegInit(0.U(entries.W))
  val xlated = if (hasXlatePass) Some(RegInit(0.U(entries.W))) else None

  val head = RegInit(0.U(ptrW.W))
  val tail = RegInit(0.U(ptrW.W))

  def inRegion(idx: UInt): Bool = wrapSub(idx, head) < wrapSub(tail, head)

  def rangeMask(base: UInt, len: UInt): UInt =
    VecInit((0 until entries).map { i =>
      val cand0 = wrapSub(i.U(ptrW.W), base)
      val cand1 = wrapSub((i + entries).U(ptrW.W), base)
      (cand0 < len) || (cand1 < len)
    }).asUInt

  for (i <- 0 until ports) {
    io.enq(i).ready := inRegion(io.enq(i).bits.idx) && !filled(phys(io.enq(i).bits.idx))
  }

  val set_mask = io.enq.map(e => Mux(e.fire, UIntToOH(phys(e.bits.idx), entries), 0.U(entries.W))).reduce(_ | _)

  val update_mask = io.update match {
    case Some(u) => u.map(x => Mux(x.valid, UIntToOH(phys(x.bits.idx), entries), 0.U(entries.W))).reduce(_ | _)
    case None    => 0.U(entries.W)
  }

  //@req-spec-lsu.b6
  //@req-spec-lsu.b7
  //@req-spec-lsu.b8
  //@req-spec-lsu.b10
  val consume_mask =
    if (isStore) 0.U(entries.W)
    else io.consume.map(c => Mux(c.valid, UIntToOH(phys(c.bits), entries), 0.U(entries.W))).reduce(_ | _)

  val free_mask = Mux(io.resv.free.valid, rangeMask(io.resv.free.bits.base, io.resv.free.bits.entries), 0.U(entries.W))

  //@req-spec-lsu.i3
  val squash_len  = wrapSub(tail, io.squash.bits)
  val squash_mask = Mux(io.squash.valid, rangeMask(io.squash.bits, squash_len), 0.U(entries.W))

  val clear_mask = consume_mask | free_mask | squash_mask

  filled := (filled & ~clear_mask) | set_mask

  //@req-spec-lsu.b9
  //@req-spec-lsu.d8
  //@req-spec-lsu.d11
  xlated.foreach { x => x := (x & ~clear_mask) | update_mask }

  val next_head = Mux(io.resv.free.valid, wrapAdd(head, io.resv.free.bits.entries), head)

  val tail_after_release = Mux(io.resv.release_tail.valid, io.resv.release_tail.bits, tail)
  val tail_after_claim   = Mux(io.resv.claim.valid, wrapAdd(tail_after_release, io.resv.claim.bits), tail_after_release)
  val next_tail          = Mux(io.squash.valid, io.squash.bits, tail_after_claim)

  head := next_head
  tail := next_tail

  val avail_reg = RegInit(entries.U(ptrW.W))
  avail_reg := wrapSub(entries.U(ptrW.W), wrapSub(next_tail, next_head))
  io.resv.avail := avail_reg
  io.resv.tail  := tail
  io.empty      := head === tail
  //@req-spec-memord.a22
  io.filled_vec := filled

  for (r <- 0 until readPorts) {
    val idxPhys = phys(io.rd(r).req.bits)
    io.rd(r).resp.filled := RegNext(Mux(io.rd(r).req.valid, filled(idxPhys), false.B), false.B)
  }

  if (useMem) {
    val m = mem.get
    for (r <- 0 until readPorts) {
      io.rd(r).resp.data := m.read(phys(io.rd(r).req.bits), io.rd(r).req.valid)
    }
    for (i <- 0 until ports) {
      when (io.enq(i).fire) { m.write(phys(io.enq(i).bits.idx), io.enq(i).bits.data) }
    }
    io.update.foreach { u => for (x <- u) { when (x.valid) { m.write(phys(x.bits.idx), x.bits.data) } } }
  } else {
    val rg = regs.get
    for (r <- 0 until readPorts) {
      io.rd(r).resp.data := RegNext(rg(phys(io.rd(r).req.bits)))
    }
    for (i <- 0 until ports) {
      when (io.enq(i).fire) { rg(phys(io.enq(i).bits.idx)) := io.enq(i).bits.data }
    }
    io.update.foreach { u => for (x <- u) { when (x.valid) { rg(phys(x.bits.idx)) := x.bits.data } } }
  }

  for (i <- 0 until ports) {
    assert(!(io.enq(i).valid && !inRegion(io.enq(i).bits.idx)),
      s"$queueName: enq idx outside occupied region")
    assert(!(io.enq(i).fire && filled(phys(io.enq(i).bits.idx))),
      s"$queueName: fill targeted an already-filled entry")
  }
  assert(!(io.resv.claim.valid && io.resv.claim.bits > avail_reg),
    s"$queueName: claim exceeds avail")
  assert(!(io.resv.free.valid && io.resv.free.bits.base =/= head),
    s"$queueName: free base does not equal head")
  assert(!(io.resv.release_tail.valid && wrapSub(tail, io.resv.release_tail.bits) > wrapSub(tail, head)),
    s"$queueName: release_tail would move tail before head")
  assert(!(io.squash.valid && squash_len > wrapSub(tail, head)),
    s"$queueName: squash would move tail before head")

  val occ = wrapSub(next_tail, next_head)

  when (io.resv.claim.valid) {
    VecTrace.traceStruct(queueName, "claim", Seq(
      ("entries_param", entries.U), ("is_store", isStore.B.asUInt),
      ("claim_entries", io.resv.claim.bits), ("occ", occ)))
  }
  for (i <- 0 until ports) {
    when (io.enq(i).fire) {
      VecTrace.traceStruct(queueName, "fill", Seq(
        ("entries_param", entries.U), ("is_store", isStore.B.asUInt),
        ("port", i.U), ("idx", io.enq(i).bits.idx), ("occ", occ)))
    }
    when (io.consume(i).valid) {
      VecTrace.traceStruct(queueName, "consume", Seq(
        ("entries_param", entries.U), ("is_store", isStore.B.asUInt),
        ("port", i.U), ("idx", io.consume(i).bits), ("occ", occ)))
    }
  }
  io.update.foreach { u =>
    for (i <- 0 until ports) {
      when (u(i).valid) {
        VecTrace.traceStruct(queueName, "update", Seq(
          ("entries_param", entries.U),
          ("port", i.U), ("idx", u(i).bits.idx), ("occ", occ)))
      }
    }
  }
  when (io.resv.free.valid) {
    VecTrace.traceStruct(queueName, "free", Seq(
      ("entries_param", entries.U), ("is_store", isStore.B.asUInt),
      ("base", io.resv.free.bits.base), ("free_entries", io.resv.free.bits.entries), ("occ", occ)))
  }
  when (io.squash.valid) {
    VecTrace.traceStruct(queueName, "squash", Seq(
      ("entries_param", entries.U), ("is_store", isStore.B.asUInt),
      ("new_tail", io.squash.bits), ("occ", occ)))
  }

  //@formal-anchor VecElemQueueChecks
  layer.block(BoomSvaLayer) {
    VecElemQueueChecks(
      enqValid       = io.enq(0).valid,
      enqInRegion    = inRegion(io.enq(0).bits.idx),
      claimValid     = io.resv.claim.valid,
      claimEntries   = io.resv.claim.bits,
      avail          = avail_reg,
      freeValid      = io.resv.free.valid,
      freeBase       = io.resv.free.bits.base,
      qHead          = head,
      updateValid    = io.update.map(_(0).valid).getOrElse(false.B),
      updateInRegion = io.update.map(u => inRegion(u(0).bits.idx)).getOrElse(false.B),
      rdReqValid     = io.rd(0).req.valid,
      rdFilled       = io.rd(0).resp.filled,
      squashValid    = io.squash.valid,
      squashLen      = wrapSub(tail, io.squash.bits),
      occRegion      = wrapSub(tail, head),
      consumeValid   = io.consume(0).valid,
      consumeInRegion = inRegion(io.consume(0).bits)
    )
  }
}
