//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Load Coalescing Buffer (LCB) -- Step 11a.2
//------------------------------------------------------------------------------
//
// The LCB is the response-side datapath for a unit-stride vector load. VecLSU
// hands it one PLACED beat at a time -- the 64b dcache response plus where its
// valid bytes land in the renamed destination register group (member PRN +
// byte offset). The LCB turns that into one VecRegFile lane write and, on the
// last beat of the group, emits a single `VecGroupDone` carrying the group's
// member PRNs (which clears the vector busy bits in the ROB / rename / issue).
//
// M1 unit-stride placement: DMEM_WIDTH is 64b, so one beat carries at most 8
// valid bytes -- exactly one 64b VRF lane for an aligned base. The placement is
// written here in a byte-general form (shift the valid bytes into position,
// derive a per-lane write mask) so a beat that lands wholly inside one lane
// writes exactly that lane and leaves the other three undisturbed. Partial-lane
// tails (vl not lane-aligned) and straddling (misaligned base) are correctness
// refinements deferred past the first vle gate; the e2e test uses lane-aligned
// configs.
//
// No accumulation register is needed under the serial single-outstanding model:
// each lane of a member is filled by exactly one beat, in element order, so the
// write can be issued per beat. The latched group descriptor only feeds
// `group_done`.

package boom.v4.vec.lsu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters

import boom.v4.common._
import boom.v4.vec.rename.{VecGroupDone, VecEmul}

class VecLoadCoalescingBuffer(implicit p: Parameters) extends BoomModule with VecLsConstants
{
  val nLanes = vecVLen / 64

  val io = IO(new Bundle {
    // latch the destination group descriptor at the start of a group
    val start = Flipped(Valid(new Bundle {
      val prn  = Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W))
      val mask = UInt(VecEmul.MAX_MEMBERS.W)
    }))
    // placed beats (64b response + where it goes); driven by VecLSU. vecMemWidth
    // beats/cycle (2 under dual-dynamic): one per vector dcache pipe that returned.
    val beat = Vec(vecMemWidth, Flipped(Valid(new Bundle {
      val data     = UInt(coreDataBits.W)                       // 64b dcache response
      val pdst     = UInt(vecPregSz.W)                          // dest member PRN
      val dst_byte = UInt(log2Ceil(VLEN_BYTES + 1).W)           // byte offset within the 256b member
      val src_off  = UInt(6.W)                                  // source byte offset within the 64b beat
      val nbytes   = UInt(log2Ceil(VLEN_BYTES + 1).W)           // number of valid bytes
      val is_fake  = Bool()                                     // all-masked / el_count==0 / bypass: write nothing
      val last     = Bool()                                     // last beat of the group
    })))
    val kill = Input(Bool())
    // VecRegFile write ports (per-byte write enable). vecMemWidth ports: two beats
    // to the SAME member PRN are merged onto port 0 (their valid bytes are disjoint,
    // so OR is exact); two beats to DISTINCT members use both ports. This keeps the
    // regfile's "no two writes to the same address" invariant.
    val vrf_write = Vec(vecMemWidth, Valid(new Bundle {
      val addr = UInt(vecPregSz.W)
      val data = UInt(vecVLen.W)
      val mask = UInt((vecVLen / 8).W)
    }))
    // group-done pulse on the last beat
    val group_done = Valid(new VecGroupDone)
  })

  // ---- latched group descriptor (feeds group_done only) ----
  val grp_prn  = Reg(Vec(VecEmul.MAX_MEMBERS, UInt(vecPregSz.W)))
  val grp_mask = RegInit(0.U(VecEmul.MAX_MEMBERS.W))
  when (io.start.valid) {
    grp_prn  := io.start.bits.prn
    grp_mask := io.start.bits.mask
  }

  // ---- per-beat lane placement (one entry per beat port) ----
  // valid bytes shifted to the LSB, then up to their destination byte offset. The
  // per-byte write mask (nbytes bytes starting at dst_byte) is passed straight to
  // the VecRegFile so a sub-lane beat (mask load, partial tail) writes exactly its
  // bytes and leaves the rest of the lane undisturbed.
  val real   = VecInit(io.beat.map(bt => bt.valid && !bt.bits.is_fake && (bt.bits.nbytes =/= 0.U)))
  val placed = VecInit(io.beat.map { bt =>
    val sa = bt.bits.data >> (bt.bits.src_off << 3)
    (sa << (bt.bits.dst_byte << 3))(vecVLen - 1, 0)
  })
  val bmask  = VecInit(io.beat.map(bt =>
    (((1.U << bt.bits.nbytes) - 1.U) << bt.bits.dst_byte)(vecVLen / 8 - 1, 0)))

  if (vecMemWidth == 1) {
    io.vrf_write(0).valid     := real(0) && !io.kill
    io.vrf_write(0).bits.addr := io.beat(0).bits.pdst
    io.vrf_write(0).bits.data := placed(0)
    io.vrf_write(0).bits.mask := bmask(0)
  } else {
    // Two beats. Merge onto port 0 when they hit the same member (or when only one
    // is real); use port 1 only for a distinct-member second beat.
    val same = real(0) && real(1) && (io.beat(0).bits.pdst === io.beat(1).bits.pdst)
    val one_fold = same || !real(0)   // port 0 also carries beat 1's bytes
    io.vrf_write(0).valid     := (real(0) || real(1)) && !io.kill
    io.vrf_write(0).bits.addr := Mux(real(0), io.beat(0).bits.pdst, io.beat(1).bits.pdst)
    io.vrf_write(0).bits.data := (Mux(real(0), placed(0), 0.U) | Mux(one_fold, placed(1), 0.U))
    io.vrf_write(0).bits.mask := (Mux(real(0), bmask(0),  0.U) | Mux(one_fold, bmask(1),  0.U))
    io.vrf_write(1).valid     := real(0) && real(1) && !same && !io.kill
    io.vrf_write(1).bits.addr := io.beat(1).bits.pdst
    io.vrf_write(1).bits.data := placed(1)
    io.vrf_write(1).bits.mask := bmask(1)
  }

  // ---- group-done on the last beat (unused under the completion-counter model) ----
  io.group_done.valid     := io.beat.map(bt => bt.valid && bt.bits.last).reduce(_ || _) && !io.kill
  io.group_done.bits.prn  := grp_prn
  io.group_done.bits.mask := grp_mask
}
