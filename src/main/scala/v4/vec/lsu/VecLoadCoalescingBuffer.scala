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
    // one placed beat (64b response + where it goes); driven by VecLSU
    val beat = Flipped(Valid(new Bundle {
      val data     = UInt(coreDataBits.W)                       // 64b dcache response
      val pdst     = UInt(vecPregSz.W)                          // dest member PRN
      val dst_byte = UInt(log2Ceil(VLEN_BYTES + 1).W)           // byte offset within the 256b member
      val src_off  = UInt(6.W)                                  // source byte offset within the 64b beat
      val nbytes   = UInt(log2Ceil(VLEN_BYTES + 1).W)           // number of valid bytes
      val is_fake  = Bool()                                     // all-masked / el_count==0 / bypass: write nothing
      val last     = Bool()                                     // last beat of the group
    }))
    val kill = Input(Bool())
    // VecRegFile write port (lane-masked)
    val vrf_write = Valid(new Bundle {
      val addr = UInt(vecPregSz.W)
      val data = UInt(vecVLen.W)
      val mask = UInt(nLanes.W)
    })
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

  // ---- per-beat lane placement ----
  val b      = io.beat.bits
  val real   = io.beat.valid && !b.is_fake && (b.nbytes =/= 0.U)

  // valid bytes shifted to the LSB, then up to their destination byte offset.
  val src_aligned = b.data >> (b.src_off << 3)
  val placed      = (src_aligned << (b.dst_byte << 3))(vecVLen - 1, 0)
  // byte-granular write mask, collapsed to one bit per 64b lane.
  val byte_mask   = (((1.U << b.nbytes) - 1.U) << b.dst_byte)(vecVLen / 8 - 1, 0)
  val lane_mask   = VecInit((0 until nLanes).map(l => byte_mask(8 * l + 7, 8 * l) =/= 0.U)).asUInt

  io.vrf_write.valid     := real && !io.kill
  io.vrf_write.bits.addr := b.pdst
  io.vrf_write.bits.data := placed
  io.vrf_write.bits.mask := lane_mask

  // ---- group-done on the last beat ----
  io.group_done.valid     := io.beat.valid && b.last && !io.kill
  io.group_done.bits.prn  := grp_prn
  io.group_done.bits.mask := grp_mask
}
