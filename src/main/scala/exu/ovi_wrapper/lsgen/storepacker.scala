// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._

class StorePacker(override val VLEN: Int, override val DMEM_WIDTH: Int)
extends Module with VecLSGenConstants {
  // ======== Parameters ========
  val CTR_WIDTH = (((1<<SEG_ENC_W)-1)+EL_ID_W+((1<<EMUL_ENC_W)-1))

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH))
    // store data interface
    val vdb_data = new Bundle {
      val read_bytes  = Output(UInt(log2Ceil(DMEM_WIDTH/8).W)) // like a ready signal
      val valid_bytes = Input(UInt(log2Ceil(DMEM_WIDTH/8).W))  // like a valid signal
      val data        = Input(UInt(DMEM_WIDTH.W))
    }
    val kill = Input(Bool())
    // store packet
    val store_packet = Flipped(DecoupledIO(new StorePacket(VLEN, DMEM_WIDTH)))
  })

  // ======== Definitions ========
  
  object State extends ChiselEnum {
    val IDLE, PACKING = Value
  }

  // ======== Config info ========

  val state      = RegInit(State.IDLE)
  val sb_id      = io.start.bits.sb_id
  val vl         = io.start.bits.vl
  val eew_enc    = io.start.bits.eew_enc
  val emul_enc   = io.start.bits.emul_enc
  val stride_dir = io.start.bits.stride_dir
  val seg_enc    = io.start.bits.seg_enc
  val base_addr  = io.start.bits.base_addr

  // ======== Packing info ========
  val EEW_CTR  = RegInit(0.U(CTR_WIDTH.W)) // single EEW wide slice counter
  val dmem_off = RegInit(0.U(DMEM_ENC.W))  // offset tracking for mem alignment
  val dmem_max = RegInit(0.U(DMEM_ENC.W))  // max elements to fit in DMEM

  // mask offs and widths (to read CTR fields: <msb> [GROUP_ID, EL_ID, SEG_ID] <lsb>)
  val seg_mask_off      = 0.U
  val seg_mask_width    = seg_enc
  val el_mask_off       = seg_mask_off + seg_mask_width    // remember this is the mask for el_id. nothing to do with masked stores
  val el_mask_width     = log2Ceil(VLEN_BYTES).U - eew_enc // eq to log2Ceil(VLEN_BYTES >> eew_enc)
  val v_group_mask_off  = el_mask_off + el_mask_width
  val v_group_mask_width = emul_enc

  // ======== Packing Constraints ========

  // elements that can be stored contiguously depending on mem alignment
  val dmem_constraint = PriorityEncoderOH(dmem_off | ~(dmem_max-1.U))

  // elements until VL is reached in CTR
  val vl_constraint   = (
    (vl << el_mask_off) |                               // EMUL_FIELD/EL_ID_FIELD: vl
    (((1.U << (seg_mask_width)) - 1.U) << seg_mask_off) // SEG_ID_FIELD: last seg_id (all 1s)
  ) - EEW_CTR                                           // take distance of value from current ctr

  // ======== Advance CTR based on constraints ========
  val ctr_inc_val = WireInit(0.U(CTR_WIDTH.W))
  when (dmem_constraint <= vl_constraint) {
    ctr_inc_val := Reverse(PriorityEncoderOH(Reverse(dmem_constraint)))
  }.elsewhen (vl_constraint <= dmem_constraint) {
    ctr_inc_val := Reverse(PriorityEncoderOH(Reverse(vl_constraint)))
  }

  val ctr_inc_enc = PriorityEncoder(ctr_inc_val)

  // check constraints hit (NOTE: multiple can be triggered due to "less OR eq")
  val dmem_constraint_met = (ctr_inc_val === dmem_constraint)
  val vl_constraint_met   = (ctr_inc_val === vl_constraint)

  // ======== Outputs ========

  // ready-valid signals
  val vdb_valid           = (io.vdb_data.valid_bytes =/= 0.U)
  io.start.ready         := (state === State.IDLE)
  io.store_packet.valid  := (state === State.PACKING) && vdb_valid
  val vdb_ready           = (state === State.PACKING) && (io.store_packet.ready)
  io.vdb_data.read_bytes := Mux(vdb_ready, (1.U << (ctr_inc_enc + eew_enc)), 0.U)

  // store packet
  io.store_packet.bits.addr     := base_addr + (Mux(stride_dir, -EEW_CTR, EEW_CTR) << eew_enc)
  io.store_packet.bits.data     := io.vdb_data.data
  io.store_packet.bits.mem_size := (ctr_inc_enc + eew_enc)
  io.store_packet.bits.sb_id    := sb_id
  io.store_packet.bits.is_fake  := false.B
  io.store_packet.bits.misaligned := false.B
  io.store_packet.bits.last     := (state === State.PACKING) && (vl_constraint_met)

  // ======== State Machine ========

  switch (state) {
    is (State.IDLE) {
      when (io.start.fire) {
        // -- CTR reset --
        state := State.PACKING
        EEW_CTR := 0.U

        // -- Init mem alignment --
        val high_off = (((1<<(ADDR_BREAK))-1).U - base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from end of DMEM (high) to base_addr
        val low_off  = (base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from start of DMEM (low) to base_addr
        dmem_off := Mux(stride_dir, high_off, low_off)
        dmem_max := (DMEM_BYTES.U >> eew_enc)
      }
    }
    is (State.PACKING) {
      when (io.kill) {
        state := State.IDLE
      } .elsewhen (io.store_packet.fire) {

        // -- Increment counter --
        EEW_CTR := EEW_CTR + ctr_inc_val

        // -- Wrap increment dmem_off --
        when (dmem_constraint_met) {
          dmem_off := 0.U
        }.otherwise {
          dmem_off := dmem_off + ctr_inc_val
        }

        // -- Last packet transition --
        when (io.store_packet.bits.last) {
          state := State.IDLE
        }

      }
    }
  }

}
