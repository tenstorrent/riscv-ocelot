// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._

class StorePacker(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends Module with VecLSGenConstants {
  // ======== Parameters ========
  val CTR_WIDTH = (((1<<SEG_ENC_W)-1)+EL_ID_W+((1<<EMUL_ENC_W)-1))

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = Flipped(DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH)))
    val use_seg_constraint = Input(Bool())
    // store data interface
    val vdb_data = new Bundle {
      val read_bytes  = Output(UInt(VDB_R_SIZE_BYTES.W)) // like a ready signal
      val read_all    = Output(Bool())
      val valid_bytes = Input(UInt(VDB_R_SIZE_BYTES.W))  // like a valid signal
      val data        = Input(UInt(DMEM_WIDTH.W))
    }
    val kill = Input(Bool())
    // store packet
    val store_packet = DecoupledIO(new StorePacket(VLEN, DMEM_WIDTH))
    // status signal
    val gen_active = Output(Bool())
  })

  // ======== Definitions ========
  
  object State extends ChiselEnum {
    val IDLE, VSTART_HANDLING, PACKING = Value
  }

  // ======== Config info ========

  val state      = RegInit(State.IDLE)
  val sb_id      = io.start.bits.sb_id
  val vl         = io.start.bits.vl
  val vstart     = io.start.bits.vstart
  val eew_enc    = io.start.bits.eew_enc
  val emul_enc   = io.start.bits.emul_enc
  val stride_dir = io.start.bits.stride_dir
  val seg_enc    = io.start.bits.seg_enc
  val base_addr  = io.start.bits.base_addr
  val use_seg_constraint = io.use_seg_constraint

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

  // elements that can be provided from VDB
  val vdb_constraint = io.vdb_data.valid_bytes >> eew_enc

  // constraint to the next element id
  val seg_constraint = (
    (EEW_CTR & ~((1.U << el_mask_off)-1.U)) +
    (1.U << el_mask_off)
  ) - EEW_CTR

  // elements until VL is reached in CTR
  val vl_constraint   = (
    ((vl - 1.U) << el_mask_off) +                     // EMUL_FIELD/EL_ID_FIELD: vl
    (((1.U << seg_mask_width) - 1.U) << seg_mask_off) // SEG_ID_FIELD: last seg_id (all 1s)
  ) + 1.U - EEW_CTR                                   // take distance of value from current ctr

  // ======== Advance CTR based on constraints ========
  val ctr_inc_val = WireInit(0.U(CTR_WIDTH.W))
  when ((dmem_constraint <= vl_constraint) && (dmem_constraint <= vdb_constraint) && ((dmem_constraint <= seg_constraint) || !use_seg_constraint)) {
    ctr_inc_val := Reverse(PriorityEncoderOH(Reverse(dmem_constraint)))
  }.elsewhen ((vdb_constraint <= dmem_constraint) && (vdb_constraint <= vl_constraint) && ((vdb_constraint <= seg_constraint) || !use_seg_constraint)) {
    ctr_inc_val := Reverse(PriorityEncoderOH(Reverse(vdb_constraint)))
  }.elsewhen ((seg_constraint <= dmem_constraint) && (seg_constraint <= vdb_constraint) && ((seg_constraint <= vl_constraint) && use_seg_constraint)) {
    ctr_inc_val := Reverse(PriorityEncoderOH(Reverse(seg_constraint)))
  }.elsewhen ((vl_constraint <= dmem_constraint) && (vl_constraint <= vdb_constraint) && ((vl_constraint <= seg_constraint) || !use_seg_constraint)) {
    ctr_inc_val := Reverse(PriorityEncoderOH(Reverse(vl_constraint)))
  }

  val ctr_inc_enc = PriorityEncoder(ctr_inc_val)

  // check constraints hit (NOTE: multiple can be triggered due to "less OR eq")
  val dmem_constraint_met = (ctr_inc_val === dmem_constraint)
  val vdb_constraint_met  = (ctr_inc_val === vdb_constraint)
  val vl_constraint_met   = (ctr_inc_val === vl_constraint)
  val seg_constraint_met  = (ctr_inc_val === seg_constraint) && use_seg_constraint
  val max_dmem_off_met    = ((dmem_off + ctr_inc_val) === dmem_max)

  // ======== Vstart Handling Constraints ========
  // vdb is always aligned to vreg, so readout of bytes below vstart happens in handling state
  // dmem_bytes is the max number of bytes that can be read out for a cycle
  val vstart_readout_done_q = RegInit(false.B)

  val vstart_EEW_CTR = (vstart << el_mask_off)
  val vstart_el_id   = (vstart & ((1.U << el_mask_width) - 1.U))

  val vstart_el_dist      = (vstart_el_id - ((EEW_CTR >> el_mask_off) & ((1.U << el_mask_width) - 1.U)))
  val vstart_last_readout = (vstart_el_dist <= (io.vdb_data.valid_bytes >> eew_enc))
  val vstart_el_id_inc    = Mux(vstart_last_readout, vstart_el_dist, (io.vdb_data.valid_bytes >> eew_enc))

  // ======== Outputs ========

  // ready-valid signals
  val vdb_valid           = (io.vdb_data.valid_bytes =/= 0.U)
  io.start.ready         := (state === State.IDLE)
  io.store_packet.valid  := (state === State.PACKING) && vdb_valid
  val vdb_ready           = ((state === State.PACKING) && (io.store_packet.ready)) ||
                            ((state === State.VSTART_HANDLING) && !(vstart_readout_done_q))
  io.vdb_data.read_bytes := Mux(vdb_ready, Mux((state === State.PACKING),
                              (1.U << (ctr_inc_enc + eew_enc)), // normal increment case
                              (vstart_el_id_inc << eew_enc)), 0.U)  // vstart handling case
  io.vdb_data.read_all   := Mux(vdb_ready, (state === State.PACKING) && (vl_constraint_met || (use_seg_constraint && seg_constraint_met)), false.B) // last packet
  io.gen_active          := (state === State.PACKING) || (state === State.VSTART_HANDLING)

  val addr_off = (Mux(
    stride_dir,
    -(((EEW_CTR & ~((1.U<<seg_mask_width)-1.U)) - (EEW_CTR & ((1.U<<seg_mask_width)-1.U))) << eew_enc).asSInt, // negate everything except the seg_id (have to double negate because of type issues)
     ((EEW_CTR) << eew_enc).asSInt
  ))
  
  // store packet
  io.store_packet.bits.addr     := (base_addr.asSInt + addr_off).asUInt
  io.store_packet.bits.data     := io.vdb_data.data
  io.store_packet.bits.mem_size := (ctr_inc_enc + eew_enc)
  io.store_packet.bits.sb_id    := sb_id
  io.store_packet.bits.is_fake  := false.B
  io.store_packet.bits.misaligned := ((base_addr & ((1.U << eew_enc) - 1.U)) =/= 0.U)
  io.store_packet.bits.last     := (state === State.PACKING) && (vl_constraint_met)
  io.store_packet.bits.uop      := io.start.bits.uop

  // ======== State Machine ========

  switch (state) {
    // IDLE STATE
    is (State.IDLE) {
      when (io.start.fire) {

        // -- CTR reset --
        state := Mux(
          (vstart =/= 0.U),
          State.VSTART_HANDLING,
          State.PACKING
        )
        EEW_CTR := Mux(
          use_seg_constraint,
          vstart_EEW_CTR,
          (vstart_EEW_CTR & ~(((1.U<<el_mask_width)-1.U)<<el_mask_off)) // will be adjusted in vstart handling
        )

        vstart_readout_done_q := use_seg_constraint // readout not required for segment mode (bypass)

        // -- Init mem alignment --
        val high_off = (((1<<(ADDR_BREAK))-1).U - base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from end of DMEM (high) to base_addr
        val low_off  = (base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from start of DMEM (low) to base_addr
        dmem_off := Mux(stride_dir && !use_seg_constraint, high_off, low_off)
        dmem_max := (DMEM_BYTES.U >> eew_enc)
      }
    }
    // VSTART HANDLING STATE
    is (State.VSTART_HANDLING) {
      when (io.kill) {
        state := State.IDLE
      } .otherwise {

        // -- Next values calculation --
        val vdb_fire = (vdb_ready && vdb_valid)
        val vstart_readout_done = vstart_readout_done_q || (vstart_last_readout && vdb_fire)
        val next_addr = (base_addr.asSInt + (Mux(
          stride_dir,
          -((vstart_EEW_CTR) << eew_enc).asSInt,
           ((vstart_EEW_CTR) << eew_enc).asSInt
        ))).asUInt

        // -- Readout state (if not done) --
        when (!vstart_readout_done_q) {
          vstart_readout_done_q := vstart_readout_done
          val next_EEW_CTR = (EEW_CTR & ~(((1.U<<el_mask_width)-1.U)<<el_mask_off)) |
                            ((EEW_CTR &  (((1.U<<el_mask_width)-1.U)<<el_mask_off)) + (vstart_el_id_inc<<el_mask_off))
          EEW_CTR := Mux(vdb_fire, next_EEW_CTR, EEW_CTR)
        }

        // -- State transition --
        when (vstart_readout_done) { // vdb.fire
          // next state
          state := State.PACKING
          // realign dmem offset to the new address
          val high_off = (((1<<(ADDR_BREAK))-1).U - next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          val low_off  = (next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          dmem_off := Mux(stride_dir && !use_seg_constraint, high_off, low_off)
        }
      }
    }
    // PACKING STATE
    is (State.PACKING) {
      when (io.kill) {
        state := State.IDLE
      } .elsewhen (io.store_packet.fire) {

        // -- Increment counter --
        EEW_CTR := EEW_CTR + ctr_inc_val

        // -- Wrap increment dmem_off --
        when (max_dmem_off_met) {
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

  // ======== Debug ========

  when (vdb_valid && vdb_ready && !io.vdb_data.read_all) {
    assert((ctr_inc_val =/= 0.U), "ctr_inc_val must be non-zero when vdb_valid and vdb_ready are true")
  }

  // IO ports
  dontTouch(io.start)
  dontTouch(io.vdb_data)
  dontTouch(io.kill)
  dontTouch(io.store_packet)

  // Internal state and config
  dontTouch(state)
  dontTouch(sb_id)
  dontTouch(vl)
  dontTouch(eew_enc)
  dontTouch(emul_enc)
  dontTouch(stride_dir)
  dontTouch(seg_enc)
  dontTouch(base_addr)

  // Packing state
  dontTouch(EEW_CTR)
  dontTouch(dmem_off)
  dontTouch(dmem_max)

  // Constraint logic
  dontTouch(dmem_constraint)
  dontTouch(vl_constraint)
  dontTouch(vdb_constraint)
  dontTouch(vdb_constraint_met)
  dontTouch(dmem_constraint_met)
  dontTouch(vl_constraint_met)
  dontTouch(ctr_inc_val)
  dontTouch(ctr_inc_enc)

}
