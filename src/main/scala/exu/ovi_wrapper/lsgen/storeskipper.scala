// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._   

class StoreSkipper(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends Module with VecLSGenConstants {
  // ======== Parameters ========
  val CTR_WIDTH = (EL_ID_W+((1<<EMUL_ENC_W)-1)) // same as max of vl

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = Flipped(DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH)))
    val use_seg_constraint = Input(Bool())
    // mask interface (for masked stores)
    val mask = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val mask_data = Input(UInt(MASK_W.W))
    }
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
    val IDLE, SKIPPING = Value
  }

  // ======== Config info ========

  val state      = RegInit(State.IDLE)
  val sb_id      = io.start.bits.sb_id
  val vl         = io.start.bits.vl
  val eew_enc    = io.start.bits.eew_enc
  val emul_enc   = io.start.bits.emul_enc
  val stride     = io.start.bits.stride
  val stride_dir = io.start.bits.stride_dir
  val seg_count  = io.start.bits.seg_count
  val is_mask    = io.start.bits.is_mask
  val base_addr  = io.start.bits.base_addr
  val use_seg_constraint = io.use_seg_constraint

  // ======== Walking state ========
  val current_seg_id    = RegInit(0.U(SEG_W.W))
  val current_el_id     = RegInit(0.U(EL_ID_W.W))
  val current_v_group_id = RegInit(0.U(((1<<EMUL_ENC_W)-1).W))
  val current_addr      = RegInit(0.U(64.W))
  val current_ctr       = RegInit(0.U(CTR_WIDTH.W))
  val current_mask_data = RegInit(0.U(MASK_W.W))
  val current_mask_off  = RegInit(0.U(MASK_W_SIZE.W))
  val dmem_off          = RegInit(0.U(DMEM_ENC.W))
  val dmem_max          = RegInit(0.U(DMEM_ENC.W))

  // ======== Skipping Constraints ========

  val mask_constraint = Mux(
    (current_mask_data =/= 0.U),
    PriorityEncoder(current_mask_data),
    MASK_W.U - current_mask_off
  )
  val vdb_constraint = io.vdb_data.valid_bytes >> eew_enc
  val vl_constraint  = vl - current_ctr

  // skip_val is the biggest power of 2 value smaller than the smallest of skipping constraints
  val skip_val = WireInit(0.U(11.W))
  when ((mask_constraint <= vl_constraint) && (mask_constraint <= vdb_constraint)) {
    skip_val := Reverse(PriorityEncoderOH(Reverse(mask_constraint)))
  }.elsewhen ((vdb_constraint <= mask_constraint) && (vdb_constraint <= vl_constraint)) {
    skip_val := Reverse(PriorityEncoderOH(Reverse(vdb_constraint)))
  }.elsewhen ((vl_constraint <= mask_constraint) && (vl_constraint <= vdb_constraint)) {
    skip_val := Reverse(PriorityEncoderOH(Reverse(vl_constraint)))
  }

  val skippable = (skip_val =/= 0.U)
  val skip_enc  = PriorityEncoder(skip_val)

  val vdb_constraint_met = (skip_val === vdb_constraint)
  val vl_constraint_met  = (skip_val === vl_constraint)

  // ======== Packing Constraints ========

  val dmem_constraint = PriorityEncoderOH(dmem_off | ~(dmem_max-1.U))
  val packing_constraint = seg_count - current_seg_id

  // seg_inc_val is the biggest power of 2 value smaller than the smallest of packing constraints
  val seg_inc_val = WireInit(0.U(11.W))
  when (dmem_constraint <= packing_constraint) {
    seg_inc_val := Reverse(PriorityEncoderOH(Reverse(dmem_constraint)))
  }.elsewhen (packing_constraint <= dmem_constraint) {
    seg_inc_val := Reverse(PriorityEncoderOH(Reverse(packing_constraint)))
  }

  val seg_inc_enc = PriorityEncoder(seg_inc_val)

  val dmem_constraint_met    = (seg_inc_val === dmem_constraint)
  val packing_constraint_met = (seg_inc_val === packing_constraint)

  // ======== Max Constraints ========

  val max_seg_id_met  = (seg_inc_val === packing_constraint)
  val max_el_id_met   = (current_el_id === ((VLEN_BYTES.U >> eew_enc) - 1.U))
  val max_v_group_met = (current_v_group_id === ((1.U << emul_enc) - 1.U))
  val max_ctr_met     = (current_ctr === (vl - 1.U)) && max_seg_id_met // last ever packet
  val next_mask_off   = current_mask_off + Mux(skippable, skip_val, Mux(max_seg_id_met, 1.U, 0.U))
  val max_mask_met    = (next_mask_off === MASK_W.U)


  // ======== Outputs ========

  // ready-valid signals
  val need_next_mask      = (is_mask && max_mask_met && !(max_ctr_met || vl_constraint_met))
  val vdb_valid           = (io.vdb_data.valid_bytes =/= 0.U)
  io.start.ready         := ((state === State.IDLE)     && (!is_mask || io.mask.valid))
  io.mask.ready          := ((state === State.SKIPPING) && need_next_mask && (io.store_packet.ready) && (vdb_valid)) ||
                            ((state === State.IDLE)     && (is_mask && io.start.valid))
  io.store_packet.valid  := ((state === State.SKIPPING) && (!need_next_mask || io.mask.valid) && (vdb_valid))
  val vdb_ready           = ((state === State.SKIPPING) && (!need_next_mask || io.mask.valid) && (io.store_packet.ready))
  io.vdb_data.read_bytes := Mux(vdb_ready, Mux(skippable, (seg_count << (skip_enc + eew_enc)), (1.U << (seg_inc_enc + eew_enc))), 0.U)
  io.vdb_data.read_all   := Mux(vdb_ready, (state === State.SKIPPING) && (vl_constraint_met || max_ctr_met || (max_seg_id_met && use_seg_constraint)), false.B) // last packet
  io.gen_active          := (state === State.SKIPPING)

  val addr_off = (current_seg_id << eew_enc).asSInt
  
  // store packet
  io.store_packet.bits.addr     := (current_addr.asSInt + Mux(stride_dir, -addr_off, addr_off)).asUInt
  io.store_packet.bits.data     := io.vdb_data.data
  io.store_packet.bits.mem_size := Mux(skippable, 0.U, (seg_inc_enc + eew_enc))
  io.store_packet.bits.sb_id    := sb_id
  io.store_packet.bits.is_fake  := (seg_inc_val === 0.U) || (is_mask && skippable)
  io.store_packet.bits.misaligned := false.B
  io.store_packet.bits.last     := (state === State.SKIPPING) && (vl_constraint_met || max_ctr_met)
  io.store_packet.bits.uop      := io.start.bits.uop

  // ======== State Machine ========

  switch (state) {
    is (State.IDLE) {
      when (io.start.fire) {
        // -- state config --
        state := State.SKIPPING

        // -- Initialize counters --
        current_el_id   := 0.U
        current_seg_id  := 0.U
        current_v_group_id := 0.U
        current_addr    := base_addr
        current_ctr     := 0.U
        current_mask_data := io.mask.mask_data
        current_mask_off  := 0.U

        // -- Init mem alignment --
        val high_off = (((1<<(ADDR_BREAK))-1).U - base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from end of DMEM (high) to base_addr
        val low_off  = (base_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from start of DMEM (low) to base_addr
        dmem_off := Mux(stride_dir, high_off, low_off)
        dmem_max := (DMEM_BYTES.U >> eew_enc)
      }
    }

    is (State.SKIPPING) {
      when (io.kill) {
        state := State.IDLE
      } .elsewhen (io.store_packet.fire) {

        // -- Next Address calculation --
        val next_addr = (current_addr.asSInt + Mux(skippable, (stride << skip_enc), stride)).asUInt
        dontTouch(next_addr)

        // -- Counter ripple logic --
        // direct skip case (seg_id is always 0 when this happens)
        when (skippable) {
          current_el_id := current_el_id + skip_val
        // [v_group, el, seg]: [x, x, +1]
        }.elsewhen (!max_seg_id_met) {
          current_seg_id := current_seg_id + seg_inc_val
        // [v_group, el, seg]: [x, +1, 0]
        }.elsewhen (!max_el_id_met) {
          current_seg_id := 0.U
          current_el_id := current_el_id + 1.U
        // [v_group, el, seg]: [+1, 0, 0]
        }.otherwise {
          current_seg_id := 0.U
          current_el_id := 0.U
          current_v_group_id := current_v_group_id + 1.U
        }

        // -- Next element config/updates --
        when (max_seg_id_met || skippable) {
          current_addr := next_addr
          current_ctr  := current_ctr + Mux(skippable, skip_val, 1.U)
        }

        // -- DMEM offset increment --
        // recalc dmem_off for new element
        when (max_seg_id_met || skippable) {
          dmem_off := (next_addr(DMEM_ENC-2, 0)) >> eew_enc
        // wrap inc dmem_off
        }.elsewhen(dmem_constraint_met) {
          dmem_off := 0.U
        // inc dmem_off for next segment
        }.otherwise{
          dmem_off := dmem_off + seg_inc_val
        }

        // -- Mask buffer update --
        when (max_mask_met) {
          current_mask_off  := 0.U
          current_mask_data := io.mask.mask_data
        } .elsewhen (skippable) {
          current_mask_off  := current_mask_off + skip_val
          current_mask_data := current_mask_data >> skip_val
        } .elsewhen (max_seg_id_met) {
          current_mask_off  := current_mask_off + 1.U
          current_mask_data := current_mask_data >> 1.U
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
    assert((seg_inc_val =/= 0.U), "seg_inc_val must be non-zero when vdb_valid and vdb_ready are true")
  }

  // IO ports
  dontTouch(io.start)
  dontTouch(io.mask)
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
  dontTouch(base_addr)
  dontTouch(stride)
  dontTouch(seg_count)
  dontTouch(is_mask)

  // Skipping state
  dontTouch(current_seg_id)
  dontTouch(current_el_id)
  dontTouch(current_v_group_id)
  dontTouch(current_addr)
  dontTouch(current_ctr)
  dontTouch(current_mask_data)
  dontTouch(current_mask_off)
  dontTouch(dmem_off)
  dontTouch(dmem_max)

  // Skipping logic
  dontTouch(skippable)
  dontTouch(skip_val)
  dontTouch(skip_enc)
  dontTouch(mask_constraint)
  dontTouch(vl_constraint)
  dontTouch(vl_constraint_met)
  dontTouch(vdb_constraint)
  dontTouch(vdb_constraint_met)

  // Packing logic
  dontTouch(dmem_constraint)
  dontTouch(packing_constraint)
  dontTouch(dmem_constraint_met)
  dontTouch(packing_constraint_met)
  dontTouch(seg_inc_val)
  dontTouch(seg_inc_enc)

  dontTouch(max_seg_id_met)
  dontTouch(max_el_id_met)
  dontTouch(max_v_group_met)
  dontTouch(max_ctr_met)
  dontTouch(max_mask_met)
  dontTouch(next_mask_off)

}