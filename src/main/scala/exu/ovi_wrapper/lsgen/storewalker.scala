// See LICENSE.TT for license details.
package boom.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{VConfig}

import boom.exu.FUConstants._

class StoreWalker(override val VLEN: Int, override val DMEM_WIDTH: Int)(implicit p: Parameters)
extends Module with VecLSGenConstants {
  // ======== Parameters ========
  val CTR_WIDTH = (EL_ID_W+((1<<EMUL_ENC_W)-1)) // same as max of vl

  // ======== Input-Output Ports ========
  val io = IO(new Bundle {
    // start config signals
    val start = Flipped(DecoupledIO(new ConfigInfo(VLEN, DMEM_WIDTH)))
    val use_seg_constraint = Input(Bool())
    // index interface (for indexed stores)
    val index = new Bundle {
      val ready = Output(Bool())
      val valid = Input(Bool())
      val index_value = Input(SInt(MASK_W.W))  // Signed index offset in bytes
      val mask_bit    = Input(Bool())      // Element valid/invalid
      val last_index  = Input(Bool())      // Last index in sequence
    }
    // kill signal (used to reset the FSM)
    val kill = Input(Bool())
    // store data interface
    val vdb_data = new Bundle {
      val read_bytes  = Output(UInt(VDB_R_SIZE_BYTES.W)) // like a ready signal
      val read_all    = Output(Bool())
      val valid_bytes = Input(UInt(VDB_R_SIZE_BYTES.W))  // like a valid signal
      val data        = Input(UInt(DMEM_WIDTH.W))
    }
    // status signal
    val gen_active = Output(Bool())
    // store packet
    val store_packet = DecoupledIO(new StorePacket(VLEN, DMEM_WIDTH))
  })

  // ======== Definitions ========

  object State extends ChiselEnum {
    val IDLE, VSTART_HANDLING, WALKING = Value
  }

  // ======== Config info ========

  val state     = RegInit(State.IDLE)
  val sb_id     = io.start.bits.sb_id
  val base_v_reg = io.start.bits.base_v_reg
  val vl        = io.start.bits.vl
  val vstart    = io.start.bits.vstart
  val eew_enc   = io.start.bits.eew_enc
  val emul_enc  = io.start.bits.emul_enc
  val stride    = io.start.bits.stride
  val seg_count = io.start.bits.seg_count
  val is_mask   = io.start.bits.is_mask
  val is_index  = io.start.bits.is_index
  val base_addr = io.start.bits.base_addr
  val use_seg_constraint = io.use_seg_constraint

  // ======== Walking state ========

  val current_seg_id   = RegInit(0.U(SEG_W.W))                 // current segment index
  val current_el_id    = RegInit(0.U(EL_ID_W.W))               // current element index
  val current_v_group_id = RegInit(0.U(((1<<EMUL_ENC_W)-1).W)) // current vector group
  val current_addr     = RegInit(0.U(64.W))                    // current address (incremental)
  val current_ctr      = RegInit(0.U(CTR_WIDTH.W))             // element counter to vl
  val current_mask_bit = RegInit(false.B)                      // current mask bit (for indexed stores)
  val current_last_index = RegInit(false.B)                    // current "last index" state (remember across segments)
  val current_dir        = RegInit(false.B)                    // current direction of stride/index
  val dmem_off   = RegInit(0.U(DMEM_ENC.W)) // memory alignment offset
  val dmem_max   = RegInit(0.U(DMEM_ENC.W)) // max elements to fit in DMEM

  // ======== Packing Constraints ========

  // elements that can be stored contiguously depending on mem alignment
  val dmem_constraint = PriorityEncoderOH(dmem_off | ~(dmem_max-1.U))

  // elements that can be provided from VDB
  val vdb_constraint = io.vdb_data.valid_bytes >> eew_enc

  // elements until the next stride update
  val packing_constraint = seg_count - current_seg_id

  val seg_inc_val = WireInit(0.U(SEG_W.W))
  when ((dmem_constraint <= packing_constraint) && (dmem_constraint <= vdb_constraint)) {
    seg_inc_val := Reverse(PriorityEncoderOH(Reverse(dmem_constraint)))
  }.elsewhen ((vdb_constraint <= dmem_constraint) && (vdb_constraint <= packing_constraint)) {
    seg_inc_val := Reverse(PriorityEncoderOH(Reverse(vdb_constraint)))
  }.elsewhen ((packing_constraint <= dmem_constraint) && (packing_constraint <= vdb_constraint)) {
    seg_inc_val := Reverse(PriorityEncoderOH(Reverse(packing_constraint)))
  }
  val seg_inc_enc = PriorityEncoder(seg_inc_val)

  val dmem_constraint_met    = (seg_inc_val === dmem_constraint)
  val vdb_constraint_met     = (seg_inc_val === vdb_constraint)
  val packing_constraint_met = (seg_inc_val === packing_constraint)

  // ======== Max Constraints ========

  val max_seg_id_met   = (seg_inc_val === packing_constraint)
  val max_el_id_met    = (current_el_id === ((VLEN_BYTES.U >> eew_enc) - 1.U))
  val max_v_group_met  = (current_v_group_id === ((1.U << emul_enc) - 1.U))
  val max_ctr_met      = ((current_ctr === (vl - 1.U)) || (is_index && current_last_index)) && max_seg_id_met // last ever packet
  val max_dmem_off_met = ((dmem_off + seg_inc_val) === dmem_max)

  // ======== Vstart Handling Constraints ========
  // vdb is always aligned to vreg, so readout of bytes below vstart happens in handling state
  // (only for strided) we also have to shift the address for vstart across multiple cycles
  val vstart_readout_done_q = RegInit(false.B)
  val vstart_addr_done_q    = RegInit(false.B)

  // split vstart into it's el_id and v_group_id components
  val el_mask_width = (log2Ceil(VLEN_BYTES).U - eew_enc)
  val vstart_el_id      = (vstart & ((1.U << el_mask_width) - 1.U))
  val vstart_v_group_id = vstart >> el_mask_width

  // calculate jump to vstart
  val vstart_dist     = (vstart - current_ctr) // distance to vstart
  val vstart_last_inc = ((vstart_dist & (vstart_dist - 1.U)) === 0.U) // if the distance is 1-hot, then only 1 jump away from vstart
  val vstart_skip_enc = PriorityEncoder(vstart_dist) // jump by power of 2

  // ======== Outputs ========

  // ready-valid signals
  val need_next_index   = (is_index && max_seg_id_met && !max_ctr_met)
  val vdb_valid           = (io.vdb_data.valid_bytes =/= 0.U)
  io.start.ready         := ((state === State.IDLE)    && (!is_index || io.index.valid))
  io.index.ready         := ((state === State.WALKING) && need_next_index && (io.store_packet.ready) && (vdb_valid)) ||
                            ((state === State.IDLE)    && (is_index && io.start.valid))
  io.store_packet.valid  := ((state === State.WALKING) && (!need_next_index || io.index.valid) && (vdb_valid))
  val vdb_ready           = ((state === State.WALKING) && (!need_next_index || io.index.valid) && (io.store_packet.ready)) ||
                            ((state === State.VSTART_HANDLING) && !(vstart_readout_done_q))
  io.vdb_data.read_bytes := Mux(vdb_ready, Mux(state === State.WALKING,
                              (1.U << (seg_inc_enc + eew_enc)), // normal increment case
                              (vstart_el_id << eew_enc) ), 0.U) // vstart handling case
  io.vdb_data.read_all   := Mux(vdb_ready, (state === State.WALKING) && (max_ctr_met || (max_seg_id_met && use_seg_constraint)), false.B) // last packet
  io.gen_active          := (state === State.WALKING) || (state === State.VSTART_HANDLING)

  val addr_off = (current_seg_id << eew_enc).asSInt
  
  // packet info
  io.store_packet.bits.addr     := (current_addr.asSInt + addr_off).asUInt
  io.store_packet.bits.data     := io.vdb_data.data
  io.store_packet.bits.mem_size := (seg_inc_enc + eew_enc)
  io.store_packet.bits.sb_id    := sb_id
  io.store_packet.bits.is_fake  := (seg_inc_val === 0.U) || (is_mask && (current_mask_bit === false.B))
  io.store_packet.bits.misaligned := ((current_addr & ((1.U << eew_enc) - 1.U)) =/= 0.U)
  io.store_packet.bits.last     := (state === State.WALKING) && (max_ctr_met)
  io.store_packet.bits.uop      := io.start.bits.uop

  // ======== State Machine ========

  switch (state) {
    // IDLE STATE
    is (State.IDLE) {
      when (io.start.fire) {
        
        // -- Next Address and dir calculation --
        val next_addr = (base_addr.asSInt + Mux(is_index, io.index.index_value, 0.S)).asUInt
        val next_direction = Mux(is_index, io.index.index_value, stride)(63)

        // -- Input config --
        val goto_handling = (vstart =/= 0.U)
        state := Mux(
          goto_handling,
          State.VSTART_HANDLING,
          State.WALKING
        )

        // -- Initialize counters --
        current_el_id   := vstart_el_id
        current_seg_id  := 0.U
        current_v_group_id := vstart_v_group_id
        current_addr    := next_addr
        current_ctr     := Mux(goto_handling, 0.U, vstart)
        current_mask_bit   := io.index.mask_bit
        current_last_index := io.index.last_index
        current_dir        := next_direction

        vstart_readout_done_q := false.B
        vstart_addr_done_q    := (goto_handling && is_index) // dont want to do this for indexed (so set to complete immediately)

        // -- Initialize DMEM info --
        val high_off  = (((1<<(ADDR_BREAK))-1).U - next_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from end of DMEM (high) to base_addr
        val low_off   = (next_addr(ADDR_BREAK-1, 0)) >> eew_enc // EEWs from start of DMEM (low) to base_addr
        dmem_off := Mux(next_direction && !use_seg_constraint, high_off, low_off)
        dmem_max := (DMEM_BYTES.U >> eew_enc)
      }
    }
    // VSTART HANDLING STATE
    is (State.VSTART_HANDLING) {
      when (io.kill) {
        state := State.IDLE
      } .otherwise {

        // -- Next values calculation --
        // vstart controls
        val vstart_readout_done = vstart_readout_done_q || (vdb_ready && vdb_valid)
        val vstart_addr_done    = vstart_addr_done_q || vstart_last_inc
        // address calculation
        val next_addr = (current_addr.asSInt + (stride << vstart_skip_enc)).asUInt

        // -- Readout state (if not done) --
        when (!vstart_readout_done_q) {
          vstart_readout_done_q := vstart_readout_done
        }
        // -- Increment address and counter (if not done) --
        when (!vstart_addr_done_q) {
          vstart_addr_done_q := vstart_addr_done
          current_addr := next_addr
          current_ctr  := current_ctr + (1.U << vstart_skip_enc)
        }

        // -- State transition --
        when (vstart_readout_done && vstart_addr_done) {
          // next state
          state := State.WALKING
          // realign dmem offset to the new address
          val high_off = (((1<<(ADDR_BREAK))-1).U - next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          val low_off  = (next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          dmem_off := Mux(current_dir && !use_seg_constraint, high_off, low_off)
        }
      }
    }
    // WALKING STATE
    is (State.WALKING) {
      when (io.kill) {
        state := State.IDLE
      } .elsewhen (io.store_packet.fire) {

        // -- Next Address calculation --
        val next_addr = Mux(
          is_index,   // load can be index / stride type
          (base_addr.asSInt + io.index.index_value).asUInt,
          (current_addr.asSInt + stride).asUInt
        )
        val next_direction = Mux(is_index, io.index.index_value, stride)(63)

        // -- Counter ripple logic --
        // [v_group, el, seg]: [x, x, +1]
        when (!max_seg_id_met) {
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
        when (max_seg_id_met) {
          current_addr       := next_addr           // update address
          current_ctr        := current_ctr + 1.U   // increment CTR
          current_mask_bit   := io.index.mask_bit   // update mask bit
          current_last_index := io.index.last_index // update last index
          current_dir        := next_direction      // update direction
        }

        // -- DMEM offset increment --
        // recalc dmem_off for next element
        when (max_seg_id_met) {
          val high_off  = (((1<<(ADDR_BREAK))-1).U - next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          val low_off   = (next_addr(ADDR_BREAK-1, 0)) >> eew_enc
          dmem_off := Mux(next_direction && !use_seg_constraint, high_off, low_off)
        // wrap inc dmem_off
        }.elsewhen(max_dmem_off_met) {
          dmem_off := 0.U
        // inc dmem_off for next segment
        }.otherwise{
          dmem_off := dmem_off + seg_inc_val
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
  dontTouch(io.index)
  dontTouch(io.vdb_data)
  dontTouch(io.kill)
  dontTouch(io.store_packet)

  // Internal state and config
  dontTouch(state)
  dontTouch(sb_id)
  dontTouch(base_v_reg)
  dontTouch(vl)
  dontTouch(eew_enc)
  dontTouch(emul_enc)
  dontTouch(stride)
  dontTouch(seg_count)
  dontTouch(is_mask)
  dontTouch(is_index)
  dontTouch(base_addr)

  // Walking state
  dontTouch(current_seg_id)
  dontTouch(current_el_id)
  dontTouch(current_v_group_id)
  dontTouch(current_addr)
  dontTouch(current_ctr)
  dontTouch(current_mask_bit)
  dontTouch(current_last_index)
  dontTouch(dmem_off)
  dontTouch(dmem_max)

  // Constraint logic
  dontTouch(dmem_constraint)
  dontTouch(packing_constraint)
  dontTouch(vdb_constraint)
  dontTouch(vdb_constraint_met)
  dontTouch(dmem_constraint_met)
  dontTouch(packing_constraint_met)
  dontTouch(seg_inc_val)
  dontTouch(seg_inc_enc)

} 