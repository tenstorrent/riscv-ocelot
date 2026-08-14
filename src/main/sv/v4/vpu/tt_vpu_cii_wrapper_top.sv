/*************************************************************************
 *
 * Tenstorrent CONFIDENTIAL
 * __________________
 *
 *  Tenstorrent Inc.
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of Tenstorrent Inc.  The intellectual
 * and technical concepts contained
 * herein are proprietary to Tenstorrent Inc.
 * and may be covered by U.S., Canadian and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Tenstorrent Inc.
 */
/*
  File Name: tt_vpu_cii_wrapper_top.sv
  Author: ading
  Date Created: 7/2026
  Description: 
    Top Level wrapper which implments the CII interface fifo and credit scheme and vector unit.
 */

`include "tt_cii_caracal_pkg.svh"

module tt_vpu_cii_wrapper_top
  import tt_cii_caracal_pkg::*;
#(
  parameter VLEN = 256
) (
  input  logic clk,
  input  logic reset_n,

  // CII coprocessor-side interface (four credit-metered channels; typed by
  // tt_cii_caracal_pkg). The relay drives this modport from the host side.
  tt_cii_interface.coprocessor cii_intf,

  // Debug signals for cosim checker
  output logic              debug_wb_vec_valid,
  output logic [VLEN*8-1:0] debug_wb_vec_wdata,
  output logic [7:0]        debug_wb_vec_wmask
);

  localparam INCL_VEC = 1;
  localparam LQ_DEPTH=8;
  localparam LQ_DEPTH_LOG2=$clog2(LQ_DEPTH);
  localparam INCL_FP = 1;
  localparam FLEN    = 32;
  localparam FP_RF_RD_PORTS  = 4;
  localparam EXP_WIDTH   = 8;
  localparam MAN_WIDTH   = 23;

  // ---- Issue path (driven by the CII issue channel in C1; tied off later) ----

  logic [31:0]                      read_issue_inst;
  logic [63:0]                      read_issue_scalar_opnd;
  logic                             read_valid;
  logic                             id_ready_to_receive;
  logic [4:0]                       read_issue_sb_id;
  tt_briscv_pkg::inst_state_e       read_issue_state;

  // ---- CSR config (from the issue packet in C2; tied off later) ----
  tt_briscv_pkg::csr_t              csr_de0;
  tt_briscv_pkg::csr_t              csr_ex0;

  // ---- tt_id <-> tt_vec datapath control ----
  logic [63:0]                      rf_vex_p0, rf_vex_p1, fprf_vex_p0;
  logic [4:0]                       id_type;
  logic                             id_vex_rts, vex_id_rtr;
  tt_briscv_pkg::vec_autogen_s      id_vpu_uop_packet;

  logic                             vex_id_incr_addrp2;
  logic                             v_vm;
  logic                             vex_div_busy;
  logic                             ex_id_rtr;
  logic [31:0]                      id_ex_instrn;


  logic                             id_replay;
  logic [4:0]                       iterate_addrp0, iterate_addrp1, iterate_addrp2;
  logic                             ignore_lmul, ignore_dstincr, ignore_srcincr;


  // ---- Source pull response data ----
  // Combinational feed to tt_vec (assigned from the per-member operand arrays
  // below); tt_vec reads them at its iterate member address, matching the
  // tt_vec_regfile combinational read they replaced.
  logic [VLEN-1:0]                  vrf_p0_rddata, vrf_p1_rddata, vrf_p2_rddata, vrf_vm0_rddata;

  // Per-instruction OPERAND REGISTERS. Each vector source holds up to MAX_MEMBERS
  // member beats (VS2 wide -> up to 2*NM, bounded by the RVV LMUL<=4 widening rule
  // => <=8 members), pulled via CII src-data and held until the whole set is staged.
  // The 4-lane src channel returns member `k` of {VS1,VS2,VS3,VM} in one dat beat,
  // so the member index equals the receive beat count (receive_data_cnt). tt_vec then reads
  // them combinationally at its iterate member address.
  logic [VLEN-1:0]                  opnd_p0 [0:7]; // opnd_p0 = VS1
  logic [VLEN-1:0]                  opnd_p1 [0:7]; // opnd_p1 = VS2
  logic [VLEN-1:0]                  opnd_p2 [0:7]; // opnd_p2 = VS3/old-dest
  logic [VLEN-1:0]                  opnd_vm;       // opnd_vm = v0 mask

  logic                             vpu_sat_csr;

  // ---- tt_vec result ports -> CII writeback (C4) ----
  logic                             vex_mem_lqvld_1c, vex_mem_lqvld_2c, vex_mem_lqvld_3c, vex_mem_lqvld_div;
  logic [VLEN-1:0]                  vex_mem_lqdata_1c, vex_mem_lqdata_2c, vex_mem_lqdata_3c, vex_mem_lqdata_div;
  tt_briscv_pkg::csr_fp_exc         vex_mem_lqexc_1c, vex_mem_lqexc_2c, vex_mem_lqexc_3c, vex_mem_lqexc_div;
  logic [LQ_DEPTH_LOG2-1:0]         vex_mem_lqid_1c, vex_mem_lqid_2c, vex_mem_lqid_3c, vex_mem_lqid_div;


  // ----- Credit Registers -----
  logic [$clog2(CII_N_REQ_CREDITS+1)-1:0] req_credit_cnt;
  logic [$clog2(CII_N_WB_CREDITS+1)-1 :0] wb_credit_cnt;


  // ----- Write Back Signals -----
  logic                       wb_result_valid;
  logic [VLEN-1:0]            wb_result_data;
  logic [LQ_DEPTH_LOG2-1:0]   wb_ldqid;
  tt_briscv_pkg::csr_fp_exc   wb_fp_flags;
  wire                        wb_fire;

  // ----- Old-dest merge writeback (vec_single_reg upper members 1..NM-1) -----
  // vec_single_reg ops emit one VPU result (member 0); the wrapper then writes the
  // remaining group members 1..NM-1 verbatim from the pulled old dest (undisturbed).
  logic [VLEN-1:0]          wb_olddest [0:7];  // old dest group, latched at S_HOLD
  logic                     wbx_active;        // merging members 1..NM-1 this op
  logic                     wbx_pending;       // op accepted; awaiting its member-0 WB
  logic [4:0]               wbx_member;        // current merge member index
  logic [4:0]               wbx_nm;            // latched dst group size (NM)
  cii_caracal_tag_t         wbx_tag;           // latched op tag
  logic [LQ_DEPTH_LOG2-1:0] wbx_base_ldqid;    // op's member-0 ldqid (merge trigger)
  wire                      wb_send_merge = wbx_active;  // this cycle drives a merge beat

  // ----- Data Response Signals -----
  typedef enum logic [2:0] { S_IDLE, S_WAIT, S_REQ, S_DRAIN, S_HOLD } iss_state_e;
  iss_state_e             iss_state;
  logic [31:0]            next_iss_insn;
  cii_caracal_tag_t       next_iss_tag;
  cii_caracal_vtype_t     next_iss_vtype;
  logic [CII_VL_W-1:0]    next_iss_vl, next_iss_vstart;
  logic [1:0]             next_iss_vxrm;
  logic [2:0]             next_iss_frm;
  logic [CII_MEMBER_W-1:0] next_src_beat;          // prefetch member index (0..NM-1)
  logic [CII_MEMBER_W-1:0] receive_data_cnt;         // dat beats received this op

  logic [5:0]             dst_wr ;  // request/receive pointers (<= 3*8+1 = 25)
  logic [63:0]            next_iss_scalar;     // captured .vx/.vf scalar (-> i_if_scalar_opnd)
  
  
  
  // CII writeback lookups keyed by the VPU lqid (results carry lqid): tag,
  // dst-kind, dst member offset, and the final-member (last) marker. Filled for
  // ALL members at accept; C4 indexes by the result lqid.
  cii_caracal_tag_t       tag_by_lqid     [0:LQ_DEPTH-1];
  cii_caracal_dst_kind_e  dstkind_by_lqid [0:LQ_DEPTH-1];
  cii_caracal_offset_t    dstoff_by_lqid  [0:LQ_DEPTH-1];
  logic                   last_by_lqid    [0:LQ_DEPTH-1];

  // Member count (EMUL) from LMUL: 1/2/4/8; fractional LMUL -> 1 register.
  wire [4:0] lmul_beats = next_iss_vtype.vlmul[2] ? 5'd1 : (5'd1 << next_iss_vtype.vlmul[1:0]);


  type(cii_intf.iss_data) iss_fifo_data;
  type(cii_intf.dat_data) dat_fifo_data;
  wire  iss_fifo_empty, dat_fifo_empty;
  logic dat_fifo_valid;   // RegNext(pop): pop_data valid this cycle

  // members to fetch for the current source (scalar is a single beat).
  logic [4:0] src_vs1_num_beats, src_vs2_num_beats, src_vs3_num_beats, src_vm_num_beats;

  logic src_en_vs1, src_en_vs2, src_en_vs3, src_en_vm;

  // Beats to send for this op = max member count over the ENABLED sources (VM is a
  // single beat). Bounds the S_REQ member walk; the shorter sources send CII_SRC_NONE
  // on their lane once exhausted.
  logic [4:0] max_src_beats;



  logic req_send;
  logic scalar_src;


  /////////////////////////////////////////////////////////////////////////////
  // END Signal Declarations
  /////////////////////////////////////////////////////////////////////////////


  // Widening / narrowing (decode is combinational -> valid during prefetch).
  //   wdeop  : dest EEW = 2*SEW  -> dest EMUL = 2*NM (2*NM result beats).
  //   nrwop  : one source (vs2) is wide 2*SEW/2*NM; dest EMUL = NM.
  //   src1hw : in a widening .w-form, vs2 is also wide (2*NM).
  // Per-operand member counts (see explore: Vvv src addr half-steps, wide src /
  // dest full-step):
  //   VS1 (narrow)      : NM
  //   VS2               : 2*NM if (nrwop || (wdeop && src1hw)) else NM
  //   VS3 (= dest group): dst_nm  (fetched for RMW old-dest)
  //   VM  (v0 mask)     : 1
  //   dst_nm (result beats) = wdeop ? 2*NM : NM.
  // B3b: a scalar-DEST op (vmv.x.s/vcpop.m/vfirst.m) produces exactly ONE result
  // (a single scalar into wb_data[63:0]) regardless of LMUL -- force dst_nm=1 so
  // the lqid writeback table sets up one beat (with last=1), matching the single
  // result tt_vec emits. Without this, LMUL>1 would expect NM beats, the `last`
  // beat would never arrive, and the CII tag would leak.
  wire       vs2_wide = id_vpu_uop_packet.nrwop || (id_vpu_uop_packet.wdeop && id_vpu_uop_packet.src1hw);

  // Ops whose DESTINATION is a single vector register regardless of LMUL, so the
  // VPU emits exactly ONE result beat (dst_nm=1). Two VPU-authoritative signals
  // (valid at S_HOLD, when the op is presented to tt_id) capture this exactly:
  //   ignore_lmul    = vmv.x.s | vmv.s.x | vfmv.s.f | vfmv.f.s  (single iteration)
  //   ignore_dstincr = mask_only | reductop  (dst reg address held: mask/reduction)
  // scalar_dest (vmv.x.s/vcpop.m/vfirst.m/vfmv.f.s) is kept explicitly since a
  // scalar-dest op writes one INT/FP result (and vcpop.m/vfirst.m are neither
  // ignore_lmul nor ignore_dstincr). Without dst_nm=1 here, an LMUL>1 op like
  // `vmv.s.x` sets up NM writeback-table entries, the VPU sends only 1 result, the
  // `last`-marked beat (member NM-1) never arrives, and the CII tag / ROB entry
  // never completes -> pipeline hang. This keeps dst_nm in lockstep with the VPU's
  // actual result-beat count; the vs2 SOURCE count is unaffected (a reduction still
  // reads all NM members via src_vs2_num_beats).
  // A VECTOR-dest op that writes only ONE register (member 0) but whose arch dest
  // is an LMUL group in BOOM (vmv.s.x/vfmv.s.f, reductions, mask-producing). The
  // VPU emits one result (member 0); the wrapper WRITES BACK the upper members
  // 1..NM-1 with the UNDISTURBED old dest (they are not part of the destination and
  // must keep their old values). scalar_dest ops (vmv.x.s/vfmv.f.s/vcpop/vfirst)
  // write INT/FP -- genuinely one result, no vector group -- so they stay dst_nm=1.
  //
  // IMPORTANT: this is decoded from tt_id (reductop/mask_only) + the raw instruction
  // (vmv.s.x/vfmv.s.f), NOT from tt_vec's ignore_lmul/ignore_dstincr. Those tt_vec
  // signals only settle once the op is presented to tt_vec (S_HOLD); during the
  // operand-pull phase (S_REQ) they still reflect the PREVIOUS op, which would give
  // this op a stale need_old_dest / max_src_beats and deadlock the source pull.
  //   vmv.s.x (OPMVX) / vfmv.s.f (OPFVF): funct6=010000, vs2 field == 0.
  wire       vmv_s_vec = (next_iss_insn[31:26] == 6'b010000) &&
                         ((next_iss_insn[14:12] == 3'b110) || (next_iss_insn[14:12] == 3'b101)) &&
                         (next_iss_insn[24:20] == 5'd0);
  wire       vec_single_reg = id_vpu_uop_packet.reductop || id_vpu_uop_packet.mask_only || vmv_s_vec;

  // dst_nm = number of writeback beats = BOOM's dest group size. scalar_dest -> 1;
  // everything else (incl. vec_single_reg) writes the full NM (or 2*NM widening)
  // group -- vec_single_reg fills members 1..NM-1 from the old dest (see WB merge).
  wire [4:0] dst_nm   = id_vpu_uop_packet.scalar_dest ? 5'd1 :
                        (id_vpu_uop_packet.wdeop ? (lmul_beats << 1) : lmul_beats);

  wire vm_needed  = id_vpu_uop_packet.usemask || ~next_iss_insn[25];



  // this request routes to the scalar operand latch, not an operand register.
  wire  req_is_scalar = scalar_src;

 
  // Pull the old dest group (VS3) for RMW/accumulate, tail-undisturbed (vta=0), or
  // mask-undisturbed masked ops -- AND for vec_single_reg ops, whose upper members
  // 1..NM-1 are always undisturbed (independent of vta) and are written back from
  // this old-dest pull (opnd_p2). src_vs3_num_beats = dst_nm covers all NM members.
  wire need_old_dest = id_vpu_uop_packet.rf_rden2 ||
                       (~next_iss_vtype.vta) ||
                       (~next_iss_vtype.vma && ~next_iss_insn[25]) ||
                       vec_single_reg;

  

  // When can i pop data from the CII FIFOs?
  wire  iss_pop = (iss_state == S_IDLE) && !iss_fifo_empty;
  wire  dat_pop = ((iss_state == S_REQ) || (iss_state == S_DRAIN)) && !dat_fifo_empty;
  

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin 
      dat_fifo_valid <= 1'b0; 
    end else begin 
      dat_fifo_valid <= dat_pop; 
    end
  end


  /////////////////////////////////////////////////////////////////////////////
  // Manage Src Request Credits
  /////////////////////////////////////////////////////////////////////////////
  always_ff @( posedge clk or negedge reset_n ) begin
    if (!reset_n) begin
      req_credit_cnt  <=  CII_N_REQ_CREDITS;
    end else begin
      // Send-credit counter: -1 per request sent, +1 per returned credit, up to max Request Credits.
      // Do not increment credit if req_send on same cycle as credit return
      if (  cii_intf.req_credit && !req_send && req_credit_cnt != CII_N_REQ_CREDITS ) begin
        req_credit_cnt <= req_credit_cnt + 1'b1;
      
      end else if (!cii_intf.req_credit && req_send) begin
        req_credit_cnt <= req_credit_cnt - 1'b1;

      end else begin
        req_credit_cnt <= req_credit_cnt;
      end
    end
  end

  /////////////////////////////////////////////////////////////////////////////
  // Manage WB Credits
  /////////////////////////////////////////////////////////////////////////////
  always_ff @(posedge clk or negedge reset_n) begin
    // Cap at the FIFO depth -- see the req counter note: the channel returns a
    // credit every cycle wb_credit is high, so an uncapped +1 wraps the counter
    // to 0 and drops results (this dropped member 0 of a multi-beat writeback).
    if (!reset_n) begin
      wb_credit_cnt <= CII_N_WB_CREDITS;
    end else begin
      if ( cii_intf.wb_credit && !wb_fire && wb_credit_cnt != CII_N_WB_CREDITS) begin
        wb_credit_cnt <= wb_credit_cnt + 1'b1;

      end else if (!cii_intf.wb_credit &&  wb_fire) begin
        wb_credit_cnt <= wb_credit_cnt - 1'b1;

      end else begin
        wb_credit_cnt <= wb_credit_cnt;
      end
    end
  end


  /////////////////////////////////////////////////////////////////////////////
  // Instruction Issue and Decode FSM
  /////////////////////////////////////////////////////////////////////////////
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      iss_state       <=  S_IDLE;
      next_src_beat   <=  '0;
      dst_wr          <=  6'd0;

    end else begin
      case (iss_state)
        S_IDLE: begin
          if (!iss_fifo_empty) begin
            iss_state   <=  S_WAIT;   // pop this cycle (iss_pop)
          end
        end

        S_WAIT: begin   // popped beat now valid
            next_iss_insn   <=  iss_fifo_data[0].instr.insn; 
            next_iss_tag    <=  iss_fifo_data[0].tag;
            next_iss_vtype  <=  iss_fifo_data[0].instr.vtype;
            next_iss_vl     <=  iss_fifo_data[0].instr.vl; 
            next_iss_vstart <=  iss_fifo_data[0].instr.vstart;
            next_iss_vxrm   <=  iss_fifo_data[0].instr.vxrm; 
            next_iss_frm    <=  iss_fifo_data[0].instr.frm;
            next_src_beat   <=  '0;
            dst_wr          <=  6'd0;

            iss_state   <=  S_REQ;
        end

        S_REQ: begin
          if (max_src_beats == 5'd0) begin  // op reads no sources -> nothing to stage
            next_src_beat <=  '0;
            iss_state     <=  S_DRAIN;

          end else if (req_credit_cnt != 0) begin  // send one 4-lane beat (member next_src_beat)
            if (next_src_beat == max_src_beats - 1'b1) begin  // last member sent
              next_src_beat <= '0;
              iss_state     <= S_DRAIN;
            end else begin
              // Count only the NON-final beats: dst_wr ends at (num_beats-1), i.e. the
              // LAST member index. This keeps the S_DRAIN target inside receive_data_cnt's
              // member-index range (CII_MEMBER_W bits) -- counting to num_beats (8) would
              // wrap the 3-bit receive_data_cnt (0..7) and the drain would never match.
              dst_wr <= dst_wr + 1'b1;
              next_src_beat <= next_src_beat + 1'b1;
            end
          end
        end

        S_DRAIN: begin
          // Done when the LAST member's dat beat actually arrives (receive_data_cnt is
          // still the last member index that cycle, and its operand is written the same
          // cycle). max_src_beats==0 means the op read no sources -> nothing to wait for.
          if ((max_src_beats == 5'd0) ||
              (dat_fifo_valid && (receive_data_cnt == dst_wr))) begin
            iss_state   <= S_HOLD; // all requested operands staged
          end
        end

        S_HOLD: begin
          if (id_ready_to_receive) begin  // tt_id accepted the instruction
            // fill the per-lqid writeback tables for every member. ldqid the VPU
            // assigns (vec_autogen.ldqid) increments by 1 per member and echoes on

            for (int m=0; m<8; m++) begin
              if (m < dst_nm) begin                        // dst_nm result beats
                tag_by_lqid    [(id_vpu_uop_packet.ldqid + m) % LQ_DEPTH] <= next_iss_tag;
                // B3b: scalar-dest routes to INT RF, except vfmv.f.s (OPFVV,
                // funct3=0b001) which routes to the FP RF. autogen_v_scalar_dest
                // fires for both OPMVV and OPFVV funct6=0b010000, so distinguish
                // on funct3 here.
                dstkind_by_lqid[(id_vpu_uop_packet.ldqid + m) % LQ_DEPTH] <= id_vpu_uop_packet.scalar_dest ? 
                                                                          ((next_iss_insn[14:12] == 3'b001) ? CII_DST_FP : CII_DST_INT) 
                                                                          : CII_DST_VEC;
                dstoff_by_lqid [(id_vpu_uop_packet.ldqid + m) % LQ_DEPTH] <= m;
                last_by_lqid   [(id_vpu_uop_packet.ldqid + m) % LQ_DEPTH] <= (m == (dst_nm-1));
              end
            end
            iss_state <= S_IDLE;
          end
        end
        default: iss_state <= S_IDLE;
      endcase


    end
  end


  always_ff @( posedge clk ) begin 
    if (iss_state == S_WAIT) begin
      receive_data_cnt         <=  6'd0; 
    end else begin
      if (dat_fifo_valid) begin
        receive_data_cnt <= receive_data_cnt + 1'b1;

        // One dat beat = the 4 lanes {VS1, VS2, VS3, VM} for member `receive_data_cnt` (the
        // beats return in request order, so the beat index IS the member index).
        // Store each lane into its per-member operand array; lanes that carried
        // CII_SRC_NONE (source shorter than max_src_beats) write a member tt_vec
        // never reads, so no gating is needed.
        //
        // lane 0: VS1 vector, OR the .vx/.vf scalar (a single beat -> member 0 only;
        //         later beats on lane 0 are NONE, so capture the scalar at receive_data_cnt==0).
        if (req_is_scalar) begin
          if (receive_data_cnt == '0) begin
             next_iss_scalar <= dat_fifo_data[0].rsp_dat[63:0];
          end

        end else begin
          opnd_p0[receive_data_cnt] <= dat_fifo_data[0].rsp_dat;
        end
        // lane 1: VS2  (wide -> up to 2*NM members)
        opnd_p1[receive_data_cnt] <= dat_fifo_data[1].rsp_dat;
        // lane 2: VS3 / old-dest
        opnd_p2[receive_data_cnt] <= dat_fifo_data[2].rsp_dat;
        // lane 3: v0 mask -- single register (member 0 only)
        if (receive_data_cnt == '0) opnd_vm <= dat_fifo_data[3].rsp_dat;
      end
    end
  end



  /////////////////////////////////////////////////////////////////////////////
  // Drive the cii interface
  /////////////////////////////////////////////////////////////////////////////

  // issue channel (RECEIVER): return 1 credit per FIFO pop.
  assign cii_intf.iss_credit      = iss_pop;
  // present the operand-staged instruction to tt_id.
  assign read_issue_inst          = next_iss_insn;
  assign read_issue_scalar_opnd   = next_iss_scalar;   // .vx/.vf scalar (0 for .vv/.vi)
  assign read_issue_state         = tt_briscv_pkg::inst_state_e'(0);
  assign read_valid               = (iss_state == S_HOLD);
  assign read_issue_sb_id         = {1'b0, next_iss_tag};
  // decoded CSR config from the issue packet.
  assign csr_de0.v_vsew           = next_iss_vtype.vsew;
  assign csr_de0.v_lmul           = next_iss_vtype.vlmul;
  assign csr_de0.v_vxrm           = next_iss_vxrm;
  assign csr_de0.v_vl             = next_iss_vl;
  assign csr_de0.v_vstart         = next_iss_vstart;
  assign csr_de0.frm              = next_iss_frm;




  // A 4-lane beat is driven this cycle iff we are walking members (S_REQ), the op
  // has at least one source, and a send credit is available. req_valid tracks this;
  // the per-lane src_en_* below only pick real-source vs CII_SRC_NONE within the beat.
  assign req_send = (iss_state == S_REQ) && (max_src_beats != 5'd0) && (req_credit_cnt != 0);


  always_comb begin
    // check is vs1 must read from int or fp regfile
    scalar_src = id_vpu_uop_packet.sel_scalar || id_vpu_uop_packet.fp_sel_scalar;

    // Enable this source's lane while its member index is still in range. Members
    // are 0..num_beats-1, so the test is `next_src_beat < num_beats` (strict).
    src_en_vs1  = (id_vpu_uop_packet.rf_rden0 || scalar_src) && (next_src_beat < src_vs1_num_beats);  // VS1 or scalar rs1
    src_en_vs2  = id_vpu_uop_packet.rf_rden1  && (next_src_beat < src_vs2_num_beats);  // VS2
    src_en_vs3  = need_old_dest               && (next_src_beat < src_vs3_num_beats);  // VS3 (3rd src / RMW / undisturbed old-dest)
    src_en_vm   = vm_needed                   && (next_src_beat == 0);        // VM (v0 mask)
  end


  always_comb begin
    src_vs1_num_beats = scalar_src ? 5'd1              : lmul_beats;    // VS1 / scalar
    src_vs2_num_beats = vs2_wide   ? (lmul_beats << 1) : lmul_beats; // VS2
    src_vs3_num_beats = dst_nm;                          // VS3 = dest group
    src_vm_num_beats  = 5'd1;                            // VM single register
  end

  // Member walk length = max member count over the ENABLED sources (a disabled
  // source must not extend the walk). VM contributes a single beat when needed.
  always_comb begin
    max_src_beats = 5'd0;
    if ((id_vpu_uop_packet.rf_rden0 || scalar_src) && src_vs1_num_beats > max_src_beats)
      max_src_beats = src_vs1_num_beats;
    if (id_vpu_uop_packet.rf_rden1 && src_vs2_num_beats > max_src_beats)
      max_src_beats = src_vs2_num_beats;
    if (need_old_dest && src_vs3_num_beats > max_src_beats)
      max_src_beats = src_vs3_num_beats;
    if (vm_needed && src_vm_num_beats > max_src_beats)
      max_src_beats = src_vm_num_beats;
  end

  // req channel (SENDER): lane 0 to 3
  assign cii_intf.req_valid       = req_send;
  assign cii_intf.req_data[0]     = '{tag           : next_iss_tag, 
                                      rsp_src_id    : (src_en_vs1 ? (scalar_src ? CII_SRC_SCALAR : CII_SRC_VS1) : CII_SRC_NONE), 
                                      rsp_src_offset: next_src_beat};

  assign cii_intf.req_data[1]     = '{tag           : next_iss_tag, 
                                      rsp_src_id    : (src_en_vs2 ? CII_SRC_VS2 : CII_SRC_NONE),
                                      rsp_src_offset: next_src_beat};

  // Slot 3 (VS3) and slot 6 (STALE_VD) name DIFFERENT groups on the host: VS3 is
  // the explicitly encoded third source (pvs3), STALE_VD is the old vd
  // (stale_pvdest). Only rf_rden2 means the datapath truly reads a third
  // register operand -- and there pvs3 and stale_pvdest coincide, so VS3 is
  // right. Every other need_old_dest case (vta=0, masked with vma=0,
  // vec_single_reg) encodes NO third source and is a merge of the old dest, so
  // it must ask for STALE_VD; asking for VS3 there returns pvs3, which for a
  // non-RMW op is not the old dest and yields a garbage tail.
  assign cii_intf.req_data[2]     = '{tag           : next_iss_tag,
                                      rsp_src_id    : (~src_en_vs3                        ? CII_SRC_NONE     :
                                                       id_vpu_uop_packet.rf_rden2         ? CII_SRC_VS3_VD   :
                                                                                            CII_SRC_STALE_VD),
                                      rsp_src_offset: next_src_beat};

  assign cii_intf.req_data[3]     = '{tag           : next_iss_tag, 
                                      rsp_src_id    : (src_en_vm ? CII_SRC_VM  : CII_SRC_NONE),
                                      rsp_src_offset: next_src_beat};



  // dat channel (RECEIVER): pop throughout fetch (drain concurrently with the
  // request stream so the dat FIFO never backs up when NM*sources > credit depth);
  // the receive logic above routes each returned beat into its operand register.
  assign cii_intf.dat_credit      = dat_pop;   // return 1 credit per src-data pop

  // ---- operand feed to tt_vec (replaces the staging-regfile combinational read).
  // tt_vec presents its per-iteration member read address on
  // id_vpu_uop_packet.rf_addrp{0,1,2} (= source base + member, incremented by tt_id
  // via i_iterate_addrp*). Convert to a member index by subtracting the source base
  // decoded from the raw instruction, and index the per-member operand arrays. VM
  // (v0) is a single register.
  wire [4:0] rd_mem_p0 = id_vpu_uop_packet.rf_addrp0 - next_iss_insn[19:15]; // VS1 base
  wire [4:0] rd_mem_p1 = id_vpu_uop_packet.rf_addrp1 - next_iss_insn[24:20]; // VS2 base
  wire [4:0] rd_mem_p2 = id_vpu_uop_packet.rf_addrp2 - next_iss_insn[11:7];  // VS3/old-dest base (vd)
  assign vrf_p0_rddata  = opnd_p0[rd_mem_p0[CII_MEMBER_W-1:0]];
  assign vrf_p1_rddata  = opnd_p1[rd_mem_p1[CII_MEMBER_W-1:0]];
  assign vrf_p2_rddata  = opnd_p2[rd_mem_p2[CII_MEMBER_W-1:0]];
  assign vrf_vm0_rddata = opnd_vm;









  /////////////////////////////////////////////////////////////////////////////
  // WriteBack Control Logic 
  /////////////////////////////////////////////////////////////////////////////


  // (C3 operand pull is merged into the C1+C3 prefetch FSM above -- operands are
  //  fetched into the staging regfile BEFORE the instruction is presented to
  //  tt_id, so no operands_ready gate on the tt_id<->tt_vec handshake.)
  // =========================================================================
  // C4: writeback. wb = SENDER (drive wb_valid/wb_data + a credit counter fed by
  // wb_credit). Priority-select one of tt_vec's result ports (1c > 2c > 3c > div),
  // map its lqid -> CII tag + dst_kind + dst member offset + last, and emit a wb
  // beat. Each LMUL member produces one result beat (lqid = base_ldqid + member);
  // the per-lqid tables filled at accept give this beat's wb_dst_offset and the
  // last (final-member) marker. fflags come from the result exception.
  // debug_wb_vec_* mirror it for the cosim commit trace.
  //
  // !! PENDING (skeleton): if two result ports assert the same cycle (only across
  // different in-flight ops -- one op's members share a latency class), the
  // priority mux drops the loser; serializing concurrent results into the single
  // wb lane, and holding a result when wb_credit==0, need a small result buffer.
  // =========================================================================

  always_comb begin
    wb_result_valid = vex_mem_lqvld_1c | vex_mem_lqvld_2c | vex_mem_lqvld_3c | vex_mem_lqvld_div;
    wb_result_data  = vex_mem_lqdata_div; 
    wb_ldqid        = vex_mem_lqid_div; 
    wb_fp_flags     = vex_mem_lqexc_div;

    if (vex_mem_lqvld_3c) begin 
      wb_result_data  = vex_mem_lqdata_3c; 
      wb_ldqid        = vex_mem_lqid_3c; 
      wb_fp_flags     = vex_mem_lqexc_3c; 
    end

    if (vex_mem_lqvld_2c) begin 
      wb_result_data  = vex_mem_lqdata_2c; 
      wb_ldqid        = vex_mem_lqid_2c; 
      wb_fp_flags     = vex_mem_lqexc_2c; 
    end
    
    if (vex_mem_lqvld_1c) begin 
      wb_result_data  = vex_mem_lqdata_1c; 
      wb_ldqid        = vex_mem_lqid_1c; 
      wb_fp_flags     = vex_mem_lqexc_1c; 
    end
  end



  // A beat is sent this cycle when either the VPU produced a result OR we are
  // merging old-dest members, and a WB credit is available. Merge takes priority
  // on the single WB lane (the vec_single_reg op's own VPU result already fired
  // member 0 before the merge starts, so there is no self-conflict).
  assign wb_fire = (wbx_active || wb_result_valid) && (wb_credit_cnt != 0);

  assign cii_intf.wb_valid   = wb_fire;
  assign cii_intf.wb_data[0] = wb_send_merge ?
    // Merge beat: write undisturbed old-dest member `wbx_member` (last on NM-1).
    '{ inst_tag      : wbx_tag,
       wb_data       : wb_olddest[wbx_member[CII_MEMBER_W-1:0]],
       wb_dst_offset : wbx_member[CII_MEMBER_W-1:0],
       wb_wr_en      : 1'b1,
       wb_fp_flags   : '{ last     : (wbx_member == wbx_nm - 1'b1),
                          dst_kind : CII_DST_VEC,
                          vxsat    : 1'b0,
                          fflags   : '0 } }
    :
    // VPU-result beat (member 0 for vec_single_reg; every member for normal ops).
    '{ inst_tag      : tag_by_lqid[wb_ldqid],
       wb_data       : wb_result_data,
       wb_dst_offset : dstoff_by_lqid[wb_ldqid],
       wb_wr_en      : 1'b1,
       wb_fp_flags   : '{ last     : last_by_lqid[wb_ldqid],
                          dst_kind : dstkind_by_lqid[wb_ldqid],
                          vxsat    : 1'b0,
                          fflags   : {wb_fp_flags.fpNV,
                                      wb_fp_flags.fpDZ,
                                      wb_fp_flags.fpOF,
                                      wb_fp_flags.fpUF,
                                      wb_fp_flags.fpNX} } };

  /////////////////////////////////////////////////////////////////////////////
  // Old-dest merge FSM: for a vec_single_reg op (VPU writes only member 0), write
  // back the remaining group members 1..NM-1 from the undisturbed old dest.
  //   S_HOLD accept  -> latch NM/tag/base-ldqid + snapshot old dest (opnd_p2).
  //   member-0 WB    -> start merging (wbx_active), member = 1.
  //   each merge beat (with credit) -> advance; last on member NM-1 completes the
  //                     group (its `last` bit drives the host group-done).
  // Emitting member 0 first then 1..NM-1 (last) guarantees all members are written
  // before the group-done fires.
  /////////////////////////////////////////////////////////////////////////////
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      wbx_pending <= 1'b0;
      wbx_active  <= 1'b0;
    end else begin
      // Snapshot the op + old dest when tt_id accepts a multi-member vec_single_reg.
      if ((iss_state == S_HOLD) && id_ready_to_receive && vec_single_reg && (dst_nm > 5'd1)) begin
        wbx_pending    <= 1'b1;
        wbx_nm         <= dst_nm;
        wbx_tag        <= next_iss_tag;
        wbx_base_ldqid <= id_vpu_uop_packet.ldqid;
        for (int m = 0; m < 8; m++) wb_olddest[m] <= opnd_p2[m];
      end
      // Kick off merging once this op's member-0 (VPU) result is emitted.
      if (wbx_pending && wb_fire && !wbx_active && (wb_ldqid == wbx_base_ldqid)) begin
        wbx_pending <= 1'b0;
        wbx_active  <= 1'b1;
        wbx_member  <= 5'd1;
      end
      // Advance one merge member per emitted beat.
      if (wbx_active && (wb_credit_cnt != 0)) begin
        if (wbx_member == wbx_nm - 1'b1) wbx_active <= 1'b0;
        wbx_member <= wbx_member + 1'b1;
      end
    end
  end

  /////////////////////////////////////////////////////////////////////////////
  // END WriteBack Control Logic 
  /////////////////////////////////////////////////////////////////////////////


  // Debug commit trace (cosim): member 0 of the dest group.
  assign debug_wb_vec_valid = wb_fire;
  assign debug_wb_vec_wdata = {{(VLEN*8-VLEN){1'b0}}, wb_result_data};
  assign debug_wb_vec_wmask = 8'h01;

  /////////////////////////////////////////////////////////////////////////////
  // Instatiate the Input CII FIFOs
  /////////////////////////////////////////////////////////////////////////////

  // ISSUE CHANNEL FIFO
  tt_cii_fifo #(
    .T            (type(cii_intf.iss_data)), 
    .DEPTH        (CII_N_ISS_CREDITS      )
  ) u_iss_fifo (
    .clk          (clk                    ),
    .rst_n        (reset_n                ),
    .push         (cii_intf.iss_valid     ), 
    .push_data    (cii_intf.iss_data      ),
    .pop          (iss_pop                ), 
    .pop_data     (iss_fifo_data          ), 
    .full         (                       ), // we dont check for this because Host should track credits  
    .empty        (iss_fifo_empty         )
  );
  
  // SRC DATA RETURN CHANNEL FIFO
  tt_cii_fifo #(
    .T            (type(cii_intf.dat_data)), 
    .DEPTH        (CII_N_DAT_CREDITS      )
  ) u_dat_fifo (
    .clk          (clk                    ), 
    .rst_n        (reset_n                ),
    .push         (cii_intf.dat_valid     ), 
    .push_data    (cii_intf.dat_data      ),
    .pop          (dat_pop                ), 
    .pop_data     (dat_fifo_data          ), 
    .full         (                       ), // we dont check for this because Host should track credits 
    .empty        (dat_fifo_empty         )
  );


  /////////////////////////////////////////////////////////////////////////////
  // Instatiate the VPU Top
  /////////////////////////////////////////////////////////////////////////////
  tt_id #(
    .LQ_DEPTH(LQ_DEPTH),
    .LQ_DEPTH_LOG2(LQ_DEPTH_LOG2), 
    .EXP_WIDTH(EXP_WIDTH),
    .MAN_WIDTH(MAN_WIDTH),
    .FLEN(FLEN),
    .VLEN(VLEN),
    .FP_RF_RD_PORTS(FP_RF_RD_PORTS),
    .INCL_VEC(INCL_VEC),
    .INCL_FP(INCL_FP)
  ) id (
    .i_clk                                 (clk),    
    .i_reset_n                             (reset_n), 

    // Dispatch signals
    .dispatch_sb_id                        ('0),
    .dispatch_next_senior                  (1'b0),
    .dispatch_kill                         (1'b0),

    .i_csr                                 (csr_de0),             
    .o_csr                                 (csr_ex0),  
    .o_id_rf_vex_p0                        (rf_vex_p0),
    .o_id_rf_vex_p1                        ( ),
    .o_id_fprf_vex_p0                      (fprf_vex_p0),

    .i_if_instrn                           (read_issue_inst),       
    .i_if_pc                               ('0),           
    .i_if_scalar_opnd                      (read_issue_scalar_opnd),
    .i_if_instrn_rts                       (read_valid),    
    .o_id_instrn_rtr                       (id_ready_to_receive),    

    .o_id_type                             (id_type),          
    .o_id_immed_op                         ( ),     

    // Vector Interface (ungated: operands are prefetched into staging before issue)
    .o_id_vex_rts                          (id_vex_rts),
    .i_vex_id_rtr                          (vex_id_rtr),
    .o_vec_autogen                         (id_vpu_uop_packet),   
    .o_id_vex_lqid                         ( ), 
    .i_vex_id_incr_addrp2                  (vex_id_incr_addrp2),    
    .o_v_vm                                (v_vm              ),    

    // EX Interface
    .i_div_resource_busy                   (vex_div_busy),
    .o_id_ex_rts                           ( ),             
    .i_ex_rtr                              (1'b1 ),    
    .o_id_ex_pc                            ( ),        
    .o_id_ex_instrn                        (id_ex_instrn),    
    .o_id_ex_lqid                          ( ), 
    .o_id_ex_vecldst                       ( ),         
    .o_id_ex_Zb_instr                      ( ),   
    .o_id_ex_units_rts                     ( ),       
    .o_id_ex_instdisp                      ( ),
    .o_vecldst_autogen                     ( ), 
    .o_id_ex_last                          ( ),  
    .i_ex_dst_vld_1c                       (1'b0),         
    .i_ex_dst_lqid_1c                      ('0), 
    .i_ex_fwd_data_1c                      ('0),  
    .i_ex_dst_vld_2c                       (1'b0),         
    .i_ex_dst_lqid_2c                      ('0), 
    .i_ex_fwd_data_2c                      ('0),  

    // Integer RegFile Interface  
    .o_rf_wr_flag                          ( ),    
    .o_rf_wraddr                           ( ),    

    // FP RegFile Interface    
    .o_fp_rf_wr_flag                       ( ),    
    .o_fp_rf_wraddr                        ( ),    

    // Mem Interface
    .i_lq_broadside_info                   ('0),     
    .o_id_mem_lqinfo                       ( ),         
    .o_id_replay                           (id_replay),             
    .o_id_mem_lqalloc                      ( ),        
    .o_id_mem_lq_done                      ( ),        
    .i_mem_dst_vld                         ('0), 
    .i_mem_dst_lqid                        ('0), 
    .i_mem_fwd_data                        ('0), 
    .i_mem_lq_op                           ('0),        
    .i_mem_lq_commit                       ('0),         
    .i_lq_broadside_data                   ('0),     
    .i_lq_broadside_valid                  ('0), 
    .i_lq_broadside_data_valid             ('0), 

    // Misc
    .i_iterate_addrp0                      (iterate_addrp0),   
    .i_iterate_addrp1                      (iterate_addrp1),   
    .i_iterate_addrp2                      (iterate_addrp2),   
    .i_ignore_lmul                         (ignore_lmul),           
    .i_ignore_dstincr                      (ignore_dstincr),        
    .i_ignore_srcincr                      (ignore_srcincr),        
    .i_mem_fe_lqfull                       (1'b0),         
    .i_mem_fe_lqempty                      ('0),
    .i_mem_fe_skidbuffull                  ('0),    
    .i_mem_id_lqnxtid                      ('0),

    .o_is_whole_memop                      ( ),
    .o_is_masked_memop                     ( ),
    .o_is_indexldst                        ( ),
    .o_is_maskldst                         ( ),
    .i_if_sb_id                            (read_issue_sb_id),
    .o_id_sb_id                            ( ),

    .i_if_state                            (read_issue_state),
    .o_id_state                            ( )
  );



  tt_vec_top #(
    .VLEN(VLEN),
    .XLEN(64  )
  ) vecu (
    .i_clk                 (clk                   ), 
    .i_reset_n             (reset_n               ), 
    .i_csr                 (csr_ex0               ), // vector CSR bundle sampled at EX0 (vtype/vl/vstart/frm/...)
    .i_v_vm                (v_vm                  ), // instr[25] mask-enable bit: 0 masked op, 1 unmasked
    .i_id_vec_autogen      (id_vpu_uop_packet     ), // decoded VPU uop packet (vec_autogen_s decode bundle)
    .o_sat_csr             (vpu_sat_csr           ), // sticky saturation flag back to CSR (vxsat)
    // ID Interface
    .o_vex_div_busy        (vex_div_busy          ), // vector divide unit is occupied; ID must not issue another divide
    .i_id_vex_rts          (id_vex_rts            ), // "ready-to-send": ID has a valid vector uop; transfer fires when qualified by o_vex_id_rtr
    .o_vex_id_rtr          (vex_id_rtr            ), // VEX ready-to-receive: back-pressure to ID (0 = stall issue)
    .i_id_ex_vecldst       ('0                    ), // this uop is a vector load/store (routes to the mem path)
    .i_id_ex_instrn        (id_ex_instrn          ), // the 32-bit vector instruction being issued
    .i_id_replay           (id_replay             ), // 0 first cycle of a fresh vector instruction. // 1 a replay/iteration cycle of the LMUL>1 op already in flight.
    .i_id_type             (id_type               ), // operand/instruction type encoding (OPIVV/OPIVX/OPIVI/OPMVV/...)
    .o_vex_id_incr_addrp2  (vex_id_incr_addrp2    ), // tell ID to bump read-port-2 reg address for the next iteration
    .o_iterate_addrp0      (iterate_addrp0        ), // per-iteration source reg address, read port 0
    .o_iterate_addrp1      (iterate_addrp1        ), // per-iteration source reg address, read port 1
    .o_iterate_addrp2      (iterate_addrp2        ), // per-iteration source reg address, read port 2
    .o_ignore_lmul         (ignore_lmul           ), // override the LMUL-driven iteration count (op needs a fixed count)
    .o_ignore_dstincr      (ignore_dstincr        ), // hold the destination reg address across iterations (no dst increment)
    .o_ignore_srcincr      (ignore_srcincr        ), // hold source reg address across iterations (mask-producing ops: vmand.mm, viota, vid, vmsof...)
    // RegFile Interface
    .i_rf_vex_p0           (rf_vex_p0             ), // integer RF read data (int->vec moves); aligns with 0a, read pre-flop
    .i_fprf_vex_p0         (fprf_vex_p0           ), // FP RF read data (fp->vec moves); aligns with 0a, read pre-flop
    .i_vrf_p0_rddata       (vrf_p0_rddata         ), // VRF read port 0 data (src operand)
    .i_vrf_p1_rddata       (vrf_p1_rddata         ), // VRF read port 1 data (src operand)
    .i_vrf_p2_rddata       (vrf_p2_rddata         ), // VRF read port 2 data (src operand / old destination for RMW)
    .i_vrf_vm0_rddata      (vrf_vm0_rddata        ), // VRF mask register v0 read data
    // Mem Interface Write Back
    .o_vex_mem_lqvld_1c    (vex_mem_lqvld_1c      ), // load-queue writeback valid, pipe stage 1c
    .o_vex_mem_lqdata_1c   (vex_mem_lqdata_1c     ), // load-queue writeback result data, 1c
    .o_vex_mem_lqexc_1c    (vex_mem_lqexc_1c      ), // load-queue writeback FP exception flags, 1c
    .o_vex_mem_lqid_1c     (vex_mem_lqid_1c       ), // load-queue entry id being written back, 1c
    .o_vex_mem_lqvld_2c    (vex_mem_lqvld_2c      ), // load-queue writeback valid, pipe stage 2c
    .o_vex_mem_lqdata_2c   (vex_mem_lqdata_2c     ), // load-queue writeback result data, 2c
    .o_vex_mem_lqexc_2c    (vex_mem_lqexc_2c      ), // load-queue writeback FP exception flags, 2c
    .o_vex_mem_lqid_2c     (vex_mem_lqid_2c       ), // load-queue entry id being written back, 2c
    .o_vex_mem_lqvld_3c    (vex_mem_lqvld_3c      ), // load-queue writeback valid, pipe stage 3c
    .o_vex_mem_lqdata_3c   (vex_mem_lqdata_3c     ), // load-queue writeback result data, 3c
    .o_vex_mem_lqexc_3c    (vex_mem_lqexc_3c      ), // load-queue writeback FP exception flags, 3c
    .o_vex_mem_lqid_3c     (vex_mem_lqid_3c       ), // load-queue entry id being written back, 3c
    .o_vex_mem_lqvld_div   (vex_mem_lqvld_div     ), // division writeback path: valid
    .o_vex_mem_lqdata_div  (vex_mem_lqdata_div    ), // division writeback path: result data
    .o_vex_mem_lqexc_div   (vex_mem_lqexc_div     ), // division writeback path: FP exception flags
    .o_vex_mem_lqid_div    (vex_mem_lqid_div      ), // division writeback path: load-queue entry id

    // C2: tail-/mask-agnostic policy from the CII issue vtype (the VPU applies it).
    .i_vta                 (next_iss_vtype.vta    ),
    .i_vma                 (next_iss_vtype.vma    )
  );

endmodule
