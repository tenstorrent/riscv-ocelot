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


  /////////////////////////////////////////////////////////////////////////////
  // CII coprocessor scaffolding + internal nets (Track C, C0 slice 1)
  /////////////////////////////////////////////////////////////////////////////

  // ---- Issue path (driven by the CII issue channel in C1; tied off later) ----
  logic [4:0]                       dispatch_sb_id;
  logic                             dispatch_next_senior, dispatch_kill;
  logic [31:0]                      read_issue_inst;
  logic [63:0]                      read_issue_scalar_opnd;
  logic                             read_valid;
  logic                             ocelot_read_req;
  logic [4:0]                       read_issue_sb_id;
  tt_briscv_pkg::inst_state_e       read_issue_state;

  // ---- CSR config (from the issue packet in C2; tied off later) ----
  tt_briscv_pkg::csr_t              csr_de0;
  tt_briscv_pkg::csr_t              csr_ex0;

  // ---- tt_id <-> tt_vec datapath control ----
  logic [63:0]                      rf_vex_p0, rf_vex_p1, fprf_vex_p0;
  logic [4:0]                       id_type;
  logic [31:0]                      id_immed_op;
  logic                             id_vex_rts, vex_id_rtr;
  tt_briscv_pkg::vec_autogen_s      id_vec_autogen;
  logic [LQ_DEPTH_LOG2-1:0]         id_vex_lqid;
  logic                             vex_id_incr_addrp2;
  logic                             v_vm;
  logic                             vex_div_busy;
  logic                             id_ex_rts, ex_id_rtr;
  logic [31:0]                      id_ex_pc, id_ex_instrn;
  logic [LQ_DEPTH_LOG2-1:0]         id_ex_lqid;
  logic                             id_ex_vecldst;
  logic [4:0]                       id_ex_Zb_instr;
  logic                             id_ex_units_rts, id_ex_instdisp, id_ex_last;
  tt_briscv_pkg::vecldst_autogen_s  id_ex_vecldst_autogen;
  logic                             id_replay;
  logic [4:0]                       iterate_addrp0, iterate_addrp1, iterate_addrp2;
  logic                             ignore_lmul, ignore_dstincr, ignore_srcincr;

  // ---- Legacy EX/MEM forwarding + LQ (tied off -- the CII owns mem/wb) ----
  logic                             ex_dst_vld_1c, ex_dst_vld_2c;
  logic [LQ_DEPTH_LOG2-1:0]         ex_dst_lqid_1c, ex_dst_lqid_2c;
  logic [31:0]                      ex_fwd_data_1c, ex_fwd_data_2c;
  logic                             id_rf_wr_flag, id_fp_rf_wr_flag;
  logic [4:0]                       id_rf_wraddr, id_fp_rf_wraddr;
  tt_briscv_pkg::arr_lq_info_s      lq_broadside_info;
  logic [LQ_DEPTH-1:0][31:0]        lq_broadside_data;
  logic [LQ_DEPTH-1:0]              lq_broadside_valid, lq_broadside_data_valid;
  tt_briscv_pkg::lq_info_s          id_mem_lqinfo;
  logic                             id_mem_lqalloc, id_mem_lq_done;
  logic                             mem_fe_lqfull;
  logic [LQ_DEPTH_LOG2-1:0]         mem_id_lqnxtid;
  logic                             id_is_whole_memop, id_is_masked_memop, id_is_indexldst, id_is_maskldst;
  logic [4:0]                       id_sb_id;
  tt_briscv_pkg::inst_state_e       id_mem_state;

  // ---- Staging VRF <-> tt_vec ----
  logic [VLEN-1:0]                  vrf_p0_rddata, vrf_p1_rddata, vrf_p2_rddata, vrf_vm0_rddata;
  logic                             mem_vrf_wr;
  logic [4:0]                       mem_vrf_wraddr;
  logic [VLEN-1:0]                  mem_vrf_wrdata;
  logic                             ocelot_sat_csr;

  // ---- tt_vec result ports -> CII writeback (C4) ----
  logic                             vex_mem_lqvld_1c, vex_mem_lqvld_2c, vex_mem_lqvld_3c, vex_mem_lqvld_div;
  logic [VLEN-1:0]                  vex_mem_lqdata_1c, vex_mem_lqdata_2c, vex_mem_lqdata_3c, vex_mem_lqdata_div;
  tt_briscv_pkg::csr_fp_exc         vex_mem_lqexc_1c, vex_mem_lqexc_2c, vex_mem_lqexc_3c, vex_mem_lqexc_div;
  logic [LQ_DEPTH_LOG2-1:0]         vex_mem_lqid_1c, vex_mem_lqid_2c, vex_mem_lqid_3c, vex_mem_lqid_div;

  // ---- Staging vector register file (per-instruction operand/result buffer) ----
  // CII operand-pull fills it (C3); the datapath reads it via the decoded source
  // addresses; results drain to CII writeback (C4). Read addresses come from the
  // decoded vec_autogen source fields. The write port is tied off in this slice.
  tt_vec_regfile #(.VLEN(VLEN)) vrf (
    .i_clk       (clk),
    .i_reset_n   (reset_n),
    .i_rden_0a   ({id_vec_autogen.rf_rden2, id_vec_autogen.rf_rden1, id_vec_autogen.rf_rden0}),
    .i_rdaddr_0a ({id_vec_autogen.rf_addrp2, id_vec_autogen.rf_addrp1, id_vec_autogen.rf_addrp0}),
    .i_wren_0a   (mem_vrf_wr),
    .i_wraddr_0a (mem_vrf_wraddr),
    .i_wrdata_0a (mem_vrf_wrdata),
    .o_rddata_0a ({vrf_p2_rddata, vrf_p1_rddata, vrf_p0_rddata}),
    .o_dstmask_0a (),
    .o_vm0_0a    (vrf_vm0_rddata)
  );

  // =========================================================================
  // C1: CII Issue channel -> tt_id. The coprocessor is the issue RECEIVER
  // (non-FWFT): assert iss_credit to pop; the popped beat's iss_valid + iss_data
  // arrive registered one cycle later. Latch it, present the instruction to
  // tt_id via the rts/rtr handshake, and drive the decoded CSR config from the
  // extended issue packet.
  //   I_IDLE (1-cycle pop request) -> I_WAIT (await popped beat) -> I_HOLD (rts)
  //
  // NOTE (validate w/ host + sim): iss_credit returns a host credit per pop,
  // incl. popping an empty FIFO (spurious idle credits). Popping once per accept
  // keeps the return rate ~ accept rate; confirm against the host issue sender.
  // =========================================================================
  // Approach (a): PREFETCH operands before presenting the instruction to tt_id,
  // so no entry-handshake gate is needed (the earlier operands_ready gate
  // deadlocked tt_id's accept through units_rtr). We decode the source regs from
  // the raw instruction, fetch vs1/vs2/vs3/v0 into the staging regfile via CII
  // req/dat, THEN release to tt_id -- staging is filled before tt_vec reads it.
  //   S_IDLE(pop) -> S_WAIT(latch) -> S_REQ(send 4 reqs) -> S_DRAIN(recv 4 dat
  //   -> staging) -> S_HOLD(rts to tt_id)
  // SKELETON: over-fetches all 4 sources at member 0 for every op (per-op source
  // set + EMUL member walk are future work).
  typedef enum logic [2:0] { S_IDLE, S_WAIT, S_REQ, S_DRAIN, S_HOLD } iss_state_e;
  iss_state_e             iss_state;
  logic [31:0]            pend_insn;
  cii_caracal_tag_t       pend_tag;
  cii_caracal_vtype_t     pend_vtype;
  logic [CII_VL_W-1:0]    pend_vl, pend_vstart;
  logic [1:0]             pend_vxrm;
  logic [2:0]             pend_frm;
  logic [2:0]             pf_idx;          // prefetch source index (0=VS1..3=VM)
  logic [2:0]             rcv_cnt;
  logic [4:0]             dst_q [0:3];     // staging write addr per request (in order)
  logic [2:0]             dst_wr, dst_rd;
  logic [$clog2(CII_N_REQ_CREDITS+1)-1:0] req_credit_cnt;
  // CII tag + dst-kind by the VPU lqid (results carry lqid), for writeback (C4).
  cii_caracal_tag_t       tag_by_lqid     [0:7];
  cii_caracal_dst_kind_e  dstkind_by_lqid [0:7];

  // Source op-id + staging address for the current prefetch index. tt_id maps
  // rf_addrp0=insn[19:15], rf_addrp1=insn[24:20], rf_addrp2=insn[11:7]; fill
  // staging at those so the datapath's reads hit the fetched data.
  cii_caracal_srcid_e src_id_sel;
  logic [4:0]         src_addr_sel;
  always_comb begin
    unique case (pf_idx)
      3'd0:    begin src_id_sel = CII_SRC_VS1; src_addr_sel = pend_insn[19:15]; end
      3'd1:    begin src_id_sel = CII_SRC_VS2; src_addr_sel = pend_insn[24:20]; end
      3'd2:    begin src_id_sel = CII_SRC_VS3; src_addr_sel = pend_insn[11:7];  end
      default: begin src_id_sel = CII_SRC_VM;  src_addr_sel = 5'd0;             end
    endcase
  end

  wire req_send = (iss_state == S_REQ) && (req_credit_cnt != 0);

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      iss_state<=S_IDLE; pf_idx<=3'd0; rcv_cnt<=3'd0; dst_wr<=3'd0; dst_rd<=3'd0;
      req_credit_cnt <= CII_N_REQ_CREDITS;
    end else begin
      if      ( cii_intf.req_credit && !req_send) req_credit_cnt <= req_credit_cnt + 1'b1;
      else if (!cii_intf.req_credit &&  req_send) req_credit_cnt <= req_credit_cnt - 1'b1;
      case (iss_state)
        S_IDLE: iss_state <= S_WAIT;                  // iss_credit asserted this cycle
        S_WAIT:
          if (cii_intf.iss_valid) begin
            pend_insn<=cii_intf.iss_data[0].instr.insn; pend_tag<=cii_intf.iss_data[0].tag;
            pend_vtype<=cii_intf.iss_data[0].instr.vtype;
            pend_vl<=cii_intf.iss_data[0].instr.vl; pend_vstart<=cii_intf.iss_data[0].instr.vstart;
            pend_vxrm<=cii_intf.iss_data[0].instr.vxrm; pend_frm<=cii_intf.iss_data[0].instr.frm;
            pf_idx<=3'd0; rcv_cnt<=3'd0; dst_wr<=3'd0; dst_rd<=3'd0;
            iss_state<=S_REQ;
          end else iss_state<=S_IDLE;
        S_REQ:
          if (req_credit_cnt != 0) begin              // a request is sent this cycle
            dst_q[dst_wr]<=src_addr_sel; dst_wr<=dst_wr+1'b1;
            if (pf_idx==3'd3) iss_state<=S_DRAIN; else pf_idx<=pf_idx+1'b1;
          end
        S_DRAIN: if (rcv_cnt==3'd4) iss_state<=S_HOLD; // all 4 operands staged
        S_HOLD:
          if (ocelot_read_req) begin                  // tt_id accepted the instruction
            // key by the lqid tt_id actually assigns (vec_autogen.ldqid); it echoes
            // back on the result port (o_id_vex_lqid is unused/undriven in this build).
            tag_by_lqid[id_vec_autogen.ldqid]     <= pend_tag;
            dstkind_by_lqid[id_vec_autogen.ldqid] <= id_vec_autogen.scalar_dest ? CII_DST_INT : CII_DST_VEC;
            iss_state <= S_IDLE;
          end
        default: iss_state <= S_IDLE;
      endcase
      if (cii_intf.dat_valid) begin rcv_cnt<=rcv_cnt+1'b1; dst_rd<=dst_rd+1'b1; end
    end
  end

  // issue channel (RECEIVER): one pop request per pass.
  assign cii_intf.iss_credit = (iss_state == S_IDLE);
  // present the operand-staged instruction to tt_id.
  assign read_issue_inst  = pend_insn;
  assign read_valid       = (iss_state == S_HOLD);
  assign read_issue_sb_id = {1'b0, pend_tag};
  // decoded CSR config from the issue packet.
  assign csr_de0.v_vsew   = pend_vtype.vsew;
  assign csr_de0.v_lmul   = pend_vtype.vlmul;
  assign csr_de0.v_vxrm   = pend_vxrm;
  assign csr_de0.v_vl     = pend_vl;
  assign csr_de0.v_vstart = pend_vstart;
  assign csr_de0.frm      = pend_frm;
  // req channel (SENDER): lane 0 = current source request, lane 1 = NONE.
  assign cii_intf.req_valid   = req_send;
  assign cii_intf.req_data[0] = '{tag:pend_tag, rsp_src_id:src_id_sel,  rsp_src_offset:'0};
  assign cii_intf.req_data[1] = '{tag:pend_tag, rsp_src_id:CII_SRC_NONE, rsp_src_offset:'0};
  // dat channel (RECEIVER): pop while draining; write returned data into staging.
  assign cii_intf.dat_credit  = (iss_state == S_DRAIN);
  assign mem_vrf_wr           = cii_intf.dat_valid;
  assign mem_vrf_wraddr       = dst_q[dst_rd];
  assign mem_vrf_wrdata       = cii_intf.dat_data[0].rsp_dat;

  // ---- remaining tie-offs: legacy inputs + null req/dat/wb (real glue C2-C4) ----
  assign dispatch_sb_id       = '0;
  assign dispatch_next_senior = 1'b0;
  assign dispatch_kill        = 1'b0;
  assign read_issue_scalar_opnd = '0;
  assign read_issue_state       = tt_briscv_pkg::inst_state_e'(0);
  assign ex_id_rtr              = 1'b1;
  assign ex_dst_vld_1c = 1'b0; assign ex_dst_lqid_1c = '0; assign ex_fwd_data_1c = '0;
  assign ex_dst_vld_2c = 1'b0; assign ex_dst_lqid_2c = '0; assign ex_fwd_data_2c = '0;
  assign lq_broadside_info       = '0;
  assign lq_broadside_data       = '0;
  assign lq_broadside_valid      = '0;
  assign lq_broadside_data_valid = '0;
  assign mem_fe_lqfull           = 1'b0;
  assign mem_id_lqnxtid          = '0;
  // (C3 operand pull is merged into the C1+C3 prefetch FSM above -- operands are
  //  fetched into the staging regfile BEFORE the instruction is presented to
  //  tt_id, so no operands_ready gate on the tt_id<->tt_vec handshake.)
  // =========================================================================
  // C4: writeback. wb = SENDER (drive wb_valid/wb_data + a credit counter fed by
  // wb_credit). Priority-select one of tt_vec's result ports (1c > 2c > 3c > div),
  // map its lqid -> CII tag + dst_kind, and emit a wb beat. fflags come from the
  // result exception. debug_wb_vec_* mirror it for the cosim commit trace.
  //
  // !! STRUCTURAL, PENDING FUNCTIONAL SIM (host loopback): wb_dst_offset=0 and
  // last=1 assume a single-member result (matches the C3 member-0 skeleton). The
  // real per-member dst_offset + last (final member of the group) and serializing
  // concurrent result ports into the single wb lane need sim validation.
  // =========================================================================
  logic                     res_v;
  logic [VLEN-1:0]          res_data;
  logic [LQ_DEPTH_LOG2-1:0] res_lqid;
  tt_briscv_pkg::csr_fp_exc res_exc;
  always_comb begin
    res_v    = vex_mem_lqvld_1c | vex_mem_lqvld_2c | vex_mem_lqvld_3c | vex_mem_lqvld_div;
    res_data = vex_mem_lqdata_div; res_lqid = vex_mem_lqid_div; res_exc = vex_mem_lqexc_div;
    if (vex_mem_lqvld_3c) begin res_data=vex_mem_lqdata_3c; res_lqid=vex_mem_lqid_3c; res_exc=vex_mem_lqexc_3c; end
    if (vex_mem_lqvld_2c) begin res_data=vex_mem_lqdata_2c; res_lqid=vex_mem_lqid_2c; res_exc=vex_mem_lqexc_2c; end
    if (vex_mem_lqvld_1c) begin res_data=vex_mem_lqdata_1c; res_lqid=vex_mem_lqid_1c; res_exc=vex_mem_lqexc_1c; end
  end

  logic [$clog2(CII_N_WB_CREDITS+1)-1:0] wb_credit_cnt;
  wire wb_fire = res_v && (wb_credit_cnt != 0);
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n)                                 wb_credit_cnt <= CII_N_WB_CREDITS;
    else if ( cii_intf.wb_credit && !wb_fire)     wb_credit_cnt <= wb_credit_cnt + 1'b1;
    else if (!cii_intf.wb_credit &&  wb_fire)     wb_credit_cnt <= wb_credit_cnt - 1'b1;
  end

  assign cii_intf.wb_valid   = wb_fire;
  assign cii_intf.wb_data[0] = '{
    inst_tag      : tag_by_lqid[res_lqid],
    wb_data       : res_data,
    wb_dst_offset : '0,                          // SKELETON: member 0 (see note)
    wb_wr_en      : 1'b1,
    wb_fp_flags   : '{ last     : 1'b1,          // SKELETON: single-member
                       dst_kind : dstkind_by_lqid[res_lqid],
                       vxsat    : 1'b0,
                       fflags   : {res_exc.fpNV,res_exc.fpDZ,res_exc.fpOF,res_exc.fpUF,res_exc.fpNX} } };
  assign cii_intf.wb_data[1] = '{default:'0};    // lane 1 unused (host reads lane 0)

  // Debug commit trace (cosim): member 0 of the dest group.
  assign debug_wb_vec_valid = wb_fire;
  assign debug_wb_vec_wdata = {{(VLEN*8-VLEN){1'b0}}, res_data};
  assign debug_wb_vec_wmask = 8'h01;

  /////////////////////////////////////////////////////////////////////////////


















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
    .dispatch_sb_id                        (dispatch_sb_id),
    .dispatch_next_senior                  (dispatch_next_senior),
    .dispatch_kill                         (dispatch_kill),

    .i_csr                                 (csr_de0),             
    .o_csr                                 (csr_ex0),  
    .o_id_rf_vex_p0                        (rf_vex_p0),
    .o_id_rf_vex_p1                        (rf_vex_p1),
    .o_id_fprf_vex_p0                      (fprf_vex_p0),

    .i_if_instrn                           (read_issue_inst),       
    .i_if_pc                               ('0),           
    .i_if_scalar_opnd                      (read_issue_scalar_opnd),
    .i_if_instrn_rts                       (read_valid),    
    .o_id_instrn_rtr                       (ocelot_read_req),    

    .o_id_type                             (id_type),          
    .o_id_immed_op                         (id_immed_op),     

    // Vector Interface (ungated: operands are prefetched into staging before issue)
    .o_id_vex_rts                          (id_vex_rts),
    .i_vex_id_rtr                          (vex_id_rtr),
    .o_vec_autogen                         (id_vec_autogen),   
    .o_id_vex_lqid                         (id_vex_lqid), 
    .i_vex_id_incr_addrp2                  (vex_id_incr_addrp2),    
    .o_v_vm                                (v_vm              ),    

    // EX Interface
    .i_div_resource_busy                   (vex_div_busy),
    .o_id_ex_rts                           (id_ex_rts),             
    .i_ex_rtr                              (ex_id_rtr         ),    
    .o_id_ex_pc                            (id_ex_pc),        
    .o_id_ex_instrn                        (id_ex_instrn),    
    .o_id_ex_lqid                          (id_ex_lqid[LQ_DEPTH_LOG2-1:0]), 
    .o_id_ex_vecldst                       (id_ex_vecldst),         
    .o_id_ex_Zb_instr                      (id_ex_Zb_instr[4:0]),   
    .o_id_ex_units_rts                     (id_ex_units_rts),       
    .o_id_ex_instdisp                      (id_ex_instdisp),
    .o_vecldst_autogen                     (id_ex_vecldst_autogen), 
    .o_id_ex_last                          (id_ex_last),  
    .i_ex_dst_vld_1c                       (ex_dst_vld_1c),         
    .i_ex_dst_lqid_1c                      (ex_dst_lqid_1c), 
    .i_ex_fwd_data_1c                      (ex_fwd_data_1c),  
    .i_ex_dst_vld_2c                       (ex_dst_vld_2c),         
    .i_ex_dst_lqid_2c                      (ex_dst_lqid_2c), 
    .i_ex_fwd_data_2c                      (ex_fwd_data_2c),  

    // Integer RegFile Interface  
    .o_rf_wr_flag                          (id_rf_wr_flag     ),    
    .o_rf_wraddr                           (id_rf_wraddr      ),    

    // FP RegFile Interface    
    .o_fp_rf_wr_flag                       (id_fp_rf_wr_flag  ),    
    .o_fp_rf_wraddr                        (id_fp_rf_wraddr   ),    

    // Mem Interface
    .i_lq_broadside_info                   (lq_broadside_info),     
    .o_id_mem_lqinfo                       (id_mem_lqinfo),         
    .o_id_replay                           (id_replay),             
    .o_id_mem_lqalloc                      (id_mem_lqalloc),        
    .o_id_mem_lq_done                      (id_mem_lq_done),        
    .i_mem_dst_vld                         ('0          ), 
    .i_mem_dst_lqid                        ('0         ), 
    .i_mem_fwd_data                        ('0         ), 
    .i_mem_lq_op                           ('0),        
    .i_mem_lq_commit                       ('0),         
    .i_lq_broadside_data                   (lq_broadside_data),     
    .i_lq_broadside_valid                  (lq_broadside_valid), 
    .i_lq_broadside_data_valid             (lq_broadside_data_valid), 

    // Misc
    .i_iterate_addrp0                      (iterate_addrp0),   
    .i_iterate_addrp1                      (iterate_addrp1),   
    .i_iterate_addrp2                      (iterate_addrp2),   
    .i_ignore_lmul                         (ignore_lmul),           
    .i_ignore_dstincr                      (ignore_dstincr),        
    .i_ignore_srcincr                      (ignore_srcincr),        
    .i_mem_fe_lqfull                       (mem_fe_lqfull),         
    .i_mem_fe_lqempty                      ('0),
    .i_mem_fe_skidbuffull                  ('0),    
    .i_mem_id_lqnxtid                      (mem_id_lqnxtid[LQ_DEPTH_LOG2-1:0]),

    .o_is_whole_memop                      (id_is_whole_memop),
    .o_is_masked_memop                     (id_is_masked_memop),
    .o_is_indexldst                        (id_is_indexldst),
    .o_is_maskldst                         (id_is_maskldst),
    .i_if_sb_id                            (read_issue_sb_id),
    .o_id_sb_id                            (id_sb_id),

    .i_if_state                            (read_issue_state),
    .o_id_state                            (id_mem_state)
  );



  tt_vec_top #(
    .VLEN(VLEN),
    .XLEN(64  )
  ) vecu (
    .i_clk                 (clk),                 
    .i_reset_n             (reset_n), 
    .i_csr                 (csr_ex0),             
    .i_v_vm                (v_vm),                  
    .i_id_vec_autogen      (id_vec_autogen),        
    .o_sat_csr             (ocelot_sat_csr),
    // ID Interface
    .o_vex_div_busy        (vex_div_busy),
    .i_id_vex_rts          (id_vex_rts),   // ungated: operands prefetched into staging
    .o_vex_id_rtr          (vex_id_rtr),
    .i_id_ex_vecldst       (id_ex_vecldst),         
    .i_id_ex_instrn        (id_ex_instrn),    
    .i_id_replay           (id_replay),             
    .i_id_type             (id_type),          
    .o_vex_id_incr_addrp2  (vex_id_incr_addrp2),    
    .o_iterate_addrp0      (iterate_addrp0),   
    .o_iterate_addrp1      (iterate_addrp1),   
    .o_iterate_addrp2      (iterate_addrp2),   
    .o_ignore_lmul         (ignore_lmul),           
    .o_ignore_dstincr      (ignore_dstincr),        
    .o_ignore_srcincr      (ignore_srcincr),        
    // RegFile Interface
    .i_rf_vex_p0           (rf_vex_p0),       
    .i_fprf_vex_p0         (fprf_vex_p0),
    .i_vrf_p0_rddata       (vrf_p0_rddata),  
    .i_vrf_p1_rddata       (vrf_p1_rddata),  
    .i_vrf_p2_rddata       (vrf_p2_rddata),  
    .i_vrf_vm0_rddata      (vrf_vm0_rddata),
    // // Mem Interface
    // .o_vex_mem_lqvld_1c    (vex_mem_lqvld_1c),      
    // .o_vex_mem_lqdata_1c   (vex_mem_lqdata_1c), 
    // .o_vex_mem_lqexc_1c    (vex_mem_lqexc_1c),      
    // .o_vex_mem_lqid_1c     (vex_mem_lqid_1c), 
    // .o_vex_mem_lqvld_2c    (vex_mem_lqvld_2c),      
    // .o_vex_mem_lqdata_2c   (vex_mem_lqdata_2c), 
    // .o_vex_mem_lqexc_2c    (vex_mem_lqexc_2c),      
    // .o_vex_mem_lqid_2c     (vex_mem_lqid_2c), 
    // .o_vex_mem_lqvld_3c    (vex_mem_lqvld_3c),      
    // .o_vex_mem_lqdata_3c   (vex_mem_lqdata_3c), 
    // .o_vex_mem_lqexc_3c    (vex_mem_lqexc_3c),      
    // .o_vex_mem_lqid_3c     (vex_mem_lqid_3c), 
    // Division connections
    .o_vex_mem_lqvld_div   (vex_mem_lqvld_div),      
    .o_vex_mem_lqdata_div  (vex_mem_lqdata_div), 
    .o_vex_mem_lqexc_div   (vex_mem_lqexc_div),      
    .o_vex_mem_lqid_div    (vex_mem_lqid_div), 
    .i_mem_vrf_wr          (mem_vrf_wr),
    .i_mem_vrf_wraddr      (mem_vrf_wraddr),
    .i_mem_vrf_wrdata      (mem_vrf_wrdata),
    .i_mem_ex_rtr          (1'b1),
    // C2: tail-/mask-agnostic policy from the CII issue vtype (the VPU applies it).
    .i_vta                 (pend_vtype.vta),
    .i_vma                 (pend_vtype.vma)
  );


// ************************ //
// Write back to registers  //
// ************************ //




endmodule
