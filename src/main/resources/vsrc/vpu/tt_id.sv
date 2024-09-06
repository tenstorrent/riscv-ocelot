// See LICENSE.TT for license details.
`include "briscv_defines.h"
`include "tt_briscv_pkg.vh"
`include "autogen_defines.h"

module tt_id #(parameter LQ_DEPTH=tt_briscv_pkg::LQ_DEPTH, LQ_DEPTH_LOG2=3, EXP_WIDTH=8, MAN_WIDTH=23, FLEN=32, VLEN=128, FP_RF_RD_PORTS=4, INCL_VEC=0, INCL_FP=0)
(
   input 				     i_clk ,
   input 				     i_reset_n ,

   // IF interface
   input [31:0] 			     i_if_instrn ,
   input [31:0] 			     i_if_pc ,
   input 				     i_if_instrn_rts ,
   output wire 				     o_id_instrn_rtr ,
   output logic 			     o_id_replay,
   // EX interface
   input 				     i_ex_rtr ,
   output wire 				     o_id_ex_rts ,
   output wire [31:0] 			     o_id_ex_pc ,
   output wire [31:0] 			     o_id_ex_instrn ,
   output wire [LQ_DEPTH_LOG2-1:0] 	     o_id_ex_lqid ,
   output wire 				     o_id_ex_vecldst,
   output wire [4:0] 			     o_id_ex_Zb_instr,
   output wire 				     o_id_ex_units_rts ,
   input 				     tt_briscv_pkg::csr_t i_csr,
   output                                    tt_briscv_pkg::csr_t o_csr,
   output logic     o_id_ex_last,

   //VEC Interface
   input 				     i_vex_id_rtr ,
   output wire 				     o_id_vex_rts ,
   output wire [LQ_DEPTH_LOG2-1:0] 	     o_id_vex_lqid ,
   input [4:0] 				     i_iterate_addrp0,
   input [4:0] 				     i_iterate_addrp1,
   input [4:0] 				     i_iterate_addrp2,
   input 				     i_vex_id_incr_addrp2,
   input 				     i_ignore_lmul,
   input 				     i_ignore_dstincr,
   input 				     i_ignore_srcincr,
   // VEC deocde signals
   output 				     o_v_vm,
   output 				     tt_briscv_pkg::vec_autogen_s o_vec_autogen,
   output 				     tt_briscv_pkg::vecldst_autogen_s o_vecldst_autogen,
   // MATRIX decode
   output wire 				     o_id_matrix_rts ,

   // Destination target registers for forwarding and RAW checking
   input 				     tt_briscv_pkg::arr_lq_info_s i_lq_broadside_info,
   input [LQ_DEPTH-1:0][31:0] 		     i_lq_broadside_data,
   input [LQ_DEPTH-1:0] 		     i_lq_broadside_valid,
   input [LQ_DEPTH-1:0] 		     i_lq_broadside_data_valid,

   // ID <--> MEM signals
   input logic 				     i_mem_fe_lqfull,
   input logic 				     i_mem_fe_lqempty,
   input logic 				     i_mem_fe_skidbuffull,
   input logic [LQ_DEPTH_LOG2-1:0] 	     i_mem_id_lqnxtid,
   output logic 			     o_id_mem_lqalloc,
   output logic 			     o_id_mem_lq_done,
   output 				     tt_briscv_pkg::lq_info_s o_id_mem_lqinfo,

   input logic 				     i_ex_dst_vld_1c, // forwarding control from EX
   input logic [LQ_DEPTH_LOG2-1:0] 	     i_ex_dst_lqid_1c, // forwarding control from EX
   input logic [31:0] 			     i_ex_fwd_data_1c, // forwarding control from EX

   input logic 				     i_ex_dst_vld_2c, // forwarding control from EX
   input logic [LQ_DEPTH_LOG2-1:0] 	     i_ex_dst_lqid_2c, // forwarding control from EX
   input logic [31:0] 			     i_ex_fwd_data_2c, // forwarding control from EX

   input logic 				     i_mem_dst_vld, // forwarding control from MEM
   input logic [LQ_DEPTH_LOG2-1:0] 	     i_mem_dst_lqid, // forwarding control from MEM
   input logic [31:0] 			     i_mem_fwd_data, // forwarding control from MEM
 
   input [6:0] 				     i_mem_lq_op ,
   input 				     i_mem_lq_commit ,

   // Instruction type decode info to send down the pipe
   output reg [4:0] 			     o_id_type , // Instruction type and Ext

   // Immediate operand for consumption in EX
   output wire [31:0] 			     o_id_immed_op ,

   // Register file read/write enables and addresses
   output logic 			     o_rf_wr_flag ,
   output reg [ 4:0] 			     o_rf_wraddr,
   output logic 			     o_fp_rf_wr_flag, 
   output reg [ 4:0] 			     o_fp_rf_wraddr,
   
   output 				     o_id_ex_instdisp,

   output logic       o_is_whole_memop,
   output logic       o_is_masked_memop,
   output logic       o_is_indexldst,
   output logic       o_is_maskldst,
   input logic  [4:0] i_if_sb_id,
   output logic [4:0] o_id_sb_id
);

wire v_ext;
wire matrix_ext;
wire matrix_mrf_wr_c;
wire matrix_opacc;

wire i_ext, m_ext, a_ext, b_ext;
wire f_ext;
wire illegal_op;
wire [4:0] EncType;
wire id_rts;
wire id_replay;

wire  [`EX_AUTOGEN_WIDTH-1:0] ex_autogen;
wire  valid_ex_autogen = (i_ext | m_ext | b_ext);
wire  [`FP_AUTOGEN_WIDTH-1:0] fp_autogen;
wire  valid_fp_autogen = f_ext;
assign o_fp_autogen = fp_autogen & {`FP_AUTOGEN_WIDTH{valid_fp_autogen}};
tt_briscv_pkg::vec_autogen_s vec_autogen;
tt_briscv_pkg::vec_autogen_s vec_autogen_replay;
tt_briscv_pkg::vec_autogen_s vec_autogen_incr;
tt_briscv_pkg::vecldst_autogen_s vecldst_autogen;
tt_briscv_pkg::vecldst_autogen_s vecldst_autogen_replay;
tt_briscv_pkg::vecldst_autogen_s vecldst_autogen_incr;
wire  valid_vec_autogen = v_ext;
tt_briscv_pkg::vec_autogen_s vec_autogen_nodbg;
assign vec_autogen_nodbg = (id_replay ? vec_autogen_replay : vec_autogen)
                         & {$bits(tt_briscv_pkg::vec_autogen_s){valid_vec_autogen | matrix_ext}};
always @* begin
  o_vec_autogen = vec_autogen_nodbg;
  o_vec_autogen.rf_addrp0 = vec_autogen_nodbg.rf_addrp0;
end

assign o_vecldst_autogen = (id_replay ? vecldst_autogen_replay : vecldst_autogen)
                           & {$bits(tt_briscv_pkg::vecldst_autogen_s){valid_vec_autogen}};
assign o_id_replay = id_replay;
wire [2:0] v_vsew;
wire [2:0] v_lmul;
wire lmul_gt1 = (~v_lmul[2] & |v_lmul[1:0]);
wire lmul_gteq1 = ~v_lmul[2];   
wire ldst_gt1 = |vecldst_autogen.ldst_iterations;
// Temp defines. These want to be primary outputs eventually
//reg        o_rf_p2_rden;
//reg [ 4:0] o_rf_p2_rdaddr;
logic [ 4:0] o_rf_p3_rdaddr;

wire raw_hazard_stall;
wire raw_hazard_stall_vex; // Vector Execution pipe doesn't use forwarding path, use this customized stall condition to avoid a slow path due to false dependency
wire raw_hazard_stall_fwd; // Other pipe needs to take forward into account
logic sync_stall;
logic [6:0] sync_stall_op;
wire [31:0] instrn_id;
wire units_rtr = (i_ex_rtr & i_vex_id_rtr);
// IMPROVE(Ashok). Mem rtr gets factored in separately, so for that reason
// FP/VEX rtr can always be held high
// EX RTL will be low for div and vec ldst. See if this can be optimized
logic [31:0] instrn_id_replay;
logic [4:0]  id_sb_id_replay;
logic [31:0] id_ex_pc_replay;
tt_briscv_pkg::csr_t id_csr_replay;
logic [$clog2(VLEN):0] id_replay_cnt_start;
logic  [5:0] lmul_replay_cnt;
logic  [2:0] id_replay_type_start;
logic  [2:0] id_replay_type;
logic fp_ldst_vld, vec_ldst_vld, vec_ldst_idx_vld;
logic [4:0] mask_rf_addrp0,mask_rf_addrp1,mask_rf_addrp2;   
logic vec_extby2_instrn,vec_extby4_instrn;
wire is_ex_instrn;
wire valid_ex_instrn;
wire is_fp_instrn;
wire valid_fp_instrn;
wire is_vec_instrn;
wire valid_vec_instrn;
wire valid_matrix_instrn;
wire valid_opacc_instrn;

reg o_id_type_r; // register to register alu op
reg o_id_type_i; // loads, JALR and alu ops with immediate operands
reg o_id_type_s; // store
reg o_id_type_sb; // branch
reg o_id_type_u; // LUI and AUIPC - unsigned immediates
reg o_id_type_uj; // JAL - unsigned immediates
reg o_id_type_e; // System
reg o_id_type_f; // Fence

logic no_lq_load_pending;
   
`define HIGH_PERF_FETCH
`ifdef HIGH_PERF_FETCH
  assign instrn_id       = id_replay ? instrn_id_replay : i_if_instrn;
  assign o_id_sb_id      = id_replay ? id_sb_id_replay : i_if_sb_id;
  assign o_id_ex_pc      = id_replay ? id_ex_pc_replay : i_if_pc;
  assign o_csr           = id_replay ? id_csr_replay : i_csr;
 
  assign id_rts          = (i_if_instrn_rts | id_replay)
                            //reduction ops don't consume another lq entry so under replay its fine to ignore full conditions.
                            //todo: do i need to add is_matrix_instrn below?
                         & (~(i_mem_fe_lqfull | i_mem_fe_skidbuffull) | (is_vec_instrn & id_replay & ~vec_autogen_replay.addrp2_incr[0]))
                            // Syncstall only applies per ldqid, and some vector replays need multiple dispatches to fill a single ldqid
                         & (id_replay | ~sync_stall )
                            // Need all units to be ready to dispatch anything
                         & (is_ex_instrn | is_fp_instrn | is_vec_instrn | matrix_ext) & units_rtr;
  assign o_id_instrn_rtr = units_rtr & ~id_replay & ~raw_hazard_stall & ~(i_mem_fe_lqfull | i_mem_fe_skidbuffull) & ~sync_stall;
`else
tt_rts_rtr_pipe_stage #(.WIDTH(32)) id_instrn_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_rts     (i_if_instrn_rts ), // input side handshake
   .o_rtr     (o_id_instrn_rtr ), // input side handshake
   .i_rtr     ((units_rtr) & (~raw_hazard_stall) ), // if EX is ready to accept next instruction and no raw hazard requiring a stall was detected
   .o_rts     (id_rts       ), // output side handshake
   .i_data    (i_if_instrn     ),
   .o_data    (instrn_id       )
);
tt_rts_rtr_pipe_stage #(.WIDTH(32)) id_pc_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_rts     (i_if_instrn_rts ), // input side handshake
/* verilator lint_off PINCONNECTEMPTY */
   .o_rtr     ( ),
   .o_rts     ( ),
/* verilator lint_on PINCONNECTEMPTY */
   .i_rtr     ((units_rtr)  & (~raw_hazard_stall) ), // if EX is ready to accept next instruction and no raw hazard requiring a stall was detected
   .i_data    (i_if_pc         ),
   .o_data    (o_id_ex_pc      )
);
tt_rts_rtr_pipe_stage #(.WIDTH($bits(tt_briscv_pkg::csr_t))) id_csr_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_rts     (i_if_instrn_rts ), // input side handshake
/* verilator lint_off PINCONNECTEMPTY */
   .o_rtr     ( ),
   .o_rts     ( ),
/* verilator lint_on PINCONNECTEMPTY */
   .i_rtr     ((units_rtr)  & (~raw_hazard_stall) ), // if EX is ready to accept next instruction and no raw hazard requiring a stall was detected
   .i_data    (i_csr),
   .o_data    (o_csr)
);
`endif

// Capture the LQ ID in a flop and use it during replay
logic [LQ_DEPTH_LOG2-1:0] id_lqid;
tt_pipe_stage #(.WIDTH(LQ_DEPTH_LOG2)) ex_vld_flop   
(
   .i_clk     (i_clk            ),
   .i_reset_n (i_reset_n        ),
   .i_en      (o_id_mem_lqalloc ),
   .i_d       (i_mem_id_lqnxtid ),
   .o_q       (id_lqid          )
);
   
// Squash LQ write flag for some vector instructions when VL is 0
logic squash_vec_wr_flag;
   
logic id_mem_lqalloc_raw; // Less qualified for timing reason
assign id_mem_lqalloc_raw = id_rts & (~id_replay | vec_autogen_replay.addrp2_incr[0]);

// LDQ Information for dispatch ops
assign o_id_mem_lqalloc = id_mem_lqalloc_raw & ~raw_hazard_stall;
assign o_id_mem_lq_done = ((o_id_vex_rts && i_vex_id_rtr) ||
                                          o_id_matrix_rts ||
                           (o_id_ex_rts  && i_ex_rtr    )   ) && (vec_autogen_incr.replay_cnt == 0);
assign o_id_mem_lqinfo.pc[31:0] = o_id_ex_pc;
assign o_id_mem_lqinfo.sim_instrn[31:0] = instrn_id;
assign o_id_mem_lqinfo.mrf_opacc = o_vec_autogen.mrf_opacc;
assign o_id_mem_lqinfo.mrf_wr_c = o_vec_autogen.mrf_wr_c;
assign o_id_mem_lqinfo.vrf_wr_flag = o_vec_autogen.rf_wren;
assign o_id_mem_lqinfo.squash_vec_wr_flag = squash_vec_wr_flag;
assign o_id_mem_lqinfo.fp_rf_wr_flag = (is_fp_instrn | is_vec_instrn) & o_fp_rf_wr_flag;
assign o_id_mem_lqinfo.rf_wr_flag = (is_ex_instrn | is_fp_instrn | is_vec_instrn) & o_rf_wr_flag;
assign o_id_mem_lqinfo.rf_wraddr[4:0] = ({5{o_rf_wr_flag}} &  o_rf_wraddr[4:0]) |
                                        ({5{o_fp_rf_wr_flag}} &  o_fp_rf_wraddr[4:0]) |
                                        ({5{o_vec_autogen.mrf_wr_c | o_vec_autogen.mrf_wr_c}} & o_vec_autogen.rf_addrp2[4:0]) |
                                        ({5{o_vec_autogen.rf_wren}} & o_vec_autogen.rf_addrp2[4:0]);
assign o_id_mem_lqinfo.is_branch = o_id_type_sb;
assign o_id_mem_lqinfo.load = (is_ex_instrn & (EncType[4:0] == `BRISCV_INSTR_TYPE_I) & ~instrn_id[4] & ~instrn_id[5]) |
			      (is_ex_instrn & (EncType[4:0] == `BRISCV_INSTR_TYPE_A)) |
                              (is_fp_instrn & (EncType[4:0] == `BRISCV_INSTR_TYPE_IF)) |
                              (is_vec_instrn & o_vecldst_autogen.load);
assign o_id_mem_lqinfo.vec_load = (is_vec_instrn & o_vecldst_autogen.load);
assign o_id_mem_lqinfo.vl_is_zero = ~|o_csr.v_vl;

assign o_id_ex_lqid = id_mem_lqalloc_raw ? i_mem_id_lqnxtid : id_lqid;
// assign o_id_vex_lqid = i_mem_id_lqnxtid;

wire vsetOp_to_ex;
wire instr_dispatch_complete;
// Decode instruction type
wire [1:0] SrcA;
wire [1:0] SrcB;
wire [1:0] SrcC;
wire rs1_used;
wire rs2_used;
wire rs3_used;
wire [1:0] Dest;

wire [6:0] opcode = instrn_id[6:0];
wire [4:0] addrp2 = instrn_id[11:7];
wire [2:0] funct3 = instrn_id[14:12];
wire [4:0] addrp0 = instrn_id[19:15];
wire [4:0] addrp1 = instrn_id[24:20];
wire [4:0] addrp3 = instrn_id[31:27];
wire [6:0] funct7 = instrn_id[31:25];
wire       vm     = instrn_id[25]; // Only needed for V-Ext
wire [1:0] mop    = instrn_id[27:26];
wire [2:0] nf     = instrn_id[31:29]; // Only needed for V-Ext   
assign o_is_whole_memop = instrn_id[24:20] == `BRISCV_VECLDST_WHOLE_REGISTER && instrn_id[6:0] == 7'b0100111 && instrn_id[27:26] == 2'b00;
assign o_is_masked_memop = (vecldst_autogen.load | vecldst_autogen.store) && !instrn_id[25];
assign o_is_indexldst = (vecldst_autogen.load | vecldst_autogen.store) && mop[0];
assign o_is_maskldst = vecldst_autogen.ldst_mask;

if (INCL_VEC == 1) begin
// Logic to replay instructions for these cases -
// - iterate over Vec elements - Max 16 = vlen_max=128/sew_min=8
// - widening ops - dest Hi/Lo
// - narrowing ops - dest Hi/Lo
// - LMUL > 1 
// - Ldst > 1

 always @(posedge i_clk) begin
   if(~i_reset_n) begin
      instrn_id_replay <= '0;
      id_sb_id_replay <= '0;
      id_ex_pc_replay <= '0;
      id_csr_replay <= '0;
      id_replay_type <= `BRISCV_REPLAY_TYPE_NONE;
      vec_autogen_replay <= '0;
      vecldst_autogen_replay <= '0;
    end
   else if (id_replay) begin			// Already in Replay mode
     if (id_rts & ~raw_hazard_stall) begin
	// Repeate these - keep same value
        // instrn_id_replay <= instrn_id_replay;
        // id_ex_pc_replay <= id_ex_pc_replay;
        // id_replay_type <= id_replay_type;
        vec_autogen_replay <= vec_autogen_incr;
        vecldst_autogen_replay <= vecldst_autogen_incr;
     end
   end
   else if (valid_vec_instrn | valid_matrix_instrn) begin				// Start replay mode
      instrn_id_replay <= instrn_id;
      id_sb_id_replay <= o_id_sb_id;
      id_ex_pc_replay <= o_id_ex_pc;
      id_csr_replay <= o_csr;
      vec_autogen_replay <= vec_autogen_incr;
      vecldst_autogen_replay <= vecldst_autogen_incr;
      id_replay_type <= id_replay_type_start;
   end
 end
   
always @* begin
   if(~i_reset_n) begin
        id_replay_type_start = '0;
        id_replay_cnt_start = '0;
        vec_autogen_incr = '0;
        vecldst_autogen_incr = '0;
   end
   else begin
      vec_autogen_incr = vec_autogen;
      vecldst_autogen_incr = vecldst_autogen;
      if (id_replay) begin					// Increment dest/index in Replay mode
         vecldst_autogen_incr.ldst_iter_cnt = vecldst_autogen_replay.ldst_iter_cnt + 1'b1;
         vec_autogen_incr.addrp0_incr  = {vec_autogen_replay.addrp0_incr[0],vec_autogen_replay.addrp0_incr[7:1]}; // Rotate by 1
         vec_autogen_incr.addrp1_incr  = {vec_autogen_replay.addrp1_incr[0],vec_autogen_replay.addrp1_incr[7:1]}; // Rotate by 1
         vec_autogen_incr.addrp1_reset = {vec_autogen_replay.addrp1_reset[0],vec_autogen_replay.addrp1_reset[7:1]}; // Rotate by 1
         vec_autogen_incr.addrp2_incr = {vec_autogen_replay.addrp2_incr[0],vec_autogen_replay.addrp2_incr[7:1]}; // Rotate by 1
         vec_autogen_incr.replay_cnt = vec_autogen_replay.replay_cnt - 1'b1;

         if(vec_autogen.iterate) begin
            // TODO: use ignore incr for opacc
            vec_autogen_incr.addrp2_incr = {7'd0,i_vex_id_incr_addrp2};   
            vec_autogen_incr.ldqid = vec_autogen_replay.ldqid + {{LQ_DEPTH_LOG2-1{1'b0}},i_vex_id_incr_addrp2};
            vec_autogen_incr.rf_addrp2 = i_iterate_addrp2[4:0];
            vec_autogen_incr.rf_addrp1 = i_iterate_addrp1[4:0];
            vec_autogen_incr.rf_addrp0 = i_iterate_addrp0[4:0];
         end else begin
            vec_autogen_incr.ldqid = vec_autogen_replay.ldqid + {{LQ_DEPTH_LOG2-1{1'b0}},(vec_autogen_incr.addrp2_incr[0])};
            vec_autogen_incr.rf_addrp2 = vec_autogen_replay.rf_addrp2 + {4'b0,vec_autogen_incr.addrp2_incr[0]};
            vec_autogen_incr.rf_addrp1 = (vec_autogen_incr.addrp1_reset[0] ? vec_autogen.rf_addrp1
                                                                           : vec_autogen_replay.rf_addrp1 + {4'b0,vec_autogen_incr.addrp1_incr[0]});
            vec_autogen_incr.rf_addrp0 = vec_autogen_replay.rf_addrp0 + {4'b0,vec_autogen_incr.addrp0_incr[0]};
         end

         // IMPROVE: Same info duplicated in both structs. Cleanup
         vecldst_autogen_incr.dest_reg = vec_autogen_incr.rf_addrp2;
         vecldst_autogen_incr.ldst_dest_incr = vec_autogen_incr.addrp2_incr;
         vecldst_autogen_incr.ldst_index_incr = vec_autogen_incr.addrp1_incr;

         id_replay_type_start = '0;
         id_replay_cnt_start = '0;
      end
      else if ((valid_vec_instrn | valid_matrix_instrn) & ~raw_hazard_stall) begin	// Start replay mode

         vecldst_autogen_incr.ldst_iter_cnt = 6'b1;
               
         if (vec_autogen.iterate) begin				// Case 1: Iterate over vector elements - Low perf
            id_replay_type_start = `BRISCV_REPLAY_TYPE_ITER;
            //id_replay_cnt_start = (v_vlmax << {'0,~vec_autogen.onecycle_iterate})- 1'b1;
            //replay atleast an integral multiple of lmul size, this is to write elements that are past VL to original dst value.
            id_replay_cnt_start = (((VLEN/8 >> v_vsew[1:0]) << ({2{~v_lmul[2]}} & v_lmul[1:0])) << {'0,~vec_autogen.onecycle_iterate})- 1'b1;	 		
            vec_autogen_incr.addrp2_incr = 8'h0;
            vec_autogen_incr.addrp1_incr = 8'h0;
            vec_autogen_incr.addrp1_reset = 8'h0;
            vec_autogen_incr.addrp1_reset = 8'h0;
            vec_autogen_incr.addrp0_incr = 8'h0;
         end else if (vec_autogen.vmvgrp) begin
            id_replay_type_start = `BRISCV_REPLAY_TYPE_ITER;
            id_replay_cnt_start = {5'd0,instrn_id[17:15]};			
            vec_autogen_incr.addrp2_incr = {2{instrn_id[17:15] == 3'd1}} | {4{instrn_id[17:15] == 3'd3}} | {8{instrn_id[17:15] == 3'd7}};	
            vec_autogen_incr.addrp1_incr = vec_autogen_incr.addrp2_incr;
            vec_autogen_incr.addrp1_reset = 8'h0;
            vec_autogen_incr.addrp0_incr = 8'h0;
         end else if (lmul_gteq1 & (vec_autogen.wdeop)) begin		// Case 2: Widening Vector Ops needs replay
            id_replay_type_start = `BRISCV_REPLAY_TYPE_WIDE;
            id_replay_cnt_start = {1 << v_lmul[1:0],1'b0}-1'b1;	//spyglass disable STARC05-2.10.3.2b_sb	// Set replay for widening to 2*LMUL

            //vec_autogen_incr.addrp2_incr =  {7'd0,v_lmul==2'b00} | {4'd0,{2{1'b0,v_lmul==2'b01}}} | {4{1'b0,v_lmul==2'b10}};
            //vec_autogen_incr.addrp0_incr =                         {5'd0,{v_lmul==2'b01},2'b00}   | {2'd0,{2{1'b0,v_lmul==2'b10}},2'b00};
            vec_autogen_incr.addrp0_incr =  (EncType[4:0] == `BRISCV_INSTR_TYPE_Vvv) ? 8'haa : 8'h00;

            vec_autogen_incr.addrp2_incr =  8'hff;
            if(vec_autogen.src1hw)
               vec_autogen_incr.addrp1_incr =  8'hff;
               //vec_autogen_incr.addrp1_incr = {7'd0,v_lmul==2'b00} | {4'd0,{2{1'b0,v_lmul==2'b01}}} | {4{1'b0,v_lmul==2'b10}};
            else
               vec_autogen_incr.addrp1_incr =   8'haa;
               //vec_autogen_incr.addrp1_incr =                        {5'd0,{v_lmul==2'b01},2'b00}   | {2'd0,{2{1'b0,v_lmul==2'b10}},2'b00};
         end
         else if (lmul_gteq1 & (vec_autogen.nrwop)) begin		// Case 2: Narrowing Vector Ops needs replay
            id_replay_type_start = `BRISCV_REPLAY_TYPE_WIDE;
            id_replay_cnt_start = {1 << v_lmul[1:0],1'b0}-1'b1;	//spyglass disable STARC05-2.10.3.2b_sb
      
            vec_autogen_incr.addrp2_incr =  8'haa;
            vec_autogen_incr.addrp1_incr =  8'hff;
            vec_autogen_incr.addrp0_incr = (EncType[4:0] == `BRISCV_INSTR_TYPE_Vvv) ? 8'haa : 8'h00;
            vec_autogen_incr.addrp1_reset =  8'h0;
         end 
         else if (ldst_gt1) begin					// Case 3: LD/ST EMUL*nf > 1 iteration
            id_replay_type_start = `BRISCV_REPLAY_TYPE_LDST;
            id_replay_cnt_start = {2'b0, (vecldst_autogen.ldst_iterations[5:0])};	// Set replay to Autogen Value
            vec_autogen_incr.addrp2_incr = vec_autogen.addrp2_incr[7:0];		//  Incr reg based on ldst autogen for incr value
            vec_autogen_incr.addrp1_incr = vec_autogen.addrp1_incr[7:0];		//  Incr reg based on ldst autogen for incr value
            vec_autogen_incr.addrp1_reset = vec_autogen.addrp1_reset[7:0];		//  Reset reg incr for segment-indexed ldst ops
         end
         else if (lmul_gt1 & ~vec_ldst_vld) begin					// Case 4: LMUL > 1 needs replay
            id_replay_type_start = `BRISCV_REPLAY_TYPE_LMUL;
            id_replay_cnt_start = (vsetOp_to_ex | i_ignore_lmul) ? 8'b0 : {2'b0, lmul_replay_cnt};			// 	Set replay to LMUL

            //todo: set to zero for opacc
            vec_autogen_incr.addrp2_incr = ((matrix_ext | vec_autogen.rf_wren)  & ~i_ignore_dstincr) ? 8'hff : 8'h00;		// Incr reg every time if valid
            vec_autogen_incr.addrp0_incr = (vec_autogen.rf_rden0 & ~i_ignore_srcincr 
                                          & (matrix_ext | (EncType[4:0] == `BRISCV_INSTR_TYPE_Vvv))) ? 8'hff : 8'h00;
            vec_autogen_incr.addrp1_incr = (vec_autogen.rf_rden1 & ~i_ignore_srcincr) ? 8'hff : 8'h00;
            //vec_autogen_incr.addrp0_reset = 8'h0; 
         end
         else begin						// DEFAULT CASE: NO Replay
            id_replay_type_start = '0;
            id_replay_cnt_start = '0;
            vec_autogen_incr.addrp2_incr = '0;
            vec_autogen_incr.addrp1_incr = '0;
            vec_autogen_incr.addrp1_reset = '0;
         end
         vec_autogen_incr.replay_cnt = id_replay_cnt_start;
         //todo: add matrix decode
         vec_autogen_incr.ldqid = vec_autogen.ldqid + {{LQ_DEPTH_LOG2-1{1'b0}},vec_autogen_incr.addrp2_incr[0]};

         if(vec_autogen.iterate) begin
            vec_autogen_incr.rf_addrp2 = i_iterate_addrp2[4:0];
            vec_autogen_incr.rf_addrp1 = i_iterate_addrp1[4:0];
            vec_autogen_incr.rf_addrp0 = i_iterate_addrp0[4:0];
         end else begin
            vec_autogen_incr.rf_addrp2 = vec_autogen.rf_addrp2 + {4'b0,vec_autogen_incr.addrp2_incr[0]};
            vec_autogen_incr.rf_addrp1 = vec_autogen.rf_addrp1 + {4'b0,vec_autogen_incr.addrp1_incr[0]};
            vec_autogen_incr.rf_addrp0 = vec_autogen.rf_addrp0 + {4'b0,vec_autogen_incr.addrp0_incr[0]};
         end
         vecldst_autogen_incr.dest_reg = vec_autogen_incr.rf_addrp2;
      end
      else begin
         id_replay_type_start = '0;
         id_replay_cnt_start = '0;
         vec_autogen_incr.addrp2_incr = '0;
         vec_autogen_incr.addrp1_incr = '0;
         vec_autogen_incr.addrp1_reset = '0;
         vec_autogen_incr.rf_addrp2 = '0;
         vec_autogen_incr.rf_addrp1 = '0;
         vec_autogen_incr.rf_addrp0 = '0;
         vecldst_autogen_incr.dest_reg = 5'b0;
         vecldst_autogen_incr.ldst_iter_cnt = 6'b0;
      end
   end
 end

 always @* begin
   casez(v_lmul[2:0])
     3'b000: lmul_replay_cnt = 6'h0; // lmul = 1. No Repeat
     3'b001: lmul_replay_cnt = 6'h1; // lmul = 2
     3'b010: lmul_replay_cnt = 6'h3; // lmul = 4
     3'b011: lmul_replay_cnt = 6'h7; // lmul = 8
    default: lmul_replay_cnt = 6'h0; // Assume lmul = 1
   endcase
 end
 assign id_replay = |vec_autogen_replay.replay_cnt;

 assign vec_ldst_vld = v_ext & (vecldst_autogen.load | vecldst_autogen.store);
 assign vec_ldst_idx_vld = vec_ldst_vld & vecldst_autogen.ldst_index;
			
 assign squash_vec_wr_flag =  is_vec_instrn                         && // A vector instruction
                             (o_csr.v_vl == 0)                && // With VL set to 0
                            !(EncType == `BRISCV_INSTR_TYPE_Vvi &&     // Except: vmv<nf>r
                              funct7[6:1] == 6'b100111            ) &&
                            !vecldst_autogen.ldst_whole_register;      //         vl<nf>r
                             

`ifdef SIM
   assign instr_dispatch_complete = ((vec_autogen_replay.replay_cnt == 1) & id_rts & ~raw_hazard_stall) | (~(|id_replay_cnt_start) & ~id_replay);
`endif
end
else begin	// No VEC INCL
  assign id_replay = 1'b0;
  assign instrn_id_replay = '0;
  assign id_ex_pc_replay = '0;
  assign id_csr_replay = '0;
  assign instr_dispatch_complete = '1;
  assign vec_ldst_vld = '0;
  assign squash_vec_wr_flag = '0;
  assign vec_autogen_replay = '0;
  assign vecldst_autogen_replay = '0;
  assign vec_ldst_idx_vld = '0;
end

assign fp_ldst_vld = fp_autogen[`FP_AUTOGEN_FP_LOAD_VLD] | fp_autogen[`FP_AUTOGEN_RF_STORE_RD_EN];
   
// INT instruction
wire valid_amo_instn = id_rts & a_ext;
wire valid_int_instn = id_rts & i_ext;
wire valid_mul_instn = id_rts & m_ext;
wire valid_bit_instn = id_rts & b_ext;
assign vsetOp_to_ex = v_ext & ((EncType == `BRISCV_INSTR_TYPE_C1) | (EncType == `BRISCV_INSTR_TYPE_C2) | (EncType == `BRISCV_INSTR_TYPE_C3));
assign is_ex_instrn =  (i_ext | m_ext | a_ext | b_ext | vsetOp_to_ex | vec_ldst_vld | fp_ldst_vld);
assign valid_ex_instrn =  id_rts & is_ex_instrn;
assign o_id_ex_rts = (!raw_hazard_stall) & valid_ex_instrn;
assign o_id_ex_instrn = instrn_id;
assign o_id_ex_vecldst = vec_ldst_vld;
assign o_id_ex_last = vec_autogen_incr.replay_cnt == 0;
//FP Instructions
assign is_fp_instrn = f_ext;
assign valid_fp_instrn = id_rts & is_fp_instrn;

//VEC Instructions
assign is_vec_instrn = v_ext;
assign valid_vec_instrn = id_rts & is_vec_instrn;
assign valid_matrix_instrn = id_rts & matrix_ext;
assign valid_opacc_instrn = id_rts & matrix_opacc;
assign o_id_vex_rts    = (!raw_hazard_stall_vex) & id_rts & v_ext & ~vec_ldst_vld & ~vsetOp_to_ex;
assign o_id_matrix_rts    = (!raw_hazard_stall_vex) & id_rts & matrix_ext & ~vec_ldst_vld & ~vsetOp_to_ex;;

assign o_id_ex_units_rts = (!raw_hazard_stall) & id_rts;
//////


//spyglass disable_block NamedAssoc
/* verilator lint_off UNUSED */
reg [63:0] OLD_id_ascii_instrn;
reg [127:0] id_ascii_instrn;
// Ascii Instruction
tt_ascii_instrn_decode ascii_decode ( .i_instrn(instrn_id), .o_ascii_instrn(OLD_id_ascii_instrn)); // DELETE THIS
autogen_Instruction autogen_Instruction ( opcode, funct7, funct3, vm, v_vsew, v_lmul, mop, addrp1, nf, id_ascii_instrn);
/* verilator lint_on UNUSED */

// Ext
// autogen_Ext autogen_Ext ( opcode, funct7, funct3, Ext);
assign illegal_op =  (instrn_id[31:0] == 32'h0);
assign i_ext = (EncType[4:3] == 2'b00) & ~matrix_ext;			// 00000 - 00111
assign m_ext = (EncType[4:0] == `BRISCV_INSTR_TYPE_RM);	// 01000
assign a_ext = (EncType[4:0] == `BRISCV_INSTR_TYPE_A);	// 01001
assign b_ext = (EncType[4:1] == 4'b0101);		// 01010 - 01011
assign f_ext = (EncType[4:2] == 3'b011);		// 01100 - 01111

assign matrix_ext = (opcode == `MATRIX_OPC);
assign matrix_mrf_wr_c = matrix_ext & (funct3 == `MATRIX_FUNC_MV_M_V);
assign matrix_mrf_rd_c = matrix_ext & (funct3 == `MATRIX_FUNC_MV_V_M);
assign matrix_opacc = matrix_ext & (funct3 == `MATRIX_FUNC_OPACC);
assign matrix_vrf_rd = matrix_mrf_wr_c | matrix_opacc;


assign v_ext = (&(EncType[4] == 1'b1) & ~(&EncType[3:0])) // 10000 - 11110
               & ~matrix_ext; 
// Int
autogen_EncType autogen_EncType ( opcode, funct3, funct7, mop, EncType[4:0]);
autogen_SrcA autogen_SrcA ( opcode, funct3, funct7, SrcA);
assign rs1_used = (SrcA == 2'b01);
autogen_SrcB autogen_SrcB ( opcode, funct7, funct3, mop, vm, SrcB);
assign rs2_used = (SrcB == 2'b01);
autogen_SrcC autogen_SrcC ( opcode, funct3, SrcC);
assign rs3_used = (SrcC == 2'b01);
autogen_Dest autogen_Dest ( opcode, funct3, funct7, Dest);
assign ex_autogen[`EX_AUTOGEN_FUNCT3] = funct3[2:0];
// assign ex_autogen[`EX_AUTOGEN_WIDENING] = i_ext & widening;
// Zb
autogen_Zb_instr autogen_Zb_instr ( opcode, funct7, funct3, addrp1, o_id_ex_Zb_instr);
// Fp Decode
if (INCL_FP) begin
autogen_fp_rf_rd_op_valid autogen_fp_rf_rd_op_valid ( opcode, funct7, fp_autogen[`FP_AUTOGEN_RF_RD_OP_VALID]);
autogen_fp_mad_type_inst autogen_fp_mad_type_inst ( opcode, funct7, fp_autogen[`FP_AUTOGEN_MAD_TYPE_INST]);
autogen_fp_rf_store_rd_en autogen_fp_rf_store_rd_en ( opcode, funct3, fp_autogen[`FP_AUTOGEN_RF_STORE_RD_EN]);
autogen_fp_op_int_rs1 autogen_fp_op_int_rs1 ( opcode, funct7, fp_autogen[`FP_AUTOGEN_OP_INT_RS1]);
autogen_fp_rf_rd_p2_is_rs2 autogen_fp_rf_rd_p2_is_rs2 ( opcode, funct7, fp_autogen[`FP_AUTOGEN_RF_RD_P2_IS_RS2]);
assign fp_autogen[`FP_AUTOGEN_RF_RD_HAS_RS3] = (SrcC == 2'b10);
assign fp_autogen[`FP_AUTOGEN_RF_RD_EN_P0] = (SrcA == 2'b10);
assign fp_autogen[`FP_AUTOGEN_RF_RD_EN_P1] = fp_autogen[`FP_AUTOGEN_RF_RD_P2_IS_RS2] ? 1'b0 : (SrcB == 2'b10);
assign fp_autogen[`FP_AUTOGEN_RF_RD_EN_P2] = fp_autogen[`FP_AUTOGEN_RF_RD_P2_IS_RS2] ? 1'b1 : (SrcC == 2'b10);
autogen_fp_rd_zero_p2 autogen_fp_rd_zero_p2 ( opcode, funct7, fp_autogen[`FP_AUTOGEN_RD_ZERO_P2]);
autogen_fp_rd_one_p1 autogen_fp_rd_one_p1 ( opcode, funct7, fp_autogen[`FP_AUTOGEN_RD_ONE_P1]);
autogen_fp_rd_neg_p1 autogen_fp_rd_neg_p1 ( opcode, funct7, funct3, fp_autogen[`FP_AUTOGEN_RD_NEG_P1]);
autogen_fp_rd_neg_p2 autogen_fp_rd_neg_p2 ( opcode, funct7, fp_autogen[`FP_AUTOGEN_RD_NEG_P2]);
autogen_fp_is_f16 autogen_fp_is_f16 ( opcode, funct7, funct3, addrp1, fp_autogen[`FP_AUTOGEN_IS_F16]);
autogen_fp_take_f16_src autogen_fp_take_f16_src ( opcode, funct7, funct3, addrp1, fp_autogen[`FP_AUTOGEN_TAKE_F16_SRC]);

autogen_fp_load_vld autogen_fp_load_vld ( opcode, funct3, fp_autogen[`FP_AUTOGEN_FP_LOAD_VLD]  );
autogen_int_to_fp_mov autogen_int_to_fp_mov ( opcode,funct7, fp_autogen[`FP_AUTOGEN_INT_TO_FP_MOV]  );
autogen_fp_to_int_mov autogen_fp_to_int_mov ( opcode,funct3, fp_autogen[`FP_AUTOGEN_FP_TO_INT_MOV]  );
autogen_int_to_fp_cvt autogen_int_to_fp_cvt ( opcode,funct7, fp_autogen[`FP_AUTOGEN_INT_TO_FP_CVT]  );

// assign fp_autogen[`FP_AUTOGEN_WIDENING] = f_ext & widening;
assign fp_autogen[`FP_AUTOGEN_FUNCT3] = funct3[2:0];
assign fp_autogen[`FP_AUTOGEN_FUNCT7] = funct7[6:0];
assign fp_autogen[`FP_AUTOGEN_DEST_RF] = o_fp_rf_wr_flag;
assign fp_autogen[`FP_AUTOGEN_DEST_WR_VALID] = 1'b0; // Not used
end else begin // NO FP Decode
assign fp_autogen = '0;
end // END FP Decode

// Vec
assign v_vsew = o_csr.v_vsew; 
assign v_lmul = o_csr.v_lmul; 
assign o_v_vm = instrn_id[25];
wire v_fp_rf_rd_op_valid_no_hazard_check;
wire v_fp_mad_type_inst_no_hazard_check;
if (INCL_VEC == 1) begin
wire [4:0] lumop = instrn_id[24:20];
wire [4:0] sumop = instrn_id[24:20];
assign vec_autogen.funct7 = funct7;
autogen_vfp_rf_rd_op_valid autogen_vfp_rf_rd_op_valid(opcode, funct7, funct3, addrp0, addrp1, v_fp_rf_rd_op_valid_no_hazard_check);
autogen_vfp_mad_type_inst autogen_vfp_mad_type_inst(opcode, funct7, funct3, v_fp_mad_type_inst_no_hazard_check);
assign vec_autogen.vfp_rf_rd_op_valid = v_fp_rf_rd_op_valid_no_hazard_check & o_id_vex_rts;
assign vec_autogen.vfp_mad_type_inst = v_fp_mad_type_inst_no_hazard_check   & o_id_vex_rts; 
assign vec_autogen.ldqid = id_mem_lqalloc_raw ? i_mem_id_lqnxtid : id_lqid;
autogen_v_rd_onep1 autogen_v_rd_onep1(opcode, funct7, funct3, vec_autogen.rd_onep1);
autogen_v_rd_onep1 autogen_v_onep1(opcode, funct7, funct3, vec_autogen.onep1); //IMPROVE: Duplicate signal
autogen_v_rd_zerop2 autogen_v_rd_zerop2(opcode, funct7, funct3, vec_autogen.zerop2);
autogen_v_rf_rdneg0 autogen_v_rf_rdneg0(opcode, funct7, funct3, vec_autogen.rd_neg0);

autogen_out_from_vec_int autogen_out_from_vec_int(opcode, funct7, funct3,  vec_autogen.out_from_vec_int);
autogen_v_wdeop autogen_v_wdeop(opcode, funct7, funct3, addrp0, vec_autogen.wdeop);
autogen_v_nrwop autogen_v_nrwop(opcode, funct7, funct3, addrp0, vec_autogen.nrwop);
autogen_v_src1hw autogen_v_src1hw(opcode, funct7, funct3,         vec_autogen.src1hw);
assign vec_autogen.rf_rden0 = (SrcA == 2'b11) | matrix_opacc | matrix_mrf_wr_c;
assign vec_autogen.rf_rden1 = (SrcB == 2'b11) | matrix_opacc;
assign vec_autogen.rf_rden2 = (SrcC == 2'b00) ? (Dest == 2'b11) : (SrcC == 2'b11);

autogen_v_fp_sel_scalar autogen_v_fp_sel_scalar(opcode, funct7, funct3, vec_autogen.fp_sel_scalar);
autogen_v_sat_instrn autogen_v_sat_instrn(opcode, funct7, funct3, vec_autogen.sat_instrn);   
assign vec_autogen.rf_wren = (Dest[1:0] == 2'b11) | matrix_mrf_rd_c;
assign vec_autogen.mrf_wr_c = matrix_mrf_wr_c;
assign vec_autogen.mrf_opacc = matrix_opacc;
assign vec_autogen.rf_addrp0 = addrp0;
assign vec_autogen.rf_addrp1 = addrp1;
assign vec_autogen.rf_addrp2 = addrp2;
//assign vec_autogen.wraddr    = addrp2;
autogen_v_imulop autogen_v_imulop(opcode, funct7, funct3,         vec_autogen.imulop);
autogen_v_cmpmul autogen_v_cmpmul(opcode, funct7, funct3,         vec_autogen.cmpmul);
autogen_v_rnden autogen_v_rnden(opcode, funct7, funct3,           vec_autogen.rnden);
autogen_v_saturate autogen_v_saturate(opcode, funct7, funct3,     vec_autogen.saturate);
autogen_v_mulh autogen_v_mulh(opcode, funct7, funct3,             vec_autogen.mulh);
autogen_v_acc_val autogen_v_acc_val(opcode, funct7, funct3,       vec_autogen.acc_val);
autogen_v_iaddop autogen_v_iaddop(opcode, funct7, funct3,         vec_autogen.iaddop);
autogen_v_avg autogen_v_avg(opcode, funct7, funct3,         vec_autogen.avg);   
autogen_v_usemask autogen_v_usemask(opcode, funct7, funct3, vm,   vec_autogen.usemask);
autogen_v_wrmask autogen_v_wrmask(opcode, funct7, funct3,         vec_autogen.wrmask);
autogen_v_addorsub autogen_v_addorsub(opcode, funct7, funct3,     vec_autogen.addorsub);
autogen_v_cryorbrw autogen_v_cryorbrw(opcode, funct7, funct3,     vec_autogen.cryorbrw);
autogen_v_inversesub autogen_v_inversesub(opcode, funct7, funct3,     vec_autogen.inversesub);   
//autogen_v_rd_neg1 autogen_v_rd_neg1(opcode, funct7, funct3,       vec_autogen.rd_neg1);
assign vec_autogen.rd_neg1 = 0 ; //This is not currently used to hadcoding it zero. We can connect it back
                                 //is some instruction needs it
autogen_v_rd_neg2 autogen_v_rd_neg2(opcode, funct7, funct3,       vec_autogen.rd_neg2);
autogen_v_issgn_src1 autogen_v_issgn_src1(opcode, funct7, funct3, vec_autogen.issgn_src1);
autogen_v_issgn_src2 autogen_v_issgn_src2(opcode, funct7, funct3, vec_autogen.issgn_src2);
autogen_v_permuteop  autogen_v_permuteop(opcode, funct7, funct3,  vec_autogen.permuteop);  
autogen_v_sel_scalar autogen_v_sel_scalar(opcode, funct7, funct3, vm, vec_autogen.sel_scalar); 
autogen_v_sel_imm    autogen_v_sel_imm(opcode, funct7, funct3,    vec_autogen.sel_imm);     
autogen_v_shftop     autogen_v_shftop (opcode, funct7, funct3,    vec_autogen.shftop);
autogen_v_bitwiseop  autogen_v_bitwiseop (opcode, funct7, funct3, vm, addrp1,  vec_autogen.bitwise);
autogen_v_scalar_dest  autogen_v_scalar_dest (opcode, funct7, funct3, vec_autogen.scalar_dest);   
autogen_v_rf_rd_p2_is_rs2 autogen_v_rf_rd_p2_is_rs2(opcode, funct7, funct3,    vec_autogen.rf_rd_p2_is_rs2);
autogen_v_vmvgrp      autogen_v_vmvgrp(opcode, funct7, funct3,  addrp0, vec_autogen.vmvgrp); 
autogen_v_reductop    autogen_v_reductop(opcode, funct7, funct3,  addrp0, vec_autogen.reductop);   
autogen_v_iterate     autogen_v_iterate (opcode, funct7, funct3,  addrp0, vec_autogen.iterate);
autogen_v_onecycle_iterate     autogen_v_onecycle_iterate (opcode, funct7, funct3,  addrp0, vec_autogen.onecycle_iterate);   
autogen_v_mask_only   autogen_v_mask_only(opcode, funct7, funct3, addrp0, vec_autogen.mask_only);
autogen_v_rf_store_rd_en autogen_v_rf_store_rd_en (opcode, funct3,    vec_autogen.rf_store_rd_en);  
autogen_v_ldst_iterations autogen_v_ldst_iterations (opcode, funct3, v_vsew, v_lmul, mop, nf, addrp1, vecldst_autogen.ldst_iterations);
autogen_v_ldst_dest_incr autogen_v_ldst_dest_incr (opcode, funct3, v_vsew, v_lmul, mop, nf, addrp1, vec_autogen.addrp2_incr);
assign vecldst_autogen.ldst_dest_incr = vec_autogen.addrp2_incr;
assign vec_autogen.addrp0_incr = '0;
autogen_v_ldst_index_incr autogen_v_ldst_index_incr (opcode, funct3, v_vsew, v_lmul, mop, nf, vec_autogen.addrp1_incr);
assign vecldst_autogen.ldst_index_incr = vec_autogen.addrp1_incr;
autogen_v_ldst_index_reset autogen_v_ldst_index_reset (opcode, funct3, v_vsew, v_lmul, mop, nf, vec_autogen.addrp1_reset);
assign vec_autogen.replay_cnt = '0;
assign vec_autogen.onecycle_nrwop = !lmul_gteq1 && vec_autogen.nrwop;  // Narrowing op with fractional lmul

assign vecldst_autogen.load  = (EncType == `BRISCV_INSTR_TYPE_L1) | (EncType == `BRISCV_INSTR_TYPE_L2) | (EncType == `BRISCV_INSTR_TYPE_L3);
assign vecldst_autogen.store = (EncType == `BRISCV_INSTR_TYPE_S1) | (EncType == `BRISCV_INSTR_TYPE_S2) | (EncType == `BRISCV_INSTR_TYPE_S3);
assign vecldst_autogen.dest_reg = addrp2;
assign vecldst_autogen.ldst_ustride = (EncType == `BRISCV_INSTR_TYPE_L1) | (EncType == `BRISCV_INSTR_TYPE_S1);
assign vecldst_autogen.ldst_strided = (EncType == `BRISCV_INSTR_TYPE_L2) | (EncType == `BRISCV_INSTR_TYPE_S2);
assign vecldst_autogen.ldst_index   = (EncType == `BRISCV_INSTR_TYPE_L3) | (EncType == `BRISCV_INSTR_TYPE_S3);
assign vecldst_autogen.ldst_mask    = (vecldst_autogen.load | vecldst_autogen.store) & vecldst_autogen.ldst_ustride & (addrp1 == `BRISCV_VECLDST_MASK);
assign vecldst_autogen.ldst_whole_register = (vecldst_autogen.load | vecldst_autogen.store) & vecldst_autogen.ldst_ustride & (addrp1 == `BRISCV_VECLDST_WHOLE_REGISTER);
assign vecldst_autogen.ldst_iter_cnt = '0;  // always start with 0
end
else begin
   assign vec_autogen = '0;
   assign vecldst_autogen = '0;
end
//spyglass enable_block NamedAssoc

assign o_fp_rf_wr_flag = (Dest[1:0] == 2'b10);
assign o_fp_rf_wraddr  = o_rf_wraddr;
   
assign o_rf_wr_flag = (Dest[1:0] == 2'b01);
assign o_id_type = EncType[4:0];

//////
// Decode operand encoding type
always @* begin
   o_id_type_r  = (EncType[4:0] == `BRISCV_INSTR_TYPE_R) | (EncType[4:0] == `BRISCV_INSTR_TYPE_RB)| (EncType[4:0] == `BRISCV_INSTR_TYPE_RM); // register to register alu op
   o_id_type_i  = (EncType[4:0] == `BRISCV_INSTR_TYPE_I) | (EncType[4:0] == `BRISCV_INSTR_TYPE_IB) | (EncType[4:0] == `BRISCV_INSTR_TYPE_IF); // loads, JALR and alu ops with immediate operands
   o_id_type_s  = (EncType[4:0] == `BRISCV_INSTR_TYPE_S) | (EncType[4:0] == `BRISCV_INSTR_TYPE_SF); // store.
   o_id_type_sb = (EncType[4:0] == `BRISCV_INSTR_TYPE_B); // branch
   o_id_type_u  = (EncType[4:0] == `BRISCV_INSTR_TYPE_U); // lui | auipc
   o_id_type_uj = (EncType[4:0] == `BRISCV_INSTR_TYPE_J); //
   o_id_type_e  = (EncType[4:0] == `BRISCV_INSTR_TYPE_E) & ~matrix_ext; // system
   o_id_type_f  = (EncType[4:0] == `BRISCV_INSTR_TYPE_F); // system
end

// Sync logic
always @(posedge i_clk) begin
  if(~i_reset_n) begin
    sync_stall <= 1'b0;
    sync_stall_op <= 'd0;
  end
  else begin
    if (sync_stall && i_mem_fe_lqempty)
        sync_stall <= 1'b0;
    else
    if (o_id_instrn_rtr & id_rts & (((o_id_type_e | o_id_type_f | vsetOp_to_ex) ))) begin     // FENCE or CSRWrite
        sync_stall_op <= opcode[6:0];
        sync_stall <= 1'b1;
    end
  end
end

`ifdef SIM

// Keeping OLD implementation (pre-Autogen) under SIM to assert against
// This was hand written decode logic - now replaced with autogen
reg OLD_it_branch;
reg OLD_it_jal;
reg OLD_it_jalr;
reg OLD_it_load;
reg OLD_it_store;
reg OLD_it_alu_immed;
reg OLD_it_alu_reg;
reg OLD_it_alu_lui;
reg OLD_it_alu_auipc;
//reg OLD_it_alu; // alu ops, including LUI and AUIPC
always @* begin
    OLD_it_jal       = (opcode[6:0] == 7'b1101111);
    OLD_it_jalr      = (opcode[6:0] == 7'b1100111);
    OLD_it_branch    = (opcode[6:0] == 7'b1100011);
    OLD_it_load      = (opcode[6:0] == 7'b0000011);
    OLD_it_store     = (opcode[6:0] == 7'b0100011);
    OLD_it_alu_immed = (opcode[6:0] == 7'b0010011);
    OLD_it_alu_reg   = (opcode[6:0] == 7'b0110011);
    OLD_it_alu_lui   = (opcode[6:0] == 7'b0110111);
    OLD_it_alu_auipc = (opcode[6:0] == 7'b0010111);
//    OLD_it_alu       = OLD_it_alu_immed | OLD_it_alu_reg | OLD_it_alu_lui | OLD_it_alu_auipc;
end

// Branch count metrics for all branches
//   7: BEQ //   6: BGE //   5: BGEU //   4: BLT
//   3: BLTU //   2: BNE //   1: JAL //   0: JALR
reg [31:0] branch_count_beq;
reg [31:0] branch_count_bge;
reg [31:0] branch_count_bgeu;
reg [31:0] branch_count_blt;
reg [31:0] branch_count_bltu;
reg [31:0] branch_count_bne;
reg [31:0] branch_count_jal;
reg [31:0] branch_count_jalr;
reg [31:0] branch_count_uncond;
reg [31:0] branch_count_call;
reg [31:0] branch_count_ret;
wire jalRdIsLink, jalRs1IsLink, jalRdEqRs1;

assign jalRdIsLink = o_rf_wr_flag & ((o_rf_wraddr == 5'h1) | (o_rf_wraddr == 5'h5));
assign jalRs1IsLink = o_rf_p0_rden & ((o_rf_p0_rdaddr == 5'h1) | (o_rf_p0_rdaddr == 5'h5));
assign jalRdEqRs1 = o_rf_wr_flag & o_rf_p0_rden & (o_rf_wraddr == o_rf_p0_rdaddr);

always @(posedge i_clk) begin
  if(~i_reset_n) begin
    branch_count_beq <= 'd0;
    branch_count_bge <= 'd0;
    branch_count_bgeu <= 'd0;
    branch_count_blt <= 'd0;
    branch_count_bltu <= 'd0;
    branch_count_bne <= 'd0;
    branch_count_jal <= 'd0;
    branch_count_jalr <= 'd0;
    branch_count_uncond <= 'd0;
    branch_count_call <= 'd0;
    branch_count_ret <= 'd0;
  end
  else begin
    // Collect Branch metrics for trace by branch type
    if(o_id_ex_rts & i_ex_rtr & (~raw_hazard_stall) ) begin
       if (OLD_it_branch & (instrn_id[14:12] == 3'b000)) branch_count_beq <= branch_count_beq + 1'b1; // BEQ
       if (OLD_it_branch & (instrn_id[14:12] == 3'b101)) branch_count_bge <= branch_count_bge + 1'b1; // BGE
       if (OLD_it_branch & (instrn_id[14:12] == 3'b111)) branch_count_bgeu <= branch_count_bgeu + 1'b1; // BGEU
       if (OLD_it_branch & (instrn_id[14:12] == 3'b100)) branch_count_blt <= branch_count_blt + 1'b1; // BLT
       if (OLD_it_branch & (instrn_id[14:12] == 3'b110)) branch_count_bltu <= branch_count_bltu + 1'b1; // BLTU
       if (OLD_it_branch & (instrn_id[14:12] == 3'b001)) branch_count_bne <= branch_count_bne + 1'b1; // BNE
       if (OLD_it_jal) branch_count_jal <= branch_count_jal + 1'b1; // JAL
       if (OLD_it_jalr) branch_count_jalr <= branch_count_jalr + 1'b1; // JALR
       if ((OLD_it_jal | OLD_it_jalr) & (~jalRdIsLink & ~jalRs1IsLink)) branch_count_uncond <= branch_count_uncond + 1'b1; // JAL is an uncond Jump
       if ((OLD_it_jal | OLD_it_jalr) & (~jalRdIsLink & jalRs1IsLink) | (jalRdIsLink & jalRs1IsLink & ~jalRdEqRs1)) branch_count_ret <= branch_count_ret + 1'b1; // JAL is a return
       if ((OLD_it_jal | OLD_it_jalr) & jalRdIsLink) branch_count_call <= branch_count_call + 1'b1; // JAL is a call
    end
  end
end

reg  OLD_id_type_r;
reg  OLD_id_type_i;
reg  OLD_id_type_s;
reg  OLD_id_type_sb;
reg  OLD_id_type_u;
reg  OLD_id_type_uj;
//////
// Decode operand encoding type
always @* begin
   OLD_id_type_r  = OLD_it_alu_reg                      ; // register to register alu op
   OLD_id_type_i  = OLD_it_load | OLD_it_jalr | OLD_it_alu_immed; // loads, JALR and alu ops with immediate operands
   OLD_id_type_s  = OLD_it_store                        ;
   OLD_id_type_sb = OLD_it_branch                       ;
   OLD_id_type_u  = OLD_it_alu_lui | OLD_it_alu_auipc       ;
   OLD_id_type_uj = OLD_it_jal                          ;
end

reg OLD_rs1_used;
reg OLD_rs2_used;
reg OLD_rf_wr_flag;

always @* begin
   OLD_rs1_used = (o_id_type_r | o_id_type_i | o_id_type_s | o_id_type_sb) | (i_vex_id_rtr & o_id_vex_rts & (INCL_VEC == 1));
   OLD_rs2_used = (o_id_type_r | o_id_type_s | o_id_type_sb);
   OLD_rf_wr_flag   = o_id_type_r | o_id_type_i | o_id_type_u | o_id_type_uj;
end

   // ABV 1 - Not tried
   // assert_old_vs_new_EncType: assert #0 ( |-> EncType[4:0] == b); 
   // ABV 2 - Implication. Errors out
   // property old_vs_new_EncType_R;
   //      @(posedge i_clk) disable iff(~i_reset_n) (OLD_id_type_r) |-> (EncType[4:0] == 3'b0);
   // endproperty

// Keeping Old vs New check as conditional asserts
always @(posedge i_clk) begin
  if (~i_reset_n) begin
  // `ASSERT_NORST(1'b0,"ERROR: Basic Assert check to see if it works");
  // `ASSERT_COND_NORST(OLD_id_ascii_instrn,id_ascii_instrn,"ERROR: Autogen Ascii Instruction mismatches, old = %s, new = %s", OLD_id_ascii_instrn, id_ascii_instrn);
  // `ASSERT_COND_NORST(OLD_id_type_r,o_id_type_r,"ERROR: Autogen EncType mismatches with Old EncType, old = %b, new = %b", OLD_id_type_r, o_id_type_r);
  // `ASSERT_COND_NORST(OLD_id_type_i,o_id_type_i,"ERROR: Autogen EncType mismatches with Old EncType, old = %b, new = %b", OLD_id_type_i, o_id_type_i);
  // `ASSERT_COND_NORST(OLD_id_type_s,o_id_type_s,"ERROR: Autogen EncType mismatches with Old EncType, old = %b, new = %b", OLD_id_type_s, o_id_type_s);
  // `ASSERT_COND_NORST(OLD_id_type_sb,o_id_type_sb,"ERROR: Autogen EncType mismatches with Old EncType, old = %b, new = %b", OLD_id_type_sb, o_id_type_sb);
  // `ASSERT_COND_NORST(OLD_id_type_u,o_id_type_u,"ERROR: Autogen EncType mismatches with Old EncType, old = %b, new = %b", OLD_id_type_u, o_id_type_u);
  // `ASSERT_COND_NORST(OLD_id_type_uj,o_id_type_uj,"ERROR: Autogen EncType mismatches with Old EncType, old = %b, new = %b", OLD_id_type_uj, o_id_type_uj);

  `ASSERT_COND_NORST(OLD_rs1_used,rs1_used,"ERROR: Autogen rs1_used mismatches with Old rs1_used, old = %b, new = %b", OLD_rs1_used, rs1_used);
  `ASSERT_COND_NORST(OLD_rs2_used,rs2_used,"ERROR: Autogen rs2_used mismatches with Old rs2_used, old = %b, new = %b", OLD_rs2_used, rs2_used);

  `ASSERT_COND_NORST(OLD_rf_wr_flag,o_rf_wr_flag,"ERROR: Autogen rf_wr_flag mismatches with Old rf_wr_flag, old = %b, new = %b", OLD_rf_wr_flag, o_rf_wr_flag);
  end
end

`endif

//////
// Register file read/write enables and addresses

always @* begin
   o_rf_wraddr    = addrp2[4: 0];
end

//////
// Immediate operand enable and assembly
wire [5:0] immed_type   = {o_id_type_i, o_id_type_s , o_id_type_sb , o_id_type_u , o_id_type_uj, o_id_type_e};
/* verilator lint_off UNUSED */
reg        immed_op_vld   ;
/* verilator lint_on UNUSED */
reg [31:0] immed_op       ;

always @* begin
   immed_op_vld    = o_id_type_i | o_id_type_s | o_id_type_sb | o_id_type_u | o_id_type_uj | o_id_type_e;
   /* verilator lint_off CASEINCOMPLETE */
   /* verilator lint_off CASEOVERLAP */
   casez(immed_type) // synopsys full_case parallel_case
    6'b1?????: immed_op = {{20{instrn_id[31]}}, instrn_id[31:20]};
    6'b?1????: immed_op = {{20{instrn_id[31]}}, instrn_id[31:25], instrn_id[11:7]};
    6'b??1???: immed_op = {{19{instrn_id[31]}}, instrn_id[31]   , instrn_id[7], instrn_id[30:25], instrn_id[11:8],1'b0};
    6'b???1??: immed_op = {instrn_id[31:12], {12{         1'b0}}};
    6'b????1?: immed_op = {{12{instrn_id[31]}}, instrn_id[19:12], instrn_id[20], instrn_id[30:21], 1'b0};
    6'b?????1: immed_op = {{27{ 1'b0}}, instrn_id[19:15]};
    default: immed_op = 'bx;
    /* verilator lint_on CASEOVERLAP */
    /* verilator lint_on CASEINCOMPLETE */
   endcase
end
assign o_id_immed_op = immed_op;

//////
// RAW hazard handling
//////
logic [LQ_DEPTH-1:0]    lq_hit_entry_matrix, lq_hit_entry_vex_p0, lq_hit_entry_vex_p1, lq_hit_entry_vex_p2, lq_hit_entry_vex_mask, lq_hit_entry_vex_r0, lq_hit_entry_vex_f0;
logic [LQ_DEPTH_LOG2:0] lq_hit_cnt_matrix, lq_hit_cnt_vex_p0, lq_hit_cnt_vex_p1, lq_hit_cnt_vex_p2, lq_hit_cnt_vex_mask, lq_hit_cnt_vex_r0, lq_hit_cnt_vex_f0;
logic vec_vs3_hazard_stall, vec_vs2_hazard_stall, vec_vs1_hazard_stall,vec_mask_hazard_stall;
logic vec_rs1_hazard_stall;   
logic vec_fs1_hazard_stall;   

always_comb begin 
   for(int x=0;x<LQ_DEPTH;++x) begin
      lq_hit_entry_matrix[x]   = (INCL_VEC == 1) & i_lq_broadside_valid[x] &  i_lq_broadside_info[x].mrf_wr_c & o_vec_autogen.mrf_opacc; 

      lq_hit_entry_vex_p0[x]   = (INCL_VEC == 1) & ((mask_rf_addrp0 & o_vec_autogen.rf_addrp0) == (i_lq_broadside_info[x].rf_wraddr & mask_rf_addrp0)) & i_lq_broadside_valid[x] &  i_lq_broadside_info[x].vrf_wr_flag   & o_vec_autogen.rf_rden0 & !(o_vec_autogen.sel_scalar || o_vec_autogen.fp_sel_scalar); 
      lq_hit_entry_vex_p1[x]   = (INCL_VEC == 1) & ((mask_rf_addrp1 & o_vec_autogen.rf_addrp1) == (i_lq_broadside_info[x].rf_wraddr & mask_rf_addrp1)) & i_lq_broadside_valid[x] &  i_lq_broadside_info[x].vrf_wr_flag   & o_vec_autogen.rf_rden1;
      lq_hit_entry_vex_p2[x]   = (INCL_VEC == 1) & ((mask_rf_addrp2 & o_vec_autogen.rf_addrp2) == (i_lq_broadside_info[x].rf_wraddr & mask_rf_addrp2)) & i_lq_broadside_valid[x] &  i_lq_broadside_info[x].vrf_wr_flag   & o_vec_autogen.rf_rden2;
      lq_hit_entry_vex_r0[x]   = (INCL_VEC == 1) & (o_vec_autogen.rf_addrp0 == i_lq_broadside_info[x].rf_wraddr) & i_lq_broadside_valid[x] & i_lq_broadside_info[x].rf_wr_flag    & o_vec_autogen.sel_scalar;
      lq_hit_entry_vex_f0[x]   = (INCL_VEC == 1) & (o_vec_autogen.rf_addrp0 == i_lq_broadside_info[x].rf_wraddr) & i_lq_broadside_valid[x] & i_lq_broadside_info[x].fp_rf_wr_flag & o_vec_autogen.fp_sel_scalar;
 
      lq_hit_entry_vex_mask[x] = (INCL_VEC == 1) & (5'd0                    == i_lq_broadside_info[x].rf_wraddr) & i_lq_broadside_valid[x] & i_lq_broadside_info[x].vrf_wr_flag & ~instrn_id[25]; //for vec instruction, check for mask hazard if vm=0
  end
end
   
//IMPROVE_msalvi: for now not supporting forwarding from mem to vrf. For now just checking for raw_hazard.   
tt_popcnt #(.WIDTH(LQ_DEPTH)) cnt_mrf   (.req_in(lq_hit_entry_matrix[LQ_DEPTH-1:0]),  .req_sum(lq_hit_cnt_matrix[LQ_DEPTH_LOG2:0])); 
tt_popcnt #(.WIDTH(LQ_DEPTH)) cnt_vrf_p0   (.req_in(lq_hit_entry_vex_p0[LQ_DEPTH-1:0]),  .req_sum(lq_hit_cnt_vex_p0[LQ_DEPTH_LOG2:0])); 
tt_popcnt #(.WIDTH(LQ_DEPTH)) cnt_vrf_p1   (.req_in(lq_hit_entry_vex_p1[LQ_DEPTH-1:0]),  .req_sum(lq_hit_cnt_vex_p1[LQ_DEPTH_LOG2:0]));
tt_popcnt #(.WIDTH(LQ_DEPTH)) cnt_vrf_p2   (.req_in(lq_hit_entry_vex_p2[LQ_DEPTH-1:0]),  .req_sum(lq_hit_cnt_vex_p2[LQ_DEPTH_LOG2:0]));
tt_popcnt #(.WIDTH(LQ_DEPTH)) cnt_vrf_mask (.req_in(lq_hit_entry_vex_mask[LQ_DEPTH-1:0]),.req_sum(lq_hit_cnt_vex_mask[LQ_DEPTH_LOG2:0]));
tt_popcnt #(.WIDTH(LQ_DEPTH)) cnt_vrf_r0   (.req_in(lq_hit_entry_vex_r0[LQ_DEPTH-1:0]),  .req_sum(lq_hit_cnt_vex_r0[LQ_DEPTH_LOG2:0]));   
tt_popcnt #(.WIDTH(LQ_DEPTH)) cnt_vrf_f0   (.req_in(lq_hit_entry_vex_f0[LQ_DEPTH-1:0]),  .req_sum(lq_hit_cnt_vex_f0[LQ_DEPTH_LOG2:0]));   
      
//IMPROVE_msalvi: No forwarding from  vex output to input(either vex or int or fp), to avoid timing issue. 
// If timing looks good we may later disable this check at the expense of a bypass from vex output..
//IMPROVE: msalvi what about v_lmul[2]? How does that affect mask, right now this check is pessimistic
wire   vrgatherei16op      = (opcode[6:0] == 'h57) & (funct7[6:1] == 6'b00_1110) & (funct3[2:0] == `OPIVV);		
//assign mask_rf_addrp0[4:0] = 5'b1_1111 << (v_lmul[1:0] + vec_autogen.nrwop + ((vrgatherei16op & (v_vsew[1:0]==2'b00)) << v_lmul[1:0])); //Note: for vsew=32, gatherei16 check is pesimisstic, as emul doesn't change as fast as lmul
assign mask_rf_addrp0[4:0] = 5'b1_1111 << ({1'b0,v_lmul[1:0]} + vec_autogen.nrwop + (vrgatherei16op & (v_vsew[1:0]==2'b00)));
assign mask_rf_addrp1[4:0] = 5'b1_1111 << ({1'b0,v_lmul[1:0] & ~{2{vec_autogen.vmvgrp | vec_ldst_idx_vld}}} + {2'd0,vec_autogen.nrwop} + {2'd0,vec_autogen.wdeop & vec_autogen.src1hw} + (instrn_id[17:15] & {3{vec_autogen.vmvgrp}}));   
assign mask_rf_addrp2[4:0] = 5'b1_1111 << ((v_lmul[1:0] + vec_autogen.wdeop) & ~{2{vec_ldst_vld}});

assign matrix_vs3_hazard_stall =  valid_opacc_instrn & (lq_hit_cnt_matrix > '0) & ~id_replay;
assign vec_vs1_hazard_stall =  (valid_vec_instrn | valid_matrix_instrn) & (lq_hit_cnt_vex_p0 > '0) & ~id_replay; 
assign vec_vs2_hazard_stall =  (valid_vec_instrn | valid_matrix_instrn) & (lq_hit_cnt_vex_p1 > '0) & ((vec_autogen_replay.addrp1_incr[0] & vec_ldst_idx_vld) | ~id_replay); 
assign vec_vs3_hazard_stall =  valid_vec_instrn & (lq_hit_cnt_vex_p2 > '0) & ((vec_autogen_replay.addrp2_incr[0] & vec_ldst_vld)     | ~id_replay); 
assign vec_rs1_hazard_stall =  valid_vec_instrn & (lq_hit_cnt_vex_r0 > '0); 
assign vec_fs1_hazard_stall =  valid_vec_instrn & (lq_hit_cnt_vex_f0 > '0); 
assign vec_mask_hazard_stall  = valid_vec_instrn & (lq_hit_cnt_vex_mask > '0) & ~id_replay;

wire rs1_hazard_stall = vec_vs1_hazard_stall  | vec_rs1_hazard_stall | vec_fs1_hazard_stall;
wire rs2_hazard_stall = vec_vs2_hazard_stall;
wire rs3_hazard_stall = vec_vs3_hazard_stall;

assign raw_hazard_stall_fwd = rs1_hazard_stall | rs2_hazard_stall | rs3_hazard_stall | vec_mask_hazard_stall;
assign raw_hazard_stall_vex = matrix_vs3_hazard_stall |
                              vec_vs1_hazard_stall  | vec_rs1_hazard_stall | vec_fs1_hazard_stall |
                              vec_vs2_hazard_stall  |
                              vec_vs3_hazard_stall  |
                              vec_mask_hazard_stall;
   
assign raw_hazard_stall     = ((valid_vec_instrn | valid_matrix_instrn) && !vec_ldst_vld && !vsetOp_to_ex)  ? raw_hazard_stall_vex 
                                                                                    : raw_hazard_stall_fwd;

// Drive out new inst dispatch -- This should also track the number of inst retired
wire id_ex_instdisp = id_rts & ~raw_hazard_stall & ~id_replay;
tt_pipe_stage #(.WIDTH(1)) I_instdisp ( i_clk, i_reset_n, 1'b1, id_ex_instdisp, o_id_ex_instdisp);

endmodule
