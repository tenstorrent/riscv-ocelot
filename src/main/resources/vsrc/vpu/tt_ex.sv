// See LICENSE.TT for license details.
`include "briscv_defines.h"
`include "tt_briscv_pkg.vh"
//`define BRISCV_LOG

module tt_ex #(parameter INCL_VEC=0, VLEN=128, ADDRWIDTH=40, ST_DATA_WIDTH_BITS=128, LQ_DEPTH_LOG2=tt_briscv_pkg::LQ_DEPTH_LOG2)
(
   input 			    i_clk ,
   input 			    i_reset_n,

   input [36:0] 		    i_if_ex_deco ,
   input 			    i_if_ex_predicted ,
   input 			    i_if_ex_nextinstr ,

   input tt_briscv_pkg::csr_to_vec i_ex_vec_csr,
   // From ID
   input [LQ_DEPTH_LOG2-1:0] 	    i_id_ex_lqid,
   input [4:0] 			    i_id_type ,
   input 			    i_id_rf_wr_flag ,
   input [ 4:0] 		    i_id_rf_wraddr ,
   input 			    i_id_fp_rf_wr_flag ,
   input [ 4:0] 		    i_id_fp_rf_wraddr ,
   input [31:0] 		    i_id_immed_op ,
   input 			    i_id_ex_rts ,
   input 			    i_id_ex_vecldst,
   input [ 4:0] 		    i_id_ex_Zb_instr ,
   input 			    i_id_ex_units_rts ,
   input [31:0] 		    i_id_ex_pc ,
   input [31:0] 		    i_id_ex_instrn ,
   input 			    tt_briscv_pkg::vecldst_autogen_s i_id_ex_vecldst_autogen, //Decode signal bundle
   input 			    i_id_ex_instdisp ,
   input          i_id_ex_last,

   // From RF
   input [63:0] 		    i_rf_p0_reg ,
   input [63:0] 		    i_rf_p1_reg ,
   input [31:0] 		    i_fp_rf_p3_reg,
 
   // From VRF
   input [VLEN-1:0] 		    i_vmask_rddata, // ID stage signals
   input [VLEN-1:0] 		    i_vs2_rddata, // ID stage signals
   input [VLEN-1:0] 		    i_vs3_rddata, // ID stage signals

   // From MEM
   //input               i_mem_ex_skidbuffull,
   input 			    i_mem_ex_rtr ,

   // To ID and IF
   output wire 			    o_ex_bp_fifo_pop ,
   output wire 			    o_ex_is_some_branch ,
   output 			    o_ex_branch_taken ,
   output 			    o_ex_id_rtr ,
   output logic     o_ex_last,

   output logic 		    o_ex_dst_vld_1c, // forwarding control to ID
   output logic [LQ_DEPTH_LOG2-1:0] o_ex_dst_lqid_1c, // forwarding control to ID
   output logic [31:0] 		    o_ex_fwd_data_1c, // forwarding control to ID

   output logic 		    o_ex_dst_vld_2c, // forwarding control to ID
   output logic [LQ_DEPTH_LOG2-1:0] o_ex_dst_lqid_2c, // forwarding control to ID
   output logic [31:0] 		    o_ex_fwd_data_2c, // forwarding control to ID

   output 			    o_ex_bp_mispredict ,
   output 			    o_ex_bp_mispredict_not_br,
   output [31:0] 		    o_ex_bp_pc ,

   // Exception flag update from FP
   input 			    tt_briscv_pkg::csr_fp_exc i_exc_fp_ex_update,

   // Exception flag update from VFP
   input                            tt_briscv_pkg::csr_fp_exc i_exc_vfp_update,

   input 			    i_sat_csr,
   output 			    tt_briscv_pkg::csr_to_id o_ex_id_csr,
   output 			    tt_briscv_pkg::csr_to_vec o_ex_vec_csr,

   // EX --> MEM LQ signals
   output logic 		    o_ex_mem_lqvld_1c,
   output logic [31:0] 		    o_ex_mem_lqdata_1c,
   output logic [LQ_DEPTH_LOG2-1:0] o_ex_mem_lqid_1c,
   
   output logic 		    o_ex_mem_lqvld_2c,
   output logic [31:0] 		    o_ex_mem_lqdata_2c,
   output logic [LQ_DEPTH_LOG2-1:0] o_ex_mem_lqid_2c,
   
   // To MEM
   output 			    tt_briscv_pkg::mem_skidbuf_s o_ex_mem_payload,
   output wire 			    o_ex_mem_vld ,

   // Unique indentifier for debug
   input [31:0] 		    i_reset_pc
);

tt_briscv_pkg::vecldst_autogen_s ex_vecldst_autogen;

tt_briscv_pkg::mem_skidbuf_s ex_mem_payload_fn;
tt_briscv_pkg::mem_skidbuf_s ex_mem_payload, ex_mem_payload_q;
   

logic 	      mul_en,mul_en_d1;
logic [31:0]  mul_res_d1;   
wire hazard_stall = mul_en & ~mul_en_d1 & 1'b0; // all stalling in ID for now, will need to stall for store value here eventually

// V-LD/ST misc signals
logic [VLEN/8-1:0] ex_vecldst_iter_mask;
logic [VLEN/8-1:0] ex_vecst_byte_mask;
logic [$clog2(VLEN/8+1)-1:0] ex_vecldst_elem_per_iter;
logic [$clog2(VLEN/8+1)-1:0] ex_vecldst_base_elem_done;  //Number of elements done till this iteration
logic [$clog2(VLEN/8+1)-1:0] ex_vecldst_data_num_elem, ex_vecldst_idx_num_elem;
logic [1:0] ex_vecldst_data_elem_top_shft, ex_vecldst_idx_elem_top_shft;
logic [$clog2(VLEN+1)-1:0] ex_vecldst_vl, ex_vecldst_vl_div8;
logic [$clog2(VLEN/8+1)-1:0] ex_vecldst_mask_num_elem;
logic [1:0] ex_vecldst_data_eew, ex_vecldst_idx_eew;
logic [2:0] ex_vecldst_data_emul, ex_vecldst_idx_emul;
logic [3:0] ex_vecldst_nf;
logic [$clog2(VLEN/8+1)-1:0] ex_vecldst_data_elem_base, ex_vecldst_idx_elem_base;
logic [4:0] id_vecldst_rf_wraddr;
logic [$clog2(VLEN/8)-1:0] ex_vecldst_elem_count;    // When count reaches top, vecldst is done
logic [63:0] ex_vecldst_idx_op2, ex_vecldst_op2;
logic [63:0] ex_vecldst_store_val;
logic [ST_DATA_WIDTH_BITS-1:0] ex_vecldst_store_val_fn;

logic [$clog2(VLEN)-1:0] ex_vs2_base, ex_vs3_base;
logic [VLEN-1:0] ex_vmask_reg, ex_vs2_reg, ex_vs3_reg;

logic [31:0] csr_pmacfg0, csr_pmacfg1;
logic [ADDRWIDTH-1:0] vec_ldst_addr, ldst_addr;
logic [31:0] alu_result;
   
// Integer Division
localparam XLEN = 32;
logic            int_div_vld;
logic            int_div_ack;
logic [XLEN-1:0] int_div_rs1;
logic [XLEN-1:0] int_div_rs2;
logic            int_div_rem;
logic            int_div_sgn;
logic            int_div_res_rts;
logic            int_div_res_rtr;
logic [XLEN-1:0] int_div_res;

logic            int_div_stall; // Stall the pipe when div is progress

// Bit Manipulation

logic                            bit_vld;
tt_briscv_pkg::bitmanip_instrn_e bit_cmd;
logic                     [31:0] bit_rs1;
logic                     [31:0] bit_rs2;
logic                     [31:0] bit_res;

  
   
////////
//// Pipeline stage and flow control
wire rtr_to_id;
wire [31:0] ex_immed_op;
wire [31:0] ex_instrn;
wire [31:0] ex_pc;
reg  [4:0]  ex_Zb_instr;
wire [7:0] if_ex_misc_sig = {i_id_ex_vecldst, i_id_type[4:0], i_id_fp_rf_wr_flag, i_id_rf_wr_flag};


wire [7:0] ex_misc_sig;
wire ex_type_vecldst  = ex_misc_sig[7];
wire ex_type_r     = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_R) | (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_RM) | (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_RB);
wire ex_type_i     = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_I) | (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_IF) | (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_I) | (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_IB);
wire ex_type_s     = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_S) | (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_SF);
wire ex_type_sb    = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_B);
wire ex_type_u     = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_U);
wire ex_type_uj    = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_J);
wire ex_type_e     = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_E);
wire ex_type_c1    = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_C1);
wire ex_type_c2    = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_C2);
wire ex_type_c3    = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_C3);
wire ex_type_amo   = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_A);
wire ex_type_div   = (ex_instrn[31:25] == 7'b0000001) &&
                     (ex_instrn[ 14]   == 1'b1      ) && 
                     (ex_instrn[6:0]   == 7'b0110011);
wire ex_type_jal   = (ex_instrn[6:0]   == 7'b1100111) | (ex_instrn[6:0]   == 7'b1101111);
wire ex_type_bit   = ex_Zb_instr != '0;
wire ex_fp_rf_wr_flag = ex_misc_sig[1];
wire ex_rf_wr_flag = ex_misc_sig[0];
wire [4:0] ex_rf_wraddr;
logic ex_vld, ex_vld_q;
logic ex_type_vecldst_q;
logic ex_vecldst_elem_mask, ex_vecldst_elem_mask_q;
logic ex_vecldst_start, ex_vecldst_start_q;
logic ex_vecldst_done,  ex_vecldst_done_q;
logic ex_vecldst_iter_done, ex_vecldst_iter_done_q;
logic [LQ_DEPTH_LOG2-1:0] ex_lqid, ex_lqid_d1;

wire mem_pipe_rtr_raw = (i_mem_ex_rtr & ~int_div_stall);
wire mem_pipe_rtr     = (i_mem_ex_rtr & ~int_div_stall & (~(ex_vld & ex_type_vecldst) | ex_vecldst_iter_done_q));
reg ex_disp_vld;
logic ex_last;

assign o_ex_id_rtr = mem_pipe_rtr;
assign o_ex_last = ex_last;

// Create a single cycle EX branch vld based on new instr popped out of the instrn_fifo
always_ff @(posedge i_clk) begin
   ex_disp_vld <= i_if_ex_nextinstr;
end

// Don't assert valid for maksed vector ldst element unless it's the first or last element (which is needed since it needs to go to lq or track the last index)
assign o_ex_mem_vld = mem_pipe_rtr_raw & ~mul_en & 
                      ((ex_vld & ~ex_type_vecldst) | 
                       (ex_vld_q & ex_type_vecldst_q & (ex_vecldst_start_q | ex_vecldst_done_q | ex_mem_payload_q.vecldst_128 | ~ex_vecldst_elem_mask_q)));
   
// Allocate on last index for vector stores otherwise vector store will commit before dispatch of all indexes
// Allocate on first index for vector loads since vector load coalescing happens in load queue and index might return before last index is dispatched  
assign o_ex_mem_lqvld_1c = ex_vld & ~mul_en & mem_pipe_rtr_raw & (~ex_type_vecldst | (o_ex_mem_payload.mem_store ? ex_vecldst_done_q : ex_vecldst_start_q));
assign o_ex_mem_lqid_1c  = ex_lqid;
assign o_ex_mem_lqdata_1c = o_ex_mem_payload.mem_alu_result;
   
assign o_ex_mem_lqvld_2c = mul_en_d1;
assign o_ex_mem_lqid_2c  = ex_lqid_d1;
assign o_ex_mem_lqdata_2c = {32{mul_en_d1}} & mul_res_d1[31:0];  //add other ooo data here... example div
   
tt_pipe_stage #(.WIDTH(1)) ex_vld_flop   
(
   .i_clk     (i_clk         ),
   .i_reset_n (i_reset_n     ),
   .i_en      (mem_pipe_rtr  ),
   .i_d       (i_id_ex_rts   ),
   .o_q       (ex_vld        )
);

logic units_vld; //MM Nov 5 2021: Remove implicit wire. 
tt_pipe_stage #(.WIDTH(1)) units_vld_flop   
(
   .i_clk     (i_clk         ),
   .i_reset_n (i_reset_n     ),
   .i_en      (mem_pipe_rtr  ),
   .i_d       (i_id_ex_units_rts),
   .o_q       (units_vld        )
);

tt_pipe_stage #(.WIDTH(1)) ex_last_flop   
(
   .i_clk     (i_clk         ),
   .i_reset_n (i_reset_n     ),
   .i_en      (i_id_ex_rts & o_ex_id_rtr  ),
   .i_d       (i_id_ex_last),
   .o_q       (ex_last        )
);

tt_pipe_stage #(.WIDTH(LQ_DEPTH_LOG2)) ex_lqid_flops 
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (i_id_ex_rts & o_ex_id_rtr),
   .i_d       (i_id_ex_lqid  ),
   .o_q       (ex_lqid       )
);

tt_pipe_stage #(.WIDTH(32)) ex_instrn_flops  
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (i_id_ex_units_rts & o_ex_id_rtr),
   .i_d       (i_id_ex_instrn  ),
   .o_q       (ex_instrn       )
);

tt_pipe_stage #(.WIDTH(32)) ex_immed_op_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (i_id_ex_rts & o_ex_id_rtr),
   .i_d       (i_id_immed_op  ),
   .o_q       (ex_immed_op       )
);

wire [31:0] ex_bp_taken_addr; 
wire        ex_bp_branch;
wire        if_ex_bp_valid;
wire        if_ex_bp_taken = i_if_ex_deco[1];
wire [31:0] if_ex_bp_taken_addr = i_if_ex_deco[36:5];

wire ex_bp_predicted;
wire ex_pc_deco_vld;
// Capture any unit dispatch, as any non-branch (ex/fp/vec) op could be predicted Taken from the BTB
// Also clear the stage signal once the mispredict has been handled
tt_pipe_stage #(.WIDTH(1)) ex_predicted_flop
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (i_id_ex_units_rts & o_ex_id_rtr | o_ex_bp_mispredict),
   .i_d       (i_if_ex_predicted & ~o_ex_bp_mispredict),
   .o_q       (ex_bp_predicted)
);

// As EX is processing a branch, we need the "branch taken" and "target addr" from the *next* request in here.. that's why we're not flopping these input signals,
// they are not in sync with the branch instruction that is being evaluated.

assign ex_bp_taken_addr = if_ex_bp_taken_addr;
wire ex_bp_taken = if_ex_bp_taken;

tt_pipe_stage #(.WIDTH(32)) ex_pc_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (i_id_ex_units_rts & o_ex_id_rtr),
   .i_d       (i_id_ex_pc      ),
   .o_q       (ex_pc           )
);

tt_pipe_stage #(.WIDTH(5)) ex_dst_reg_addr_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (i_id_ex_rts & o_ex_id_rtr),
   .i_d       (i_id_fp_rf_wr_flag ? i_id_fp_rf_wraddr : i_id_ex_vecldst ? id_vecldst_rf_wraddr : i_id_rf_wraddr  ),
   .o_q       (ex_rf_wraddr    )
);

tt_pipe_stage #(.WIDTH(8)) ex_misc_sig_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (i_id_ex_units_rts & o_ex_id_rtr),
   .i_d       (if_ex_misc_sig  ),
   .o_q       (ex_misc_sig     )
);

if (INCL_VEC == 1) begin: Gen_VecLdSt   
logic [1:0] id_vecldst_sew, id_vecldst_data_eew, id_vecldst_idx_eew;
logic [2:0] id_vecldst_lmul, id_vecldst_data_emul, id_vecldst_idx_emul;

// V-LD/ST Logic
assign id_vecldst_rf_wraddr = i_id_ex_vecldst_autogen.dest_reg;
   
// Flop signals (These are constant through ex iterations)
assign id_vecldst_sew       = i_ex_vec_csr.v_vsew[1:0];  
assign id_vecldst_lmul      = i_ex_vec_csr.v_lmul;

assign id_vecldst_data_eew  = i_id_ex_vecldst_autogen.ldst_index ? i_ex_vec_csr.v_vsew[1:0] : i_id_ex_instrn[13:12];
assign id_vecldst_data_emul = {3{~i_id_ex_vecldst_autogen.ldst_whole_register}} & get_vecldst_emul(id_vecldst_sew, id_vecldst_lmul, id_vecldst_data_eew);

assign id_vecldst_idx_eew   = i_id_ex_instrn[13:12];
assign id_vecldst_idx_emul  = i_id_ex_vecldst_autogen.ldst_index ? get_vecldst_emul(id_vecldst_sew, id_vecldst_lmul, id_vecldst_idx_eew) : id_vecldst_data_emul;
   
tt_pipe_stage #(.WIDTH($bits(tt_briscv_pkg::vecldst_autogen_s))) ex_vecldst_autogen_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (i_id_ex_rts & o_ex_id_rtr & i_id_ex_vecldst),
   .i_d       (i_id_ex_vecldst_autogen  ),
   .o_q       (ex_vecldst_autogen     )
);
   
tt_pipe_stage #(.WIDTH(3*VLEN)) ex_vecldst_vrf_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (i_id_ex_rts & o_ex_id_rtr & i_id_ex_vecldst),
   .i_d       ({i_vmask_rddata, i_vs2_rddata, i_vs3_rddata}),
   .o_q       ({ex_vmask_reg, ex_vs2_reg, ex_vs3_reg})
);

tt_pipe_stage #(.WIDTH(10)) ex_vecldst_misc_flops
(
   .i_clk     (i_clk),
   .i_reset_n (i_reset_n), 
   .i_en      (i_id_ex_rts & o_ex_id_rtr & i_id_ex_vecldst),
   .i_d       ({id_vecldst_data_eew, id_vecldst_data_emul, id_vecldst_idx_eew, id_vecldst_idx_emul}), 
   .o_q       ({ex_vecldst_data_eew, ex_vecldst_data_emul, ex_vecldst_idx_eew, ex_vecldst_idx_emul}) 
);
   
tt_pipe_stage #(.WIDTH(6)) ex_vecldst_2c_flops
(
   .i_clk     (i_clk),
   .i_reset_n (i_reset_n),
   .i_en      (mem_pipe_rtr_raw),
   .i_d       ({ex_vld,   ex_type_vecldst,   ex_vecldst_start,   ex_vecldst_done,   ex_vecldst_iter_done,   ex_vecldst_elem_mask} & {6{~mem_pipe_rtr}}), 
   .o_q       ({ex_vld_q, ex_type_vecldst_q, ex_vecldst_start_q, ex_vecldst_done_q, ex_vecldst_iter_done_q, ex_vecldst_elem_mask_q})
);
   
tt_pipe_stage #(.WIDTH($bits(tt_briscv_pkg::mem_skidbuf_s))) ex_mem_payload_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_en      (ex_type_vecldst & mem_pipe_rtr_raw),
   .i_d       (ex_mem_payload_fn),
   .o_q       (ex_mem_payload_q)
);

logic [2:0] ex_vecldst_id_lmul_iter, ex_vecldst_id_seg_iter;
logic [3:0] ex_vecldst_num_lmul_iter;
logic [5:0] ex_vecldst_id_iter;
logic [VLEN/8-1:0] ex_vecldst_id_iter_mask;

assign ex_vecldst_start =     (ex_mem_payload_fn.vecldst_128  & (ex_vecldst_elem_count == '0)) |
			      (~ex_mem_payload_fn.vecldst_128 & ((ex_vecldst_data_elem_base + ex_vecldst_elem_count) == '0));
assign ex_vecldst_iter_done = (ex_mem_payload_fn.vecldst_128 & ex_mem_payload_fn.vecldst_idx_last) | 
                              (~ex_mem_payload_fn.vecldst_128 & (ex_vecldst_elem_count == (ex_vecldst_elem_per_iter - 1)));
assign ex_vecldst_done      = ex_mem_payload_fn.vecldst_idx_last;
   
// Mux the vecldst address
always_comb begin
   ex_mem_payload_fn = ex_mem_payload;
   ex_mem_payload_fn.mem_addr = vec_ldst_addr;
   ex_mem_payload_fn.vecldst_idx_last = 1;  
   casez({ex_mem_payload.vecldst_128, ex_mem_payload.mem_sz[1:0]})
      // 3'b1??: begin
      //   ex_mem_payload_fn.mem_byten = '0;
      //   ex_vecldst_store_val_fn     = '0;
      // end
      3'b?00: begin
        ex_mem_payload_fn.mem_byten = ex_vecldst_elem_mask ? '0 : 'h1;
        ex_vecldst_store_val_fn     = {ST_DATA_WIDTH_BITS/8{ex_vecldst_store_val[7:0]}};
      end
      3'b?01: begin
        ex_mem_payload_fn.mem_byten = ex_vecldst_elem_mask ? '0 : 'h3;
        ex_vecldst_store_val_fn     = {ST_DATA_WIDTH_BITS/16{ex_vecldst_store_val[15:0]}};
      end
      3'b?10: begin
        ex_mem_payload_fn.mem_byten = ex_vecldst_elem_mask ? '0 : 'hf;
        ex_vecldst_store_val_fn     = {ST_DATA_WIDTH_BITS/32{ex_vecldst_store_val[31:0]}};
      end
      3'b?11: begin
        ex_mem_payload_fn.mem_byten = ex_vecldst_elem_mask ? '0 : 'hff;
        ex_vecldst_store_val_fn     = {ST_DATA_WIDTH_BITS/64{ex_vecldst_store_val[63:0]}};
      end
      default: begin
        ex_mem_payload_fn.mem_byten = 'hf;
        ex_vecldst_store_val_fn     = '0;
      end
   endcase
   ex_mem_payload_fn.mem_store_data =  ex_mem_payload.mem_store ? ex_vecldst_store_val_fn : ex_vs3_reg;
end   
   
assign ex_vecldst_vl        = i_ex_vec_csr.v_vl;
assign ex_vecldst_vl_div8   = ex_vecldst_vl >> 3;
assign ex_vecldst_mask_num_elem = (ex_vecldst_vl == '0) ? 'h1 : ((ex_vecldst_vl[2:0] == 3'b0) ? ex_vecldst_vl_div8 : (ex_vecldst_vl_div8 + 1'b1));
assign ex_vecldst_nf        = 4'(ex_instrn[31:29] + 1'b1);
assign ex_vecldst_num_lmul_iter = 4'h1 << (ex_vecldst_data_emul[2] ? (ex_vecldst_idx_emul[2] ? 2'h0 : ex_vecldst_idx_emul[1:0]) :
				                                     (ex_vecldst_idx_emul[2] ? ex_vecldst_data_emul[1:0] : 
                                                                                      ((ex_vecldst_idx_emul > ex_vecldst_data_emul) ? ex_vecldst_idx_emul[1:0] : ex_vecldst_data_emul[1:0])));
assign ex_vecldst_id_iter   = ex_vecldst_autogen.ldst_iter_cnt[5:0]; 
assign ex_vecldst_id_iter_mask = ex_vecldst_autogen.ldst_mask ? ((ex_vecldst_vl == '0) ? {VLEN/8{1'b1}} : ({VLEN/8{1'b1}} << ex_vecldst_mask_num_elem)) : 
                                     (({VLEN{1'b1}} << ex_vecldst_vl) >> ($clog2(VLEN))'(ex_vecldst_id_lmul_iter*ex_vecldst_elem_per_iter)) & {VLEN/8{~ex_vecldst_autogen.ldst_whole_register}};  // 1 mean masked. Decode will send all lmul iterations irrespective of vlen
assign ex_vecldst_id_lmul_iter = 3'(ex_vecldst_id_iter%ex_vecldst_num_lmul_iter);   
assign ex_vecldst_id_seg_iter  = 3'(ex_vecldst_id_iter/ex_vecldst_num_lmul_iter);   
   
assign ex_vecldst_data_elem_top_shft[1:0]  = ex_vecldst_data_emul[2] ? 2'(~ex_vecldst_data_emul + 1'b1) : 2'b0;  // For fractional emul
assign ex_vecldst_data_num_elem = ex_vecldst_autogen.ldst_mask ? ex_vecldst_mask_num_elem : 
                                                                 ((ex_vecldst_data_eew[1:0] == 2'h0) ? VLEN/8  : 
                                                                  (ex_vecldst_data_eew[1:0] == 2'h1) ? VLEN/16 : 
                                                                  (ex_vecldst_data_eew[1:0] == 2'h2) ? VLEN/32 :
                                                                                                       VLEN/64  ) >> ex_vecldst_data_elem_top_shft;
assign ex_vecldst_data_elem_base = (ex_vecldst_base_elem_done%ex_vecldst_data_num_elem);

assign ex_vecldst_idx_elem_top_shft[1:0] = ex_vecldst_idx_emul[2] ? 2'(~ex_vecldst_idx_emul + 1'b1) : 2'b0;  // For fractional emul
assign ex_vecldst_idx_num_elem = ((ex_vecldst_idx_eew[1:0] == 2'h0) ? VLEN/8  :
                                  (ex_vecldst_idx_eew[1:0] == 2'h1) ? VLEN/16 :
                                  (ex_vecldst_idx_eew[1:0] == 2'h2) ? VLEN/32 :
                                                                      VLEN/64  ) >> ex_vecldst_idx_elem_top_shft;
assign ex_vecldst_idx_elem_base = (ex_vecldst_base_elem_done%ex_vecldst_idx_num_elem);

assign ex_vecldst_elem_per_iter = ex_vecldst_data_num_elem;
assign ex_vecldst_base_elem_done = (ex_vecldst_id_lmul_iter*ex_vecldst_elem_per_iter);
					     
assign ex_vecldst_store_val = (ex_vs3_reg >> ((ex_vs3_base + {ex_vecldst_elem_count,3'b0}) << ex_vecldst_data_eew));
// Extract the mask bits for the iteration
assign ex_vecldst_iter_mask = ({VLEN/8{~ex_instrn[25]}} & (~ex_vmask_reg >> ($clog2(VLEN))'(ex_vecldst_id_lmul_iter*ex_vecldst_elem_per_iter))) | ex_vecldst_id_iter_mask;
   
assign ex_vecldst_elem_mask = ex_vecldst_iter_mask[ex_vecldst_elem_count];

// Generate the byte mask for vector stores. Used for suppressing the store byte enables (1 implies no store)
always_comb begin
   ex_vecst_byte_mask = '0;
   for (int i=0; i<VLEN/8; i++) begin
      ex_vecst_byte_mask[i]  = ex_mem_payload.vecldst_128 ? (ex_vecldst_data_eew[1:0] == 2'h3 ? ex_vecldst_iter_mask[i/8] :
                                                             ex_vecldst_data_eew[1:0] == 2'h2 ? ex_vecldst_iter_mask[i/4] :
                                                             ex_vecldst_data_eew[1:0] == 2'h1 ? ex_vecldst_iter_mask[i/2] :
                                                                                                ex_vecldst_iter_mask[i  ]  ) :
                                                            ex_vecldst_elem_mask;
   end   
end				      
   
// Get the base of vs3 register for data (in bits). This is needed for Index EMUL > 1 and (Index EMUL > Data EMUL)
assign ex_vs3_base = {ex_vecldst_data_elem_base[$clog2(VLEN/8)-1:0],3'b0} & {$clog2(VLEN/8)+3{ex_vecldst_autogen.ldst_index}};
			       
// Get the op2 for vec ld/st index
always_comb begin
   case(ex_vecldst_idx_eew)
     2'h0: ex_vecldst_idx_op2[63:0] = {56'h0,ex_vs2_reg[( 8*(($clog2(VLEN/8 ))'(ex_vecldst_idx_elem_base + ex_vecldst_elem_count)))+: 8]};
     2'h1: ex_vecldst_idx_op2[63:0] = {48'h0,ex_vs2_reg[(16*(($clog2(VLEN/16))'(ex_vecldst_idx_elem_base + ex_vecldst_elem_count)))+:16]};
     2'h2: ex_vecldst_idx_op2[63:0] = {32'h0,ex_vs2_reg[(32*(($clog2(VLEN/32))'(ex_vecldst_idx_elem_base + ex_vecldst_elem_count)))+:32]};
     2'h3: ex_vecldst_idx_op2[63:0] =        ex_vs2_reg[(64*(($clog2(VLEN/64))'(ex_vecldst_idx_elem_base + ex_vecldst_elem_count)))+:64];
     default: ex_vecldst_idx_op2[63:0] = 'x;
   endcase
end

// Get the vec op2 for AGU
always_comb begin
   ex_vecldst_op2[63:0] = '0;
   case({ex_vecldst_autogen.ldst_ustride, ex_vecldst_autogen.ldst_strided, ex_vecldst_autogen.ldst_index})
      3'b100: begin
         ex_vecldst_op2[63:0] = 64'(ex_mem_payload.vecldst_128 ? (ex_vecldst_autogen.ldst_whole_register ? 16*(ex_vecldst_id_seg_iter + ex_vecldst_elem_count) :
                                                                                                           16*(ex_vecldst_id_lmul_iter + ex_vecldst_elem_count)) :   // Max Elem count = 1 for 128b
                                                        ((ex_vecldst_autogen.ldst_whole_register ? (ex_vecldst_elem_per_iter*ex_vecldst_id_seg_iter + ex_vecldst_elem_count) : // EMUL=1 for whole register ld/st
                                                            (ex_vecldst_id_lmul_iter*ex_vecldst_elem_per_iter*ex_vecldst_nf + ex_vecldst_elem_count*ex_vecldst_nf + ex_vecldst_id_seg_iter)) << ex_vecldst_data_eew));
      end
      3'b010: begin
         ex_vecldst_op2[63:0] = 64'((i_rf_p1_reg*ex_vecldst_id_lmul_iter*ex_vecldst_elem_per_iter) + (i_rf_p1_reg*ex_vecldst_elem_count) + (ex_vecldst_id_seg_iter << ex_vecldst_data_eew));
      end
      3'b001: begin			       
         ex_vecldst_op2[63:0] = 64'(ex_vecldst_idx_op2[63:0] + (ex_vecldst_id_seg_iter << ex_vecldst_data_eew));
      end
      default: ex_vecldst_op2[63:0] = 'x;
   endcase
end

always_ff @(posedge i_clk) begin
   if (~i_reset_n)
     ex_vecldst_elem_count <= 4'b0;
   else if (mem_pipe_rtr & ex_type_vecldst)
     ex_vecldst_elem_count <= 4'b0;
   else if (mem_pipe_rtr_raw & ex_type_vecldst)
     ex_vecldst_elem_count <= ex_vecldst_elem_count + 1'b1;
   else
     ex_vecldst_elem_count <= ex_vecldst_elem_count;
end

end else begin: Gen_NoVecLdst // block: Gen_VecLdSt

   assign ex_vecldst_elem_count  = '0;
   assign ex_vs3_base            = '0;
   assign ex_vecldst_iter_mask   = '0;

   assign ex_vld_q               = '0;
   assign ex_type_vecldst_q      = '0;
   assign ex_vecldst_start_q     = '0;
   assign ex_vecldst_done_q      = '0;
   assign ex_vecldst_iter_done_q = '0;
   assign ex_vecldst_elem_mask_q = '0;
   assign ex_mem_payload_q       = '0;

   assign id_vecldst_rf_wraddr   = '0;
   assign ex_vecldst_data_eew    = '0;
   assign ex_vecldst_autogen     = '0;
   assign ex_vecldst_nf          = '0;

 
end
   
wire branch_taken;
wire ex_branch_target_rts = ex_disp_vld &  ex_type_sb;
wire ex_jalr_target_rts   = ex_disp_vld &  ((ex_instrn[6:0] == 7'b1100111) | (ex_instrn[6:0] == 7'b1101111));
assign o_ex_branch_taken = branch_taken | ex_jalr_target_rts ;

wire [31:0] ex_target_pc;
wire   branch_addr_match  = ~(ex_bp_predicted && ex_bp_taken) || (ex_bp_taken_addr == ex_target_pc);

// BP predicted taken, but it wasn't a branch at all
assign o_ex_is_some_branch = ex_branch_target_rts | ex_jalr_target_rts; 
wire mis_bp_taken_not_br = ex_disp_vld & units_vld &  ex_bp_predicted &  ex_bp_taken & ~o_ex_is_some_branch;

//Target mispredicts
// BP predicted, and the target doesn't match
wire mis_bp_addr_mismatch = ex_disp_vld &  ex_bp_predicted & ex_bp_taken & (ex_bp_taken_addr != o_ex_bp_pc);

//Direction mispredicts
// Branch taken, but BP either missed it or predicted it wouldn't be
wire mis_bp_ntkn_ex_tkn = ~(ex_bp_taken & ex_bp_predicted) & o_ex_branch_taken;
// Branch not taken, but BP said taken
wire mis_bp_tkn_ex_ntkn = units_vld & (ex_bp_predicted & ex_bp_taken) & ~o_ex_branch_taken;

assign o_ex_bp_mispredict        = ex_disp_vld & (mis_bp_taken_not_br | mis_bp_addr_mismatch | (mis_bp_ntkn_ex_tkn | mis_bp_tkn_ex_ntkn));
assign o_ex_bp_mispredict_not_br = mis_bp_taken_not_br;
assign o_ex_bp_pc                = o_ex_branch_taken ? ex_target_pc : (ex_pc + 'd4);

`ifdef SIM
//  `ASSERT_CLK(1'b1,"ERROR: Basic Assert check to see if this works");

// This is a subset of below signal:mis_bp_target_not_pred. Commenting out
// BP predicted not taken, but it was a taken branch
wire OLD_mis_bp_not_ex_taken = ex_disp_vld & !int_div_stall & ex_bp_predicted & ~ex_bp_taken & ( (branch_taken & ex_branch_target_rts) | ex_jalr_target_rts);

`ASSERT_COND_CLK(OLD_mis_bp_not_ex_taken, mis_bp_ntkn_ex_tkn,"ERROR: BP redundant term check failed MisPred = %b, OldTerm = %b", mis_bp_ntkn_ex_tkn, OLD_mis_bp_not_ex_taken);
`endif

// reg holding_predicted;
// always @(posedge i_clk)
//   if (~i_reset_n)
//     holding_predicted <= 1'b0;
//   else if (mem_pipe_rtr)
//     holding_predicted <= 1'b0;
//   else if (ex_vld & !int_div_stall & ex_bp_predicted)
//     holding_predicted <= 1'b1;

assign o_ex_bp_fifo_pop   = (mem_pipe_rtr & units_vld & ex_bp_predicted );
//                          || (holding_predicted & mem_pipe_rtr); // Ashok: Id never sends unless ex is ready to receive. This holding logic seems stale. Commenting out

/* verilator lint_off UNUSED */
reg [63:0] ex_ascii_instrn;
/* verilator lint_on UNUSED */

tt_ascii_instrn_decode ascii_decode
(
  .i_instrn      (ex_instrn      ),
  .o_ascii_instrn(ex_ascii_instrn)
);

// Separate AGU adder
logic [31:0] agu_op1, agu_op2;

if (INCL_VEC == 1) begin   
   logic [63:0] vec_agu_op1, vec_agu_op2;
   assign vec_agu_op1[63:0] = i_rf_p0_reg;
   assign vec_agu_op2[63:0] = ex_vecldst_op2;
   assign vec_ldst_addr = vec_agu_op1 + vec_agu_op2;
end else begin
   assign vec_ldst_addr = '0;
end
   
assign agu_op1[31:0] = i_rf_p0_reg[31:0];
assign agu_op2[31:0] = ex_immed_op & {32{~ex_type_amo}};
assign ldst_addr     = agu_op1 + agu_op2;
   
////////
//// Assemble operands
reg [31:0] op1;
reg [31:0] op2;
wire       load = (ex_instrn[6:0] == 7'b0000011);
wire       ex_type_fldst = (ex_instrn[14:12] inside {3'b001, 3'b010}) & ((ex_instrn[6:0] == 7'b0000111) | (ex_instrn[6:0] == 7'b0100111));
wire       ex_type_fld16 = (ex_instrn[14:12] == 3'b001) & (ex_instrn[6:0] == 7'b0000111);
wire       ex_type_vsetvl = (ex_type_c1 | ex_type_c2 | ex_type_c3);
wire [7:0] instrn_type  = {(ex_type_amo | ex_type_fldst | ex_type_vsetvl), ex_type_r, ex_type_i, ex_type_s , ex_type_sb , ex_type_u , ex_type_uj, ex_type_e};
wire [2:0] func = ex_instrn[14:12];
wire       func_secondary = ex_instrn[30];
reg [8:0]  decoded_op;
always @* begin
   /* verilator lint_off CASEINCOMPLETE */
   /* verilator lint_off CASEOVERLAP */
   casez(instrn_type) // synopsys full_case parallel_case
    8'b1???_????: // vsetvl
    begin
      op1 = ex_type_c2 ? {27'h0, ex_instrn[19:15]} : i_rf_p0_reg;
      op2 = ex_type_fldst ? ex_immed_op : ex_type_c3 ? i_rf_p1_reg : '0;
      decoded_op = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // Doesn't matter
    end
    8'b?1??_????: // reg 2 reg
    begin
      op1 = i_rf_p0_reg;
      op2 = i_rf_p1_reg;
      decoded_op = decode_type_r(func,func_secondary);
    end
    8'b??1?_????: // normal immediate - ALU + LOAD + JALR
    begin
      op1 = i_rf_p0_reg;
      op2 = ex_immed_op;
      decoded_op = decode_type_i(func,func_secondary,load);
    end
    8'b???1_????: // STORE immediate address calculation
    begin
      op1 = i_rf_p0_reg;
      op2 = ex_immed_op;
      decoded_op = decode_type_s();
    end
    8'b????_1???: // STORE branch immediate - comparison of registers
    begin
      op1 = i_rf_p0_reg;
      op2 = i_rf_p1_reg;
      decoded_op = decode_type_sb(func);
    end
    8'b????_?1??: // unsigned - LUI, AUIPC - add 0 to immed, or PC to immed, respectively
    begin
      op1 = (ex_instrn[6:0] == 7'b0110111) ? 'd0 : ex_pc;
      op2 = ex_immed_op;
      decoded_op = decode_type_u();
    end
    8'b????_??1?: // unsigned jump (JAL) - nothing to do here, just leave at default mux settings
    begin
      op1 = ex_pc;
      op2 = ex_immed_op;
      decoded_op = decode_type_uj();
    end
    8'b????_???1: // CSRs
    begin
      op1 = func[2] ? {27'b0, ex_instrn[19:15]} : i_rf_p0_reg;
      op2 = i_rf_p1_reg;
      decoded_op = decode_type_e(func);
    end
    default:
    begin
      op1 = 'bx;
      op2 = 'bx;
      decoded_op = 'bx;
    end
   endcase
   /* verilator lint_on CASEINCOMPLETE */
   /* verilator lint_on CASEOVERLAP */
end

////////
//// Execute

//// CSRs
//TODO: This is temporary, forcing CSR's reads and writes here, this does not match spec and a 
// detailed implemetation needs to be done

wire [11:0] csr_wr_addr;
wire [31:0] csr_wr_data;
wire [1:0] csr_wr_mask;
wire csr_read;
wire csr_write;   
wire [11:0] csr_rd_addr;
logic [31:0] csr_rdata;

assign csr_write = o_ex_mem_vld
		   & (ex_type_e ? (  (func[1] & ~(ex_instrn[19:15] == 5'b0)) // NO write if rs1/uimm[4:0] == x0
                          || (func[2] | func[0]))
		                : (ex_type_c1 | ex_type_c2 | ex_type_c3));  

assign csr_read = ex_type_e && ~(ex_rf_wraddr == 5'b0); // & func[1]; // NO read if rd == x0

assign csr_wr_addr = ex_type_e ? ex_instrn[31:20] :
		    (ex_type_c1 | ex_type_c2 | ex_type_c3) ? '0 :
		    12'b0;
assign csr_rd_addr = ex_type_e ? ex_instrn[31:20] :
		     (ex_type_c1 | ex_type_c2 | ex_type_c3) ? '0 :
		     12'b0;

assign csr_wr_data = ({32{ex_type_e}}  & op1)                      |
		     ({32{ex_type_c1}} & {21'b0,ex_instrn[30:20]}) |
		     ({32{ex_type_c2}} & {22'b0,ex_instrn[29:20]}) |
		     ({32{ex_type_c3}} & op2);

// 2b zero-hot value: 1-Clear, 0-Set
assign csr_wr_mask =   ex_type_e & (func[1:0] == 2'b10) ? 2'b01 // Csr set instruction mask
                     : ex_type_e & (func[1:0] == 2'b11) ? 2'b10 // Csr clear instruction mask
                     : 2'b0;

`ifdef SIM
`ASSERT_COND_CLK(ex_vld, $onehot0({ex_type_e, ex_type_c1, ex_type_c2, ex_type_c3}), "csr_wr_data selects are not one-hot");
`endif
   
// Derive vl from avl using instrn:rs1 or imm. Only matters for type_c1/c2/c3. Dont care in other cases
wire [7:0] csr_avl;
wire [7:0] csr_vl;
wire [7:0] csr_vl_wrdata;
wire [7:0] csr_vlmax;
assign csr_avl = &ex_instrn[31:30]    ? {3'd0,ex_instrn[19:15]}                               // vsetivli
                                      : (|ex_instrn[19:15])  ? {(|op1[31:7]),op1[6:0]}        // (rs1 != x0). VLMAX <= 128 so reduce op1 to 8 bits
                                                             : (|ex_instrn[11:7]) ? 8'hff     // (rs1 == x0) & (rd != x0)  Set AVL > 128 so that vl == vlmax
                                                                                  : csr_vl ;  // (rs1 == x0) & (rd == x0)
assign csr_vl_wrdata = (csr_avl <= csr_vlmax) ? csr_avl : csr_vlmax;
   
//tt_csr csr (
//	    .rdata			(csr_rdata[31:0]),
//	    .write			(csr_write),
//	    .waddr			(csr_wr_addr[11:0]),
//	    .raddr			(csr_rd_addr[11:0]),
//	    .wdata			(csr_wr_data[31:0]),
//	    .wmask			(csr_wr_mask[1:0]),
//
//            // Vec
//	    .i_sat_csr                  (i_sat_csr),
//	    .i_csr_vl			(csr_vl_wrdata),
//	    .o_csr_vlmax		(csr_vlmax),
//	    .o_csr_vl		        (csr_vl),
//  
//            .o_csr_pmacfg0              (csr_pmacfg0),
//            .o_csr_pmacfg1              (csr_pmacfg1),
//
//            .i_exc_fp_ex_update         (i_exc_fp_ex_update),
//            .i_exc_vfp_update           (i_exc_vfp_update),
//
//            // ID
//            .i_id_ex_instdisp           (i_id_ex_instdisp),
//
//	    .o_csr_to_id		(o_ex_id_csr),
//	    .o_csr_to_vec		(o_ex_vec_csr),
//
//            /*AUTOINST*/
//	    // Inputs
//	    .i_clk			(i_clk),
//	    .i_reset_n			(i_reset_n));
assign csr_rdata = '0;
assign csr_vlmax = '0;
assign csr_vl = '0;
assign csr_pmacfg0 = '0;
assign csr_pmacfg1 = '0;
assign o_ex_id_csr = '0;
assign o_ex_vec_csr = '0;

reg [31:0] mul_res_d2;
wire sub_flavor = decoded_op[3];

wire unsigned_flavor = decoded_op[4];
wire [1:0] return_type = decoded_op[6:5];
wire result_or_flag = decoded_op[7];

   
always @* begin
  casez(decoded_op[2:0]) // synopsys full_case parallel_case
    3'd0: alu_result = execute_add(op1, op2, unsigned_flavor, sub_flavor, result_or_flag, return_type);
    3'd1: alu_result = op1 ^ op2;
    3'd2: alu_result = op1 | op2;
    3'd3: alu_result = op1 & op2;
    3'd4: alu_result = {31'h00000000,(op1 == op2)};
    3'd5: alu_result = op1 << (ex_type_i ? ex_instrn[24:20] : op2[4:0]);
    3'd6: alu_result = op1 >> (ex_type_i ? ex_instrn[24:20] : op2[4:0]);
    3'd7: alu_result = $signed(op1) >>> (ex_type_i ? ex_instrn[24:20] : op2[4:0]);
  endcase  
end

////////
// Branch and JALR related calculations
wire [31:0] branch_pc_val = alu_result[0] ? (ex_pc + ex_immed_op) : (ex_pc + 32'd4); // immed op is already sign extended
wire [31:0] jalr_pc_val = {alu_result[31:1],1'b0}; // zero the bottom bit per RV32I ISA Spec
assign ex_target_pc     = ex_type_sb ? branch_pc_val : jalr_pc_val;
assign branch_taken = alu_result[0] & ex_branch_target_rts;
wire branch_not_taken = (!alu_result[0]) & ex_branch_target_rts;

////////
// Outputs to MEM

logic [ST_DATA_WIDTH_BITS-1:0] ex_store_val_fn;
logic [31:0] ex_store_val;
assign ex_store_val  = '0;
logic [1:0]    store_case_muxsel;
assign         store_case_muxsel[1:0] = ex_mem_payload.mem_sz[1:0]; 
always_comb begin
  casez({store_case_muxsel})
    2'b00: begin
      ex_mem_payload.mem_byten = 'h1;
      ex_store_val_fn   = {ST_DATA_WIDTH_BITS/8{ex_store_val[7:0]}};
    end
    2'b01: begin
      ex_mem_payload.mem_byten = 'h3;
      ex_store_val_fn   = {ST_DATA_WIDTH_BITS/16{ex_store_val[15:0]}};
    end
    2'b10: begin
      ex_mem_payload.mem_byten = 'hf;
      ex_store_val_fn   = {ST_DATA_WIDTH_BITS/32{ex_store_val[31:0]}};
    end
    default: begin
      ex_mem_payload.mem_byten = 'hf;
      ex_store_val_fn   = {ST_DATA_WIDTH_BITS/32{ex_store_val[31:0]}};
    end
  endcase
end

assign ex_mem_payload.mem_addr  = ldst_addr;
assign ex_mem_payload.mem_sz    = ex_type_vecldst ? {1'b0,ex_vecldst_data_eew[1:0]} : ex_type_fld16 ? 3'b110 : ex_instrn[14:12];
assign ex_mem_payload.mem_store = ex_vld & ((ex_instrn[6:0] == 7'b0100011) |                                    // RV32I stores 
                                        ((ex_instrn[6:0] == 7'b0100111) & (ex_instrn[14:12] inside {3'b001, 3'b010})) |  // RV32F stores
					(ex_type_vecldst & ex_vecldst_autogen.store) );
assign ex_mem_payload.mem_load  = ex_vld & ((ex_instrn[6:0] == 7'b0000011) |                                    // RV32I loads
					((ex_instrn[6:0] == 7'b0000111) & (ex_instrn[14:12] inside {3'b001, 3'b010})) |  // RV32F loads
					(ex_type_vecldst & ex_vecldst_autogen.load));                                    // RV32V stores
assign ex_mem_payload.vecldst_vld = ex_type_vecldst;
assign ex_mem_payload.mem_fence = (ex_instrn[6:0] == 7'b0001111) & (ex_instrn[14:12] == 3'b000);

// 128b mode for
// 1. Regular and fault-first unit stride with nf=0
// 2. Whole register unit stride load for all nf values
// 3. Unit stride mask load (Covered by 1 since nf=0 for mask loads)
assign ex_mem_payload.vecldst_128 = 1'b1; // ex_type_vecldst & ex_vecldst_autogen.ldst_ustride & ~ex_vecldst_autogen.ldst_mask & ((ex_vecldst_nf == 4'h1) | ex_vecldst_autogen.ldst_whole_register);
assign ex_mem_payload.mem_amo     = ex_type_amo;
assign ex_mem_payload.mem_amotype = ex_instrn[31:27];
   
if (INCL_VEC == 1) begin
   assign ex_mem_payload.vecldst_idx = (ex_vecldst_elem_count + ex_vs3_base[$clog2(VLEN/8)+2:3]);
   assign ex_mem_payload.vecldst_idx_last = '0;   

   // This is byte mask   
   for (genvar i=0; i<VLEN/8; i++) begin
      assign ex_mem_payload.vecldst_byte_mask[i] = ex_type_vecldst & (ex_vecldst_data_eew[1:0] == 2'h3 ? ex_vecldst_iter_mask[i/8] :
                                                                      ex_vecldst_data_eew[1:0] == 2'h2 ? ex_vecldst_iter_mask[i/4] :
                                                                      ex_vecldst_data_eew[1:0] == 2'h1 ? ex_vecldst_iter_mask[i/2] :
                                                                                                         ex_vecldst_iter_mask[i  ]  );
   end
end else begin
   assign ex_mem_payload.vecldst_idx = '0;
   assign ex_mem_payload.vecldst_idx_last = '0;
   assign ex_mem_payload.vecldst_byte_mask = '0;
end
    
assign ex_mem_payload.mem_store_data = ex_store_val_fn;
assign ex_mem_payload.mem_alu_result[31:0] =  ({32{ex_type_bit}}                                         & bit_res)            | 
                                              ({32{ex_type_div}}                                         & int_div_res)        |
                                              ({32{ex_type_jal}}                                         & 32'(ex_pc + 32'd4)) |
                                              ({32{csr_read}}                                            & csr_rdata)          |
                                              ({32{ex_type_vsetvl}}                                      & csr_vl_wrdata)      |
                                              ({32{~(ex_type_bit | ex_type_div | ex_type_jal | csr_read | ex_type_vsetvl)}} & alu_result);

assign ex_mem_payload.mem_tx_valid = 1'b0;
assign ex_mem_payload.mem_lq_valid = 1'b0;
assign ex_mem_payload.mem_lqid     = '0;

				     
// Send the 1c or 2c payload to mem (1c and 2c can't be active at the same)
assign o_ex_mem_payload = (ex_vld_q & ex_type_vecldst_q) ? ex_mem_payload_q : ex_mem_payload;
   
// Forwarding controls to ID
wire mem_instrn_1c = ex_mem_payload.mem_store | ex_mem_payload.mem_load | ex_mem_payload.mem_amo;
assign o_ex_dst_vld_1c     = o_ex_mem_lqvld_1c & ~mem_instrn_1c;
assign o_ex_dst_lqid_1c    = o_ex_mem_lqid_1c;
assign o_ex_fwd_data_1c    = o_ex_mem_lqdata_1c;
  
assign o_ex_dst_vld_2c     = o_ex_mem_lqvld_2c;
assign o_ex_dst_lqid_2c    = o_ex_mem_lqid_2c;
assign o_ex_fwd_data_2c    = o_ex_mem_lqdata_2c;
  
////////

// PMA Config
// CSR mapping: Bit0 - Valid, Mask - Bit1 all the way to first 0 (minimum mask is 4 bytes)
logic        pmacfg0_vld, pmacfg1_vld;
logic [31:0] pmacfg0_mask, pmacfg1_mask;
logic 	     pmacfg0_match, pmacfg1_match;
   
assign pmacfg0_vld = csr_pmacfg0[0];
assign pmacfg0_match = |((ex_mem_payload.mem_addr[31:0] | pmacfg0_mask[31:0]) == (csr_pmacfg0[31:0] | pmacfg0_mask[31:0]));
always_comb begin
   pmacfg0_mask[31:0] = 32'h1;
   for (int i=1; i<32; i++) begin
      pmacfg0_mask[i] = 1'b1;
      if (~pmacfg0_mask[i]) break;
   end
end

assign pmacfg1_vld = csr_pmacfg1[0];
assign pmacfg1_match = |((ex_mem_payload.mem_addr[31:0] | pmacfg1_mask[31:0]) == (csr_pmacfg1[31:0] | pmacfg1_mask[31:0]));
always_comb begin
   pmacfg1_mask[31:0] = 32'h1;
   for (int i=1; i<32; i++) begin
      pmacfg1_mask[i] = 1'b1;
      if (~pmacfg1_mask[i]) break;
   end
end
   
assign ex_mem_payload.mem_ordered = ((pmacfg0_vld & pmacfg0_match) | (pmacfg1_vld & pmacfg1_match)) & (ex_mem_payload.mem_store | ex_mem_payload.mem_load);  // AMO are always ordered so don't need to put them here

//Integer multiplier.
logic [63:0] mul_csa_s_d1,mul_csa_r_d1;
logic [64:0] mul_sum_d1;
logic        mul_issgn;
logic        mul_retl,mul_reth; 
logic        mul_reth_d1;
logic        mul_src2_unsgn;
  
assign mul_en         = ex_vld & ((ex_instrn[6:0] == 7'b0110011)) & ex_instrn[25] & !ex_instrn[14];
assign mul_issgn      =  func[2:0] != 3'b011;
logic mul_issrc2sgn; assign mul_issrc2sgn  = ~func[1]; //MM Nov 5 2021: Remove implicit wire.
assign mul_reth       = func[2:0] != 3'b000;   

//tt_int_mul_32 mul32 (.i_src1_0a(op1[31:0]), .i_src2_0a(op2[31:0]), .i_issgn_0a(mul_issgn), .i_issgnsrc2_0a(mul_issrc2sgn), .i_clk(i_clk), .i_mulen_0a(mul_en), .o_s6_1a(mul_csa_s_d1[63:0]), .o_r6_1a(mul_csa_r_d1[63:0]));
assign mul_csa_s_d1 = '0;
assign mul_csa_r_d1 = '0;

assign mul_sum_d1[64:0] = {1'b0, mul_csa_s_d1[63:0]} + {1'b0, mul_csa_r_d1[63:0]};
assign mul_res_d1[31:0] = mul_reth_d1 ? mul_sum_d1[63:32] : mul_sum_d1[31:0];
   
always_ff @(posedge i_clk) begin
   mul_reth_d1 <= mul_reth;
   mul_en_d1   <= mul_en; // Should be enabled for next cycle when trxn is passed to mem
   if (mul_en)  ex_lqid_d1  <= ex_lqid;
end

//////////////////////
// Integer Division // 
//////////////////////
//tt_int_div_r2 #(.XLEN(XLEN))
//int_div
//(
//   .i_clk,
//   .i_reset_n,
//   .i_vld(int_div_vld    ),
//   .o_ack(int_div_ack    ),
//   .i_sgn(int_div_sgn    ), // 0: unsigned, 1: signed
//   .i_rem(int_div_rem    ), // 0: quotient, 1: remainder
//   .i_rs1(int_div_rs1    ), // Dividend
//   .i_rs2(int_div_rs2    ), // Divisor
//   .o_rts(int_div_res_rts),
//   .i_rtr(int_div_res_rtr),
//   .o_res(int_div_res    )  // Result
//);
assign int_div_ack = '0;
assign int_div_res_rts = '0;
assign int_div_res = '0;

assign int_div_vld     = ex_vld && ex_type_div;
assign int_div_sgn     = !func[0];
assign int_div_rem     =  func[1];
assign int_div_rs1     =  i_rf_p0_reg[XLEN-1:0];
assign int_div_rs2     =  i_rf_p1_reg[XLEN-1:0];
assign int_div_res_rtr =  mem_pipe_rtr;

assign int_div_stall   =  int_div_vld && !int_div_res_rts;
 
//////////////////////
// Bit Manipulation //
//////////////////////
// Capture instruction for Bit Manipulator unit
always @(posedge i_clk) begin
   if (~i_reset_n) begin
      ex_Zb_instr <= '0;
   end else begin
      // Reset when a bitmanip instruction executed
      if ( bit_vld      &&
           mem_pipe_rtr &&
          !i_id_ex_rts    ) begin
         ex_Zb_instr <= '0;
      end else
      if ( i_id_ex_rts &&
           o_ex_id_rtr   ) begin
         ex_Zb_instr <= i_id_ex_Zb_instr;
      end
   end
end

//tt_bitmanip bitmanip
//(
//   .i_clk,
//   .i_reset_n,
//   .i_vld(bit_vld),
//   .i_cmd(bit_cmd),
//   .i_rs1(bit_rs1),
//   .i_rs2(bit_rs2),
//   .o_res(bit_res)
//);
assign bit_res = '0;

assign bit_vld = ex_vld && ex_type_bit;
assign bit_cmd = tt_briscv_pkg::bitmanip_instrn_e'(ex_Zb_instr);
assign bit_rs1 = i_rf_p0_reg;
assign bit_rs2 = (ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_I  || // IMPROVE DJX clean this one once Enctype is fixed
                  ex_misc_sig[6:2] == `BRISCV_INSTR_TYPE_IB   ) ? {25'h0, ex_immed_op[6:0]} 
                                                                : i_rf_p1_reg;


//// Functions

// ADD flavor bits:
// - sub
// - unsigned
// - gte vs. lte
// - equal
// - branch
// - store
// 4'h0 : ADD
// 4'h0 : SUB - ADD, flavor sub
// 4'h0 : GTE - ADD, flavor sub
// 4'h0 : GTEU - ADD, flavor sub unsigned
// 4'h0 : LTE - ADD, flavor sub
// 4'h0 : LTEU - ADD, flavor sub unsigned
// 4'h1 : XOR
// 4'h2 : OR
// 4'h3 : AND
// 4'h4 : EQ
// 4'h5 : SLLI, SLL - logical shift left (zeros go in bottom bits) - for SLL shift amount is in [4:0] of rs2
// 4'h6 : SRLI, SRL - logical shift right - zeros go in top bits - for SRL shift amount is in [4:0] of rs2
// 4'h7 : SRAI, SRA - arithmetic shift right - sign bit goes in top bits - for SRA shift amount is in [4:0] of rs2

/* verilator lint_off VARHIDDEN */
function automatic [2:0] get_vecldst_emul;
   input [1:0] sew;
   input [2:0] lmul;
   input [1:0] eew;

   logic [2:0] emul;
   
   logic [2:0] eew_divide_sew;  

   eew_divide_sew[2:0] = ({3{(eew == 2'b00) & (sew == 2'b00)}} & 3'b000) |    // EEW/SEW=1 
                         ({3{(eew == 2'b00) & (sew == 2'b01)}} & 3'b111) |    // EEW/SEW=1/2 
                         ({3{(eew == 2'b00) & (sew == 2'b10)}} & 3'b110) |    // EEW/SEW=1/4 
                         ({3{(eew == 2'b00) & (sew == 2'b11)}} & 3'b101) |    // EEW/SEW=1/8 
                         ({3{(eew == 2'b01) & (sew == 2'b00)}} & 3'b001) |    // EEW/SEW=2 
                         ({3{(eew == 2'b01) & (sew == 2'b01)}} & 3'b000) |    // EEW/SEW=1 
                         ({3{(eew == 2'b01) & (sew == 2'b10)}} & 3'b111) |    // EEW/SEW=1/2 
                         ({3{(eew == 2'b01) & (sew == 2'b11)}} & 3'b110) |    // EEW/SEW=1/4 
                         ({3{(eew == 2'b10) & (sew == 2'b00)}} & 3'b010) |    // EEW/SEW=4 
                         ({3{(eew == 2'b10) & (sew == 2'b01)}} & 3'b001) |    // EEW/SEW=2 
                         ({3{(eew == 2'b10) & (sew == 2'b10)}} & 3'b000) |    // EEW/SEW=1 
                         ({3{(eew == 2'b10) & (sew == 2'b11)}} & 3'b111) |    // EEW/SEW=1/2
                         ({3{(eew == 2'b11) & (sew == 2'b00)}} & 3'b011) |    // EEW/SEW=8 
                         ({3{(eew == 2'b11) & (sew == 2'b01)}} & 3'b010) |    // EEW/SEW=4 
                         ({3{(eew == 2'b11) & (sew == 2'b10)}} & 3'b001) |    // EEW/SEW=2 
                         ({3{(eew == 2'b11) & (sew == 2'b11)}} & 3'b000);     // EEW/SEW=1
			       
   // EMUL = (EEW/SEW)*LMUL
   case(lmul)
      3'b101: begin  // LMUL=1/8
         emul = ({3{eew_divide_sew == 3'b000}} & 3'b101) |  // EMUL = (1)*(1/8)   = 1/8 
                ({3{eew_divide_sew == 3'b001}} & 3'b110) |  // EMUL = (2)*(1/8)   = 1/4 
                ({3{eew_divide_sew == 3'b010}} & 3'b111) |  // EMUL = (4)*(1/8)   = 1/2 
                ({3{eew_divide_sew == 3'b011}} & 3'b000);   // EMUL = (8)*(1/8)   = 1
      end
      3'b110: begin  // LMUL=1/4
         emul = ({3{eew_divide_sew == 3'b111}} & 3'b101) |  // EMUL = (1/2)*(1/4) = 1/8 
                ({3{eew_divide_sew == 3'b000}} & 3'b110) |  // EMUL = (1)*(1/4)   = 1/4 
                ({3{eew_divide_sew == 3'b001}} & 3'b111) |  // EMUL = (2)*(1/4)   = 1/2 
                ({3{eew_divide_sew == 3'b010}} & 3'b000) |  // EMUL = (4)*(1/4)   = 1 
                ({3{eew_divide_sew == 3'b011}} & 3'b001);   // EMUL = (8)*(1/4)   = 2 
      end
      3'b111: begin  // LMUL=1/2
         emul = ({3{eew_divide_sew == 3'b110}} & 3'b101) |  // EMUL = (1/4)*(1/2) = 1/8
                ({3{eew_divide_sew == 3'b111}} & 3'b110) |  // EMUL = (1/2)*(1/2) = 1/4 
                ({3{eew_divide_sew == 3'b000}} & 3'b111) |  // EMUL = (1)*(1/2)   = 1/2 
                ({3{eew_divide_sew == 3'b001}} & 3'b000) |  // EMUL = (2)*(1/2)   = 1 
                ({3{eew_divide_sew == 3'b010}} & 3'b001) |  // EMUL = (4)*(1/2)   = 2 
                ({3{eew_divide_sew == 3'b011}} & 3'b010);   // EMUL = (8)*(1/2)   = 4 
      end
      3'b000: begin  // LMUL=1
         emul = eew_divide_sew[2:0];
      end
      3'b001: begin  // LMUL=2
         emul = ({3{eew_divide_sew == 3'b110}} & 3'b111) |  // EMUL = (1/4)*2 = 1/2
                ({3{eew_divide_sew == 3'b111}} & 3'b000) |  // EMUL = (1/2)*2 = 1 
                ({3{eew_divide_sew == 3'b000}} & 3'b001) |  // EMUL = (1)*2 = 2 
                ({3{eew_divide_sew == 3'b001}} & 3'b010) |  // EMUL = (2)*2 = 4 
                ({3{eew_divide_sew == 3'b010}} & 3'b011);   // EMUL = (4)*2 = 8 
      end
      3'b010: begin  // LMUL=4
         emul = ({3{eew_divide_sew == 3'b110}} & 3'b000) |  // EMUL = (1/4)*4 = 1
                ({3{eew_divide_sew == 3'b111}} & 3'b001) |  // EMUL = (1/2)*4 = 2 
                ({3{eew_divide_sew == 3'b000}} & 3'b010) |  // EMUL = (1)*4 = 4 
                ({3{eew_divide_sew == 3'b001}} & 3'b011);   // EMUL = (2)*4 = 8 
      end
      3'b011: begin  // LMUL=8
         emul = ({3{eew_divide_sew == 3'b110}} & 3'b001) |  // EMUL = (1/4)*8 = 2
                ({3{eew_divide_sew == 3'b111}} & 3'b010) |  // EMUL = (1/2)*8 = 4 
                ({3{eew_divide_sew == 3'b000}} & 3'b011);   // EMUL = (1)*8 = 8 
      end
      default: emul = 3'b0;
   endcase
   return emul;
endfunction      

function automatic [31:0] execute_add;
input [31:0] op1;
input [31:0] op2;
input        unsigned_flavor;
input        sub            ;
input        return_result_or_flag;
input [ 1:0] return_type    ; // 0 - eq, 1 - neq, 2 - lt flag, 3 - gt flag

reg   [31:0] res;
reg          ge;
reg          lt;
reg          eq;
reg          neq;
  begin
    if(sub) begin
      res = $signed(op1) - $signed(op2);
      if(unsigned_flavor) begin
        eq       = (op1 == op2);
        neq      = (op1 != op2);
        lt       = (op1  < op2);
        ge       = (op1  >= op2);
      end
      else begin
        lt       = ($signed(op1) < $signed(op2));
        ge       = ($signed(op1) >= $signed(op2));
        neq      = (op1 != op2);
        eq       = (op1 == op2);
      end
    end
    else begin
      res = op1 + op2;
      eq  = 1'b0;
      neq = 1'b0;
      lt  = 1'b0;
      ge  = 1'b0;
    end

    if(return_result_or_flag) execute_add = res;
    else begin
      case(return_type)
        2'b00: execute_add = {31'h00000000,eq };
        2'b01: execute_add = {31'h00000000,neq};
        2'b10: execute_add = {31'h00000000,lt };
        2'b11: execute_add = {31'h00000000,ge };
      endcase
    end
  end
endfunction

function automatic [8:0] decode_type_s;
  begin
    // {branch flag, return result or flag, flag return type, unsigned flavor, sub flavor, decoded op}
    decode_type_s = {1'b0,1'b1,2'b00,1'b0,1'b0,3'b000};
  end
endfunction

function automatic [8:0] decode_type_u;
  begin
    decode_type_u = {1'b0,1'b1,2'b00,1'b0,1'b0,3'b000};
  end
endfunction

function automatic [8:0] decode_type_uj;
  begin
    decode_type_uj = {1'b0,1'b1,2'b00,1'b0,1'b0,3'b000};
  end
endfunction

function automatic [8:0] decode_type_e;
//TODO: Temporary needs to be modified to match the CSR spec
input [2:0] func;
  begin
    /* verilator lint_off CASEINCOMPLETE */
    /* verilator lint_off CASEOVERLAP */
    casez(func) // synopsys full_case parallel_case
                            // {branch flag, return result or flag, flag return type, unsigned flavor, sub flavor, decoded op}
    3'b000: decode_type_e = {1'b0,1'b0, 2'b00,1'b0,1'b0,3'b000}; // CSRRW
    3'b001: decode_type_e = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // CSRRS
    3'b100: decode_type_e = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // CSRRC
    3'b101: decode_type_e = {1'b0,1'b0, 2'b00,1'b0,1'b0,3'b000}; // CSRRWI
    3'b110: decode_type_e = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // CSRRSI
    3'b111: decode_type_e = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // CSRRCI
    `ifdef SIM
    default: decode_type_e = 'bx;
    `endif
    endcase
   /* verilator lint_on CASEINCOMPLETE */
   /* verilator lint_on CASEOVERLAP */
  end
endfunction


function automatic [8:0] decode_type_sb;
input [2:0] func;
  begin
    /* verilator lint_off CASEINCOMPLETE */
    /* verilator lint_off CASEOVERLAP */
    casez(func) // synopsys full_case parallel_case
                            // {branch flag, return result or flag, flag return type, unsigned flavor, sub flavor, decoded op}
    3'b000: decode_type_sb = {1'b1,1'b0, 2'b00,1'b0,1'b1,3'b000}; // ADD BEQ
    3'b001: decode_type_sb = {1'b1,1'b0, 2'b01,1'b0,1'b1,3'b000}; // SUB NE
    3'b100: decode_type_sb = {1'b1,1'b0, 2'b10,1'b0,1'b1,3'b000}; // SUB LT
    3'b101: decode_type_sb = {1'b1,1'b0, 2'b11,1'b0,1'b1,3'b000}; // SUB GE
    3'b110: decode_type_sb = {1'b1,1'b0, 2'b10,1'b1,1'b1,3'b000}; // SUB unsigned LTU
    3'b111: decode_type_sb = {1'b1,1'b0, 2'b11,1'b1,1'b1,3'b000}; // SUB unsigned GEU
    `ifdef SIM
    default: decode_type_sb = 'bx;
    `endif
    endcase
   /* verilator lint_on CASEINCOMPLETE */
   /* verilator lint_on CASEOVERLAP */
  end
endfunction

function automatic [8:0] decode_type_i;
input [2:0] func;
input       func_secondary;
input       load;
  begin
    if(load)
    begin
      /* verilator lint_off CASEINCOMPLETE */
      casez(func) // synopsys full_case parallel_case
                                // {branch flag, return result or flag, flag return type, unsigned flavor, sub flavor, decoded op}
        3'b000: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // LB
        3'b001: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // LH
        3'b010: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // LW
        3'b100: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // LBU
        3'b101: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // LHU
        `ifdef SIM
        default: decode_type_i = 'bx;
        `endif
      endcase
      /* verilator lint_on CASEINCOMPLETE */
    end
    else
    begin
      casez(func) // synopsys full_case parallel_case
                                // {branch flag, return result or flag, flag return type, unsigned flavor, sub flavor, decoded op}
        3'b000: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // ADD
        3'b001: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b101}; // SLLI
        3'b010: decode_type_i = {1'b0,1'b0, 2'b10,1'b0,1'b1,3'b000}; // SLTI
        3'b011: decode_type_i = {1'b0,1'b0, 2'b10,1'b1,1'b1,3'b000}; // SLTIU
        3'b100: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b001}; // XORI
        3'b101:
        begin
          if(!func_secondary) decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b110}; // SRLI
          else decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b111};  // SRAI
        end
        3'b110: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b010}; // ORI
        3'b111: decode_type_i = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b011}; // ANDI
      endcase
    end
  end
endfunction

function automatic [8:0] decode_type_r;
input [2:0] func;
input       func_secondary;
  begin
    casez(func) // synopsys full_case parallel_case
      3'b000:
      begin
                                          // {branch flag, return result or flag, flag return type, unsigned flavor, sub flavor, decoded op}
        if(!func_secondary) decode_type_r = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b000}; // ADD
        else decode_type_r = {1'b0,1'b1, 2'b00,1'b0,1'b1,3'b000};  // SUB
      end
      3'b001: decode_type_r = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b101}; // SLL
      3'b010: decode_type_r = {1'b0,1'b0, 2'b10,1'b0,1'b1,3'b000}; // SLT
      3'b011: decode_type_r = {1'b0,1'b0, 2'b10,1'b1,1'b1,3'b000}; // SLTU
      3'b100: decode_type_r = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b001}; // XOR
      3'b101:
      begin
        if(!func_secondary) decode_type_r = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b110}; // SRL
        else decode_type_r = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b111};  // SRA
      end
      3'b110: decode_type_r = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b010}; // OR
      3'b111: decode_type_r = {1'b0,1'b1, 2'b00,1'b0,1'b0,3'b011}; // AND
    endcase
  end
endfunction
/* verilator lint_on VARHIDDEN */

// Logging

`ifdef SIM
`ifdef BRISCV_LOG

integer jump_trace_file;
integer branch_stats_file;
initial begin
`ifdef VCS
  #20ns
`endif
  jump_trace_file = $fopen($sformatf("jump_trace_%x.dat", i_reset_pc),"w");
  branch_stats_file = $fopen($sformatf("branch_stats_%x.dat", i_reset_pc),"w");
end

  //    $fwrite(File, "3 0x%x (0x%x) x%d 0x%x\n", disp_pc, disp_instrn, disp_reg_index, disp_reg_val);

  integer ex_log_cycle_counter;
  integer ex_log_stalled_cycle_counter;
  integer ex_log_branch_count;
  integer ex_log_mispredict_count;
  integer ex_log_taken_branch_count;
  integer ex_log_not_taken_branch_count;
  integer ex_log_jal_count;

//  final begin
//    $fwrite(branch_stats_file, "Branches: %d (taken %d, not %d), mispredict count: %d (%.2f%%)\n", 
//        ex_log_branch_count, ex_log_taken_branch_count, ex_log_not_taken_branch_count, 
//        ex_log_mispredict_count, 100.0 * ex_log_mispredict_count / ex_log_branch_count);
//  end

  initial begin
     ex_log_cycle_counter          = 0;
     ex_log_stalled_cycle_counter  = 0;
     ex_log_branch_count           = 0;
     ex_log_mispredict_count       = 0;
     ex_log_taken_branch_count     = 0;
     ex_log_not_taken_branch_count = 0;
     ex_log_jal_count = 0;
  end

  always @(posedge i_clk) begin
    if(!i_reset_n) begin
     ex_log_cycle_counter          <= 0;
     ex_log_stalled_cycle_counter  <= 0;
     ex_log_branch_count           <= 0;
     ex_log_mispredict_count       <= 0;
     ex_log_taken_branch_count     <= 0;
     ex_log_not_taken_branch_count <= 0;
     ex_log_jal_count              <= 0; 
    end
    else begin
     ex_log_cycle_counter          <= ex_log_cycle_counter + 1;
     if(!(i_mem_ex_rtr & o_ex_mem_vld)) ex_log_stalled_cycle_counter  <= ex_log_stalled_cycle_counter;
     if(ex_branch_target_rts | ex_jalr_target_rts) ex_log_branch_count <= ex_log_branch_count + 1;
     if(o_ex_bp_mispredict) ex_log_mispredict_count       <= ex_log_mispredict_count + 1;
     if(ex_jalr_target_rts | (ex_branch_target_rts & branch_taken)) ex_log_taken_branch_count     <= ex_log_taken_branch_count + 1;
     if(ex_branch_target_rts & branch_not_taken) ex_log_not_taken_branch_count <= ex_log_not_taken_branch_count + 1;    
     if(ex_jalr_target_rts) ex_log_jal_count <= ex_log_jal_count + 1;  

     if (ex_branch_target_rts | ex_jalr_target_rts) begin
        // Branch, record some stats
        $fwrite(jump_trace_file, "PC: 0x%x %s Target: 0x%x Predicted: %s 0x%x %s #b: %4d (%4d/%4d) #m: %4d (%.2f%%)\n",
            ex_pc, 
            (ex_jalr_target_rts | (ex_branch_target_rts & branch_taken)) ? "taken    " : "not_taken", ex_target_pc,
            ex_bp_predicted ?
              ex_bp_taken ? "taken        " : "not_taken    " : "not_predicted",
            ex_bp_predicted ? ex_bp_taken_addr : 32'h0,
            o_ex_bp_mispredict ? "  ** mispredict ** " : "                   ",
          
            ex_log_branch_count, ex_log_taken_branch_count, ex_log_not_taken_branch_count, 
            ex_log_mispredict_count, 100.0 * ex_log_mispredict_count / ex_log_branch_count
            );
      end

    end
  end

`endif
`endif

`ifdef SIM
`ASSERT_COND_CLK(o_ex_mem_payload.mem_amo, !((o_ex_mem_payload.mem_amotype == 5'b00010) || (o_ex_mem_payload.mem_amotype == 5'b00011)), "Illegal AMO instruction");
`endif
   
endmodule
