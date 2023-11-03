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

   input tt_briscv_pkg::csr_t       i_csr,
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
   
// V-LD/ST misc signals
logic [VLEN/8-1:0] ex_vecldst_iter_mask;
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


logic ex_vld, ex_vld_q;
logic ex_type_vecldst, ex_type_vecldst_q;
logic ex_vecldst_elem_mask, ex_vecldst_elem_mask_q;
logic ex_vecldst_start, ex_vecldst_start_q;
logic ex_vecldst_done,  ex_vecldst_done_q;
logic ex_vecldst_iter_done, ex_vecldst_iter_done_q;

wire mem_pipe_rtr_raw = (i_mem_ex_rtr);
wire mem_pipe_rtr     = (i_mem_ex_rtr) | ex_vecldst_iter_done_q;

assign o_ex_id_rtr = 1'b1;
assign o_ex_last = 1'b1;

assign ex_type_vecldst = 1'b1;

// Create a single cycle EX branch vld based on new instr popped out of the instrn_fifo
//always_ff @(posedge i_clk) begin
//end

// Don't assert valid for maksed vector ldst element unless it's the first or last element (which is needed since it needs to go to lq or track the last index)
assign o_ex_mem_vld = i_id_ex_rts;
   
// Allocate on last index for vector stores otherwise vector store will commit before dispatch of all indexes
// Allocate on first index for vector loads since vector load coalescing happens in load queue and index might return before last index is dispatched  
assign o_ex_mem_lqvld_1c = i_id_ex_rts && ex_mem_payload.mem_load;
assign o_ex_mem_lqid_1c  = i_id_ex_lqid;
assign o_ex_mem_lqdata_1c = '0;
   
assign o_ex_mem_lqvld_2c = i_id_ex_rts && ex_mem_payload.mem_store;
assign o_ex_mem_lqid_2c  = i_id_ex_lqid;
assign o_ex_mem_lqdata_2c = '0;
   
assign ex_vld = i_id_ex_rts;
assign ex_instrn = i_id_ex_instrn;

assign ex_immed_op = i_id_immed_op;

wire        ex_bp_branch;
wire        if_ex_bp_valid;
wire        if_ex_bp_taken = i_if_ex_deco[1];
wire [31:0] if_ex_bp_taken_addr = i_if_ex_deco[36:5];

// As EX is processing a branch, we need the "branch taken" and "target addr" from the *next* request in here.. that's why we're not flopping these input signals,
// they are not in sync with the branch instruction that is being evaluated.

assign ex_pc = i_id_ex_pc;

logic [1:0] id_vecldst_sew, id_vecldst_data_eew, id_vecldst_idx_eew;
logic [2:0] id_vecldst_lmul, id_vecldst_data_emul, id_vecldst_idx_emul;

// V-LD/ST Logic
assign id_vecldst_rf_wraddr = i_id_ex_vecldst_autogen.dest_reg;
   
// Flop signals (These are constant through ex iterations)
assign id_vecldst_sew       = i_csr.v_vsew[1:0];  
assign id_vecldst_lmul      = i_csr.v_lmul;

assign id_vecldst_data_eew  = i_id_ex_vecldst_autogen.ldst_index ? i_csr.v_vsew[1:0] : i_id_ex_instrn[13:12];
assign id_vecldst_data_emul = {3{~i_id_ex_vecldst_autogen.ldst_whole_register}} & tt_briscv_pkg::get_vecldst_emul(id_vecldst_sew, id_vecldst_lmul, id_vecldst_data_eew);

assign id_vecldst_idx_eew   = i_id_ex_instrn[13:12];
assign id_vecldst_idx_emul  = i_id_ex_vecldst_autogen.ldst_index ? tt_briscv_pkg::get_vecldst_emul(id_vecldst_sew, id_vecldst_lmul, id_vecldst_idx_eew) : id_vecldst_data_emul;
   
assign ex_vecldst_autogen = i_id_ex_vecldst_autogen;
   
assign {ex_vmask_reg, ex_vs2_reg, ex_vs3_reg} = {i_vmask_rddata, i_vs2_rddata, i_vs3_rddata};

assign {ex_vecldst_data_eew, ex_vecldst_data_emul, ex_vecldst_idx_eew, ex_vecldst_idx_emul} = {id_vecldst_data_eew, id_vecldst_data_emul, id_vecldst_idx_eew, id_vecldst_idx_emul};

assign {ex_vld_q, ex_type_vecldst_q, ex_vecldst_start_q, ex_vecldst_done_q, ex_vecldst_iter_done_q, ex_vecldst_elem_mask_q} = {ex_vld,   ex_type_vecldst,   ex_vecldst_start,   ex_vecldst_done,   ex_vecldst_iter_done,   ex_vecldst_elem_mask};

assign ex_mem_payload_q = ex_mem_payload_fn;

logic [2:0] ex_vecldst_id_lmul_iter, ex_vecldst_id_seg_iter;
logic [3:0] ex_vecldst_num_lmul_iter;
logic [5:0] ex_vecldst_id_iter;
logic [VLEN/8-1:0] ex_vecldst_id_iter_mask;

assign ex_vecldst_start =     '0;
assign ex_vecldst_iter_done = '0;
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
   
assign ex_vecldst_vl        = i_csr.v_vl;
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

assign o_ex_branch_taken = 1'b0;
assign o_ex_is_some_branch = 1'b0; 
assign o_ex_bp_mispredict        = '0;
assign o_ex_bp_mispredict_not_br = '0;
assign o_ex_bp_pc                = '0;
assign o_ex_bp_fifo_pop   = 1'b0;
//                          || (holding_predicted & mem_pipe_rtr); // Ashok: Id never sends unless ex is ready to receive. This holding logic seems stale. Commenting out

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
assign ex_mem_payload.mem_sz    = ex_type_vecldst ? {1'b0,ex_vecldst_data_eew[1:0]} : ex_instrn[14:12];
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
assign ex_mem_payload.mem_amo     = 1'b0;
assign ex_mem_payload.mem_amotype = ex_instrn[31:27];
   
assign ex_mem_payload.vecldst_idx = (ex_vecldst_elem_count + ex_vs3_base[$clog2(VLEN/8)+2:3]);
assign ex_mem_payload.vecldst_idx_last = '0;   

// This is byte mask   
for (genvar i=0; i<VLEN/8; i++) begin
  assign ex_mem_payload.vecldst_byte_mask[i] = ex_type_vecldst & (ex_vecldst_data_eew[1:0] == 2'h3 ? ex_vecldst_iter_mask[i/8] :
                                                                  ex_vecldst_data_eew[1:0] == 2'h2 ? ex_vecldst_iter_mask[i/4] :
                                                                  ex_vecldst_data_eew[1:0] == 2'h1 ? ex_vecldst_iter_mask[i/2] :
                                                                                                     ex_vecldst_iter_mask[i  ]  );
end
    
assign ex_mem_payload.mem_store_data = ex_store_val_fn;
assign ex_mem_payload.mem_alu_result[31:0] =  '0;

assign ex_mem_payload.mem_tx_valid = 1'b0;
assign ex_mem_payload.mem_lq_valid = 1'b0;
assign ex_mem_payload.mem_lqid     = '0;

				     
// Send the 1c or 2c payload to mem (1c and 2c can't be active at the same)
assign o_ex_mem_payload = (ex_vld_q & ex_type_vecldst_q) ? ex_mem_payload_q : ex_mem_payload;
   
// Forwarding controls to ID
//wire mem_instrn_1c = ex_mem_payload.mem_store | ex_mem_payload.mem_load | ex_mem_payload.mem_amo;
assign o_ex_dst_vld_1c     = '0;
assign o_ex_dst_lqid_1c    = '0;
assign o_ex_fwd_data_1c    = '0;
  
assign o_ex_dst_vld_2c     = '0;
assign o_ex_dst_lqid_2c    = '0;
assign o_ex_fwd_data_2c    = '0;
    
assign ex_mem_payload.mem_ordered = 1'b0;

endmodule
