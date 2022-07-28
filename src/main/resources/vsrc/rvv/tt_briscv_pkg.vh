// See LICENSE.TT for license details.

`ifndef TT_BRISCV_PKG_VH
 `define  TT_BRISCV_PKG_VH
package tt_briscv_pkg;
   typedef struct packed {
      logic 	   vrf_wr_flag;
      logic 	   fp_rf_wr_flag;
      logic        rf_wr_flag;
      logic [31:0] pc;
      logic [31:0] sim_instrn;
      logic 	   is_branch;
      logic 	   load;
      logic 	   vec_load;
      logic [4:0]  rf_wraddr;
   } lq_info_s;

localparam ACTUAL_LQ_DEPTH = 8;

localparam VLEN=256;
localparam ADDRWIDTH=40;

`ifdef SIM
   `ifdef MUTATE_RISCV_CORE_FIFO_MIN
      localparam LQ_DEPTH = 2;
   `elsif MUTATE_RISCV_CORE_FIFO_MAX
      localparam LQ_DEPTH = (2 * ACTUAL_LQ_DEPTH);
   `else
      localparam LQ_DEPTH = ACTUAL_LQ_DEPTH;
   `endif
`else
   localparam LQ_DEPTH = ACTUAL_LQ_DEPTH;
`endif
   
   localparam LQ_DEPTH_LOG2 = $clog2(LQ_DEPTH);
   typedef lq_info_s [LQ_DEPTH-1:0] arr_lq_info_s ;

   typedef struct packed {
      logic [LQ_DEPTH_LOG2-1:0]  mem_lqid;
      logic [2:0] 		 mem_sz;
      logic [31:0]               mem_alu_result;
      logic [VLEN-1:0]           mem_store_data;
      logic                      mem_tx_valid;
      logic                      mem_lq_valid;
      logic 	                 mem_load;
      logic 	                 mem_store;
      logic 	                 mem_amo;
      logic [4:0]                mem_amotype;
      logic 	                 mem_fence;
      logic 	                 mem_ordered;
      logic [ADDRWIDTH-1:0]      mem_addr;
      logic [7:0]                mem_byten;
      logic 	                 vecldst_vld;
      logic                      vecldst_128;
      logic [$clog2(VLEN/8)-1:0] vecldst_idx;
      logic                      vecldst_idx_last;
      logic [VLEN/8-1:0]         vecldst_byte_mask;
   } mem_skidbuf_s;

   typedef struct packed {
      logic [31:0] vgsrc;
      logic [2:0] v_vsew;
      logic [2:0] v_lmul;
      logic [7:0] v_vlmax;
      logic [$clog2(VLEN+1)-1:0] v_vl;
   } csr_to_id;

   typedef struct packed {
      logic [2:0] v_vsew;
      logic [2:0] v_lmul;
      logic [1:0] v_vxrm;
      logic [$clog2(VLEN+1)-1:0] v_vl;
   } csr_to_vec;

   typedef struct packed {
      logic [LQ_DEPTH_LOG2-1:0] ldqid;
      logic       onep1;  //fixme: these needs drivers
      logic       zerop2;
      logic       out_from_vec_int;
      logic 	  vfp_rf_rd_op_valid;
      logic 	  vfp_mad_type_inst;
      logic 	  rf_wren;
      logic 	  wdeop;		// implies destination is 2*sew.
      logic 	  nrwop;
      logic [4:0] rf_addrp0;
      logic [4:0] rf_addrp1;
      logic [4:0] rf_addrp2;
      //logic [4:0] wraddr;               //When p2/p1 is flipped for Vmadd, the write destination doesn't change , just that the data is flipped. 
      logic 	  src1hw;		// src1 = sew , src2 = 2*sew and dst = 2*sew
      logic 	  rf_store_rd_en;
    //  logic [3:0] math_fmt;
    //  logic 	  fp_op_int_rs1;
      logic 	  rf_rd_p2_is_rs2;
      logic 	  rf_rden0;
      logic 	  rf_rden1;
      logic 	  rf_rden2;
      logic 	  acc_val ;
      logic 	  rd_onep1;
      logic 	  rd_neg0 ;
      logic 	  rd_neg1 ;
      logic 	  rd_neg2 ;
      logic 	  cmpmul;
      logic 	  rnden;
      logic 	  saturate;
      logic 	  mulh;
      logic 	  usemask;
      logic 	  wrmask;
      logic 	  inversesub;
      logic 	  issgn_src2;
      logic 	  issgn_src1;
      logic 	  cryorbrw;
      logic 	  addorsub;
      logic 	  imulop;
      logic 	  iaddop;
      logic       avg;
      logic       permuteop;

      logic       sel_scalar;
      logic       fp_sel_scalar;
      logic       sel_imm;
      logic 	  shftop;
      logic       bitwise;
      logic 	  sat_instrn;
      logic       scalar_dest;
      logic [6:0] funct7;
      logic       iterate;
      logic 	  vmvgrp;
      logic       reductop;
      logic       onecycle_iterate;
      logic       onecycle_nrwop;   // Narrowing op that doesn't need additional operand
      logic [$clog2(VLEN):0] replay_cnt;
      logic [7:0] addrp2_incr;
      logic [7:0] addrp1_incr;
      logic [7:0] addrp0_incr;
      logic [7:0] addrp1_reset; // For Iterating through segment-indexed LdSt ops 
      logic 	  mask_only;

   } vec_autogen_s;

   typedef struct packed {
      logic 	  load;
      logic 	  store;
      logic [4:0] dest_reg;	// IMPROVE: This is a duplicate of vec_autogen_s rf_wraddr.  Clean up
      logic 	  ldst_ustride;
      logic 	  ldst_strided;
      logic 	  ldst_index;
      logic       ldst_mask;
      logic       ldst_whole_register;
      logic [5:0] ldst_iter_cnt;
      logic [5:0] ldst_iterations;
      logic [7:0] ldst_dest_incr;	// IMPROVE: This is a duplicate of vec_autogen_s addrp2_incr. Clean up
      logic [7:0] ldst_index_incr;	// IMPROVE: This is a duplicate of vec_autogen_s addrp1_incr.  Clean up
   } vecldst_autogen_s;

     typedef struct packed {
      logic   fpNV; // 4  - Invalid Operation
      logic   fpDZ; // 3  - Divide by Zero
      logic   fpOF; // 2  - Overflow
      logic   fpUF; // 1  - Underflow
      logic   fpNX; // 0  - Inexact
   } csr_fp_exc;

   typedef enum logic [4:0] {
      BIT_NONE   = 0,
      BIT_CLZ    = 1,
      BIT_CTZ    = 2,
      BIT_CPOP   = 3,
      BIT_MIN    = 4,
      BIT_MINU   = 5,
      BIT_MAX    = 6,
      BIT_MAXU   = 7,
      BIT_SEXT_B = 8,
      BIT_SEXT_H = 9,
      BIT_PACK   = 10,
      BIT_ANDN   = 11,
      BIT_ORN    = 12,
      BIT_XNOR   = 13,
      BIT_ROL    = 14,
      BIT_ROR    = 15,
      BIT_RORI   = 16,
      BIT_GREVI  = 17,
      BIT_GORCI  = 18,
      BIT_SH1ADD = 19,
      BIT_SH2ADD = 20,
      BIT_SH3ADD = 21,
      BIT_RSVD22 = 22,
      BIT_RSVD23 = 23,
      BIT_RSVD24 = 24,
      BIT_RSVD25 = 25,
      BIT_RSVD26 = 26,
      BIT_RSVD27 = 27,
      BIT_RSVD28 = 28,
      BIT_RSVD29 = 29,
      BIT_RSVD30 = 30,
      BIT_RSVD31 = 31
   } bitmanip_instrn_e;

   typedef enum logic [2:0] {
      RRO_BYP    = 0,
      RRO_ROR    = 1,
      RRO_REV    = 2,
      RRO_FF1    = 3,
      RRO_ORC    = 4,
      RRO_SHF    = 5,
      RRO_RSVD6  = 6,
      RRO_RSVD7  = 7
   } bitmanip_rro_cmd_e;

   function automatic logic [31:0] f16_to_f32_unboxing(input logic [31:0] boxed_f16);
      // Check if it's properly boxed
      if (boxed_f16[31:16] == 16'hffff) begin
         f16_to_f32_unboxing[31   ] =                          boxed_f16[15   ];         // Sign
         f16_to_f32_unboxing[30:23] = {{3{&boxed_f16[14:10]}}, boxed_f16[14:10]};        // Exponent
         f16_to_f32_unboxing[22: 0] = {                        boxed_f16[ 9: 0], 13'h0}; // Mantissa
      end else begin
      // Override to NaN otherwise
         f16_to_f32_unboxing = 32'h7fc0_0000;
      end
   endfunction
      
   function automatic void float_exam_special_conditions(input  logic [31:0] data_in,
                                                          input  logic        is_fp16,
                                                          output logic        is_nan,
                                                          output logic        is_zero,
                                                          output logic        is_inf  );
      if (is_fp16) begin
         is_nan  = data_in[27:23] == '1 &&
                   data_in[22:13] != '0;
         is_zero = data_in[27:23] == '0; // Force denorm to zero
         is_inf  = data_in[27:23] == '1 &&
                   data_in[22:13] == '0;
      end else begin
         is_nan  = data_in[30:23] == '1 &&
                   data_in[22: 0] != '0;
         is_zero = data_in[30:23] == '0; // Force denorm to zero
         is_inf  = data_in[30:23] == '1 &&
                   data_in[22: 0] == '0;
      end

   endfunction
 
// Floating-point minimum-number and maximum-number instructions FMIN.S and FMAX.S write,
// respectively, the smaller or larger of rs1 and rs2 to rd. For the purposes of these instructions only,
// the value −0.0 is considered to be less than the value +0.0.
// If both inputs are quiet NaNs, the result is canonical NaN
// If any input is a signaling NaN, the result is canonical NaN
// If only one operand is a quiet NaN, the result is the non-NaN operand.
function automatic logic [31:0] rv_fmin;
    input i_a_nan;
    input i_b_nan;
    input i_a_sign;
    input i_b_sign;
    input b_gt_a;
    input [31:0] reg_a;
    input [31:0] reg_b;
    begin
        // If both inputs are NaNs, the result is canonical NaN
        if (i_a_nan &&
            i_b_nan   ) begin
            rv_fmin  = 32'h7fc0_0000;
        end else
        // If only one operand is a NaN, the result is the non-NaN operand.
        if (i_a_nan) begin
            rv_fmin  = reg_b;
        end else 
        if (i_b_nan) begin
            rv_fmin  = reg_a;
        end else if (i_a_sign & i_b_sign ) begin
            //both negative, pick larger
            rv_fmin  = b_gt_a ? reg_b : reg_a;
        end else if (i_a_sign ^ i_b_sign    ) begin
            // pick negative
            rv_fmin  = i_a_sign? reg_a : reg_b;
        end else begin
            // both positive, pick smaller
            rv_fmin  = b_gt_a ? reg_a : reg_b;
        end
    end

endfunction





function automatic logic [31:0] rv_fmax;
    input i_a_nan;
    input i_b_nan;
    input i_a_sign;
    input i_b_sign;
    input b_gt_a;
    input [31:0] reg_a;
    input [31:0] reg_b;
    begin
        // If both inputs are NaNs, the result is canonical NaN
        if (i_a_nan &&
            i_b_nan   ) begin
            rv_fmax  = 32'h7fc0_0000;
        end else
        // If only one operand is a NaN, the result is the non-NaN operand.
        if (i_a_nan) begin
            rv_fmax  = reg_b;
        end else 
        if (i_b_nan) begin
            rv_fmax  = reg_a;
        end else if (i_a_sign & i_b_sign ) begin
            //both negative, pick smaller
            rv_fmax  = b_gt_a ? reg_a : reg_b;
        end else if (i_a_sign ^ i_b_sign    ) begin
            // pick positive
            rv_fmax  = i_a_sign? reg_b : reg_a;
        end else begin
            // both positive, pick larger
            rv_fmax  = b_gt_a ? reg_b : reg_a;
        end
    end

endfunction     
endpackage
   
`endif
