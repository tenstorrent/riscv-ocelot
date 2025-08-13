// See LICENSE.TT for license details.
`include "briscv_defines.h"
`include "tt_briscv_pkg.vh"

// Vector division-related instruction wrapper.
// Supports: vdiv, vdivu, vrem, vremu, vfdiv, vfrdiv, vfsqrt, vfsqrt7, vfrec7
// This module is currently a placeholder that accepts all needed inputs
// but does not perform computation yet.
//
// Instruction decoding:
// - Integer div/rem: funct7[6:0] = 100000(vdivu), 100001(vdiv), 100010(vremu), 100011(vrem)
// - FP operations: funct7[6:0] = 100000(vfdiv), 100011(vfsqrt), 100100(vfsqrt7), 100101(vfrec7)
// - vs1 field used for single-operand FP instructions (vfsqrt, vfrec7, vfsqrt7)
//
// Operation types based on funct3:
// - 3'b000 (OPIVV): Integer vector-vector (vdiv.vv, vdivu.vv, vrem.vv, vremu.vv)
// - 3'b001 (OPFVV): FP vector-vector (vfdiv.vv) or single-operand FP (vfsqrt, vfrec7, vfsqrt7)
// - 3'b100 (OPIVX): Integer vector-scalar (vdiv.vx, vdivu.vx, vrem.vx, vremu.vx)
// - 3'b101 (OPFVF): FP vector-scalar (vfdiv.vf)
//
// SEW support:
// - Integer operations: SEW = 8, 16, 32, 64 bits
// - FP operations: SEW = 16, 32 bits (FP16, FP32)
//
// Current behavior (placeholder):
// - Always reports not busy
// - Never asserts result valid
// - Outputs zero data and zero exceptions
// - Passes through the provided LQ ID
module tt_vec_div_unit
#(
   parameter NUM_LANE = 2,     // lanes = VLEN/64 from parent
   parameter VLEN     = 128
)
(
   input                              i_clk,
   input                              i_reset_n,

   // Handshake and control from ID stage
   input                              i_id_vdiv_ex0_rts,
   
   // Operation type decode (from vec_autogen_s)
   input  logic                           i_idivop,    // Integer division operation flag
   input  logic                           i_fdivop,    // Floating-point division operation flag
   input  logic [tt_briscv_pkg::LQ_DEPTH_LOG2-1:0] i_ldqid,     // Load queue ID for writeback

   // Source operands - full VLEN width for flexible SEW handling
   input  logic          [VLEN-1:0]       i_src1,      // vs2 source (dividend)
   input  logic          [VLEN-1:0]       i_src2,      // vs1/scalar/imm source (divisor)
   input  logic          [VLEN-1:0]       i_src3,      // vd source (for masked ops)
   
   // Vector control signals
   input  logic          [VLEN/8-1:0]     i_vm0,       // mask register
   input  logic [1:0]                     i_sew,       // element width: 00=8b, 01=16b, 10=32b, 11=64b
   input  logic [2:0]                     i_lmul,      // length multiplier
   input  logic [7:0]                     i_vl,        // vector length
   
   // Instruction decode signals
   input  logic [6:0]                     i_funct7,    // instruction-specific function
   input  logic [2:0]                     i_funct3,    // operation category (000=OPIVV, 001=OPFVV, 100=OPIVX, 101=OPFVF)
   input  logic [4:0]                     i_vs1,       // vs1 field (used for single-operand FP instructions)
   input  logic                           i_vm,        // vector mask enable
   
   // Floating-point control (IEEE FP only, no fixed-point)
   input  logic [2:0]                     i_frm,       // FP rounding mode
   
   // Replay control
   input  logic [2:0]                     i_lmul_cnt,   // LMUL counter for replay

   // Outputs towards MEM/LQ (division write port)
   output logic                           o_result_valid,
   output logic          [VLEN-1:0]       o_result,
   output tt_briscv_pkg::csr_fp_exc       o_result_exc,
   output logic [tt_briscv_pkg::LQ_DEPTH_LOG2-1:0] o_result_lqid,

   // Global busy indicator for ID resource hazard checks
   output logic                           o_busy
);

   // Internal operation decode
   logic is_integer_op, is_fp_op, is_single_operand;
   logic is_div_op, is_rem_op, is_sqrt_op, is_rec_op;
   logic is_signed_op, is_vector_scalar;
   
   // Decode operation type from high-level flags and instruction fields
   always_comb begin
      // Default values
      is_integer_op = 1'b0;
      is_fp_op = 1'b0;
      is_single_operand = 1'b0;
      is_div_op = 1'b0;
      is_rem_op = 1'b0;
      is_sqrt_op = 1'b0;
      is_rec_op = 1'b0;
      is_signed_op = 1'b0;
      is_vector_scalar = 1'b0;
      
      // Use high-level operation flags for main categorization
      is_integer_op = i_idivop;
      is_fp_op = i_fdivop;
      is_vector_scalar = (i_funct3 == 3'b100) || (i_funct3 == 3'b101); // OPIVX or OPFVF
      
      // Detailed operation decode based on funct7
      if (i_idivop) begin
         // Integer division/remainder operations
         case (i_funct7)
            7'b1000000: begin // vdivu
               is_div_op = 1'b1;
               is_signed_op = 1'b0;
            end
            7'b1000001: begin // vdiv
               is_div_op = 1'b1;
               is_signed_op = 1'b1;
            end
            7'b1000010: begin // vremu
               is_rem_op = 1'b1;
               is_signed_op = 1'b0;
            end
            7'b1000011: begin // vrem
               is_rem_op = 1'b1;
               is_signed_op = 1'b1;
            end
         endcase
      end else if (i_fdivop) begin
         // Floating-point operations
         case (i_funct7)
            7'b1000000: begin // vfdiv
               is_div_op = 1'b1;
            end
            7'b1000011: begin // vfsqrt
               is_sqrt_op = 1'b1;
               is_single_operand = 1'b1;
            end
            7'b1000100: begin // vfsqrt7
               is_sqrt_op = 1'b1;
               is_single_operand = 1'b1;
            end
            7'b1000101: begin // vfrec7
               is_rec_op = 1'b1;
               is_single_operand = 1'b1;
            end
         endcase
      end
   end

   // Square root unit signals and instances
   logic [VLEN/16-1:0][15:0] fp16_sqrt_in, fp16_sqrt_out;
   logic [VLEN/16-1:0][4:0]  fp16_sqrt_exc;
   logic [VLEN/32-1:0][31:0] fp32_sqrt_in, fp32_sqrt_out;
   logic [VLEN/32-1:0][4:0]  fp32_sqrt_exc;
   
   // Input data preparation for square root units
   generate
      for (genvar i = 0; i < VLEN/16; i++) begin : gen_fp16_input
         always_comb begin
            if (is_sqrt_op && i_sew == 2'b01) begin // SEW=16
               fp16_sqrt_in[i] = i_src1[(i+1)*16-1:i*16];
            end else begin
               fp16_sqrt_in[i] = '0;
            end
         end
      end
      
      for (genvar i = 0; i < VLEN/32; i++) begin : gen_fp32_input
         always_comb begin
            if (is_sqrt_op && i_sew == 2'b10) begin // SEW=32
               fp32_sqrt_in[i] = i_src1[(i+1)*32-1:i*32];
            end else begin
               fp32_sqrt_in[i] = '0;
            end
         end
      end
   endgenerate
   
   // Generate FP16 square root units (for SEW=16)
   generate
      for (genvar i = 0; i < VLEN/16; i++) begin : gen_fp16_sqrt
         VecFP16rsqrt7 fp16_sqrt_inst (
            .io_in            (fp16_sqrt_in[i]),
            .io_roundingMode  (i_frm),
            .io_out           (fp16_sqrt_out[i]),
            .io_exceptionFlags(fp16_sqrt_exc[i])
         );
      end
   endgenerate
   
   // Generate FP32 square root units (for SEW=32)
   generate
      for (genvar i = 0; i < VLEN/32; i++) begin : gen_fp32_sqrt
         VecFP32rsqrt7 fp32_sqrt_inst (
            .io_in            (fp32_sqrt_in[i]),
            .io_roundingMode  (i_frm),
            .io_out           (fp32_sqrt_out[i]),
            .io_exceptionFlags(fp32_sqrt_exc[i])
         );
      end
   endgenerate
   
   // Output result multiplexing - simple mux between FP16 and FP32 results
   logic [VLEN-1:0] sqrt_result;
   tt_briscv_pkg::csr_fp_exc sqrt_exc;
   
   always_comb begin
      sqrt_result = '0;
      sqrt_exc = '0;
      
      if (is_sqrt_op && i_sew == 2'b01) begin // SEW=16
         // Pack FP16 results into VLEN
         for (int i = 0; i < VLEN/16; i++) begin
            sqrt_result[i*16 +: 16] = fp16_sqrt_out[i];
         end
         // Combine FP16 exceptions (OR all exception flags)
         for (int i = 0; i < VLEN/16; i++) begin
            sqrt_exc |= {fp16_sqrt_exc[i][4], 1'b0, fp16_sqrt_exc[i][3:0]};
         end
      end else if (is_sqrt_op && i_sew == 2'b10) begin // SEW=32
         // Pack FP32 results into VLEN  
         for (int i = 0; i < VLEN/32; i++) begin
            sqrt_result[i*32 +: 32] = fp32_sqrt_out[i];
         end
         // Combine FP32 exceptions (OR all exception flags)
         for (int i = 0; i < VLEN/32; i++) begin
            sqrt_exc |= {fp32_sqrt_exc[i][4], 1'b0, fp32_sqrt_exc[i][3:0]};
         end
      end
   end


   logic [VLEN-1:0] compute_result, merged_result;
   assign compute_result = is_sqrt_op ? sqrt_result : '0; // tying this to sqrt for now
   
   // Mask extraction and merging logic
   logic [VLEN/8-1:0] active_mask;  // Mask for current register slice
   
   // Extract appropriate mask slice based on SEW and LMUL_CNT
   always_comb begin
      active_mask = '0;
      
      case (i_sew)
         2'b00: begin // SEW=8, 1 mask bit per element
            // Use all lmul_cnt bits for indexing
            active_mask = i_vm0[i_lmul_cnt[2:0] * (VLEN/8) +: VLEN/8];
         end
         2'b01: begin // SEW=16, 1 mask bit per 2 bytes  
            // Use upper lmul_cnt bits, replicate mask bits
            active_mask = {(VLEN/16){i_vm0[i_lmul_cnt[2:1] * (VLEN/16) +: VLEN/16]}};
         end
         2'b10: begin // SEW=32, 1 mask bit per 4 bytes
            // Use top lmul_cnt bit, replicate mask bits  
            active_mask = {(VLEN/32){i_vm0[i_lmul_cnt[2] * (VLEN/32) +: VLEN/32]}};
         end
         2'b11: begin // SEW=64, 1 mask bit per 8 bytes
            // Always use first slice, replicate mask bits
            active_mask = {(VLEN/64){i_vm0[0 +: VLEN/64]}};
         end
      endcase
   end
   
   // Merge compute_result with i_src3 based on active_mask
   // mask=1: use compute_result, mask=0: use i_src3 (destination merge)
   always_comb begin
      merged_result = '0;
      for (int i = 0; i < VLEN/8; i++) begin
         if (active_mask[i]) begin
            merged_result[i*8 +: 8] = compute_result[i*8 +: 8];  // Use computed result
         end else begin
            merged_result[i*8 +: 8] = i_src3[i*8 +: 8];         // Use destination (merge)
         end
      end
   end

   // Output assignments
   // For now, only square root is implemented
   assign o_busy         = 1'b0; // Combinational operation
   assign o_result_valid = i_id_vdiv_ex0_rts & is_sqrt_op;
   assign o_result       = merged_result;
   assign o_result_exc   = is_sqrt_op ? sqrt_exc : '0;
   assign o_result_lqid  = i_ldqid;

endmodule