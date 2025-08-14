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
   input  logic          [VLEN-1:0]       i_vm0,       // mask register
   input  logic [1:0]                     i_sew,       // element width: 00=8b, 01=16b, 10=32b, 11=64b
   input  logic [2:0]                     i_lmul,      // length multiplier
   input  logic [7:0]                     i_vl,        // vector length
   input  logic                           i_vta,       // vector tail agnostic
   input  logic                           i_vma,       // vector mask agnostic
   
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
   logic is_div_op, is_rem_op, is_sqrt_op, is_sqrt7_op, is_rec_op;
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
      is_sqrt7_op = 1'b0;
      is_rec_op = 1'b0;
      is_signed_op = 1'b0;
      is_vector_scalar = 1'b0;
      
      // Use high-level operation flags for main categorization
      is_integer_op = i_idivop;
      is_fp_op = i_fdivop;
      is_vector_scalar = (i_funct3 == 3'b100) || (i_funct3 == 3'b101); // OPIVX or OPFVF
      
      // Detailed operation decode based on funct7[6:1] (ignore vm bit)
      if (i_idivop) begin
         // Integer division/remainder operations
         case (i_funct7[6:1])
            6'b100000: begin // vdivu
               is_div_op = 1'b1;
               is_signed_op = 1'b0;
            end
            6'b100001: begin // vdiv
               is_div_op = 1'b1;
               is_signed_op = 1'b1;
            end
            6'b100010: begin // vremu
               is_rem_op = 1'b1;
               is_signed_op = 1'b0;
            end
            6'b100011: begin // vrem
               is_rem_op = 1'b1;
               is_signed_op = 1'b1;
            end
         endcase
      end else if (i_fdivop) begin
         // Floating-point operations
         case (i_funct7[6:1])
            6'b100000: begin // vfdiv
               is_div_op = 1'b1;
            end
            6'b010011: begin // vfsqrt, vfrsqrt7, vfrec7 (all share same funct7[6:1])
               if (i_vs1 == 5'b00000) begin // vs1=00000 for vfsqrt
                  is_sqrt_op = 1'b1;
                  is_single_operand = 1'b1;
               end else if (i_vs1 == 5'b00100) begin // vs1=00100 for vfrsqrt7
                  is_sqrt7_op = 1'b1;
                  is_single_operand = 1'b1;
               end else if (i_vs1 == 5'b00101) begin // vs1=00101 for vfrec7
                  is_rec_op = 1'b1;
                  is_single_operand = 1'b1;
               end
            end
         endcase
      end
   end

   // Generalized SEW-based input data slicing
   logic [VLEN/8-1:0][7:0]   src1_sew8;   // SEW=8  (VLEN/8 elements)
   logic [VLEN/16-1:0][15:0] src1_sew16;  // SEW=16 (VLEN/16 elements)
   logic [VLEN/32-1:0][31:0] src1_sew32;  // SEW=32 (VLEN/32 elements)
   logic [VLEN/64-1:0][63:0] src1_sew64;  // SEW=64 (VLEN/64 elements)
   
   logic [VLEN/8-1:0][7:0]   src2_sew8;   // SEW=8  (VLEN/8 elements)
   logic [VLEN/16-1:0][15:0] src2_sew16;  // SEW=16 (VLEN/16 elements)
   logic [VLEN/32-1:0][31:0] src2_sew32;  // SEW=32 (VLEN/32 elements)
   logic [VLEN/64-1:0][63:0] src2_sew64;  // SEW=64 (VLEN/64 elements)
   
   // Generate input data slicing for all SEW values
   generate
      for (genvar i = 0; i < VLEN/8; i++) begin : gen_sew8
         assign src1_sew8[i] = i_src1[i*8 +: 8];
         assign src2_sew8[i] = i_src2[i*8 +: 8];
      end
      for (genvar i = 0; i < VLEN/16; i++) begin : gen_sew16
         assign src1_sew16[i] = i_src1[i*16 +: 16];
         assign src2_sew16[i] = i_src2[i*16 +: 16];
      end
      for (genvar i = 0; i < VLEN/32; i++) begin : gen_sew32
         assign src1_sew32[i] = i_src1[i*32 +: 32];
         assign src2_sew32[i] = i_src2[i*32 +: 32];
      end
      for (genvar i = 0; i < VLEN/64; i++) begin : gen_sew64
         assign src1_sew64[i] = i_src1[i*64 +: 64];
         assign src2_sew64[i] = i_src2[i*64 +: 64];
      end
   endgenerate

   // Square root unit signals and instances
   logic [VLEN/16-1:0][15:0]  fp16_sqrt7_out;
   logic [VLEN/16-1:0][4:0]   fp16_sqrt7_exc;
   logic [VLEN/32-1:0][31:0] fp32_sqrt7_out;
   logic [VLEN/32-1:0][4:0]  fp32_sqrt7_exc;
   
   // Generate FP16 square root units (for SEW=16) - always connected
   generate
      for (genvar i = 0; i < VLEN/16; i++) begin : gen_fp16_sqrt
         VecFP16rsqrt7 fp16_sqrt_inst (
            .io_in            (src2_sew16[i]),
            .io_roundingMode  (i_frm),
            .io_out           (fp16_sqrt7_out[i]),
            .io_exceptionFlags(fp16_sqrt7_exc[i])
         );
      end
   endgenerate
   
   // Generate FP32 square root units (for SEW=32) - always connected
   generate
      for (genvar i = 0; i < VLEN/32; i++) begin : gen_fp32_sqrt
         VecFP32rsqrt7 fp32_sqrt_inst (
            .io_in            (src2_sew32[i]),
            .io_roundingMode  (i_frm),
            .io_out           (fp32_sqrt7_out[i]),
            .io_exceptionFlags(fp32_sqrt7_exc[i])
         );
      end
   endgenerate
   
   // Reciprocal unit signals and instances (vfrec7)
   logic [VLEN/16-1:0][15:0]  fp16_rec7_out;
   logic [VLEN/16-1:0][4:0]   fp16_rec7_exc;
   logic [VLEN/32-1:0][31:0] fp32_rec7_out;
   logic [VLEN/32-1:0][4:0]  fp32_rec7_exc;
   
   // Generate FP16 reciprocal units (for SEW=16) - always connected
   generate
      for (genvar i = 0; i < VLEN/16; i++) begin : gen_fp16_rec
         VecFP16rec7 fp16_rec_inst (
            .io_in            (src2_sew16[i]),
            .io_roundingMode  (i_frm),
            .io_out           (fp16_rec7_out[i]),
            .io_exceptionFlags(fp16_rec7_exc[i])
         );
      end
   endgenerate
   
   // Generate FP32 reciprocal units (for SEW=32) - always connected
   generate
      for (genvar i = 0; i < VLEN/32; i++) begin : gen_fp32_rec
         VecFP32rec7 fp32_rec_inst (
            .io_in            (src2_sew32[i]),
            .io_roundingMode  (i_frm),
            .io_out           (fp32_rec7_out[i]),
            .io_exceptionFlags(fp32_rec7_exc[i])
         );
      end
   endgenerate
   
   // Separate output result multiplexing for sqrt, sqrt7, and rec7 operations
   logic [VLEN-1:0] sqrt_result, sqrt7_result, rec7_result;
   tt_briscv_pkg::csr_fp_exc sqrt_exc, sqrt7_exc, rec7_exc;
   
   // Full precision square root results (vfsqrt)

   // 7-bit reciprocal square root results (vfrsqrt7)
   always_comb begin
      sqrt7_result = '0;
      sqrt7_exc = '0;
      
      if (is_sqrt7_op && i_sew == 2'b01) begin // SEW=16
         // Pack FP16 results into VLEN
         for (int i = 0; i < VLEN/16; i++) begin
            sqrt7_result[i*16 +: 16] = fp16_sqrt7_out[i];
         end
         // Combine FP16 exceptions (OR all exception flags)
         for (int i = 0; i < VLEN/16; i++) begin
            sqrt7_exc |= {fp16_sqrt7_exc[i][4], 1'b0, fp16_sqrt7_exc[i][3:0]};
         end
      end else if (is_sqrt7_op && i_sew == 2'b10) begin // SEW=32
         // Pack FP32 results into VLEN  
         for (int i = 0; i < VLEN/32; i++) begin
            sqrt7_result[i*32 +: 32] = fp32_sqrt7_out[i];
         end
         // Combine FP32 exceptions (OR all exception flags)
         for (int i = 0; i < VLEN/32; i++) begin
            sqrt7_exc |= {fp32_sqrt7_exc[i][4], 1'b0, fp32_sqrt7_exc[i][3:0]};
         end
      end
   end

   // 7-bit reciprocal results (vfrec7)
   always_comb begin
      rec7_result = '0;
      rec7_exc = '0;
      
      if (is_rec_op && i_sew == 2'b01) begin // SEW=16
         // Pack FP16 results into VLEN
         for (int i = 0; i < VLEN/16; i++) begin
            rec7_result[i*16 +: 16] = fp16_rec7_out[i];
         end
         // Combine FP16 exceptions (OR all exception flags)
         for (int i = 0; i < VLEN/16; i++) begin
            rec7_exc |= {fp16_rec7_exc[i][4], 1'b0, fp16_rec7_exc[i][3:0]};
         end
      end else if (is_rec_op && i_sew == 2'b10) begin // SEW=32
         // Pack FP32 results into VLEN  
         for (int i = 0; i < VLEN/32; i++) begin
            rec7_result[i*32 +: 32] = fp32_rec7_out[i];
         end
         // Combine FP32 exceptions (OR all exception flags)
         for (int i = 0; i < VLEN/32; i++) begin
            rec7_exc |= {fp32_rec7_exc[i][4], 1'b0, fp32_rec7_exc[i][3:0]};
         end
      end
   end

   // Final result selection
   logic [VLEN-1:0] compute_result, merged_result;
   tt_briscv_pkg::csr_fp_exc compute_exc;
   
   always_comb begin
      if (is_sqrt7_op) begin
         compute_result = sqrt7_result;
         compute_exc = sqrt7_exc;
      end else if (is_rec_op) begin
         compute_result = rec7_result;
         compute_exc = rec7_exc;
      end else begin
         compute_result = '0;
         compute_exc = '0;
      end
   end
   
   logic [2:0] log2_elements_per_reg; // log2(elements_per_reg) = log2(VLEN) - log2(SEW)
   assign log2_elements_per_reg = $clog2(VLEN) - (3 + i_sew);
   // Mask extraction and merging logic
   logic [VLEN/8-1:0] active_mask;  // Mask for current register slice
   logic [7:0] mask_base_offset;
   assign mask_base_offset = i_lmul_cnt << log2_elements_per_reg;
   
   // Extract appropriate mask slice based on SEW and LMUL_CNT
   always_comb begin
      active_mask = '0;
      
      // Calculate mask base offset using shift trick (same as VL calculation)
      
      // Generate mask per byte with nested loops
      case (i_sew)
         2'b00: begin // SEW=8, 1 mask bit per byte
            for (int i = 0; i < VLEN/8; i++) begin
               active_mask[i] = i_vm0[mask_base_offset + i];
            end
         end
         2'b01: begin // SEW=16, 1 mask bit per 2 bytes
            for (int i = 0; i < VLEN/16; i++) begin // VLEN/SEW elements
               for (int j = 0; j < 2; j++) begin // SEW/8 bytes per element
                  active_mask[i*2 + j] = i_vm0[mask_base_offset + i];
               end
            end
         end
         2'b10: begin // SEW=32, 1 mask bit per 4 bytes
            for (int i = 0; i < VLEN/32; i++) begin // VLEN/SEW elements
               for (int j = 0; j < 4; j++) begin // SEW/8 bytes per element
                  active_mask[i*4 + j] = i_vm0[mask_base_offset + i];
               end
            end
         end
         2'b11: begin // SEW=64, 1 mask bit per 8 bytes
            for (int i = 0; i < VLEN/64; i++) begin // VLEN/SEW elements
               for (int j = 0; j < 8; j++) begin // SEW/8 bytes per element
                  active_mask[i*8 + j] = i_vm0[mask_base_offset + i];
               end
            end
         end
      endcase
   end
   
   // Vector length mask for tail agnostic behavior
   logic [VLEN/8-1:0] vl_mask;
   logic [7:0] effective_vl;  // VL for this LMUL iteration
   
   always_comb begin
      // Calculate log2(elements_per_reg) = log2(VLEN) - i_sew
      // log2(VLEN=256) = 8, i_sew: 00=3, 01=4, 10=5, 11=6 (log2 of SEW)
      
      // Calculate effective VL for this LMUL iteration using shifts
      // effective_vl = vl - (elements_per_reg * lmul_cnt) 
      //              = vl - (lmul_cnt << log2_elements_per_reg)
      if (i_vl > (i_lmul_cnt << log2_elements_per_reg)) begin
         effective_vl = i_vl - (i_lmul_cnt << log2_elements_per_reg);
      end else begin
         effective_vl = 8'b0;  // No active elements in this iteration
      end
      
      // Generate VL mask based on effective VL
      vl_mask = '0;
      case (i_sew)
         2'b00: begin // SEW=8, 1 element per byte
            for (int i = 0; i < VLEN/8; i++) begin
               vl_mask[i] = (i < effective_vl);
            end
         end
         2'b01: begin // SEW=16, 1 element per 2 bytes
            for (int i = 0; i < VLEN/8; i++) begin
               vl_mask[i] = ((i/2) < effective_vl);
            end
         end
         2'b10: begin // SEW=32, 1 element per 4 bytes
            for (int i = 0; i < VLEN/8; i++) begin
               vl_mask[i] = ((i/4) < effective_vl);
            end
         end
         2'b11: begin // SEW=64, 1 element per 8 bytes
            for (int i = 0; i < VLEN/8; i++) begin
               vl_mask[i] = ((i/8) < effective_vl);
            end
         end
      endcase
   end

   // Merge compute_result with i_src3 based on active_mask and agnostic policies
   // Handle mask agnostic (vma) and tail agnostic (vta) behavior
   always_comb begin
      merged_result = '0;
      if (i_vm) begin
         // Unmasked operation (i_vm=1): handle tail agnostic
         for (int i = 0; i < VLEN/8; i++) begin
            if (vl_mask[i]) begin
               merged_result[i*8 +: 8] = compute_result[i*8 +: 8];  // Active elements
            end else begin
               // Tail elements: vta=1 -> agnostic (can be anything), vta=0 -> undisturbed (keep old)
               merged_result[i*8 +: 8] = i_vta ? 8'hFF : i_src3[i*8 +: 8];
            end
         end
      end else begin
         // Masked operation (i_vm=0): apply mask and handle agnostic policies
         for (int i = 0; i < VLEN/8; i++) begin
            if (vl_mask[i]) begin
               if (active_mask[i]) begin
                  merged_result[i*8 +: 8] = compute_result[i*8 +: 8];  // Active masked elements
               end else begin
                  // Masked-off elements: vma=1 -> agnostic (can be anything), vma=0 -> undisturbed (keep old)
                  merged_result[i*8 +: 8] = i_vma ? 8'hFF : i_src3[i*8 +: 8];
               end
            end else begin
               // Tail elements: vta=1 -> agnostic (can be anything), vta=0 -> undisturbed (keep old)
               merged_result[i*8 +: 8] = i_vta ? 8'hFF : i_src3[i*8 +: 8];
            end
         end
      end
   end

   // Output assignments
   // For now, vfrsqrt7 and vfrec7 are implemented
   assign o_busy         = 1'b0; // Combinational operation
   assign o_result_valid = i_id_vdiv_ex0_rts & (is_sqrt7_op | is_rec_op);
   assign o_result       = merged_result;
   assign o_result_exc   = is_sqrt7_op ? sqrt7_exc : 
                          is_rec_op ? rec7_exc : '0;
   assign o_result_lqid  = i_ldqid;

endmodule