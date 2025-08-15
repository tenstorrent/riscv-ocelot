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
   // For .vx operations, src2 is a scalar that gets broadcast to all elements
   generate
      for (genvar i = 0; i < VLEN/8; i++) begin : gen_sew8
         assign src1_sew8[i] = i_src1[i*8 +: 8];
         assign src2_sew8[i] = is_vector_scalar ? i_src2[7:0] : i_src2[i*8 +: 8];
      end
      for (genvar i = 0; i < VLEN/16; i++) begin : gen_sew16
         assign src1_sew16[i] = i_src1[i*16 +: 16];
         assign src2_sew16[i] = is_vector_scalar ? i_src2[15:0] : i_src2[i*16 +: 16];
      end
      for (genvar i = 0; i < VLEN/32; i++) begin : gen_sew32
         assign src1_sew32[i] = i_src1[i*32 +: 32];
         assign src2_sew32[i] = is_vector_scalar ? i_src2[31:0] : i_src2[i*32 +: 32];
      end
      for (genvar i = 0; i < VLEN/64; i++) begin : gen_sew64
         assign src1_sew64[i] = i_src1[i*64 +: 64];
         assign src2_sew64[i] = is_vector_scalar ? i_src2[63:0] : i_src2[i*64 +: 64];
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
   
   // Integer division state and control (sequential approach)
   typedef enum logic [1:0] {
      INT_IDLE = 0,    // No integer operation in progress
      INT_BUSY = 1,    // Integer division in progress
      INT_DONE = 2,    // Integer result ready
      INT_RSVD = 3
   } int_div_state_e;
   
   int_div_state_e int_div_state, int_div_state_nxt;
   logic int_div_state_update;
   
   // Integer division context storage (preserved during multi-cycle operation)
   logic [tt_briscv_pkg::LQ_DEPTH_LOG2-1:0] stored_ldqid;
   logic [7:0] stored_vl;
   logic [2:0] stored_lmul_cnt;
   logic [1:0] stored_sew;
   logic stored_vm, stored_vta, stored_vma;
   logic [VLEN-1:0] stored_vm0;
   logic [VLEN-1:0] stored_src3;
   logic stored_is_div_op, stored_is_rem_op, stored_is_signed_op;
   logic stored_context_valid;
   
   // Integer division unit interface
   logic int_div_vld, int_div_ack, int_div_rts, int_div_rtr;
   logic int_div_sgn, int_div_rem;
   logic [63:0] int_div_rs1, int_div_rs2;  // Max SEW=64
   logic [63:0] int_div_res;
   
   // Current element being processed for integer operations
   logic [7:0] int_element_idx;
   logic [7:0] int_elements_per_reg;
   logic int_operation_complete;
   
   // Integer division unit instantiation
   tt_int_div_r2 #(
      .XLEN(64)  // Support up to SEW=64
   ) int_div_unit (
      .i_clk(i_clk),
      .i_reset_n(i_reset_n),
      .i_vld(int_div_vld),
      .o_ack(int_div_ack),
      .i_sgn(int_div_sgn),
      .i_rem(int_div_rem),
      .i_rs1(int_div_rs1),
      .i_rs2(int_div_rs2),
      .o_rts(int_div_rts),
      .i_rtr(int_div_rtr),
      .o_res(int_div_res)
   );
   
   // Separate output result multiplexing for sqrt, sqrt7, rec7, and integer operations
   logic [VLEN-1:0] sqrt_result, sqrt7_result, rec7_result, int_div_result;
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

   // Integer division state machine and control logic
   always_ff @(posedge i_clk) begin
      if (!i_reset_n) begin
         int_div_state <= INT_IDLE;
         stored_context_valid <= 1'b0;
         int_element_idx <= '0;
      end else if (int_div_state_update) begin
         int_div_state <= int_div_state_nxt;
         
         // Store context when starting integer operation
         if (int_div_state == INT_IDLE && int_div_state_nxt == INT_BUSY) begin
            stored_ldqid <= i_ldqid;
            stored_vl <= i_vl;
            stored_lmul_cnt <= i_lmul_cnt;
            stored_sew <= i_sew;
            stored_vm <= i_vm;
            stored_vta <= i_vta;
            stored_vma <= i_vma;
            stored_vm0 <= i_vm0;
            stored_src3 <= i_src3;
            stored_is_div_op <= is_div_op;
            stored_is_rem_op <= is_rem_op;
            stored_is_signed_op <= is_signed_op;
            stored_context_valid <= 1'b1;
            int_element_idx <= '0;
         end
         
         // Update element index during processing
         if (int_div_state == INT_BUSY && int_div_ack) begin
            int_element_idx <= int_element_idx + 1;
         end
         
         // Clear context when done
         if (int_div_state == INT_DONE && int_div_state_nxt == INT_IDLE) begin
            stored_context_valid <= 1'b0;
         end
      end
   end
   
   // Integer division state machine
   always_comb begin
      int_div_state_update = 1'b0;
      int_div_state_nxt = int_div_state;
      
      case (int_div_state)
         INT_IDLE: begin
            if (i_id_vdiv_ex0_rts && (is_div_op || is_rem_op)) begin
               int_div_state_update = 1'b1;
               int_div_state_nxt = INT_BUSY;
            end
         end
         
         INT_BUSY: begin
            // Move to next element or completion
            if (int_div_ack) begin
               int_div_state_update = 1'b1;
               if (int_operation_complete) begin
                  int_div_state_nxt = INT_DONE;
               end
               // Stay in INT_BUSY for next element
            end
         end
         
         INT_DONE: begin
            // Result is ready, immediately return to IDLE (blocking operation)
            int_div_state_update = 1'b1;
            int_div_state_nxt = INT_IDLE;
         end
         
         default: begin
            int_div_state_update = 1'b1;
            int_div_state_nxt = INT_IDLE;
         end
      endcase
   end
   
   // Calculate elements per register and completion status
   always_comb begin
      case (stored_context_valid ? stored_sew : i_sew)
         2'b00: int_elements_per_reg = VLEN / 8;   // SEW=8
         2'b01: int_elements_per_reg = VLEN / 16;  // SEW=16
         2'b10: int_elements_per_reg = VLEN / 32;  // SEW=32
         2'b11: int_elements_per_reg = VLEN / 64;  // SEW=64
      endcase
      
      int_operation_complete = (int_element_idx >= (int_elements_per_reg - 1));
   end
   
   // Integer division unit control and data preparation
   always_comb begin
      // Default values
      int_div_vld = 1'b0;
      int_div_sgn = 1'b0;
      int_div_rem = 1'b0;
      int_div_rs1 = '0;
      int_div_rs2 = '0;
      
      if (int_div_state == INT_BUSY) begin
         // Extract current element data based on SEW
         case (stored_sew)
            2'b00: begin // SEW=8
               int_div_rs1 = {56'b0, src1_sew8[int_element_idx]};
               int_div_rs2 = {56'b0, src2_sew8[int_element_idx]};
            end
            2'b01: begin // SEW=16
               int_div_rs1 = {48'b0, src1_sew16[int_element_idx]};
               int_div_rs2 = {48'b0, src2_sew16[int_element_idx]};
            end
            2'b10: begin // SEW=32
               int_div_rs1 = {32'b0, src1_sew32[int_element_idx]};
               int_div_rs2 = {32'b0, src2_sew32[int_element_idx]};
            end
            2'b11: begin // SEW=64
               int_div_rs1 = src1_sew64[int_element_idx];
               int_div_rs2 = src2_sew64[int_element_idx];
            end
         endcase
         
         // Control signals
         int_div_sgn = stored_is_signed_op;
         int_div_rem = stored_is_rem_op;
         int_div_vld = !int_div_rts;  // Start next operation when previous is done
      end
   end
   
   // Always ready to accept results (blocking operation with dedicated writeback)
   assign int_div_rtr = 1'b1;
   
   // Integer division result register - updated incrementally as each element completes
   always_ff @(posedge i_clk) begin
      if (!i_reset_n) begin
         int_div_result <= '0;
      end else begin
         // Clear result when starting new operation
         if (int_div_state == INT_IDLE && int_div_state_nxt == INT_BUSY) begin
            int_div_result <= '0;
         end
         // Update result when each division completes
         else if (int_div_state == INT_BUSY && int_div_rts && int_div_rtr) begin
            case (stored_sew)
               2'b00: begin // SEW=8
                  int_div_result[int_element_idx*8 +: 8] <= int_div_res[7:0];
               end
               2'b01: begin // SEW=16
                  int_div_result[int_element_idx*16 +: 16] <= int_div_res[15:0];
               end
               2'b10: begin // SEW=32
                  int_div_result[int_element_idx*32 +: 32] <= int_div_res[31:0];
               end
               2'b11: begin // SEW=64
                  int_div_result[int_element_idx*64 +: 64] <= int_div_res[63:0];
               end
            endcase
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
      end else if (int_div_state == INT_DONE) begin
         compute_result = int_div_result;
         compute_exc = '0;  // Integer operations don't generate FP exceptions
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

      logic use_stored_context;
      logic sel_vm, sel_vta, sel_vma;
      logic [VLEN-1:0] sel_src3;
      
      assign use_stored_context = (int_div_state == INT_DONE);
      assign sel_vm = use_stored_context ? stored_vm : i_vm;
      assign sel_vta = use_stored_context ? stored_vta : i_vta;
      assign sel_vma = use_stored_context ? stored_vma : i_vma;
      assign sel_src3 = use_stored_context ? stored_src3 : i_src3;
   // Merge compute_result with src3 based on mask and agnostic policies
   // Use stored context for integer operations (INT_DONE), current context for FP operations
   always_comb begin
      merged_result = '0;
      
      // Select mask information based on operation state
      
      if (sel_vm) begin
         // Unmasked operation (vm=1): handle tail agnostic
         for (int i = 0; i < VLEN/8; i++) begin
            if (vl_mask[i]) begin
               merged_result[i*8 +: 8] = compute_result[i*8 +: 8];  // Active elements
            end else begin
               // Tail elements: vta=1 -> agnostic (can be anything), vta=0 -> undisturbed (keep old)
               merged_result[i*8 +: 8] = sel_vta ? 8'hFF : sel_src3[i*8 +: 8];
            end
         end
      end else begin
         // Masked operation (vm=0): apply mask and handle agnostic policies
         for (int i = 0; i < VLEN/8; i++) begin
            if (vl_mask[i]) begin
               if (active_mask[i]) begin
                  merged_result[i*8 +: 8] = compute_result[i*8 +: 8];  // Active masked elements
               end else begin
                  // Masked-off elements: vma=1 -> agnostic (can be anything), vma=0 -> undisturbed (keep old)
                  merged_result[i*8 +: 8] = sel_vma ? 8'hFF : sel_src3[i*8 +: 8];
               end
            end else begin
               // Tail elements: vta=1 -> agnostic (can be anything), vta=0 -> undisturbed (keep old)
               merged_result[i*8 +: 8] = sel_vta ? 8'hFF : sel_src3[i*8 +: 8];
            end
         end
      end
   end

   // Output assignments
   // Supports vfrsqrt7, vfrec7 (combinational) and vdiv/vdivu/vrem/vremu (sequential multi-cycle)
   assign o_busy         = (int_div_state != INT_IDLE); // Busy when state machine is active
   assign o_result_valid = (is_sqrt7_op | is_rec_op) ? (i_id_vdiv_ex0_rts & (is_sqrt7_op | is_rec_op)) :
                          (int_div_state == INT_DONE);
   assign o_result       = merged_result;
   assign o_result_exc   = is_sqrt7_op ? sqrt7_exc : 
                          is_rec_op ? rec7_exc : '0;
   assign o_result_lqid  = stored_context_valid ? stored_ldqid : i_ldqid;

endmodule