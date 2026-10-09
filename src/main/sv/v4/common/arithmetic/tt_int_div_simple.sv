/*************************************************************************
 * Simple Integer Division Unit - Restoring Division Algorithm
 * 
 * Drop-in replacement for tt_int_div_r2.sv with same interface
 * Uses simple restoring division algorithm that's easy to verify
 *************************************************************************/

module tt_int_div_simple
#(parameter 
   XLEN = 32
)
(
   input                   i_clk,
   input                   i_reset_n,

   // Input Interface
   input  logic            i_vld,
   output logic            o_ack,
   input  logic            i_sgn,      // 0: unsigned, 1: signed
   input  logic            i_rem,      // 0: quotient, 1: remainder
   input  logic [XLEN-1:0] i_rs1,      // Dividend
   input  logic [XLEN-1:0] i_rs2,      // Divisor

   // Output Interface
   output logic            o_rts,
   input  logic            i_rtr,
   output logic [XLEN-1:0] o_res       // Result
);

   // State machine
   typedef enum logic [1:0] {
      IDLE = 0,
      BUSY = 1,
      DONE = 2,
      RSVD = 3
   } state_e;
   
   state_e state, state_nxt;
   
   // Division registers  
   logic [XLEN-1:0]   quotient, quotient_nxt;
   logic [XLEN-1:0]   remainder, remainder_nxt;  // Partial remainder (R)
   logic [XLEN-1:0]   divisor;
   logic [6:0]        bit_count, bit_count_nxt;  // Supports up to XLEN=64
   
   // Control signals
   logic              is_signed, is_remainder;
   logic              dividend_neg, divisor_neg, result_neg;
   logic [XLEN-1:0]   abs_dividend, abs_divisor;
   
   // Exception detection
   logic              div_by_zero, overflow;
   
   // State machine
   always_ff @(posedge i_clk) begin
      if (!i_reset_n) begin
         state <= IDLE;
      end else begin
         state <= state_nxt;
      end
   end
   
   always_comb begin
      state_nxt = state;
      case (state)
         IDLE: if (i_vld) state_nxt = BUSY;
         BUSY: if (bit_count == 0) state_nxt = DONE;
         DONE: if (i_rtr) state_nxt = IDLE;
         default: state_nxt = IDLE;
      endcase
   end
   
   // Division registers
   always_ff @(posedge i_clk) begin
      if (!i_reset_n) begin
         quotient  <= '0;
         remainder <= '0;
         divisor   <= '0;
         bit_count <= 7'(XLEN);  // Initialize to XLEN
         is_signed <= '0;
         is_remainder <= '0;
         dividend_neg <= '0;
         divisor_neg  <= '0;
      end else begin
         quotient  <= quotient_nxt;
         remainder <= remainder_nxt;
         bit_count <= bit_count_nxt;
         
         if (state == IDLE && i_vld) begin
            // Capture control signals
            is_signed    <= i_sgn;
            is_remainder <= i_rem;
            
            // Handle signed operands
            dividend_neg <= i_sgn && i_rs1[XLEN-1];
            divisor_neg  <= i_sgn && i_rs2[XLEN-1];
            
            // Store absolute values
            divisor <= abs_divisor;
            
            // Initialize division: R=0, Q=dividend
            quotient  <= abs_dividend;  // Q = dividend
            remainder <= '0;            // R = 0
            bit_count <= 7'(XLEN);
         end else if (state == DONE && i_rtr) begin
            // Reset bit_count when going back to IDLE
            bit_count <= 7'(XLEN);
         end
      end
   end
   
   // Absolute value computation
   assign abs_dividend = (i_sgn && i_rs1[XLEN-1]) ? -i_rs1 : i_rs1;
   assign abs_divisor  = (i_sgn && i_rs2[XLEN-1]) ? -i_rs2 : i_rs2;
   
   // Exception detection
   assign div_by_zero = (i_rs2 == '0);
   assign overflow    = i_sgn && (i_rs1 == {1'b1, {XLEN-1{1'b0}}}) && (i_rs2 == '1);
   
   // Division step logic - Correct restoring division
   always_comb begin
      quotient_nxt  = quotient;
      remainder_nxt = remainder;
      bit_count_nxt = bit_count;
      
      if (state == BUSY && bit_count > 0) begin
         // Shift {R,Q} left by 1: R gets MSB of Q, Q shifts left
         remainder_nxt = (remainder << 1) | quotient[XLEN-1];
         quotient_nxt  = quotient << 1;
         
         // Check if R >= divisor
         if (remainder_nxt >= divisor) begin
            // Subtract divisor and set quotient LSB
            remainder_nxt = remainder_nxt - divisor;
            quotient_nxt[0] = 1'b1;
         end else begin
            // Leave quotient LSB as 0 (already shifted)
         end
         
         bit_count_nxt = bit_count - 1;
      end
   end
   
   // Result computation
   logic [XLEN-1:0] final_quotient, final_remainder;
   
   always_comb begin
      // Determine result sign
      result_neg = is_signed && (dividend_neg ^ divisor_neg) && !div_by_zero && !overflow;
      
      // Compute final results
      final_quotient  = result_neg ? -quotient : quotient;
      final_remainder = (is_signed && dividend_neg) ? -remainder : remainder;
   end
   
   // Output logic
   assign o_ack = (state == DONE) && i_rtr;
   assign o_rts = (state == DONE);
   
   always_comb begin
      if (div_by_zero) begin
         // Division by zero: quotient = all 1s, remainder = dividend
         o_res = is_remainder ? i_rs1 : '1;
      end else if (overflow) begin
         // Overflow: quotient = most negative, remainder = 0
         o_res = is_remainder ? '0 : {1'b1, {XLEN-1{1'b0}}};
      end else begin
         // Normal case
         o_res = is_remainder ? final_remainder : final_quotient;
      end
   end

endmodule