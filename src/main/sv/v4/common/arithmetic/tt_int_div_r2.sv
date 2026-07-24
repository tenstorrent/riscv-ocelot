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

module tt_int_div_r2
// Integer division unit
// Use binary non-restoring method
// Provides result, either quotient or remainder, in XLEN+1 cycles
// Support both signed and unsigned operands
// Support divide-by-zero and overflow conditions
#(parameter 
   XLEN    = 32,
   CP_BITS = 4   // Number of bits each checkpoint covers
)
(
   input                   i_clk ,
   input                   i_reset_n,

   // Input Interface
   input  logic            i_vld,
   output logic            o_ack,
   input  logic            i_sgn, // 0: unsigned, 1: signed
   input  logic            i_rem, // 0: quotient, 1: remainder
   input  logic [XLEN-1:0] i_rs1, // Dividend
   input  logic [XLEN-1:0] i_rs2, // Divisor

   // Output Interface
   output logic            o_rts,
   input  logic            i_rtr,
   output logic [XLEN-1:0] o_res  // Result
);

   // Mux A
   logic          opa_sel_rs1;
   logic          opa_sel_rx2;
   logic          opa_sel_r;
   logic          opa_sel_q;

   logic [XLEN:0] opa_rs1;
   logic [XLEN:0] opa_rx2;
   logic [XLEN:0] opa_r;
   logic [XLEN:0] opa_q;

   // Mux B
   logic          opb_sel_rs2;
   logic          opb_sel_rs2n;
   logic          opb_sel_qn;

   logic [XLEN:0] opb_rs2;
   logic [XLEN:0] opb_rs2n;
   logic [XLEN:0] opb_qn;

   // Adder
   logic [XLEN:0] opa;
   logic [XLEN:0] opb;
   logic    [1:0] incr; // 0, +1, +2
   logic [XLEN:0] addr_out;
   
   // Control
   typedef enum logic [1:0] {
      INIT = 0, // Initial State
      LOOP = 1, // Loop each quotient digit
      RRDY = 2, // Result is ready
      RSVD = 3      
   } state_e;

   logic                    state_update;
   state_e                  state;
   state_e                  state_nxt;
   logic [$clog2(XLEN)-1:0] i;
   logic [$clog2(XLEN)-1:0] i_nxt;

   logic                    rs1_is_pos;        // rs1 is zero or positive
   logic                    rs2_is_pos;        // rs2 is zero or positive
   logic                    remainder_is_pos;  // remainder is zero or positive
   logic                    remainder_is_zero; // remainder is zero

   // Remainder Latch
   logic          remainder_clken;
   logic [XLEN:0] remainder; 
   logic [XLEN:0] remainder_nxt; 
   
   // Quotient Shift Register
   typedef struct packed {
      logic pos;
      logic mag;
   } quotient_t;

   logic      [XLEN-1:0] quotient_clken;
   quotient_t            quotient_in;
   quotient_t [XLEN-1:0] quotient;
   logic      [XLEN-1:0] q_pos;
   logic      [XLEN-1:0] q_neg;

   // Special cases
   logic is_dbz; // Division by zero
   logic is_dbo; // Division by one
   logic is_ovf; // Overflow

   localparam NUM_CP  = XLEN / CP_BITS;
   logic [NUM_CP-1:0] jump_enable;

   ////////////////////////
   // Dectect Skip Cases // 
   ////////////////////////
   assign jump_enable[0] = 1'b0;

   generate 
      for (genvar cp_ix=1; cp_ix<NUM_CP; cp_ix++) begin
         assign jump_enable[cp_ix] = ~|i_rs1[XLEN-1:cp_ix*CP_BITS]           ||
                                    ( &i_rs1[XLEN-1:cp_ix*CP_BITS] && i_sgn) ||
                                      |jump_enable[cp_ix-1:0];
      end
   endgenerate

   /////////////
   // Control // 
   /////////////
   assign rs1_is_pos        = !i_sgn || !i_rs1[XLEN-1];
   assign rs2_is_pos        = !i_sgn || !i_rs2[XLEN-1];
   assign remainder_is_pos  = ( (state == INIT) ? (!i_rs1    [XLEN-1] || !i_sgn)
                                                :  !remainder[XLEN  ]            );
   assign remainder_is_zero = ( (state == RRDY) ? ( remainder[XLEN:0] == '0    )
                                                : ( opa      [XLEN:0] == '0    ) );

   always_ff @(posedge i_clk) begin
      if (~i_reset_n) begin
         state <= INIT;
         i     <= $clog2(XLEN)'(XLEN-1);
      end else begin
         if (state_update) begin
            state <= state_nxt;
            i     <= i_nxt;
         end
      end
   end

   always_comb begin
      case (state)
         INIT: begin
            state_update = i_vld;
            state_nxt    = (is_dbz || is_dbo || is_ovf) ? RRDY : LOOP;
            i_nxt        = i - 1;
            for (int cp_ix=NUM_CP-1; cp_ix>0; cp_ix--) begin
               if (jump_enable[cp_ix]) begin
                  i_nxt = $clog2(XLEN)'((cp_ix * CP_BITS) - 1);
               end
            end
         end
         LOOP: begin
            state_update = i_vld;
            state_nxt    = (i == 0) ? RRDY : LOOP;
            i_nxt        = i - 1;
         end
         RRDY: begin
            state_update = o_rts && i_rtr;
            state_nxt    = INIT;
            i_nxt        = $clog2(XLEN)'(XLEN-1);
         end 
         default: begin
            state_update = 1'b0;
            state_nxt    = RSVD;
            i_nxt        = '0;
         end
      endcase
   end

   assign o_ack = (state == RRDY) && i_rtr;
                  
   ///////////
   // Mux A // 
   ///////////
   assign opa_rs1[XLEN:1] = (i_sgn) ? {XLEN{i_rs1[XLEN-1]}} : '0;
   assign opa_rs1[     0] = i_rs1[XLEN-1];
   assign opa_rx2[XLEN:1] = remainder[XLEN-1:0];
   assign opa_rx2[     0] = i_rs1[i];
   assign opa_r  [XLEN:0] = remainder;
   assign opa_q  [XLEN:0] = {1'b0, q_pos};

   assign opa_sel_rs1 = i_vld && state == INIT;
   assign opa_sel_rx2 =          state == LOOP;
   assign opa_sel_r   =          state == RRDY &&  i_rem;
   assign opa_sel_q   =          state == RRDY && !i_rem;

   assign opa         = ({XLEN+1{opa_sel_rs1}} & opa_rs1) |
                        ({XLEN+1{opa_sel_rx2}} & opa_rx2) |
                        ({XLEN+1{opa_sel_r  }} & opa_r  ) |
                        ({XLEN+1{opa_sel_q  }} & opa_q  );

   ///////////
   // Mux B // 
   ///////////
   assign opb_rs2  =  {i_sgn && i_rs2[XLEN-1], i_rs2};
   assign opb_rs2n = ~{i_sgn && i_rs2[XLEN-1], i_rs2};
   assign opb_qn   = ~{i_sgn && q_neg[XLEN-1], q_neg};

   always_comb begin
      if (i_vld) begin
         case (state)
            INIT,
            LOOP: begin
               opb_sel_rs2  = !quotient_in.pos && quotient_in.mag;
               opb_sel_rs2n =  quotient_in.pos && quotient_in.mag;
               opb_sel_qn   = 1'b0;
            end
            RRDY: begin
               if (i_rem) begin
                  // No rounding when remainder is 0
                  if (remainder_is_zero) begin
                     opb_sel_rs2  = 1'b0;
                     opb_sel_rs2n = 1'b0;
                     opb_sel_qn   = 1'b0;
                  end else begin
                      case ({rs1_is_pos, rs2_is_pos, remainder_is_pos})
                         3'b000 : begin opb_sel_rs2 = 1'b0; opb_sel_rs2n = 1'b0; opb_sel_qn = 1'b0; end // +0
                         3'b001 : begin opb_sel_rs2 = 1'b1; opb_sel_rs2n = 1'b0; opb_sel_qn = 1'b0; end // +D
                         3'b010 : begin opb_sel_rs2 = 1'b0; opb_sel_rs2n = 1'b0; opb_sel_qn = 1'b0; end // +0
                         3'b011 : begin opb_sel_rs2 = 1'b0; opb_sel_rs2n = 1'b1; opb_sel_qn = 1'b0; end // -D
                         3'b100 : begin opb_sel_rs2 = 1'b0; opb_sel_rs2n = 1'b1; opb_sel_qn = 1'b0; end // -D
                         3'b101 : begin opb_sel_rs2 = 1'b0; opb_sel_rs2n = 1'b0; opb_sel_qn = 1'b0; end // +0
                         3'b110 : begin opb_sel_rs2 = 1'b1; opb_sel_rs2n = 1'b0; opb_sel_qn = 1'b0; end // +D
                         3'b111 : begin opb_sel_rs2 = 1'b0; opb_sel_rs2n = 1'b0; opb_sel_qn = 1'b0; end // +0
                      endcase
                  end
               // Produce quotient
               end else begin
                  // Use opb to get negative quotient
                  opb_sel_rs2  = 1'b0;
                  opb_sel_rs2n = 1'b0;
                  opb_sel_qn   = 1'b1;
               end
            end
            default: begin
               opb_sel_rs2  = 1'b0;
               opb_sel_rs2n = 1'b0;
               opb_sel_qn   = 1'b0;
            end
         endcase
      end else begin
         opb_sel_rs2  = 1'b0;
         opb_sel_rs2n = 1'b0;
         opb_sel_qn   = 1'b0;
      end
   end

   assign opb         = ({XLEN+1{opb_sel_rs2 }} & opb_rs2 ) |
                        ({XLEN+1{opb_sel_rs2n}} & opb_rs2n) |
                        ({XLEN+1{opb_sel_qn  }} & opb_qn  );
   ///////////
   // Adder // 
   ///////////
   assign addr_out = opa + opb + incr;

   always_comb begin
      if (i_vld) begin
         // Step 1-N
         if (state inside {INIT, LOOP}) begin
            // +1 as part of 2's comp of Divisor
            incr = {1'b0, quotient_in.pos && quotient_in.mag};
         end else begin
         // Final Step
            // For Remainder +1 as part of 2's comp of Divisor
            if (i_rem) begin
               // No rounding when remainder is 0
               if (remainder_is_zero) begin
                  incr = 2'h0;   
               end else begin
                  case ({rs1_is_pos, rs2_is_pos, remainder_is_pos})
                     3'b000 : incr = 2'h0; // +0
                     3'b001 : incr = 2'h0; // +D
                     3'b010 : incr = 2'h0; // +0
                     3'b011 : incr = 2'h1; // -D
                     3'b100 : incr = 2'h1; // -D
                     3'b101 : incr = 2'h0; // +0
                     3'b110 : incr = 2'h0; // +D
                     3'b111 : incr = 2'h0; // +0
                     default: incr = '0;
                  endcase
               end
            end else begin
            // For Quotient
            // There's a shared +1 as part of 2's comp of negative quotient digits
            // Based on rounding, need adjustment of: -1,  0, +1 
            // Thus the final incr amount will be   :  0,  1,  2
               if (remainder_is_zero) begin
                  incr = 2'h1;   
               end else begin
                  case ({rs1_is_pos, rs2_is_pos, remainder_is_pos})
                     3'b000 : incr = 2'h1; // +0
                     3'b001 : incr = 2'h0; // +D
                     3'b010 : incr = 2'h1; // +0
                     3'b011 : incr = 2'h2; // -D
                     3'b100 : incr = 2'h2; // -D
                     3'b101 : incr = 2'h1; // +0
                     3'b110 : incr = 2'h0; // +D
                     3'b111 : incr = 2'h1; // +0
                     default: incr = '0;
                  endcase
               end
            end
         end
      end else begin
         incr = 2'h0;
      end
   end

   ///////////////
   // Remainder //
   ///////////////
   always_ff @(posedge i_clk) begin
      if (remainder_clken) begin
         remainder <= remainder_nxt;
      end
   end

   assign remainder_nxt = addr_out;

   assign remainder_clken = ((state == INIT) && i_vld) ||
                            ( state == LOOP          );

   //////////////
   // Quotient //
   //////////////
   always_ff @(posedge i_clk) begin
      for (int b_ix=0; b_ix<XLEN; b_ix++) begin: quotient_reg
         // Initialize MSB quotients when jump forward
         if (state == INIT && |jump_enable) begin
            // Initialize to zeros when Dividend is positive
            if (!i_sgn || !i_rs1[XLEN-1]) begin
               quotient[b_ix].pos <= 1'b1;
               quotient[b_ix].mag <= 1'b0;
            end else
            // Initialize to -1,+1,+1... when Divisor is positive
            if (!i_rs2[XLEN-1]) begin
               if (b_ix == (XLEN-1)) begin
                   quotient[b_ix].pos <= 1'b0;
                   quotient[b_ix].mag <= 1'b1;
               end else
               if (jump_enable[b_ix/CP_BITS]) begin
                   quotient[b_ix].pos <= 1'b1;
                   quotient[b_ix].mag <= 1'b1;
               end
            end else begin
            // Initialize to +1,-1,-1... when Divisor is negative
               if (b_ix == (XLEN-1)) begin
                   quotient[b_ix].pos <= 1'b1;
                   quotient[b_ix].mag <= 1'b1;
               end else
               if (jump_enable[b_ix/CP_BITS]) begin
                   quotient[b_ix].pos <= 1'b0;
                   quotient[b_ix].mag <= 1'b1;
               end
            end
         end else
         if (quotient_clken[b_ix]) begin
            quotient[b_ix] <= quotient_in;
         end
      end
   end

   always_comb begin
      for (int b_ix=0; b_ix<XLEN; b_ix++) begin
         q_pos[b_ix] = quotient[b_ix].mag &&  quotient[b_ix].pos;
         q_neg[b_ix] = quotient[b_ix].mag && !quotient[b_ix].pos;
      end
   end

   always_comb begin
      quotient_clken = '0;
      if (i_vld && state inside {INIT, LOOP}) begin
         quotient_clken[i] = 1'b1;
      end
   end

   always_comb begin
      if (rs2_is_pos) begin
         quotient_in.pos = (remainder_is_pos) ? 1'b1 : 1'b0;
         quotient_in.mag = !remainder_is_zero;
      end else begin
         quotient_in.pos = (remainder_is_pos) ? 1'b0 : 1'b1;
         quotient_in.mag = !remainder_is_zero;
      end
   end

   ///////////////
   // Exception //
   ///////////////
   assign is_dbz = i_vld && (i_rs2 ==   '0);
   assign is_dbo = i_vld && (i_rs2 ==    1);
   assign is_ovf = i_vld && (i_sgn == 1'b1                    &&
                             i_rs1 == {1'b1, {XLEN-1{1'b0}}}  && 
                             i_rs2 ==   '1                      );
   
   ////////////
   // Output //
   ////////////
   assign o_rts = state == RRDY;

   always_comb begin
      if (is_dbz) begin
         o_res = (i_rem) ? i_rs1 : '1;
      end else
      if (is_dbo) begin
         o_res = (i_rem) ?    '0 : i_rs1;
      end else
      if (is_ovf) begin
         o_res = (i_rem) ?    '0 : {1'b1, {XLEN-1{1'b0}}};
      end else begin
         o_res = addr_out[XLEN-1:0];
      end 
   end

endmodule
