// See LICENSE.TT for license details.
`include "briscv_defines.h"
module tt_skid_buffer
#(
   parameter
   WIDTH = 32
)
(
   input              i_clk,
   input              i_reset_n,

   input              i_rts,
   output reg         o_rtr,
   input  [WIDTH-1:0] i_pld,
   
   output wire        o_rts,
   input              i_rtr,
   output [WIDTH-1:0] o_pld
  
);

   reg             skid_valid;
   reg [WIDTH-1:0] skid_pld;

   assign o_rts     = i_rts ||
                      skid_valid;

   // Stall back when skid buffer is occupied
   assign o_rtr     = !skid_valid;

   assign o_pld     = (!skid_valid) ? i_pld 
                                    : skid_pld;

   always_ff @(posedge i_clk) begin
      if (~i_reset_n) begin
         skid_valid <= 1'b0;
      end else begin
         // Load skid buffer when downstream is not ready
         if ( i_rts &&
              o_rtr &&
             !i_rtr    ) begin
            skid_valid <= 1'b1;
         end else 
         // Clear when drain to down stream
         if (skid_valid &&
             i_rtr        ) begin
            skid_valid <=  1'b0;
         end
      end 
   end
  
   always_ff @(posedge i_clk) begin
      // Load skid buffer when downstream is not ready
      if ( i_rts &&
           o_rtr &&
          !i_rtr    ) begin
         skid_pld <= i_pld;
      end
   end
  
endmodule
