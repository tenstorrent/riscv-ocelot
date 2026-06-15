// See LICENSE.TT for license details.
// Set NORTR==1 if o_rtr is ignored on the input side, or expected to always be high. This disabled stability checking, and random suppression of RTR.
module tt_rts_rtr_pipe_stage #(parameter WIDTH = 1, parameter NORTR = 0, parameter NORTS_DROPPED = 0, parameter SUPPRESSOR = 1)
(
   input       i_clk,
   input       i_reset_n,

   input       i_rts,
  output wire o_rtr,
  
  input       i_rtr,
  output wire o_rts,
  
  input  [WIDTH-1:0] i_data,
  output [WIDTH-1:0] o_data
);

reg occupied;

reg [WIDTH-1:0] data;

wire rtr;
`ifdef SIM

generate
  if (SUPPRESSOR==1) begin : gen_suppressor
    wire sup_rtr;
    tt_suppressor #(.WIDTH(1)) arb_suppressor
    (
      .i_clk     (i_clk        ),
      .i_reset_n (i_reset_n    ),
      .i_input   (rtr          ),
      .o_output  (sup_rtr      )
    );
    assign o_rtr = NORTR ? rtr : sup_rtr; // if NORTR is set, then generate as usual... otherwise, allow suppression to irritate the input
  end else begin : gen_no_suppressor
    assign o_rtr = rtr;
  end
endgenerate

`else
assign o_rtr = rtr;
`endif

assign rtr = !(occupied & (!i_rtr));

assign o_rts = occupied;

always @(posedge i_clk) begin
   if(!i_reset_n) begin
      occupied <= 1'b0;
    data <= {WIDTH{1'b0}};
  end
   else begin
     occupied <= (i_rts & o_rtr) | (occupied & (!i_rtr));
    if(i_rts & o_rtr) data <= i_data;
    else data <= data;
  end
end

assign o_data = data;

`ifdef SIM
// Check for stable RTS request
reg[WIDTH-1:0] pending_data;
reg            pending_rts;
always @(posedge i_clk) begin
  // If RTR is ignored on the output side, then there's nothing to check here
  pending_rts <= (!i_reset_n || o_rtr || NORTR) ? 1'b0 : i_rts;
  pending_data <= i_data;
end

`ASSERT_COND_CLK(pending_rts, (i_rts || NORTS_DROPPED), "RTS dropped without RTR");
`ASSERT_COND_CLK(pending_rts, i_data == pending_data, "Request data not stable while waiting for RTR");
`endif

endmodule
