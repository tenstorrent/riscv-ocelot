// See LICENSE.TT for license details.
module tt_pipe_stage #(parameter WIDTH=8)
(
   input i_clk,
   input i_reset_n,
   input i_en,
   input [WIDTH-1:0] i_d,

   output reg [WIDTH-1:0] o_q
);

always @(posedge i_clk) begin
        if(!i_reset_n) o_q <= {WIDTH{1'b0}};
        else begin
                if(i_en) o_q <= i_d;
        end
end

endmodule
