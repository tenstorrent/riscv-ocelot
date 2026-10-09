// See LICENSE.TT for license details.
module tt_decoded_mux #(parameter 
                    DISABLE_ASSERTIONS=0,
                    VALUE_WIDTH=32,                     
                    MUX_WIDTH=4
)
(
input                                i_clk                   , // This is needed for the assertion
input                                i_reset_n               , // This is needed for the assertion
input                                i_enable                , // This is only used to qualify the assertion 
input         [VALUE_WIDTH-1:0]      i_inputs [MUX_WIDTH-1:0],
input         [MUX_WIDTH-1:0]        i_select                ,
output  logic [VALUE_WIDTH-1:0]      o_output 
);

integer  m;

`ifdef SIM
   generate
     // If these assertions are "don't care" for this instance of the module, set the DISABLE_ASSERTIONS=1 parameter when instantiating the module
     if(DISABLE_ASSERTIONS == 0) begin
         // Check if the select is one-hot
         `ASSERT_COND_CLK(i_enable, $onehot(i_select[MUX_WIDTH-1:0]), "tt_decoded_mux received a select input signal which was not one-hot");
     end
   endgenerate
`endif

always_comb begin 
   o_output[VALUE_WIDTH-1:0] = {VALUE_WIDTH{1'b0}};
   for(m=0; m<MUX_WIDTH; m++) begin
      o_output[VALUE_WIDTH-1:0] = o_output[VALUE_WIDTH-1:0] | ({VALUE_WIDTH{i_select[m]}} & i_inputs[m][VALUE_WIDTH-1:0]);
   end
end

endmodule
