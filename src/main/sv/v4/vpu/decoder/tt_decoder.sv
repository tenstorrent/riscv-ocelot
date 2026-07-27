// See LICENSE.TT for license details.
`include "briscv_defines.h"

module tt_decoder #(parameter 
                    DISABLE_ASSERTIONS=0,
                    DECODED_WIDTH=32,                     
                    ENCODED_WIDTH=$clog2(DECODED_WIDTH)
)
(
   input                               i_clk                   ,       // This is needed for the assertion
   input                               i_reset_n               ,       // This is needed for the assertion
   input                               i_enable                ,       // This can be used to qualify the output (either for functionality or as a data-gating signal for power), also used by the assertion
   input        [ENCODED_WIDTH-1:0]    i_encoded_signal        ,       // encoded input value which is to be converted to a decoded value
   output logic [DECODED_WIDTH-1:0]    o_decoded_signal                // output decoded value which will be one-hot when the i_enable is set (and zero when i_enable is not set)
);


integer i;

`ifdef SIM
  generate
    if(DISABLE_ASSERTIONS == 0) begin
        // Check if the encoded input signal is within the range of the output decoded signal width; 
        // If this is a true "don't care", set the DISABLE_ASSERTIONS=1 parameter when instantiating the module
        `ASSERT_COND_CLK( i_enable, (i_encoded_signal[ENCODED_WIDTH-1:0] < DECODED_WIDTH), "tt_decoder received an encoded input signal value which was larger than the configured output decoded width");
        `ASSERT_COND_CLK( i_enable, $onehot(o_decoded_signal[DECODED_WIDTH-1:0]), "tt_decoder produced a decoded output signal which was not one-hot (check for xprop!)");        
        `ASSERT_COND_CLK(~i_enable, (o_decoded_signal[DECODED_WIDTH-1:0] == {DECODED_WIDTH{1'b0}}), "tt_decoder should produce a zero output when the i_enable is not set (check for xprop!)");                
    end
  endgenerate
`endif

always_comb begin 
    for(i=0; i<DECODED_WIDTH; i++) begin
      if(i_enable && (i_encoded_signal == i)) begin
        o_decoded_signal[i] = 1'b1;
      end
      else begin
        o_decoded_signal[i] = 1'b0;
      end
    end
end

endmodule
