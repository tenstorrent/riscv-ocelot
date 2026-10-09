// See LICENSE.TT for license details.
`include "briscv_defines.h"

module tt_reshape #(parameter 
                    DISABLE_ASSERTIONS=0,
                    X_WIDTH=32,                     
                    Y_WIDTH=4
)
(
   input  logic [Y_WIDTH-1:0]    i_xy_signal       [X_WIDTH-1:0]        ,       // input  signal of the shape i_xy_signal[X_WIDTH-1:0][Y_WIDTH-1:0]
   output logic [X_WIDTH-1:0]    o_yx_signal       [Y_WIDTH-1:0]                // output signal of the shape o_yx_signal[Y_WIDTH-1:0][X_WIDTH-1:0]
);


integer x, y;

`ifdef SIM
  generate
    if(DISABLE_ASSERTIONS == 0) begin
        // Check if the encoded input signal is within the range of the output decoded signal width; 
        // If this is a true "don't care", set the DISABLE_ASSERTIONS=1 parameter when instantiating the module
        // `ASSERT_COND_CLK(i_enable, (i_encoded_signal[ENCODED_WIDTH-1:0] < DECODED_WIDTH), "tt_decoder received an encoded input signal value which was larger than the configured output decoded width");
    end
  endgenerate
`endif

always_comb begin 
    for(y=0; y<Y_WIDTH; y++) begin
      for(x=0; x<X_WIDTH; x++) begin
        o_yx_signal[y][x] = i_xy_signal[x][y];
      end
    end
end

endmodule
