// See LICENSE.TT for license details.
module tt_vfp_encoder
#( parameter NUM_LANE = 2)
(
   input  logic [NUM_LANE-1:0]     [63:0] data_in,
   input  logic                    [ 1:0] data_sel,     // 0: same , 1: reserved, 2: upscale_lo, 3: upscale_hi
   output logic [NUM_LANE-1:0][3:0][16:0] f16_data_out,
   output logic [NUM_LANE-1:0][1:0][32:0] f32_data_out,
   output logic [NUM_LANE-1:0][0:0][64:0] f64_data_out
);

`include "HardFloat_localFuncs.vi"
`include "HardFloat_consts.vi"

   logic                           lane_sel;
   logic [NUM_LANE-1:0][1:0][31:0] lane_in;

   generate
      for (genvar i=0; i<NUM_LANE; i++) begin: lane
         tt_vfp_encoder_lane
         encoder_lane
         (
            .data_in     (lane_in     [i]),
            .data_sel    (lane_sel       ),
            .f16_data_out(f16_data_out[i]),
            .f32_data_out(f32_data_out[i]),
            .f64_data_out(f64_data_out[i])
         );
      end
   endgenerate

   assign lane_sel = data_sel[1];

   always_comb begin
      for (int i=0; i<NUM_LANE; i++) begin
         // inline
         if (data_sel == 2'h0) begin
            lane_in[i] = data_in[i];
         end else
         // widen_lo
         if (data_sel == 2'h2) begin
            lane_in[i][0] = data_in[i/2][32*(i%2)+:32];
            lane_in[i][1] = '0;
         end else
         // widen_hi
         if (data_sel == 2'h3) begin
            lane_in[i][0] = data_in[(NUM_LANE+i)/2][32*(i%2)+:32];
            lane_in[i][1] = '0;
         end else begin
            lane_in[i] = '0;
         end
      end
   end

endmodule
