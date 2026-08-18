// See LICENSE.TT for license details.
module tt_vfp_encoder_lane
(
   input  logic      [63:0] data_in,
   input  logic             data_sel,     // 0: same , 1: upscale
   output logic [3:0][16:0] f16_data_out,
   output logic [1:0][32:0] f32_data_out,
   output logic [0:0][64:0] f64_data_out
);

`include "HardFloat_localFuncs.vi"
`include "HardFloat_consts.vi"

   logic [3:0][16:0] f16_converted;
   logic [1:0][32:0] f32_converted, f32_upscaled;
   logic [0:0][64:0] f64_converted, f64_upscaled;

   generate
      for (genvar i=0; i<4; i++) begin: f16
         fNToRecFN #(.expWidth(5), .sigWidth(11))
         fp16_fNToRecFN
         (
            .in (data_in[16*i+:16]),
            .out(f16_converted[i] )
         );

         assign f16_data_out[i] = f16_converted[i];
      end

      for (genvar i=0; i<2; i++) begin: f32
         fNToRecFN #(.expWidth(8), .sigWidth(24))
         fp32_fNToRecFN
         (
            .in (data_in[32*i+:32]),
            .out(f32_converted[i] )
         );

         recFNToRecFN #(.inExpWidth(5), .inSigWidth(11), .outExpWidth(8), .outSigWidth(24))
         fp16_to_fp32
         (
            .control       (`flControl_tininessBeforeRounding),
            .in            (f16_converted[i]),
            .roundingMode  (`round_near_even),
            .out           (f32_upscaled [i]),
            .exceptionFlags()
         );

         assign f32_data_out[i] = data_sel ? f32_upscaled[i] : f32_converted[i];
      end

      for (genvar i=0; i<1; i++) begin: f64
         fNToRecFN #(.expWidth(11), .sigWidth(53))
         fp64_fNToRecFN
         (
            .in (data_in[63:0]),
            .out(f64_converted)
         );

         recFNToRecFN #(.inExpWidth(8), .inSigWidth(24), .outExpWidth(11), .outSigWidth(53))
         fp32_to_fp64
         (
            .control       (`flControl_tininessBeforeRounding),
            .in            (f32_converted[0]),
            .roundingMode  (`round_near_even),
            .out           (f64_upscaled    ),
            .exceptionFlags()
         );

         assign f64_data_out[i] = data_sel ? f64_upscaled[i] : f64_converted[i];
      end
   endgenerate

endmodule
