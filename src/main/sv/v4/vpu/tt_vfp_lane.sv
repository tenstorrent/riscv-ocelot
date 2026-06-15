// See LICENSE.TT for license details.
module tt_vfp_lane 
(
   input  logic                            i_clk,
   input  logic                            i_reset_n,

   // Handshaking
   input  logic                      [3:0] i_valid,
   input  logic                      [1:0] i_mode,
   input  logic                            i_signedIn,
   input  logic                            i_signedOut,
   input  logic                      [1:0] i_fma_op, // 0: madd, 1: msub, 2: nmsub, 3: nmadd
   input  logic                      [2:0] i_roundingMode,
   input  logic                            i_cmp_sig,
   input  logic                            i_sel_op, // 0: min, 1: max
   input  logic                      [1:0] i_sew,
   input  logic                      [5:0] i_funct6, // Used for decoding compare instructions
   input  logic                      [4:0] i_vs1,    // Used for decoding convert instructions

   input  logic                [3:0][16:0] i_f16_a,
   input  logic                [3:0][16:0] i_f16_b,
   input  logic                [3:0][16:0] i_f16_c,
   input  logic                [1:0][32:0] i_f32_a,
   input  logic                [1:0][32:0] i_f32_b,
   input  logic                [1:0][32:0] i_f32_c,
   input  logic                [0:0][64:0] i_f64_a,
   input  logic                [0:0][64:0] i_f64_b,
   input  logic                [0:0][64:0] i_f64_c,

   input  logic                     [63:0] i_int,

   output logic                     [63:0] o_res_s1,
   output logic                      [4:0] o_exc_s1,

   output logic                     [63:0] o_res_s2,
   output logic                      [4:0] o_exc_s2
);

`include "HardFloat_localFuncs.vi"
`include "HardFloat_consts.vi"
   

   logic [3:0][ 7:0] i8_ncvtxf;
   logic [3:0][ 2:0] i8_ncvtxf_exc;

   logic [3:0][16:0] f16_fp_res;
   logic [3:0][ 4:0] f16_fp_exc;
   logic [3:0][15:0] i16_int_res;
   logic [3:0][ 4:0] i16_int_exc;
   logic [3:0]       f16_cmp_lt;
   logic [3:0]       f16_cmp_eq;
   logic [3:0]       f16_cmp_gt;
   logic [3:0][ 4:0] f16_cmp_exc;
   logic [3:0][ 9:0] f16_cls_res;
   logic [3:0][16:0] f16_sel_res;
   logic [3:0][ 4:0] f16_sel_exc;
   logic [3:0][15:0] f16_fma_res;
   logic [3:0][ 4:0] f16_fma_exc;
   logic [3:0][16:0] f16_ncvtfx;
   logic [3:0][ 4:0] f16_ncvtfx_exc;
   logic [3:0][16:0] f16_ncvtff;
   logic [3:0][ 4:0] f16_ncvtff_exc;
   logic [3:0][15:0] i16_ncvtxf;
   logic [3:0][ 2:0] i16_ncvtxf_exc;
   logic [3:0][16:0] f16_wcvtfx;
   logic [3:0][ 4:0] f16_wcvtfx_exc;
   logic [3:0][16:0] f16_recFN_in;
   logic [3:0][15:0] f16_FN_out;

   logic [1:0][32:0] f32_fp_res;
   logic [1:0][ 4:0] f32_fp_exc;
   logic [1:0][31:0] i32_int_res;
   logic [1:0][ 4:0] i32_int_exc;
   logic [1:0]       f32_cmp_lt;
   logic [1:0]       f32_cmp_eq;
   logic [1:0]       f32_cmp_gt;
   logic [1:0][ 4:0] f32_cmp_exc;
   logic [1:0][ 9:0] f32_cls_res;
   logic [1:0][32:0] f32_sel_res;
   logic [1:0][ 4:0] f32_sel_exc;
   logic [1:0][31:0] f32_fma_res;
   logic [1:0][ 4:0] f32_fma_exc;
   logic [1:0][32:0] f32_ncvtfx;
   logic [1:0][ 4:0] f32_ncvtfx_exc;
   logic [1:0][32:0] f32_ncvtff;
   logic [1:0][ 4:0] f32_ncvtff_exc;
   logic [1:0][31:0] i32_ncvtxf;
   logic [1:0][ 2:0] i32_ncvtxf_exc;
   logic [1:0][32:0] f32_wcvtfx;
   logic [1:0][ 4:0] f32_wcvtfx_exc;
   logic [1:0][32:0] f32_wcvtff;
   logic [1:0][ 4:0] f32_wcvtff_exc;
   logic [1:0][31:0] i32_wcvtxf;
   logic [1:0][ 2:0] i32_wcvtxf_exc;
   logic [1:0][32:0] f32_recFN_in;
   logic [1:0][31:0] f32_FN_out;

   logic [0:0][64:0] f64_fp_res;
   logic [0:0][ 4:0] f64_fp_exc;
   logic [0:0][63:0] i64_int_res;
   logic [0:0][ 4:0] i64_int_exc;
   logic [0:0]       f64_cmp_lt;
   logic [0:0]       f64_cmp_eq;
   logic [0:0]       f64_cmp_gt;
   logic [0:0][ 4:0] f64_cmp_exc;
   logic [0:0][ 9:0] f64_cls_res;
   logic [0:0][64:0] f64_sel_res;
   logic [0:0][ 4:0] f64_sel_exc;
   logic [0:0][63:0] f64_fma_res;
   logic [0:0][ 4:0] f64_fma_exc;
   logic [0:0][64:0] f64_wcvtfx;
   logic [0:0][ 4:0] f64_wcvtfx_exc;
   logic [0:0][64:0] f64_wcvtff;
   logic [0:0][ 4:0] f64_wcvtff_exc;
   logic [0:0][63:0] i64_wcvtxf;
   logic [0:0][ 2:0] i64_wcvtxf_exc;
   logic [0:0][64:0] f64_recFN_in;
   logic [0:0][63:0] f64_FN_out;

   logic [63:0] cvt_res;
   logic [ 4:0] cvt_exc;
   logic [ 3:0] msk_res;
   logic [ 4:0] msk_exc;
   logic [63:0] cls_res;
   logic [63:0] sel_res;
   logic [ 4:0] sel_exc;
   logic [63:0] sgn_res;
   logic [63:0] fmv_res;

   generate
      for (genvar i=0; i<4; i++) begin: sew_8
         // Floating Point to Integer
         recFNToIN #(.expWidth( 5),
                     .sigWidth(11),
                     .intWidth( 8) )
         fp2int
         (
            .control          (`flControl_tininessBeforeRounding),
            .in               (i_f16_b      [i]),
            .roundingMode     (i_roundingMode  ),
            .signedOut        (i_signedOut     ),
            .out              (i8_ncvtxf    [i]),
            .intExceptionFlags(i8_ncvtxf_exc[i])
         );
      end

      for (genvar i=0; i<4; i++) begin: sew_16
         ////////////////
         // Same Width //
         ////////////////
         tt_vfp_ex_unit #(.expWidth( 5),
                          .sigWidth(11) )
         ex_unit
         (
            .i_clk         (i_clk            ),
            .i_reset_n     (i_reset_n        ),
            .i_valid       (i_valid       [i]),
            .i_mode        (i_mode           ),
            .i_signedIn    (i_signedIn       ),
            .i_signedOut   (i_signedOut      ),
            .i_fma_op      (i_fma_op         ),
            .i_roundingMode(i_roundingMode   ),
            .i_cmp_sig     (i_cmp_sig        ),
            .i_sel_op      (i_sel_op         ),
            .i_a           (i_f16_a       [i]),
            .i_b           (i_f16_b       [i]),
            .i_c           (i_f16_c       [i]),
            .i_int         (i_int  [i*16+:16]),
            .o_fp_res      (f16_fp_res    [i]),
            .o_fp_exc      (f16_fp_exc    [i]),
            .o_int_res     (i16_int_res   [i]),
            .o_int_exc     (i16_int_exc   [i]),
            .o_cmp_lt      (f16_cmp_lt    [i]),
            .o_cmp_eq      (f16_cmp_eq    [i]),
            .o_cmp_gt      (f16_cmp_gt    [i]),
            .o_cmp_exc     (f16_cmp_exc   [i]),
            .o_cls_res     (f16_cls_res   [i]),
            .o_sel_res     (f16_sel_res   [i]),
            .o_sel_exc     (f16_sel_exc   [i]),
            .o_fma_res     (f16_fma_res   [i]),
            .o_fma_exc     (f16_fma_exc   [i])
         );

         ///////////////
         // Narrowing //
         ///////////////
         if (i < 2) begin: narrowing
            // Integer to Floating Point
            iNToRecFN #(.intWidth(32),
                        .expWidth( 5),
                        .sigWidth(11) )
            int2fp
            (
               .control       (`flControl_tininessBeforeRounding),
               .signedIn      (i_signedIn       ),
               .in            (i_int  [i*32+:32]),
               .roundingMode  (i_roundingMode   ),
               .out           (f16_ncvtfx    [i]),
               .exceptionFlags(f16_ncvtfx_exc[i])
            );

            // Floating Point to Floating Point
            recFNToRecFN #(.inExpWidth ( 8),
                           .inSigWidth (24),
                           .outExpWidth( 5),
                           .outSigWidth(11) )
            fp2fp
            (
               .control       (`flControl_tininessBeforeRounding),
               .in            (i_f32_b       [i]),
               .roundingMode  (i_roundingMode   ),
               .out           (f16_ncvtff    [i]),
               .exceptionFlags(f16_ncvtff_exc[i])
            );

            // Floating Point to Integer
            recFNToIN #(.expWidth( 8),
                        .sigWidth(24),
                        .intWidth(16) )
            fp2int
            (
               .control          (`flControl_tininessBeforeRounding),
               .in               (i_f32_b       [i]),
               .roundingMode     (i_roundingMode   ),
               .signedOut        (i_signedOut      ),
               .out              (i16_ncvtxf    [i]),
               .intExceptionFlags(i16_ncvtxf_exc[i])
            );
         end else begin : empty
            assign f16_ncvtfx    [i] = '0;
            assign f16_ncvtfx_exc[i] = '0;
            assign f16_ncvtff    [i] = '0;
            assign f16_ncvtff_exc[i] = '0;
            assign i16_ncvtxf    [i] = '0;
            assign i16_ncvtxf_exc[i] = '0;
         end

         ///////////////
         // Widenning //
         ///////////////
         if (1) begin: widenning
            // Integer to Floating Point
            iNToRecFN #(.intWidth( 8),
                        .expWidth( 5),
                        .sigWidth(11) )
            int2fp
            (
               .control       (`flControl_tininessBeforeRounding),
               .signedIn      (i_signedIn       ),
               .in            (i_int    [i*8+:8]),
               .roundingMode  (i_roundingMode   ),
               .out           (f16_wcvtfx    [i]),
               .exceptionFlags(f16_wcvtfx_exc[i])
            );
         end

         recFNToFN#(.expWidth( 5),
                    .sigWidth(11))
         recFNToFN(
            .in (f16_recFN_in[i]),
            .out(f16_FN_out  [i])
         );
      end

      for (genvar i=0; i<2; i++) begin: sew_32
         tt_vfp_ex_unit #(.expWidth(8),
                          .sigWidth(24))
         ex_unit
         (
            .i_clk         (i_clk            ),
            .i_reset_n     (i_reset_n        ),
            .i_valid       (i_valid       [i]),
            .i_mode        (i_mode           ),
            .i_signedIn    (i_signedIn       ),
            .i_signedOut   (i_signedOut      ),
            .i_fma_op      (i_fma_op         ),
            .i_roundingMode(i_roundingMode   ),
            .i_cmp_sig     (i_cmp_sig        ),
            .i_sel_op      (i_sel_op         ),
            .i_a           (i_f32_a       [i]),
            .i_b           (i_f32_b       [i]),
            .i_c           (i_f32_c       [i]),
            .i_int         (i_int  [i*32+:32]),
            .o_fp_res      (f32_fp_res    [i]),
            .o_fp_exc      (f32_fp_exc    [i]),
            .o_int_res     (i32_int_res   [i]),
            .o_int_exc     (i32_int_exc   [i]),
            .o_cmp_lt      (f32_cmp_lt    [i]),
            .o_cmp_eq      (f32_cmp_eq    [i]),
            .o_cmp_gt      (f32_cmp_gt    [i]),
            .o_cmp_exc     (f32_cmp_exc   [i]),
            .o_cls_res     (f32_cls_res   [i]),
            .o_sel_res     (f32_sel_res   [i]),
            .o_sel_exc     (f32_sel_exc   [i]),
            .o_fma_res     (f32_fma_res   [i]),
            .o_fma_exc     (f32_fma_exc   [i])
         );

         ///////////////
         // Narrowing //
         ///////////////
         if (i < 1) begin: narrowing
            // Integer to Floating Point
            iNToRecFN #(.intWidth(64),
                        .expWidth( 8),
                        .sigWidth(24) )
            int2fp
            (
               .control       (`flControl_tininessBeforeRounding),
               .signedIn      (i_signedIn       ),
               .in            (i_int  [i*64+:64]),
               .roundingMode  (i_roundingMode   ),
               .out           (f32_ncvtfx    [i]),
               .exceptionFlags(f32_ncvtfx_exc[i])
            );

            // Floating Point to Floating Point
            recFNToRecFN #(.inExpWidth (11),
                           .inSigWidth (53),
                           .outExpWidth( 8),
                           .outSigWidth(24) )
            fp2fp
            (
               .control       (`flControl_tininessBeforeRounding),
               .in            (i_f64_b       [i]),
               .roundingMode  (i_roundingMode   ),
               .out           (f32_ncvtff    [i]),
               .exceptionFlags(f32_ncvtff_exc[i])
            );

            // Floating Point to Integer
            recFNToIN #(.expWidth(11),
                        .sigWidth(53),
                        .intWidth(32) )
            fp2int
            (
               .control          (`flControl_tininessBeforeRounding),
               .in               (i_f64_b       [i]),
               .roundingMode     (i_roundingMode   ),
               .signedOut        (i_signedOut      ),
               .out              (i32_ncvtxf    [i]),
               .intExceptionFlags(i32_ncvtxf_exc[i])
            );
         end else begin: empty
            assign f32_ncvtfx    [i] = '0;
            assign f32_ncvtfx_exc[i] = '0;
            assign f32_ncvtff    [i] = '0;
            assign f32_ncvtff_exc[i] = '0;
            assign i32_ncvtxf    [i] = '0;
            assign i32_ncvtxf_exc[i] = '0;
         end

         ///////////////
         // Widenning //
         ///////////////
         if (1) begin: widenning
            // Integer to Floating Point
            iNToRecFN #(.intWidth(16),
                        .expWidth( 8),
                        .sigWidth(24) )
            int2fp
            (
               .control       (`flControl_tininessBeforeRounding),
               .signedIn      (i_signedIn       ),
               .in            (i_int  [i*16+:16]),
               .roundingMode  (i_roundingMode   ),
               .out           (f32_wcvtfx    [i]),
               .exceptionFlags(f32_wcvtfx_exc[i])
            );

            // Floating Point to Floating Point
            recFNToRecFN #(.inExpWidth ( 5),
                           .inSigWidth (11),
                           .outExpWidth( 8),
                           .outSigWidth(24) )
            fp2fp
            (
               .control       (`flControl_tininessBeforeRounding),
               .in            (i_f16_b       [i]),
               .roundingMode  (i_roundingMode   ),
               .out           (f32_wcvtff    [i]),
               .exceptionFlags(f32_wcvtff_exc[i])
            );

            // Floating Point to Integer
            recFNToIN #(.expWidth( 5),
                        .sigWidth(11),
                        .intWidth(32) )
            fp2int
            (
               .control          (`flControl_tininessBeforeRounding),
               .in               (i_f16_b       [i]),
               .roundingMode     (i_roundingMode   ),
               .signedOut        (i_signedOut      ),
               .out              (i32_wcvtxf    [i]),
               .intExceptionFlags(i32_wcvtxf_exc[i])
            );
         end

         recFNToFN#(.expWidth( 8),
                    .sigWidth(24))
         recFNToFN(
            .in (f32_recFN_in[i]),
            .out(f32_FN_out  [i])
         );
      end

      for (genvar i=0; i<1; i++) begin: sew_64
         tt_vfp_ex_unit #(.expWidth(11),
                          .sigWidth(53))
         ex_unit
         (
            .i_clk         (i_clk            ),
            .i_reset_n     (i_reset_n        ),
            .i_valid       (i_valid       [i]),
            .i_mode        (i_mode           ),
            .i_signedIn    (i_signedIn       ),
            .i_signedOut   (i_signedOut      ),
            .i_fma_op      (i_fma_op         ),
            .i_roundingMode(i_roundingMode   ),
            .i_cmp_sig     (i_cmp_sig        ),
            .i_sel_op      (i_sel_op         ),
            .i_a           (i_f64_a       [i]),
            .i_b           (i_f64_b       [i]),
            .i_c           (i_f64_c       [i]),
            .i_int         (i_int  [i*64+:64]),
            .o_fp_res      (f64_fp_res    [i]),
            .o_fp_exc      (f64_fp_exc    [i]),
            .o_int_res     (i64_int_res   [i]),
            .o_int_exc     (i64_int_exc   [i]),
            .o_cmp_lt      (f64_cmp_lt    [i]),
            .o_cmp_eq      (f64_cmp_eq    [i]),
            .o_cmp_gt      (f64_cmp_gt    [i]),
            .o_cmp_exc     (f64_cmp_exc   [i]),
            .o_cls_res     (f64_cls_res   [i]),
            .o_sel_res     (f64_sel_res   [i]),
            .o_sel_exc     (f64_sel_exc   [i]),
            .o_fma_res     (f64_fma_res   [i]),
            .o_fma_exc     (f64_fma_exc   [i])
         );

         ///////////////
         // Widenning //
         ///////////////
         if (1) begin: widenning
            // Integer to Floating Point
            iNToRecFN #(.intWidth(32),
                        .expWidth(11),
                        .sigWidth(53) )
            int2fp
            (
               .control       (`flControl_tininessBeforeRounding),
               .signedIn      (i_signedIn       ),
               .in            (i_int  [i*32+:32]),
               .roundingMode  (i_roundingMode   ),
               .out           (f64_wcvtfx    [i]),
               .exceptionFlags(f64_wcvtfx_exc[i])
            );

            // Floating Point to Floating Point
            recFNToRecFN #(.inExpWidth ( 8),
                           .inSigWidth (24),
                           .outExpWidth(11),
                           .outSigWidth(53) )
            fp2fp
            (
               .control       (`flControl_tininessBeforeRounding),
               .in            (i_f32_b       [i]),
               .roundingMode  (i_roundingMode   ),
               .out           (f64_wcvtff    [i]),
               .exceptionFlags(f64_wcvtff_exc[i])
            );

            // Floating Point to Integer
            recFNToIN #(.expWidth( 8),
                        .sigWidth(24),
                        .intWidth(64) )
            fp2int
            (
               .control          (`flControl_tininessBeforeRounding),
               .in               (i_f32_b       [i]),
               .roundingMode     (i_roundingMode   ),
               .signedOut        (i_signedOut      ),
               .out              (i64_wcvtxf    [i]),
               .intExceptionFlags(i64_wcvtxf_exc[i])
            );
         end

         recFNToFN#(.expWidth(11),
                    .sigWidth(53))
         recFNToFN(
            .in (f64_recFN_in[i]),
            .out(f64_FN_out  [i])
         );
      end
   endgenerate

   always_comb begin
      f16_recFN_in = '0;
      f32_recFN_in = '0;
      f64_recFN_in = '0;

      cvt_res      = '0;
      cvt_exc      = '0;
      sel_res      = '0;
      sel_exc      = '0;
      sgn_res      = '0;
      fmv_res      = '0;

      // VFMV.S.F
      if (i_funct6 == 6'b010000) begin
         fmv_res = (i_sew == 2'h1) ? {48'hffff_ffff_ffff, f16_FN_out[0]} :
                   (i_sew == 2'h2) ? {32'hffff_ffff,      f32_FN_out[0]} :
                                                          f64_FN_out[0];
         f16_recFN_in[0] = i_f16_b[0];
         f16_recFN_in[1] = i_f16_b[1];
         f16_recFN_in[2] = i_f16_b[2];
         f16_recFN_in[3] = i_f16_b[3];

         f32_recFN_in[0] = i_f32_b[0];
         f32_recFN_in[1] = i_f32_b[1];

         f64_recFN_in[0] = i_f64_b[0];
      end else
      // MIN / MAX
      if (i_funct6 inside {6'b000100, 6'b000110}) begin
         sel_res = (i_sew == 2'h1) ? f16_FN_out :
                   (i_sew == 2'h2) ? f32_FN_out :
                                     f64_FN_out;
         sel_exc = (i_sew == 2'h1) ? f16_sel_exc[0] | f16_sel_exc[1] | f16_sel_exc[2] | f16_sel_exc[3] :
                   (i_sew == 2'h2) ? f32_sel_exc[0] | f32_sel_exc[1]                                   :
                                     f64_sel_exc[0];

         f16_recFN_in[0] = f16_sel_res[0];
         f16_recFN_in[1] = f16_sel_res[1];
         f16_recFN_in[2] = f16_sel_res[2];
         f16_recFN_in[3] = f16_sel_res[3];

         f32_recFN_in[0] = f32_sel_res[0];
         f32_recFN_in[1] = f32_sel_res[1];

         f64_recFN_in[0] = f64_sel_res[0];
      end else
      // vfsgnj
      if (i_funct6 == 6'b001000) begin
         sgn_res         = (i_sew == 2'h1) ? f16_FN_out :
                           (i_sew == 2'h2) ? f32_FN_out :
                                             f64_FN_out;
         f16_recFN_in[0] = { i_f16_a[0][16],                  i_f16_b[0][15:0]};
         f16_recFN_in[1] = { i_f16_a[1][16],                  i_f16_b[1][15:0]};
         f16_recFN_in[2] = { i_f16_a[2][16],                  i_f16_b[2][15:0]};
         f16_recFN_in[3] = { i_f16_a[3][16],                  i_f16_b[3][15:0]};
         f32_recFN_in[0] = { i_f32_a[0][32],                  i_f32_b[0][31:0]};
         f32_recFN_in[1] = { i_f32_a[1][32],                  i_f32_b[1][31:0]};
         f64_recFN_in[0] = { i_f64_a[0][64],                  i_f64_b[0][63:0]};
      end else
      // vfsgnjn
      if (i_funct6 == 6'b001001) begin
         sgn_res         = (i_sew == 2'h1) ? f16_FN_out :
                           (i_sew == 2'h2) ? f32_FN_out :
                                             f64_FN_out;
         f16_recFN_in[0] = {!i_f16_a[0][16],                  i_f16_b[0][15:0]};
         f16_recFN_in[1] = {!i_f16_a[1][16],                  i_f16_b[1][15:0]};
         f16_recFN_in[2] = {!i_f16_a[2][16],                  i_f16_b[2][15:0]};
         f16_recFN_in[3] = {!i_f16_a[3][16],                  i_f16_b[3][15:0]};
         f32_recFN_in[0] = {!i_f32_a[0][32],                  i_f32_b[0][31:0]};
         f32_recFN_in[1] = {!i_f32_a[1][32],                  i_f32_b[1][31:0]};
         f64_recFN_in[0] = {!i_f64_a[0][64],                  i_f64_b[0][63:0]};
      end else
      // vfsgnjx
      if (i_funct6 == 6'b001010) begin
         sgn_res         = (i_sew == 2'h1) ? f16_FN_out :
                           (i_sew == 2'h2) ? f32_FN_out :
                                             f64_FN_out;
         f16_recFN_in[0] = { i_f16_a[0][16] ^ i_f16_b[0][16], i_f16_b[0][15:0]};
         f16_recFN_in[1] = { i_f16_a[1][16] ^ i_f16_b[1][16], i_f16_b[1][15:0]};
         f16_recFN_in[2] = { i_f16_a[2][16] ^ i_f16_b[2][16], i_f16_b[2][15:0]};
         f16_recFN_in[3] = { i_f16_a[3][16] ^ i_f16_b[3][16], i_f16_b[3][15:0]};
         f32_recFN_in[0] = { i_f32_a[0][32] ^ i_f32_b[0][32], i_f32_b[0][31:0]};
         f32_recFN_in[1] = { i_f32_a[1][32] ^ i_f32_b[1][32], i_f32_b[1][31:0]};
         f64_recFN_in[0] = { i_f64_a[0][64] ^ i_f64_b[0][64], i_f64_b[0][63:0]};
      end else begin
      // Conversion instruction
         case (i_vs1)
            // vfcvt.xu.f.v
            // vfcvt.x.f.v
            // vfcvt.rtz.xu.f.v
            // vfcvt.rtz.x.f.v
            5'b00000,
            5'b00001,
            5'b00110,
            5'b00111: begin
               cvt_res = (i_sew == 2'h1) ? i16_int_res :
                         (i_sew == 2'h2) ? i32_int_res :
                                           i64_int_res;
               cvt_exc = (i_sew == 2'h1) ? i16_int_exc[0] | 
                                           i16_int_exc[1] | 
                                           i16_int_exc[2] | 
                                           i16_int_exc[3]  :
                         (i_sew == 2'h2) ? i32_int_exc[0] |
                                           i32_int_exc[1]  :
                                           i64_int_exc[0];
            end
            // vfcvt.f.xu.v
            // vfcvt.f.x.v
            5'b00010,
            5'b00011: begin
               cvt_res = (i_sew == 2'h1) ? f16_FN_out :
                         (i_sew == 2'h2) ? f32_FN_out :
                                           f64_FN_out;
               cvt_exc = (i_sew == 2'h1) ? f16_fp_exc[0] | f16_fp_exc[1] | f16_fp_exc[2] | f16_fp_exc[3] :
                         (i_sew == 2'h2) ? f32_fp_exc[0] | f32_fp_exc[1]                                 :
                                           f64_fp_exc[0];

               f16_recFN_in[0] = f16_fp_res[0];
               f16_recFN_in[1] = f16_fp_res[1];
               f16_recFN_in[2] = f16_fp_res[2];
               f16_recFN_in[3] = f16_fp_res[3];

               f32_recFN_in[0] = f32_fp_res[0];
               f32_recFN_in[1] = f32_fp_res[1];

               f64_recFN_in[0] = f64_fp_res[0];
            end
            // vfwcvt.xu.f.v
            // vfwcvt.x.f.v
            // vfwcvt.rtz.xu.f.v
            // vfwcvt.rtz.x.f.v
            5'b01000,
            5'b01001,
            5'b01110,
            5'b01111: begin
               cvt_res = (i_sew == 2'h2) ? i32_wcvtxf :
                                           i64_wcvtxf;
               cvt_exc = (i_sew == 2'h2) ? ({5{i_valid[0]}} & i32_wcvtxf_exc[0]) |
                                           ({5{i_valid[1]}} & i32_wcvtxf_exc[1])  :
                                           ({5{i_valid[0]}} & i64_wcvtxf_exc[0]) ;
            end
            // vfwcvt.f.xu.v
            // vfwcvt.f.x.v
            5'b01010,
            5'b01011: begin
               cvt_res = (i_sew == 2'h1) ? f16_FN_out :
                         (i_sew == 2'h2) ? f32_FN_out :
                                           f64_FN_out;
               cvt_exc = (i_sew == 2'h1) ? ({5{i_valid[0]}} & f16_wcvtfx_exc[0]) |
                                           ({5{i_valid[1]}} & f16_wcvtfx_exc[1]) |
                                           ({5{i_valid[2]}} & f16_wcvtfx_exc[2]) |
                                           ({5{i_valid[3]}} & f16_wcvtfx_exc[3])  :
                         (i_sew == 2'h2) ? ({5{i_valid[0]}} & f32_wcvtfx_exc[0]) |
                                           ({5{i_valid[1]}} & f32_wcvtfx_exc[1])  :
                                           ({5{i_valid[0]}} & f64_wcvtfx_exc[0]) ;

               f16_recFN_in[0] = f16_wcvtfx[0];
               f16_recFN_in[1] = f16_wcvtfx[1];
               f16_recFN_in[2] = f16_wcvtfx[2];
               f16_recFN_in[3] = f16_wcvtfx[3];

               f32_recFN_in[0] = f32_wcvtfx[0];
               f32_recFN_in[1] = f32_wcvtfx[1];

               f64_recFN_in[0] = f64_wcvtfx[0];
            end
            // vfwcvt.f.f.v
            5'b01100: begin
               cvt_res = (i_sew == 2'h2) ? f32_FN_out :
                                           f64_FN_out;
               cvt_exc = (i_sew == 2'h2) ? ({5{i_valid[0]}} & f32_wcvtff_exc[0]) |
                                           ({5{i_valid[1]}} & f32_wcvtff_exc[1])  :
                                           ({5{i_valid[0]}} & f64_wcvtff_exc[0]) ;

               f32_recFN_in[0] = f32_wcvtff[0];
               f32_recFN_in[1] = f32_wcvtff[1];

               f64_recFN_in[0] = f64_wcvtff[0];
            end
            // vfncvt.xu.f.v
            // vfncvt.x.f.v
            // vfncvt.rtz.xu.f.v
            // vfncvt.rtz.x.f.v
            5'b10000,
            5'b10001,
            5'b10110,
            5'b10111: begin
               cvt_res = (i_sew == 2'h0) ? i8_ncvtxf  :
                         (i_sew == 2'h1) ? i16_ncvtxf :
                                           i32_ncvtxf;
               cvt_exc = (i_sew == 2'h0) ? ({5{i_valid[0]}} & { i8_ncvtxf_exc[0][2], 3'h0,  i8_ncvtxf_exc[0][0]}) |
                                           ({5{i_valid[1]}} & { i8_ncvtxf_exc[1][2], 3'h0,  i8_ncvtxf_exc[1][0]}) |
                                           ({5{i_valid[2]}} & { i8_ncvtxf_exc[2][2], 3'h0,  i8_ncvtxf_exc[2][0]}) |
                                           ({5{i_valid[3]}} & { i8_ncvtxf_exc[3][2], 3'h0,  i8_ncvtxf_exc[3][0]})  :
                         (i_sew == 2'h1) ? ({5{i_valid[0]}} & {i16_ncvtxf_exc[0][2], 3'h0, i16_ncvtxf_exc[0][0]}) |
                                           ({5{i_valid[1]}} & {i16_ncvtxf_exc[1][2], 3'h0, i16_ncvtxf_exc[1][0]})  :
                                           ({5{i_valid[0]}} & {i32_ncvtxf_exc[0][2], 3'h0, i32_ncvtxf_exc[0][0]});
            end
            // vfncvt.f.f.v
            // vfncvt.rod.f.f.v
            5'b10100,
            5'b10101: begin
               cvt_res = (i_sew == 2'h1) ? f16_FN_out :
                                           f32_FN_out;
               cvt_exc = (i_sew == 2'h1) ? ({5{i_valid[0]}} & f16_ncvtff_exc[0]) |
                                           ({5{i_valid[1]}} & f16_ncvtff_exc[1]) |
                                           ({5{i_valid[2]}} & f16_ncvtff_exc[2]) |
                                           ({5{i_valid[3]}} & f16_ncvtff_exc[3])  :
                                           ({5{i_valid[0]}} & f32_ncvtff_exc[0]) |
                                           ({5{i_valid[1]}} & f32_ncvtff_exc[1]);
            
               f16_recFN_in[0] = f16_ncvtff[0];
               f16_recFN_in[1] = f16_ncvtff[1];
               f16_recFN_in[2] = f16_ncvtff[2];
               f16_recFN_in[3] = f16_ncvtff[3];

               f32_recFN_in[0] = f32_ncvtff[0];
               f32_recFN_in[1] = f32_ncvtff[1];
            end
            // vfncvt.f.xu.v
            // vfncvt.f.x.v
            5'b10010,
            5'b10011: begin
               cvt_res = (i_sew == 2'h1) ? f16_FN_out :
                                           f32_FN_out;
               cvt_exc = (i_sew == 2'h1) ? ({5{i_valid[0]}} & f16_ncvtfx_exc[0]) |
                                           ({5{i_valid[1]}} & f16_ncvtfx_exc[1]) |
                                           ({5{i_valid[2]}} & f16_ncvtfx_exc[2]) |
                                           ({5{i_valid[3]}} & f16_ncvtfx_exc[3])  :
                                           ({5{i_valid[0]}} & f32_ncvtfx_exc[0]) |
                                           ({5{i_valid[1]}} & f32_ncvtfx_exc[1]);
            
               f16_recFN_in[0] = f16_ncvtfx[0];
               f16_recFN_in[1] = f16_ncvtfx[1];
               f16_recFN_in[2] = f16_ncvtfx[2];
               f16_recFN_in[3] = f16_ncvtfx[3];

               f32_recFN_in[0] = f32_ncvtfx[0];
               f32_recFN_in[1] = f32_ncvtfx[1];
            end
            default: begin
               cvt_res = '0;
               cvt_exc = '0;
            end
         endcase
      end
   end

   always_comb begin
      case (i_funct6)
         // vmfeq
         6'b011000: begin
            msk_res = (i_sew == 2'h1) ?        f16_cmp_eq  :
                      (i_sew == 2'h2) ? {2'h0, f32_cmp_eq} :
                                        {3'h0, f64_cmp_eq};
         end
         // vmfle
         6'b011001: begin
            msk_res = (i_sew == 2'h1) ?        f16_cmp_lt | f16_cmp_eq  :
                      (i_sew == 2'h2) ? {2'h0, f32_cmp_lt | f32_cmp_eq} :
                                        {3'h0, f64_cmp_lt | f64_cmp_eq};
         end
         // vmflt
         6'b011011: begin
            msk_res = (i_sew == 2'h1) ?        f16_cmp_lt  :
                      (i_sew == 2'h2) ? {2'h0, f32_cmp_lt} :
                                        {3'h0, f64_cmp_lt};
         end
         // vmfne
         6'b011100: begin
            msk_res = (i_sew == 2'h1) ?        ~f16_cmp_eq  :
                      (i_sew == 2'h2) ? {2'h0, ~f32_cmp_eq} :
                                        {3'h0, ~f64_cmp_eq};
         end
         // vmfgt
         6'b011101: begin
            msk_res = (i_sew == 2'h1) ?        f16_cmp_gt  :
                      (i_sew == 2'h2) ? {2'h0, f32_cmp_gt} :
                                        {3'h0, f64_cmp_gt};
         end
         // vmfge
         6'b011111: begin
            msk_res = (i_sew == 2'h1) ?        f16_cmp_gt | f16_cmp_eq  :
                      (i_sew == 2'h2) ? {2'h0, f32_cmp_gt | f32_cmp_eq} :
                                        {3'h0, f64_cmp_gt | f64_cmp_eq};
         end
         default: begin
            msk_res = '0;
         end
      endcase
   end

   assign msk_exc = (i_sew == 2'h1) ? (f16_cmp_exc[0] | f16_cmp_exc[1] | f16_cmp_exc[2] | f16_cmp_exc[3]) :
                    (i_sew == 2'h2) ? (f32_cmp_exc[0] | f32_cmp_exc[1]                                  ) :
                                      (f64_cmp_exc[0]                                                   );

   assign cls_res = (i_sew == 2'h1) ? { 6'h0, f16_cls_res[3],
                                        6'h0, f16_cls_res[2],
                                        6'h0, f16_cls_res[1],
                                        6'h0, f16_cls_res[0] } :
                    (i_sew == 2'h2) ? {22'h0, f32_cls_res[1],
                                       22'h0, f32_cls_res[0] } :
                                      {54'h0, f64_cls_res[0] };

   always_comb begin
      o_res_s1 = '0;
      o_exc_s1 = '0;

      case (i_funct6)
         // Move instructions
         6'b010000: begin
            o_res_s1 = fmv_res;
            o_exc_s1 = '0;
         end
         // Mask compare instructions
         6'b011000,
         6'b011001,
         6'b011011,
         6'b011100,
         6'b011101,
         6'b011111: begin
            o_res_s1 = {60'h0, msk_res[3:0]};
            o_exc_s1 = msk_exc;
         end
         // Classify instruction
         6'b010011: begin
            o_res_s1 = cls_res;
            o_exc_s1 = '0;
         end
         6'b000100,
         6'b000110: begin
            o_res_s1 = sel_res;
            o_exc_s1 = sel_exc;
         end
         // Sign Injection instruciton
         6'b001000,
         6'b001001,
         6'b001010: begin
            o_res_s1 = sgn_res;
         end
         // Conversion
         6'b010010: begin
            o_res_s1 = cvt_res;
            o_exc_s1 = cvt_exc;
         end
         default  : begin
            o_res_s1 = '0;
            o_exc_s1 = '0;
         end
      endcase
   end

   logic [1:0] sew_s2;

   always_ff @(posedge i_clk) begin
      if (|i_valid) begin
         sew_s2 <= i_sew;
      end
   end

   assign o_res_s2 = (sew_s2 == 2'h1) ? f16_fma_res :
                     (sew_s2 == 2'h2) ? f32_fma_res :
                                        f64_fma_res;
   assign o_exc_s2 = (sew_s2 == 2'h1) ? f16_fma_exc[3] |
                                        f16_fma_exc[2] |
                                        f16_fma_exc[1] |
                                        f16_fma_exc[0] :
                     (sew_s2 == 2'h2) ? f32_fma_exc[1] |
                                        f32_fma_exc[0] :
                                        f64_fma_exc;

endmodule
