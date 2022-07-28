// See LICENSE.TT for license details.
module tt_vfp_ex_unit 
#(
   parameter expWidth=5,
   parameter sigWidth=11
)
(
   input  logic                            i_clk,
   input  logic                            i_reset_n,

   // Handshaking
   input  logic                            i_valid,
   input  logic                      [1:0] i_mode,   // 0: cvt, 1: add/sub, 2: mul, 3: fma
   input  logic                            i_signedIn,
   input  logic                            i_signedOut,
   input  logic                      [1:0] i_fma_op, // 0: madd, 1: msub, 2: nmsub, 3: nmadd
   input  logic                      [2:0] i_roundingMode,
   input  logic                            i_cmp_sig,
   input  logic                            i_sel_op, // 0: min, 1: max

   input  logic      [expWidth+sigWidth:0] i_a,
   input  logic      [expWidth+sigWidth:0] i_b,
   input  logic      [expWidth+sigWidth:0] i_c,

   input  logic    [expWidth+sigWidth-1:0] i_int,

   output logic      [expWidth+sigWidth:0] o_fp_res,
   output logic                      [4:0] o_fp_exc,

   output logic    [expWidth+sigWidth-1:0] o_int_res,
   output logic                      [4:0] o_int_exc,

   output logic                            o_cmp_lt,
   output logic                            o_cmp_eq,
   output logic                            o_cmp_gt,
   output logic                      [4:0] o_cmp_exc,

   output logic                      [9:0] o_cls_res,

   output logic      [expWidth+sigWidth:0] o_sel_res,
   output logic                      [4:0] o_sel_exc,

   output logic    [expWidth+sigWidth-1:0] o_fma_res,
   output logic                      [4:0] o_fma_exc
);

`include "HardFloat_localFuncs.vi"
`include "HardFloat_consts.vi"
   
   logic      [expWidth+sigWidth:0] a;
   logic      [expWidth+sigWidth:0] b;
   logic      [expWidth+sigWidth:0] c;

   // Fused-Multiply-Add
   tt_vfp_fma #(.sigWidth(sigWidth),
                .expWidth(expWidth) )
   fma
   (
      .i_clk         (i_clk),
      .i_reset_n     (i_reset_n),
      .i_valid       (i_valid && (i_mode != 2'h0)),
      .i_op          (i_fma_op),
      .i_roundingMode(i_roundingMode),
      .i_a           (a),
      .i_b           (b),
      .i_c           (c),
      .o_res         (o_fma_res),
      .o_exc         (o_fma_exc)
   );

   assign a =                    i_a;
   assign b = (i_mode == 2'h1) ? (expWidth+sigWidth+1)'(1<<(expWidth+sigWidth-1))
                               : i_b;
   assign c = (i_mode == 2'h2) ? (expWidth+sigWidth+1)'((i_a[expWidth+sigWidth] ^
                                                         i_b[expWidth+sigWidth]  ) << (expWidth+sigWidth))
                               : i_c;

   // Integer to Floating Point
   logic [4:0] fp_exc_raw;

   iNToRecFN #(.intWidth(expWidth+sigWidth),
               .expWidth(expWidth),
               .sigWidth(sigWidth))
   int2fp
   (
      .control       (`flControl_tininessBeforeRounding),
      .signedIn      (i_signedIn),
      .in            (i_int),
      .roundingMode  (i_roundingMode),
      .out           (o_fp_res),
      .exceptionFlags(fp_exc_raw)
   );
   assign o_fp_exc = {5{i_valid}} & fp_exc_raw;

   // Floating Point to Integer
   logic [2:0] int_exc_raw;
   recFNToIN #(.expWidth(expWidth         ),
               .sigWidth(sigWidth         ),
               .intWidth(expWidth+sigWidth) )
   fp2int
   (
      .control          (`flControl_tininessBeforeRounding),
      .in               (i_b),
      .roundingMode     (i_roundingMode),
      .signedOut        (i_signedOut),
      .out              (o_int_res),
      .intExceptionFlags(int_exc_raw)
   );
   assign o_int_exc = {5{i_valid}} & {int_exc_raw[2], 3'h0, int_exc_raw[0]};

   // Comparison
   logic [4:0] cmp_exc_raw;
   compareRecFN #(.expWidth(expWidth),
                  .sigWidth(sigWidth) )
   compare
   (
      .a             (i_b), // vs2
      .b             (i_a), // vs1
      .signaling     (i_cmp_sig),
      .lt            (o_cmp_lt),
      .eq            (o_cmp_eq),
      .gt            (o_cmp_gt),
      .unordered     (),
      .exceptionFlags(cmp_exc_raw)
   );
   assign o_cmp_exc = {5{i_valid}} & cmp_exc_raw;

   // Classify
   logic                cls_a_isNaN, cls_b_isNaN;
   logic                cls_a_isInf, cls_b_isInf;
   logic                cls_a_isZero,cls_b_isZero;
   logic                cls_a_sign,  cls_b_sign;
   logic [expWidth+1:0] cls_a_sExp,  cls_b_sExp;
   logic [sigWidth  :0] cls_a_sig,   cls_b_sig;
   logic                cls_subnormal;
   logic                cls_normal;

   recFNToRawFN #(.expWidth(expWidth),
                  .sigWidth(sigWidth) )
   classify_a
   (
      .in    (i_a),
      .isNaN (cls_a_isNaN),
      .isInf (cls_a_isInf),
      .isZero(cls_a_isZero),
      .sign  (cls_a_sign),
      .sExp  (cls_a_sExp),
      .sig   (cls_a_sig)
   );

   recFNToRawFN #(.expWidth(expWidth),
                  .sigWidth(sigWidth) )
   classify_b
   (
      .in    (i_b),
      .isNaN (cls_b_isNaN),
      .isInf (cls_b_isInf),
      .isZero(cls_b_isZero),
      .sign  (cls_b_sign),
      .sExp  (cls_b_sExp),
      .sig   (cls_b_sig)
   );

   //  If those three bits are 000, 110, or 111, the floating-point value is a zero, infinity, or NaN, respectively;
   //  otherwise, the value is a normalized finite number.
   //  In the latter case, if the recoded exponent field is 2k + 2 or more, the value is a regular normal number,
   //  and if it’s less, the value is a normalized subnormal number. 
   assign cls_subnormal = !cls_b_isNaN  &&
                          !cls_b_isInf  &&
                          !cls_b_isZero &&
                           cls_b_sExp <  ((1 << (expWidth-1))+2);
   assign cls_normal    = !cls_b_isNaN  &&
                          !cls_b_isInf  &&
                          !cls_b_isZero &&
                           cls_b_sExp >= ((1 << (expWidth-1))+2);
   assign o_cls_res     = {   
                           cls_b_isNaN &&  cls_b_sig[sigWidth-2],  // quiet NaN
                           cls_b_isNaN && !cls_b_sig[sigWidth-2],  // signaling NaN
                          !cls_b_sign  &&  cls_b_isInf,            // positive inf
                          !cls_b_sign  &&  cls_normal,             // positive normal number.
                          !cls_b_sign  &&  cls_subnormal,          // positive subnormal number.
                          !cls_b_sign  &&  cls_b_isZero,           // +0 
                           cls_b_sign  &&  cls_b_isZero,           // -0   
                           cls_b_sign  &&  cls_subnormal,          // negative subnormal number.
                           cls_b_sign  &&  cls_normal,             // negative normal number.
                           cls_b_sign  &&  cls_b_isInf             // negative inf
                          };

   // Min/Max
   logic       sel_a_isSigNaN;
   logic       sel_b_isSigNaN;
   logic       sel_b_gt_a;

   isSigNaNRecFN #(.expWidth(expWidth),
                   .sigWidth(sigWidth) )
   isSigNaN_a
   (
      .in      (i_a),
      .isSigNaN(sel_a_isSigNaN)
   );

   isSigNaNRecFN #(.expWidth(expWidth),
                   .sigWidth(sigWidth) )
   isSigNaN_b
   (
      .in      (i_b),
      .isSigNaN(sel_b_isSigNaN)
   );

   assign sel_b_gt_a = (cls_a_sign && cls_b_sign ) ? i_b < i_a  :
                       (cls_a_sign ^  cls_b_sign ) ? cls_a_sign :
                                                     i_b > i_a;

   assign o_sel_res = (cls_a_isNaN && cls_b_isNaN) ? {1'b0, 3'b111, (expWidth-2)'(0), 1'b1, (sigWidth-2)'(0)} : // Canonical NaN
                      (cls_a_isNaN               ) ? i_b                                                      :
                      (               cls_b_isNaN) ? i_a                                                      :
                      ( i_sel_op   && !sel_b_gt_a) ? i_a                                                      :
                      (!i_sel_op   &&  sel_b_gt_a) ? i_a                                                      :
                                                     i_b;
                          
   assign o_sel_exc = {5{i_valid}} & {sel_a_isSigNaN || sel_b_isSigNaN, 4'h0};

endmodule

