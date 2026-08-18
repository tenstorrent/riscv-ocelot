// See LICENSE.TT for license details.
module tt_vfp_fma
#(parameter sigWidth=11,
  parameter expWidth=5  )
(
   input  logic                         i_clk,
   input  logic                         i_reset_n,
   input  logic                         i_valid,
   input  logic                   [1:0] i_op, // 0: madd, 1: msub, 2: nmsub, 3: nmadd
   input  logic                   [2:0] i_roundingMode,
   input  logic   [expWidth+sigWidth:0] i_a,
   input  logic   [expWidth+sigWidth:0] i_b,
   input  logic   [expWidth+sigWidth:0] i_c,

   output logic [expWidth+sigWidth-1:0] o_res,
   output logic                   [4:0] o_exc
);

`include "HardFloat_localFuncs.vi"
`include "HardFloat_consts.vi"

   logic       valid_s1;
   logic [2:0] roundingMode_s1;
   logic [(sigWidth - 1):0] mulAddA, mulAddB;
   logic [(sigWidth*2 - 1):0] mulAddC;
   logic [5:0] intermed_compactState, intermed_compactState_s1;
   logic signed [(expWidth + 1):0] intermed_sExp, intermed_sExp_s1;
   logic [(clog2(sigWidth + 1) - 1):0] intermed_CDom_CAlignDist, intermed_CDom_CAlignDist_s1;
   logic [(sigWidth + 1):0] intermed_highAlignedSigC, intermed_highAlignedSigC_s1;
   logic [expWidth+sigWidth:0] out_s1;
   logic                 [4:0] exc_s1;
  

   mulAddRecFNToRaw_preMul#(expWidth, sigWidth)
       mulAddToRaw_preMul(
           `flControl_tininessBeforeRounding,
           i_op,
           i_a,
           i_b,
           i_c,
           i_roundingMode,
           mulAddA,
           mulAddB,
           mulAddC,
           intermed_compactState,
           intermed_sExp,
           intermed_CDom_CAlignDist,
           intermed_highAlignedSigC
       );
   wire  [sigWidth*2:0] mulAddResult = mulAddA * mulAddB + mulAddC;
   logic [sigWidth*2:0] mulAddResult_s1;

   always_ff @(posedge i_clk, negedge i_reset_n) begin
      if (~i_reset_n) begin
         valid_s1 <= '0;
      end else begin
         valid_s1 <= i_valid;
      end
   end

   always_ff @(posedge i_clk) begin
      if (i_valid) begin
         intermed_compactState_s1     <= intermed_compactState;
         intermed_sExp_s1             <= intermed_sExp;
         intermed_CDom_CAlignDist_s1  <= intermed_CDom_CAlignDist;
         intermed_highAlignedSigC_s1  <= intermed_highAlignedSigC;
         mulAddResult_s1              <= mulAddResult;
         roundingMode_s1              <= i_roundingMode;
      end
   end

   logic invalidExc, out_isNaN, out_isInf, out_isZero, out_sign;
   logic signed [(expWidth + 1):0] out_sExp;
   logic [(sigWidth + 2):0] out_sig;

   mulAddRecFNToRaw_postMul#(expWidth, sigWidth)
       mulAddToRaw_postMul(
           intermed_compactState_s1,
           intermed_sExp_s1,
           intermed_CDom_CAlignDist_s1,
           intermed_highAlignedSigC_s1,
           mulAddResult_s1,
           roundingMode_s1,
           invalidExc,
           out_isNaN,
           out_isInf,
           out_isZero,
           out_sign,
           out_sExp,
           out_sig
       );

   roundRawFNToRecFN#(expWidth, sigWidth, 0)
   roundRawOut(
            `flControl_tininessBeforeRounding,
            invalidExc,
            1'b0,
            out_isNaN,
            out_isInf,
            out_isZero,
            out_sign,
            out_sExp,
            out_sig,
            roundingMode_s1,
            out_s1,
            exc_s1
        );

   recFNToFN#(expWidth, sigWidth)
   recFNToFN(
      .in (out_s1),
      .out(o_res)
   );

   assign o_exc = {5{valid_s1}} & exc_s1;

endmodule
