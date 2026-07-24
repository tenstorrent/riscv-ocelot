// See LICENSE.TT for license details.
module tt_vfp_red 
#(
   parameter expWidth=5,
   parameter sigWidth=11
)
(
   input  logic                            i_clk,
   input  logic                            i_reset_n,

   input  logic                            i_valid,
   input  logic                            i_en,    // 0: skip i_c, 1: add i_c
   input  logic                            i_sel,   // 0: pick i_a, 1: pick acc
   input  logic                      [1:0] i_op,    // 0: sum, 1: reserve, 2: min, 3: max
   input  logic                      [2:0] i_roundingMode,
   input  logic      [expWidth+sigWidth:0] i_a,
   input  logic      [expWidth+sigWidth:0] i_c,

   output logic    [expWidth+sigWidth-1:0] o_res,
   output logic                      [4:0] o_exc

);

`include "HardFloat_localFuncs.vi"
`include "HardFloat_consts.vi"
   
   logic [expWidth+sigWidth:0] input_data;

   logic [expWidth+sigWidth:0] add_res;
   logic                 [4:0] add_exc_raw;
   logic                 [4:0] add_exc;

   logic [expWidth+sigWidth:0] sel_res;
   logic                 [4:0] sel_exc;

   logic                       acc_valid_s1;
   logic [expWidth+sigWidth:0] acc_res_nxt;
   logic [expWidth+sigWidth:0] acc_res_s1;
   logic                 [4:0] acc_exc_s1;

   assign input_data = i_sel ? acc_res_s1 : i_a;

   /////////////
   // Min/Max //
   /////////////
   logic                cls_a_sign,  cls_c_sign;
   logic                cls_a_isNaN, cls_c_isNaN;

   recFNToRawFN #(.expWidth(expWidth),
                  .sigWidth(sigWidth) )
   classify_a
   (
      .in    (input_data),
      .isNaN (cls_a_isNaN),
      .isInf (),
      .isZero(),
      .sign  (cls_a_sign),
      .sExp  (),
      .sig   ()
   );

   recFNToRawFN #(.expWidth(expWidth),
                  .sigWidth(sigWidth) )
   classify_c
   (
      .in    (i_c),
      .isNaN (cls_c_isNaN),
      .isInf (),
      .isZero(),
      .sign  (cls_c_sign),
      .sExp  (),
      .sig   ()
   );

   logic       sel_a_isSigNaN;
   logic       sel_c_isSigNaN;
   logic       sel_a_gt_c;

   isSigNaNRecFN #(.expWidth(expWidth),
                   .sigWidth(sigWidth) )
   isSigNaN_a
   (
      .in      (input_data),
      .isSigNaN(sel_a_isSigNaN)
   );

   isSigNaNRecFN #(.expWidth(expWidth),
                   .sigWidth(sigWidth) )
   isSigNaN_c
   (
      .in      (i_c),
      .isSigNaN(sel_c_isSigNaN)
   );

   assign sel_a_gt_c = (cls_a_sign  && cls_c_sign ) ? input_data < i_c  :
                       (cls_a_sign  ^  cls_c_sign ) ? cls_c_sign        :
                                                      input_data > i_c;

   assign sel_res    = (cls_a_isNaN && cls_c_isNaN) ? {1'b0, 3'b111, (expWidth-2)'(0), 1'b1, (sigWidth-2)'(0)} : // Canonical NaN
                       (cls_a_isNaN               ) ? i_c                                                      :
                       (               cls_c_isNaN) ? input_data                                               :
                       (i_op==2'h3  && !sel_a_gt_c) ? i_c                                                      :
                       (i_op==2'h2  &&  sel_a_gt_c) ? i_c                                                      :
                                                      input_data;
                          
   assign sel_exc    = {5{i_valid && i_en}} & {sel_a_isSigNaN || sel_c_isSigNaN, 4'h0};

   ///////////
   // Adder //
   ///////////
   addRecFN #(.expWidth(expWidth),
              .sigWidth(sigWidth) )
   addRecFN
   (
      .control       (`flControl_tininessBeforeRounding),
      .subOp         (1'b0),
      .a             (i_c),
      .b             (input_data),
      .roundingMode  (i_roundingMode),
      .out           (add_res),
      .exceptionFlags(add_exc_raw)
   );

   assign add_exc = {5{i_valid && i_en}} & add_exc_raw;

   assign acc_res_nxt = (i_op == 2'h0) ? add_res : sel_res;
   assign acc_exc_nxt = (i_op == 2'h0) ? add_exc : sel_exc;

   always_ff @(posedge i_clk, negedge i_reset_n) begin
      if (~i_reset_n) begin
         acc_valid_s1 <= '0;
      end else begin
         acc_valid_s1 <= i_valid;
      end
   end

   always_ff @(posedge i_clk) begin
      if (i_valid) begin
         if (i_en) begin
            acc_res_s1 <= acc_res_nxt;
         end else
         // Latch i_a if the first element's mask is 0
         if (!i_sel) begin
            acc_res_s1 <= i_a;
         end
      end

      acc_exc_s1 <= acc_exc_nxt;
   end

   recFNToFN #(.expWidth(expWidth),
               .sigWidth(sigWidth))
   recFNToFN
   (
      .in (acc_res_s1),
      .out(o_res)
   );

   assign o_exc = acc_exc_s1;

endmodule

