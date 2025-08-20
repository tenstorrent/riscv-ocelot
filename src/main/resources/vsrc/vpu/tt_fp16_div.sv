// See LICENSE.TT for license details.
// 16-bit IEEE 754 Floating-Point Division/Square Root Unit  
// Uses Berkeley HardFloat for internal computation

module tt_fp16_div (
    input  logic        i_clk,
    input  logic        i_reset_n,
    
    // Handshake interface
    input  logic        i_vld_div,  // Valid signal for division
    input  logic        i_vld_sqrt, // Valid signal for square root
    output logic        o_ack,
    output logic        o_rts,
    input  logic        i_rtr,
    
    // Operands (IEEE FP16 format)
    input  logic [15:0] i_a,        // Dividend/Radicand (vs2)
    input  logic [15:0] i_b,        // Divisor (vs1, ignored for sqrt)  
    input  logic [2:0]  i_rm,       // Rounding mode
    
    // Results (IEEE FP16 format)
    output logic [15:0] o_result,   // Division/Square root result
    output logic [4:0]  o_exc       // Exception flags
);

    // HardFloat parameters for FP16
    localparam EXP_WIDTH = 5;
    localparam SIG_WIDTH = 11;  // 10 + 1 implicit bit
    
    // Operation control logic
    logic i_vld_combined, sqrt_op;
    assign i_vld_combined = i_vld_div | i_vld_sqrt;
    assign sqrt_op = i_vld_sqrt;  // sqrt operation when i_vld_sqrt is active
    
    // Convert IEEE FP16 to recoded format
    logic [16:0] a_recoded, b_recoded;  // 17 bits for recoded FP16
    
    fNToRecFN #(
        .expWidth(EXP_WIDTH),
        .sigWidth(SIG_WIDTH)
    ) fNToRecFN_a (
        .in(i_a),
        .out(a_recoded)
    );
    
    fNToRecFN #(
        .expWidth(EXP_WIDTH),
        .sigWidth(SIG_WIDTH)
    ) fNToRecFN_b (
        .in(i_b),
        .out(b_recoded)
    );
    
    // HardFloat division unit (recoded format)
    logic div_outValid;
    logic div_sqrtOpOut;
    logic [16:0] div_result_recoded;
    logic [4:0] div_exceptionFlags;
    
    divSqrtRecFN_small #(
        .expWidth(EXP_WIDTH),
        .sigWidth(SIG_WIDTH),
        .options(0)
    ) div_unit (
        .nReset(i_reset_n),
        .clock(i_clk),
        .control(`flControl_default),
        
        // Input handshake
        .inReady(o_ack),
        .inValid(i_vld_combined),
        .sqrtOp(sqrt_op),           // sqrt_op = 1 for sqrt, 0 for division
        .a(a_recoded),              // Dividend (recoded)
        .b(b_recoded),              // Divisor (recoded)
        .roundingMode(i_rm),
        
        // Output handshake
        .outValid(div_outValid),
        .sqrtOpOut(div_sqrtOpOut),
        .out(div_result_recoded),
        .exceptionFlags(div_exceptionFlags)
    );
    
    // Convert recoded result back to IEEE FP16
    recFNToFN #(
        .expWidth(EXP_WIDTH),
        .sigWidth(SIG_WIDTH)
    ) recFNToFN_result (
        .in(div_result_recoded),
        .out(o_result)
    );
    
    // Output assignments
    assign o_rts = div_outValid;
    assign o_exc = div_exceptionFlags;

endmodule