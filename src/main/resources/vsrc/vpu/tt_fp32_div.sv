// See LICENSE.TT for license details.
// 32-bit IEEE 754 Floating-Point Division Unit
// Uses Berkeley HardFloat for internal computation

module tt_fp32_div (
    input  logic        i_clk,
    input  logic        i_reset_n,
    
    // Handshake interface
    input  logic        i_vld,
    output logic        o_ack,
    output logic        o_rts,
    input  logic        i_rtr,
    
    // Operands (IEEE FP32 format)
    input  logic [31:0] i_a,        // Dividend (vs2)
    input  logic [31:0] i_b,        // Divisor (vs1)
    input  logic [2:0]  i_rm,       // Rounding mode
    
    // Results (IEEE FP32 format)
    output logic [31:0] o_result,   // Division result
    output logic [4:0]  o_exc       // Exception flags
);

    // HardFloat parameters for FP32
    localparam EXP_WIDTH = 8;
    localparam SIG_WIDTH = 24;  // 23 + 1 implicit bit
    
    // Convert IEEE FP32 to recoded format
    logic [32:0] a_recoded, b_recoded;  // 33 bits for recoded FP32
    
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
    logic [32:0] div_result_recoded;
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
        .inValid(i_vld),
        .sqrtOp(1'b0),              // Division operation
        .a(a_recoded),              // Dividend (recoded)
        .b(b_recoded),              // Divisor (recoded)
        .roundingMode(i_rm),
        
        // Output handshake  
        .outValid(div_outValid),
        .sqrtOpOut(div_sqrtOpOut),
        .out(div_result_recoded),
        .exceptionFlags(div_exceptionFlags)
    );
    
    // Convert recoded result back to IEEE FP32
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