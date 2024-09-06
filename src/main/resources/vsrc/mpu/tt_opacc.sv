// See LICENSE.TT for license details.
module tt_opacc #(parameter
    vl=4,
    ml=4,
    NUM_MREGS=2,
    MREG_ADDR_WIDTH=$clog2(NUM_MREGS),
    ROW_ADDR_WIDTH=$clog2(ml),
    XLEN=64)(
    input i_clk,
    input i_reset_n,
    input i_vab_valid,
    input i_c_valid,
    input [MREG_ADDR_WIDTH+ROW_ADDR_WIDTH-1:0] i_rdaddr,
    input [MREG_ADDR_WIDTH+ROW_ADDR_WIDTH-1:0] i_wraddr,
    input [ml-1:0][XLEN-1:0] i_a,
    input [vl-1:0][XLEN-1:0] i_b,
    input [vl-1:0][XLEN-1:0] i_c,
    output [vl-1:0][XLEN-1:0] o_c
    );
    integer i, j;
    // localparam LMUL_MAX = 8;
    logic [NUM_MREGS*ml-1:0][vl-1:0][XLEN-1:0] c_mreg;
    logic [8-1:0][vl-1:0][XLEN-1:0] c_opacc_ab;
    // Ensure i_wraddr is properly initialized and does not contain 'x' values
    // Extract the upper part of i_wraddr
    wire [MREG_ADDR_WIDTH-1:0] upper_part = i_wraddr[MREG_ADDR_WIDTH+ROW_ADDR_WIDTH-1 -: MREG_ADDR_WIDTH];
    // Create the lower part with zeros
    wire [ROW_ADDR_WIDTH-1:0] lower_part = {ROW_ADDR_WIDTH{1'b0}};
    // Combine the parts
    wire [MREG_ADDR_WIDTH+ROW_ADDR_WIDTH-1:0] opacc_wraddr = {upper_part, lower_part};
    // wire [MREG_ADDR_WIDTH+ROW_ADDR_WIDTH-1:0] opacc_wraddr = {i_wraddr[MREG_ADDR_WIDTH+ROW_ADDR_WIDTH-1 -: MREG_ADDR_WIDTH], {ROW_ADDR_WIDTH{1'b0}}};
    always @(posedge i_clk) begin 
        if (~i_reset_n) begin
            c_mreg <= 0;
        end
        else begin
            if (i_c_valid) begin
                c_mreg[i_wraddr] <= i_c;
            end
            if (i_vab_valid) begin        
                c_mreg[opacc_wraddr+ml-1 -: ml]  <= c_opacc_ab[ml-1:0];
                // for(i=0; i<ml; i++) begin
                //     c_mreg[opacc_wraddr + i]  <= c_opacc_ab[ml-1:0];;
                // end
            end
        end
    end

    always @* begin
        for(i=0; i<ml; i++) begin
            for(j=0; j<vl; j++) begin
                c_opacc_ab[i][j] =  c_mreg[opacc_wraddr + i][j] 
                                    + i_a[i]*i_b[j];
            end
        end
    end

    assign o_c = c_mreg[i_rdaddr];

endmodule

