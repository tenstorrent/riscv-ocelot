// See LICENSE.TT for license details.
`include "autogen_defines.h"
`include "briscv_defines.h"
`include "tt_briscv_pkg.vh"
module tt_matrix_unit #(parameter
  LQ_DEPTH_LOG2=3,
  VLEN=256,
  MLEN=256,
  XLEN=64,
  vl = VLEN/XLEN,
  ml = MLEN/XLEN,
  NUM_MREGS=2,
  MREG_ADDR_WIDTH = $clog2(NUM_MREGS),
  ROW_ADDR_WIDTH = $clog2(ml)
  )(
  input i_clk,
  input i_reset_n,

  input i_id_matrix_rts,
  input i_wr_c_0a,
  input i_opacc_0a,
  input [LQ_DEPTH_LOG2-1:0] i_mvex_lq_id_0c,

  input [MREG_ADDR_WIDTH+ROW_ADDR_WIDTH-1:0]  i_rdaddr_0a,
  input [MREG_ADDR_WIDTH+ROW_ADDR_WIDTH-1:0]  i_wraddr_c_0a,
  input [MREG_ADDR_WIDTH+ROW_ADDR_WIDTH-1:0]  i_wraddr_opacc_0a,

  input [MLEN-1:0]  i_vrf_p0_rddata,
  input [VLEN-1:0]  i_vrf_p1_rddata,
  // input [VLEN-1:0]  i_vrf_p2_rddata,
  input [VLEN-1:0]  i_wrdata_0a,
      
  output logic o_mvex_lq_vld_1c,
  output logic [VLEN-1:0] o_mvex_lq_data_1c,
  output logic [LQ_DEPTH_LOG2-1:0] o_mvex_lq_id_1c
);
  wire matrix_en_0a = i_id_matrix_rts;
  wire ab_valid_0c = matrix_en_0a & i_opacc_0a;
  wire c_valid_0c = i_wr_c_0a;

  logic [ml-1:0][XLEN-1:0] au;
  logic [vl-1:0][XLEN-1:0] bu;
  logic [vl-1:0][XLEN-1:0] cu;
  logic [vl-1:0][XLEN-1:0] o_c;
  logic [VLEN-1:0] mvex_lq_data_0c;

  always @(posedge i_clk) begin
    if (~i_reset_n) begin
      o_mvex_lq_id_1c <= 0;
      o_mvex_lq_vld_1c <= 0;
      o_mvex_lq_data_1c <= 0;
    end
    else begin
      o_mvex_lq_vld_1c <= matrix_en_0a;
      if(matrix_en_0a) begin
        o_mvex_lq_id_1c <= i_mvex_lq_id_0c;
        o_mvex_lq_data_1c <= mvex_lq_data_0c;
      end
    end
  end

  always @* begin
    for(int i=1; i<ml+1; i++) begin
      au[i-1]   = i_vrf_p0_rddata[i*XLEN-1 -: XLEN];
    end
    
    for(int i=1; i<vl+1; i++) begin
      bu[i-1] = i_vrf_p1_rddata[i*XLEN-1 -: XLEN];
      cu[i-1] = i_wrdata_0a[i*XLEN-1 -: XLEN];
      mvex_lq_data_0c[i*XLEN-1 -: XLEN] = o_c[i-1];
    end
  end
  
  logic [NUM_MREGS*ml-1:0][vl-1:0][XLEN-1:0] c_mreg;
  logic [8-1:0][vl-1:0][XLEN-1:0] c_opacc_ab;
  integer i, j;
  always @(posedge i_clk) begin 
      if (~i_reset_n) begin
          c_mreg <= 0;
      end
      else begin
          if (c_valid_0c) begin
              c_mreg[i_wraddr_c_0a] <= cu;
          end
          if (ab_valid_0c) begin        
              c_mreg[i_wraddr_opacc_0a+ml-1 -: ml]  <= c_opacc_ab[ml-1:0];
              // for(i=0; i<ml; i++) begin
              //     c_mreg[i_wraddr_opacc_0a + i]  <= c_opacc_ab[ml-1:0];;
              // end
          end
      end
  end

  always @* begin
      for(i=0; i<vl; i++) begin
          for(j=0; j<ml; j++) begin
              c_opacc_ab[i][j] =  c_mreg[i][i_wraddr_opacc_0a + j] 
                                  + au[j]*bu[i];
          end
      end
  end

  assign o_c = c_mreg[i_rdaddr_0a];

endmodule

