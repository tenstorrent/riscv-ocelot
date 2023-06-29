// See LICENSE.TT for license details.
// Open Vector Interface wrapper module for Ocelot

module tt_vpu_ovi #(parameter VLEN = 256)
(                 input  logic clk, reset_n,

                  input  logic [31:0] issue_inst,
                  input  logic [4:0]  issue_sb_id,
                  input  logic [63:0] issue_scalar_opnd,
                  input  logic [39:0] issue_vcsr,
                  input  logic        issue_vcsr_lmulb2, // Added 1 more bit for vlmul
                  input  logic        issue_valid,
                  output logic        issue_credit,

                  input  logic [4:0]  dispatch_sb_id,
                  input  logic        dispatch_next_senior,
                  input  logic        dispatch_kill,

                  output logic        completed_valid,
                  output logic [4:0]  completed_sb_id,
                  output logic [4:0]  completed_fflags,
                  output logic [63:0] completed_dest_reg,
                  output logic        completed_vxsat,
                  output logic [13:0] completed_vstart,
                  output logic        completed_illegal,

                  // Debug signals for cosim checker
                  output logic              debug_wb_vec_valid,
                  output logic [VLEN*8-1:0] debug_wb_vec_wdata,
                  output logic [7:0]        debug_wb_vec_wmask
);
                  // TODO: Add the signals for memory operations...

  localparam LOCAL_MEM_BYTE_ADDR_WIDTH = 12;
  localparam INCL_VEC = 1;
//  localparam VLEN = 256;
  localparam ADDRWIDTH = 48;
  localparam LD_DATA_WIDTH_BITS = VLEN;
  localparam ST_DATA_WIDTH_BITS = VLEN;
  localparam LQ_DEPTH=8;
  localparam LQ_DEPTH_LOG2=$clog2(LQ_DEPTH);
  localparam DATA_REQ_ID_WIDTH=INCL_VEC ? (LQ_DEPTH_LOG2+$clog2(VLEN/8)+2) : LQ_DEPTH_LOG2;
  localparam INCL_FP = 1;

  logic ocelot_read_req;

  logic        read_valid;
  logic [31:0] read_issue_inst;
  logic [4:0]  read_issue_sb_id;
  logic [63:0] read_issue_scalar_opnd;
  logic [39:0] read_issue_vcsr;
  logic        read_issue_vcsr_lmulb2;

  // I'm using dummy wires to connect the memory interface for now
  logic o_data_req;
  logic [ADDRWIDTH-1:0] o_data_addr;
  logic [ST_DATA_WIDTH_BITS/8-1:0] o_data_byten;
  logic [ST_DATA_WIDTH_BITS-1:0] o_wr_data;
  logic [DATA_REQ_ID_WIDTH-1:0] o_data_req_id;
  logic o_mem_load;
  logic [2:0] o_mem_size;
  logic o_mem_last;

  logic [VLEN*8-1:0] ocelot_instrn_commit_data;
  logic [7:0] ocelot_instrn_commit_mask;
  logic [4:0] ocelot_instrn_commit_fflags;
  logic       ocelot_sat_csr;
  logic       ocelot_instrn_commit_valid;

  // Hack to keep the vector CSRs constant for ocelot
  logic [39:0] vcsr_reg;
  logic        vcsr_lmulb2_reg;
  logic [39:0] vcsr;
  logic        vcsr_lmulb2;

  // This queue should never overflow, so I'm not going to add phase bits
  // to check if it's full or not.
  localparam sbid_queue_depth = 8;
  logic [sbid_queue_depth-1:0][4:0] sbid_queue;
  logic [$clog2(sbid_queue_depth)-1:0] sbid_queue_wptr;
  logic [$clog2(sbid_queue_depth)-1:0] sbid_queue_rptr;

  always_ff@(posedge clk) begin
    if(!reset_n) begin
      for(int i=0; i<sbid_queue_depth; i=i+1)
        sbid_queue[i] <= 0;
      sbid_queue_wptr <= 0;
      sbid_queue_rptr <= 0;
    end
    else begin
      if(ocelot_read_req && read_valid) begin
        sbid_queue[sbid_queue_wptr] <= read_issue_sb_id;
        sbid_queue_wptr <= sbid_queue_wptr + 1;
      end
      if(ocelot_instrn_commit_valid) begin
        sbid_queue_rptr <= sbid_queue_rptr + 1;
      end
    end
  end

  assign issue_credit = 0;

  tt_fifo #(
    .DEPTH(16)
  ) fifo0
  (
    .clk(clk),
    .reset_n(reset_n),

    .issue_inst(issue_inst),
    .issue_sb_id(issue_sb_id),
    .issue_scalar_opnd(issue_scalar_opnd),
    .issue_vcsr(issue_vcsr),
    .issue_vcsr_lmulb2(issue_vcsr_lmulb2),
    .issue_valid(issue_valid),

    .dispatch_sb_id(dispatch_sb_id),
    .dispatch_next_senior(dispatch_next_senior),
    .dispatch_kill(dispatch_kill),

    .read_req(ocelot_read_req),
    .read_valid(read_valid),
    .read_issue_inst(read_issue_inst),
    .read_issue_sb_id(read_issue_sb_id),
    .read_issue_scalar_opnd(read_issue_scalar_opnd),
    .read_issue_vcsr(read_issue_vcsr),
    .read_issue_vcsr_lmulb2(read_issue_vcsr_lmulb2)
  );

  assign vcsr        = ocelot_read_req && read_valid ? read_issue_vcsr : vcsr_reg;
  assign vcsr_lmulb2 = ocelot_read_req && read_valid ? read_issue_vcsr_lmulb2 : vcsr_lmulb2_reg;

  always_ff@(posedge clk) begin
    if(!reset_n)
      {vcsr_reg, vcsr_lmulb2_reg} <= 0;
    else if(read_valid && ocelot_read_req) begin
      vcsr_reg <= read_issue_vcsr;
      vcsr_lmulb2_reg <= read_issue_vcsr_lmulb2;
    end
  end

  // Just pass the parameters down...
  vfp_pipeline #(
    .LOCAL_MEM_BYTE_ADDR_WIDTH(LOCAL_MEM_BYTE_ADDR_WIDTH),
    .INCL_VEC(INCL_VEC),
    .VLEN(VLEN),
    .ADDRWIDTH(ADDRWIDTH),
    .LD_DATA_WIDTH_BITS(LD_DATA_WIDTH_BITS),
    .ST_DATA_WIDTH_BITS(ST_DATA_WIDTH_BITS),
    .LQ_DEPTH(LQ_DEPTH),
    .LQ_DEPTH_LOG2(LQ_DEPTH_LOG2),
    .DATA_REQ_ID_WIDTH(DATA_REQ_ID_WIDTH),
    .INCL_FP(INCL_FP)
  ) ocelot
  (
    .i_clk(clk),
    .i_reset(!reset_n),
    .i_csr_vl(vcsr[$clog2(VLEN+1)-1+14:14]),
    .i_csr_vsew(vcsr[38:36]),
    .i_csr_vlmul({vcsr_lmulb2,vcsr[35:34]}),
    .i_csr_vxrm(vcsr[30:29]),
    .i_csr_frm(vcsr[33:31]),

    // IF Interface
    .i_if_instrn_rts(read_valid),
    .o_id_instrn_rtr(ocelot_read_req),
    .i_if_instrn(read_issue_inst),
    .i_if_pc(0),
    .i_rf_vex_p0(read_issue_scalar_opnd),
    .i_rf_vex_p1(read_issue_scalar_opnd),
    .i_fprf_vex_p0(read_issue_scalar_opnd),

    // Commit Interface
    .o_instrn_commit_valid(ocelot_instrn_commit_valid),
    .o_instrn_commit_data(ocelot_instrn_commit_data),
    .o_instrn_commit_mask(ocelot_instrn_commit_mask),
    .o_instrn_commit_fflags(ocelot_instrn_commit_fflags),

    .o_sat_csr(ocelot_sat_csr),

    // Memory Interface
    .o_data_req(o_data_req),
    .o_data_addr(o_data_addr),
    .o_data_byten(o_data_byten),
    .o_wr_data(o_wr_data),
    .o_data_req_id(o_data_req_id),
    .o_mem_load(o_mem_load),
    .o_mem_size(o_mem_size),
    .o_mem_last(o_mem_last),
    .i_data_req_rtr('1),
    .i_rd_data_vld_0('0),
    .i_rd_data_resp_id_0('0),
    .i_rd_data_0('0),
    .i_rd_data_vld_1('0),
    .i_rd_data_resp_id_1('0),
    .i_rd_data_1('0)
  );

  assign completed_valid = ocelot_instrn_commit_valid;
  assign completed_sb_id = sbid_queue[sbid_queue_rptr];
  assign completed_fflags = ocelot_instrn_commit_fflags;
  assign completed_dest_reg = ocelot_instrn_commit_data[63:0];
  assign completed_vxsat = 0;
  assign completed_vstart = 0;
  assign completed_illegal = 0;

  assign debug_wb_vec_valid = ocelot_instrn_commit_valid;
  assign debug_wb_vec_wdata = ocelot_instrn_commit_data;
  assign debug_wb_vec_wmask = ocelot_instrn_commit_mask;

endmodule
