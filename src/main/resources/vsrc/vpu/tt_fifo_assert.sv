module tt_fifo_assert #(parameter DEPTH = 4)
(
  input logic        clk,
  input logic        reset_n,
  // Inputs coming from the OVI interface
  input logic [31:0] issue_inst,
  input logic [4:0]  issue_sb_id,
  input logic [63:0] issue_scalar_opnd,
  input logic [39:0] issue_vcsr,
  input logic        issue_vcsr_lmulb2,
  input logic        issue_valid,

  // Outputs to the Ocelot VPU
  input logic        read_req,
  input logic        read_valid,
  input logic [31:0] read_issue_inst,
  input logic [4:0]  read_issue_sb_id,
  input logic [63:0] read_issue_scalar_opnd,
  input logic [39:0] read_issue_vcsr,
  input logic        read_issue_vcsr_lmulb2,

  input logic        is_empty
);


immediate_dispatch: assert property(@(posedge clk) disable iff (!reset_n)
            ((!queue_full && issue_valid && dispatch_next_senior) tt_fifo.wr_ptr == tt_fifo.dispatch_ptr));

endmodule

bind tt_fifo tt_fifo_assert fifo_assertions (.*);
