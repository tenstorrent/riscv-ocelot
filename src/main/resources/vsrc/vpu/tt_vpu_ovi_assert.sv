// See LICENSE.TT for license details.
// Open Vector Interface wrapper module for Ocelot

module tt_vpu_ovi_assert(input logic clk, reset_n,

                  input  logic [31:0] issue_inst,
                  input  logic [4:0]  issue_sb_id,
                  input  logic [63:0] issue_scalar_opnd,
                  input  logic [39:0] issue_vcsr,
                  input  logic        issue_vcsr_lmulb2, // Added 1 more bit for vlmul
                  input  logic        issue_valid,
                  input logic        issue_credit,

                  input  logic [4:0]  dispatch_sb_id,
                  input  logic        dispatch_next_senior,
                  input  logic        dispatch_kill,

                  input logic        completed_valid,
                  input logic [4:0]  completed_sb_id,
                  input logic [4:0]  completed_fflags,
                  input logic [63:0] completed_dest_reg,
                  input logic        completed_vxsat,
                  input logic [13:0] completed_vstart,
                  input logic        completed_illegal,

                  input logic         store_valid,
                  input logic [511:0] store_data,
                  input logic         store_credit,

                  input logic         memop_sync_end,
                  input logic         memop_sync_start,

                  // Debug signals for cosim checker
                  input logic              debug_wb_vec_valid,
                  input logic [VLEN*8-1:0] debug_wb_vec_wdata,
                  input logic [7:0]        debug_wb_vec_wmask);
                  // TODO: Add the signals for memory operations...

// When we get an issue.valid, we expect a dispatch.next_senior or dispatch.kill with the same sb_id
// before another issue.valid comes in with the same sb_id, or a completed.valid signal with the same sb_id
logic [31:0] issue_sb_ids;
logic [31:0] dispatch_sb_ids;
always_ff @( posedge clk ) begin
  if(issue.valid) begin
    sb_ids[issue.sb_id] <= 1;
    if(dispatch.valid)
      dispatch_sb_ids[dispatch_sb_id] <= 1;
    if(completed.valid) begin
      assert(completed.sb_id != issue.sb_id) else $error("issue and complete buses cannot have the same sb_id in the same cycle.")
      sb_ids[completed.sb_id] <= 0;
    end
  end
  if(completed.valid) begin
    assert(completed.sb_id != issue.sb_id) else $error("issue and complete buses cannot have the same sb_id in the same cycle.")
    sb_ids[completed.sb_id] <= 0;
  end
end
cannot_issue_dispatch_twice_assume: assume property(@(posedge clk) disable iff (!reset_n)
            (!issue_sb_ids[issue.sb_id] && !dispatch_sb_ids[dispatch_sb_id]));

// complete bus must have a different sb_id than issue/dispatch in a given cycle.
// issue and dispatch may have the same sb_id.
issue_complete_sbid_assume: assume property(@(posedge clk) disable iff (!reset_n)
            (issue.sb_id != completed_valid && dispatch_sb_id != completed_valid));

// dispatch.next_senior and dispatch.kill cannot be high at the same time
kill_senior_assume: assume property(@(posedge clk) disable iff (!reset_n)
            (!(dispatch_next_senior && dispatch_kill)));




endmodule

bind tt_vpu_ovi tt_vpu_ovi_assert i_assertions (.*); 
