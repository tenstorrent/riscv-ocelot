// See LICENSE.TT for license details.
// Open Vector Interface wrapper module for Ocelot

module tt_vpu_ovi_assert
(                 input  logic clk, reset_n,

                  input  logic [31:0] issue_inst,
                  input  logic [4:0]  issue_sb_id,
                  input  logic [63:0] issue_scalar_opnd,
                  input  logic [39:0] issue_vcsr,
                  input  logic        issue_vcsr_lmulb2, // Added 1 more bit for vlmul
                  input  logic        issue_valid,
                  input  logic        issue_credit,

                  input  logic [4:0]  dispatch_sb_id,
                  input  logic        dispatch_next_senior,
                  input  logic        dispatch_kill,

                  input  logic        completed_valid,
                  input  logic [4:0]  completed_sb_id,
                  input  logic [4:0]  completed_fflags,
                  input  logic [63:0] completed_dest_reg,
                  input  logic        completed_vxsat,
                  input  logic [13:0] completed_vstart,
                  input  logic        completed_illegal,

                  input  logic         store_valid,
                  input  logic [511:0] store_data,
                  input  logic         store_credit,

                  input  logic [33:0]  load_seq_id,
                  input  logic [511:0] load_data,
                  input  logic         load_valid,
                  input  logic [63:0]  load_mask,
                  input  logic         load_mask_valid,

                  input  logic         memop_sync_end,
                  input  logic         memop_sync_start

);
                  // TODO: Add the signals for memory operations...

  // When we get an issue.valid, we expect a dispatch.next_senior or dispatch.kill with the same sb_id
  // before another issue.valid comes in with the same sb_id, or a completed.valid signal with the same sb_id
  logic [31:0] issue_sb_ids;
  logic [31:0] dispatch_sb_ids;
  always_ff @( posedge clk ) begin
    if (~reset_n) begin
      issue_sb_ids <= '0;
      dispatch_sb_ids <= '0;
    end else begin
      if(issue_valid) begin
        issue_sb_ids[issue_sb_id] <= 1;
      end
  
      if(dispatch_next_senior) begin
        dispatch_sb_ids[dispatch_sb_id] <= 1;
      end
  
      if(dispatch_kill) begin
        dispatch_sb_ids[dispatch_sb_id] <= 0;
      end
  
      if(completed_valid) begin
        issue_sb_ids   [completed_sb_id] <= 0;
        dispatch_sb_ids[completed_sb_id] <= 0;
      end
    end
  end
  
  dispatch_no_early_than_issue_assume: assume property(@(posedge clk) disable iff (!reset_n)
              ((dispatch_next_senior || dispatch_kill) |-> (issue_valid && issue_sb_id == dispatch_sb_id) || issue_sb_ids[dispatch_sb_id]));
  
  cannot_issue_dispatch_twice_assume: assume property(@(posedge clk) disable iff (!reset_n)
              (~(issue_sb_ids[issue_sb_id] && dispatch_sb_ids[dispatch_sb_id])));
  
  // complete bus must have a different sb_id than issue/dispatch in a given cycle.
  // issue and dispatch may have the same sb_id.
  issue_complete_sbid_assume: assume property(@(posedge clk) disable iff (!reset_n)
              (completed_valid |-> issue_sb_ids[completed_sb_id] && dispatch_sb_ids[completed_sb_id]));
  
  // dispatch.next_senior and dispatch.kill cannot be high at the same time
  kill_senior_assume: assume property(@(posedge clk) disable iff (!reset_n)
              (!(dispatch_next_senior && dispatch_kill)));
  
  logic [5:0] pending_mem_cnt, pending_mem_cnt_nxt;

  always_ff @(posedge clk) begin
    if (~reset_n) begin
      pending_mem_cnt <= '0;
    end else begin
      pending_mem_cnt <= pending_mem_cnt_nxt;
    end
  end 

  assign pending_mem_cnt_nxt = pending_mem_cnt + memop_sync_start - memop_sync_end;

  no_pending_memop_assert: assert property(@(posedge clk) disable iff (!reset_n)
              s_eventually pending_mem_cnt == 0);

  pending_mem_cnt_underflow_assert: assert property(@(posedge clk) disable iff (!reset_n)
              pending_mem_cnt == 0 |-> ~(memop_sync_end && !memop_sync_start));
  
endmodule

bind tt_vpu_ovi tt_vpu_ovi_assert i_assertions (.*); 
