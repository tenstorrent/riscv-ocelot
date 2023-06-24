// See LICENSE.TT for license details.
// Open Vector Interface wrapper module for Ocelot

module tt_vpu_ovi_assert(input logic clk, reset_n,

                  input  logic [31:0] issue_inst,
                  input  logic [4:0]  issue_sb_id,
                  input  logic [63:0] issue_scalar_opnd,
                  input  logic [39:0] issue_vcsr,
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
                  input logic        completed_illegal);
                  // TODO: Add the signals for memory operations...


killsenior_assume: assume property(@(posedge clk) disable iff (!reset_n)
            (!(dispatch_next_senior && dispatch_kill)));



endmodule

bind tt_vpu_ovi tt_vpu_ovi_assert i_assertions (.*); 
