module Vpu
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
                  output logic        completed_illegal);
                  // TODO: Add the signals for memory operations...
                  assign issue_credit = 1'b1;
                  assign completed_valid = issue_valid;
endmodule