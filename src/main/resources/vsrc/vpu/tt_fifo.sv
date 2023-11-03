// See LICENSE.TT for license details.
`include "tt_briscv_pkg.vh"

module tt_fifo #(parameter DEPTH = 4)
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

  input logic [4:0]  dispatch_sb_id,
  input logic        dispatch_next_senior,
  input logic        dispatch_kill,

  // Initiate memory sync when resources are ready
  output logic        memop_sync_start,
  output logic [4:0]  memop_sync_start_sb_id,

  // Load Data Buffer allocation interface
  output logic        ldb_alloc_valid,
  input  logic        ldb_alloc_ack,
  output logic [4:0]  ldb_alloc_sb_id,
  output logic [3:0]  ldb_alloc_size,

  // Outputs to the Ocelot VPU
  input  logic        read_req,
  output logic        read_valid,
  output logic [31:0] read_issue_inst,
  output logic [4:0]  read_issue_sb_id,
  output logic [63:0] read_issue_scalar_opnd,
  output logic [39:0] read_issue_vcsr,
  output logic        read_issue_vcsr_lmulb2
);

  typedef struct packed {
    logic [31:0] issue_inst;
    logic [4:0]  issue_sb_id;
    logic [63:0] issue_scalar_opnd;
    logic [39:0] issue_vcsr;
    logic        issue_vcsr_lmulb2;
    logic        next_senior;
    logic        pending_mem_sync;
    logic        ldb_allocated;
    logic        valid;
  } fifo_entry;

  fifo_entry fifo [DEPTH-1:0];
  fifo_entry ls_candidate;
  localparam PTR_SIZE = $clog2(DEPTH);
  logic [PTR_SIZE:0] wr_ptr;
  logic [PTR_SIZE:0] rd_ptr;
  logic [PTR_SIZE:0] ls_ptr; // pointing to the oldest entry that needs to send memop_sync_start
  // keeps track of the next instruction to be dispatched
  // either going to kill or make it next_senior
  logic [$clog2(DEPTH)-1:0] dispatch_ptr;
  logic queue_full;
  logic queue_empty;
  // This 1-bit register saves the last dispatch command
  // 0: next_senior, 1: kill
  logic last_dispatch_was_kill;

  assign queue_full = (wr_ptr[PTR_SIZE] != rd_ptr[PTR_SIZE]) && (wr_ptr[PTR_SIZE-1:0] == rd_ptr[PTR_SIZE-1:0]);
  assign queue_empty = (wr_ptr[PTR_SIZE] == rd_ptr[PTR_SIZE]) && (wr_ptr[PTR_SIZE-1:0] == rd_ptr[PTR_SIZE-1:0]);

  always_ff @(posedge clk) begin
    if (!reset_n) begin
      wr_ptr <= 0;
      rd_ptr <= 0;
      ls_ptr <= 0;
      dispatch_ptr <= 0;
      last_dispatch_was_kill <= 0;
      for (int i = 0; i < DEPTH; i = i + 1) begin
        fifo[i].issue_inst       <= 0;
        fifo[i].issue_sb_id      <= 0;
        fifo[i].issue_scalar_opnd <= 0;
        fifo[i].issue_vcsr       <= 0;
        fifo[i].issue_vcsr_lmulb2 <= 0;
        fifo[i].next_senior <= 0;
        fifo[i].pending_mem_sync <= 0;
        fifo[i].ldb_allocated <= 0;
        fifo[i].valid <= 0;
      end
    end
    else if (!queue_full && issue_valid) begin
      fifo[wr_ptr[PTR_SIZE-1:0]].issue_inst       <= issue_inst;
      fifo[wr_ptr[PTR_SIZE-1:0]].issue_sb_id      <= issue_sb_id;
      fifo[wr_ptr[PTR_SIZE-1:0]].issue_scalar_opnd <= issue_scalar_opnd;
      fifo[wr_ptr[PTR_SIZE-1:0]].issue_vcsr       <= issue_vcsr;
      fifo[wr_ptr[PTR_SIZE-1:0]].issue_vcsr_lmulb2 <= issue_vcsr_lmulb2;
      fifo[wr_ptr[PTR_SIZE-1:0]].valid            <= 1;
      fifo[wr_ptr[PTR_SIZE-1:0]].pending_mem_sync <= issue_inst[6:0] inside {7'h7, 7'h27}; // set for vector load/store
      fifo[wr_ptr[PTR_SIZE-1:0]].ldb_allocated    <= 1'b0;
      // dispatch.next_senior can be sent in the same cycle as the instruction
      if(dispatch_next_senior) begin
        // TODO: assert that wr_ptr == dispatch_ptr
        fifo[wr_ptr[PTR_SIZE-1:0]].next_senior <= 1;
        dispatch_ptr <= dispatch_ptr + 1;
        last_dispatch_was_kill <= 0;
      end
      else
        fifo[wr_ptr[PTR_SIZE-1:0]].next_senior <= 0;

      wr_ptr <= wr_ptr + 1;
    end
    else begin
      if(dispatch_next_senior) begin
        // TODO: assert that dispatch.sb_id == fifo[dispatch_ptr].sb_id
        fifo[dispatch_ptr].next_senior <= 1;
        last_dispatch_was_kill <= 0;
        dispatch_ptr <= dispatch_ptr + 1;
      end
      else if(dispatch_kill && !last_dispatch_was_kill) begin
        wr_ptr <= dispatch_ptr;
        last_dispatch_was_kill <= 1;
      end
    end

    if (read_req && read_valid) begin
      rd_ptr <= rd_ptr + 1;
      fifo[rd_ptr[PTR_SIZE-1:0]].valid <= 0;
    end

    // Clear the pending bit
    if (memop_sync_start) begin
      fifo[ls_ptr[PTR_SIZE-1:0]].pending_mem_sync <= 1'b0;
    end

    if (ldb_alloc_valid && ldb_alloc_ack) begin
      fifo[ls_ptr[PTR_SIZE-1:0]].ldb_allocated <= 1'b1;
    end

    if ((  memop_sync_start                  ||
         ( ls_candidate.valid            &&
          !ls_candidate.pending_mem_sync   )   ) &&
           ls_ptr != wr_ptr                        ) begin
      ls_ptr <= ls_ptr + 1;
    end
         
  end

  always_comb begin
    read_issue_inst        = fifo[rd_ptr[PTR_SIZE-1:0]].issue_inst;
    read_issue_sb_id       = fifo[rd_ptr[PTR_SIZE-1:0]].issue_sb_id;
    read_issue_scalar_opnd = fifo[rd_ptr[PTR_SIZE-1:0]].issue_scalar_opnd;
    read_issue_vcsr        = fifo[rd_ptr[PTR_SIZE-1:0]].issue_vcsr;
    read_issue_vcsr_lmulb2  = fifo[rd_ptr[PTR_SIZE-1:0]].issue_vcsr_lmulb2;
    // We don't speculatively issue vector instructions to Ocelot
    // as it does not support flushing.
    if (!queue_empty && fifo[rd_ptr[PTR_SIZE-1:0]].next_senior && fifo[rd_ptr[PTR_SIZE-1:0]].valid && !fifo[rd_ptr[PTR_SIZE-1:0]].pending_mem_sync)
      read_valid             = 1'b1;
    else
      read_valid             = 1'b0;
  end

  // immediate_dispatch: assert property(@(posedge clk) disable iff (!reset_n)
  //     ((!queue_full && issue_valid && dispatch_next_senior) wr_ptr == dispatch_ptr));

  logic [31:0] ls_inst;

  assign ls_candidate = fifo[ls_ptr[PTR_SIZE-1:0]];

  assign memop_sync_start = ls_candidate.valid &&
                            ls_candidate.pending_mem_sync &&
                          ( ls_candidate.issue_inst[6:0] == 7'h27 ||
                           (ls_candidate.issue_inst[6:0] == 7'h7 && ls_candidate.ldb_allocated));
  assign memop_sync_start_sb_id = ls_candidate.issue_sb_id;

  // allocate Load Data Buffer entry(s) for load
  assign ldb_alloc_valid = ls_candidate.valid &&
                           ls_candidate.issue_inst[6:0] == 7'h7 &&
                          !ls_candidate.ldb_allocated;

  // Allocation size calculation
  logic [1:0] ldb_alloc_mop;
  logic [4:0] ldb_alloc_lumop;
  logic [2:0] ldb_alloc_nf;
  logic [2:0] ldb_alloc_width;
  logic [2:0] ldb_alloc_eew;
  logic [2:0] ldb_alloc_sew;
  logic [2:0] ldb_alloc_lmul;
  logic [2:0] ldb_alloc_emul;

  assign ldb_alloc_nf    =  ls_candidate.issue_inst[31:29];
  assign ldb_alloc_mop   =  ls_candidate.issue_inst[27:26];
  assign ldb_alloc_lumop =  ls_candidate.issue_inst[24:20];
  assign ldb_alloc_width =  ls_candidate.issue_inst[14:12];
  assign ldb_alloc_eew   = (ldb_alloc_width == 3'b000) ? 3'h0 :
                           (ldb_alloc_width == 3'b101) ? 3'h1 :
                           (ldb_alloc_width == 3'b110) ? 3'h2 :
                                                         3'h3;
  assign ldb_alloc_sew   =  ls_candidate.issue_vcsr[38:36];
  assign ldb_alloc_lmul  = {ls_candidate.issue_vcsr_lmulb2,
                            ls_candidate.issue_vcsr[35:34]};
  always_comb begin
     // Indexed Load: emul = lmul
     if (ldb_alloc_mop inside {2'b01, 2'b11}) begin
        ldb_alloc_emul = ldb_alloc_lmul;
     end else
     // Whole Register Load: emul = nfield
     if (ldb_alloc_mop   == 2'b00    &&
         ldb_alloc_lumop == 5'b01000   ) begin
        ldb_alloc_emul = ldb_alloc_nf == 3'b000 ? 3'b000 : // 1
                         ldb_alloc_nf == 3'b001 ? 3'b001 : // 2
                         ldb_alloc_nf == 3'b011 ? 3'b010 : // 4
                                                  3'b011;
     end else
     // Mask Load: emul = 0
     if (ldb_alloc_mop   == 2'b00    &&
         ldb_alloc_lumop == 5'b01011   ) begin
        ldb_alloc_emul = 3'b000;
     end else begin
     // Otherwise: emul = eew / sew * lmul
        ldb_alloc_emul = tt_briscv_pkg::get_vecldst_emul(.sew (ldb_alloc_sew),
                                                         .lmul(ldb_alloc_lmul),
                                                         .eew (ldb_alloc_eew));
     end
  end

  always_comb begin
     case (ldb_alloc_emul) inside
        // Fractional
        3'b1??: ldb_alloc_size = 4'h1;
        3'b000: ldb_alloc_size = 4'h1;
        3'b001: ldb_alloc_size = 4'h2;
        3'b010: ldb_alloc_size = 4'h4;
        3'b011: ldb_alloc_size = 4'h8;
        default: ldb_alloc_size = 4'h0;
     endcase
  end
  assign ldb_alloc_sb_id  = ls_candidate.issue_sb_id;


endmodule
