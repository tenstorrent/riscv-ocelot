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
    logic        valid;
  } fifo_entry;

  fifo_entry fifo [DEPTH-1:0];
  logic [$clog2(DEPTH)-1:0] wr_ptr;
  logic [$clog2(DEPTH)-1:0] rd_ptr;
  // keeps track of the next instruction to be dispatched
  // either going to kill or make it next_senior
  logic [$clog2(DEPTH)-1:0] dispatch_ptr;
  // Need phase bits to determine if the queue is full or empty
  logic wr_phase, rd_phase;
  logic queue_full;
  logic queue_empty;
  // This 1-bit register saves the last dispatch command
  // 0: next_senior, 1: kill
  logic last_dispatch_was_kill;

  assign queue_full = (wr_phase != rd_phase) && (wr_ptr == rd_ptr);
  assign queue_empty = (wr_phase == rd_phase) && (wr_ptr == rd_ptr);

  always_ff @(posedge clk) begin
    if (!reset_n) begin
      wr_ptr <= 0;
      rd_ptr <= 0;
      wr_phase <= 0;
      rd_phase <= 0;
      dispatch_ptr <= 0;
      last_dispatch_was_kill <= 0;
      for (int i = 0; i < DEPTH; i = i + 1) begin
        fifo[i].issue_inst       <= 0;
        fifo[i].issue_sb_id      <= 0;
        fifo[i].issue_scalar_opnd <= 0;
        fifo[i].issue_vcsr       <= 0;
        fifo[i].issue_vcsr_lmulb2 <= 0;
        fifo[i].next_senior <= 0;
        fifo[i].valid <= 0;
      end
    end
    else if (!queue_full && issue_valid) begin
      fifo[wr_ptr].issue_inst       <= issue_inst;
      fifo[wr_ptr].issue_sb_id      <= issue_sb_id;
      fifo[wr_ptr].issue_scalar_opnd <= issue_scalar_opnd;
      fifo[wr_ptr].issue_vcsr       <= issue_vcsr;
      fifo[wr_ptr].issue_vcsr_lmulb2 <= issue_vcsr_lmulb2;
      fifo[wr_ptr].valid            <= 1;
      // dispatch.next_senior can be sent in the same cycle as the instruction
      if(dispatch_next_senior) begin
        // TODO: assert that wr_ptr == dispatch_ptr
        fifo[wr_ptr].next_senior <= 1;
        dispatch_ptr <= dispatch_ptr + 1;
        last_dispatch_was_kill <= 0;
      end
      else
        fifo[wr_ptr].next_senior <= 0;

      wr_ptr <= wr_ptr + 1;
      // Flip the phase bit when wr_ptr wraps around
      if(wr_ptr == DEPTH - 1)
        wr_phase <= ~wr_phase;
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
        if(wr_phase != rd_phase) begin
          wr_phase <= dispatch_ptr < rd_ptr ? wr_phase : ~wr_phase;
        end
        last_dispatch_was_kill <= 1;
      end
    end

    if (!queue_empty && read_req && !fifo[rd_ptr].valid) begin
      rd_ptr <= rd_ptr + 1;
      // Flip the phase bit when rd_ptr wraps around
      if(rd_ptr == DEPTH - 1)
        rd_phase <= ~rd_phase;
    end
    else if(!queue_empty && fifo[rd_ptr].next_senior && read_req && fifo[rd_ptr].valid) 
      fifo[rd_ptr].valid <= 0;
  end

  always_comb begin
    read_issue_inst        = fifo[rd_ptr].issue_inst;
    read_issue_sb_id       = fifo[rd_ptr].issue_sb_id;
    read_issue_scalar_opnd = fifo[rd_ptr].issue_scalar_opnd;
    read_issue_vcsr        = fifo[rd_ptr].issue_vcsr;
    read_issue_vcsr_lmulb2  = fifo[rd_ptr].issue_vcsr_lmulb2;
    // We don't speculatively issue vector instructions to Ocelot
    // as it does not support flushing.
    if (!queue_empty && fifo[rd_ptr].next_senior && fifo[rd_ptr].valid)
      read_valid             = 1'b1;
    else
      read_valid             = 1'b0;
  end

  // immediate_dispatch: assert property(@(posedge clk) disable iff (!reset_n)
  //     ((!queue_full && issue_valid && dispatch_next_senior) wr_ptr == dispatch_ptr));

endmodule
