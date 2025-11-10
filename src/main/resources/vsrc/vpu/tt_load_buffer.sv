// See LICENSE.TT for license details.


module tt_load_buffer #(
  parameter VLEN = 256,
  parameter LQ_DEPTH = 8,
  parameter LQ_DEPTH_LOG2 = $clog2(LQ_DEPTH)
) (
  input logic clk,
  input logic reset_n,

  input logic       i_drain_start,
  input logic [2:0] i_drain_rptr,
  input logic [2:0] i_drain_vcount, // number of vectors to read
  input logic [2:0] i_drain_eew,
  input logic [2:0] i_drain_emul,
  input logic [2:0] i_drain_lqid,
  input logic [7:0] i_drain_vstart_vlfof_idx,

  output logic                     o_drain_valid,
  output logic [VLEN-1:0]          o_drain_data,
  output logic [2:0]               o_drain_rptr,
  output logic [LQ_DEPTH_LOG2-1:0] o_drain_lqid,
  output logic [VLEN/8-1:0]        o_drain_vstart_vlfof_byte_mask,

  input logic       i_write_valid,
  input logic [2:0] i_write_idx,
  input logic [4:0] i_write_start_vreg,
  input logic [4:0] i_write_dest_vreg,
  input logic [2:0] i_write_emul,

  input logic [VLEN-1:0]   i_write_data    [7:0],
  input logic [VLEN/8-1:0] i_write_byte_en [7:0]
);

  // =============== load buffer ===============
  logic [VLEN-1:0] load_data_buffer [7:0];

  // =============== drain FSM ===============

  // is_draining
  logic drain_valid;
  // drain counter
  logic [2:0] drain_ctr, drain_ctr_max; // cut to [2:0] since max is 7 after "-1'b1"
  // meta data
  logic [1:0] drain_emul_reg;
  logic [2:0] idx_vgroup, idx_vgroup_reg;
  logic [7:0] idx_byte_offset, idx_byte_offset_reg;
  logic       idx_is_zero_reg;
  // pointers
  logic [2:0] drain_rptr;
  logic [LQ_DEPTH_LOG2-1:0] drain_lqid;

  // decode the vgroup and byte offset from the vstart_vlfof index
  always_comb begin // mux to avoid div or mod
    unique case (i_drain_eew[1:0])
      'd0: begin
        idx_vgroup      = i_drain_vstart_vlfof_idx[7:5];
        idx_byte_offset = i_drain_vstart_vlfof_idx[4:0] << i_drain_eew[1:0];
      end
      'd1: begin
        idx_vgroup      = i_drain_vstart_vlfof_idx[6:4];
        idx_byte_offset = i_drain_vstart_vlfof_idx[3:0] << i_drain_eew[1:0];
      end
      'd2: begin
        idx_vgroup      = i_drain_vstart_vlfof_idx[5:3];
        idx_byte_offset = i_drain_vstart_vlfof_idx[2:0] << i_drain_eew[1:0];
      end
      'd3: begin
        idx_vgroup      = i_drain_vstart_vlfof_idx[4:2];
        idx_byte_offset = i_drain_vstart_vlfof_idx[1:0] << i_drain_eew[1:0];
      end
    endcase
  end

  // FSM for draining the load buffer
  always_ff @(posedge clk) begin
    // -- reset case --
    if (!reset_n) begin
      // FSM state case
      drain_valid <= '0;
      // drain counter
      drain_ctr <= '0;
      drain_ctr_max <= '0;
      // meta data
      drain_emul_reg <= '0;
      idx_vgroup_reg <= '0;
      idx_byte_offset_reg <= '0;
      idx_is_zero_reg <= '0;
      // pointers
      drain_rptr <= '0;
      drain_lqid <= '0;
    end
    // -- start draining ldb case --
    else if(!drain_valid && i_drain_start) begin
      // FSM state case
      drain_valid <= 1'b1;
      // drain counter
      drain_ctr <= '0;
      drain_ctr_max <= i_drain_vcount - 1'b1;
      // meta data
      drain_emul_reg <= i_drain_emul[1:0];
      idx_vgroup_reg <= idx_vgroup;
      idx_byte_offset_reg <= idx_byte_offset;
      idx_is_zero_reg <= (i_drain_vstart_vlfof_idx == '0);
      // pointers
      drain_rptr <= i_drain_rptr;
      drain_lqid <= i_drain_lqid;
    end
    // -- ldb reading case --
    else begin
      // check end condition
      if (drain_ctr == drain_ctr_max) begin
        drain_valid <= 1'b0;
      end
      // drain counter decrement
      if (drain_valid && drain_ctr != drain_ctr_max) begin
        drain_ctr <= drain_ctr + 1'b1;
      end
      // ptr increment
      if (drain_valid) begin
        drain_rptr <= drain_rptr + 1'b1;
        drain_lqid <= drain_lqid + 1'b1;
      end
    end
  end

  logic [2:0] ctr_vgroup;
  assign ctr_vgroup = drain_ctr & ((4'b1<<drain_emul_reg)-1'b1);

  // output logic
  assign o_drain_valid = drain_valid;
  assign o_drain_data = load_data_buffer[drain_rptr];
  assign o_drain_rptr = drain_rptr;
  assign o_drain_lqid = drain_lqid;
  assign o_drain_vstart_vlfof_byte_mask = (
    // unhandled case (assume no except and mask on)
    (idx_is_zero_reg) ? '1 :
    // not yet reached the vgroup (mask on)
    (ctr_vgroup < idx_vgroup_reg) ? '1 :
    // at the vgroup (use mask)
    (ctr_vgroup == idx_vgroup_reg) ? (('d1<<idx_byte_offset_reg)-1'b1) :
    // past the vgroup (mask off)
    '0
  );

  // =============== write FSM ===============

  // pointer for load data buffer
  logic [10:0] ldb_write_idx;   // base pointer for each vreg based on emul and dest vreg
  logic [2:0]  ldb_wptrs [7:0]; // write pointers for each seg field (by adding shift)

  assign ldb_write_idx = ((
    (i_write_start_vreg - i_write_dest_vreg) & // difference in reg column
    ((5'b1 << i_write_emul[1:0]) - 1'b1)) +    // extract emul part / ignore seg field
    i_write_idx                                // add difference to base write idx
  );

  // pointer per seg field
  always_comb begin
    logic [10:0] temp_ptr; // for overflow
    for (int k1 = 0; k1 < 8; k1 = k1 + 1) begin
      temp_ptr = (ldb_write_idx + (k1 << i_write_emul));
      ldb_wptrs[k1] = temp_ptr[2:0]; // take the [2:0] since overflow is irrelevant
    end
  end

  // writing to the load data buffer
  always_ff @(posedge clk) begin
    // -- reset case --
    if (!reset_n) begin
      for (int k2 = 0; k2 < 8; k2 = k2 + 1)
        load_data_buffer[k2] <= '0;
    end
    else begin
    // -- drain case --
      if (drain_valid) // this is not required but is nice to debug
        load_data_buffer[drain_rptr] <= '0;
    // -- write case --
      if (i_write_valid)
        for (int k1 = 0; k1 < 8; k1 = k1 + 1)      // segment output of lrm_model
          for (int k2 = 0; k2 < VLEN; k2 = k2 + 8) // element byte into ldb
            if (i_write_byte_en[k1][k2/8]) begin
              load_data_buffer[ldb_wptrs[k1]][k2+:8] <= i_write_data[k1][k2+:8];
            end
    end
  end

endmodule
