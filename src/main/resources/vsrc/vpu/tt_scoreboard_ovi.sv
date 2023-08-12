module tt_scoreboard_ovi(input logic       clk,
                         input logic       reset_n,
                         input logic [4:0] i_vd,
                         input logic [63:0] i_rd,
                         input logic [4:0] i_fflags,
                         input logic        i_rd_valid,
                         input logic [2:0] i_rd_lqid,
                         input logic [2:0] i_lqnxtid,
                         input logic [1:0] i_data_size,
                         input logic [1:0] i_index_size,
                         input logic [2:0] i_load_stride_eew,
                         input logic       i_memop_sync_end,
                         input logic [4:0] i_memop_sync_end_sb_id,
                         input logic       i_id_store,
                         input logic       i_id_load,
                         input logic       i_id_ex_rts,
                         input logic       i_ex_id_rtr,
                         input logic	     i_vex_id_rtr,
                         input logic	     i_id_vex_rts,
                         input logic [4:0] i_id_sb_id,
                         input logic       i_id_mem_lqalloc,
                         input logic       i_lq_commit,
                         input logic [2:0] i_dest_lqid,
                         input logic       i_first_alloc,
                         input logic       i_last_alloc,
                         input logic [4:0] i_load_sb_id,

                         output logic [4:0] o_vd,
                         output logic [2:0] o_lqid,
                         output logic [1:0] o_data_size,
                         output logic [1:0] o_index_size,
                         output logic [2:0] o_load_stride_eew,

                        // This is like a handshake between the load buffer and the scoreboard
                         output logic       o_drain_load_buffer,
                         input  logic       i_draining_load_buffer,
                         output logic [2:0] o_drain_ref_count,
                         output logic [2:0] o_drain_lqid_start,

                         output logic      o_completed_valid,
                         output logic [4:0]  o_completed_sb_id,
                         output logic [4:0]  o_completed_fflags,
                         output logic [63:0] o_completed_dest_reg,
                         
                         input logic [2047:0] i_debug_commit_data,
                         input logic [7:0]    i_debug_commit_mask,
                         output logic [2047:0] o_debug_commit_data,
                         output logic [7:0]    o_debug_commit_mask);

  typedef struct packed {
    logic [4:0] vd;
    logic [63:0] rd;
    logic [2:0] lqid;
    logic [1:0] data_size;
    logic [1:0] index_size;
    logic [2:0] load_stride_eew;
    logic       is_load;
    logic       drained;
    logic       got_sync_end;
    logic [3:0] ref_count; // counts the number of lq entries allocated for this instruction
    logic       got_last_alloc;
    logic [4:0] fflags;
    logic [2047:0] debug_commit_data;
    logic [7:0] debug_commit_mask;
  } scoreboard_entry;
  logic            [31:0] scoreboard_valid;
  scoreboard_entry [31:0] scoreboard;
  // maps lqid to an sb_id
  logic [7:0][4:0] sb_id_buffer;
  logic [31:0] sync_ends;
  logic [4:0] drain_sb_id;

  integer k;
  always @(posedge clk) begin
    if(!reset_n)
      for(k=0; k<8; k=k+1)
        sb_id_buffer[k] <= 0;
    else begin
      if(i_id_mem_lqalloc)
        sb_id_buffer[i_lqnxtid] <= i_id_sb_id;
    end
  end

  logic [31:0][3:0] ref_count_nxt; // counts the number of lq entries allocated for this instruction
 
  always_comb begin
     for (int i=0; i<32; i++) begin
        ref_count_nxt[i] = scoreboard[i].ref_count;
     end

     if (i_id_mem_lqalloc) begin
        if (i_first_alloc) begin
           ref_count_nxt[i_id_sb_id] = 4'h1;
        end else begin
           ref_count_nxt[i_id_sb_id] += 4'h1;
        end
     end

     if (i_lq_commit) begin
        ref_count_nxt[sb_id_buffer[i_dest_lqid]] -= 4'h1;
     end
  end

  integer m;
  integer c;
  always @(posedge clk) begin
    if(!reset_n) begin
        scoreboard_valid <= '0;
    end else begin
      if(i_id_mem_lqalloc && i_first_alloc) begin
        scoreboard[i_id_sb_id].vd <= i_vd;
        scoreboard[i_id_sb_id].rd <= i_rd;
        scoreboard[i_id_sb_id].lqid <= i_lqnxtid;
        scoreboard[i_id_sb_id].data_size <= i_data_size;
        scoreboard[i_id_sb_id].index_size <= i_index_size;
        scoreboard[i_id_sb_id].load_stride_eew <= i_load_stride_eew;
        scoreboard[i_id_sb_id].is_load <= i_id_load;
        scoreboard[i_id_sb_id].fflags <= '0;
        if(i_last_alloc)
          scoreboard[i_id_sb_id].got_last_alloc <= 1;
        else
          scoreboard[i_id_sb_id].got_last_alloc <= 0;
      end
      else if(i_last_alloc) begin
          scoreboard[i_id_sb_id].got_last_alloc <= 1;
      end

      for(c=0; c<32; c=c+1) begin
        if(i_ex_id_rtr && i_id_ex_rts && i_id_mem_lqalloc && i_first_alloc && i_id_sb_id == c) begin
          scoreboard_valid[c] <= 1;
          scoreboard[c].got_sync_end <= 0;
          scoreboard[c].drained <= 0;
        end
        else if(i_vex_id_rtr && i_id_vex_rts && i_id_mem_lqalloc && i_first_alloc && i_id_sb_id == c) begin
          scoreboard_valid[c] <= 1;
          scoreboard[c].got_sync_end <= 1;
        end
        else if(i_memop_sync_end && i_memop_sync_end_sb_id == c) 
          scoreboard[c].got_sync_end <= 1;
        else if(o_completed_valid && o_completed_sb_id == c)
           scoreboard_valid[c] <= 0;
        else if(o_drain_load_buffer && !i_draining_load_buffer && drain_sb_id == c)
          scoreboard[c].drained <= 1;

        // Update ref_cnt
        if(i_id_mem_lqalloc || i_lq_commit) begin
          scoreboard[c].ref_count <= ref_count_nxt[c];
        end
      end

      if(i_rd_valid) begin
        scoreboard[sb_id_buffer[i_rd_lqid]].rd <= i_rd;
        scoreboard[sb_id_buffer[i_rd_lqid]].fflags <= i_fflags;
        scoreboard[sb_id_buffer[i_rd_lqid]].debug_commit_data <= i_debug_commit_data;
        scoreboard[sb_id_buffer[i_rd_lqid]].debug_commit_mask <= i_debug_commit_mask;
      end
    end
  end

  always_comb begin
    for(int i=0; i<32; i++)
      sync_ends[i] = scoreboard[i].got_sync_end && scoreboard[i].is_load && scoreboard_valid[i] && !scoreboard[i].drained;
  end

  scoreboard_entry selected_entry;
  tt_ffs #(.WIDTH(32), //Number of inputs.
          .DATA_WIDTH($bits(scoreboard_entry)))              //Width of data 
        (.req_in(sync_ends),
          .data_in(scoreboard),
          .req_sum(o_drain_load_buffer), 
          .data_out(selected_entry),
          .req_out(),
          .enc_req_out(drain_sb_id));

  assign o_drain_ref_count = selected_entry.ref_count;
  assign o_drain_lqid_start = selected_entry.lqid;

  logic [31:0] completed_entries;
  always_comb begin
    for(int i=0; i<32; i++)
      completed_entries[i] = scoreboard_valid[i] && scoreboard[i].ref_count == 0 && scoreboard[i].got_sync_end && scoreboard[i].got_last_alloc;
  end
  
  scoreboard_entry completed_entry;
  tt_ffs #(.WIDTH(32),              //Number of inputs.
		       .DATA_WIDTH($bits(scoreboard_entry)))              //Width of data 
          (.req_in(completed_entries),
           .data_in(scoreboard),
           .req_sum(o_completed_valid), 
           .data_out(completed_entry),
           .req_out(),
           .enc_req_out(o_completed_sb_id));

  assign o_completed_fflags  = completed_entry.fflags;
  assign o_completed_dest_reg = completed_entry.rd;
  assign o_debug_commit_data = completed_entry.debug_commit_data;
  assign o_debug_commit_mask = completed_entry.debug_commit_mask;

  assign o_vd              = scoreboard[i_load_sb_id].vd;
  assign o_lqid            = scoreboard[i_load_sb_id].lqid;
  assign o_data_size       = scoreboard[i_load_sb_id].data_size;
  assign o_index_size      = scoreboard[i_load_sb_id].index_size;
  assign o_load_stride_eew = scoreboard[i_load_sb_id].load_stride_eew;

endmodule
