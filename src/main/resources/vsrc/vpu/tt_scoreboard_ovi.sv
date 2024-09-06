module tt_scoreboard_ovi(input logic       clk,
                         input logic       reset_n,
                         // Issue interface
                         input logic         i_issue_valid,
                         input  logic [ 4:0] i_issue_sb_id,
                         input  logic [31:0] i_issue_inst,
                         input  logic [ 2:0] i_issue_vsew,
                         input  logic [63:0] i_issue_scalar_opnd,
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
                         input logic	     i_id_matrix_rts,
                         input logic [4:0] i_id_sb_id,
                         input logic       i_id_mem_lqalloc,
                         input logic       i_first_alloc,
                         input logic       i_last_alloc,
                         input logic [4:0] i_load_sb_id,

                         // Memop Sync Start interface
                         input  logic       i_memop_sync_start,
                         input  logic [4:0] i_memop_sync_start_sb_id,

                         // Load Data Buffer allocation interface from issue queue
                         input  logic        ldb_alloc_valid,
                         output logic        ldb_alloc_ack,
                         input  logic [4:0]  ldb_alloc_sb_id,
                         input  logic [3:0]  ldb_alloc_size,

                         output logic [4:0] o_vd,
                         output logic [2:0] o_ldb_start,
                         output logic [1:0] o_data_size,
                         output logic [1:0] o_index_size,
                         output logic [2:0] o_load_stride_eew,

                        // This is like a handshake between the load buffer and the scoreboard
                         output logic       o_drain_load_buffer,
                         input  logic       i_draining_load_buffer,
                         output logic [2:0] o_drain_ref_count,
                         output logic [2:0] o_drain_lqid_start,
                         output logic [2:0] o_drain_ldb_start,

                         input  logic       i_drain_complete_valid,
                         input  logic [2:0] i_drain_complete_ldb_idx,

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
    logic [2:0] ldb_start;
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
  logic [31:0] ready_to_drain;
  logic [4:0] drain_sb_id;

  // Load Data Buffer management
  logic [7:0] ldb_valid;
  logic [2:0] ldb_free_ptr;
  logic [7:0] ldb_alloc_msk_raw;
  logic [15:0] ldb_alloc_msk_2x;
  logic [7:0] ldb_alloc_msk;

  // Calulate information for Load Return Mux
  logic [2:0] issue_load_stride_eew;
  logic issue_is_unit_stride;
  logic issue_is_indexldst;

  // from 0 to 3, 8-bit to 64-bit EEW
  logic [1:0]   issue_index_size;
  logic [1:0]   issue_data_size;
  logic signed [63:0]  issue_load_stride;

  // this is in bytes
  always_ff @(posedge clk) begin
     if (!reset_n) begin
        ldb_valid <= '0;
     end else begin
        if (ldb_alloc_valid &&
            ldb_alloc_ack     ) begin
           ldb_valid <= ldb_valid | ldb_alloc_msk;
        end

        if (i_drain_complete_valid) begin
           ldb_valid[i_drain_complete_ldb_idx] <= 1'b0;
        end
     end
  end

  assign ldb_alloc_msk_raw = (ldb_alloc_size == 3'h1) ? 8'b0000_0001 :
                             (ldb_alloc_size == 3'h2) ? 8'b0000_0011 :
                             (ldb_alloc_size == 3'h3) ? 8'b0000_0111 :
                             (ldb_alloc_size == 3'h4) ? 8'b0000_1111 :
                             (ldb_alloc_size == 3'h5) ? 8'b0001_1111 :
                             (ldb_alloc_size == 3'h6) ? 8'b0011_1111 :
                             (ldb_alloc_size == 3'h7) ? 8'b0111_1111 :
                                                        8'b1111_1111;
  assign ldb_alloc_msk_2x = {8'h0, ldb_alloc_msk_raw} << ldb_free_ptr;

  assign ldb_alloc_msk = ldb_alloc_msk_2x[15:8] |
                         ldb_alloc_msk_2x[ 7:0];

  always_ff @(posedge clk) begin
     if (!reset_n) begin
        ldb_free_ptr <= '0;
     end else begin
        if (ldb_alloc_valid &&
            ldb_alloc_ack     ) begin
           ldb_free_ptr <= ldb_free_ptr + ldb_alloc_size;
        end
     end
  end

  assign ldb_alloc_ack =   ldb_alloc_valid &&
                        ~|(ldb_valid & ldb_alloc_msk);

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

     if (i_rd_valid) begin
        ref_count_nxt[sb_id_buffer[i_rd_lqid]] -= 4'h1;
     end
  end

  integer m;
  integer c;
  always @(posedge clk) begin
    if(!reset_n) begin
        scoreboard_valid <= '0;
    end else begin
      if(i_issue_valid) begin
        scoreboard[i_issue_sb_id].vd              <= i_issue_inst[11:7];
        scoreboard[i_issue_sb_id].data_size       <= issue_data_size;
        scoreboard[i_issue_sb_id].index_size      <= issue_index_size;
        scoreboard[i_issue_sb_id].load_stride_eew <= issue_load_stride_eew;
      end

      if(ldb_alloc_valid && ldb_alloc_ack) begin
        scoreboard[ldb_alloc_sb_id].ldb_start <= ldb_free_ptr;
      end

      if(i_id_mem_lqalloc && i_first_alloc) begin
        scoreboard[i_id_sb_id].rd <= i_rd;
        scoreboard[i_id_sb_id].lqid <= i_lqnxtid;
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
        // Reset for load when allocating load data buffer
        if(i_memop_sync_start && i_memop_sync_start_sb_id == c) begin
          scoreboard[c].got_sync_end <= 0;
        end else
        if(i_vex_id_rtr && i_id_vex_rts && i_id_mem_lqalloc && i_first_alloc && i_id_sb_id == c) begin
          scoreboard[c].got_sync_end <= 1;
        end else
        if(i_id_matrix_rts && i_id_mem_lqalloc && i_first_alloc && i_id_sb_id == c) begin
          scoreboard[c].got_sync_end <= 1;
        end else
        if(i_memop_sync_end && i_memop_sync_end_sb_id == c) begin
          scoreboard[c].got_sync_end <= 1;
        end

        if(i_ex_id_rtr && i_id_ex_rts && i_id_mem_lqalloc && i_first_alloc && i_id_sb_id == c) begin
          scoreboard_valid[c] <= 1;
          scoreboard[c].drained <= 0;
        end
        else if(i_vex_id_rtr && i_id_vex_rts && i_id_mem_lqalloc && i_first_alloc && i_id_sb_id == c) begin
          scoreboard_valid[c] <= 1;
        end
        else if(i_id_matrix_rts && i_id_mem_lqalloc && i_first_alloc && i_id_sb_id == c) begin
          scoreboard_valid[c] <= 1;
        end
        else if(o_completed_valid && o_completed_sb_id == c)
           scoreboard_valid[c] <= 0;
        else if(o_drain_load_buffer && !i_draining_load_buffer && drain_sb_id == c)
          scoreboard[c].drained <= 1;

        // Update ref_cnt
        if(i_id_mem_lqalloc || i_rd_valid) begin
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
    issue_is_unit_stride = i_issue_inst[27:26] == 2'b00;
    issue_is_indexldst = i_issue_inst[27:26] inside {2'b01, 2'b11};
    issue_data_size = issue_is_indexldst ? i_issue_vsew[1:0] :
                i_issue_inst[14:12] == 3'b000 ? 2'd0 : // 8-bit EEW
                i_issue_inst[14:12] == 3'b101 ? 2'd1 : // 16-bit EEW
                i_issue_inst[14:12] == 3'b110 ? 2'd2 : 2'd3; // 32-bit, 64-bit EEW
    issue_index_size =  i_issue_inst[14:12] == 3'b000 ? 2'd0 : // 8-bit EEW
                  i_issue_inst[14:12] == 3'b101 ? 2'd1 : // 16-bit EEW
                  i_issue_inst[14:12] == 3'b110 ? 2'd2 : 2'd3; // 32-bit, 64-bit EEW      
    issue_load_stride = !issue_is_indexldst ? i_issue_scalar_opnd : 
                i_issue_vsew == 3'b000 ? 1 : // 8-bit EEW
                i_issue_vsew == 3'b101 ? 2 : // 16-bit EEW
                i_issue_vsew == 3'b110 ? 4 : 8; // 32-bit, 64-bit EEW;                            
    if(issue_is_unit_stride)
        issue_load_stride_eew = 0;
    else if((issue_data_size == 0 && issue_load_stride == 64'd1) ||
            (issue_data_size == 1 && issue_load_stride == 64'd2) || 
            (issue_data_size == 2 && issue_load_stride == 64'd4) || 
            (issue_data_size == 3 && issue_load_stride == 64'd8))
        issue_load_stride_eew = 0;
    else if((issue_data_size == 0 && issue_load_stride == 64'd2) ||
            (issue_data_size == 1 && issue_load_stride == 64'd4) || 
            (issue_data_size == 2 && issue_load_stride == 64'd8) || 
            (issue_data_size == 3 && issue_load_stride == 64'd16))
        issue_load_stride_eew = 1;
    else if((issue_data_size == 0 && issue_load_stride == 64'd4) ||
            (issue_data_size == 1 && issue_load_stride == 64'd8) || 
            (issue_data_size == 2 && issue_load_stride == 64'd16) || 
            (issue_data_size == 3 && issue_load_stride == 64'd32))
        issue_load_stride_eew = 2;
    else if((issue_data_size == 0 && issue_load_stride == -64'd1) ||
            (issue_data_size == 1 && issue_load_stride == -64'd2) || 
            (issue_data_size == 2 && issue_load_stride == -64'd4) || 
            (issue_data_size == 3 && issue_load_stride == -64'd8))
        issue_load_stride_eew = 4;
    else if((issue_data_size == 0 && issue_load_stride == -64'd2) ||
            (issue_data_size == 1 && issue_load_stride == -64'd4) || 
            (issue_data_size == 2 && issue_load_stride == -64'd8) || 
            (issue_data_size == 3 && issue_load_stride == -64'd16))
        issue_load_stride_eew = 5;
    else if((issue_data_size == 0 && issue_load_stride == -64'd4) ||
            (issue_data_size == 1 && issue_load_stride == -64'd8) || 
            (issue_data_size == 2 && issue_load_stride == -64'd16) || 
            (issue_data_size == 3 && issue_load_stride == -64'd32))
        issue_load_stride_eew = 6;
    else
        issue_load_stride_eew = 0;
  end

  always_comb begin
    for(int i=0; i<32; i++)
      ready_to_drain[i] = scoreboard[i].got_sync_end && scoreboard[i].is_load && scoreboard_valid[i] && !scoreboard[i].drained && scoreboard[i].got_last_alloc;
  end

  scoreboard_entry selected_entry;
  tt_ffs #(.WIDTH(32), //Number of inputs.
          .DATA_WIDTH($bits(scoreboard_entry)))              //Width of data 
  i_drain_ffs
        (.req_in(ready_to_drain),
          .data_in(scoreboard),
          .req_sum(o_drain_load_buffer), 
          .data_out(selected_entry),
          .req_out(),
          .enc_req_out(drain_sb_id));

  assign o_drain_ref_count = selected_entry.ref_count;
  assign o_drain_lqid_start = selected_entry.lqid;
  assign o_drain_ldb_start = selected_entry.ldb_start;

  logic [31:0] completed_entries;
  always_comb begin
    for(int i=0; i<32; i++)
      completed_entries[i] = scoreboard_valid[i] && scoreboard[i].ref_count == 0 && scoreboard[i].got_sync_end && scoreboard[i].got_last_alloc;
  end
  
  scoreboard_entry completed_entry;
  tt_ffs #(.WIDTH(32),              //Number of inputs.
		       .DATA_WIDTH($bits(scoreboard_entry)))              //Width of data 
  i_completed_ffs
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
  assign o_ldb_start       = scoreboard[i_load_sb_id].ldb_start;
  assign o_data_size       = scoreboard[i_load_sb_id].data_size;
  assign o_index_size      = scoreboard[i_load_sb_id].index_size;
  assign o_load_stride_eew = scoreboard[i_load_sb_id].load_stride_eew;

endmodule
