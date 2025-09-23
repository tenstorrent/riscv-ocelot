// See LICENSE.TT for license details.
module tt_mask_fsm #(
  parameter VLEN = 256,
  parameter MASK_CREDITS = 2
) (
  input  logic                      i_clk,
  input  logic                      i_reset_n,
  input  logic                      i_is_masked_memop,
  input  logic                      i_is_indexed,
  input  logic [VLEN-1:0]           i_mask_data,
  input  logic [VLEN-1:0]           i_index_data,
  input  logic                      i_index_data_valid,
  input  logic                      i_last_index,
  input  logic                      i_memop_sync_start_next,
  input  logic [$clog2(VLEN+1)-1:0] i_vl,
  input  logic [$clog2(VLEN+1)-1:0] i_vstart,
  input  logic [1:0]                i_eew,
  input  logic                      i_mask_idx_credit,
  output logic                      o_draining_mask_idx,
  output logic [64:0]               o_mask_idx_item,
  output logic                      o_mask_idx_valid,
  output logic                      o_mask_idx_last_idx
);

  // ----- For masked memops -----
  // if strided, send 1, 2, 3 or 4 transactions based on vl,
  // if indexed, send vl transactions 

  // FSM to drive the mask interface
  typedef enum logic [1:0] {
    IDLE = 2'b00,
    WAIT = 2'b01, // Wait for ID finish cracking
    SEND = 2'b10, // Send mask packets
    RSVD = 2'b11
  } mask_state_t;
  mask_state_t mask_fsm_state;
  mask_state_t mask_fsm_next_state;

  logic [$clog2(MASK_CREDITS+1)-1:0] mask_credits;
  logic [$clog2(MASK_CREDITS+1)-1:0] mask_credits_next;

  logic [$clog2(VLEN+1):0] num_transactions;
  logic [$clog2(VLEN+1):0] num_transactions_next;
  logic drain_index_buffer;
  logic drain_index_buffer_next;

  logic [VLEN-1:0]   i_index_shifted;
  logic [$clog2(VLEN+1)-1:0] byte_shift;
  logic [$clog2(VLEN+1)-1:0] start_reg;
  logic [$clog2(VLEN+1)-1:0] reg_cnt;

  logic [VLEN-1:0]   mask_buffer; 
  logic [8*VLEN-1:0] idx_buffer;
  logic [VLEN-1:0]   idx_buffer_wptr;
  logic [64:0] mask_idx_item_next;
  logic mask_idx_valid_next;
  logic is_indexed;
  logic [1:0] eew;

  always @(posedge i_clk) begin
    if(!i_reset_n) begin
      is_indexed <= 0;
      eew <= 0;
    end
    else if(i_is_indexed && i_memop_sync_start_next) begin
      is_indexed <= 1;
      eew <= i_eew;
    end
    else if(num_transactions_next == 0)
      is_indexed <= 0;
  end

  always_comb begin
    if (i_eew == 2'd0) begin
      byte_shift = {'0, i_vstart[4:0], 0'b0};
      start_reg  = {'0, i_vstart[7:5]};
    end
    else if (i_eew == 2'd1) begin
      byte_shift = {'0, i_vstart[3:0], 1'b0};
      start_reg  = {'0, i_vstart[6:4]};
    end
    else if (i_eew == 2'd2) begin
      byte_shift = {'0, i_vstart[2:0], 2'b0};
      start_reg  = {'0, i_vstart[5:3]};
    end
    else if (i_eew == 2'd3) begin
      byte_shift = {'0, i_vstart[1:0], 3'b0};
      start_reg  = {'0, i_vstart[4:2]};
    end
  end

  assign i_index_shifted = i_index_data >> (byte_shift << 'd3);

  integer k;
  always @(posedge i_clk) begin
    if(!i_reset_n)
      idx_buffer <= 0;
    else if(i_is_indexed && i_index_data_valid) begin
      if (reg_cnt == start_reg)
        idx_buffer[idx_buffer_wptr*8 +: VLEN] <= i_index_shifted;
      else if (reg_cnt > start_reg)
        idx_buffer[idx_buffer_wptr*8 +: VLEN] <= i_index_data;
    end
    else if(mask_idx_valid_next) begin
      case(eew)
        2'd0: idx_buffer <= idx_buffer >> 8;
        2'd1: idx_buffer <= idx_buffer >> 16;
        2'd2: idx_buffer <= idx_buffer >> 32;
        2'd3: idx_buffer <= idx_buffer >> 64;
      endcase
    end
  end
  always @(posedge i_clk) begin
    if(!i_reset_n) begin
      idx_buffer_wptr <= 0;
      reg_cnt <= 0;
    end
    else begin
      if(i_is_indexed && i_index_data_valid) begin
        if (reg_cnt == start_reg)
          idx_buffer_wptr <= idx_buffer_wptr + VLEN/8 - byte_shift;
        else if (reg_cnt > start_reg)
          idx_buffer_wptr <= idx_buffer_wptr + VLEN/8;
        reg_cnt <= reg_cnt + 1;
      end
      else if(num_transactions_next == 0) begin
        idx_buffer_wptr <= 0;
        reg_cnt <= 0;
      end
    end
  end

  always_comb begin
    if(i_index_data_valid && i_is_indexed && i_last_index)
      drain_index_buffer_next = 1;
    else if(num_transactions_next == 0)
      drain_index_buffer_next = 0;
    else 
      drain_index_buffer_next = drain_index_buffer;
  end
  always @(posedge i_clk) begin
    if(!i_reset_n)
      drain_index_buffer <= 0;
    else
      drain_index_buffer <= drain_index_buffer_next;
  end

  always @(posedge i_clk) begin
    if(!i_reset_n)
      mask_buffer <= 0;
    else if(i_is_masked_memop && mask_fsm_state == IDLE)
      mask_buffer <= i_mask_data >> i_vstart;
    else if(mask_idx_valid_next)
      mask_buffer <= is_indexed ? mask_buffer >> 1 : mask_buffer >> 64;
  end

  assign mask_credits_next = mask_credits + i_mask_idx_credit - o_mask_idx_valid;
  always @(posedge i_clk) begin
    if(!i_reset_n)
      mask_credits <= MASK_CREDITS;
    else
      mask_credits <= mask_credits_next;
  end

  always_comb begin
    case(mask_fsm_state)
      IDLE: begin
        if(!i_is_indexed) begin
          if((i_vl - i_vstart) <= 64)
            num_transactions_next = 1;
          else if ((i_vl - i_vstart) <= 128)
            num_transactions_next = 2;
          else if ((i_vl - i_vstart) <= 192)
            num_transactions_next = 3;
          else
            num_transactions_next = 4;
        end
        else begin
          num_transactions_next = (i_vl - i_vstart);
        end
      end
      SEND: begin
        num_transactions_next = num_transactions - mask_idx_valid_next;
      end
    endcase
  end

  always @(posedge i_clk) begin
    if(!i_reset_n)
      num_transactions <= 0;
    else begin
      if (i_memop_sync_start_next || mask_idx_valid_next) begin
        num_transactions <= num_transactions_next;
      end
    end
  end

  always_comb begin
    case(mask_fsm_state)
      IDLE: begin
        if((i_is_masked_memop || i_is_indexed) && i_memop_sync_start_next && (i_vl != 0) && (i_vstart < i_vl)) begin
          if (i_last_index) begin
            mask_fsm_next_state = SEND;
          end else begin
            mask_fsm_next_state = WAIT;
          end
        end else
          mask_fsm_next_state = IDLE;
      end
      WAIT: begin
        if (i_index_data_valid && i_last_index) begin
          mask_fsm_next_state = SEND;
        end else begin
          mask_fsm_next_state = WAIT;
        end
      end 
      SEND: begin
        if(num_transactions == 0)
          mask_fsm_next_state = IDLE;
        else
          mask_fsm_next_state = SEND;
      end
    endcase
  end

  always @(posedge i_clk) begin
    if(!i_reset_n)
      mask_fsm_state <= IDLE;
    else begin
      mask_fsm_state <= mask_fsm_next_state;
    end
  end

  always_comb begin
    mask_idx_item_next[64] = mask_buffer[0];
    if(is_indexed) begin
      case(eew)
        2'd0: mask_idx_item_next[63:0] = {56'b0,idx_buffer[7:0]};
        2'd1: mask_idx_item_next[63:0] = {48'b0,idx_buffer[15:0]};
        2'd2: mask_idx_item_next[63:0] = {32'b0,idx_buffer[31:0]};
        2'd3: mask_idx_item_next[63:0] = idx_buffer[63:0];
      endcase
    end
    else
      mask_idx_item_next[63:0] = mask_buffer[63:0];
  end
  assign mask_idx_valid_next = (mask_fsm_state==SEND) && (mask_credits_next > 0) && (num_transactions > 0) && (!is_indexed || drain_index_buffer);
  always @(posedge i_clk) begin
    if(!i_reset_n)
      {o_mask_idx_valid, o_mask_idx_item, o_mask_idx_last_idx} <= 0;
    else begin
      o_mask_idx_valid <= mask_idx_valid_next;
      o_mask_idx_item <= mask_idx_item_next;
      o_mask_idx_last_idx <= (num_transactions_next == 0) && mask_idx_valid_next;
    end
  end

  assign o_draining_mask_idx = mask_fsm_state == SEND;

endmodule
