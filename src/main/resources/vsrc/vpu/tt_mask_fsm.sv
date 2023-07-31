module tt_mask_fsm #(parameter VLEN = 256,
                     paramater MASK_CREDITS = 2)
                    (input  logic                      i_clk,
                     input  logic                      i_reset_n,
                     input  logic                      i_is_masked_memop,
                     input  logic                      i_is_indexed,
                     input  logic [VLEN-1:0]           i_reg_data,
                     input  logic                      i_reg_data_valid,
                     input  logic                      i_memop_sync_start,
                     input  logic                      i_memop_sync_end,
                     input  logic [$clog2(VLEN+1)-1:0] i_vl,
                     input  logic                      i_mask_idx_credit,
                     output logic [64:0]               o_mask_idx_item,
                     output logic                      o_mask_idx_valid,
                     output logic                      o_mask_idx_last_idx);

  // ----- For masked memops -----
  // if strided, send 1, 2, 3 or 4 transactions based on vl,
  // if indexed, send vl transactions 

  // FSM to drive the mask interface
  typedef enum logic {
  IDLE = 1'b0,
  SEND = 1'b1,
  } mask_state_t;
  mask_state_t mask_fsm_state;
  mask_state_t mask_fsm_next_state;

  logic [$clog2(MASK_CREDITS+1)-1:0] mask_credits;
  logic [$clog2(MASK_CREDITS+1)-1:0] mask_credits_next;

  logic done_sending;
  logic [2:0] num_transactions;
  logic [2:0] num_transactions_next;

  logic                 mask_buffer; 
  logic [7:0][VLEN-1:0] idx_buffer;
  logic [2:0]           idx_buffer_wptr;

  integer k;
  always @(posedge clk) begin
    if(!reset_n)
      for(k=0; k<8; k=k+1)
        idx_buffer[k] <= 0;
    else if(i_is_masked_memop && i_reg_data_valid) begin
      for(k=0; k<VLEN; k=k+8)
        idx_buffer[idx_buffer_wptr] <= i_reg_data;
    end
  end

  always @(posedge clk) begin
    if(!reset_n)
      idx_buffer_wptr <= 0;
    else begin
      if(i_is_masked_memop && i_reg_data_valid)
        idx_buffer_wptr <= idx_buffer_wptr + 1;
      else if(i_memop_sync_end)
        idx_buffer_wptr <= 0;
    end
  end

  always @(posedge clk) begin
    if(!reset_n)
      mask_buffer <= 0;
    else if(i_is_masked_memop && i_reg_data_valid) begin
      mask_buffer <= i_reg_data;
    else if(o_mask_idx_valid)
      mask_buffer <= mask_buffer >> 64;
    end
  end

  assign mask_credits_next = mask_credits + i_mask_idx_credit - o_mask_idx_valid;
  always @(posedge clk) begin
    if(!reset_n)
      mask_credits <= MASK_CREDITS;
    else
      mask_credits <= mask_credits_next;
  end

  always_comb begin
    case(mask_fsm_state)
      IDLE: begin
        if(i_vl <= 64)
          num_transactions_next = 1;
        else if (i_vl <= 128)
          num_transactions_next = 2;
        else if (i_vl <= 192)
          num_transactions_next = 3;
        else
          num_transactions_next = 4;
      end
      SEND:
        if(mask_credits_next > 0 && mask_idx_valid_next)
          num_transactions_next = num_transactions - 1;
        else
          num_transactions_next = num_transactions;
    endcase
  end

  always @(posedge clk) begin
    if(!i_reset_n)
      num_transactions <= 0;
    else begin
      num_transactions <= num_transactions_next;
    end
  end

  always_comb begin
    case(mask_fsm_state)
      IDLE: begin
        if(i_is_masked_memop && i_memop_sync_start)
          mask_fsm_next_state = SEND;
        else
          mask_fsm_next_state = IDLE;
      end
      SEND: begin
        if(num_transactions == 0)
          mask_fsm_next_state = IDLE;
        else
          mask_fsm_next_state = SEND;
      end
    endcase
  end

  always @(posedge clk) begin
    if(!i_reset_n)
      mask_fsm_state <= 0;
    else begin
      mask_fsm_state <= mask_fsm_next_state;
    end
  end

  logic [64:0] mask_idx_item_next;
  logic mask_idx_valid_next;
  assign o_mask_idx_last_idx = 0;
  assign mask_idx_item_next[64] = 0;
  assign mask_idx_item_next[63:0] = mask_buffer[63:0];
  assign mask_idx_valid_next = (mask_fsm_state==SEND) && (mask_credits_next > 0) && (num_transactions > 0);
  always @(posedge clk) begin
    if(!reset_n)
      {mask_idx_valid, mask_idx_item} <= 0;
    else begin
      mask_idx_valid <= mask_idx_valid_next;
      mask_idx_item <= mask_idx_item_next;
    end
  end

endmodule