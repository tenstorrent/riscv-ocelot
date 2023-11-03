// See LICENSE.TT for license details.
module tt_store_fsm #(parameter VLEN = 256,
                     parameter STORE_CREDITS = 4)
                    (input  logic                      i_clk,
                     input  logic                      i_reset_n,
                     input  logic                      i_uop_last,
                     input  logic                      i_uop_first,
                     input  logic                      i_uop_fire,
                     input  logic                      i_uop_is_store,
                     input  logic                      i_uop_is_vsm, // mask store
                     input  logic                      i_uop_is_vsx, // index store
                     input  logic                      i_uop_is_vsr, // whole register store
                     input  logic [1:0]                i_uop_index_size, // used to calculate index/data ratio
                     input  logic [1:0]                i_uop_data_size,
                     input  logic [$clog2(VLEN+1)-1:0] i_uop_vl,
                     input  logic [2:0]                i_uop_nfield, // used for whole register store

                     input  logic [VLEN-1:0]           i_store_data,
                     input  logic                      i_store_credit,

                     output logic                      o_store_valid,
                     output logic [511:0]              o_store_data,
                     output logic                      o_stall
);

  // Store logic
  logic [7:0][VLEN-1:0] store_buffer;
  logic [2:0] store_buffer_wptr;
  logic [2:0] store_buffer_rptr;
  logic [2:0] num_store_transactions;
  logic [2:0] num_store_transactions_next;

  typedef enum logic [1:0] {
  IDLE = 2'b00,
  WAIT = 2'b01, // Wait for ID finish cracking
  SEND = 2'b10, // Send mask packets
  RSVD = 2'b11
  } store_state_t;
  store_state_t store_fsm_state;
  store_state_t store_fsm_next_state;

  always_comb begin
    case(store_fsm_state)
      IDLE: begin
        if(i_uop_fire     &&
           i_uop_is_store &&
           i_uop_first      ) begin
          if (i_uop_last) begin
            store_fsm_next_state = SEND;
          end else begin
            store_fsm_next_state = WAIT;
          end
        end else
          store_fsm_next_state = IDLE;
      end
      WAIT: begin
        if (i_uop_fire && i_uop_last) begin
          store_fsm_next_state = SEND;
        end else begin
          store_fsm_next_state = WAIT;
        end
      end 
      SEND: begin
        if(num_store_transactions_next == 0)
          store_fsm_next_state = IDLE;
        else
          store_fsm_next_state = SEND;
      end
    endcase
  end

  always @(posedge i_clk) begin
    if(!i_reset_n)
      store_fsm_state <= IDLE;
    else begin
      store_fsm_state <= store_fsm_next_state;
    end
  end

  logic [$clog2(STORE_CREDITS+1)-1:0] store_credits, store_credits_nxt;

  always @(posedge i_clk) begin
    if(!i_reset_n)
      store_credits <= STORE_CREDITS;
    else begin
      store_credits <= store_credits_nxt;
    end
  end

  assign store_credits_nxt = store_credits + i_store_credit - o_store_valid;

  always_ff@(posedge i_clk) begin
     if(!i_reset_n) begin
        store_buffer_rptr <= 0;
        store_buffer_wptr <= '0;
     end else begin
        if(o_store_valid) begin
           if (num_store_transactions_next == 0) begin
              store_buffer_rptr <= '0;
           end else begin
              store_buffer_rptr <= store_buffer_rptr + 2;
           end
        end

        if(i_uop_fire && i_uop_is_store) begin
           if (i_uop_last) begin
              store_buffer_wptr <= '0;
           end else begin
              store_buffer_wptr <= store_buffer_wptr + 1;
           end
        end
     end
  end

  always_ff@(posedge i_clk) begin
     if (i_uop_fire     &&
         i_uop_is_store   ) begin
        store_buffer[store_buffer_wptr] <= i_store_data;
     end 
  end

  logic [$clog2(VLEN+1)+2:0] vl_adj;
  assign vl_adj = {3'h0, $clog2(VLEN+1)'(i_uop_vl)} << i_uop_data_size;
  always_comb begin
     num_store_transactions_next = num_store_transactions;

     if (i_uop_fire  &&
         i_uop_first   ) begin
        // Whole register store doesn't care about vtype and vl
        if (i_uop_is_vsr) begin
           case (i_uop_nfield)
              3'b000,
              3'b001 : num_store_transactions_next = 1;
              3'b011 : num_store_transactions_next = 2;
              3'b111 : num_store_transactions_next = 4;
           endcase
        end else
        // Vl = 0
        if (i_uop_vl == 0) begin
           num_store_transactions_next = 0;
        end else
        // Mask store only send up to VLEN
        if (i_uop_is_vsm) begin
           num_store_transactions_next = 1;
        end else begin
           num_store_transactions_next = 3'(vl_adj >> 6) + |vl_adj[5:0];
        end
     end

     if (o_store_valid) begin
        num_store_transactions_next -= 1;
     end
  end

  always @(posedge i_clk) begin
    if(!i_reset_n) begin
       num_store_transactions <= 0;
    end else begin
       num_store_transactions <= num_store_transactions_next;
    end
  end

  assign o_store_valid         = store_fsm_state == SEND && store_credits > 0;
  assign o_store_data[255:  0] = store_buffer[store_buffer_rptr];
  assign o_store_data[511:256] = store_buffer[store_buffer_rptr+1];
  assign o_stall               = store_fsm_state == SEND;
endmodule
