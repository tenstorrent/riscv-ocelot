module tt_memop_fsm(input i_clk, 
                    input i_reset_n,
                    input i_load,  
                    input i_store,
                    input i_ex_rtr,
                    input i_id_ex_rts,
                    input i_last_uop,
                    input i_lq_empty,
                    input i_mem_req, // load memory request coming from ocelot
                    input i_memop_sync_end,
                    output o_memop_sync_start,
                    output o_completed_valid,
                    output wait_for_sync_end,
                    output o_ovi_stall);

  // Unified FSM
  typedef enum logic [1:0] {
  IDLE = 2'b00,
  PREPARE = 2'b01,
  BUSY = 2'b10,
  COMMIT = 2'b11
  } memop_state_t;
  memop_state_t memop_fsm_state;
  memop_state_t memop_fsm_next_state;

  logic sent_sync;
  always @(posedge i_clk) begin
    if(!i_reset_n)
      sent_sync <= 0;
    else begin
      if(memop_fsm_state == BUSY && !i_mem_req)
        sent_sync <= 1;
      else if(memop_fsm_state != BUSY)
        sent_sync <= 0;
    end
  end

  assign o_memop_sync_start = memop_fsm_state == BUSY && (i_store || !i_mem_req) && !sent_sync;
  assign o_completed_valid = memop_fsm_state == COMMIT && i_lq_empty;
  assign o_ovi_stall = memop_fsm_state != 0 && memop_fsm_state != 1;

  always_comb begin
    case(memop_fsm_state)
      IDLE: begin
        if((i_store || i_load) && i_id_ex_rts && i_ex_rtr && !i_last_uop)
          memop_fsm_next_state = PREPARE;
        else if((i_store || i_load) && i_id_ex_rts && i_ex_rtr && i_last_uop)
          memop_fsm_next_state = BUSY;
        else
          memop_fsm_next_state = IDLE;
      end
      PREPARE: begin
        if(i_id_ex_rts && i_ex_rtr && i_last_uop)
          memop_fsm_next_state = BUSY;
        else
          memop_fsm_next_state = PREPARE;
      end
      BUSY: begin
        if(i_memop_sync_end)
          memop_fsm_next_state = COMMIT;
        else
          memop_fsm_next_state = BUSY;
      end
      COMMIT: begin
        if(i_lq_empty)
          memop_fsm_next_state = IDLE;
        else
          memop_fsm_next_state = COMMIT; 
      end
    endcase
  end

  always @(posedge i_clk) begin
    if(!i_reset_n)
      memop_fsm_state <= IDLE;
    else
      memop_fsm_state <= memop_fsm_next_state;
  end

endmodule