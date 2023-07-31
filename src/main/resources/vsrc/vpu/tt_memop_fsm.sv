module tt_memop_fsm(input  logic i_clk, 
                    input  logic i_reset_n,
                    input  logic i_load,  
                    input  logic i_store,
                    input  logic i_ex_rtr,
                    input  logic i_id_ex_rts,
                    input  logic i_last_uop,
                    input  logic i_lq_empty,
                    input  logic i_mem_req, // load memory request coming from ocelot
                    input  logic i_memop_sync_end,
                    output logic o_memop_sync_start,
                    output logic o_completed_valid,
                    output logic o_ovi_stall,
                    output logic o_is_load);

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

  assign o_memop_sync_start = memop_fsm_state == IDLE && (i_store || i_load) && i_id_ex_rts && i_ex_rtr;
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
    if(!i_reset_n) begin
      memop_fsm_state <= IDLE;
      o_is_load <= 1'b0;
    end else begin
      memop_fsm_state <= memop_fsm_next_state;
      if (memop_fsm_state == IDLE && (i_store || i_load) && i_id_ex_rts && i_ex_rtr) begin
        o_is_load <= i_load;
      end else
      if (memop_fsm_state == COMMIT && i_lq_empty) begin
        o_is_load <= 1'b0;
      end
    end
      
  end

endmodule
