// See LICENSE.TT for license details.
`include "briscv_defines.h"

module tt_store_fifo #(parameter 
   DATA_WIDTH=32, 
   ADDR_WIDTH=48, 
   DEPTH=8, 
   LOCAL_MEM_BYTE_ADDR_WIDTH=12,
   INCL_VEC=0,
   BYPASS_PATH=0)
(
   input logic 			     i_clk,
   input logic 			     i_reset_n,
   input logic 			     i_rden,
   input logic 			     i_wren,
   input logic                       i_disable_stmerge,
   input logic                       i_exit_merge_mode,
   input logic [4:0]                 i_sttimer_max,
   input logic [DATA_WIDTH-1:0]      i_store_data,
   input logic [ADDR_WIDTH-1:0] 		     i_store_addr,
   input logic [(DATA_WIDTH/8)-1:0]  i_store_byten,
   input logic 			     i_store_ordered,
   input logic [2:0]			     i_store_ldst_sz,

   // Compare signals for load
   input logic 			     i_cmpen,
   input logic [ADDR_WIDTH-1:0] 		     i_cmpaddr,
   input logic [(DATA_WIDTH/8)-1:0]  i_cmpbyten,

   output logic 		     o_store_valid, 
   output logic [DATA_WIDTH-1:0]     o_store_data,
   output logic [ADDR_WIDTH-1:0] 		     o_store_addr,
   output logic [(DATA_WIDTH/8)-1:0] o_store_byten,
   output logic 		     o_store_ordered,
   output logic [2:0]		     o_store_ldst_sz,

   output logic 		     o_cmphit,
   output logic 		     o_empty,
   output logic 		     o_full

);

`ifdef SIM_FIFO_HALF
  localparam DEPTH_MOD = DEPTH/2;
`else
  localparam DEPTH_MOD = DEPTH;
`endif
localparam DEPTH_LOG2 = (DEPTH_MOD == 1) ? 1 : $clog2(DEPTH_MOD);
localparam FIFO_WIDTH = DATA_WIDTH + (DATA_WIDTH/8) + ADDR_WIDTH + 1;
//localparam MERGE_TIMER_MAX = 16;
   
logic [DEPTH_LOG2-1:0] last_wr_ptr;
logic [DEPTH_LOG2-1:0] curr_wr_ptr;
logic [DEPTH_LOG2-1:0] wr_ptr;
logic [DEPTH_LOG2-1:0] rd_ptr;
logic [DEPTH_MOD-1:0] fifo_valid;
logic [DEPTH_MOD-1:0][DATA_WIDTH-1:0] fifo_data;
logic [DEPTH_MOD-1:0][ADDR_WIDTH-1:0] 	      fifo_addr;
logic [DEPTH_MOD-1:0][(DATA_WIDTH/8)-1:0] fifo_byten;
logic [DEPTH_MOD-1:0] 		      fifo_ordered;
logic [DEPTH_MOD-1:0][2:0] 		      fifo_ldst_sz;

logic [DEPTH_LOG2:0] 	      num_vld;
logic bypass_sel;
logic fifo_rd_en, fifo_wr_en;

logic        fifo_merge_addr, fifo_merge_en;
logic [ADDR_WIDTH-1:0] store_addr_offset;
logic        first_write_detected;
logic        store_merge_mode_in, store_merge_mode;
logic [4:0]  store_merge_timer;

logic [DATA_WIDTH-1:0]     fifo_data_in;
logic [ADDR_WIDTH-1:0]               fifo_addr_in;
logic [(DATA_WIDTH/8)-1:0] fifo_byten_in;
logic 	                   fifo_ordered_in;
logic [2:0]                fifo_ldst_sz_in;

logic [DATA_WIDTH-1:0]     fifo_data_out;
logic [ADDR_WIDTH-1:0]               fifo_addr_out;
logic [(DATA_WIDTH/8)-1:0] fifo_byten_out;
logic 	                   fifo_ordered_out;
logic [2:0]	                fifo_ldst_sz_out;
   

assign fifo_wr_en = i_wren & (~o_full || i_rden);
assign fifo_rd_en = o_store_valid;

// Indicate store valid if no coalescing
// Coalescing happens to last entry only to assert valid if more than 1 entry
assign o_store_valid = i_rden & ((~fifo_merge_en & (~store_merge_mode | (store_merge_timer == i_sttimer_max))) | (num_vld > 1));
   
// Full / Empty and output data
assign o_empty = ~(|fifo_valid[DEPTH_MOD-1:0]);
assign o_full  =  &fifo_valid[DEPTH_MOD-1:0];
   
// Determine number of valids
always_comb begin
   num_vld = '0;
   for (int i=0; i<DEPTH_MOD; i++)
     num_vld += fifo_valid[i];
end

// Storage block
assign curr_wr_ptr = fifo_merge_en ? last_wr_ptr : wr_ptr;
always_ff @(posedge i_clk) begin
   if (fifo_wr_en) begin
      fifo_data[curr_wr_ptr]    <= fifo_data_in;
      fifo_addr[curr_wr_ptr]    <= fifo_addr_in;
      fifo_byten[curr_wr_ptr]   <= fifo_byten_in;
      fifo_ordered[curr_wr_ptr] <= fifo_ordered_in;
      fifo_ldst_sz[curr_wr_ptr] <= fifo_ldst_sz_in;
   end
end

assign {fifo_data_out, fifo_addr_out, fifo_byten_out, fifo_ordered_out, fifo_ldst_sz_out} = {fifo_data[rd_ptr], fifo_addr[rd_ptr], fifo_byten[rd_ptr], fifo_ordered[rd_ptr], fifo_ldst_sz[rd_ptr]};  

// Output data
assign {o_store_data, o_store_addr, o_store_byten, o_store_ordered, o_store_ldst_sz} = BYPASS_PATH & o_empty ? {i_store_data, i_store_addr, i_store_byten, i_store_ordered, i_store_ldst_sz} : 
                                                                                                               {fifo_data_out, fifo_addr_out, fifo_byten_out, fifo_ordered_out, fifo_ldst_sz_out};

// Compare logic
always_comb begin
   o_cmphit = '0;
   for (int i=0; i<DEPTH_MOD; i++) begin
      o_cmphit |= (fifo_valid[i] & i_cmpen & (fifo_ordered[i] | ((fifo_addr[i][ADDR_WIDTH-1:4] == i_cmpaddr[ADDR_WIDTH-1:4]) & |(fifo_byten[i] & i_cmpbyten)))); // Match on addressed or ordered 
   end
end

// Coalescing logic
assign last_wr_ptr = wr_ptr - 1;

// Coalesce only for L1/DATA MEM
assign fifo_merge_addr = '0;
assign fifo_merge_en = store_merge_mode & ~i_disable_stmerge & fifo_merge_addr & ~(i_store_ordered | fifo_ordered[last_wr_ptr]) & i_wren & fifo_valid[last_wr_ptr] & 
                       (i_store_addr[ADDR_WIDTH-1:$clog2(DATA_WIDTH/8)] == fifo_addr[last_wr_ptr][ADDR_WIDTH-1:$clog2(DATA_WIDTH/8)]);

assign fifo_addr_in[ADDR_WIDTH-1:0] = i_store_addr[ADDR_WIDTH-1:0];					  
assign fifo_ordered_in    = i_store_ordered;
assign fifo_ldst_sz_in    = i_store_ldst_sz;
for (genvar i=0; i<(DATA_WIDTH/8); i++) begin
   assign fifo_data_in[(8*i)+:8] = (fifo_merge_en & ~i_store_byten[i]) ? fifo_data[last_wr_ptr][(8*i)+:8] : i_store_data[(8*i)+:8];
   assign fifo_byten_in[i]       = fifo_merge_en ? (i_store_byten[i] | fifo_byten[last_wr_ptr][i]) : i_store_byten[i];
end

// Determine the addr offset between current and last store and go to coalescing mode if it's 4 or less
assign store_addr_offset = (i_store_addr[ADDR_WIDTH-1:0] - fifo_addr[last_wr_ptr]);				      
assign store_merge_mode_in = first_write_detected & ~i_disable_stmerge & ~i_exit_merge_mode & i_wren & fifo_merge_addr & ((store_addr_offset <= 4) | (store_addr_offset >= ({{ADDR_WIDTH-2{1'b1}}, 2'h0})));
					  
// Flop logic starts here
// Valid flops
always_ff @(posedge i_clk) begin
  if(~i_reset_n) begin
     fifo_valid <= 'd0;
   end
   else begin
     if(fifo_wr_en & ~fifo_merge_en) fifo_valid[wr_ptr[DEPTH_LOG2-1:0]] <= 1'b1;
     if(fifo_rd_en) fifo_valid[rd_ptr[DEPTH_LOG2-1:0]] <= 1'b0;
   end
end

// Pointer logic
always_ff @(posedge i_clk) begin
   if(!i_reset_n) begin
      wr_ptr <= 'b0;
      rd_ptr <= 'b0;
   end
   else begin
      if(fifo_wr_en & ~fifo_merge_en) wr_ptr <= (wr_ptr == (DEPTH_MOD - 1)) ? '0 : wr_ptr + 1'b1;
      else           wr_ptr <= wr_ptr;

      if(fifo_rd_en) rd_ptr <= (rd_ptr == (DEPTH_MOD - 1)) ? '0 : rd_ptr + 1'b1;
      else           rd_ptr <= rd_ptr;
   end
end

// Coalescing flops
always_ff @(posedge i_clk) begin
   if (~i_reset_n) begin
      first_write_detected <= '0;
      store_merge_mode  <= '0;
      store_merge_timer    <= '0;
   end else begin				      
      if (fifo_wr_en) begin
         first_write_detected <= '1;
         store_merge_mode  <= store_merge_mode_in;
         store_merge_timer <= '0;
      end else begin
         first_write_detected <= first_write_detected;
         store_merge_mode  <= store_merge_mode & ~i_exit_merge_mode;
         store_merge_timer <= store_merge_mode ? ((store_merge_timer == i_sttimer_max) ? store_merge_timer : (store_merge_timer + 1)) : '0;
      end
   end
end

				      
`ifdef SIM

`ASSERT_COND_CLK(i_rden, !o_empty, "Reading from an empty FIFO");
`ASSERT_COND_CLK(i_wren, !o_full,  "Writing to a full FIFO");

`endif

endmodule // tt_store_fifo

