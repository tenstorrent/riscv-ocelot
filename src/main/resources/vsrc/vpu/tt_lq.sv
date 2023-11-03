// See LICENSE.TT for license details.
`include "tt_briscv_pkg.vh"
module tt_lq #(parameter 
   LQ_DEPTH=8, 
   LQ_DEPTH_LOG2=$clog2(LQ_DEPTH),
   DATA_REQ_ID_WIDTH=tt_briscv_pkg::LQ_DEPTH_LOG2,
   LD_DATA_WIDTH_BITS=32,
   VLEN=128,
   INCL_VEC=0)
(
   input logic 				i_clk,
   input logic 				i_reset_n,

   input logic                          i_bypass_disable,
 
   // ID <--> MEM signals
   output logic [LQ_DEPTH_LOG2-1:0] 	o_mem_id_lqnxtid,
   input logic 				i_id_mem_lqalloc,
   input 				tt_briscv_pkg::lq_info_s i_id_mem_lqinfo,
 
   input logic 				i_vecld_elem_sent,
   input logic 				i_vecld_idx_last,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_vecld_id,

   // SKid buffer signals (EX 1Cycle signals will also come muxed in skidbuf signals)
   input logic 				i_skidbuf_lqvld_1c,
   input logic 				i_skidbuf_lqvecld128_1c,
   input logic [VLEN/8-1:0] 		i_skidbuf_lqmask_1c, // Used for vector load  
   input logic [2:0] 			i_skidbuf_lqsz_1c,
   input logic [$clog2(VLEN/8)-1:0] 	i_skidbuf_lqaddr_1c,
   input logic [LD_DATA_WIDTH_BITS-1:0] i_skidbuf_lqdata_1c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_skidbuf_lqid_1c,
 
   // EX --> MEM signals
   input logic 				i_ex_mem_lqvld_2c,
   input logic [31:0] 			i_ex_mem_lqdata_2c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_ex_mem_lqid_2c,
   
   // FP EX --> MEM signals
   input logic 				i_fp_ex_mem_lqvld_1c,
   input logic [31:0] 			i_fp_ex_mem_lqdata_1c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_fp_ex_mem_lqid_1c,
   
   input logic 				i_fp_ex_mem_lqvld_2c,
   input logic [31:0] 			i_fp_ex_mem_lqdata_2c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_fp_ex_mem_lqid_2c,
   
   // VEX --> MEM signals
   input logic 				i_vex_mem_lqvld_1c,
   input logic [VLEN-1:0] 		i_vex_mem_lqdata_1c,
   input tt_briscv_pkg::csr_fp_exc      i_vex_mem_lqexc_1c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_vex_mem_lqid_1c,
   
   input logic 				i_vex_mem_lqvld_2c,
   input logic [VLEN-1:0] 		i_vex_mem_lqdata_2c,
   input tt_briscv_pkg::csr_fp_exc      i_vex_mem_lqexc_2c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_vex_mem_lqid_2c,
   
   input logic 				i_vex_mem_lqvld_3c,
   input logic [VLEN-1:0] 		i_vex_mem_lqdata_3c,
   input tt_briscv_pkg::csr_fp_exc      i_vex_mem_lqexc_3c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_vex_mem_lqid_3c,
   
   // Load data return
   input 				i_data_vld_0,
   input 				i_data_vld_cancel_0,
   input [DATA_REQ_ID_WIDTH-1:0] 	i_data_resp_id_0,
   input [VLEN-1:0] 	                i_data_rddata_0,

   input 				i_data_vld_1, 
   input 				i_data_vld_cancel_1,
   input [DATA_REQ_ID_WIDTH-1:0] 	i_data_resp_id_1,
   input [63:0] 	                i_data_rddata_1, 

   input 				i_data_vld_2, 
   input 				i_data_vld_cancel_2,
   input [DATA_REQ_ID_WIDTH-1:0] 	i_data_resp_id_2,
   input [63:0] 	                i_data_rddata_2, 

   // LQ Read signals
   input logic 				i_lq_rden,
   output logic [LQ_DEPTH_LOG2-1:0] 	o_lq_rdid,
   output 				tt_briscv_pkg::lq_info_s o_lq_rdinfo,
   output [2:0] 			o_lq_rdldstsz,
   output [1:0] 			o_lq_rdmemaddr,
   output [VLEN-1:0] 			o_lq_rddata,
   output [4:0] 			o_lq_rdexc,

   // RF Fwd signals
   output logic 			o_lq_fwdvld,
   output logic [LQ_DEPTH_LOG2-1:0] 	o_lq_fwdid,
   output logic [31:0] 			o_lq_fwddata, 

   output logic 			lq_full,
   output logic 			lq_empty,
   output logic 			o_lq_data_ready,
   output logic 			o_lq_mem_load,
   output logic 			o_lq_mem_vec_load,

   // Broadside data
   output 				tt_briscv_pkg::lq_info_s [LQ_DEPTH-1:0] o_lq_broadside_info,
   output logic [LQ_DEPTH-1:0][31:0] 	o_lq_broadside_data,
   output logic [LQ_DEPTH-1:0] 		o_lq_broadside_valid,
   output logic [LQ_DEPTH-1:0] 		o_lq_broadside_data_valid
 
);

localparam LQ_TAG_WIDTH = $bits(tt_briscv_pkg::lq_info_s);
localparam LQ_DATA_WIDTH = INCL_VEC ? (LD_DATA_WIDTH_BITS + // data
                                       $clog2(VLEN/8)     + // offset
                                       3                  + // sz
                                       VLEN/8             + // mask
                                       1                   )
                                    : (LD_DATA_WIDTH_BITS + 4 + 3);
localparam LQ_RD_PORTS      = 1;
localparam LQ_TAG_WR_PORTS  = 1;
localparam LQ_DATA_WR_PORTS = 6;
localparam LQ_CAM_PORTS     = 1;               // really don't need any CAM ports, but we can just tie down the inputs / leave unused the outputs

logic                   ptrs_equal;
logic [LQ_DEPTH_LOG2:0] wr_ptr,rd_ptr;
logic [LQ_DEPTH-1:0]    lq_set_pending, lq_clear_pending;

logic [2:0]                    lq_bypass_en;
logic [LD_DATA_WIDTH_BITS-1:0] lq_bypass_data;
logic [2:0]                    lq_rf_fwd_en;
logic [LD_DATA_WIDTH_BITS-1:0] lq_fwd_data;
logic lq_wr_en, lq_rd_en;
   
logic [LQ_DEPTH-1:0] lq_set_tag_valid;
logic [LQ_DEPTH-1:0] lq_clear_tag_valid;
logic [LQ_DEPTH-1:0] lq_broadside_tag_valid;
logic [LQ_TAG_WIDTH-1:0] lq_broadside_tag_value [LQ_DEPTH-1:0];

logic [LQ_DEPTH-1:0] lq_set_data_valid;
logic [LQ_DEPTH-1:0] lq_clear_data_valid;
logic [LQ_DEPTH-1:0] lq_broadside_data_valid;
logic [LQ_DATA_WIDTH-1:0] lq_broadside_data_value [LQ_DEPTH-1:0];

logic [LQ_DEPTH-1:0][4:0] lq_refcount, lq_refcount_in;
logic [LQ_DEPTH-1:0]   lq_vecld_last_idx_sent, lq_vecld_last_idx_sent_in;
   
logic                       lq_fifo_write_tag_en    [LQ_TAG_WR_PORTS-1:0];
logic  [LQ_DEPTH_LOG2-1:0]  lq_fifo_write_tag_addr  [LQ_TAG_WR_PORTS-1:0];
logic  [LQ_TAG_WIDTH-1:0]   lq_fifo_write_tag_value [LQ_TAG_WR_PORTS-1:0]; 

logic                       lq_fifo_write_data_en    [LQ_DATA_WR_PORTS-1:0];
logic  [LQ_DEPTH_LOG2-1:0]  lq_fifo_write_data_addr  [LQ_DATA_WR_PORTS-1:0];
logic  [LQ_DATA_WIDTH-1:0]  lq_fifo_write_data_value [LQ_DATA_WR_PORTS-1:0]; 

logic 	                                  lq_fifo_read_en    [LQ_RD_PORTS-1:0];
logic [LQ_DEPTH_LOG2-1:0]                 lq_fifo_read_addr  [LQ_RD_PORTS-1:0];
logic [(LQ_TAG_WIDTH+LQ_DATA_WIDTH)-1:0]  lq_fifo_read_value [LQ_RD_PORTS-1:0]; 

logic                      lq_compare_en              [LQ_CAM_PORTS-1:0]; 
logic                      lq_compare_read_en         [LQ_CAM_PORTS-1:0]; 
logic [LQ_TAG_WIDTH-1:0]   lq_compare_tag_value       [LQ_CAM_PORTS-1:0]; 
logic [LQ_TAG_WIDTH-1:0]   lq_compare_tag_value_mask  [LQ_CAM_PORTS-1:0]; 
logic                      lq_compare_tag_valid_mask  [LQ_CAM_PORTS-1:0]; 

logic [LD_DATA_WIDTH_BITS-1:0] lq_fifo_vecld_write_datafn_0, lq_fifo_vecld_write_datafn_1, lq_fifo_vecld_write_datafn_2;
logic [31:0] 	               lq_fifo_load_write_data_0,  lq_fifo_load_write_data_1,  lq_fifo_load_write_data_2;
logic         ret_vecld_vld_0, ret_vecld_vld_1, ret_vecld_vld_2;
   
// Broadside data
for (genvar i=0; i<LQ_DEPTH; i++) begin
   assign o_lq_broadside_valid[i]      = lq_broadside_tag_valid[i];
   assign o_lq_broadside_data_valid[i] = lq_broadside_data_valid[i];
   assign o_lq_broadside_info[i] = tt_briscv_pkg::lq_info_s'(lq_broadside_tag_value[i]);
   assign o_lq_broadside_data[i] = lq_broadside_data_value[i][31:0];
end   
   
assign lq_wr_en = i_id_mem_lqalloc;
assign lq_rd_en = i_lq_rden;

// Pointer logic    
always_ff @(posedge i_clk) begin
   if (~i_reset_n) begin
      wr_ptr <= '0;
      rd_ptr <= '0;
   end
   else begin
      if (lq_wr_en) wr_ptr <= wr_ptr + 1;
      if (lq_rd_en) rd_ptr <= rd_ptr + 1;
   end
end
   
// full/empty signals
assign ptrs_equal = (wr_ptr[LQ_DEPTH_LOG2-1:0] == rd_ptr[LQ_DEPTH_LOG2-1:0]);
assign lq_empty = ptrs_equal & (rd_ptr[LQ_DEPTH_LOG2] == wr_ptr[LQ_DEPTH_LOG2]);
assign lq_full = ptrs_equal & (rd_ptr[LQ_DEPTH_LOG2] != wr_ptr[LQ_DEPTH_LOG2]);

assign o_mem_id_lqnxtid[LQ_DEPTH_LOG2-1:0] = wr_ptr[LQ_DEPTH_LOG2-1:0];
   
// Set the compare signals to zero
for (genvar i=0; i<LQ_CAM_PORTS; i++) begin
   assign lq_compare_en[i]             = '0;
   assign lq_compare_read_en[i]        = '0;
   assign lq_compare_tag_value[i]      = '0;
   assign lq_compare_tag_value_mask[i] = '0;
   assign lq_compare_tag_valid_mask[i] = '0;
end


for (genvar i=0; i<LQ_DEPTH; i++) begin
   assign lq_set_tag_valid[i]    = lq_wr_en & (wr_ptr[LQ_DEPTH_LOG2-1:0] == i);
   assign lq_clear_tag_valid[i]  = lq_rd_en & (rd_ptr[LQ_DEPTH_LOG2-1:0] == i);

   //lq_set_data_valid[i]   = 1'b0;
   assign lq_clear_data_valid[i] = lq_clear_tag_valid[i];
   assign lq_set_data_valid[i] = ((lq_fifo_write_data_en[0] & (lq_fifo_write_data_addr[0] == i))) |
                                 ((lq_fifo_write_data_en[1] & (lq_fifo_write_data_addr[1] == i))) |
                                 ((lq_fifo_write_data_en[2] & (lq_fifo_write_data_addr[2] == i))) | 
                                 ((lq_fifo_write_data_en[3] & (lq_fifo_write_data_addr[3] == i))) |
// Don't set the valid for loads ((lq_fifo_write_data_en[4] & (lq_fifo_write_data_addr[4] == i))) |
                                 ((lq_fifo_write_data_en[5] & (lq_fifo_write_data_addr[5] == i))); 
end
   
// Bypass the read return if it's to the top entry
assign lq_bypass_en[2:0] = { {(~i_bypass_disable & i_data_vld_2 & ~i_data_vld_cancel_2 & ~o_lq_broadside_info[i_data_resp_id_2[LQ_DEPTH_LOG2-1:0]].vec_load & (i_data_resp_id_2[LQ_DEPTH_LOG2-1:0] == rd_ptr[LQ_DEPTH_LOG2-1:0]))},
                             {(~i_bypass_disable & i_data_vld_1 & ~i_data_vld_cancel_1 & ~o_lq_broadside_info[i_data_resp_id_1[LQ_DEPTH_LOG2-1:0]].vec_load & (i_data_resp_id_1[LQ_DEPTH_LOG2-1:0] == rd_ptr[LQ_DEPTH_LOG2-1:0]))},
                             {(~i_bypass_disable & i_data_vld_0 & ~i_data_vld_cancel_0 & ~o_lq_broadside_info[i_data_resp_id_0[LQ_DEPTH_LOG2-1:0]].vec_load & (i_data_resp_id_0[LQ_DEPTH_LOG2-1:0] == rd_ptr[LQ_DEPTH_LOG2-1:0]))} };
assign lq_bypass_data    = ({LD_DATA_WIDTH_BITS{lq_bypass_en[2]}} & lq_fifo_load_write_data_2) |
			   ({LD_DATA_WIDTH_BITS{lq_bypass_en[1]}} & lq_fifo_load_write_data_1) |
                           ({LD_DATA_WIDTH_BITS{lq_bypass_en[0]}} & lq_fifo_load_write_data_0);
   
// Generate per port read signals				 
assign lq_fifo_read_en[0]   = lq_rd_en;
assign lq_fifo_read_addr[0] = rd_ptr[LQ_DEPTH_LOG2-1:0];

assign o_lq_data_ready      = o_lq_broadside_data_valid[rd_ptr[LQ_DEPTH_LOG2-1:0]] | (|lq_bypass_en[2:0]);
assign o_lq_mem_load        = o_lq_broadside_info[rd_ptr[LQ_DEPTH_LOG2-1:0]].load;
assign o_lq_mem_vec_load    = o_lq_broadside_info[rd_ptr[LQ_DEPTH_LOG2-1:0]].vec_load;

assign o_lq_rdid            = rd_ptr[LQ_DEPTH_LOG2-1:0];
assign o_lq_rdinfo          = tt_briscv_pkg::lq_info_s'(lq_broadside_tag_value[rd_ptr[LQ_DEPTH_LOG2-1:0]]);
assign o_lq_rdldstsz[2:0]   = lq_broadside_data_value[rd_ptr[LQ_DEPTH_LOG2-1:0]][(LD_DATA_WIDTH_BITS+4)+:3];
assign o_lq_rdmemaddr[1:0]  = lq_broadside_data_value[rd_ptr[LQ_DEPTH_LOG2-1:0]][LD_DATA_WIDTH_BITS+:2];
assign o_lq_rddata          = (|lq_bypass_en[2:0]) ? lq_bypass_data[LD_DATA_WIDTH_BITS-1:0] : lq_broadside_data_value[rd_ptr[LQ_DEPTH_LOG2-1:0]][LD_DATA_WIDTH_BITS-1:0];
assign o_lq_rdexc[4:0]      = (|lq_bypass_en[2:0]) ? '0                                     : lq_broadside_data_value[rd_ptr[LQ_DEPTH_LOG2-1:0]][LD_DATA_WIDTH_BITS+:5];
				 
// Generate the per port write tag enable and write tag
assign lq_fifo_write_tag_en[0]     = i_id_mem_lqalloc;
assign lq_fifo_write_tag_addr[0]   = wr_ptr[LQ_DEPTH_LOG2-1:0];
assign lq_fifo_write_tag_value[0]  = LQ_TAG_WIDTH'(i_id_mem_lqinfo);

// Generate the fwd enable (this excludes the cancel)
assign lq_rf_fwd_en[2:0] = { {(i_data_vld_2 & ~o_lq_broadside_info[i_data_resp_id_2[LQ_DEPTH_LOG2-1:0]].vec_load & (i_data_resp_id_2[LQ_DEPTH_LOG2-1:0] == rd_ptr[LQ_DEPTH_LOG2-1:0]))},
                             {(i_data_vld_1 & ~o_lq_broadside_info[i_data_resp_id_1[LQ_DEPTH_LOG2-1:0]].vec_load & (i_data_resp_id_1[LQ_DEPTH_LOG2-1:0] == rd_ptr[LQ_DEPTH_LOG2-1:0]))},
                             {(i_data_vld_0 & ~o_lq_broadside_info[i_data_resp_id_0[LQ_DEPTH_LOG2-1:0]].vec_load & (i_data_resp_id_0[LQ_DEPTH_LOG2-1:0] == rd_ptr[LQ_DEPTH_LOG2-1:0]))} };
assign o_lq_fwdvld   = (|lq_rf_fwd_en[2:0]) & ~(i_data_vld_cancel_2 | i_data_vld_cancel_1 | i_data_vld_cancel_0); // Use cancel late for timing
assign o_lq_fwdid    = rd_ptr[LQ_DEPTH_LOG2-1:0];
assign o_lq_fwddata  = ({32{lq_rf_fwd_en[2]}} & lq_fifo_load_write_data_2[31:0]) |
		       ({32{lq_rf_fwd_en[1]}} & lq_fifo_load_write_data_1[31:0]) |
                       ({32{lq_rf_fwd_en[0]}} & lq_fifo_load_write_data_0[31:0]);
   
// Generate the per port write data enable and write data
assign lq_fifo_write_data_en[0]    = i_vex_mem_lqvld_1c;
assign lq_fifo_write_data_addr[0]  = i_vex_mem_lqid_1c;
assign lq_fifo_write_data_value[0] = LQ_DATA_WIDTH'({i_vex_mem_lqexc_1c, i_vex_mem_lqdata_1c});   
   
assign lq_fifo_write_data_en[1]    = i_vex_mem_lqvld_2c;
assign lq_fifo_write_data_addr[1]  = i_vex_mem_lqid_2c;
assign lq_fifo_write_data_value[1] = LQ_DATA_WIDTH'({i_vex_mem_lqexc_2c, i_vex_mem_lqdata_2c});

assign lq_fifo_write_data_en[2]    = i_vex_mem_lqvld_3c;
assign lq_fifo_write_data_addr[2]  = i_vex_mem_lqid_3c;
assign lq_fifo_write_data_value[2] = LQ_DATA_WIDTH'({i_vex_mem_lqexc_3c, i_vex_mem_lqdata_3c});

assign lq_fifo_write_data_en[3]    = i_data_vld_0 & ~i_data_vld_cancel_0 & ~lq_bypass_en[0];
assign lq_fifo_write_data_addr[3]  = i_data_resp_id_0[LQ_DEPTH_LOG2-1:0];
assign lq_fifo_write_data_value[3] = LQ_DATA_WIDTH'({lq_broadside_data_value[lq_fifo_write_data_addr[3]][LQ_DATA_WIDTH-1:LD_DATA_WIDTH_BITS], 
                                                     (ret_vecld_vld_0 ? lq_fifo_vecld_write_datafn_0 : LD_DATA_WIDTH_BITS'(lq_fifo_load_write_data_0))});
   
assign lq_fifo_write_data_en[4]    = i_skidbuf_lqvld_1c;
assign lq_fifo_write_data_addr[4]  = i_skidbuf_lqid_1c;
assign lq_fifo_write_data_value[4] = LQ_DATA_WIDTH'({i_skidbuf_lqvecld128_1c,i_skidbuf_lqmask_1c,i_skidbuf_lqsz_1c,i_skidbuf_lqaddr_1c,i_skidbuf_lqdata_1c});

   
assign lq_fifo_write_data_en[5]    = i_ex_mem_lqvld_2c;
assign lq_fifo_write_data_addr[5]  = i_ex_mem_lqid_2c;
assign lq_fifo_write_data_value[5] = LQ_DATA_WIDTH'(i_ex_mem_lqdata_2c);

// Align the load return data before storing in load queue
assign lq_fifo_load_write_data_0 = align_load_data(i_data_rddata_0[31:0], 
                                                   lq_broadside_data_value[i_data_resp_id_0[LQ_DEPTH_LOG2-1:0]][LD_DATA_WIDTH_BITS+:2], 
                                                   lq_broadside_data_value[i_data_resp_id_0[LQ_DEPTH_LOG2-1:0]][(LD_DATA_WIDTH_BITS+4)+:3]);
assign lq_fifo_load_write_data_1 = align_load_data(i_data_rddata_1[31:0], 
                                                   lq_broadside_data_value[i_data_resp_id_1[LQ_DEPTH_LOG2-1:0]][LD_DATA_WIDTH_BITS+:2], 
                                                   lq_broadside_data_value[i_data_resp_id_1[LQ_DEPTH_LOG2-1:0]][(LD_DATA_WIDTH_BITS+4)+:3]);
assign lq_fifo_load_write_data_2 = align_load_data(i_data_rddata_2[31:0], 
                                                   lq_broadside_data_value[i_data_resp_id_2[LQ_DEPTH_LOG2-1:0]][LD_DATA_WIDTH_BITS+:2], 
                                                   lq_broadside_data_value[i_data_resp_id_2[LQ_DEPTH_LOG2-1:0]][(LD_DATA_WIDTH_BITS+4)+:3]);

if (INCL_VEC == 1) begin: GenInclVec
   logic [LD_DATA_WIDTH_BITS-1:0] lq_fifo_vecld_write_data_0, lq_fifo_vecld_write_data_1, lq_fifo_vecld_write_data_2;
   logic [LD_DATA_WIDTH_BITS-1:0] lq_fifo_vecld_write_data_mask_0, lq_fifo_vecld_write_data_mask_1, lq_fifo_vecld_write_data_mask_2;
   logic [LD_DATA_WIDTH_BITS-1:0] lq_fifo_vecld_hold_data_mask_0, lq_fifo_vecld_hold_data_mask_1, lq_fifo_vecld_hold_data_mask_2;
   logic 			  rsp_entry_match_0_1, rsp_entry_match_0_2, rsp_entry_match_1_2;
   
   assign rsp_entry_match_0_1 = (i_data_vld_0 & ~i_data_vld_cancel_0 & i_data_vld_1 & ~i_data_vld_cancel_1 & (i_data_resp_id_0[LQ_DEPTH_LOG2-1:0] == i_data_resp_id_1[LQ_DEPTH_LOG2-1:0]));
   assign rsp_entry_match_0_2 = (i_data_vld_0 & ~i_data_vld_cancel_0 & i_data_vld_2 & ~i_data_vld_cancel_2 & (i_data_resp_id_0[LQ_DEPTH_LOG2-1:0] == i_data_resp_id_2[LQ_DEPTH_LOG2-1:0]));
   assign rsp_entry_match_1_2 = (i_data_vld_1 & ~i_data_vld_cancel_1 & i_data_vld_2 & ~i_data_vld_cancel_2 & (i_data_resp_id_1[LQ_DEPTH_LOG2-1:0] == i_data_resp_id_2[LQ_DEPTH_LOG2-1:0]));

   // Write data for vector loads
   assign ret_vecld_vld_0   = o_lq_broadside_info[i_data_resp_id_0[LQ_DEPTH_LOG2-1:0]].vec_load;
   assign {lq_fifo_vecld_write_data_mask_0, lq_fifo_vecld_write_data_0} = get_vecld_data(lq_broadside_data_value[i_data_resp_id_0[LQ_DEPTH_LOG2-1:0]], i_data_rddata_0, i_data_resp_id_0);
   assign lq_fifo_vecld_hold_data_mask_0 = ~(lq_fifo_vecld_write_data_mask_0 | 
                                             ({LD_DATA_WIDTH_BITS{rsp_entry_match_0_1}} & lq_fifo_vecld_write_data_mask_1) | 
                                             ({LD_DATA_WIDTH_BITS{rsp_entry_match_0_2}} & lq_fifo_vecld_write_data_mask_2));
   
   assign ret_vecld_vld_1 = o_lq_broadside_info[i_data_resp_id_1[LQ_DEPTH_LOG2-1:0]].vec_load;;
   assign {lq_fifo_vecld_write_data_mask_1, lq_fifo_vecld_write_data_1} = get_vecld_data(lq_broadside_data_value[i_data_resp_id_1[LQ_DEPTH_LOG2-1:0]], i_data_rddata_1, i_data_resp_id_1);
   assign lq_fifo_vecld_hold_data_mask_1 = ~(lq_fifo_vecld_write_data_mask_1 | 
                                             ({LD_DATA_WIDTH_BITS{rsp_entry_match_0_1}} & lq_fifo_vecld_write_data_mask_0) | 
                                             ({LD_DATA_WIDTH_BITS{rsp_entry_match_1_2}} & lq_fifo_vecld_write_data_mask_2));
      
   assign ret_vecld_vld_2 = o_lq_broadside_info[i_data_resp_id_2[LQ_DEPTH_LOG2-1:0]].vec_load;;
   assign {lq_fifo_vecld_write_data_mask_2, lq_fifo_vecld_write_data_2} = get_vecld_data(lq_broadside_data_value[i_data_resp_id_2[LQ_DEPTH_LOG2-1:0]], i_data_rddata_2, i_data_resp_id_2);
   assign lq_fifo_vecld_hold_data_mask_2 = ~(lq_fifo_vecld_write_data_mask_2 | 
                                             ({LD_DATA_WIDTH_BITS{rsp_entry_match_0_2}} & lq_fifo_vecld_write_data_mask_0) | 
                                             ({LD_DATA_WIDTH_BITS{rsp_entry_match_1_2}} & lq_fifo_vecld_write_data_mask_1));
   
   for (genvar i=0; i<LD_DATA_WIDTH_BITS; i++) begin
      assign lq_fifo_vecld_write_datafn_0[i] = (lq_fifo_vecld_write_data_0[i] & lq_fifo_vecld_write_data_mask_0[i]) | 
                                               (lq_broadside_data_value[i_data_resp_id_0[LQ_DEPTH_LOG2-1:0]][i] & lq_fifo_vecld_hold_data_mask_0[i]);
      assign lq_fifo_vecld_write_datafn_1[i] = (lq_fifo_vecld_write_data_1[i] & lq_fifo_vecld_write_data_mask_1[i]) | 
                                               (lq_broadside_data_value[i_data_resp_id_1[LQ_DEPTH_LOG2-1:0]][i] & lq_fifo_vecld_hold_data_mask_1[i]);
      assign lq_fifo_vecld_write_datafn_2[i] = (lq_fifo_vecld_write_data_2[i] & lq_fifo_vecld_write_data_mask_2[i]) | 
                                               (lq_broadside_data_value[i_data_resp_id_2[LQ_DEPTH_LOG2-1:0]][i] & lq_fifo_vecld_hold_data_mask_2[i]);
   end
				 
   // Update the refcount for vector load entries
   for (genvar i=0; i<LQ_DEPTH; i++) begin
      assign lq_refcount_in[i][4:0] = lq_refcount[i]                                                                                + 
                                      (i_vecld_elem_sent & (i_vecld_id == i))                                                       - 
                                      (i_data_vld_0 & ~i_data_vld_cancel_0 & o_lq_broadside_info[i].vec_load & (i_data_resp_id_0[LQ_DEPTH_LOG2-1:0] == i)) -
                                      (i_data_vld_1 & ~i_data_vld_cancel_1 & o_lq_broadside_info[i].vec_load & (i_data_resp_id_1[LQ_DEPTH_LOG2-1:0] == i)) -
                                      (i_data_vld_2 & ~i_data_vld_cancel_2 & o_lq_broadside_info[i].vec_load & (i_data_resp_id_2[LQ_DEPTH_LOG2-1:0] == i));
   
      assign lq_vecld_last_idx_sent_in[i] = ~lq_clear_tag_valid[i] & (lq_vecld_last_idx_sent[i] | ((i_vecld_elem_sent & i_vecld_idx_last) & (i_vecld_id == i)));
   
      tt_pipe_stage #(.WIDTH(6)) refcount_ff (
         .i_clk    (i_clk), 
         .i_reset_n(i_reset_n),
         .i_en     (1'b1),
         .i_d      ({lq_vecld_last_idx_sent_in[i], lq_refcount_in[i]}),
         .o_q      ({lq_vecld_last_idx_sent[i],   lq_refcount[i]})
      );
   end
      
   function automatic logic [(2*LD_DATA_WIDTH_BITS)-1:0] get_vecld_data;
      input logic [LQ_DATA_WIDTH-1:0]      lq_data;
      input logic [LD_DATA_WIDTH_BITS-1:0] return_data;
      input logic [DATA_REQ_ID_WIDTH-1:0]  return_id;
   
      logic                                    vecld_128;
      logic [$clog2(LD_DATA_WIDTH_BITS/8)-1:0] vecld_idx;
      logic [VLEN/8-1:0]                       vecld_mask;
      logic [1:0]                              vecld_sz;
      logic [$clog2(VLEN/8)-1:0]               vecld_addr;
   
      logic [VLEN/8-1:0]             vec_load_vrf_wrdata_byten;
      logic [LD_DATA_WIDTH_BITS-1:0] vec_load_vrf_wrdata, vec_load_vrf_wrdata_mask;
   
      begin
         vecld_idx   = return_id[LQ_DEPTH_LOG2+:$clog2(LD_DATA_WIDTH_BITS/8)];
         vecld_128   = lq_data[LQ_DATA_WIDTH-1];
         vecld_mask  = lq_data[(LD_DATA_WIDTH_BITS+$clog2(VLEN/8)+3)+:LD_DATA_WIDTH_BITS/8];
         vecld_sz    = lq_data[(LD_DATA_WIDTH_BITS+$clog2(VLEN/8))+:2];
         vecld_addr  = vecld_128 ? lq_data[LD_DATA_WIDTH_BITS+:$clog2(VLEN/8)] : '0;   
   
         // Byte mask (1 implies byte is masked)
         vec_load_vrf_wrdata_byten = vecld_128 ? 32'hffff_ffff :
                                                 ((vecld_sz[1:0] == 2'h3 ? 'hff :
                                                   vecld_sz[1:0] == 2'h2 ? 'hf  :
                                                   vecld_sz[1:0] == 2'h1 ? 'h3  :
                                                                           'h1   ) << (vecld_idx << vecld_sz[1:0]));
          
         // Convert return data to register format
         vec_load_vrf_wrdata[LD_DATA_WIDTH_BITS-1:0] = vecld_128 ? return_data : 
                                                                   (vecld_sz[1:0] == 2'h3 ? {LD_DATA_WIDTH_BITS/64{return_data[63:0]}} :
                                                                    vecld_sz[1:0] == 2'h2 ? {LD_DATA_WIDTH_BITS/32{return_data[31:0]}} :
                                                                    vecld_sz[1:0] == 2'h1 ? {LD_DATA_WIDTH_BITS/16{return_data[(16*vecld_addr[1])+:16]}} :
                                                                                            {LD_DATA_WIDTH_BITS/8 {return_data[(8*vecld_addr[1:0])+:8]}});
   
         // Final return data after gating with mask
         for (int i=0; i<(LD_DATA_WIDTH_BITS/8); i++) begin
            //vec_load_vrf_wrdata[(8*i)+:8]      = vec_load_aligned_data[(8*i)+:8];
            vec_load_vrf_wrdata_mask[(8*i)+:8] = {8{vec_load_vrf_wrdata_byten[i] & ~vecld_mask[i]}};
         end
            
         return {vec_load_vrf_wrdata_mask, vec_load_vrf_wrdata};
      end
      
   endfunction // get_vecld_data
      
end else begin: GenExclVec

   assign ret_vecld_vld_0 = '0;
   assign lq_fifo_vecld_write_datafn_0 = '0;
   
   assign ret_vecld_vld_1 = '0;
   assign lq_fifo_vecld_write_datafn_1 = '0;
   
   assign ret_vecld_vld_2 = '0;
   assign lq_fifo_vecld_write_datafn_2 = '0;

   assign lq_refcount = '0;
   assign lq_vecld_last_idx_sent = '0;
  
   assign lq_refcount_in = '0; 
end   
   
tt_cam_buffer #( 
  .TAG_WIDTH(LQ_TAG_WIDTH),                          
  .DATA_WIDTH(LQ_DATA_WIDTH),                     
  .ENTRIES(LQ_DEPTH), 
  .ENTRIES_LOG2(LQ_DEPTH_LOG2),
  .TAG_WRITE_PORTS(LQ_TAG_WR_PORTS), 
  .DATA_WRITE_PORTS(LQ_DATA_WR_PORTS), 
  .READ_PORTS(LQ_RD_PORTS), 
  .DISABLE_WRITE_MUX_ASSERTIONS(INCL_VEC), // For the vector core, we can have multiple write selects on, so we need to disable the internal assertions
  .CAM_PORTS(LQ_CAM_PORTS)                    
)
lq_fifo
(
  .i_clk                         (i_clk                   ),
  .i_reset_n                     (i_reset_n               ),
  
  .i_read_en                     (lq_fifo_read_en         ),  // enable signal for direct read of an entire buffer entry (tag+data) 
  .i_read_addr                   (lq_fifo_read_addr       ),  // address of the buffer entry to be read   
  .o_read_value                  (lq_fifo_read_value      ),  // output value of the buffer entry which is being read 

  .i_compare_en                  (lq_compare_en           ),  // will compare the given tag value against the existing entries (but not necessarily mux out the associated data when a match is found)  
  .i_compare_read_en             (lq_compare_read_en      ),  // when both the i_compare_en and i_compare_read_en are active, then the data from the matching entry will be muxed onto the o_cam_data bus  
  .i_compare_tag_value           (lq_compare_tag_value    ),  // tag value to compare  
  .i_compare_tag_value_mask      (lq_compare_tag_value_mask),  // tag value to mask the compare; setting the bit position will exclude the bit from the comparison  
  .i_compare_tag_valid_mask      (lq_compare_tag_valid_mask),  // masks the tag_valid bit from the compare  
  .o_cam_data_value              (                        ),  // data value associated with the matching tag value (only valid when the compare_read_en signal is active)  
  .o_compare_tag_hit             (                        ),  // bit vector which indicates which valid entry(s) were matched by the given tag_value  

  .i_write_tag_en                (lq_fifo_write_tag_en    ),  // enable signal for direct write of the tag portion of a buffer entry  
  .i_write_tag_addr              (lq_fifo_write_tag_addr  ),  // address of the buffer entry whose tag portion is to be written  
  .i_write_tag_value             (lq_fifo_write_tag_value ),  // value to be written into the tag portion of the buffer entry  

  .i_write_data_en               (lq_fifo_write_data_en   ),  // enable signal for direct write of the data portion of a buffer entry  
  .i_write_data_addr             (lq_fifo_write_data_addr ),  // address of the buffer entry whose data portion is to be written  
  .i_write_data_value            (lq_fifo_write_data_value),  // value to be written into the data portion of the buffer entry 

  .i_set_tag_valid               (lq_set_tag_valid        ),  // signal to set the tag valid bit of a buffer entry (would be expected to accompany a write of the tag to enable a later compare match)  
  .i_clear_tag_valid             (lq_clear_tag_valid      ),  // signal to clear the tag valid bit of a buffer entry (to prevent a later compare match); Clear is dominant over set!  
  .o_broadside_tag_valid         (lq_broadside_tag_valid  ),  // broadside output of the tag valid bit for all buffer entries 
  .o_broadside_tag_value         (lq_broadside_tag_value  ),  // broadside output of the tag values for all buffer entries 

  .i_set_data_valid              (lq_set_data_valid       ),  // signal to set the data valid bit of a buffer entry (would be expected to accompany a write of the data to enable a later check of the data valid status)  
  .i_clear_data_valid            (lq_clear_data_valid     ),  // signal to clear the data valid bit of a buffer entry (might accompnay the set_tag_valid when the data portion is not yet written); Clear is dominant over set!  
  .o_broadside_data_valid        (lq_broadside_data_valid ),   // broadside output of the data valid bit for all buffer entries
  .o_broadside_data_value        (lq_broadside_data_value )    // broadside output of the data values for all buffer entries
);
   
function automatic [31:0] align_load_data;
   input logic [31:0] load_data;
   input logic [1:0]  addr;
   input logic [2:0]  sz;

   logic 	halfw_sel;
   logic [1:0] 	byte_sel;
   logic [7:0] 	byte_data;
   logic [15:0] halfw_data;
   
   byte_sel  = addr[1:0];
   halfw_sel = addr[1];

   halfw_data = load_data[(halfw_sel*16)+:16];
   byte_data  = load_data[(byte_sel*8)+:8];
   /* verilator lint_off CASEINCOMPLETE */
   /* verilator lint_off CASEOVERLAP */
   casez(sz[2:0]) // synopsys full_case parallel_case
    3'b000: align_load_data  = {{24{byte_data[7]}},byte_data[7:0]};
    3'b001: align_load_data  = {{16{halfw_data[15]}},halfw_data[15:0]};
    3'b010: align_load_data  = load_data[31:0];
    3'b100: align_load_data  = {{24{1'b0}},byte_data[7:0]};
    3'b101: align_load_data  = {{16{1'b0}},halfw_data[15:0]};
    3'b110: align_load_data  = { 16'hffff, halfw_data[15:0]}; // NaN boxing for fp16
    default: align_load_data = 'x;
   endcase
   /* verilator lint_on CASEINCOMPLETE */
   /* verilator lint_on CASEOVERLAP */
endfunction      

`ifdef SIM
   // Can't set the data valid if tag valid is not high
   for (genvar i=0; i<LQ_DATA_WR_PORTS; i++) begin
      `ASSERT_COND_CLK(lq_fifo_write_data_en[i], lq_broadside_tag_valid[lq_fifo_write_data_addr[i]], "Setting LQ Data valid for port %0d but Tag valid is low", i);
   end
`endif

endmodule // tt_lq
