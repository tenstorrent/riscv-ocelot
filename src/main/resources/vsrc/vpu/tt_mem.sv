// See LICENSE.TT for license details.
`include "tt_briscv_pkg.vh"
module tt_mem #( parameter 
   LQ_DEPTH=8, 
   LQ_DEPTH_LOG2=3, 
   VLEN=128,
   ADDRWIDTH=48,
   FP_RF_WR_PORTS=1, 
   RF_WR_PORTS=1,
   LD_DATA_WIDTH_BITS=128,
   ST_DATA_WIDTH_BITS=128,
   DATA_REQ_ID_WIDTH=3,
   LOCAL_MEM_BYTE_ADDR_WIDTH=12,
   INCL_VEC = 0)
(
   input 				i_clk ,
   input 				i_reset_n ,

   // LQ signals
   // ID <--> MEM signals
   output logic 			o_mem_id_lqfull,
   output logic 			o_mem_id_lqempty,
   output logic 			o_mem_id_skidbuffull,
   output logic [LQ_DEPTH_LOG2-1:0] 	o_mem_id_lqnxtid,
   input logic 				i_id_mem_lqalloc,
   input 				tt_briscv_pkg::lq_info_s i_id_mem_lqinfo, // ID drives pc, instrn, regfile data on this
 
   // EX --> MEM signals
   input logic 				i_ex_mem_lqvld_1c,
   input logic [31:0] 			i_ex_mem_lqdata_1c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_ex_mem_lqid_1c,
   
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
   input logic [VLEN-1:0] 			i_vex_mem_lqdata_1c,
   input tt_briscv_pkg::csr_fp_exc      i_vex_mem_lqexc_1c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_vex_mem_lqid_1c,
   
   input logic 				i_vex_mem_lqvld_2c,
   input logic [VLEN-1:0] 			i_vex_mem_lqdata_2c,
   input tt_briscv_pkg::csr_fp_exc      i_vex_mem_lqexc_2c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_vex_mem_lqid_2c,
   
   input logic 				i_vex_mem_lqvld_3c,
   input logic [VLEN-1:0] 			i_vex_mem_lqdata_3c,
   input tt_briscv_pkg::csr_fp_exc      i_vex_mem_lqexc_3c,
   input logic [LQ_DEPTH_LOG2-1:0] 	i_vex_mem_lqid_3c,
   
   // From EX
   input tt_briscv_pkg::mem_skidbuf_s   i_ex_mem_payload,
   input 				i_ex_mem_vld,
 
   output wire 				o_mem_ex_rtr ,
//    output wire                          o_instret,

   // To FP Regfile
   output wire [FP_RF_WR_PORTS-1:0] 	  o_mem_fp_rf_wr ,
   output wire [FP_RF_WR_PORTS-1:0][4:0]  o_mem_fp_rf_wraddr ,
   output wire [FP_RF_WR_PORTS-1:0][63:0] o_mem_fp_rf_wrdata ,

   // To Regfile
   output wire [RF_WR_PORTS-1:0] 	o_mem_rf_wr ,
   output wire [ 5*RF_WR_PORTS-1:0] 	o_mem_rf_wraddr ,
   output wire [64*RF_WR_PORTS-1:0] 	o_mem_rf_wrdata ,

   // To vec
   output wire 				o_mem_vrf_wr ,
   output wire [ 4:0] 			o_mem_vrf_wraddr ,
   output wire [VLEN-1:0] 			o_mem_vrf_wrdata ,
   output wire [4:0] 			o_mem_vrf_wrexc ,

   output wire                o_vec_store_commit ,
   output wire                o_vec_nonstore_commit ,

   // To ID
   output wire 				o_mem_dst_vld ,  // forwarding control to ID
   output wire [LQ_DEPTH_LOG2-1:0] 	o_mem_dst_lqid,  // forwarding control to ID
   output wire [31:0]                   o_mem_fwd_data,  // forwarding control to ID
 
   output [6:0] 			o_mem_lq_op ,
   output 				o_mem_lq_commit ,

   // To/from memory system
   output wire [ADDRWIDTH-1:0] 		o_data_addr ,
   output wire [ST_DATA_WIDTH_BITS-1:0] o_data_wrdata,
   output wire 				o_data_128b, // 128b read (This signal is valid only for reads)
   output wire 				o_data_ordered,
   output wire [ 3:0] 			o_data_reqtype,
   output wire [VLEN/8-1:0] 		o_data_byten,
   output wire 				o_data_req,
   output [DATA_REQ_ID_WIDTH-1:0] 	o_data_req_id, 

   input                                i_dmem_brisc_memory_idle, 

   input 				i_data_req_rtr,
   input 				i_data_vld_0,
   input 				i_data_vld_cancel_0,
   input [DATA_REQ_ID_WIDTH-1:0] 	i_data_resp_id_0,
   input [63:0] 	                i_data_rddata_0,
   input 				i_data_vld_1,
   input 				i_data_vld_cancel_1,
   input [DATA_REQ_ID_WIDTH-1:0] 	i_data_resp_id_1,
   input [63:0] 	                i_data_rddata_1,
   input 				i_data_vld_2,
   input 				i_data_vld_cancel_2,
   input [DATA_REQ_ID_WIDTH-1:0] 	i_data_resp_id_2,
   input [63:0] 	                i_data_rddata_2,

   output 				tt_briscv_pkg::arr_lq_info_s o_lq_broadside_info,
   output [LQ_DEPTH-1:0][31:0] 		o_lq_broadside_data,
   output [LQ_DEPTH-1:0] 		o_lq_broadside_data_valid,
   output [LQ_DEPTH-1:0] 		o_lq_broadside_valid,
 
   output 				o_mem_store,
   output 				o_mem_load,
   output [2:0]		                o_mem_size,
   output                               o_mem_last,

   // Misc
   input [31:0] 			i_reset_pc,
   output reg 				o_trap,
   output logic       o_lq_empty

);

localparam ACTUAL_WB_DEPTH = 1;

`ifdef SIM
  `ifdef MUTATE_RISCV_CORE_FIFO_MIN   
     localparam WB_DEPTH = 2;
  `elsif MUTATE_RISCV_CORE_FIFO_MAX
     localparam WB_DEPTH = (2 * ACTUAL_WB_DEPTH);
  `else   
     localparam WB_DEPTH = ACTUAL_WB_DEPTH;
  `endif
`else   
   localparam WB_DEPTH = ACTUAL_WB_DEPTH;
`endif
   
logic                           data_wren;

logic                           mem_load_valid, mem_store_valid;
logic                           mem_load, mem_store;
logic [2:0]                     mem_instrn;
logic                           mem_amo, mem_fence, mem_ordered;
logic [4:0]                     mem_amotype;
logic [1:0]                     mem_ldst_addr;
logic [2:0]                     mem_ldst_sz;
logic [ADDRWIDTH-1:0]           mem_addr;
logic [ST_DATA_WIDTH_BITS/8-1:0]mem_byten;
logic [ST_DATA_WIDTH_BITS-1:0]  mem_store_val;
logic                           mem_tx_valid, mem_lq_valid;
logic [LQ_DEPTH_LOG2-1:0]       mem_lqid;
logic                           mem_vecld_vld, mem_vecldst_128;
logic [$clog2(VLEN/8)-1:0]      mem_vecldst_idx;
logic                           mem_vecldst_idx_last;
   
logic                           wb_empty, wb_full, wb_full_raw, wb_hit;
logic                           lq_empty, lq_full, lq_full_raw;
logic                           lq_data_ready;
logic                           lq_mem_vec_load;
logic                           lq_ready_to_commit;
   
logic                           lq_rden;
logic [LQ_DEPTH_LOG2-1:0]       lq_rdid;
tt_briscv_pkg::lq_info_s        lq_rdinfo;
logic [2:0] 		        lq_rdldstsz;
logic [1:0] 		        lq_rdmemaddr;
logic [VLEN-1:0]                lq_rddata;
logic [4:0]                     lq_rdexc;

logic                           lq_fwdvld;
logic [LQ_DEPTH_LOG2-1:0]       lq_fwdid;
logic [31:0]                    lq_fwddata;
   
logic                              wb_store_valid;
logic [ST_DATA_WIDTH_BITS-1:0]     wb_store_data;
logic [ADDRWIDTH-1:0]              wb_addr;
logic [(ST_DATA_WIDTH_BITS/8)-1:0] wb_byten;
logic                              wb_ordered;
logic [2:0]                        wb_ldst_sz;

logic                           load_gnt, store_gnt;
   
// *********************** //
// Mem stage skid buffer            //
// *********************** //
localparam SKIDBUF_WIDTH = $bits(tt_briscv_pkg::mem_skidbuf_s);
localparam SKIDBUF_DEPTH = 1;
logic skidbuf_rtr, skidbuf_rts;
logic skidbuf_empty, skidbuf_full;
logic skidbuf_wren, skidbuf_rden;
logic skidbuf_bypass_allow;
logic mem_ready;

logic                          skidbuf_lqvld_1c;
logic [$clog2(VLEN/8)-1:0]     skidbuf_lqaddr_1c;
logic [2:0]                    skidbuf_lqsz_1c;
logic                          skidbuf_lqvecld128_1c;
logic [VLEN/8-1:0]             skidbuf_lqmask_1c;
logic [LD_DATA_WIDTH_BITS-1:0] skidbuf_lqdata_1c;
logic [LQ_DEPTH_LOG2-1:0]      skidbuf_lqid_1c;
   
tt_briscv_pkg::mem_skidbuf_s skidbuf_wrdata, skidbuf_rddata;

// This indicates there are no stalls and entry from ex or skidbuf can go to lq   
assign mem_ready = (mem_store_valid | mem_load_valid | ~(mem_load | mem_fence | mem_store)) & ~i_fp_ex_mem_lqvld_1c & ~i_vex_mem_lqvld_1c & 
                   ~(mem_load_valid & ~load_gnt) & ~(mem_load_valid & load_gnt & ~i_data_req_rtr);

assign skidbuf_full = ~skidbuf_rtr;

// Skid buffer <--> LQ signals
assign skidbuf_lqvld_1c           = mem_lq_valid & mem_ready;
assign skidbuf_lqaddr_1c          = skidbuf_rddata.mem_addr[$clog2(VLEN/8)-1:0];
assign skidbuf_lqsz_1c[2:0]       = skidbuf_rddata.mem_sz[2:0];
assign skidbuf_lqvecld128_1c      = skidbuf_rddata.vecldst_128;
assign skidbuf_lqmask_1c          = skidbuf_rddata.vecldst_byte_mask;
assign skidbuf_lqdata_1c          = skidbuf_rddata.vecldst_vld ? LD_DATA_WIDTH_BITS'(skidbuf_rddata.mem_store_data) : LD_DATA_WIDTH_BITS'(skidbuf_rddata.mem_alu_result);
assign skidbuf_lqid_1c            = skidbuf_rddata.mem_lqid;
   
always_comb begin
   skidbuf_wrdata              = i_ex_mem_payload;
   skidbuf_wrdata.mem_load     = i_ex_mem_payload.mem_load | i_ex_mem_payload.mem_amo;
   skidbuf_wrdata.mem_lqid     = i_ex_mem_lqid_1c;
   skidbuf_wrdata.mem_tx_valid = i_ex_mem_vld;
   skidbuf_wrdata.mem_lq_valid = i_ex_mem_lqvld_1c;
end

// Read data
assign mem_load                 = skidbuf_rddata.mem_load;
assign mem_store                = skidbuf_rddata.mem_store;
assign mem_ldst_addr[1:0]       = skidbuf_rddata.mem_addr[1:0];
assign mem_ldst_sz[2:0]         = skidbuf_rddata.mem_sz[2:0];
assign mem_store_val            = skidbuf_rddata.mem_store_data;    

assign mem_tx_valid             = skidbuf_rddata.mem_tx_valid & skidbuf_rts;
assign mem_lq_valid             = skidbuf_rddata.mem_lq_valid & skidbuf_rts;
assign mem_ordered              = skidbuf_rddata.mem_ordered;
assign mem_fence                = skidbuf_rddata.mem_fence;
assign mem_amo                  = skidbuf_rddata.mem_amo;
assign mem_amotype              = skidbuf_rddata.mem_amotype;
assign mem_addr[ADDRWIDTH-1:0]  = skidbuf_rddata.mem_addr;
assign mem_byten                = skidbuf_rddata.mem_byten;
assign mem_lqid                 = skidbuf_rddata.mem_lqid;
logic mem_vecldst_vld; 
assign mem_vecldst_vld          = skidbuf_rddata.vecldst_vld; //MM Nov 5 2021: Fix implicit wire error 
assign mem_vecldst_128          = skidbuf_rddata.vecldst_128;
assign mem_vecldst_idx          = skidbuf_rddata.vecldst_idx;
assign mem_vecldst_idx_last     = skidbuf_rddata.vecldst_idx_last;

tt_skid_buffer #(.WIDTH(SKIDBUF_WIDTH)) skidbuf_fifo
(
  .i_clk            (i_clk                    ),
  .i_reset_n        (i_reset_n                ),

  .i_rts            (i_ex_mem_vld             ),
  .o_rtr            (skidbuf_rtr              ), 
  .i_pld            (skidbuf_wrdata           ),

  .o_rts            (skidbuf_rts              ),
  .i_rtr            (mem_ready                ),
  .o_pld            (skidbuf_rddata           )
 
);
 
// *********************** //
// Write buffer            //
// *********************** //
//localparam WB_WIDTH = ST_DATA_WIDTH_BITS+32+(ST_DATA_WIDTH_BITS/8)+1;
//tt_store_fifo #( .DATA_WIDTH(ST_DATA_WIDTH_BITS), 
//                 .DEPTH(WB_DEPTH), 
//                 .LOCAL_MEM_BYTE_ADDR_WIDTH(LOCAL_MEM_BYTE_ADDR_WIDTH),
//                 .INCL_VEC(INCL_VEC),
//                 .BYPASS_PATH(0)) wb_fifo 
//(
//  .i_clk            (i_clk                    ),
//  .i_reset_n        (i_reset_n                ),
//  .i_rden           (i_data_req_rtr & store_gnt),
//  .i_wren           (mem_store_valid & (|mem_byten)),
//  .i_exit_merge_mode((mem_fence | mem_amo | wb_hit) & mem_tx_valid),
//  .i_store_data     (mem_store_val            ),
//  .i_store_addr     (mem_addr                 ),
//  .i_store_byten    (mem_byten                ),
//  .i_store_ordered  (mem_ordered              ),
//  .i_store_ldst_sz  (mem_ldst_sz              ),
//
//  .i_cmpen          (mem_load                 ),
//  .i_cmpaddr        (mem_addr                 ),
//  .i_cmpbyten       (mem_byten                ),
// 
//  .o_store_valid    (wb_store_valid           ),
//  .o_store_data     (wb_store_data            ),
//  .o_store_addr     (wb_addr                  ),
//  .o_store_byten    (wb_byten                 ),
//  .o_store_ordered  (wb_ordered               ),
//  .o_store_ldst_sz  (wb_ldst_sz               ),
//
//  .o_cmphit         (wb_hit                   ),
//  .o_empty          (wb_empty                 ),
//  .o_full           (wb_full_raw              ),
//
//);

assign wb_store_valid = mem_store_valid;
assign wb_store_data  = mem_store_val;
assign wb_addr        = mem_addr;
assign wb_byten       = mem_byten;
assign wb_ordered     = mem_ordered;
assign wb_ldst_sz     = mem_ldst_sz;
assign wb_hit         = 1'b0;
assign wb_empty       = !wb_store_valid;
assign wb_full_raw    = !i_data_req_rtr;

assign wb_full = wb_full_raw;


// *********************** //
// Load Queue              //
// *********************** //

wire [2:0]     lq_mem_instrn;
wire           lq_mem_rf_wr_flag;
logic          lq_mem_fp_rf_wr_flag;
wire           lq_mem_vrf_wr_flag;
wire [4:0]     lq_mem_rf_wraddr;
logic [31:0]   lq_mem_pc;
logic [31:0]   lq_sim_ex_mem_instrn;

tt_lq #(.LQ_DEPTH(LQ_DEPTH), 
        .LQ_DEPTH_LOG2(LQ_DEPTH_LOG2), 
        .DATA_REQ_ID_WIDTH(DATA_REQ_ID_WIDTH),
        .LD_DATA_WIDTH_BITS(LD_DATA_WIDTH_BITS),
        .VLEN(VLEN),
        .INCL_VEC(INCL_VEC)) lq_fifo
(
   .i_clk            (i_clk                    ),
   .i_reset_n        (i_reset_n                ),

   .i_bypass_disable (1'b0),

   .i_vecld_elem_sent(mem_vecldst_vld & mem_load_valid & ~mem_fence & mem_ready),
   .i_vecld_idx_last (mem_vecldst_idx_last),
   .i_vecld_id       (mem_lqid                 ),
 
   // ID <--> MEM signals
   .o_mem_id_lqnxtid(o_mem_id_lqnxtid),
   .i_id_mem_lqalloc(i_id_mem_lqalloc),
   .i_id_mem_lqinfo(i_id_mem_lqinfo),
 
   // Skid buffer signals
   .i_skidbuf_lqvld_1c(skidbuf_lqvld_1c),
   .i_skidbuf_lqsz_1c(skidbuf_lqsz_1c),
   .i_skidbuf_lqaddr_1c(skidbuf_lqaddr_1c),
   .i_skidbuf_lqmask_1c(skidbuf_lqmask_1c),
   .i_skidbuf_lqvecld128_1c(skidbuf_lqvecld128_1c),
   .i_skidbuf_lqdata_1c(skidbuf_lqdata_1c),
   .i_skidbuf_lqid_1c(skidbuf_lqid_1c),

   // EX 2 cycle signals
   .i_ex_mem_lqvld_2c(i_ex_mem_lqvld_2c),
   .i_ex_mem_lqdata_2c(i_ex_mem_lqdata_2c),
   .i_ex_mem_lqid_2c(i_ex_mem_lqid_2c),
 
   // FP EX 2 cycle signals
   .i_fp_ex_mem_lqvld_1c(i_fp_ex_mem_lqvld_1c),
   .i_fp_ex_mem_lqdata_1c(i_fp_ex_mem_lqdata_1c),
   .i_fp_ex_mem_lqid_1c(i_fp_ex_mem_lqid_1c),
 
   .i_fp_ex_mem_lqvld_2c(i_fp_ex_mem_lqvld_2c),
   .i_fp_ex_mem_lqdata_2c(i_fp_ex_mem_lqdata_2c),
   .i_fp_ex_mem_lqid_2c(i_fp_ex_mem_lqid_2c),
 
   // VEX 1/2/3 cycle signals
   .i_vex_mem_lqvld_1c(i_vex_mem_lqvld_1c),
   .i_vex_mem_lqdata_1c(i_vex_mem_lqdata_1c),
   .i_vex_mem_lqexc_1c(i_vex_mem_lqexc_1c),
   .i_vex_mem_lqid_1c(i_vex_mem_lqid_1c),

   .i_vex_mem_lqvld_2c(i_vex_mem_lqvld_2c),
   .i_vex_mem_lqdata_2c(i_vex_mem_lqdata_2c),
   .i_vex_mem_lqexc_2c(i_vex_mem_lqexc_2c),
   .i_vex_mem_lqid_2c(i_vex_mem_lqid_2c),

   .i_vex_mem_lqvld_3c(i_vex_mem_lqvld_3c),
   .i_vex_mem_lqdata_3c(i_vex_mem_lqdata_3c),
   .i_vex_mem_lqexc_3c(i_vex_mem_lqexc_3c),
   .i_vex_mem_lqid_3c(i_vex_mem_lqid_3c),

   // Load return data
   .i_data_vld_0(i_data_vld_0),
   .i_data_vld_cancel_0(i_data_vld_cancel_0),
   .i_data_resp_id_0(i_data_resp_id_0),
   .i_data_rddata_0(i_data_rddata_0),
 
   .i_data_vld_1(i_data_vld_1),
   .i_data_vld_cancel_1(i_data_vld_cancel_1),
   .i_data_resp_id_1(i_data_resp_id_1),
   .i_data_rddata_1(i_data_rddata_1),
 
   .i_data_vld_2(i_data_vld_2),
   .i_data_vld_cancel_2(i_data_vld_cancel_2),
   .i_data_resp_id_2(i_data_resp_id_2),
   .i_data_rddata_2(i_data_rddata_2),
 
   .lq_full(lq_full_raw),
   .lq_empty(lq_empty),

   // These come from broadside data due to CombLoop
   .o_lq_data_ready(lq_data_ready),
   .o_lq_mem_vec_load(lq_mem_vec_load),
   .o_lq_mem_load(),

   // LQ Read signals
   .i_lq_rden(lq_rden),
   .o_lq_rdid(lq_rdid),
   .o_lq_rdinfo(lq_rdinfo),
   .o_lq_rdmemaddr(lq_rdmemaddr),
   .o_lq_rdldstsz(lq_rdldstsz),
   .o_lq_rddata(lq_rddata),
   .o_lq_rdexc(lq_rdexc),

   .o_lq_fwdvld(lq_fwdvld),
   .o_lq_fwdid(lq_fwdid),
   .o_lq_fwddata(lq_fwddata),

   // Broadside data
   .o_lq_broadside_info(o_lq_broadside_info),
   .o_lq_broadside_data(o_lq_broadside_data),
   .o_lq_broadside_valid(o_lq_broadside_valid),
   .o_lq_broadside_data_valid(o_lq_broadside_data_valid)
);

assign lq_ready_to_commit = lq_data_ready;
assign lq_rden = lq_ready_to_commit;

assign lq_mem_vrf_wr_flag     =   lq_rdinfo.vrf_wr_flag;        
assign lq_mem_fp_rf_wr_flag   =   lq_rdinfo.fp_rf_wr_flag;       
assign lq_mem_rf_wr_flag      =   lq_rdinfo.rf_wr_flag;          
assign lq_mem_pc              =   lq_rdinfo.pc           [31:0]; 
assign lq_sim_ex_mem_instrn   =   lq_rdinfo.sim_instrn   [31:0]; 
assign lq_mem_rf_wraddr       =   lq_rdinfo.rf_wraddr    [4:0];  

assign o_vec_store_commit =  lq_rden                 &&
                            !lq_rdinfo.rf_wr_flag    &&
                            !lq_rdinfo.vrf_wr_flag   &&
                            !lq_rdinfo.fp_rf_wr_flag;
   
assign o_vec_nonstore_commit =  lq_rden                 &&
                               !o_vec_store_commit       &&
                                lq_rdinfo.vrf_wr_flag;
   

assign lq_full = lq_full_raw | ((~lq_empty)); 
assign o_lq_empty = lq_empty;

// ******************************************************************************************************************** //
// Ratio arbiter between loads and WB stores to avoid WB starvation when endless loads (as in tight polling loop)       //
// ******************************************************************************************************************** //
// Fence/AMO will wait for WB to be empty
assign mem_store_valid = mem_tx_valid & mem_store & ~i_vex_mem_lqvld_1c & ~wb_full & (~mem_ordered);
assign mem_load_valid = mem_tx_valid & (mem_load | mem_fence) & ~i_fp_ex_mem_lqvld_1c & ~i_vex_mem_lqvld_1c & 
                           ~(wb_hit & mem_load) & (wb_empty | ~(mem_fence | mem_amo | mem_ordered)) &
                           (i_dmem_brisc_memory_idle | ~(mem_fence | mem_amo | mem_ordered));

reg [3:0] shift_arb;
wire load_token = |shift_arb[3:1];
wire store_token = shift_arb[0];
always @(posedge i_clk) begin
  if(~i_reset_n) begin
    shift_arb <= 4'b0001;
  end
  else if (((load_gnt & load_token) | (store_gnt & store_token)) & i_data_req_rtr) begin
    shift_arb <= {shift_arb[2:0],shift_arb[3]};
  end
end
assign store_gnt = (~wb_empty) & (store_token | ~mem_load_valid); //data_arb_gnt[0];
assign load_gnt  = mem_load_valid & (load_token | wb_empty); //|data_arb_gnt[3:1];

// ************************ //
// Backpressure to EX       //
// ************************ //
assign o_mem_ex_rtr = ~skidbuf_full;

assign o_mem_id_lqfull = lq_full;
assign o_mem_id_lqempty = lq_empty;
assign o_mem_id_skidbuffull = skidbuf_full;
   
// ************************ //
// Output request interface //
// ************************ //
assign o_data_addr        = (load_gnt ? mem_addr : wb_addr);
assign o_data_wrdata      = (mem_load & mem_amo & wb_empty) ? mem_store_val : wb_store_data; // AMO will wait for the wb to be empty (use wb_empty for timing)
assign data_wren          = store_gnt;
assign o_data_ordered     = (store_gnt ? wb_ordered : mem_ordered); 
assign o_data_reqtype     = data_wren ? `BRISCV_DATA_REQTYPE_WR : (({4{mem_load & ~mem_amo}} & `BRISCV_DATA_REQTYPE_RD)  |
                                                                   ({4{mem_fence}} & `BRISCV_DATA_REQTYPE_FENCE) |
                                                                   ({4{mem_load & mem_amo & (mem_amotype == 5'b00001)}} & `BRISCV_DATA_REQTYPE_AMOSWAP) |  //amoswap
                                                                   ({4{mem_load & mem_amo & (mem_amotype != 5'b00001)}} & {1'b1,mem_amotype[4:2]}));
assign o_data_byten       = store_gnt ? wb_byten : mem_byten;
assign o_data_req         = (mem_load_valid | wb_store_valid); //load_gnt | store_gnt;
assign o_data_128b        = '0;
assign o_data_req_id      = DATA_REQ_ID_WIDTH'({2'h0,mem_vecldst_idx, mem_lqid});

assign o_mem_store        = store_gnt;
assign o_mem_load         = load_gnt;
assign o_mem_size         = store_gnt ? wb_ldst_sz : mem_ldst_sz;
assign o_mem_last         = mem_vecldst_idx_last;

assign o_mem_lq_op      = lq_sim_ex_mem_instrn[6:0];
assign o_mem_lq_commit  = lq_ready_to_commit;
//TODO: Check this is the intention. Looks like the below line is for EBREAK, but it was breaking for every system
// instruction, qualified with funct3 and funct7, rs2 (ECALL vs EBREAK)

// ************************ //
// Write back to registers  //
// ************************ //
assign o_mem_rf_wr             = lq_rden & lq_mem_rf_wr_flag;
assign o_mem_rf_wraddr[4:0]    = lq_mem_rf_wraddr[4:0];
assign o_mem_rf_wrdata[63:0]   = lq_rddata[63:0]; 

assign o_mem_vrf_wr            = lq_rden & lq_mem_vrf_wr_flag;   
assign o_mem_vrf_wraddr[4:0]   = lq_mem_rf_wraddr[4:0];    
assign o_mem_vrf_wrdata[VLEN-1:0] = lq_rddata[VLEN-1:0];
assign o_mem_vrf_wrexc         = lq_rdinfo.vec_load ? '0 : lq_rdexc;

//fixme_msalvi asserrt one hot between vrf and rf wr and fp
//Use FP Port0 for now for ex results. Will use port 1 when dual issue is enabled
assign o_mem_fp_rf_wr[0]        = lq_rden & lq_mem_fp_rf_wr_flag;
assign o_mem_fp_rf_wraddr[0][4:0]  = lq_mem_rf_wraddr;
assign o_mem_fp_rf_wrdata[0][63:0] = lq_rddata[63:0];

//FP RF Port 1 zeroed out for now
//assign o_mem_fp_rf_wr[1] = 0;
//assign o_mem_fp_rf_wraddr[1][4:0] = 'h0;
//assign o_mem_fp_rf_wrdata[1][31:0] = 'h0;

assign o_mem_dst_vld  = lq_fwdvld;
assign o_mem_dst_lqid = lq_fwdid;
assign o_mem_fwd_data = lq_fwddata;
//assign o_mem_dst_vld        = lq_rden & (lq_mem_rf_wr_flag | lq_mem_fp_rf_wr_flag);
//assign o_mem_dst_lqid       = lq_rdid;
//assign o_mem_fwd_data       = lq_mem_rf_wr_flag ? o_mem_rf_wrdata : o_mem_fp_rf_wrdata[0][31:0];
   
`ifdef SIM
// ************************************ //
// Commit log generator and trap setter //
// ************************************ //
//`define BRISCV_LOG
`ifdef BRISCV_LOG

/* verilator lint_off UNUSED */
reg [63:0] mem_ascii_instrn;
/* verilator lint_on UNUSED */

tt_ascii_instrn_decode ascii_decode
(
  .i_instrn      (lq_sim_ex_mem_instrn),
  .o_ascii_instrn(mem_ascii_instrn )
);

reg [31:0] instrn_counter;

reg [63:0] disp_pc;
reg [31:0] disp_instrn;
reg [ 5:0] disp_reg_index;
reg [63:0] disp_reg_val;
/* verilator lint_off WIDTH */
always @* begin
  disp_pc        = lq_mem_pc;
  disp_instrn    = lq_sim_ex_mem_instrn;
  disp_reg_index = o_mem_rf_wraddr;
  disp_reg_val   = $signed(o_mem_rf_wrdata);
end
/* verilator lint_on WIDTH */
integer File;
integer clock;
initial begin
`ifdef VCS
  wait (i_reset_n === 1'b1);
`endif
  File = $fopen($sformatf("trace_%x.dat", i_reset_pc),"w");
  clock = 0;
end

always @(posedge i_clk) begin
  clock <= clock + 1;
  if(lq_rden) begin // An instruction is committing
    if(o_mem_rf_wr && (|disp_reg_index)) begin // It's a register-modifying instruction
      $fwrite(File, "3 0x%x (0x%x) x%d 0x%x\n", disp_pc, disp_instrn, disp_reg_index, disp_reg_val);
    end
    else begin // not register-modifying
      $fwrite(File, "3 0x%x (0x%x)\n", disp_pc, disp_instrn);
      //$fwrite(File, "3 0x%x (0x%x) clock: %d\n", disp_pc, disp_instrn, clock);
    end
  end
end

always @(posedge i_clk) begin
  if(!i_reset_n) begin
    instrn_counter <= 'd0;
  end
  else begin
    if(lq_rden)
    begin
      instrn_counter <= instrn_counter + 1'b1;
    end
  end
end

////////////////
reg [15:0] rd_cnt                         ;
reg [15:0] wr_cnt                         ;
reg [ 7:0] rd_gnt_latency_cnt             ;
reg [ 7:0] wr_gnt_latency_cnt             ;
reg [ 7:0] rdvld_latency_cnt              ;
reg [15:0] wr_gnt_latency_histogram   [0:255];
reg [15:0] rd_gnt_latency_histogram   [0:255];
reg [15:0] rdvld_latency_histogram [0:255];
reg [16*256-1:0] wr_gnt_latency_histogram_row;
reg [16*256-1:0] rd_gnt_latency_histogram_row;
reg [16*256-1:0] rdvld_latency_histogram_row;

integer w;
always @* begin
   for(w=0;w<256;w=w+1) begin
     wr_gnt_latency_histogram[w] = wr_gnt_latency_histogram_row[16*w +: 16];
     rd_gnt_latency_histogram[w] = rd_gnt_latency_histogram_row[16*w +: 16];
     rdvld_latency_histogram[w]  = rdvld_latency_histogram_row[16*w +: 16];
   end
end

always @(posedge i_clk) begin
  if(!i_reset_n) begin
    rd_cnt                  <= 'd0;
    wr_cnt                  <= 'd0;
    rd_gnt_latency_cnt      <= 'd0;
    wr_gnt_latency_cnt      <= 'd0;
    rdvld_latency_cnt       <= 'd0;
    wr_gnt_latency_histogram_row <= 'd0;
    rd_gnt_latency_histogram_row <= 'd0;
    rdvld_latency_histogram_row <= 'd0;
  end
  else begin
    if(o_data_req & (o_data_reqtype == `BRISCV_DATA_REQTYPE_WR) & (!i_data_req_rtr)) begin
      wr_gnt_latency_cnt <= wr_gnt_latency_cnt + 1;
    end
    else begin
      if(o_data_req & (o_data_reqtype == `BRISCV_DATA_REQTYPE_WR) & i_data_req_rtr) begin
        wr_cnt <= wr_cnt + 1;
        wr_gnt_latency_cnt <= 0;
        wr_gnt_latency_histogram_row[wr_gnt_latency_cnt*16 +: 16] <= wr_gnt_latency_histogram[wr_gnt_latency_cnt] + 1;
      end
      else begin
        wr_gnt_latency_cnt <= 0;
      end
    end

    if(o_data_req & (!(o_data_reqtype == `BRISCV_DATA_REQTYPE_WR)) & (!i_data_req_rtr)) begin
      rd_gnt_latency_cnt <= rd_gnt_latency_cnt + 1;
    end
    else begin
      if(o_data_req & (!(o_data_reqtype == `BRISCV_DATA_REQTYPE_WR)) & i_data_req_rtr) begin
        rd_cnt <= rd_cnt + 1;
        rd_gnt_latency_cnt <= 0;
        rd_gnt_latency_histogram_row[wr_gnt_latency_cnt*16 +: 16] <= rd_gnt_latency_histogram[wr_gnt_latency_cnt] + 1;
      end
      else begin
        rd_gnt_latency_cnt <= 0;
      end
    end

    if(o_data_req & (!(o_data_reqtype == `BRISCV_DATA_REQTYPE_WR)) & i_data_req_rtr & (!i_data_vld_0)) begin
      rdvld_latency_cnt <= 1;
    end
    else begin
      if(i_data_vld_0) begin
        rdvld_latency_cnt <= 0;
        if(o_data_req & (!(o_data_reqtype == `BRISCV_DATA_REQTYPE_WR)) & i_data_req_rtr) begin
          rdvld_latency_histogram_row[0 +: 16] <= rdvld_latency_histogram[0] + 1;
        end
        else begin
          rdvld_latency_histogram_row[rdvld_latency_cnt*16 +: 16] <= rdvld_latency_histogram[rdvld_latency_cnt] + 1;
        end
      end
      else begin
        rdvld_latency_cnt <= rdvld_latency_cnt + 1;
      end
    end 
  end
end


`endif
`endif /* SIM */

// Set trap indicator
always @(posedge i_clk) begin
  if(~i_reset_n) begin
    o_trap <= 1'b0;
  end
  else begin
    if(lq_rden & (lq_sim_ex_mem_instrn[6:0] == 7'b1110011) & (lq_sim_ex_mem_instrn[14:12] == 3'b0)) begin // detect ECALL or EBRK
      o_trap <= 1'b1;
    end
  end  
end


endmodule
