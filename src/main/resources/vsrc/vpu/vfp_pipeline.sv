// See LICENSE.TT for license details.
module vfp_pipeline #(parameter 
   LOCAL_MEM_BYTE_ADDR_WIDTH = 12,
   INCL_VEC = 1,
   VLEN = 256,
   ADDRWIDTH = 48,
   LD_DATA_WIDTH_BITS = VLEN,
   ST_DATA_WIDTH_BITS = VLEN,
   LQ_DEPTH=8,
   LQ_DEPTH_LOG2=$clog2(LQ_DEPTH),
   DATA_REQ_ID_WIDTH=INCL_VEC ? (LQ_DEPTH_LOG2+$clog2(VLEN/8)+2) : LQ_DEPTH_LOG2,
   INCL_FP = 1
)
(
   input  logic         i_clk,
   input  logic         i_reset,
   input  logic [$clog2(VLEN+1)-1:0]   i_csr_vl,
   input  logic [2:0]   i_csr_vsew,
   input  logic [2:0]   i_csr_vlmul,
   input  logic [1:0]   i_csr_vxrm,
   input  logic [2:0]   i_csr_frm,

   // IF Interface
   input  logic         i_if_instrn_rts,
   output logic         o_id_instrn_rtr,
   input  logic [31:0]  i_if_instrn,
   input  logic [31:0]  i_if_pc,
   input  logic [63:0]  i_rf_vex_p0,
   input  logic [63:0]  i_rf_vex_p1,
   input  logic [63:0]  i_fprf_vex_p0,

   // Commit Interface
   output logic                           o_instrn_commit_valid,
   output logic [VLEN*8-1:0]              o_instrn_commit_data,
   output logic [7:0]                     o_instrn_commit_mask,
   output logic [4:0]                     o_instrn_commit_fflags,

   output logic                           o_sat_csr,

   // Memory Interface
   output logic                           o_data_req           ,
   output logic [ADDRWIDTH-1:0]           o_data_addr          ,
   output logic [ST_DATA_WIDTH_BITS/8-1:0] o_data_byten        ,
   output logic [ST_DATA_WIDTH_BITS-1:0]  o_wr_data            ,
   output logic [DATA_REQ_ID_WIDTH-1:0]   o_data_req_id        ,    
   output logic                           o_mem_load           ,
   output logic [2:0]                     o_mem_size           ,
   output logic                           o_mem_last           ,
   input  logic                           i_data_req_rtr       ,
   input  logic                           i_rd_data_vld_0      ,
   input  logic [DATA_REQ_ID_WIDTH-1:0]   i_rd_data_resp_id_0  , 
   input  logic [63:0]                    i_rd_data_0          ,
   input  logic                           i_rd_data_vld_1      ,
   input  logic [DATA_REQ_ID_WIDTH-1:0]   i_rd_data_resp_id_1  , 
   input  logic [63:0]                    i_rd_data_1          ,

   output logic                           o_vecldst_autogen_store,
   output logic                           o_id_mem_lq_done,
   output logic                           o_id_ex_units_rts,
   output logic [VLEN-1:0]                o_vs3_rddata,
   output logic                           o_lq_empty
);

   localparam                                          FLEN    = 32;
   localparam                                          FP_RF_RD_PORTS  = 4;
   localparam                                          FP_RF_WR_PORTS  = 2;
   localparam                                          FP_RF_ADDR_WIDTH    = 5;
   localparam                                          EXP_WIDTH   = 8;
   localparam                                          MAN_WIDTH   = 23;
   localparam                                          RS1_IDX = 0;
   localparam                                          RS2_IDX = 1;
   localparam                                          RS3_IDX = 2;

   /////////
   // MEM signals
   wire                     mem_dst_vld;  // forwarding control to ID
   wire [LQ_DEPTH_LOG2-1:0] mem_dst_lqid; // forwarding control to ID
   wire [31:0]              mem_fwd_data; // forwarding control to ID
   wire                     mem_ex_rtr           ;
   wire [ 6:0]              mem_lq_op            ; // mem status to ID
   wire                     mem_lq_commit        ;
      
   tt_briscv_pkg::arr_lq_info_s  lq_broadside_info;
   logic [LQ_DEPTH-1:0][31:0]    lq_broadside_data;
   logic [LQ_DEPTH-1:0]          lq_broadside_valid;
   logic [LQ_DEPTH-1:0]          lq_broadside_data_valid;
   logic                         lq_empty;

   /////////
   // ID signals
   wire                             id_rtr               ;
   wire [4:0]                       id_type              ;
   wire                             id_rf_wr_flag        ;
   wire [ 4:0]                      id_rf_wraddr         ;
   wire                             id_fp_rf_wr_flag     ;
   wire [ 4:0]                      id_fp_rf_wraddr      ;
   wire [31:0]                      id_immed_op          ;
   wire                             id_ex_rts            ;
   wire                             id_ex_units_rts      ;
   wire [LQ_DEPTH_LOG2-1:0]         id_ex_lqid;
   wire [4:0]                       id_ex_Zb_instr       ;
   wire                             id_ex_vecldst        ;
   wire [VLEN-1:0]                  vmask_rddata        ;
   wire [VLEN-1:0]                  vs2_rddata          ; 
   wire [VLEN-1:0]                  vs3_rddata          ; 
   wire [31:0]                      id_ex_pc             ;
   wire                             id_vex_rts           ;
   wire                             vex_id_rtr           ;
   wire [31:0]                      id_ex_instrn         ;
   wire                             v_vm                 ;
   logic [4:0]                      iterate_addrp0,iterate_addrp1,iterate_addrp2       ;
   logic                            ignore_lmul,ignore_dstincr,ignore_srcincr;
   wire [3:0]                       id_fp_fmt             ;
   tt_briscv_pkg::vec_autogen_s     id_vec_autogen;
   tt_briscv_pkg::vecldst_autogen_s id_ex_vecldst_autogen;
   wire                             id_ex_instdisp       ;

   logic                     id_replay;              // From id of tt_id.v
   logic [LQ_DEPTH_LOG2-1:0] id_vex_lqid;          // From id of tt_id.v
   logic                     vex_id_incr_addrp2;     // From vecu of tt_vec.v

   logic                           i_rd_data_vld_2      ;
   logic [DATA_REQ_ID_WIDTH-1:0]   i_rd_data_resp_id_2  ; 

   // Vector signals
   logic              vrf_p0_rden, vrf_p1_rden, vrf_p2_rden;
   logic [VLEN  -1:0] vrf_vm0_rddata;
   logic [VLEN  -1:0] vrf_p0_rddata, vrf_p1_rddata, vrf_p2_rddata;
   logic [       4:0] vrf_p0_rdaddr, vrf_p1_rdaddr, vrf_p2_rdaddr;
   logic              mem_rf_wr;
   logic [       4:0] mem_rf_wraddr;
   logic [      63:0] mem_rf_wrdata;
   logic              mem_fp_rf_wr;
   logic [       4:0] mem_fp_rf_wraddr;
   logic [      63:0] mem_fp_rf_wrdata;
   logic              mem_vrf_wr;
   logic [       4:0] mem_vrf_wraddr;
   logic [VLEN  -1:0] mem_vrf_wrdata;
   logic [VLEN*8-1:0] mem_vrf_wrdata_reg;
   logic [VLEN*8-1:0] mem_vrf_wrdata_nxt;
   logic [       4:0] mem_vrf_wrexc;
   logic [       4:0] mem_vrf_wrexc_reg;
   logic [       4:0] mem_vrf_wrexc_nxt;
   logic [       7:0] mem_vrf_wrmask_reg;
   logic [       7:0] mem_vrf_wrmask_nxt;
   
   logic         vec_store_commit;
   logic         vec_nonstore_commit;

   // LQ signals
   // ID <--> MEM signals
   logic                     mem_fe_lqfull;
   logic                     mem_fe_lqempty;
   logic                     mem_fe_skidbuffull;
   logic [LQ_DEPTH_LOG2-1:0] mem_id_lqnxtid;
   logic [LQ_DEPTH_LOG2-1:0] mem_id_lqnxtid_r;
   logic                     id_mem_lqalloc;
   logic                     id_mem_lq_done;
   tt_briscv_pkg::lq_info_s  id_mem_lqinfo;
    
   // EX --> ID signals
   wire                     ex_dst_vld_1c;
   wire [LQ_DEPTH_LOG2-1:0] ex_dst_lqid_1c;
   wire [31:0]              ex_fwd_data_1c;
      
   wire                     ex_dst_vld_2c;
   wire [LQ_DEPTH_LOG2-1:0] ex_dst_lqid_2c;
   wire [31:0]              ex_fwd_data_2c;
   
   // EX --> MEM signals
   tt_briscv_pkg::mem_skidbuf_s ex_mem_payload;
   wire                         ex_mem_vld           ;

   logic                        ex_mem_lqvld_1c;
   logic [31:0]                 ex_mem_lqdata_1c;
   logic [LQ_DEPTH_LOG2-1:0]    ex_mem_lqid_1c;
      
   logic                        ex_mem_lqvld_2c;
   logic [31:0]                 ex_mem_lqdata_2c;
   logic [LQ_DEPTH_LOG2-1:0]    ex_mem_lqid_2c;
      
   // VEX --> MEM signals
   logic                     vex_mem_lqvld_1c;
   logic [VLEN-1:0]          vex_mem_lqdata_1c;
   logic [LQ_DEPTH_LOG2-1:0] vex_mem_lqid_1c;
   tt_briscv_pkg::csr_fp_exc vex_mem_lqexc_1c;
                          
   logic                     vex_mem_lqvld_2c;
   logic [VLEN-1:0]          vex_mem_lqdata_2c;
   logic [LQ_DEPTH_LOG2-1:0] vex_mem_lqid_2c;
   tt_briscv_pkg::csr_fp_exc vex_mem_lqexc_2c;
                          
   logic                     vex_mem_lqvld_3c;
   logic [VLEN-1:0]          vex_mem_lqdata_3c;
   logic [LQ_DEPTH_LOG2-1:0] vex_mem_lqid_3c;
   tt_briscv_pkg::csr_fp_exc vex_mem_lqexc_3c;

   logic [63:0]  rf_vex_p0_reg;
   logic [63:0]  rf_vex_p1_reg;
   logic [63:0]  fprf_vex_p0_reg;

   logic [63:0]  rf_vex_p0_sel;
   logic [63:0]  fprf_vex_p0_sel;

   always_ff @(posedge i_clk) begin
      if (i_if_instrn_rts &&
          o_id_instrn_rtr   ) begin
         rf_vex_p0_reg   <= i_rf_vex_p0;
         rf_vex_p1_reg   <= i_rf_vex_p1;
         fprf_vex_p0_reg <= i_fprf_vex_p0;
      end
   end

   assign rf_vex_p0_sel   = (i_if_instrn_rts && o_id_instrn_rtr) ? i_rf_vex_p0   : rf_vex_p0_reg;
   assign rf_vex_p1_sel   = (i_if_instrn_rts && o_id_instrn_rtr) ? i_rf_vex_p1   : rf_vex_p1_reg;
   assign fprf_vex_p0_sel = (i_if_instrn_rts && o_id_instrn_rtr) ? i_fprf_vex_p0 : fprf_vex_p0_reg;

   tt_briscv_pkg::csr_to_id ex_id_csr;
   assign ex_id_csr.vgsrc   = 0;
   assign ex_id_csr.v_vsew  = i_csr_vsew;
   assign ex_id_csr.v_lmul  = i_csr_vlmul;
   assign ex_id_csr.v_vlmax = '0;
   assign ex_id_csr.v_vl    = i_csr_vl;

   tt_id
   #(
     .LQ_DEPTH(LQ_DEPTH),
     .LQ_DEPTH_LOG2(LQ_DEPTH_LOG2), 
     .EXP_WIDTH(EXP_WIDTH),
     .MAN_WIDTH(MAN_WIDTH),
     .FLEN(FLEN),
     .VLEN(VLEN),
     .FP_RF_RD_PORTS(FP_RF_RD_PORTS),
     .INCL_VEC(INCL_VEC),
     .INCL_FP(INCL_FP)
   ) id
   (
      .i_clk                                 (i_clk             ),    
      .i_reset_n                             (!i_reset), 

      .i_ex_id_csr                           (ex_id_csr),             

      .i_if_instrn                           (i_if_instrn[31:0]),       
      .i_if_pc                               (i_if_pc[31:0]),           
      .i_if_instrn_rts                       (i_if_instrn_rts),    
      .o_id_instrn_rtr                       (o_id_instrn_rtr),    

      .o_id_type                             (id_type),          
      .o_id_immed_op                         (id_immed_op),     

      // Vector Interface
      .o_id_vex_rts                          (id_vex_rts),            
      .i_vex_id_rtr                          (vex_id_rtr),            
      .o_vec_autogen                         (id_vec_autogen),   
      .o_id_vex_lqid                         (id_vex_lqid), 
      .i_vex_id_incr_addrp2                  (vex_id_incr_addrp2),    
      .o_v_vm                                (v_vm              ),    

      // EX Interface
      .o_id_ex_rts                           (id_ex_rts),             
      .i_ex_rtr                              (ex_id_rtr         ),    
      .o_id_ex_pc                            (id_ex_pc),        
      .o_id_ex_instrn                        (id_ex_instrn),    
      .o_id_ex_lqid                          (id_ex_lqid[LQ_DEPTH_LOG2-1:0]), 
      .o_id_ex_vecldst                       (id_ex_vecldst),         
      .o_id_ex_Zb_instr                      (id_ex_Zb_instr[4:0]),   
      .o_id_ex_units_rts                     (id_ex_units_rts),       
      .o_id_ex_instdisp                      (id_ex_instdisp),
      .o_vecldst_autogen                     (id_ex_vecldst_autogen), 
      .i_ex_bp_mispredict                    ('0),      
      .i_ex_dst_vld_1c                       (ex_dst_vld_1c),         
      .i_ex_dst_lqid_1c                      (ex_dst_lqid_1c), 
      .i_ex_fwd_data_1c                      (ex_fwd_data_1c),  
      .i_ex_dst_vld_2c                       (ex_dst_vld_2c),         
      .i_ex_dst_lqid_2c                      (ex_dst_lqid_2c), 
      .i_ex_fwd_data_2c                      (ex_fwd_data_2c),  
      .o_fwd_p0_reg                          (),      
      .o_fwd_p1_reg                          (),      

      // FP Interface
      .o_id_fp_ex0_rts                       (),         
      .i_fp_ex0_id_rtr                       (1'b1),         
      .o_id_fp_ex0_lqid                      (), 
      .o_ex_autogen                          (),    
      .o_fp_autogen                          (),   
      .o_fp_fwd_sign_reg                     (),     
      .o_fp_fwd_zero_reg                     (),     
      .o_fp_fwd_nan_reg                      (),      
      .o_fp_fwd_inf_reg                      (),      
      .o_fp_fwd_exp_reg                      (),      
      .o_fp_fwd_man_reg                      (),      
      .o_fp_rf_p3_reg                        (),          
      .i_fp_ex_dst_vld_1c                    ('0),      
      .i_fp_ex_dst_lqid_1c                   ('0), 
      .i_fp_ex_fwd_data_1c                   ('0), 
      .i_fp_ex_dst_vld_2c                    ('0),      
      .i_fp_ex_dst_lqid_2c                   ('0), 
      .i_fp_ex_fwd_data_2c                   ('0), 

      // Integer RegFile Interface
      .o_rf_p0_rden                          (),    
      .o_rf_p0_rdaddr                        (),    
      .o_rf_p1_rden                          (),    
      .o_rf_p1_rdaddr                        (),    
      .o_rf_p2_rden                          (),    
      .o_rf_p2_rdaddr                        (),    
      .o_rf_wr_flag                          (id_rf_wr_flag     ),    
      .o_rf_wraddr                           (id_rf_wraddr      ),    

      // FP RegFile Interface
      .i_fp_rf_rd_ret_reg                    ('0),      
      .i_fp_rf_sign                          ('0),     
      .i_fp_rf_zero                          ('0),     
      .i_fp_rf_nan                           ('0),      
      .i_fp_rf_inf                           ('0),      
      .i_fp_rf_exp                           ('0),      
      .i_fp_rf_man                           ('0),      
      .i_rf_p0_reg                           ('0),       
      .i_rf_p1_reg                           ('0),       
      .o_fp_rf_wr_flag                       (id_fp_rf_wr_flag  ),    
      .o_fp_rf_wraddr                        (id_fp_rf_wraddr   ),    

      // Mem Interface
      .i_lq_broadside_info                   (lq_broadside_info),     
      .o_id_mem_lqinfo                       (id_mem_lqinfo),         
      .o_id_replay                           (id_replay),             
      .o_id_mem_lqalloc                      (id_mem_lqalloc),        
      .o_id_mem_lq_done                      (id_mem_lq_done),        
      .i_mem_dst_vld                         (mem_dst_vld          ), 
      .i_mem_dst_lqid                        (mem_dst_lqid         ), 
      .i_mem_fwd_data                        (mem_fwd_data         ), 
      .i_mem_lq_op                           (mem_lq_op[6:0]),        
      .i_mem_lq_commit                       (mem_lq_commit),         
      .i_lq_broadside_data                   (lq_broadside_data),     
      .i_lq_broadside_valid                  (lq_broadside_valid), 
      .i_lq_broadside_data_valid             (lq_broadside_data_valid), 

      // Misc
      .i_iterate_addrp0                      (iterate_addrp0),   
      .i_iterate_addrp1                      (iterate_addrp1),   
      .i_iterate_addrp2                      (iterate_addrp2),   
      .i_ignore_lmul                         (ignore_lmul),           
      .i_ignore_dstincr                      (ignore_dstincr),        
      .i_ignore_srcincr                      (ignore_srcincr),        
      .i_mem_fe_lqfull                       (mem_fe_lqfull),         
      .i_mem_fe_lqempty                      (mem_fe_lqempty),
      .i_mem_fe_skidbuffull                  (mem_fe_skidbuffull),    
      .i_mem_id_lqnxtid                      (mem_id_lqnxtid[LQ_DEPTH_LOG2-1:0])
   );  

   //////////
   // EX
   tt_briscv_pkg::csr_to_vec ex_vec_csr;
   assign ex_vec_csr.v_vsew =  i_csr_vsew;
   assign ex_vec_csr.v_lmul =  i_csr_vlmul;
   assign ex_vec_csr.v_vxrm =  i_csr_vxrm;
   assign ex_vec_csr.v_vl   =  i_csr_vl;
   assign o_vecldst_autogen_store = id_ex_vecldst_autogen.store;
   assign o_id_mem_lq_done = id_mem_lq_done;
   assign o_id_ex_units_rts = id_ex_units_rts;
   assign o_vs3_rddata = vs3_rddata;
   assign o_lq_empty = lq_empty;

   tt_ex
   #(.INCL_VEC(INCL_VEC),
     .VLEN(VLEN),
     .LQ_DEPTH_LOG2(LQ_DEPTH_LOG2)
   ) ex
   (
      .i_clk               (i_clk             ),
      .i_reset_n           (!i_reset),
      .i_sat_csr           ('0),
      .i_ex_vec_csr        (ex_vec_csr),             
      // From ID
      .i_if_ex_deco        ('0    ),
      .i_if_ex_predicted   ('0    ),
      .i_if_ex_nextinstr   ('0    ),
      .i_exc_fp_ex_update  ('0    ),
      .i_exc_vfp_update    ('0    ),
      
      // From ID
      .i_id_ex_rts         (id_ex_rts         ),
      .o_ex_id_rtr         (ex_id_rtr         ),
      .i_id_type           (id_type           ),
      .i_id_rf_wr_flag     (id_rf_wr_flag     ),
      .i_id_rf_wraddr      (id_rf_wraddr      ),
      .i_id_fp_rf_wr_flag  (id_fp_rf_wr_flag  ),
      .i_id_fp_rf_wraddr   (id_fp_rf_wraddr   ),
      .i_id_immed_op       (id_immed_op       ),
      .i_id_ex_lqid        (id_ex_lqid        ),
      .i_id_ex_pc          (id_ex_pc          ),
      .i_id_ex_instrn      (id_ex_instrn      ),
      .i_id_ex_vecldst     (id_ex_vecldst     ),
      .i_id_ex_Zb_instr    (id_ex_Zb_instr    ),
      .i_id_ex_units_rts   (id_ex_units_rts   ),
      .i_id_ex_instdisp    (id_ex_instdisp    ),
      
      .i_id_ex_vecldst_autogen(id_ex_vecldst_autogen),
      
      // From RF
      .i_rf_p0_reg         (rf_vex_p0_reg),
      .i_rf_p1_reg         (rf_vex_p1_reg),
      .i_fp_rf_p3_reg      ('0),
      
      // From VRF
      .i_vmask_rddata      (vmask_rddata      ),
      .i_vs2_rddata        (vs2_rddata        ),
      .i_vs3_rddata        (vs3_rddata        ),
      
      // From MEM
      .i_mem_ex_rtr        (mem_ex_rtr        ),
      
      // To ID and IF
      .o_ex_bp_fifo_pop    (),
      .o_ex_is_some_branch (),
      .o_ex_branch_taken   (),
      
      .o_ex_bp_mispredict         (),
      .o_ex_bp_mispredict_not_br  (),
      .o_ex_bp_pc                 (),
      .o_ex_id_csr                (),
      .o_ex_vec_csr               (),
      
      .o_ex_dst_vld_1c        (ex_dst_vld_1c        ),
      .o_ex_dst_lqid_1c       (ex_dst_lqid_1c       ),
      .o_ex_fwd_data_1c       (ex_fwd_data_1c       ),
      .o_ex_dst_vld_2c        (ex_dst_vld_2c        ),
      .o_ex_dst_lqid_2c       (ex_dst_lqid_2c       ),
      .o_ex_fwd_data_2c       (ex_fwd_data_2c       ),
      
      // To MEM
      .o_ex_mem_lqvld_1c      (ex_mem_lqvld_1c     ),
      .o_ex_mem_lqid_1c       (ex_mem_lqid_1c      ),
      .o_ex_mem_lqdata_1c     (ex_mem_lqdata_1c    ),
      
      .o_ex_mem_lqvld_2c      (ex_mem_lqvld_2c     ),
      .o_ex_mem_lqid_2c       (ex_mem_lqid_2c      ),
      .o_ex_mem_lqdata_2c     (ex_mem_lqdata_2c    ),
      
      .o_ex_mem_payload       (ex_mem_payload      ),
      .o_ex_mem_vld           (ex_mem_vld          ),
      
      // Debug
      .i_reset_pc             ('0)
   );

   assign vrf_p2_rden    = id_vec_autogen.rf_rden2; // FIXME | ex_vec_csr.v_lmul[2];
   assign vrf_p1_rden    = id_vec_autogen.rf_rden1;
   assign vrf_p0_rden    = id_vec_autogen.rf_rden0;

   // VRF read data for EX (V-LD/ST)
   assign vmask_rddata  = vrf_vm0_rddata;
   assign vs2_rddata    = vrf_p1_rddata;
   assign vs3_rddata    = vrf_p2_rddata;

   assign vrf_p0_rdaddr =                                  id_vec_autogen.rf_addrp0;
   assign vrf_p1_rdaddr = id_vec_autogen.rf_rd_p2_is_rs2 ? id_vec_autogen.rf_addrp2
                                                         : id_vec_autogen.rf_addrp1;
   assign vrf_p2_rdaddr = id_vec_autogen.rf_rd_p2_is_rs2 ? id_vec_autogen.rf_addrp1
                                                         : id_vec_autogen.rf_addrp2;

   tt_vec_regfile #(.VLEN(VLEN))
   regfile
   (
      .i_clk               (i_clk),
      .i_reset_n           (!i_reset),
      // Outputs
      .o_rddata_0a         ({vrf_p2_rddata,vrf_p1_rddata,vrf_p0_rddata}),
      .o_dstmask_0a        (),
      .o_vm0_0a            (vrf_vm0_rddata),
      // Inputs
      .i_rden_0a           ({vrf_p2_rden,vrf_p1_rden,vrf_p0_rden}),
      .i_wren_0a           (mem_vrf_wr),
      .i_rdaddr_0a         ({vrf_p2_rdaddr, vrf_p1_rdaddr, vrf_p0_rdaddr}),
      .i_wraddr_0a         (mem_vrf_wraddr),
      .i_wrdata_0a         (mem_vrf_wrdata)
   );

   tt_vec #(.VLEN(VLEN),
            .XLEN(64  ) )
   vecu
   (
      .i_clk                 (i_clk),                 
      .i_reset_n             (!i_reset), 
      .i_ex_vec_csr          (ex_vec_csr),             
      .i_csr_frm             (i_csr_frm),
      .i_v_vm                (v_vm),                  
      .i_id_vec_autogen      (id_vec_autogen),        
      .o_sat_csr             (o_sat_csr),
      // ID Interface
      .i_id_vex_rts          (id_vex_rts),            
      .o_vex_id_rtr          (vex_id_rtr),            
      .i_id_ex_vecldst       (id_ex_vecldst),         
      .i_id_ex_instrn        (id_ex_instrn),    
      .i_id_replay           (id_replay),             
      .i_id_type             (id_type),          
      .o_vex_id_incr_addrp2  (vex_id_incr_addrp2),    
      .o_iterate_addrp0      (iterate_addrp0),   
      .o_iterate_addrp1      (iterate_addrp1),   
      .o_iterate_addrp2      (iterate_addrp2),   
      .o_ignore_lmul         (ignore_lmul),           
      .o_ignore_dstincr      (ignore_dstincr),        
      .o_ignore_srcincr      (ignore_srcincr),        
      // RegFile Interface
      .i_rf_vex_p0           (rf_vex_p0_sel),       
      .i_fprf_vex_p0         (fprf_vex_p0_sel),
      .i_vrf_p0_rddata       (vrf_p0_rddata),  
      .i_vrf_p1_rddata       (vrf_p1_rddata),  
      .i_vrf_p2_rddata       (vrf_p2_rddata),  
      .i_vrf_vm0_rddata      (vrf_vm0_rddata),
      // Mem Interface
      .o_vex_mem_lqvld_1c    (vex_mem_lqvld_1c),      
      .o_vex_mem_lqdata_1c   (vex_mem_lqdata_1c), 
      .o_vex_mem_lqexc_1c    (vex_mem_lqexc_1c),      
      .o_vex_mem_lqid_1c     (vex_mem_lqid_1c), 
      .o_vex_mem_lqvld_2c    (vex_mem_lqvld_2c),      
      .o_vex_mem_lqdata_2c   (vex_mem_lqdata_2c), 
      .o_vex_mem_lqexc_2c    (vex_mem_lqexc_2c),      
      .o_vex_mem_lqid_2c     (vex_mem_lqid_2c), 
      .o_vex_mem_lqvld_3c    (vex_mem_lqvld_3c),      
      .o_vex_mem_lqdata_3c   (vex_mem_lqdata_3c), 
      .o_vex_mem_lqexc_3c    (vex_mem_lqexc_3c),      
      .o_vex_mem_lqid_3c     (vex_mem_lqid_3c), 
      .i_mem_vrf_wr          (mem_vrf_wr),            
      .i_mem_vrf_wraddr      (mem_vrf_wraddr),   
      .i_mem_vrf_wrdata      (mem_vrf_wrdata), 
      .i_mem_ex_rtr          (mem_ex_rtr)            
   );    

   logic                        o_data_128b          ;  // 128b read (This signal is valid only for reads)
   logic                        o_data_ordered       ;
   logic [ 3:0]                 o_data_reqtype       ;
   logic                           o_data_req_prequal;
   logic                           i_dmem_brisc_memory_idle; 
   logic                           o_mem_store;

   tt_mem 
   #(
     .LQ_DEPTH(LQ_DEPTH), 
     .LQ_DEPTH_LOG2(LQ_DEPTH_LOG2),
     .VLEN(VLEN),
     .ADDRWIDTH(ADDRWIDTH),
     .LD_DATA_WIDTH_BITS(LD_DATA_WIDTH_BITS),
     .ST_DATA_WIDTH_BITS(ST_DATA_WIDTH_BITS),  
     .LOCAL_MEM_BYTE_ADDR_WIDTH(LOCAL_MEM_BYTE_ADDR_WIDTH),
     .DATA_REQ_ID_WIDTH(DATA_REQ_ID_WIDTH),
     .INCL_VEC(INCL_VEC)
   ) mem
   (
      .i_clk                (i_clk  ),
      .i_reset_n            (!i_reset),

      // From EX
      .i_ex_mem_payload     (ex_mem_payload      ),
      .i_ex_mem_vld         (ex_mem_vld          ),

      .i_ex_mem_lqvld_1c    (ex_mem_lqvld_1c     ),
      .i_ex_mem_lqid_1c     (ex_mem_lqid_1c      ),
      .i_ex_mem_lqdata_1c   (ex_mem_lqdata_1c    ),

      .i_ex_mem_lqvld_2c    (ex_mem_lqvld_2c     ),
      .i_ex_mem_lqid_2c     (ex_mem_lqid_2c      ),
      .i_ex_mem_lqdata_2c   (ex_mem_lqdata_2c    ),

      .i_fp_ex_mem_lqvld_1c ('0),
      .i_fp_ex_mem_lqid_1c  ('0),
      .i_fp_ex_mem_lqdata_1c('0),

      .i_fp_ex_mem_lqvld_2c ('0),
      .i_fp_ex_mem_lqid_2c  ('0),
      .i_fp_ex_mem_lqdata_2c('0),

      .i_vex_mem_lqvld_1c   (vex_mem_lqvld_1c),
      .i_vex_mem_lqdata_1c  (vex_mem_lqdata_1c),
      .i_vex_mem_lqexc_1c   (vex_mem_lqexc_1c),
      .i_vex_mem_lqid_1c    (vex_mem_lqid_1c),             
      
      .i_vex_mem_lqvld_2c   (vex_mem_lqvld_2c),
      .i_vex_mem_lqdata_2c  (vex_mem_lqdata_2c),
      .i_vex_mem_lqexc_2c   (vex_mem_lqexc_2c),
      .i_vex_mem_lqid_2c    (vex_mem_lqid_2c),             
      
      .i_vex_mem_lqvld_3c   (vex_mem_lqvld_3c),
      .i_vex_mem_lqdata_3c  (vex_mem_lqdata_3c),
      .i_vex_mem_lqexc_3c   (vex_mem_lqexc_3c),
      .i_vex_mem_lqid_3c    (vex_mem_lqid_3c),             

      // To EX
      .o_mem_ex_rtr         (mem_ex_rtr),

      // To Regfile
      .o_mem_rf_wr          (mem_rf_wr),
      .o_mem_rf_wraddr      (mem_rf_wraddr),
      .o_mem_rf_wrdata      (mem_rf_wrdata),

      //To FP RF
      .o_mem_fp_rf_wr       (mem_fp_rf_wr),
      .o_mem_fp_rf_wraddr   (mem_fp_rf_wraddr),
      .o_mem_fp_rf_wrdata   (mem_fp_rf_wrdata),

      .o_mem_vrf_wr         (mem_vrf_wr),
      .o_mem_vrf_wraddr     (mem_vrf_wraddr),
      .o_mem_vrf_wrdata     (mem_vrf_wrdata),
      .o_mem_vrf_wrexc      (mem_vrf_wrexc),

      .o_vec_store_commit   (vec_store_commit),
      .o_vec_nonstore_commit(vec_nonstore_commit),

      // To ID
      .i_id_mem_lqalloc     (id_mem_lqalloc    ),
      .i_id_mem_lqinfo      (id_mem_lqinfo     ),
      .o_mem_id_lqfull      (mem_fe_lqfull     ),
      .o_mem_id_lqempty     (mem_fe_lqempty     ),
      .o_mem_id_skidbuffull (mem_fe_skidbuffull),
      .o_mem_id_lqnxtid     (mem_id_lqnxtid    ),
 
      .o_mem_dst_vld        (mem_dst_vld       ),
      .o_mem_dst_lqid       (mem_dst_lqid      ),
      .o_mem_fwd_data       (mem_fwd_data      ),
      .o_mem_lq_op          (mem_lq_op         ),
      .o_mem_lq_commit      (mem_lq_commit     ),

      .o_lq_broadside_info      (lq_broadside_info ),
      .o_lq_broadside_data      (lq_broadside_data ),
      .o_lq_broadside_valid     (lq_broadside_valid),
      .o_lq_broadside_data_valid(lq_broadside_data_valid),

      // To/from memory system
      .i_dmem_brisc_memory_idle('0), 
      .o_data_addr              (o_data_addr       ),
      .o_data_wrdata            (o_wr_data         ),
      .o_data_reqtype           (o_data_reqtype    ),
      .o_data_ordered           (o_data_ordered    ),
      .o_data_byten             (o_data_byten      ), // IMPROVE: rename this signal to reflect its use for loads and stores
      .o_data_req               (o_data_req_prequal),
      .o_data_req_id            (o_data_req_id     ),
      .o_data_128b              (o_data_128b       ),
      .i_data_req_rtr           (i_data_req_rtr    ),
      .i_data_vld_0             (i_rd_data_vld_0   ),
      .i_data_vld_cancel_0      ('0),
      .i_data_resp_id_0         (i_rd_data_resp_id_0),
      .i_data_rddata_0          (i_rd_data_0       ),
      .i_data_vld_1             (i_rd_data_vld_1   ),
      .i_data_vld_cancel_1      ('0),
      .i_data_resp_id_1         (i_rd_data_resp_id_1),
      .i_data_rddata_1          (i_rd_data_1       ),
      .i_data_vld_2             (i_rd_data_vld_2   ),
      .i_data_vld_cancel_2      ('0),
      .i_data_resp_id_2         (i_rd_data_resp_id_2),
      .i_data_rddata_2          ('0),

      .o_mem_store              (o_mem_store ),
      .o_mem_load               (o_mem_load),
      .o_mem_size               (o_mem_size),
      .o_mem_last               (o_mem_last_raw),
      // Trap
      .i_reset_pc               ('0),
      .o_trap                   (  ),
      .o_lq_empty               (lq_empty)
   );

   logic [LQ_DEPTH     -1:0] lq_last;
   logic [LQ_DEPTH_LOG2-1:0] lq_rd_ptr;

   assign o_instrn_commit_valid       =   mem_rf_wr || mem_fp_rf_wr ||
                                         (vec_nonstore_commit && lq_last[lq_rd_ptr]);
   assign o_instrn_commit_data[VLEN*8-1:64] =  {VLEN*8-64{vec_nonstore_commit}} & mem_vrf_wrdata_nxt[VLEN*8-1:64];
   assign o_instrn_commit_data[      63: 0] =        ({64{vec_nonstore_commit}} & mem_vrf_wrdata_nxt[      63: 0]) |
                                                     ({64{mem_rf_wr          }} & mem_rf_wrdata     [      63: 0]) |
                                                     ({64{mem_fp_rf_wr       }} & mem_fp_rf_wrdata  [      63: 0]);
   assign o_instrn_commit_fflags            =        ({ 5{vec_nonstore_commit}} & mem_vrf_wrexc_nxt [       4: 0]);
   assign o_instrn_commit_mask              =                                     mem_vrf_wrmask_nxt;
   assign o_data_req =  o_data_req_prequal && 
                       !(  o_mem_load   &&
                         ~|o_data_byten   );

   assign o_mem_last = o_mem_last_raw     &&
                       lq_last[o_data_req_id[LQ_DEPTH_LOG2-1:0]];

   always_ff @(posedge i_clk, posedge i_reset) begin
      if (i_reset) begin
         lq_last          <= '0;
         lq_rd_ptr        <= '0;
         mem_id_lqnxtid_r <= '0;
      end else begin
         if (mem_lq_commit) begin
            lq_rd_ptr <= lq_rd_ptr + 1;
         end

         if (id_mem_lqalloc) begin
            lq_last[mem_id_lqnxtid] <= id_mem_lq_done;
            mem_id_lqnxtid_r        <= mem_id_lqnxtid;
         end else
         if (id_mem_lq_done) begin
            lq_last[mem_id_lqnxtid_r] <= 1'b1;
         end
      end
   end


   logic [2:0] lmul_cnt;

   always_ff @(posedge i_clk, posedge i_reset) begin
      if (i_reset) begin
         mem_vrf_wrdata_reg <= '0;
         mem_vrf_wrmask_reg <= '0;
         mem_vrf_wrexc_reg  <= '0;
      end else begin
         if (o_instrn_commit_valid) begin
            mem_vrf_wrdata_reg <= '0;
            mem_vrf_wrmask_reg <= '0;
            mem_vrf_wrexc_reg  <= '0;
         end else
         if (mem_vrf_wr) begin
            mem_vrf_wrdata_reg <= mem_vrf_wrdata_nxt;
            mem_vrf_wrmask_reg <= mem_vrf_wrmask_nxt;
            mem_vrf_wrexc_reg  <= mem_vrf_wrexc_nxt;
         end
      end
   end

   always_comb begin
      mem_vrf_wrdata_nxt = mem_vrf_wrdata_reg;
      mem_vrf_wrmask_nxt = mem_vrf_wrmask_reg;
      mem_vrf_wrexc_nxt  = mem_vrf_wrexc_reg;

      mem_vrf_wrdata_nxt[VLEN*lmul_cnt +: VLEN] = mem_vrf_wrdata;
      mem_vrf_wrmask_nxt[     lmul_cnt        ] = |i_csr_vl;
      mem_vrf_wrexc_nxt                        |= mem_vrf_wrexc;
   end

   always_ff @(posedge i_clk, posedge i_reset) begin
      if (i_reset) begin
         lmul_cnt <= '0;
      end else begin
         if (o_instrn_commit_valid) begin
            lmul_cnt <= '0;
         end else
         if (mem_vrf_wr) begin
            lmul_cnt <= lmul_cnt + 1;
         end
      end
   end

   always_ff @(posedge i_clk, posedge i_reset) begin
      if (i_reset) begin
         i_rd_data_vld_2 <= '0;
         i_rd_data_resp_id_2 <= '0;
      end else begin
         i_rd_data_vld_2 <=   o_data_req_prequal &&
                              i_data_req_rtr     &&
                           (  o_mem_load   &&
                             ~|o_data_byten   );
         i_rd_data_resp_id_2 <= o_data_req_id;
      end
   end


endmodule
