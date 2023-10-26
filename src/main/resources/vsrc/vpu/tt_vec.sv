// See LICENSE.TT for license details.
`include "autogen_defines.h"
`include "briscv_defines.h"
`include "tt_briscv_pkg.vh"
module tt_vec #(parameter
   VLEN=256,
   XLEN=64
)
(
              input                                           i_clk,
              input                                           i_reset_n,

              input                                           tt_briscv_pkg::csr_t i_csr,
              input                                           i_v_vm, //Instr [bit 25]  
                    
              input                                           tt_briscv_pkg::vec_autogen_s i_id_vec_autogen, //Decode signal bundle

              //From Mem; used to both write in the RF and also to bypass the write.
              input                                           i_mem_vrf_wr,
              input [4:0]                                     i_mem_vrf_wraddr,
              input [VLEN-1:0]                                i_mem_vrf_wrdata,
              input                                           i_mem_ex_rtr ,
              
              // From VRF
              input [VLEN-1:0]                                i_vrf_p0_rddata,
              input [VLEN-1:0]                                i_vrf_p1_rddata,
              input [VLEN-1:0]                                i_vrf_p2_rddata,
              input [VLEN-1:0]                                i_vrf_vm0_rddata,
         
              output logic                                    o_vex_mem_lqvld_1c,
              output logic [VLEN-1:0]                         o_vex_mem_lqdata_1c,
              output logic [tt_briscv_pkg::LQ_DEPTH_LOG2-1:0] o_vex_mem_lqid_1c,
              output tt_briscv_pkg::csr_fp_exc                o_vex_mem_lqexc_1c,
              
              output logic                                    o_vex_mem_lqvld_2c,
              output logic [VLEN-1:0]                         o_vex_mem_lqdata_2c,
              output logic [tt_briscv_pkg::LQ_DEPTH_LOG2-1:0] o_vex_mem_lqid_2c,
              output tt_briscv_pkg::csr_fp_exc                o_vex_mem_lqexc_2c,
              
              output logic                                    o_vex_mem_lqvld_3c,
              output logic [VLEN-1:0]                         o_vex_mem_lqdata_3c,
              output logic [tt_briscv_pkg::LQ_DEPTH_LOG2-1:0] o_vex_mem_lqid_3c,
              output tt_briscv_pkg::csr_fp_exc                o_vex_mem_lqexc_3c,

              output logic                                    o_sat_csr,

              //From ID
              input                                           i_id_vex_rts ,
              input                                           i_id_ex_vecldst,
              input [31:0]                                    i_id_ex_instrn ,
              input                                           i_id_replay,
              input [4:0]                                     i_id_type, 
              
              //To ID
              output                                          o_vex_id_rtr,
              output logic [4:0]                              o_iterate_addrp0,
              output logic [4:0]                              o_iterate_addrp1,
              output logic [4:0]                              o_iterate_addrp2,
              output logic                                    o_vex_id_incr_addrp2,
              output logic                                    o_ignore_lmul,
              output logic                                    o_ignore_dstincr,
              output logic                                    o_ignore_srcincr, //This should be set for instructions that are processing mask bits.(eg: vmand.mm, viota vid ,vmsof...)
              //From Int RF
              input [XLEN-1:0]                                    i_rf_vex_p0, //int to vrf moves; note this align with 0a, and read pre flop.
              
              input [XLEN-1:0]                                    i_fprf_vex_p0  //fp to vrf moves; note this align with 0a, and read pre flop.                   
              );

   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   logic                sat_csr_2a;             // From idp of tt_vec_idp.v
   logic [XLEN-1:0]         slide_shft_amt_1a;      // From idp of tt_vec_idp.v
   // End of automatics
   logic                slide_dwn_1a,slide_dwn_2a,slide1_up_1a,slide1_up_2a;
    logic               vex_mem_rts;
   logic [31:0]         vex_mem_pc;
   logic [31:0]         vex_mem_instrn;
   logic                vex_mem_rf_wr_flag_1c,vex_mem_frf_to_scalar_1c,vex_mem_rf_to_scalar_1c;
   logic [VLEN-1:0]        src1_0a;                
   logic [VLEN-1:0]        src2_0a;                
   logic [VLEN-1:0]        src3_0a;                
   logic                v_tox_mv_1a;            
   logic [VLEN-1:0]        vm0_0a;         

   logic [5:0]          wraddr_1a,wraddr_2a;
   logic                wren_1a,fwren_1a,fwren_2a;
   //logic [VLEN-1:0]      iwrdata_1a;//,fwrdata_1a,wrdata_1a;
   tt_briscv_pkg::csr_fp_exc fwrexc_1a, fwrexc_2a;
   logic [VLEN-1:0]        fwrdata_2a;
   logic                ix_tov_mv_0a,ixv_tov_mv_0a,v_tov_mv_0a,f_tov_mv_0a,v_tox_mv_0a,vmerge_0a,funct3_vvxi_0a;
   logic [XLEN-1:0]         itov_src_0a,usgn_itov_src_0a;
   logic                sel_xfi_src_0a;
   logic                id_replay_1a;
   logic                reduct_wdeop_0a,reduct_wdeop_1a,reduct_wdeop_2a;
        
   //read the mask reg, and update destination accordingly.
   logic [VLEN/8-1:0]      dstwr_bytemask_0a,dstwr_bytemask_1a,dstwr_bytemask_2a;
   logic [VLEN-1:0]        dstwr_bitmask_2a,dstwr_bitmask_1a,prermw_wrdata_1a,prermw_wrdata_2a;    
   logic [VLEN-1:0]        rddst_data_0a,rddst_data_1a,rddst_data_2a;

   logic [1:0]          vsew_2a,vsew_1a;
                
   //shared multiplier between ivec and fvec
   logic [VLEN/8-1:0][2*XLEN:0]  mulsum_1a;
   logic [VLEN/8-1:0][XLEN-1:0]  mulsrc2_0a,mulsrc1_0a;
   logic [VLEN/8-1:0][XLEN-1:0]  imulsrc2_0a,imulsrc1_0a;
   logic                vfp_hole_vld_1a;
   //logic              imulissgn_0a,fmulissgn_0a,mulissgn_0a;
   logic                imulen_0a,mulen_0a,mulen_1a, fmulen_0a,rnden_1a,rnden_0a;
   logic                vslideop_0a,vslideop_1a,slide1op_0a,slide_dwn_0a,vslideup_0a,vslidedwn_0a,vslideup_1a,vslidedwn_1a;
   logic                vrgatherop_0a,vrgatherei16op_0a,vrgatherei16op_1a,vgatherall_0a,vgatherall_1a,vgatherall_2a;
   logic [1:0]          vsew_gather_0a,vsew_gather_1a;          
   logic                vrf_rden2_0a,vrf_rden1_0a,vrf_rden0_0a;
   logic                imul_accen_0a,iadden_0a;
   logic [VLEN-1:0]        idata_1a,idata_2a;
   logic                idata_vld_1a,idata_vld_2a;
   logic [VLEN-1:0]        src1_mux_0a;    
   //handshaking
   logic                vex_rts_1a;
   logic                rtr_to_id_1a;
   logic                wdeop_1a;
   logic                vex_invalid_instrn_1a;
   logic [31:0]         pvex_instrn_1a,vex_instrn_1a;
   logic [XLEN-1:0]     scalar_imm_slide_1a,scalar_imm_slide_0a;        
   logic [6:0]          opcode_0a,opcode_1a;    
   logic [2:0]          funct3_0a,funct3_1a;     
   logic [6:0]          funct7_0a,funct7_1a;    
   logic                vex_en_0a,vex_en_1a,vex_en_2a;
   logic                vex_per_0a,vex_shft_0a,vex_bitwise_0a,vex_vmaskbit_0a,vex_bitwise_1a,mask_only_rts_1a;
   logic                compress_0a,compress_1a,compress_2a,compress_rts_1a,compress_rts_0a;
   logic                force_completion_rts_0a,force_completion_rts_1a,force_completion_rts_2a;
   logic                fp_sel_scalar_0a,sel_scalar_0a,sel_imm_0a,scalar_dest_0a,sat_instrn_0a;
   logic                scalar_dest_1a;
   logic                vmv_x_s,vmv_s_x,vmv_s_f;                
   logic [VLEN-1:0]     instrn_dec_0a,instrn_dec_1a;
   logic                vex_post_rts_1a,vex_post_rts_2a,vex_post_rts_3a;
   logic [VLEN/8-1:0]   vm0_muxed_0a,vm0_muxed_1a,vm0_sized_0a;
   logic [VLEN/8-1:0]   vl_muxed_0a,vl_muxed_1a;
   logic [VLEN/8-1:0]   vl_sized_0a;
   logic [VLEN/8-1:0]   compress_mask_0a;
   logic [VLEN-1:0]     compress_mask_selects_0a;       
   logic [$clog2(VLEN+1)-1:0]          vl_cnt_1a;              
   logic [$clog2(VLEN/8)+2:0]          compress_cnt_int_0a,compress_cnt_0a,compress_cnt_1a,compress_cnt_2a;
   
   logic                compress_byte_en_0a,compress_byte_en_1a,compress_byte_en_2a;
   logic                mask_only_instrn_0a,mask_only_instrn_1a;
   logic                rfwren_0a, rfwren_1a;
   logic [2:0]          lmul_2a,lmul_1a;
   logic [2:0]          lmul_cnt_2a,lmul_cnt_1a,lmul_cnt_0a,lmul_cnt_foraddr_0a,emul_cnt_foraddr_0a,compress_cnt_foraddr_0a;
   logic [VLEN-1:0]     mask_only_bitmask_1a;
   logic                v_vm_1a,wrmask_1a;
   logic [XLEN-1:0]     xorf_mv_data_0a;
   logic [$clog2(VLEN/8)+3:0] iterate_cnt_max_0a,iterate_cnt_max_1a;
   logic [$clog2(VLEN/8)-1:0] iterate_cnt_0a,iterate_cnt_1a,iterate_cnt_2a;
   logic                iterate_0a,iterate_1a,iterate_2a;
   logic                nrwop_0a,nrwop_1a,nrwop_2a,nrwop_lmul_0a,nrwop_lmul_1a;
   logic                wdeop_0a;
   logic                out_from_vec_int_0a,out_from_vec_int_1a,out_from_vec_int_2a;
   //Making iterate data 2a to overload the ooo_data wires. The last cycle of iterate count will send data from the unit and not from the iterate flop
   logic [VLEN-1:0]     iterate_data_2a;
   logic [VLEN-1:0]     iterate_data_saved_3a;
   logic [XLEN-1:0]     reductop_data_0a; 
   logic [$clog2(VLEN/8)+3:0]          last_lmul_iter_sum_0a;          
   logic [VLEN/8-1:0]   iterate_byte_en_2a;     
   logic                iterate_or_nrw_0a,iterate_or_nrw_1a,iterate_or_nrw_2a;
   logic                sel_iterate_or_nrw_3a,sel_iterate_or_nrw_1a;
   logic                sel_zero_or_vd_0a,sel_zero_or_vd_1a,sel_zero_or_vd_2a;
   logic                inc_iterate_2a,inc_iterate_1a,inc_iterate_0a;
   logic [4:0]          base_addrp2,base_addrp1,base_addrp0,final_addrp1_0a,sized_iterate_addrp1_0a;
   logic [XLEN-1:0]     inc_addrp1_0a;
   logic [$clog2(VLEN/8)-1:0]          vs1_gather_indx_1a;
   logic                ovfl_iterate_addrp1_0a;
   logic                reductop_0a,reductop_1a,reductop_2a;
   logic [XLEN-1:0]     slide_shft_amt_0a;
   logic [$clog2(VLEN/8)+2:0]          tot_iterate_norply_cnt_0a,tot_iterate_cnt_1a,tot_iterate_cnt_0a;                
   logic                two_cycle_iterate_0a,two_cycle_iterate_1a,two_cycle_iterate_2a;
   logic                vmvgrp_0a;
   logic [2:0]          vmvgrp_evl_0a;  

   assign o_sat_csr = sat_csr_2a &  vex_en_2a;
   
   always_comb begin
      if(nrwop_2a)
        iterate_byte_en_2a = iterate_cnt_2a[0] ? {{VLEN/16{1'b1}}, {VLEN/16{1'b0}}} :
                                                 {{VLEN/16{1'b0}}, {VLEN/16{1'b1}}};
      else if(reductop_2a)
        case(vsew_2a + reduct_wdeop_2a)
          2'b00: iterate_byte_en_2a = 'h1; 
          2'b01: iterate_byte_en_2a = 'h3; 
          2'b10: iterate_byte_en_2a = 'hf;
          2'b11: iterate_byte_en_2a = 'hff;
        endcase
      else if(iterate_2a & inc_iterate_2a & two_cycle_iterate_2a)       
        case(vsew_2a)
          2'b00: iterate_byte_en_2a = 'h1  << (iterate_cnt_2a[$clog2(VLEN/8)-1:0])  ; 
          2'b01: iterate_byte_en_2a = 'h3  << (iterate_cnt_2a[$clog2(VLEN/8)-2:0])*2; 
          2'b10: iterate_byte_en_2a = 'hf  << (iterate_cnt_2a[$clog2(VLEN/8)-3:0])*4; 
          2'b11: iterate_byte_en_2a = 'hff << (iterate_cnt_2a[$clog2(VLEN/8)-4:0])*8; 
        endcase
      else if(iterate_2a & ~two_cycle_iterate_2a & ~compress_2a)
        case(vsew_2a)
          2'b00: iterate_byte_en_2a = 'h1  << (iterate_cnt_2a[$clog2(VLEN/8)-1:0])  ; 
          2'b01: iterate_byte_en_2a = 'h3  << (iterate_cnt_2a[$clog2(VLEN/8)-2:0])*2; 
          2'b10: iterate_byte_en_2a = 'hf  << (iterate_cnt_2a[$clog2(VLEN/8)-3:0])*4;
          2'b11: iterate_byte_en_2a = 'hff << (iterate_cnt_2a[$clog2(VLEN/8)-4:0])*8;
        endcase 
      else if(iterate_2a & compress_2a & compress_byte_en_2a)
        case(vsew_2a)
          2'b00: iterate_byte_en_2a = 'h1  << (compress_cnt_2a[$clog2(VLEN/8)-1:0])  ; 
          2'b01: iterate_byte_en_2a = 'h3  << (compress_cnt_2a[$clog2(VLEN/8)-2:0])*2; 
          2'b10: iterate_byte_en_2a = 'hf  << (compress_cnt_2a[$clog2(VLEN/8)-3:0])*4;
          2'b11: iterate_byte_en_2a = 'hff << (compress_cnt_2a[$clog2(VLEN/8)-4:0])*8;
        endcase
      else
        iterate_byte_en_2a = '0;
   end

   assign vsew_gather_1a[1:0] = vrgatherei16op_1a ? 2'b01 : vsew_1a;
   assign vsew_gather_0a[1:0] = vrgatherei16op_0a ? 2'b01 : i_csr.v_vsew[1:0];

   //Write the dst bytes in the last iteration( i believe the destination doesn't change fast enough, since I'm using compress_1a to change destination)
   logic [VLEN/8-1:0] compress_remaining_bytes_2a;
   always_comb
     case(vsew_2a)
       2'b00:  compress_remaining_bytes_2a = {VLEN/8{&iterate_cnt_2a[$clog2(VLEN/8)-1:0] & compress_2a & force_completion_rts_2a}} <<  compress_cnt_2a[$clog2(VLEN/8)-1:0];
       2'b01:  compress_remaining_bytes_2a = {VLEN/8{&iterate_cnt_2a[$clog2(VLEN/8)-2:0] & compress_2a & force_completion_rts_2a}} << (compress_cnt_2a[$clog2(VLEN/8)-1:0] << 1);
       2'b10:  compress_remaining_bytes_2a = {VLEN/8{&iterate_cnt_2a[$clog2(VLEN/8)-3:0] & compress_2a & force_completion_rts_2a}} << (compress_cnt_2a[$clog2(VLEN/8)-1:0] << 2);
       2'b11:  compress_remaining_bytes_2a = {VLEN/8{&iterate_cnt_2a[$clog2(VLEN/8)-4:0] & compress_2a & force_completion_rts_2a}} << (compress_cnt_2a[$clog2(VLEN/8)-1:0] << 3);
     endcase
   
   always_ff @(posedge i_clk)
     if(reductop_2a)
       for (int i=0;         i<VLEN/8; i++) iterate_data_saved_3a[i*8+:8] <= (~dstwr_bytemask_2a[i])                                 ? rddst_data_2a[i*8+:8] : prermw_wrdata_2a[i*8+:8];
     else if(nrwop_2a) begin
        if(~iterate_cnt_2a[0]) 
          for (int i=0;      i<VLEN/16;i++) iterate_data_saved_3a[i*8+:8] <= (iterate_byte_en_2a[i] & ~dstwr_bytemask_2a[i        ]) ? rddst_data_2a[i*8+:8] : fwren_2a ? fwrdata_2a[(i%(VLEN/16))*8+:8] : idata_2a[(i%(VLEN/16))*8+:8];
        if(iterate_cnt_2a[0]) 
          for (int i=VLEN/16;i<VLEN/8; i++) iterate_data_saved_3a[i*8+:8] <= (iterate_byte_en_2a[i] & ~dstwr_bytemask_2a[i-VLEN/16]) ? rddst_data_2a[i*8+:8] : fwren_2a ? fwrdata_2a[(i%(VLEN/16))*8+:8] : idata_2a[(i%(VLEN/16))*8+:8];
     end else begin
       for (int i=0; i<VLEN/8;i++)
	 if(iterate_byte_en_2a[i] | compress_remaining_bytes_2a[i]) begin
           if(~dstwr_bytemask_2a[i] | compress_remaining_bytes_2a[i] & ~iterate_byte_en_2a[i])
             iterate_data_saved_3a[i*8+:8] <= rddst_data_2a[i*8+:8]; 
           else
             unique case(1'b1)
               (sel_zero_or_vd_2a & iterate_2a) : iterate_data_saved_3a[i*8+:8] <=  rddst_data_2a[i*8+:8] & ~{8{slide_dwn_2a | vgatherall_2a}};
              
               default                          : iterate_data_saved_3a[i*8+:8] <=  (vsew_2a==2'h0) ? idata_2a[       0+:8] :
                                                                                    (vsew_2a==2'h1) ? idata_2a[i[  0]*8+:8] :
                                                                                    (vsew_2a==2'h2) ? idata_2a[i[1:0]*8+:8] :
                                                                                                      idata_2a[i[2:0]*8+:8];
             endcase
	 end  // if
     end //else

  
   //Internally the DP either sends src1 back(when mask=0) or computed result(when mask=1) ; need to ensure prermw_wrdata_1a survives for next lmul iteration.
   always_comb begin
      if ((~i_id_replay) & reductop_0a)
        reductop_data_0a[XLEN-1:0] = src1_mux_0a[XLEN-1:0]; 
      else if(idata_vld_1a &  out_from_vec_int_1a)
        reductop_data_0a[XLEN-1:0] = idata_1a[XLEN-1:0];
      else 
        reductop_data_0a[XLEN-1:0] = rddst_data_2a[XLEN-1:0]; //Note:REDUCT Needed when in middle of iteration the RTS goes away, But I don't think it will ever happen. This is just fail safe.
   end
   
   always_comb begin
      if((nrwop_0a | wdeop_0a) & ~lmul_2a[2])  //Will be set for narrow shift and clip.64 bits processed in a cycle. Therefore iterate_max = 2
        iterate_cnt_max_0a = ('d1 << ~i_csr.v_lmul[2]); //spyglass disable STARC05-2.10.3.2b_sb
      else if(iterate_0a)
        //iterate_cnt_max_0a = (5'd1 << (4-i_csr.v_vsew[1:0] + i_csr.v_lmul[1:0] - {i_csr.v_lmul[2],2'b00})); //spyglass disable STARC05-2.10.3.2b_sb
        iterate_cnt_max_0a = ('d1 << ($clog2(VLEN/8)-i_csr.v_vsew[1:0] + i_csr.v_lmul[1:0] - {i_csr.v_lmul[2],2'b00}));  //spyglass disable STARC05-2.10.3.2b_sb
      else
        iterate_cnt_max_0a = 'd1; //spyglass disable STARC05-2.10.3.2b_sb
   end

   always_ff @(posedge i_clk)
     if(~i_reset_n) begin
        nrwop_2a               <= '0;
        reduct_wdeop_2a        <= '0;
        iterate_cnt_2a         <= '0;
        compress_cnt_2a        <= '0;
        lmul_2a                <= '0;
        iterate_2a             <= '0;
        slide_dwn_2a           <= '0;
        sel_zero_or_vd_2a      <= '0;

        vgatherall_2a          <= '0;
        vex_post_rts_2a        <= '0;
        reductop_2a            <= '0;
        reduct_wdeop_2a        <= '0;
        compress_2a            <= '0;
        compress_byte_en_2a    <= '0;
        force_completion_rts_2a <='0;
     end else if(vex_en_1a | vex_en_2a) begin
        nrwop_2a               <= nrwop_1a;
        reduct_wdeop_2a        <= reduct_wdeop_1a;
        iterate_cnt_2a         <= tot_iterate_cnt_1a;
        compress_cnt_2a        <= compress_cnt_1a;
        lmul_2a                <= lmul_1a;
        iterate_2a             <= iterate_1a;
        slide_dwn_2a           <= slide_dwn_1a;
        sel_zero_or_vd_2a      <= sel_zero_or_vd_1a;

        vgatherall_2a          <= vgatherall_1a;
        vex_post_rts_2a        <= vex_post_rts_1a;
        reductop_2a            <= reductop_1a;
        reduct_wdeop_2a        <= reduct_wdeop_1a;
        compress_2a            <= compress_1a;
        compress_byte_en_2a    <= compress_byte_en_1a;
        force_completion_rts_2a <= force_completion_rts_1a;
     end
   
   assign opcode_0a       = {7{vex_en_0a}} & i_id_ex_instrn[6:0];
   assign funct3_0a       = {3{vex_en_0a}} & i_id_ex_instrn[14:12];
   assign funct7_0a       = {7{vex_en_0a}} & i_id_ex_instrn[31:25];
   wire [4:0] reg_p0      = i_id_vec_autogen.rf_addrp0[4:0];
   wire [4:0] reg_p1      = i_id_vec_autogen.rf_addrp1[4:0];
   wire [4:0] reg_p2      = i_id_vec_autogen.rf_rd_p2_is_rs2 ? i_id_vec_autogen.rf_addrp1[4:0] : i_id_vec_autogen.rf_addrp2[4:0];
   assign vex_instrn_1a   = {32{vex_rts_1a}} & pvex_instrn_1a[31:0];
  
   //handshaking
   assign vex_en_0a       = i_id_vex_rts & o_vex_id_rtr;// | iterate_1a;
   wire hazard_stall_1a = 1'b0;//iterate_1a & (iterate_cnt_1a != '0); //MM Nov 5 2021: Remove implicit wire.
   assign out_from_vec_int_0a = vex_en_0a & i_id_vec_autogen.out_from_vec_int;
   logic fwren_0a; assign fwren_0a =  vex_en_0a & i_id_vec_autogen.vfp_mad_type_inst; //MM Nov 5 2021: Remove implicit wire.
   assign imulen_0a       = vex_en_0a & i_id_vec_autogen.imulop;
   assign vslideup_0a     = (funct7_0a[6:1] == 6'b00_1110) & ((funct3_0a[2:0] == `OPIVX) | (funct3_0a[2:0] == `OPIVI));
   assign vslidedwn_0a    = (funct7_0a[6:1] == 6'b00_1111) & ((funct3_0a[2:0] == `OPIVX) | (funct3_0a[2:0] == `OPIVI));
   assign vslideop_0a     = (funct7_0a[6:1] == 6'b00_1110) & ((funct3_0a[2:0] == `OPIVX) | (funct3_0a[2:0] == `OPIVI))
                           |(funct7_0a[6:1] == 6'b00_1111) & ((funct3_0a[2:0] == `OPIVX) | (funct3_0a[2:0] == `OPIVI))
                           |(funct7_0a[6:1] == 6'b00_1110) &  (funct3_0a[2:0] == `OPMVX)
                           |(funct7_0a[6:1] == 6'b00_1111) &  (funct3_0a[2:0] == `OPMVX)
                           |(funct7_0a[6:1] == 6'b00_1110) &  (funct3_0a[2:0] == `OPFVF)
                           |(funct7_0a[6:1] == 6'b00_1111) &  (funct3_0a[2:0] == `OPFVF);
   
   assign slide1op_0a     = (funct7_0a[6:1] == 6'b00_1110) &  ((funct3_0a[2:0] == `OPMVX) | (funct3_0a[2:0] == `OPFVF))
                           |(funct7_0a[6:1] == 6'b00_1111) &  ((funct3_0a[2:0] == `OPMVX) | (funct3_0a[2:0] == `OPFVF));
   
   assign slide_dwn_0a    = (funct7_0a[6:1] == 6'b00_1111) & ((funct3_0a[2:0] == `OPIVX) | (funct3_0a[2:0] == `OPMVX) | (funct3_0a[2:0] == `OPFVF) | (funct3_0a[2:0] == `OPIVI));
   
   assign vrgatherop_0a    = (funct7_0a[6:1] == 6'b00_1100) & ((funct3_0a[2:0] == `OPIVX) | (funct3_0a[2:0] == `OPIVI)  | (funct3_0a[2:0] == `OPIVV) );
   assign vrgatherei16op_0a = (funct7_0a[6:1] == 6'b00_1110) & ((funct3_0a[2:0] == `OPIVV) );
   assign vgatherall_0a    = vrgatherop_0a | vrgatherei16op_0a;
   assign rnden_0a        = i_id_vec_autogen.rnden & vex_en_0a;
   assign imul_accen_0a   = vex_en_0a & i_id_vec_autogen.acc_val;
   assign iterate_0a      = vex_en_0a & i_id_vec_autogen.iterate;
   //assign reduct_0a       = vex_en_0a & i_id_vec_autogen.reductop; //MM Nov 5 2021: Remove implicit wire. This signal appears to be unused
   assign nrwop_0a        = vex_en_0a & i_id_vec_autogen.nrwop & ~i_csr.v_lmul[2]; //nrwop needs iterateion for lmul>=1 only
   assign nrwop_lmul_0a   = vex_en_0a & i_id_vec_autogen.nrwop; //but internally, need to increment sew to select correct rounding bits.
   assign wdeop_0a        = vex_en_0a & i_id_vec_autogen.wdeop;
   assign vex_shft_0a     = vex_en_0a & i_id_vec_autogen.shftop;
   assign vex_bitwise_0a  = vex_en_0a & i_id_vec_autogen.bitwise;
   assign scalar_dest_0a  = vex_en_0a & i_id_vec_autogen.scalar_dest;
   assign iadden_0a       = vex_en_0a & i_id_vec_autogen.iaddop;
   logic avg_0a; assign avg_0a = vex_en_0a & i_id_vec_autogen.avg; //MM Nov 5 2021: Remove implicit wire.
   assign vmv_x_s         = vex_en_0a & (funct7_0a[6:1] == 6'b010_000) & (funct3_0a[2:0] == `OPMVV) & (reg_p0[4:0] == 5'b00000);
   assign reductop_0a     = vex_en_0a & i_id_vec_autogen.reductop;
   assign vmvgrp_0a       = vex_en_0a & i_id_vec_autogen.vmvgrp;
   assign vmvgrp_evl_0a[2:0]   = vmvgrp_0a & (i_id_ex_instrn[17:15]);  //this is th vl->evl multiplier. if this appear in the timing path , a post flop expansion with Or with VL is needed.
   assign vmv_s_x         = vex_en_0a & (funct7_0a[6:1] == 6'b010_000) & (funct3_0a[2:0] == `OPMVX) & (reg_p1[4:0] == 5'b00000); 
   logic vmv_f_s; assign vmv_f_s = vex_en_0a & (funct7_0a[6:1] == 6'b010_000) & (funct3_0a[2:0] == `OPFVV) & (reg_p0[4:0] == 5'b00000); //MM Nov 5 2021: Remove implicit wire.
   assign vmv_s_f         = vex_en_0a & (funct7_0a[6:1] == 6'b010_000) & (funct3_0a[2:0] == `OPFVF) & (reg_p1[4:0] == 5'b00000);                                         
   assign vex_per_0a      = vex_en_0a & i_id_vec_autogen.permuteop | vmv_x_s | vmv_s_x  | vmv_s_f; //| vmv_f_s
   assign o_ignore_lmul   = vmv_x_s | vmv_s_x | vmv_s_f | vmv_f_s;
   assign o_ignore_dstincr = mask_only_instrn_0a | reductop_0a;
   assign o_ignore_srcincr = mask_only_instrn_0a & (funct3_0a[2:0] == `OPMVV) & vex_bitwise_0a  //to prevent lmul increment changing the src for vmand.mm type instructions
                            | vex_vmaskbit_0a;   //iota, vmsof etc
   assign compress_0a     = vex_en_0a & (funct7_0a[6:1] == 6'b010_111) & (funct3_0a[2:0] == `OPMVV); 
   assign sel_scalar_0a   = vex_en_0a & i_id_vec_autogen.sel_scalar;
   assign sel_imm_0a      = vex_en_0a & i_id_vec_autogen.sel_imm; 
   assign fp_sel_scalar_0a   = vex_en_0a & i_id_vec_autogen.fp_sel_scalar;
   assign sat_instrn_0a      = vex_en_0a & i_id_vec_autogen.sat_instrn;

    //decodes.
   assign funct3_vvxi_0a    = (funct3_0a[2:0] == `OPIVV) | (funct3_0a[2:0] == `OPIVX) | (funct3_0a[2:0] == `OPIVI);
   assign f_tov_mv_0a       = vex_en_0a & (funct7_0a[6:0] == 7'b010_1111) & (funct3_0a[2:0] == `OPFVF);
   assign ix_tov_mv_0a      = vex_en_0a & (funct7_0a[6:0] == 7'b010_1111) & (funct3_0a[2:0] == `OPIVI | funct3_0a[2:0] == `OPIVX | funct3_0a[2:0] == `OPFVF);
   assign v_tov_mv_0a       = vex_en_0a & (funct7_0a[6:0] == 7'b010_1111) & (funct3_0a[2:0] == `OPIVV);
   assign v_tox_mv_0a       = vex_en_0a & (funct7_0a[6:0] == 7'b010_0001) & (funct3_0a[2:0] == `OPMVV);
   assign vmerge_0a         = vex_en_0a & ((funct7_0a[6:0] == 7'b010_1110 && funct3_vvxi_0a) || (funct7_0a[6:1] == 6'b010111 && funct3_0a[2:0] == `OPFVF));
   assign reduct_wdeop_0a   = vex_en_0a & (((funct7_0a[6:1] == 6'b110_000) | (funct7_0a[6:1] == 6'b110_001)) & (funct3_0a[2:0] == `OPIVV) ||  //creating a reduct_wdeop because there is no dest change required as is for a wdeop's
                                           ((funct7_0a[6:1] == 6'b110_001) | (funct7_0a[6:1] == 6'b110_011)) & (funct3_0a[2:0] == `OPFVV)   );
   assign ixv_tov_mv_0a     = ix_tov_mv_0a | v_tov_mv_0a;

   assign sel_xfi_src_0a    = ix_tov_mv_0a | f_tov_mv_0a | v_tox_mv_0a | vmv_s_x | vmv_s_f | sel_scalar_0a | sel_imm_0a | fp_sel_scalar_0a;  //IMPROVE, eventually remove all other decodes, only scalar and imm needed.
   assign xorf_mv_data_0a   = (f_tov_mv_0a || fp_sel_scalar_0a) ? ((i_csr.v_vsew[1:0] == 2'h2       &&
                                                                    i_fprf_vex_p0[63:32]     != {32{1'b1}}   ) ? 64'hffff_ffff_7fc0_0000 :
                                                                   (i_csr.v_vsew[1:0] == 2'h1       &&
                                                                    i_fprf_vex_p0[63:16]     != {48{1'b1}}   ) ? 64'hffff_ffff_ffff_7e00 :
                                                                                                                 i_fprf_vex_p0            )
                                                                : i_rf_vex_p0;

   assign itov_src_0a        [XLEN-1:0] = sel_imm_0a ? {{XLEN-5{reg_p0[4]}},reg_p0[4:0]} : xorf_mv_data_0a;//i_rf_vex_p0;
   assign usgn_itov_src_0a   [XLEN-1:0] = sel_imm_0a ? {{XLEN-5{     1'b0}},reg_p0[4:0]} : xorf_mv_data_0a;//i_rf_vex_p0;
   assign scalar_imm_slide_0a[XLEN-1:0] = vgatherall_0a ? inc_addrp1_0a[XLEN-1:0] : usgn_itov_src_0a[XLEN-1:0];
   logic mem_pipe_rtr_1a; assign mem_pipe_rtr_1a =  1'b1;  //MM Nov 5 2021: Remove implicit wire.
   tt_rts_rtr_pipe_stage #(.WIDTH(1), .NORTR(1)) ex_invalid_instruction_flop
  (
  .i_clk     (i_clk           ),
  .i_reset_n (i_reset_n       ),
  .i_rts     (i_id_vex_rts    ), // input side handshake
  .o_rtr     (rtr_to_id_1a    ),
  .o_rts     (vex_rts_1a      ),
  .i_rtr     (mem_pipe_rtr_1a ), // if EX is ready to accept next instruction and no raw hazard requiring a stall was detected
  .i_data    (1'b0), 
  .o_data    (vex_invalid_instrn_1a )
  );
   tt_rts_rtr_pipe_stage #(.WIDTH(32),.NORTS_DROPPED(1)) ex_instrn_flops
(
   .i_clk     (i_clk           ),
   .i_reset_n (i_reset_n       ),
   .i_rts     (i_id_vex_rts    ), // input side handshake
   .o_rtr     (                ), // input side handshake
   .i_rtr     (mem_pipe_rtr_1a ), // if EX is ready to accept next instruction and no raw hazard requiring a stall was detected
   .o_rts     (                ), // output side handshake
   .i_data    (i_id_ex_instrn  ),
   .o_data    (pvex_instrn_1a  ) 
);
  
   assign o_vex_id_rtr      = rtr_to_id_1a & (!hazard_stall_1a);

   logic vsetvli_kill_1a;
   logic iterate_rts_1a;
   always_comb
     case(vsew_1a)
       2'b00  : iterate_rts_1a =  ~|tot_iterate_cnt_1a[0+:$clog2(VLEN/ 8)] & (~two_cycle_iterate_1a | inc_iterate_1a);
       2'b01  : iterate_rts_1a =  ~|tot_iterate_cnt_1a[0+:$clog2(VLEN/16)] & (~two_cycle_iterate_1a | inc_iterate_1a);
       2'b10  : iterate_rts_1a =  ~|tot_iterate_cnt_1a[0+:$clog2(VLEN/32)] & (~two_cycle_iterate_1a | inc_iterate_1a);
       2'b11  : iterate_rts_1a =  ~|tot_iterate_cnt_1a[0+:$clog2(VLEN/64)] & (~two_cycle_iterate_1a | inc_iterate_1a);
       default: iterate_rts_1a = 'x;
     endcase 

   always_comb
     casez(lmul_1a[2:0])
       3'b1??,
       3'b000  : mask_only_rts_1a =  tot_iterate_cnt_1a[2:0]==3'd0;
       3'b001  : mask_only_rts_1a =  tot_iterate_cnt_1a[2:0]==3'd1;
       3'b010  : mask_only_rts_1a =  tot_iterate_cnt_1a[2:0]==3'd3;
       3'b011  : mask_only_rts_1a =  tot_iterate_cnt_1a[2:0]==3'd7;
       default : mask_only_rts_1a = 'x;
     endcase 
   
   always_comb
     case(i_csr.v_vsew[1:0])
       2'b00  : compress_rts_0a =  &compress_cnt_0a[0+:$clog2(VLEN/ 8)] &  compress_byte_en_0a;
       2'b01  : compress_rts_0a =  &compress_cnt_0a[0+:$clog2(VLEN/16)] &  compress_byte_en_0a;
       2'b10  : compress_rts_0a =  &compress_cnt_0a[0+:$clog2(VLEN/32)] &  compress_byte_en_0a;
       2'b11  : compress_rts_0a =  &compress_cnt_0a[0+:$clog2(VLEN/64)] &  compress_byte_en_0a;
       default: compress_rts_0a = 'x;
     endcase

   always_ff@(posedge i_clk)
     if(~i_reset_n)
        compress_cnt_int_0a     <= '0;
     else if(compress_0a)
       compress_cnt_int_0a      <= (compress_cnt_int_0a & {$clog2(VLEN/8)+3{i_id_replay}}) + compress_byte_en_0a;
   
   assign compress_byte_en_0a = compress_0a & (compress_mask_selects_0a[tot_iterate_cnt_0a]);
   assign compress_cnt_0a     = compress_cnt_int_0a & {$clog2(VLEN/8)+3{i_id_replay}};  //need to start at 0.
   always_ff@(posedge i_clk)
     if(~i_reset_n) begin
        compress_byte_en_1a <= '0;
        compress_cnt_1a     <= '0;
     end else if(compress_0a) begin
        compress_byte_en_1a <= compress_byte_en_0a;
        compress_cnt_1a     <= compress_cnt_0a;
     end
   
   assign vex_mem_rts = ~vsetvli_kill_1a & vex_rts_1a & vex_en_1a & (~iterate_or_nrw_1a | iterate_rts_1a & ~compress_1a | compress_1a & (compress_rts_1a | force_completion_rts_1a));
  
   logic [VLEN/8-1:0] reductop_mask_0a; 
   always_comb
     case(i_csr.v_vsew[1:0] + reduct_wdeop_0a)
       2'b00  : reductop_mask_0a = 'h1;
       2'b01  : reductop_mask_0a = 'h3;
       2'b10  : reductop_mask_0a = 'hf;
       2'b11  : reductop_mask_0a = 'hff;
       default: reductop_mask_0a = 'x;
     endcase

   //This is the true number of active elements, these are not based on size. Ex: Max of 128 is possible for SEW=8,  and Max of 32 is possible for SEW=32
   wire [2*VLEN-1:0] vl_mask_dbl_0a   = {{VLEN{1'b0}},{VLEN{1'b1}}} << i_csr.v_vl; //<< vmvgrp_evl_0a[2:0]));//* ( ((i_csr.v_vsew[1:0] == 2'b10) * 4) + ((i_csr.v_vsew[1:0] == 2'b01) * 2) + (i_csr.v_vsew[1:0] == 2'b00)));
   wire [  VLEN-1:0] vl_mask_0a       = vl_mask_dbl_0a[2*VLEN-1:VLEN];
   wire        sel_fs_mv_0a      = vmv_x_s | vmv_s_x  | vmv_s_f | vmv_f_s;
   wire [VLEN/8-1:0] vmv_s_x_mask_0a   = (i_csr.v_vsew[1:0] == 2'h0) ? {{VLEN/8-1{1'b0}},{1{vmv_s_x | vmv_s_f}}} :
                                         (i_csr.v_vsew[1:0] == 2'h1) ? {{VLEN/8-2{1'b0}},{2{vmv_s_x | vmv_s_f}}} :
                                         (i_csr.v_vsew[1:0] == 2'h2) ? {{VLEN/8-4{1'b0}},{4{vmv_s_x | vmv_s_f}}} :
                                                                              {{VLEN/8-8{1'b0}},{8{vmv_s_x | vmv_s_f}}};
   wire [VLEN/8:0] nrw_mask_0a       = '1;
//;{16{~i_csr.v_lmul[2] | vmvgrp_0a}} | (16'hffff >> (4'b1000 + {1'b0,i_csr.v_lmul[1:0]!=2'b11,2'b00} + {2'b00,i_csr.v_lmul[1:0]==2'b01,1'b0})); 
  
   assign dstwr_bytemask_0a = sel_fs_mv_0a ?  vmv_s_x_mask_0a | {{VLEN/16{1'b0}}, {VLEN/16{vmv_x_s | vmv_f_s}}}
                                                  //If VL = 0 cancel all updates to the destination register.
                                                  : (reductop_0a ?  reductop_mask_0a & {VLEN/8{i_csr.v_vl != 0}} 
                                                                 :  (vm0_sized_0a | {VLEN/8{i_v_vm | vmerge_0a | i_id_vec_autogen.usemask}}) & (vl_sized_0a | {VLEN/8{compress_0a}}) & nrw_mask_0a);

   wire  [$clog2(VLEN/8+1)-1:0] lmul_amt_multiple_0a = {$clog2(VLEN/8+1){~i_csr.v_lmul[2]}} & (i_csr.v_vsew[1:0]==2'b11 ? VLEN/64 :
                                                                                                      i_csr.v_vsew[1:0]==2'b10 ? VLEN/32 :
                                                                                                      i_csr.v_vsew[1:0]==2'b01 ? VLEN/16 :
                                                                                                                                        VLEN/8   );
   //assign lmul_cnt_change_0a = two_cycle_iterate_0a ? lmul_cnt_1a[2:0] : lmul_cnt_0a[2:0];
   assign vl_muxed_0a  =  (vl_mask_0a[VLEN-1:0] >> (($clog2(VLEN+1))'(lmul_amt_multiple_0a) * lmul_cnt_0a[2:0])) | {VLEN/8{vmvgrp_0a}};    //spyglass disable STARC05-2.10.3.2b_sa
   assign vm0_muxed_0a =  (vm0_0a    [VLEN-1:0] >> (($clog2(VLEN+1))'(lmul_amt_multiple_0a) * lmul_cnt_0a[2:0])) | {VLEN/8{i_v_vm}};       //spyglass disable STARC05-2.10.3.2b_sa
   assign compress_mask_selects_0a[VLEN-1:0] = src1_0a[VLEN-1:0] & vl_mask_0a[VLEN-1:0];
   //In each cycle, narrow ops only need half as many mask bits.
  
   always_comb
     for(int i=0;i<VLEN/8;i++) 
       case(i_csr.v_vsew[1:0] + wdeop_0a) 
         2'b00: vm0_sized_0a[i] =                                                                   (nrwop_0a & tot_iterate_cnt_0a[0]) ? vm0_muxed_0a[VLEN/16+i[$clog2(VLEN/16)-1:0]  ] : vm0_muxed_0a[i  ];    
         2'b01: vm0_sized_0a[i] =  (wdeop_0a & tot_iterate_cnt_0a[0]) ? vm0_muxed_0a[VLEN/16+i/2] : (nrwop_0a & tot_iterate_cnt_0a[0]) ? vm0_muxed_0a[VLEN/32+i[$clog2(VLEN/16)-1:0]/2] : vm0_muxed_0a[i/2];
         2'b10: vm0_sized_0a[i] =  (wdeop_0a & tot_iterate_cnt_0a[0]) ? vm0_muxed_0a[VLEN/32+i/4] : (nrwop_0a & tot_iterate_cnt_0a[0]) ? vm0_muxed_0a[VLEN/64+i[$clog2(VLEN/16)-1:0]/4] : vm0_muxed_0a[i/4];
         2'b11: vm0_sized_0a[i] =  (wdeop_0a & tot_iterate_cnt_0a[0]) ? vm0_muxed_0a[VLEN/64+i/8] :                                                                                       vm0_muxed_0a[i/8];
       endcase

   always_comb
     for(int i=0;i<VLEN/8;i++) 
       case(i_csr.v_vsew[1:0] + wdeop_0a) 
         2'b00: vl_sized_0a[i] =                                                                  (nrwop_0a & tot_iterate_cnt_0a[0]) ? vl_muxed_0a[VLEN/16+i[$clog2(VLEN/16)-1:0]  ] : vl_muxed_0a[i];  
         2'b01: vl_sized_0a[i] =  (wdeop_0a & tot_iterate_cnt_0a[0]) ? vl_muxed_0a[VLEN/16+i/2] : (nrwop_0a & tot_iterate_cnt_0a[0]) ? vl_muxed_0a[VLEN/32+i[$clog2(VLEN/16)-1:0]/2] : vl_muxed_0a[i/2];
         2'b10: vl_sized_0a[i] =  (wdeop_0a & tot_iterate_cnt_0a[0]) ? vl_muxed_0a[VLEN/32+i/4] : (nrwop_0a & tot_iterate_cnt_0a[0]) ? vl_muxed_0a[VLEN/64+i[$clog2(VLEN/16)-1:0]/4] : vl_muxed_0a[i/4];
         2'b11: vl_sized_0a[i] =  (wdeop_0a & tot_iterate_cnt_0a[0]) ? vl_muxed_0a[VLEN/64+i/8] :                                                                                      vl_muxed_0a[i/8];
       endcase 

   //Mask only instructions write mask bits, inputs mask does tell which of the mask bits are to be written (look at vmseq). 
   //Based on lmul the vm0_muxed_1a is already shifted, further masking is done based on SEW.
   wire [VLEN/8-1:0] nums_mask_set_1a =  (lmul_1a == 3'b101 ? {'0, {VLEN/64{1'b1}}} : // 1/8
                                          lmul_1a == 3'b110 ? {'0, {VLEN/32{1'b1}}} : // 1/4
                                          lmul_1a == 3'b111 ? {'0, {VLEN/16{1'b1}}} : // 1/2
                                                                   {VLEN/8 {1'b1}}   )
                                       & (vm0_muxed_1a  | {VLEN/8{mask_only_instrn_1a & wrmask_1a}})
                                       & ({VLEN/8{1'b1}} >> ({|i_csr.v_vsew[1:0],3'b000} + {1'b0,i_csr.v_vsew[1],2'b00})) //shift by 8 (01 or 10) + shift by 4 for (10)
                                       & vl_muxed_1a;
 
   always_comb
     case(vsew_1a[1:0] )
       2'b00:   mask_only_bitmask_1a[VLEN-1:0] = {'0,nums_mask_set_1a} << (lmul_cnt_1a[2:0]*VLEN/ 8);
       2'b01:   mask_only_bitmask_1a[VLEN-1:0] = {'0,nums_mask_set_1a} << (lmul_cnt_1a[2:0]*VLEN/16);
       2'b10:   mask_only_bitmask_1a[VLEN-1:0] = {'0,nums_mask_set_1a} << (lmul_cnt_1a[2:0]*VLEN/32);
       2'b11:   mask_only_bitmask_1a[VLEN-1:0] = {'0,nums_mask_set_1a} << (lmul_cnt_1a[2:0]*VLEN/64);
       default: mask_only_bitmask_1a[VLEN-1:0] = 'x;
     endcase
   
   
   always_comb begin
      for(int i=0; i<VLEN/8; i++) begin
         dstwr_bitmask_2a[i*8+:8] = {8 {dstwr_bytemask_2a[i]}};
         dstwr_bitmask_1a[i*8+:8] = {8{scalar_dest_1a && i<4}}  | (mask_only_instrn_1a ? mask_only_bitmask_1a[i*8+:8] : {8 {dstwr_bytemask_1a[i]}});  //scalar destination should ignore mask bits.
      end
   end

   always_comb begin
      case(i_csr.v_vsew[1:0])
        2'b00: for(int i=0; i<VLEN/8;  i++) src1_mux_0a[i*8+:8]   = sel_xfi_src_0a ? itov_src_0a [7:0] : src1_0a[i*8+:8];  
        2'b01: for(int i=0; i<VLEN/16; i++) src1_mux_0a[i*16+:16] = sel_xfi_src_0a ? itov_src_0a[15:0] : src1_0a[i*16+:16];
        2'b10: for(int i=0; i<VLEN/32; i++) src1_mux_0a[i*32+:32] = sel_xfi_src_0a ? itov_src_0a[31:0] : src1_0a[i*32+:32];
        2'b11: for(int i=0; i<VLEN/64; i++) src1_mux_0a[i*64+:64] = sel_xfi_src_0a ? itov_src_0a[63:0] : src1_0a[i*64+:64];
      endcase
   end
   
   wire [VLEN-1:0] fwrdata_1a;
   always_comb begin
      unique case(1'b1)
        idata_vld_1a &  out_from_vec_int_1a: prermw_wrdata_1a[VLEN-1:0] = idata_1a[VLEN-1:0];
        fwren_1a                           : prermw_wrdata_1a[VLEN-1:0] = fwrdata_1a[VLEN-1:0];
        default:                             prermw_wrdata_1a[VLEN-1:0] = 'x;  
      endcase
   end

   assign iterate_or_nrw_2a       = iterate_2a | nrwop_2a;
   assign iterate_or_nrw_1a       = iterate_1a | nrwop_1a;
   //assign iterate_or_nrw_0a       = iterate_0a | nrwop_0a;
   assign slide_shft_amt_0a[63:0] = slide1op_0a ? 'd1 : usgn_itov_src_0a[63:0];

   assign vex_post_rts_1a = iterate_1a & (inc_iterate_1a | ~two_cycle_iterate_1a) & ( (compress_1a & compress_rts_1a | force_completion_rts_1a)) | ((nrwop_1a|wdeop_1a & ~iterate_1a) & tot_iterate_cnt_1a[0]);
      
   always_ff @ (posedge i_clk) begin
      sel_iterate_or_nrw_3a               <=  ((iterate_2a & (inc_iterate_2a | ~two_cycle_iterate_2a)) | nrwop_2a) & vex_post_rts_2a & out_from_vec_int_2a;
      vex_post_rts_3a                     <=  vex_post_rts_2a;
      o_vex_mem_lqid_3c                   <= o_vex_mem_lqid_2c;
   end

   //Note: I'm adding another leg that muxes in iterate_data_3a, even though its not really required(thats true atleast for reduction ops).
   //I have ensured in ID, that once the first cycle of reduction op goes through all iterations go thorugh, as full conditions are ignored. Its better to have this mux in case eco is needed.
   always_comb
     unique case(1'b1)
       idata_vld_2a & out_from_vec_int_2a: prermw_wrdata_2a[VLEN-1:0] = idata_2a[VLEN-1:0];
       fwren_2a:                           prermw_wrdata_2a[VLEN-1:0] = fwrdata_2a[VLEN-1:0];
       default:                            prermw_wrdata_2a[VLEN-1:0] = iterate_data_saved_3a[VLEN-1:0];
     endcase
      
   always_comb begin
      unique case(1'b1)
        imulen_0a: 
          begin
             mulsrc2_0a        = imulsrc2_0a;
             mulsrc1_0a        = imulsrc1_0a;
             //mulissgn_0a       = imulissgn_0a;
             mulen_0a          = 1'b1;
             fmulen_0a         = 1'b0; 
          end
        default:
          begin
             mulsrc2_0a        = 'x;
             mulsrc1_0a        = 'x;
             //mulissgn_0a       = 'x;
             mulen_0a          = '0;
             fmulen_0a         = '0;
          end
      endcase
   end

   assign o_vex_mem_lqdata_1c[VLEN-1:0]   =   dstwr_bitmask_1a[VLEN-1:0] & prermw_wrdata_1a[VLEN-1:0] 
                                        | ~dstwr_bitmask_1a[VLEN-1:0] & rddst_data_1a[VLEN-1:0];

   assign o_vex_mem_lqdata_2c[VLEN-1:0]   =   dstwr_bitmask_2a[VLEN-1:0] & prermw_wrdata_2a[VLEN-1:0] 
                                        | ~dstwr_bitmask_2a[VLEN-1:0] & rddst_data_2a[VLEN-1:0];
   
   assign o_vex_mem_lqdata_3c[VLEN-1:0]   = iterate_data_saved_3a[VLEN-1:0];  //mask applied in prior cycle to save rd dst flops

   assign o_vex_mem_lqexc_1c           = fwren_1a ? fwrexc_1a : '0; 
   assign o_vex_mem_lqexc_2c           = fwren_2a ? fwrexc_2a : '0;

   always_ff @(posedge i_clk, negedge i_reset_n) begin
      if (~i_reset_n) begin
         o_vex_mem_lqexc_3c <= '0;
      end else begin
         o_vex_mem_lqexc_3c <= (nrwop_2a && fwren_2a) ? fwrexc_2a : '0;
      end
   end
   
   //~nrwop_lmul_1a to disable write in 1c, since 2c is when the  narrow ops should create data.
   assign o_vex_mem_lqvld_1c           = vex_mem_rts & (   out_from_vec_int_1a & (idata_vld_1a & ~mask_only_instrn_1a & ~rfwren_1a & ~rnden_1a & ~nrwop_1a & ~nrwop_lmul_1a & ~iterate_1a)
                                                        |  out_from_vec_int_1a & (idata_vld_1a & (mask_only_instrn_1a | rfwren_1a) & mask_only_rts_1a)
                                                        |  (fwren_1a & (~mask_only_instrn_1a || mask_only_rts_1a) & ~vfp_hole_vld_1a & ~nrwop_1a & ~iterate_1a));

   assign o_vex_mem_lqvld_2c           = (~iterate_or_nrw_2a & idata_vld_2a & out_from_vec_int_2a) | fwren_2a;
   assign o_vex_mem_lqvld_3c           = sel_iterate_or_nrw_3a; 
  
   
      
   always_ff @(posedge i_clk) begin
      if(~i_reset_n) begin
         iterate_1a                        <= '0;
         reductop_1a                       <= '0;
         id_replay_1a                      <= '0;
         nrwop_1a                          <= '0;
         wdeop_1a                          <= '0;
         inc_iterate_1a                    <= '0;
         vslideop_1a                       <= '0;
         vgatherall_1a                     <= '0;
         vrgatherei16op_1a                 <= '0;
         vex_bitwise_1a                    <= '0;
         reduct_wdeop_1a                   <= '0;
         compress_rts_1a                   <= '0;  
         vslidedwn_1a                      <= '0;
         vslideup_1a                       <= '0; 
      end else if(vex_en_0a | vex_en_1a) begin
         iterate_1a                        <= iterate_0a;        
         reductop_1a                       <= reductop_0a;       
         id_replay_1a                      <= i_id_replay;       
         nrwop_1a                          <= nrwop_0a;          
         wdeop_1a                          <= wdeop_0a;          
         inc_iterate_1a                    <= inc_iterate_0a;    
         vslideop_1a                       <= vslideop_0a;       
         vgatherall_1a                     <= vgatherall_0a;     
         vrgatherei16op_1a                 <= vrgatherei16op_0a; 
         vex_bitwise_1a                    <= vex_bitwise_0a;    
         reduct_wdeop_1a                   <= reduct_wdeop_0a;   
         compress_rts_1a                   <= compress_rts_0a; 
         vslidedwn_1a                      <= vslidedwn_0a;
         vslideup_1a                       <= vslideup_0a; 
      end
   end

   always_comb
     if((compress_0a | reductop_0a) & ~i_csr.v_lmul[2])
       force_completion_rts_0a = ((iterate_cnt_max_0a - 'd1) == tot_iterate_cnt_0a);
     else 
       force_completion_rts_0a =  ( &tot_iterate_cnt_0a[0+:$clog2(VLEN/ 8)] & (i_csr.v_vsew[1:0]==2'b00))
                                 |( &tot_iterate_cnt_0a[0+:$clog2(VLEN/16)] & (i_csr.v_vsew[1:0]==2'b01))
                                 |( &tot_iterate_cnt_0a[0+:$clog2(VLEN/32)] & (i_csr.v_vsew[1:0]==2'b10)) 
                                 |( &tot_iterate_cnt_0a[0+:$clog2(VLEN/64)] & (i_csr.v_vsew[1:0]==2'b11));
   always_ff @(posedge i_clk)
     if(vex_en_0a | vex_en_1a) begin
        vsetvli_kill_1a                   <= (opcode_0a == 7'b101_0111 & funct3_0a == 3'b111);
        {mulen_1a}                        <= mulen_0a;
        rnden_1a                          <= rnden_0a;
        scalar_dest_1a                    <= scalar_dest_0a;
        mask_only_instrn_1a               <= mask_only_instrn_0a;
        rfwren_1a                         <= rfwren_0a;
        lmul_1a[2:0]                      <= i_csr.v_lmul[2:0];
        lmul_cnt_1a[2:0]                  <= lmul_cnt_0a[2:0];
        vm0_muxed_1a                      <= vm0_muxed_0a;
        vl_muxed_1a                       <= vl_muxed_0a;
        vl_cnt_1a                         <= i_csr.v_vl;
        v_vm_1a                           <= i_v_vm;
        wrmask_1a                         <= i_id_vec_autogen.wrmask;
        compress_1a                       <= compress_0a;
        force_completion_rts_1a           <= force_completion_rts_0a;
        scalar_imm_slide_1a[63:0]         <= scalar_imm_slide_0a[63:0];  //this is now overloaded and is more than scalar_imm_slide it also has the next rf addr to pick from gather ops
        iterate_cnt_max_1a                <= iterate_cnt_max_0a; 
     end 
   
   always_ff @(posedge i_clk)
     if(vex_en_0a) begin
        wraddr_1a[4:0]                  <= i_id_vec_autogen.rf_addrp2[4:0];
        dstwr_bytemask_1a               <= dstwr_bytemask_0a;
        
        vsew_1a[1:0]                    <= i_csr.v_vsew[1:0];
        two_cycle_iterate_1a            <= two_cycle_iterate_0a;
        o_vex_mem_lqid_1c               <= i_id_vec_autogen.ldqid;
        nrwop_lmul_1a                   <= nrwop_lmul_0a;
        //iterate_cnt_1a[3:0]           <= iterate_cnt_0a[3:0]; 
     end

   //making the control for rddst_data_1a more complex as the bitwise instructions write a few bits per lmul iteration and I don't want to spend additional flops for holding the intermediate data.
   always_ff @(posedge i_clk)
     if(vex_en_0a & ~mask_only_instrn_0a)
       {rddst_data_1a[VLEN-1:0]}           <= {i_id_vec_autogen.rf_rd_p2_is_rs2 ? src2_0a[VLEN-1:0] : src3_0a[VLEN-1:0]};
     else if(mask_only_instrn_0a & (tot_iterate_cnt_0a[2:0]==0))
       rddst_data_1a[VLEN-1:0]             <= src3_0a[VLEN-1:0];
     else if(mask_only_instrn_1a & ~lmul_1a[2] & |lmul_1a[1:0])
       for(int i=0;i<VLEN;i++)
         rddst_data_1a[i]               <= mask_only_bitmask_1a[i] ? prermw_wrdata_1a[i] : rddst_data_1a[i];
   
   always_ff @(posedge i_clk)
     if(vex_en_1a) begin
        wraddr_2a[4:0]                  <= wraddr_1a[4:0]; 
        dstwr_bytemask_2a               <= dstwr_bytemask_1a;
        vsew_2a[1:0]                    <= vsew_1a[1:0];
        two_cycle_iterate_2a            <= two_cycle_iterate_1a;
        o_vex_mem_lqid_2c               <= o_vex_mem_lqid_1c;
     end
   
   logic [VLEN-1:0] rddst_data_mux_1a;
   always_comb
     case(vsew_1a + reduct_wdeop_1a)
       2'b00: rddst_data_mux_1a[VLEN-1:0] = {rddst_data_1a[VLEN-1:8 ],prermw_wrdata_1a[ 7:0]};
       2'b01: rddst_data_mux_1a[VLEN-1:0] = {rddst_data_1a[VLEN-1:16],prermw_wrdata_1a[15:0]};
       2'b10: rddst_data_mux_1a[VLEN-1:0] = {rddst_data_1a[VLEN-1:32],prermw_wrdata_1a[31:0]};
       2'b11: rddst_data_mux_1a[VLEN-1:0] = {rddst_data_1a[VLEN-1:64],prermw_wrdata_1a[63:0]};
       default: rddst_data_mux_1a[VLEN-1:0] = 'x;
     endcase

   logic [63:0] prermw_mux_1a;
   always_comb
     case(vsew_1a + reduct_wdeop_1a)
       2'b00: prermw_mux_1a[63:0] = {rddst_data_2a[63:8 ],prermw_wrdata_1a[ 7:0]};
       2'b01: prermw_mux_1a[63:0] = {rddst_data_2a[63:16],prermw_wrdata_1a[15:0]};
       2'b10: prermw_mux_1a[63:0] = {rddst_data_2a[63:32],prermw_wrdata_1a[31:0]};
       2'b11: prermw_mux_1a[63:0] = {                     prermw_wrdata_1a[63:0]};
       default: prermw_mux_1a[63:0] = 'x;
     endcase
       
   //Note:REDUCT,Saving prior iteration results, if RTS goes way in middle of a REDUCT OP, Perhaps, will never happen, taken care of in ID. THis is fail safe.
   //well units_rtr is killed by ex_rtr which is still dependent on mem_rtr, but under replay vex unit doesnt care about RTR for reduction operation, as no new lq entry is consumed.
   always_ff @(posedge i_clk)
     if(vex_en_1a & (~id_replay_1a | ~reductop_1a))
       {rddst_data_2a[VLEN-1:0]}           <= reductop_1a ? rddst_data_mux_1a[VLEN-1:0]
                                                       : {rddst_data_1a[VLEN-1:0]};
     else if(vex_en_1a & reduct_wdeop_1a)
       {rddst_data_2a[63:0]}            <= {prermw_mux_1a[63:0]}; 
     else if(vex_en_1a & reductop_1a)
       {rddst_data_2a[63:0]}            <= {prermw_mux_1a[63:0]};
   
   always_ff @(posedge i_clk)
     if(iterate_1a | iterate_2a) begin
        lmul_cnt_2a[2:0]                <= lmul_cnt_1a[2:0];
        inc_iterate_2a                  <= inc_iterate_1a;
     end

   always_ff @(posedge i_clk) begin
      vex_en_1a                         <= vex_en_0a;
      vex_en_2a                         <= vex_en_1a;
      out_from_vec_int_1a               <= out_from_vec_int_0a;
      out_from_vec_int_2a               <= out_from_vec_int_1a;
   end
   
   //This is a functional gater, use it in 1a of the next cycle(essentially a 2a)
   always_ff@(posedge i_clk)
     if(vgatherall_0a & ~inc_iterate_0a) begin
        vs1_gather_indx_1a <= inc_addrp1_0a[$clog2(VLEN/8)-1:0]; 
     end

 
   always_ff @(posedge i_clk)
     if(vex_en_0a & ~i_id_replay) begin
        base_addrp0                     <= reg_p0[4:0];
        base_addrp1                     <= reg_p1[4:0]; 
        base_addrp2                     <= i_id_vec_autogen.rf_addrp2[4:0]; 
     end
   
   wire [$clog2(VLEN/8)+2:0] tot_iterate_cnt_adjust_0a = tot_iterate_cnt_0a + {'0,(inc_iterate_0a | ~two_cycle_iterate_0a)};

   always_comb begin
      unique case(1'b1)
        vslideop_0a :
          case(i_csr.v_vsew[1:0])
            2'b00   : inc_addrp1_0a[63:0] = (slide_dwn_0a ? (tot_iterate_cnt_adjust_0a + slide_shft_amt_0a[63:0]) : slide_shft_amt_0a[63:0] > tot_iterate_cnt_0a ? '0 : (tot_iterate_cnt_adjust_0a - slide_shft_amt_0a[63:0])) ; 
            2'b01   : inc_addrp1_0a[63:0] = (slide_dwn_0a ? (tot_iterate_cnt_adjust_0a + slide_shft_amt_0a[63:0]) : slide_shft_amt_0a[63:0] > tot_iterate_cnt_0a ? '0 : (tot_iterate_cnt_adjust_0a - slide_shft_amt_0a[63:0])) ; 
            2'b10   : inc_addrp1_0a[63:0] = (slide_dwn_0a ? (tot_iterate_cnt_adjust_0a + slide_shft_amt_0a[63:0]) : slide_shft_amt_0a[63:0] > tot_iterate_cnt_0a ? '0 : (tot_iterate_cnt_adjust_0a - slide_shft_amt_0a[63:0])) ; 
            2'b11   : inc_addrp1_0a[63:0] = (slide_dwn_0a ? (tot_iterate_cnt_adjust_0a + slide_shft_amt_0a[63:0]) : slide_shft_amt_0a[63:0] > tot_iterate_cnt_0a ? '0 : (tot_iterate_cnt_adjust_0a - slide_shft_amt_0a[63:0])) ; 
            default : inc_addrp1_0a[63:0] = 'x;
          endcase
        vgatherall_0a:
          case(vsew_gather_0a[1:0])  //Note: itov_src_0a for sel_imm_0a is already 0 extended. src1_mux_0a is only looked at when its not immediate or scalar, and that is not zero extended as it has to be SEW size or EEW=16 size for gatherei16
            2'b00   : inc_addrp1_0a[63:0] = sel_xfi_src_0a ? usgn_itov_src_0a[63:0] : src1_mux_0a[tot_iterate_cnt_0a[0+:$clog2(VLEN/ 8)]* 8+: 8] ; 
            2'b01   : inc_addrp1_0a[63:0] = sel_xfi_src_0a ? usgn_itov_src_0a[63:0] : src1_mux_0a[tot_iterate_cnt_0a[0+:$clog2(VLEN/16)]*16+:16] ; 
            2'b10   : inc_addrp1_0a[63:0] = sel_xfi_src_0a ? usgn_itov_src_0a[63:0] : src1_mux_0a[tot_iterate_cnt_0a[0+:$clog2(VLEN/32)]*32+:32] ; 
            2'b11   : inc_addrp1_0a[63:0] = sel_xfi_src_0a ? usgn_itov_src_0a[63:0] : src1_mux_0a[tot_iterate_cnt_0a[0+:$clog2(VLEN/64)]*64+:64] ; 
            default : inc_addrp1_0a[63:0] = 'x;
          endcase
        
        default     : inc_addrp1_0a[63:0] = 'x;
      endcase 
   end

   //if timing fails on slide* move this logic in 1a, there are already slide_shft_amt_1a flops in tt_vec_iadd. Will then need to spend inc_addrp1_1a flops.
   always_comb
      unique case(1'b1)
        vslideup_1a:  sel_zero_or_vd_1a = |slide_shft_amt_1a[63:$clog2(VLEN/8)+3] 
                                          |        slide_shft_amt_1a[$clog2(VLEN/8)+2:0] > tot_iterate_cnt_1a; 
                
        vslidedwn_1a: sel_zero_or_vd_1a = |slide_shft_amt_1a[63:$clog2(VLEN/8)+3] 
                                          | ({1'b0,tot_iterate_cnt_1a} + {1'b0,slide_shft_amt_1a[$clog2(VLEN/8)+2:0]}) >= {2'b00,iterate_cnt_max_1a};
                                
        
        vgatherall_1a:sel_zero_or_vd_1a =  (slide_shft_amt_1a[63:0] >= iterate_cnt_max_1a);
                                         
        default:      sel_zero_or_vd_1a = '0;
      endcase
      
   assign sized_iterate_addrp1_0a[4:0] = inc_addrp1_0a[63:0] >> ($clog2(VLEN/8) - i_csr.v_vsew[1:0]); //spyglass disable STARC05-2.10.3.2b_sa  //SEW based shift to check whether the offset is greater than the range of the max address for that group
   logic sgn_extby4_0a; assign sgn_extby4_0a = (funct7_0a[6:1] == 6'b01_0010) & (funct3_0a[2:0] == `OPMVV) & ((i_id_ex_instrn[19:15] == 5'b00100) | (i_id_ex_instrn[19:15] == 5'b00101)); //MM Nov 5 2021: Remove implicit wire.
   logic sgn_extby2_0a; assign sgn_extby2_0a = (funct7_0a[6:1] == 6'b01_0010) & (funct3_0a[2:0] == `OPMVV) & ((i_id_ex_instrn[19:15] == 5'b00110) | (i_id_ex_instrn[19:15] == 5'b00111)); //MM Nov 5 2021: Remove implicit wire.
   assign o_iterate_addrp0[4:0]        = (~i_id_replay ? reg_p0[4:0] : base_addrp0[4:0]) | {2'd0,emul_cnt_foraddr_0a[2:0] & ~{3{compress_0a}} & {3{(i_id_type[4:0] == `BRISCV_INSTR_TYPE_Vvv)}}} ;
   assign o_iterate_addrp1[4:0]        = (~i_id_replay ? reg_p1[4:0] : base_addrp1[4:0]) | ((iterate_0a & two_cycle_iterate_0a & ~reductop_0a) ? sized_iterate_addrp1_0a[4:0] : {2'b00,lmul_cnt_foraddr_0a[2:0]} >> {sgn_extby4_0a,sgn_extby2_0a});
                                                                                            
    //Its fine to use 1a for compress_rts_1a, as it takes a min of 4 cycles(for 32bit) to write to destination; Also I'm updating the remaining bytes or the ungathered bytes in the last cycle of the lmul iteration
   assign o_iterate_addrp2[4:0]        = (~i_id_replay ? i_id_vec_autogen.rf_addrp2[4:0] : base_addrp2[4:0]) + (compress_0a ? {2'd0,compress_cnt_foraddr_0a[2:0]} : {2'd0,lmul_cnt_foraddr_0a[2:0] & {3{~o_ignore_dstincr}}});

   always_comb
     if(compress_0a)
       o_vex_id_incr_addrp2 = compress_rts_0a;
     else if(o_ignore_dstincr)
       o_vex_id_incr_addrp2 = 1'b0;
     else
       case(i_csr.v_vsew[1:0])
         2'b00:   o_vex_id_incr_addrp2 = &tot_iterate_cnt_0a[0+:$clog2(VLEN/ 8)] & vex_en_0a & inc_iterate_0a;
         2'b01:   o_vex_id_incr_addrp2 = &tot_iterate_cnt_0a[0+:$clog2(VLEN/16)] & vex_en_0a & inc_iterate_0a;
         2'b10:   o_vex_id_incr_addrp2 = &tot_iterate_cnt_0a[0+:$clog2(VLEN/32)] & vex_en_0a & inc_iterate_0a;
         2'b11:   o_vex_id_incr_addrp2 = &tot_iterate_cnt_0a[0+:$clog2(VLEN/64)] & vex_en_0a & inc_iterate_0a;
         default: o_vex_id_incr_addrp2 = 'x;
       endcase
         
   always_ff @(posedge i_clk) begin
      if(~i_reset_n)
        inc_iterate_0a                  <= '0;
      else if(iterate_0a & ~inc_iterate_0a) 
        inc_iterate_0a                  <= '1;
      else if(iterate_0a & inc_iterate_0a)
        inc_iterate_0a                  <=  0;
   end

   assign two_cycle_iterate_0a = vslideop_0a | vgatherall_0a;// | reductop_0a; vex_en_0a & ~i_id_vec_autogen.onecycle_iterate;
   //Keep track of total iterate _cnt and derive lmul_cnt based on sew.
   always_ff @(posedge i_clk) begin
      if(~i_reset_n)
        tot_iterate_norply_cnt_0a              <= '0;
      else if(~i_id_replay & vex_en_0a & (iterate_0a | nrwop_0a | wdeop_0a | ~i_csr.v_lmul[2] & |i_csr.v_lmul[1:0]))
        tot_iterate_norply_cnt_0a              <= {'0,~two_cycle_iterate_0a};
      else if(i_id_replay & vex_en_0a  & (iterate_0a | nrwop_0a | wdeop_0a | ~i_csr.v_lmul[2] & |i_csr.v_lmul[1:0]))
        tot_iterate_norply_cnt_0a              <= tot_iterate_cnt_0a + {'0,{inc_iterate_0a | ~two_cycle_iterate_0a}};
      else if(~i_id_replay )
        tot_iterate_norply_cnt_0a              <= '0; //reset to 0 after iteration is done. 
   end
   assign tot_iterate_cnt_0a = tot_iterate_norply_cnt_0a & {$clog2(VLEN/8)+3{i_id_replay}};
   always_ff @(posedge i_clk) begin 
      if(~i_reset_n)
        tot_iterate_cnt_1a <= '0;
      else if( vex_en_1a)// & (iterate_1a | nrwop_1a | wdeop_1a))
        tot_iterate_cnt_1a <= tot_iterate_cnt_0a; //& {7{i_id_replay}};  
      else if(~id_replay_1a)
        tot_iterate_cnt_1a <= '0; //reset to 0 after iteration is done.
   end
   
   //For vslide/gather op increment lmul iter cnt in every other cycle,(because each iteration is 2 cycles)
   //I need 2 versions for lmul_cnt, one for the rf addr which has to be a cycle earlier, so that in the next cycle the correct address is recieved. This is because there is a flop through which the address increment goes through.
   //the other version of lmul_cnt is needed in the same cycle to pick the correct mask bits.
   assign compress_cnt_foraddr_0a = {3{i_id_replay}} & (compress_cnt_0a + ({'0,compress_byte_en_0a})) >> ($clog2(VLEN/8) - i_csr.v_vsew[1:0]); //spyglass disable STARC05-2.10.3.2b_sa
   assign lmul_cnt_foraddr_0a     = {3{i_id_replay}} & (tot_iterate_cnt_0a + ({'0,~two_cycle_iterate_0a | inc_iterate_0a})) >> ($clog2(VLEN/8) - i_csr.v_vsew[1:0]); //spyglass disable STARC05-2.10.3.2b_sa
   assign emul_cnt_foraddr_0a     = {3{i_id_replay}} & (tot_iterate_cnt_0a + ({'0,~two_cycle_iterate_0a | inc_iterate_0a})) >> ($clog2(VLEN/8) - vsew_gather_0a[1:0]); //spyglass disable STARC05-2.10.3.2b_sa
   assign lmul_cnt_0a             = {3{i_id_replay}} & (iterate_0a ? tot_iterate_cnt_0a >> ($clog2(VLEN/8) - i_csr.v_vsew[1:0]) //spyglass disable STARC05-2.10.3.2b_sa
                                                                   : tot_iterate_cnt_0a >> (nrwop_0a | wdeop_0a));  //every other op produces 128 bits except for narrow.
      
   assign vex_vmaskbit_0a = vex_en_0a &( (funct7_0a[6:1] == 6'b010_000) & (funct3_0a[2:0] == `OPMVV) & (reg_p0[4:0] == 5'b10000)
                                        |(funct7_0a[6:1] == 6'b010_000) & (funct3_0a[2:0] == `OPMVV) & (reg_p0[4:0] == 5'b10001)
                                        |(funct7_0a[6:1] == 6'b010_100) & (funct3_0a[2:0] == `OPMVV) & (reg_p0[4:0] == 5'b00011)
                                        |(funct7_0a[6:1] == 6'b010_100) & (funct3_0a[2:0] == `OPMVV) & (reg_p0[4:0] == 5'b00001)
                                        |(funct7_0a[6:1] == 6'b010_100) & (funct3_0a[2:0] == `OPMVV) & (reg_p0[4:0] == 5'b00010)
                                        |(funct7_0a[6:1] == 6'b010_100) & (funct3_0a[2:0] == `OPMVV) & (reg_p0[4:0] == 5'b10000)                     
                                        |(funct7_0a[6:1] == 6'b010_100) & (funct3_0a[2:0] == `OPMVV) & (reg_p0[4:0] == 5'b10001));

   
   assign mask_only_instrn_0a = vex_en_0a & i_id_vec_autogen.mask_only;
   assign rfwren_0a           = vex_en_0a & ~i_id_vec_autogen.rf_wren & ~o_ignore_lmul; //popc etc which write to int rf, ignore_lmul is here, because these instruction play only once irrespective of lmul setting
   assign src1_0a[VLEN-1:0] = i_vrf_p0_rddata[VLEN-1:0];
   assign src2_0a[VLEN-1:0] = i_vrf_p1_rddata[VLEN-1:0];
   assign src3_0a[VLEN-1:0] = i_vrf_p2_rddata[VLEN-1:0];
   assign vm0_0a[VLEN-1:0]  = i_vrf_vm0_rddata[VLEN-1:0];

   // VFP shared mul input muxing
   wire [3:0] fp_input_fmt = {i_csr.v_vsew[1:0]==2'd3 /*FP64*/, i_csr.v_vsew[1:0]==2'd2 /*FP32*/, i_csr.v_vsew[1:0]==2'd1 /*FP16A*/, 1'b0 /*FP16B*/ };
   wire       fp16_on_fp32_phase = i_id_vec_autogen.wdeop         &&
                                   i_id_vec_autogen.replay_cnt[0];

        //VFP32/VFP16 lanes
       // tt_vfp_array vfp_array
       //(
       //.i_clk                 (i_clk), 
       //.i_reset_n             (i_reset_n),
       //.i_id_vfp_ex0_rts      (i_id_vex_rts & i_id_vec_autogen.vfp_rf_rd_op_valid  & ~i_id_vec_autogen.out_from_vec_int), // set for valid vfp instructions that are executed in fp side.
       //.i_id_vfp_autogen      (i_id_vec_autogen),
       //.i_rddata              ({src3_0a[VLEN-1:0],src2_0a[VLEN-1:0],src1_mux_0a[VLEN-1:0]}),
       //.i_vm0                 (vm0_muxed_0a & vl_muxed_0a),
       //.i_reduct_wdeop        (reduct_wdeop_0a),
       //.i_lmul_cnt            (lmul_cnt_1a),
       //.i_prod                (mulsum_1a),
       //.i_sew                 (i_csr.v_vsew[1:0]),
       //.i_rs1                 (reg_p0[4:0]), //,,
       //.i_rs2                 (reg_p1[4:0]),
       //.i_rs3                 (reg_p2[4:0]),
       //          
       //.i_fp16_on_fp32_phase  (fp16_on_fp32_phase),
       //
       //.o_result                 (fwrdata_1a),
       //.o_result_valid           (fwren_1a),
       //.o_result_hole_valid      (vfp_hole_vld_1a),
       //.o_result_ooo_data_valid  (fwren_2a),
       //.o_result_ooo_data        (fwrdata_2a),

       //.o_vfp_exc_update         (o_vfp_exc_update)
       //          
       // );
   tt_vfp_unit #(.NUM_LANE(VLEN/64))
   vfp
   (
      .i_clk                    (i_clk), 
      .i_reset_n                (i_reset_n),
      .i_id_vfp_ex0_rts         (i_id_vex_rts & i_id_vec_autogen.vfp_rf_rd_op_valid  & ~i_id_vec_autogen.out_from_vec_int), // set for valid vfp instructions that are executed in fp side.
      .i_id_vfp_autogen         (i_id_vec_autogen),
      .i_lmul_cnt               (lmul_cnt_0a),
      .i_rddata                 ({src3_0a[VLEN-1:0],src2_0a[VLEN-1:0],src1_mux_0a[VLEN-1:0]}),
      .i_vm0                    (vm0_muxed_0a & vl_muxed_0a),
      .i_sew                    (i_csr.v_vsew[1:0]),
      .i_xrm                    (i_csr.v_vxrm),
      .i_frm                    (i_csr.frm),
      .i_funct6                 (funct7_0a[6:1]),
      .i_vs1                    (reg_p0[4:0]),
                
      .o_result_valid           (fwren_1a),
      .o_result                 (fwrdata_1a),
      .o_result_exc             (fwrexc_1a),
      .o_result_hole_valid      (vfp_hole_vld_1a),
      .o_result_ooo_data_valid  (fwren_2a),
      .o_result_ooo_data        (fwrdata_2a),
      .o_result_ooo_exc         (fwrexc_2a)

   );

   
   //Note these inputs are 0a and outputs are 2a...
   /*tt_vec_idp AUTO_TEMPLATE (
    //.i_\(.*\)_\(.*?\)a      (\1_@"(+ 0 @)"a[]),
    //.o_slide\(.*\)           (),
    .i_iterate_cnt_\(.*?\)   (tot_iterate_cnt_\1[]),
    .i_\(.*\)                 (\1[]),
    .o_\(.*\)                 (\1[]),
    .i_clk                    (i_clk),
    .i_v_vm_0a		      (i_v_vm),
     
    .i_id_replay_0a           (i_id_replay),
    )*/
   tt_vec_idp #(.VLEN(VLEN),
                .XLEN(XLEN) )
   idp(//Inputs
                  .i_vxrm_0a            (i_csr.v_vxrm),                   
                  .i_issgn_src1_0a      (i_id_vec_autogen.issgn_src1),   
                  .i_issgn_src2_0a      (i_id_vec_autogen.issgn_src2),   
                  
                  .i_adden_0a           (iadden_0a),             
                  .i_vsew_0a            (i_csr.v_vsew[1:0]),
                  .i_lmul_0a            (i_csr.v_lmul[2:0]),              
                  .i_addorsub_0a        (i_id_vec_autogen.addorsub),  
                  .i_src1hw_0a          (i_id_vec_autogen.src1hw),
                  .i_inversesub_0a      (i_id_vec_autogen.inversesub),   
                  .i_mask_0a            (vm0_muxed_0a & vl_muxed_0a),
                  .i_usemask_0a         (i_id_vec_autogen.usemask),              
                  .i_wrmask_0a          (i_id_vec_autogen.wrmask),               
                  .i_mulh_0a            (i_id_vec_autogen.mulh),                 
                  .i_mulen_0a           (mulen_0a),              
                  .i_macc_0a            (imul_accen_0a),                 
                  .i_rnden_0a           (rnden_0a),              
                  .i_cmpmul_0a          (i_id_vec_autogen.cmpmul),       
                  .i_mulsum_1a          (mulsum_1a),
                  .i_lmul_cnt_0a        (lmul_cnt_0a),      
                  .i_nrwop_0a           (nrwop_lmul_0a),                 
                  // Outputs
     
                  .o_sized_src1_0a      (imulsrc1_0a),
                  .o_sized_src2_0a      (imulsrc2_0a),
                  //.o_mulissgn_0a      (imulissgn_0a),
                  .o_data_vld_2a        (idata_vld_2a),
                  .o_data_2a            (idata_2a[VLEN-1:0]),
                  .o_data_vld_1a        (idata_vld_1a),
                  .o_data_1a            (idata_1a[VLEN-1:0]),
                  .i_ixv_tov_mv_0a      (ixv_tov_mv_0a),        
                  .i_src1_0a            (src1_mux_0a[VLEN-1:0]),    
                  .i_reset_n            (i_reset_n),             
                  /*AUTOINST*/
                  // Outputs
                  .o_v_tox_mv_1a        (v_tox_mv_1a),           // Templated
                  .o_sat_csr_2a         (sat_csr_2a),            // Templated
                  .o_slide_dwn_1a       (slide_dwn_1a),          // Templated
                  .o_slide_shft_amt_1a  (slide_shft_amt_1a[63:0]), // Templated
                  // Inputs
                  .i_clk                (i_clk),                 // Templated
                  .i_vex_en_0a          (vex_en_0a),             // Templated
                  .i_vex_en_1a          (vex_en_1a),             // Templated
                  .i_src2_0a            (src2_0a[VLEN-1:0]),        // Templated
                  .i_src3_0a            (src3_0a[VLEN-1:0]),        // Templated
                  .i_avg_0a             (avg_0a),                // Templated
                  .i_reductop_0a        (reductop_0a),           // Templated
                  .i_inc_iterate_0a     (inc_iterate_0a),        // Templated
                  .i_wdeop_0a           (wdeop_0a),              // Templated
                  .i_vm0_sized_0a       (vm0_sized_0a),    // Templated
                  .i_vmerge_0a          (vmerge_0a),             // Templated
                  .i_iterate_0a         (iterate_0a),            // Templated
                  .i_iterate_1a         (iterate_1a),            // Templated
                  .i_iterate_cnt_1a     (tot_iterate_cnt_1a), // Templated
                  .i_vl_cnt_1a          (vl_cnt_1a),        // Templated
                  .i_iterate_cnt_0a     (tot_iterate_cnt_0a), // Templated
                  .i_vs1_gather_indx_1a (vs1_gather_indx_1a), // Templated
                  .i_reduct_wdeop_1a    (reduct_wdeop_1a),       // Templated
                  .i_vmvgrp_0a          (vmvgrp_0a),             // Templated
                  .i_v_tox_mv_0a        (v_tox_mv_0a),           // Templated
                  .i_vex_per_0a         (vex_per_0a),            // Templated
                  .i_vex_shft_0a        (vex_shft_0a),           // Templated
                  .i_vex_bitwise_0a     (vex_bitwise_0a),        // Templated
                  .i_vex_vmaskbit_0a    (vex_vmaskbit_0a),       // Templated
                  .i_id_replay_0a       (i_id_replay),           // Templated
                  .i_mask_only_instrn_1a(mask_only_instrn_1a),   // Templated
                  .i_vex_instrn_1a      (vex_instrn_1a[31:0]),   // Templated
                  .i_reductop_data_0a   (reductop_data_0a[63:0]), // Templated
                  .i_scalar_imm_slide_1a(scalar_imm_slide_1a[63:0]), // Templated
                  .i_v_vm_1a            (v_vm_1a),               // Templated
		  .i_v_vm_0a		(i_v_vm),		 // Templated
                  .i_sat_instrn_0a      (sat_instrn_0a));        // Templated
   
                 

   tt_vec_mul_dp #(.VLEN(VLEN))
   mul_dp
                 (
                  
                  .o_sum_1a             (mulsum_1a),
                  .i_sized_src2_0a      (mulsrc2_0a),
                  .i_sized_src1_0a      (mulsrc1_0a),
                  
                  .i_issgn_0a           (i_id_vec_autogen.issgn_src1),
                  .i_issgnsrc2_0a       (i_id_vec_autogen.issgn_src2),
                  .i_mulen_0a           (mulen_0a | fmulen_0a),
                  .i_clk                (i_clk)
                  
                  /*AUTOINST*/);
   

endmodule // tt_vec
// Local Variables:
// verilog-library-directories:(".")
// verilog-library-extensions:(".sv" ".h" ".v")
// End:

