// See LICENSE.TT for license details.
module tt_vec_iadd #(parameter
   VLEN=256,
   XLEN=64
)
(         
		   input 		     i_reset_n,
		   input 		     i_vex_en_0a,
                   input 		     i_vex_en_1a,	    
		   input [1:0] 		     i_vxrm_0a, 
		   input 		     i_clk,
		   input [VLEN-1:0] 	     i_src1_0a,
		   input [VLEN-1:0] 	     i_src2_0a,
		   input [VLEN-1:0] 	     i_src3_0a,
		   input 		     i_issgn_src1_0a,
		   input 		     i_issgn_src2_0a,
		   input 		     i_adden_0a,
		   input 		     i_avg_0a,
		   input [1:0] 		     i_vsew_0a,
		   input [2:0] 		     i_lmul_0a, 
		   input 		     i_addorsub_0a, // 1  = sub and 0 = add

		   input 		     i_inc_iterate_0a,
		   input 		     i_reductop_0a,
		   input 		     i_src1hw_0a,
		   input 		     i_wdeop_0a,
		   input 		     i_inversesub_0a, //subtract vs2 instead of vs1
		   input [VLEN/8-1:0] 	     i_mask_0a, //Create crybrwmask
		   //input [15:0] 	     i_mask_1a,
		   input [VLEN/8-1:0] 	     i_vm0_sized_0a, 
		   input 		     i_vmerge_0a, 
		   input 		     i_usemask_0a, //to actually use mask for borrow or carry.
		   input 		     i_wrmask_0a,
		   input 		     i_ixv_tov_mv_0a,
		   input 		     i_vmvgrp_0a,
		   input 		     i_v_tox_mv_0a,
		   input 		     i_vex_per_0a,
		   input 		     i_vex_shft_0a,
		   input 		     i_vex_bitwise_0a,
		   input 		     i_vex_vmaskbit_0a,
		   input 		     i_id_replay_0a,
		   input 		     i_mask_only_instrn_1a,
		   input [31:0] 	     i_vex_instrn_1a,
		   input [2:0] 		     i_lmul_cnt_0a,
		   input [63:0] 	     i_reductop_data_0a, 
		   input 		     i_v_vm_0a,
		   input [XLEN-1:0] 	     i_scalar_imm_slide_1a,
		   input 		     i_v_vm_1a,
		   input 		     i_sat_instrn_0a,
		   input 		     i_iterate_0a,
		   input 		     i_iterate_1a,
		   input [$clog2(VLEN/8)+2:0] i_iterate_cnt_1a,
		   input [$clog2(VLEN+1)-1:0] i_vl_cnt_1a,
		   input [$clog2(VLEN/8)+2:0] i_iterate_cnt_0a,
		   input [$clog2(VLEN/8)-1:0] i_vs1_gather_indx_1a,
		   input 		     i_reduct_wdeop_1a,

		  
		   //output logic 	     o_slide1_dwn_1a,
		   //output logic 	     o_slide1_up_1a,
		   output logic 	     o_slide_dwn_1a,
		   output logic [XLEN-1:0] 	     o_slide_shft_amt_1a,
		   output logic [VLEN/16-1:0] 	     o_sat_vnclip_pos_1a,
		   output logic [VLEN/16-1:0] 	     o_sat_vnclip_neg_1a,
		   output logic 	     o_vnclipu_1a,
		   output logic 	     o_vnclip_1a,
		   output logic 	     o_avg_1a,
		   output logic [VLEN/8-1:0] 	     o_avgbitd_1a,
		   output logic [VLEN/8-1:0] 	     o_avgbitdm1_1a,
		   output logic [VLEN/8-1:0][7:0]    o_src2_rs8_dm1to0_1a,
		   output logic [VLEN/16-1:0] [15:0] o_src2_rs16_dm1to0_1a,
		   output logic [VLEN/32-1:0] [31:0] o_src2_rs32_dm1to0_1a,
		   output logic [VLEN/64-1:0] [63:0] o_src2_rs64_dm1to0_1a,
		   
		   output logic 	     o_v_tox_mv_1a,
		   output logic 	     o_data_vld_1a,
		   output logic [VLEN-1:0]   o_data_1a,
		   output logic              o_satval_2a
		 
		   );
   
   localparam WIDTH = 64; //SEW = 64 max
   logic [VLEN-1:0] 			vmsk_data_1a; 			
   logic [VLEN-1:0] 			final_src1_1a;
   logic [VLEN+7:-8]                    final_src2_1a;  //-1 to-8 is unused but lint can't figure that out.
   logic [VLEN-1:0] 			final_src1_0a,final_src2_0a;
 			
   logic [VLEN-1:0] 			prop_1a,gen_1a,sum_1a,satsum_1a;
   logic [VLEN-1:0] 			addres_1a,avgres_1a;
   logic [VLEN/8-1:0] 			cout_1a,c_in_1a,cin_1a,wrmask_data_1a,brw_bit_1a;
   logic [VLEN/8-1:0] 			src1_gt_src2_sized_1a,unsgn_src1_gt_src2_1a,sgn_src1_gt_src2_1a;	
   logic [VLEN/8-1:0] 			pos_unsgn_sat_1a,neg_unsgn_sat_1a,pos_sgn_sat_1a,neg_sgn_sat_1a,ovfl_1a;
   logic [VLEN/8-1:0] 			cryorbrw_mask_1a,cryorbrw_mask_0a; 			
   logic 				cryorbrw_1a,addorsub_1a,usemask_1a;
   logic 				wrmask_1a,sat_instrn_1a;
   logic [1:0] 				vsew_1a;
   logic 				ixv_tov_mv_1a,vmvgrp_1a;
   logic 				adden_1a;
   logic 				vex_per_1a,vex_shft_1a;
   logic 				src1_en_0a,src2_en_0a;
   logic [VLEN-1:0] 			vex_shft_data_1a, vex_bitwise_data_1a,vex_per_data_1a,vex_minmax_data_1a;
   logic 				vand_1a,vor_1a,vxor_1a,vredand_1a,vredor_1a,vredxor_1a,vex_bitwise_1a;
   logic [2:0] 				lmul_cnt_1a; 				
   logic [6:0] 				opcode_1a;	
   logic [2:0] 				funct3_1a;	 
   logic [6:0] 				funct7_1a;
   logic 				vpopc_1a,vffs_1a,vmsif_1a,vmsbf_1a,vmsof_1a,viota_1a,vid_1a,vex_maskbit_1a;
   logic [2:0]                          lmul_1a; 
   logic 				vmand_1a,vmnand_1a,vmandnot_1a,vmxor_1a,vmor_1a,vmnor_1a,vmornot_1a,vmxnor_1a;
   logic 				vminu_1a,vmin_1a,vmax_1a,vmaxu_1a,vmerge_1a,vredmax_1a,vredmin_1a;
   logic [VLEN/8-1:0] 			vmask_only_data_1a,elem_ltu_sized_1a,elem_lt_sized_1a;
   logic 				sel_minmax_1a;
   logic [VLEN/8-1:0] 			sgn_bits_differ_0a,sgn_bits_differ_1a;
   logic 				issgn_src1_1a;
   logic 				wdeop_1a,reductop_1a;
   logic [$clog2(VLEN/8)-1:0] 		cnt_for_mask_1a;
   logic [$clog2(VLEN+1)-1:0]           last_lmul_iter_result_1a,last_lmul_iter_result_0a; 				
   logic 				ffs_last_lmul_iter_1a,ffs_last_lmul_iter_0a;
   logic [$clog2(VLEN+1)-1:0] 		last_ffs_ptr_0a,last_ffs_ptr_1a; 			
   logic                                satval_1a;

   assign opcode_1a  = i_vex_instrn_1a[6:0];
   assign funct3_1a  = i_vex_instrn_1a[14:12];
   assign funct7_1a  = i_vex_instrn_1a[31:25];
   
   
   always_comb begin
      final_src1_0a = 'x;
      casez({i_inversesub_0a,i_addorsub_0a,i_src1hw_0a|i_wdeop_0a,i_reductop_0a})
	4'b0000: final_src1_0a = i_src1_0a; 
	
	4'b0100: final_src1_0a = ~i_src1_0a; 
	4'b1?00: final_src1_0a = ~i_src2_0a;
	4'b0?10: 
	  case(i_vsew_0a[1:0])
	    2'b00: for(int i=0; i<VLEN/16;  i++) final_src1_0a[i* 16+: 16] = { 16{i_addorsub_0a}} ^  {{8{i_issgn_src1_0a & i_src1_0a[i_iterate_cnt_0a[0]*VLEN/2+i*8+7]  }},i_src1_0a[i_iterate_cnt_0a[0]*VLEN/2+i* 8+: 8]} ;
	    2'b01: for(int i=0; i<VLEN/32;  i++) final_src1_0a[i* 32+: 32] = { 32{i_addorsub_0a}} ^ {{16{i_issgn_src1_0a & i_src1_0a[i_iterate_cnt_0a[0]*VLEN/2+i*16+15]}},i_src1_0a[i_iterate_cnt_0a[0]*VLEN/2+i*16+:16]} ;
	    2'b10: for(int i=0; i<VLEN/64;  i++) final_src1_0a[i* 64+: 64] = { 64{i_addorsub_0a}} ^ {{32{i_issgn_src1_0a & i_src1_0a[i_iterate_cnt_0a[0]*VLEN/2+i*32+31]}},i_src1_0a[i_iterate_cnt_0a[0]*VLEN/2+i*32+:32]} ;
	    2'b11: for(int i=0; i<VLEN/128; i++) final_src1_0a[i*128+:128] = {128{i_addorsub_0a}} ^ {{64{i_issgn_src1_0a & i_src1_0a[i_iterate_cnt_0a[0]*VLEN/2+i*64+63]}},i_src1_0a[i_iterate_cnt_0a[0]*VLEN/2+i*64+:64]} ;
	  endcase 
	4'b0?01: final_src1_0a[63:0] = {64{i_addorsub_0a}} ^ {i_reductop_data_0a};
	default: final_src1_0a = 'x; //includes 101 and 111 case, which are not possible , no inverse with src1hw
      endcase
      // end
   end
   //Note: For widening ops, src1 is always SEW size. Therefore the src1 register never increments in between 1 widening op
   //      For widening ops, src2 can be SEW or 2*SEW size. Therfore the src2 register will increment in the second cycle of widening op for 2*SEW size.
   always_comb
     casez({i_inversesub_0a,i_addorsub_0a,i_src1hw_0a,i_wdeop_0a,i_reductop_0a})
       5'b0?000: final_src2_0a =  i_src2_0a; 
       5'b1?000: final_src2_0a =  i_src1_0a; 
       5'b0?1?0: 
	 case(i_vsew_0a[1:0])
	   2'b00: for(int i=0; i<VLEN/16;  i++) final_src2_0a[i* 16+: 16] = {i_src2_0a[i* 16+: 16]} ;
	   2'b01: for(int i=0; i<VLEN/32;  i++) final_src2_0a[i* 32+: 32] = {i_src2_0a[i* 32+: 32]} ;
	   2'b10: for(int i=0; i<VLEN/64;  i++) final_src2_0a[i* 64+: 64] = {i_src2_0a[i* 64+: 64]} ;
	   2'b11: for(int i=0; i<VLEN/128; i++) final_src2_0a[i*128+:128] = {i_src2_0a[i*128+:128]} ;
	   default:                             final_src2_0a             = 'x;
	 endcase
       5'b0?010: 
	 case(i_vsew_0a[1:0])
	   2'b00: for(int i=0; i<VLEN/16;  i++) final_src2_0a[i* 16+: 16] = {{8 {i_issgn_src2_0a & i_src2_0a[i_iterate_cnt_0a[0]*VLEN/2+i*8 + 7]}},i_src2_0a[i_iterate_cnt_0a[0]*VLEN/2+i*8 +: 8]} ;
	   2'b01: for(int i=0; i<VLEN/32;  i++) final_src2_0a[i* 32+: 32] = {{16{i_issgn_src2_0a & i_src2_0a[i_iterate_cnt_0a[0]*VLEN/2+i*16+15]}},i_src2_0a[i_iterate_cnt_0a[0]*VLEN/2+i*16+:16]} ;
	   2'b10: for(int i=0; i<VLEN/64;  i++) final_src2_0a[i* 64+: 64] = {{32{i_issgn_src2_0a & i_src2_0a[i_iterate_cnt_0a[0]*VLEN/2+i*32+31]}},i_src2_0a[i_iterate_cnt_0a[0]*VLEN/2+i*32+:32]} ;
	   2'b11: for(int i=0; i<VLEN/128; i++) final_src2_0a[i*128+:128] = {{64{i_issgn_src2_0a & i_src2_0a[i_iterate_cnt_0a[0]*VLEN/2+i*64+63]}},i_src2_0a[i_iterate_cnt_0a[0]*VLEN/2+i*64+:64]} ;
	   default:                             final_src2_0a             = 'x;
	 endcase 
       5'b0?001: begin //for reductop, there is a single element selected per cycle, always considered widened.
	  final_src2_0a = 'x; 
	  case(i_vsew_0a[1:0])
	    2'b00:  final_src2_0a[ 15:0] = {{8 {i_issgn_src2_0a & i_src2_0a[i_iterate_cnt_0a[0+:$clog2(VLEN/ 8)]*8 + 7]}},i_src2_0a[i_iterate_cnt_0a[0+:$clog2(VLEN/ 8)]*8 +:8 ]} ;
	    2'b01:  final_src2_0a[ 31:0] = {{16{i_issgn_src2_0a & i_src2_0a[i_iterate_cnt_0a[0+:$clog2(VLEN/16)]*16+15]}},i_src2_0a[i_iterate_cnt_0a[0+:$clog2(VLEN/16)]*16+:16]} ;
	    2'b10:  final_src2_0a[ 63:0] = {{32{i_issgn_src2_0a & i_src2_0a[i_iterate_cnt_0a[0+:$clog2(VLEN/32)]*32+31]}},i_src2_0a[i_iterate_cnt_0a[0+:$clog2(VLEN/32)]*32+:32]} ;
	    2'b11:  final_src2_0a[127:0] = {{64{i_issgn_src2_0a & i_src2_0a[i_iterate_cnt_0a[0+:$clog2(VLEN/64)]*64+63]}},i_src2_0a[i_iterate_cnt_0a[0+:$clog2(VLEN/64)]*64+:64]} ;
	    default final_src2_0a       = 'x;
	  endcase 
       end
	 
       default: final_src2_0a = 'x;
     endcase 

   //assign cryorbrw_mask_0a[15:0] = i_lmul_0a[2] ? i_mask_0a[15:0] : i_mask_0a[127:0] >> (i_lmul_0a[1:0] * (16 >> i_vsew_0a[1:0]));
   assign cryorbrw_mask_0a = i_vmerge_0a ? i_vm0_sized_0a : i_mask_0a;
   assign cin_1a           = usemask_1a ? cryorbrw_mask_1a ^ {VLEN/8{addorsub_1a}} : {VLEN/8{addorsub_1a}};
   
   always_comb begin
     for(int i=0; i<8;      i++) sgn_bits_differ_0a[i] = i_reductop_0a ? final_src2_0a[i*8+7] ^ i_reductop_data_0a[i*8+7] : i_src2_0a[i*8+7] ^ i_src1_0a[i*8+7];
     for(int i=8; i<VLEN/8; i++) sgn_bits_differ_0a[i] =                                                                    i_src2_0a[i*8+7] ^ i_src1_0a[i*8+7];  //reduct data is max 64 bits wide.
   end
   always_comb begin
      for(int i=0;i<VLEN/8;i++) begin 
	 case(vsew_1a[1:0] + {1'b0,wdeop_1a | i_reduct_wdeop_1a})
	   2'b00: c_in_1a[i] =                 cin_1a[i];
	   2'b01: c_in_1a[i] = (i[0] == 0)   ? cin_1a[i/2]  : cout_1a[i-1];
	   2'b10: c_in_1a[i] = (i[1:0] == 0) ? cin_1a[i/4]  : cout_1a[i-1];
	   2'b11: c_in_1a[i] = (i[2:0] == 0) ? cin_1a[i/8]  : cout_1a[i-1];  //special case for widening ops
	 endcase 
      end
   end
   
   always_comb
     for(int i=0; i<VLEN/8; i++) begin
	prop_1a      [i*8+:8]  = final_src2_1a[i*8+:8] | final_src1_1a[i*8+:8];
	gen_1a       [i*8+:8]  = final_src2_1a[i*8+:8] & final_src1_1a[i*8+:8];
	
	
	{cout_1a[i], sum_1a[i*8+:8]} = {1'b0,gen_1a[i*8+:8]} + {1'b0,prop_1a[i*8+:8]} + {8'd0,c_in_1a[i]};
	//rely on adder for borrow determination when the MSB's are same(note src1 is inverted). 
	// 0100 - 0101 = 0100 + 1011 = 1111 ; sgn = 1
	// 1000 - 1000 = 1000 + 1000 = 0000 ; sgn = 0
	// 1111 - 1110 = 1111 + 0010 = 0001 ; sgn = 0
	// 1110 - 1111 = 1110 + 0001 = 1111 ; sgn = 1
	brw_bit_1a[i]                = sgn_bits_differ_1a[i] ? ~final_src1_1a[i*8+7] : sum_1a[i*8 + 7];  //Re-Inverting sgn of src1, since its inverted in 0a.
	pos_unsgn_sat_1a[i]          = sat_instrn_1a & ~issgn_src1_1a & ~addorsub_1a & cout_1a[i];
	neg_unsgn_sat_1a[i]          = sat_instrn_1a & ~issgn_src1_1a &  addorsub_1a & brw_bit_1a[i];
	pos_sgn_sat_1a[i]            = sat_instrn_1a &  issgn_src1_1a & ~(addorsub_1a ^ sgn_bits_differ_1a[i]) & (~final_src2_1a[i*8+7] & sum_1a[i*8+7]);
	neg_sgn_sat_1a[i]            = sat_instrn_1a &  issgn_src1_1a & ~(addorsub_1a ^ sgn_bits_differ_1a[i]) & ( final_src2_1a[i*8+7] & ~sum_1a[i*8+7]);
	//unsigned add, always include cout. unsigned sub can never ovfl , ignore cout,but use borrow for sign extension
	//signed add, include cout if sgn bits are same(1), signed sub , include cout if sign bits differ
	ovfl_1a[i]                   = ~addorsub_1a & ~issgn_src1_1a | ~(addorsub_1a ^ sgn_bits_differ_1a[i]) | brw_bit_1a[i] & addorsub_1a & ~issgn_src1_1a;

	//[i] isn't needed for reduction ops'(since single element) but I plan to remove compare operations altogther... IMPROVE_AREA, when compare operation use adder output
	//compare operations are always addorsub=1
	unsgn_src1_gt_src2_1a[i]     =  ~issgn_src1_1a &  brw_bit_1a[i];
	sgn_src1_gt_src2_1a[i]       =   issgn_src1_1a & (sgn_bits_differ_1a[i] ? final_src2_1a[i*8+7] : (sum_1a[i*8+7]));
     end

   always_comb
     case(vsew_1a[1:0] + {1'b0,wdeop_1a})
       2'b00: for(int i=0; i<VLEN/8;  i++) satsum_1a[i*8+:8]   = {8{pos_unsgn_sat_1a[i]}}      | {1'b0,{7{pos_sgn_sat_1a[i]}}}      | {neg_sgn_sat_1a[i],7'h0}      | sum_1a[i*8+:8]   & ~{8{ neg_sgn_sat_1a[i]     | neg_unsgn_sat_1a[i] | pos_sgn_sat_1a[i]}};
       2'b01: for(int i=0; i<VLEN/16; i++) satsum_1a[i*16+:16] = {16{pos_unsgn_sat_1a[i*2+1]}} | {1'b0,{15{pos_sgn_sat_1a[i*2+1]}}} | {neg_sgn_sat_1a[i*2+1],15'h0} | sum_1a[i*16+:16] & ~{16{neg_sgn_sat_1a[i*2+1] | neg_unsgn_sat_1a[i*2+1] | pos_sgn_sat_1a[i*2+1]}};
       2'b10: for(int i=0; i<VLEN/32; i++) satsum_1a[i*32+:32] = {32{pos_unsgn_sat_1a[i*4+3]}} | {1'b0,{31{pos_sgn_sat_1a[i*4+3]}}} | {neg_sgn_sat_1a[i*4+3],31'h0} | sum_1a[i*32+:32] & ~{32{neg_sgn_sat_1a[i*4+3] | neg_unsgn_sat_1a[i*4+3] | pos_sgn_sat_1a[i*4+3]}};
       2'b11: for(int i=0; i<VLEN/64; i++) satsum_1a[i*64+:64] = {64{pos_unsgn_sat_1a[i*8+7]}} | {1'b0,{63{pos_sgn_sat_1a[i*8+7]}}} | {neg_sgn_sat_1a[i*8+7],63'h0} | sum_1a[i*64+:64] & ~{64{neg_sgn_sat_1a[i*8+7] | neg_unsgn_sat_1a[i*8+7] | pos_sgn_sat_1a[i*8+7]}};
     endcase
     
   always_comb begin
      satval_1a = '0;
      case(vsew_1a[1:0])
           2'b00: for(int i=0; i<16; i++) satval_1a   |= pos_unsgn_sat_1a[i]     | pos_sgn_sat_1a[i]      | neg_sgn_sat_1a[i]     | neg_unsgn_sat_1a[i];
           2'b01: for(int i=0; i<8;  i++) satval_1a   |= pos_unsgn_sat_1a[i*2+1] | pos_sgn_sat_1a[i*2+1]  | neg_sgn_sat_1a[i*2+1] | neg_unsgn_sat_1a[i*2+1];
           2'b10: for(int i=0; i<4;  i++) satval_1a   |= pos_unsgn_sat_1a[i*4+3] | pos_sgn_sat_1a[i*4+3]  | neg_sgn_sat_1a[i*4+3] | neg_unsgn_sat_1a[i*4+3];
           2'b11: satval_1a = '0;
           default: satval_1a  = 'x;
      endcase
   end
         
   always_ff@(posedge i_clk) o_satval_2a <= satval_1a;

   always_comb begin
      wrmask_data_1a  = 'x;
      case(vsew_1a[1:0])
	2'b00: for(int i=0; i<VLEN/8;  i++) wrmask_data_1a[i] = (addorsub_1a ? brw_bit_1a[i]     : cout_1a[i]    );
	2'b01: for(int i=0; i<VLEN/16; i++) wrmask_data_1a[i] = (addorsub_1a ? brw_bit_1a[i*2+1] : cout_1a[i*2+1]);
	2'b10: for(int i=0; i<VLEN/32; i++) wrmask_data_1a[i] = (addorsub_1a ? brw_bit_1a[i*4+3] : cout_1a[i*4+3]);
	2'b11: for(int i=0; i<VLEN/64; i++) wrmask_data_1a[i] = (addorsub_1a ? brw_bit_1a[i*8+7] : cout_1a[i*8+7]);
	default: wrmask_data_1a  = 'x;
      endcase
   end
   //When unsigned subtract borrows, 0 out the results.
   always_comb
     case(vsew_1a[1:0])
       2'b00: for(int i=0; i<VLEN/8;  i++) avgres_1a[i*8+:8]   = {ovfl_1a[i]     ? (addorsub_1a & ~issgn_src1_1a ? brw_bit_1a[i]     :  cout_1a[i]    ) : issgn_src1_1a & satsum_1a[i*8+7]  ,satsum_1a[i*8+:8]}   >> 1; //spyglass disable STARC05-2.10.3.2b_sa
       2'b01: for(int i=0; i<VLEN/16; i++) avgres_1a[i*16+:16] = {ovfl_1a[i*2+1] ? (addorsub_1a & ~issgn_src1_1a ? brw_bit_1a[i*2+1] :  cout_1a[i*2+1]) : issgn_src1_1a & satsum_1a[i*16+15],satsum_1a[i*16+:16]} >> 1; //spyglass disable STARC05-2.10.3.2b_sa
       2'b10: for(int i=0; i<VLEN/32; i++) avgres_1a[i*32+:32] = {ovfl_1a[i*4+3] ? (addorsub_1a & ~issgn_src1_1a ? brw_bit_1a[i*4+3] :  cout_1a[i*4+3]) : issgn_src1_1a & satsum_1a[i*32+31],satsum_1a[i*32+:32]} >> 1; //spyglass disable STARC05-2.10.3.2b_sa
       2'b11: for(int i=0; i<VLEN/64; i++) avgres_1a[i*64+:64] = {ovfl_1a[i*8+7] ? (addorsub_1a & ~issgn_src1_1a ? brw_bit_1a[i*8+7] :  cout_1a[i*8+7]) : issgn_src1_1a & satsum_1a[i*64+63],satsum_1a[i*64+:64]} >> 1; //spyglass disable STARC05-2.10.3.2b_sa
       default:                            avgres_1a           =  'x;
     endcase
   //wire [6:0] wrmask_shft_amt;
   //always_comb
   //  case(vsew_1a[1:0])
   //    2'b00: wrmask_shft_amt[6:0] = lmul_cnt_1a[2:0] <<
	      
   assign addres_1a[VLEN-1:0]  =  wrmask_1a  ? (wrmask_data_1a[VLEN/8-1:0] << (lmul_cnt_1a[2:0] * (VLEN/8 >> vsew_1a[1:0])))  //spyglass disable STARC05-2.10.3.2b_sa
			                     : satsum_1a;
   //dm2to0 = 0 , dm1to0 = bit that gets thrown out. 
   always_comb for(int i=0; i<VLEN/8; i++) o_avgbitdm1_1a[i]   = satsum_1a[i*8];   //this bit gets thrown out on RS,
   always_comb for(int i=0; i<VLEN/8; i++) o_avgbitd_1a[i]     = satsum_1a[i*8+1];

   assign cnt_for_mask_1a = i_iterate_cnt_1a[$clog2(VLEN/8)-1:0] & ({'0, {$clog2(VLEN/8){1'b1}}} >> vsew_1a[1:0]);
   
   always_comb begin
      case(1'b1)
	adden_1a    
                      : o_data_1a = o_avg_1a ? avgres_1a
                                             : sel_minmax_1a ? ((~cryorbrw_mask_1a[cnt_for_mask_1a] & reductop_1a) ? {vex_bitwise_data_1a[VLEN-1:VLEN/2],~final_src1_1a[VLEN/2-1:0]} : vex_minmax_data_1a[VLEN-1:0])
					                     : ((~cryorbrw_mask_1a[cnt_for_mask_1a] & reductop_1a) ? {vex_bitwise_data_1a[VLEN-1:VLEN/2], final_src1_1a[VLEN/2-1:0]} : addres_1a         [VLEN-1:0]);
	
	vex_per_1a | i_iterate_1a & ~vex_bitwise_1a & ~adden_1a   
                      : o_data_1a = vex_per_data_1a;
	vex_shft_1a   : o_data_1a = vex_shft_data_1a;
	vex_bitwise_1a: o_data_1a = (cryorbrw_mask_1a[cnt_for_mask_1a] | ~reductop_1a) ? vex_bitwise_data_1a : {vex_bitwise_data_1a[VLEN-1:VLEN/2],final_src1_1a[VLEN/2-1:0]}; //when mask unset, avoid computed output
	ixv_tov_mv_1a : o_data_1a = final_src1_1a;
	vmvgrp_1a     : o_data_1a = final_src2_1a[VLEN-1:0];
	vex_maskbit_1a: o_data_1a = vmsk_data_1a;
	default       : o_data_1a = 'x;
      endcase
   end

   assign o_data_vld_1a     =  adden_1a | ixv_tov_mv_1a | vex_per_1a | vex_shft_1a | vex_bitwise_1a | vex_maskbit_1a | vmvgrp_1a;
   assign src1_en_0a        =  i_ixv_tov_mv_0a | i_adden_0a | i_vex_per_0a | i_vex_shft_0a | i_vex_bitwise_0a                     | i_iterate_0a;
   assign src2_en_0a        =                    i_adden_0a | i_vex_per_0a | i_vex_shft_0a | i_vex_bitwise_0a | i_vex_vmaskbit_0a | i_iterate_0a | i_vmvgrp_0a;


   /////////////////////Shft and logical instructions///////////////////////////////
   logic [VLEN/8-1:0][$clog2(XLEN)-1:0] shft_amt_1a,sized_shft_amt_1a;
   logic 	     shft_nrw_1a;
   logic 	     vsll_1a,vsrl_1a,vsra_1a,vnsra_1a,vnsrl_1a,vssrl_1a,vssra_1a;
   logic             funct3_vvxi,funct3_vvx,funct3_vxi;
   logic [VLEN/8-1:0][15:0] src2_ls_1a,src2_rs_1a;
   logic [VLEN/8-1:0] sel_rs_ext_1a,sel_ls_zero_1a;
   logic 	      vmseq_1a,vmsne_1a,vmsltu_1a,vmslt_1a,vmsleu_1a,vmsle_1a,vmsgtu_1a,vmsgt_1a;
   
   logic [VLEN-1:0]      src2_ls16_1a,src2_ls32_1a,src2_ls64_1a;
   logic [VLEN+ 8-1:0]   src2_rs8_1a;
   logic [VLEN+16-1:0]   src2_rs16_1a;
   logic [VLEN+32-1:0]   src2_rs32_1a;
   logic [VLEN+64-1:0]   src2_rs64_1a;
   logic [VLEN-1:0]      src2_ls8_1a;
   
   wire [1:0] 	      sew_nrw_1a = vsew_1a[1:0] + {1'b0,shft_nrw_1a}; 	   
   assign funct3_vvxi     = (funct3_1a[2:0] == `OPIVV) | (funct3_1a[2:0] == `OPIVX) | (funct3_1a[2:0] == `OPIVI);
   assign funct3_vvx      = (funct3_1a[2:0] == `OPIVV) | (funct3_1a[2:0] == `OPIVX);
   assign funct3_vxi      =                              (funct3_1a[2:0] == `OPIVX) | (funct3_1a[2:0] == `OPIVI);
   assign vsll_1a         = vex_shft_1a & (funct7_1a[6:1] == 6'b10_0101) & funct3_vvxi;
   assign vsrl_1a         = vex_shft_1a & (funct7_1a[6:1] == 6'b10_1000) & funct3_vvxi;
   assign vsra_1a         = vex_shft_1a & (funct7_1a[6:1] == 6'b10_1001) & funct3_vvxi;
   assign vnsra_1a        = vex_shft_1a & (funct7_1a[6:1] == 6'b10_1101) & funct3_vvxi;
   assign vnsrl_1a        = vex_shft_1a & (funct7_1a[6:1] == 6'b10_1100) & funct3_vvxi;

   assign vssrl_1a        = vex_shft_1a & (funct7_1a[6:1] == 6'b10_1010) & funct3_vvxi;
   assign vssra_1a        = vex_shft_1a & (funct7_1a[6:1] == 6'b10_1011) & funct3_vvxi;

   

   assign o_vnclipu_1a       = vex_shft_1a & (funct7_1a[6:1] == 6'b10_1110) & funct3_vvxi;
   assign o_vnclip_1a        = vex_shft_1a & (funct7_1a[6:1] == 6'b10_1111) & funct3_vvxi;
   
   assign shft_nrw_1a     = vnsra_1a | vnsrl_1a | o_vnclipu_1a | o_vnclip_1a;
   logic inst_sign_ext_1a; assign inst_sign_ext_1a = vsra_1a  | vnsra_1a | vssra_1a | o_vnclip_1a; //MM Nov 5 2021: Remove implicit wire.

   //for narrow ops, in the seocnd cycle the shift amount comes from upper half.
   always_comb begin
      for(int i=0;      i<VLEN/16;i++)  shft_amt_1a[i] = (shft_nrw_1a & i_iterate_cnt_1a[0]) ? final_src1_1a[VLEN/2+i*8+:$clog2(XLEN)] & ({'0, {$clog2(XLEN)+1{1'b1}}} >> ( {2'b0,~shft_nrw_1a} + (3'h3 - vsew_1a[1:0])))
                                                                                             : final_src1_1a[       i*8+:$clog2(XLEN)] & ({'0, {$clog2(XLEN)+1{1'b1}}} >> ( {2'b0,~shft_nrw_1a} + (3'h3 - vsew_1a[1:0])));

      for(int i=VLEN/16;i<VLEN/8; i++)  shft_amt_1a[i] =                                       final_src1_1a[       i*8+:$clog2(XLEN)] & ({'0, {$clog2(XLEN)+1{1'b1}}} >> ( {2'b0,~shft_nrw_1a} + (3'h3 - vsew_1a[1:0])));
   end
      
   always_comb
     for(int i=0;i<VLEN/8;i++)
      // case(vsew_1a[1:0] | {2{(funct3_1a[2:0] == `OPMVV)}})
       case(sew_nrw_1a[1:0])
   	 2'b00:   sized_shft_amt_1a[i] =                                                            shft_amt_1a[ i                            ];
   	 2'b01:   sized_shft_amt_1a[i] = shft_nrw_1a ? shft_amt_1a[ i[$clog2(VLEN/8)-1:1]       ] : shft_amt_1a[{i[$clog2(VLEN/8)-1:1],  1'b0}];
   	 2'b10:   sized_shft_amt_1a[i] = shft_nrw_1a ? shft_amt_1a[{i[$clog2(VLEN/8)-1:2], 1'b0}] : shft_amt_1a[{i[$clog2(VLEN/8)-1:2], 2'b00}];
	 2'b11:   sized_shft_amt_1a[i] = shft_nrw_1a ? shft_amt_1a[{i[$clog2(VLEN/8)-1:3],2'b00}] : shft_amt_1a[{i[$clog2(VLEN/8)-1:3],3'b000}];
       endcase

   always_comb for(int i=0;i<VLEN/8;i++) sel_rs_ext_1a [i]      =  (sew_nrw_1a[1:0] == 2'b00)             |
                                                                  ((sew_nrw_1a[1:0] == 2'b01) &&  i[0]  ) |
                                                                  ((sew_nrw_1a[1:0] == 2'b10) && &i[1:0]) |
                                                                  ((sew_nrw_1a[1:0] == 2'b11) && &i[2:0]);

   always_comb for(int i=0;i<VLEN/8;i++) sel_ls_zero_1a[i]      = (vsew_1a[1:0] == 2'b00) ? 1'b1            :
                                                                  (vsew_1a[1:0] == 2'b01) ? i[0]   == 1'b0  :
                                                                  (vsew_1a[1:0] == 2'b10) ? i[1:0] == 2'b00 :
                                                                                            i[2:0] == 3'b000;
   
   always_comb for(int i=0;i<VLEN/8;i++) src2_ls_1a    [i]      =  sel_ls_zero_1a [i] ? {final_src2_1a[i*8 +:8], 8'h0} 
			                                                              : {final_src2_1a[i*8 +:8],final_src2_1a[(i-1)*8 +:8]}; 
   
   always_comb for(int i=0;i<VLEN/8;i++) src2_rs_1a    [i] =  sel_rs_ext_1a[i] ? {{8{inst_sign_ext_1a & final_src2_1a[i*8+7]}},final_src2_1a[i*8+:8]}
			                                                         : {final_src2_1a[(i+1)*8+:8],final_src2_1a[i*8+:8]};
   

   always_comb
     for(int i=0;i<VLEN/8;i++) begin
	 src2_ls8_1a[i*8+:8]                               =  (src2_ls_1a[i]        << sized_shft_amt_1a[i][2:0]) >> 8; //spyglass disable STARC05-2.10.3.2b_sa
	{src2_rs8_1a[i*8+:8],o_src2_rs8_dm1to0_1a[i][7:0]} = ({src2_rs_1a[i],8'd0}  >> sized_shft_amt_1a[i][2:0]);      //spyglass disable STARC05-2.10.3.2b_sa
     end
   assign src2_rs8_1a[VLEN+:8] = '0;
   //assign src2_ls8_1a[-1:-8] = 'x;
   always_comb
     for(int i=0;i<VLEN/16;i++) begin
	src2_ls16_1a[i*16+:16] = ((vsew_1a[1:0]==2'b10 && i[0]        ) ||
                                  (vsew_1a[1:0]==2'b11 && i[1:0]!=2'h0)   ) ? sized_shft_amt_1a[i*2][3] ? {src2_ls8_1a[i*16+:8],src2_ls8_1a[(i-1)*16+8+:8]} :  src2_ls8_1a[i*16+:16]
	                                                                    : sized_shft_amt_1a[i*2][3] ? {src2_ls8_1a[i*16+:8],8'h0}                       :  src2_ls8_1a[i*16+:16];
                          
	src2_rs16_1a[i*16+:16] = ((sew_nrw_1a[1:0]==2'b01                 ) ||
                                  (sew_nrw_1a[1:0]==2'b10 && i[0]         ) ||
                                  (sew_nrw_1a[1:0]==2'b11 && i[1:0]==2'b11)   ) ? sized_shft_amt_1a[i*2][3] ? {{8{inst_sign_ext_1a & src2_rs8_1a [i*16+15 ]}},src2_rs8_1a[i*16+8+:8]} : src2_rs8_1a[i*16+:16]
	                                                                        : sized_shft_amt_1a[i*2][3] ?                                                 src2_rs8_1a[i*16+8+:16] : src2_rs8_1a[i*16+:16];
	
	o_src2_rs16_dm1to0_1a[i][15:0] = sized_shft_amt_1a[i*2][3]    ? {src2_rs8_1a[i*16+0+:8],o_src2_rs8_dm1to0_1a[i*2+1][7:0]}
					                              : {o_src2_rs8_dm1to0_1a[i*2][7:0],8'd0};
     end
   assign src2_rs16_1a[VLEN+:16] = '0;
   always_comb
     for(int i=0;i<VLEN/32;i++) begin
	src2_ls32_1a[i*32+:32] = (vsew_1a[1:0]!=2'b10 & i[0])         ? sized_shft_amt_1a[i*4][4] ? {src2_ls16_1a[i*32+:16],src2_ls16_1a[(i-1)*32+16+:16]} : src2_ls16_1a[i*32+:32]
                                                                      : sized_shft_amt_1a[i*4][4] ? {src2_ls16_1a[i*32+:16],16'h0}                         : src2_ls16_1a[i*32+:32];
	src2_rs32_1a[i*32+:32] = ((sew_nrw_1a[1:0]==2'b10        ) ||
                                  (sew_nrw_1a[1:0]==2'b11 && i[0])   ) ? sized_shft_amt_1a[i*4][4] ? {{16{inst_sign_ext_1a & src2_rs16_1a [i*32+31 ]}},src2_rs16_1a[i*32+16+:16]} : src2_rs16_1a[i*32+:32]
	                                                               : sized_shft_amt_1a[i*4][4] ?                                                   src2_rs16_1a[i*32+16+:32]  : src2_rs16_1a[i*32+:32];
	o_src2_rs32_dm1to0_1a[i][31:0] = sized_shft_amt_1a[i*4][4]    ? {src2_rs16_1a[i*32+0+:16],o_src2_rs16_dm1to0_1a[i*2+1][15:0]}
                                                                      : {o_src2_rs16_dm1to0_1a[i*2][15:0],16'd0};
     end
   assign src2_rs32_1a[VLEN+:32] = '0;
   
   //narrow needs shifting by another 32.
   always_comb
     for(int i=0;i<VLEN/64;i++) begin
	src2_ls64_1a[i*64+:64] =                                        sized_shft_amt_1a[i*8][5] ? {src2_ls32_1a[i*64+:32],32'h0}                    : src2_ls32_1a[i*64+:64];
	src2_rs64_1a[i*64+:64] = (sew_nrw_1a[1:0]==2'b11 | i[0])      ? sized_shft_amt_1a[i*8][5] ? {{32{inst_sign_ext_1a & src2_rs32_1a [i*64+63 ]}},src2_rs32_1a[i*64+32+:32]} : src2_rs32_1a[i*64+:64]
                                                                      : sized_shft_amt_1a[i*8][5] ?                                                   src2_rs32_1a[i*64+32+:64]  : src2_rs32_1a[i*64+:64];
	
	o_src2_rs64_dm1to0_1a[i][63:0] = sized_shft_amt_1a[i*8][5]    ? {src2_rs32_1a[i*64+0+:32],o_src2_rs32_dm1to0_1a[i*2+1][31:0]}
                                                                      : {o_src2_rs32_dm1to0_1a[i*2][31:0],32'd0}; 
     end
   assign src2_rs64_1a[VLEN+:64] = '0;
   
   always_comb begin
      vex_shft_data_1a = 'x;
      case(1'b1)
	vsll_1a             : vex_shft_data_1a = src2_ls64_1a; 
	vsrl_1a | vsra_1a | vssrl_1a | vssra_1a 
                            : vex_shft_data_1a = src2_rs64_1a; 
       
	vnsra_1a | vnsrl_1a | o_vnclipu_1a | o_vnclip_1a:
   	 
   	  case(vsew_1a[1:0])
   	    2'b00:   for(int i=0;i<VLEN/16; i++) vex_shft_data_1a[i*8+:8]   =  src2_rs64_1a[i*16+:8];  
   	    2'b01:   for(int i=0;i<VLEN/32; i++) vex_shft_data_1a[i*16+:16] =  src2_rs64_1a[i*32+:16];  
   	    2'b10:   for(int i=0;i<VLEN/64; i++) vex_shft_data_1a[i*32+:32] =  src2_rs64_1a[i*64+:32];//src2_rs32_1a[i*64+:32];  
   	    2'b11:   for(int i=0;i<VLEN/128;i++) vex_shft_data_1a[i*64+:64] =  src2_rs64_1a[i*128+:64];
   	  endcase
	default:                          vex_shft_data_1a           = 'x;
      endcase
   end

   always_comb begin
      o_sat_vnclip_pos_1a = 'x;
      case(vsew_1a[1:0])
   	     2'b00:   for(int i=0;i<VLEN/16; i++) o_sat_vnclip_pos_1a[i] =  o_vnclipu_1a ? |src2_rs64_1a[i* 16 +8+:8]  : |src2_rs64_1a[i* 16 +7+:8]  & ~final_src2_1a[i* 16+ 15];  
   	     2'b01:   for(int i=0;i<VLEN/32; i++) o_sat_vnclip_pos_1a[i] =  o_vnclipu_1a ? |src2_rs64_1a[i* 32+16+:16] : |src2_rs64_1a[i* 32+15+:16] & ~final_src2_1a[i* 32+ 31];
   	     2'b10:   for(int i=0;i<VLEN/64; i++) o_sat_vnclip_pos_1a[i] =  o_vnclipu_1a ? |src2_rs64_1a[i* 64+32+:32] : |src2_rs64_1a[i* 64+31+:32] & ~final_src2_1a[i* 64+ 63];
   	     2'b11:   for(int i=0;i<VLEN/128;i++) o_sat_vnclip_pos_1a[i] =  o_vnclipu_1a ? |src2_rs64_1a[i*128+64+:64] : |src2_rs64_1a[i*128+63+:64] & ~final_src2_1a[i*128+127];
      endcase
   end
   //-ve saturation happens for signed vnclip only.
   always_comb begin
      o_sat_vnclip_neg_1a = 'x;
      case(vsew_1a[1:0])
   	     2'b00:   for(int i=0;i<VLEN/16; i++) o_sat_vnclip_neg_1a[i] =  o_vnclip_1a & ~(&src2_rs64_1a[i* 16 +7+:9] ) & final_src2_1a[i* 16+ 15];  
   	     2'b01:   for(int i=0;i<VLEN/32; i++) o_sat_vnclip_neg_1a[i] =  o_vnclip_1a & ~(&src2_rs64_1a[i* 32+15+:17]) & final_src2_1a[i* 32+ 31];
   	     2'b10:   for(int i=0;i<VLEN/64; i++) o_sat_vnclip_neg_1a[i] =  o_vnclip_1a & ~(&src2_rs64_1a[i* 64+31+:33]) & final_src2_1a[i* 64+ 63];
   	     2'b11:   for(int i=0;i<VLEN/128;i++) o_sat_vnclip_neg_1a[i] =  o_vnclip_1a & ~(&src2_rs64_1a[i*128+63+:65]) & final_src2_1a[i*128+127];
      endcase
   end

   assign vmseq_1a  = (funct7_1a[6:1] == 6'b011_000) & funct3_vvxi;
   assign vmsne_1a  = (funct7_1a[6:1] == 6'b011_001) & funct3_vvxi;
   assign vmsltu_1a = (funct7_1a[6:1] == 6'b011_010) & funct3_vvxi;
   assign vmslt_1a  = (funct7_1a[6:1] == 6'b011_011) & funct3_vvx;
   assign vmsleu_1a = (funct7_1a[6:1] == 6'b011_100) & funct3_vvxi;
   assign vmsle_1a  = (funct7_1a[6:1] == 6'b011_101) & funct3_vvxi;
   assign vmsgtu_1a = (funct7_1a[6:1] == 6'b011_110) & funct3_vxi;
   assign vmsgt_1a  = (funct7_1a[6:1] == 6'b011_111) & funct3_vxi;
   
   
  
   assign vand_1a = (funct7_1a[6:1] == 6'b001_001) & funct3_vvxi;
   assign vor_1a  = (funct7_1a[6:1] == 6'b001_010) & funct3_vvxi;
   assign vxor_1a = (funct7_1a[6:1] == 6'b001_011) & funct3_vvxi;
   
   assign vredand_1a = (funct7_1a[6:1] == 6'b000_001) & (funct3_1a[2:0] == `OPMVV);
   assign vredor_1a  = (funct7_1a[6:1] == 6'b000_010) & (funct3_1a[2:0] == `OPMVV);
   assign vredxor_1a = (funct7_1a[6:1] == 6'b000_011) & (funct3_1a[2:0] == `OPMVV);
   
   logic  [VLEN/8-1:0] mask_bitwise_1a,mask_bitwise_src1_1a,mask_bitwise_src2_1a,mask_bitwise_preshift_1a;
   always_comb begin
      case(vsew_1a[1:0])
	2'b00: begin mask_bitwise_src1_1a  = final_src1_1a[lmul_cnt_1a[2:0]*VLEN/8+:VLEN/8]; mask_bitwise_src2_1a = final_src2_1a[lmul_cnt_1a[2:0]*VLEN/8+:VLEN/8]; end
	2'b01: begin mask_bitwise_src1_1a  = final_src1_1a[lmul_cnt_1a[2:1]*VLEN/8+:VLEN/8]; mask_bitwise_src2_1a = final_src2_1a[lmul_cnt_1a[2:1]*VLEN/8+:VLEN/8]; end
	2'b10: begin mask_bitwise_src1_1a  = final_src1_1a[lmul_cnt_1a[2]  *VLEN/8+:VLEN/8]; mask_bitwise_src2_1a = final_src2_1a[lmul_cnt_1a[2]  *VLEN/8+:VLEN/8]; end
	2'b11: begin mask_bitwise_src1_1a  = final_src1_1a[                      0+:VLEN/8]; mask_bitwise_src2_1a = final_src2_1a[                      0+:VLEN/8]; end
      endcase
   end
								
   assign mask_bitwise_preshift_1a =  (({VLEN/8{vmandnot_1a}} ^ mask_bitwise_src1_1a) & mask_bitwise_src2_1a) & {VLEN/8{vmand_1a | vmnand_1a | vmandnot_1a}}
                                    | (({VLEN/8{vmornot_1a}}  ^ mask_bitwise_src1_1a) | mask_bitwise_src2_1a) & {VLEN/8{vmor_1a  | vmnor_1a  |  vmornot_1a}}
			            | ((                        mask_bitwise_src1_1a) ^ mask_bitwise_src2_1a) & {VLEN/8{vmxor_1a | vmxnor_1a              }}; 

   always_comb
     case(vsew_1a[1:0])
       2'b00: mask_bitwise_1a = mask_bitwise_preshift_1a;
       2'b01: mask_bitwise_1a = mask_bitwise_preshift_1a >> lmul_cnt_1a[0]  *(VLEN/16);
       2'b10: mask_bitwise_1a = mask_bitwise_preshift_1a >> lmul_cnt_1a[1:0]*(VLEN/32);
       2'b11: mask_bitwise_1a = mask_bitwise_preshift_1a >> lmul_cnt_1a[2:0]*(VLEN/64);
     endcase
       
       
   localparam DATA_WIDTH=8;
   localparam NUM_ELEM = VLEN/DATA_WIDTH;
   localparam MSIZE = $clog2(NUM_ELEM);
   logic [MSIZE:0][NUM_ELEM-1:0] elem_eq_1a,elem_ltu_1a;
   logic [VLEN/8-1:0] elem_lt_1a,elem_lt_unsized_1a; 
   //IMPROVE_AREA: Remove all these compares and use the adder's borrow bit to make a decision for  < or gt, and compare adders output with 0 for equal to
   always_comb begin
      for(int i=0;i<VLEN/8;i++)
	case(vsew_1a[1:0])
	  2'b00:   elem_lt_1a[i] =  (final_src2_1a[i*8+7]       ^ final_src1_1a[i*8+7])       ^ (final_src2_1a[i*8+:8]       < final_src1_1a[i*8+:8]);
	  2'b01:   elem_lt_1a[i] =  (final_src2_1a[(i/2)*16+15] ^ final_src1_1a[(i/2)*16+15]) ^ (final_src2_1a[(i/2)*16+:16] < final_src1_1a[(i/2)*16+:16]);
	  2'b10:   elem_lt_1a[i] =  (final_src2_1a[(i/4)*32+31] ^ final_src1_1a[(i/4)*32+31]) ^ (final_src2_1a[(i/4)*32+:32] < final_src1_1a[(i/4)*32+:32]);
	  2'b11:   elem_lt_1a[i] =  (final_src2_1a[(i/8)*64+63] ^ final_src1_1a[(i/8)*64+63]) ^ (final_src2_1a[(i/8)*64+:64] < final_src1_1a[(i/8)*64+:64]);
	endcase        
   end
   always_comb begin
      for(int i=0; i<NUM_ELEM;i++) begin
	 elem_eq_1a[MSIZE][i]  = final_src2_1a[i*8+:8] == final_src1_1a[i*8+:8];
	 elem_ltu_1a[MSIZE][i] = ~vmerge_1a ? final_src2_1a[i*8+:8]  < final_src1_1a[i*8+:8] : ~cryorbrw_mask_1a[i];
      end
	 
      for(int LVL=MSIZE-1;LVL>=1;LVL--)
	for(int NODE=0;NODE < 1<<LVL;NODE++) begin
	   elem_eq_1a[LVL][NODE]  = elem_eq_1a[LVL+1][2*NODE+1]  & elem_eq_1a[LVL+1][2*NODE+0];
	   elem_ltu_1a[LVL][NODE] = elem_ltu_1a[LVL+1][2*NODE+1] | elem_eq_1a[LVL+1][2*NODE+1] & elem_ltu_1a[LVL+1][2*NODE+0];
	end
   end 
   
   //assign elem_ltu_sized_1a[15:0] = tt_vec_pkg::copy_mask(elem_ltu_1a[MSIZE-vsew_1a[1:0]],vsew_1a[1:0]);
   always_comb
     for(int i=0;i<VLEN/8;i++)
       case(vsew_1a[1:0])
   	 2'b00:  elem_ltu_sized_1a[i] = elem_ltu_1a[MSIZE][i];
   	 2'b01:  elem_ltu_sized_1a[i] = elem_ltu_1a[MSIZE-1][i/2];
   	 2'b10:  elem_ltu_sized_1a[i] = elem_ltu_1a[MSIZE-2][i/4];
   	 2'b11:  elem_ltu_sized_1a[i] = elem_ltu_1a[MSIZE-3][i/8];
   	 default:elem_ltu_sized_1a[i] = 'x;
       endcase

   assign vminu_1a   = (funct7_1a[6:1] == 6'b000_100) & funct3_vvxi | vmerge_1a;
   assign vmin_1a    = (funct7_1a[6:1] == 6'b000_101) & funct3_vvxi;
   assign vmaxu_1a   = (funct7_1a[6:1] == 6'b000_110) & funct3_vvxi;
   assign vmax_1a    = (funct7_1a[6:1] == 6'b000_111) & funct3_vvxi;
   //assign vmerge_1a  = (funct7_1a[6:1] == 6'b010_111) & funct3_vvxi & ~i_v_vm_1a;
   assign vredmax_1a = ((funct7_1a[6:1] == 6'b000_110) |  (funct7_1a[6:1] == 6'b000_111)) & (funct3_1a[2:0] == `OPMVV);
   assign vredmin_1a = ((funct7_1a[6:1] == 6'b000_100) |  (funct7_1a[6:1] == 6'b000_101)) & (funct3_1a[2:0] == `OPMVV);
   
   assign sel_minmax_1a = vminu_1a | vmin_1a | vmaxu_1a | vmax_1a | vmerge_1a | vredmax_1a | vredmin_1a;

   always_comb
     for(int i=0;i<VLEN/8;i++)
	case(vsew_1a[1:0] + i_reduct_wdeop_1a)
	  2'b00:   src1_gt_src2_sized_1a[i] =  unsgn_src1_gt_src2_1a[i]          | sgn_src1_gt_src2_1a[i] 	  ;	   
	  2'b01:   src1_gt_src2_sized_1a[i] =  unsgn_src1_gt_src2_1a[i[3:1]*2+1] | sgn_src1_gt_src2_1a[i[3:1]*2+1];
	  2'b10:   src1_gt_src2_sized_1a[i] =  unsgn_src1_gt_src2_1a[i[3:2]*4+3] | sgn_src1_gt_src2_1a[i[3:2]*4+3];
	  2'b11:   src1_gt_src2_sized_1a[i] =  unsgn_src1_gt_src2_1a[i[3  ]*8+7] | sgn_src1_gt_src2_1a[i[3  ]*8+7];
	  default: src1_gt_src2_sized_1a[i] = 'x;   
	endcase
   always_comb
     for(int i=0;i<VLEN/8;i++)
       case(1'b1)
	 vminu_1a: vex_minmax_data_1a[i*8+:8] = elem_ltu_sized_1a[i] ? final_src2_1a[i*8+:8] : final_src1_1a[i*8+:8];
	 vmin_1a : vex_minmax_data_1a[i*8+:8] = elem_lt_1a[i]  ? final_src2_1a[i*8+:8] : final_src1_1a[i*8+:8];
	 vmaxu_1a: vex_minmax_data_1a[i*8+:8] = elem_ltu_sized_1a[i] ? final_src1_1a[i*8+:8] : final_src2_1a[i*8+:8];
	 vmax_1a : vex_minmax_data_1a[i*8+:8] = elem_lt_1a[i]  ? final_src1_1a[i*8+:8] : final_src2_1a[i*8+:8];
	 vredmax_1a: vex_minmax_data_1a[i*8+:8] = src1_gt_src2_sized_1a[i]  ? ~final_src1_1a[i*8+:8] :  final_src2_1a[i*8+:8];
	 vredmin_1a: vex_minmax_data_1a[i*8+:8] = src1_gt_src2_sized_1a[i]  ?  final_src2_1a[i*8+:8] : ~final_src1_1a[i*8+:8];
	 default:  vex_minmax_data_1a[i*8+:8] = 'x;
       endcase
       
   always_comb
     case(1'b1)
       vand_1a | vredand_1a:  vex_bitwise_data_1a = final_src2_1a[VLEN-1:0] & final_src1_1a;
       vor_1a  | vredor_1a:   vex_bitwise_data_1a = final_src2_1a[VLEN-1:0] | final_src1_1a;
       vxor_1a | vredxor_1a:  vex_bitwise_data_1a = final_src2_1a[VLEN-1:0] ^ final_src1_1a;
       i_mask_only_instrn_1a: 
	 case(vsew_1a[1:0])
	   2'b00:             vex_bitwise_data_1a = {'x,vmask_only_data_1a} << (lmul_cnt_1a[2:0]*VLEN/8); //spyglass disable STARC05-2.10.3.2b_sb
	   2'b01:             vex_bitwise_data_1a = {'x,vmask_only_data_1a} << (lmul_cnt_1a[2:0]*VLEN/16); //spyglass disable STARC05-2.10.3.2b_sb
	   2'b10:             vex_bitwise_data_1a = {'x,vmask_only_data_1a} << (lmul_cnt_1a[2:0]*VLEN/32); //spyglass disable STARC05-2.10.3.2b_sb
	   2'b11:             vex_bitwise_data_1a = {'x,vmask_only_data_1a} << (lmul_cnt_1a[2:0]*VLEN/64); //spyglass disable STARC05-2.10.3.2b_sb
	 endcase
       sel_minmax_1a:         vex_bitwise_data_1a = vex_minmax_data_1a;
       default:               vex_bitwise_data_1a = 'x;
     endcase 

    //elem_lt is already sized, meaning if size=32 [3:0] will have the same value.
    //This is a hack. I want to remove all this logic, and use adders borrow bit
    always_comb begin
       elem_lt_unsized_1a = 'x;
       case(vsew_1a)
	 2'b00: for(int i=0; i<VLEN/8; i++) elem_lt_unsized_1a[i] = elem_lt_1a[i];
	 2'b01: for(int i=0; i<VLEN/16;i++) elem_lt_unsized_1a[i] = elem_lt_1a[i*2];
	 2'b10: for(int i=0; i<VLEN/32;i++) elem_lt_unsized_1a[i] = elem_lt_1a[i*4];
	 2'b11: for(int i=0; i<VLEN/64;i++) elem_lt_unsized_1a[i] = elem_lt_1a[i*8];
       endcase // case (vsew_1a)
    end

   //The compare instructions which generate mask bits , align data to the lower index's example sew=8, 15:0 , sew=16 7:0 sew=32, 3:0
   //But the instructions that operate on mask operate on 16 bits and need to be adjusted based on the lmul_iteration, to align with the outputs of compare instructions
   always_comb 
     case(1'b1)
	vmand_1a | vmor_1a | vmxor_1a | vmandnot_1a | vmornot_1a:
	           vmask_only_data_1a =  mask_bitwise_1a;
	vmnand_1a | vmnor_1a | vmxnor_1a:
	           vmask_only_data_1a = ~mask_bitwise_1a;

	vmseq_1a:  vmask_only_data_1a =   elem_eq_1a  [MSIZE-vsew_1a];
	vmsne_1a:  vmask_only_data_1a =  ~elem_eq_1a  [MSIZE-vsew_1a];
	vmsltu_1a: vmask_only_data_1a =   elem_ltu_1a [MSIZE-vsew_1a];
	vmsleu_1a: vmask_only_data_1a =   elem_ltu_1a [MSIZE-vsew_1a] | elem_eq_1a  [MSIZE-vsew_1a];
	vmsgtu_1a: vmask_only_data_1a = ~(elem_ltu_1a [MSIZE-vsew_1a] | elem_eq_1a  [MSIZE-vsew_1a]);

	
	vmslt_1a:  vmask_only_data_1a =   elem_lt_unsized_1a;
	vmsle_1a:  vmask_only_data_1a =   elem_lt_unsized_1a   | elem_eq_1a  [MSIZE-vsew_1a];
	vmsgt_1a:  vmask_only_data_1a = ~(elem_lt_unsized_1a   | elem_eq_1a  [MSIZE-vsew_1a]);
       default:    vmask_only_data_1a =  'x;
     endcase
   
   ////////////////////End Shft and logical instructions
   localparam MASK_WIDTH = VLEN/8;
   localparam SIZE = $clog2(MASK_WIDTH);
   logic [MASK_WIDTH:0][$clog2(VLEN+1)-1:0] pop_sum_1a;
   logic [$clog2(VLEN/8)-1:0]          ffs_ptr_1a;
   logic 			       ffs_or_1a;
   logic [VLEN/8-1:0] 		       ffs_onehot_1a,inp_popc_1a,bit_mask_sized_1a;
    
   logic [VLEN/8-1:0] mask_bit_sel_mask_1a;
   always_comb begin
      casez ({lmul_1a[2:0], vsew_1a[1:0]})
         5'b0??_00: mask_bit_sel_mask_1a =      {VLEN/8 {1'b1}};
         5'b0??_01: mask_bit_sel_mask_1a = {'0, {VLEN/16{1'b1}}};
         5'b0??_10: mask_bit_sel_mask_1a = {'0, {VLEN/32{1'b1}}};
         5'b0??_11: mask_bit_sel_mask_1a = {'0, {VLEN/64{1'b1}}};
         // Fractional
         // 1/2
         5'b111_00: mask_bit_sel_mask_1a = {'0, {VLEN/16 {1'b1}}};
         5'b111_01: mask_bit_sel_mask_1a = {'0, {VLEN/32 {1'b1}}};
         5'b111_10: mask_bit_sel_mask_1a = {'0, {VLEN/64 {1'b1}}};
         5'b111_11: mask_bit_sel_mask_1a = {'0, {VLEN/128{1'b1}}};
         // 1/4
         5'b110_00: mask_bit_sel_mask_1a = {'0, {VLEN/32 {1'b1}}};
         5'b110_01: mask_bit_sel_mask_1a = {'0, {VLEN/64 {1'b1}}};
         5'b110_10: mask_bit_sel_mask_1a = {'0, {VLEN/128{1'b1}}};
         5'b110_11: mask_bit_sel_mask_1a = {'0, {VLEN/256{1'b1}}};
         // 1/8
         5'b101_00: mask_bit_sel_mask_1a = {'0, {VLEN/64 {1'b1}}};
         5'b101_01: mask_bit_sel_mask_1a = {'0, {VLEN/128{1'b1}}};
         5'b101_10: mask_bit_sel_mask_1a = {'0, {VLEN/256{1'b1}}};
         5'b101_11: mask_bit_sel_mask_1a = {'0, {VLEN/512{1'b1}}};
         default:   mask_bit_sel_mask_1a = '0;
      endcase
   end

 
   assign bit_mask_sized_1a   = cryorbrw_mask_1a & mask_bit_sel_mask_1a;
       
   assign inp_popc_1a         = {MASK_WIDTH{vid_1a}} | ((final_src2_1a[VLEN-1:0]  >> ((VLEN/8 >> vsew_1a[1:0]) * lmul_cnt_1a[2:0])) & bit_mask_sized_1a); //spyglass disable STARC05-2.10.3.2b_sa

   tt_ffs    #(.WIDTH(MASK_WIDTH),.DIR_L2H(1),.DATA_WIDTH(1)) i_ffs(.data_in({MASK_WIDTH{1'b0}}), .req_in(inp_popc_1a),.enc_req_out(ffs_ptr_1a),.req_sum(ffs_or_1a),.req_out(ffs_onehot_1a), .data_out()); //spyglass disable UndrivenInTerm-ML
   
      
   always_comb begin
      pop_sum_1a    = '0;
      pop_sum_1a[0] = last_lmul_iter_result_1a; //Width is sufficient , max is VLEN elements.
      for(int i=1;i<=MASK_WIDTH;i++) begin
         pop_sum_1a[i] = last_lmul_iter_result_1a; //Width is sufficient , max is VLEN elements.
         for(int j=0;j<i;j++) begin
            pop_sum_1a[i] += inp_popc_1a[j];
         end
      end
   end 
   
   
   wire [VLEN/8-1:0] sized_sew_mask = {VLEN/8{~ffs_or_1a}} >> (vsew_1a == 2'b00 ? (VLEN/8 - VLEN/8 ) :
                                                               vsew_1a == 2'b01 ? (VLEN/8 - VLEN/16) :
                                                               vsew_1a == 2'b10 ? (VLEN/8 - VLEN/32) :
                                                                                  (VLEN/8 - VLEN/64)  );
   wire [VLEN/4-1:0] fill_ones_below_ptr = {'0,{VLEN/8{1'b1}}} << ffs_ptr_1a;
   assign vpopc_1a  = (funct7_1a[6:1] == 6'b010_000) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b10000);
   assign vffs_1a   = (funct7_1a[6:1] == 6'b010_000) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b10001);
   assign vmsif_1a  = (funct7_1a[6:1] == 6'b010_100) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b00011);
   assign vmsbf_1a  = (funct7_1a[6:1] == 6'b010_100) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b00001);
   assign vmsof_1a  = (funct7_1a[6:1] == 6'b010_100) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b00010);
   assign viota_1a  = (funct7_1a[6:1] == 6'b010_100) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b10000);		     
   assign vid_1a    = (funct7_1a[6:1] == 6'b010_100) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b10001);
 
   assign vmand_1a    = (funct7_1a[6:1] == 6'b011_001) & (funct3_1a[2:0] == `OPMVV);
   assign vmnand_1a   = (funct7_1a[6:1] == 6'b011_101) & (funct3_1a[2:0] == `OPMVV);
   assign vmandnot_1a = (funct7_1a[6:1] == 6'b011_000) & (funct3_1a[2:0] == `OPMVV);
   assign vmxor_1a    = (funct7_1a[6:1] == 6'b011_011) & (funct3_1a[2:0] == `OPMVV);
   assign vmor_1a     = (funct7_1a[6:1] == 6'b011_010) & (funct3_1a[2:0] == `OPMVV);
   assign vmnor_1a    = (funct7_1a[6:1] == 6'b011_110) & (funct3_1a[2:0] == `OPMVV);
   assign vmornot_1a  = (funct7_1a[6:1] == 6'b011_100) & (funct3_1a[2:0] == `OPMVV);
   assign vmxnor_1a   = (funct7_1a[6:1] == 6'b011_111) & (funct3_1a[2:0] == `OPMVV);

     
   //recirculate last iterations result, note these should reflect what should be the next index or pop_cnt.
   //forexample , vid,size=8, the last index of the first iteration would be 'f, therefore the first index of the next iteration should be 'hf+1 = 'h10
   always_comb
      case(vsew_1a[1:0])
	2'b00: last_lmul_iter_result_0a   =  pop_sum_1a[VLEN/8 ];
	2'b01: last_lmul_iter_result_0a   =  pop_sum_1a[VLEN/16];
	2'b10: last_lmul_iter_result_0a   =  pop_sum_1a[VLEN/32];
	2'b11: last_lmul_iter_result_0a   =  pop_sum_1a[VLEN/64];
      endcase 
   assign ffs_last_lmul_iter_0a = ffs_or_1a & vex_maskbit_1a;
   assign last_ffs_ptr_0a = {'0,ffs_ptr_1a} +  (lmul_cnt_1a[2:0] * (VLEN/8 >> vsew_1a[1:0])); //spyglass disable STARC05-2.10.3.2b_sa
   
   always_comb begin
     vmsk_data_1a = '0;
     case(1'b1)
       vpopc_1a: vmsk_data_1a[63:0]   = {'0,pop_sum_1a[VLEN/8]};
       vffs_1a:  vmsk_data_1a[63:0]   = {64{~|inp_popc_1a & ~ffs_last_lmul_iter_1a}}           | (ffs_last_lmul_iter_1a ? {'0,last_ffs_ptr_1a} : {'0,last_ffs_ptr_0a});
       
       vmsbf_1a: vmsk_data_1a         = (~{VLEN/8{ffs_last_lmul_iter_1a}}  & (sized_sew_mask | fill_ones_below_ptr[VLEN/4-1:VLEN/8]))                  << (lmul_cnt_1a[2:0] * (VLEN/8 >> vsew_1a[1:0])); //spyglass disable STARC05-2.10.3.2b_sb
       vmsif_1a: vmsk_data_1a         = (~{VLEN/8{ffs_last_lmul_iter_1a}}  & (sized_sew_mask | fill_ones_below_ptr[VLEN/4-1:VLEN/8]  | ffs_onehot_1a)) << (lmul_cnt_1a[2:0] * (VLEN/8 >> vsew_1a[1:0])); //spyglass disable STARC05-2.10.3.2b_sb
       vmsof_1a: vmsk_data_1a         = (~{VLEN/8{ffs_last_lmul_iter_1a}}  & (ffs_onehot_1a))                                                          << (lmul_cnt_1a[2:0] * (VLEN/8 >> vsew_1a[1:0])); //spyglass disable STARC05-2.10.3.2b_sb
       viota_1a | vid_1a:  
	 case(vsew_1a)
	   2'b00: for(int i=0; i<VLEN/8;  i++) vmsk_data_1a[i*8+:8]    = {'0, pop_sum_1a[i]};
	   2'b01: for(int i=0; i<VLEN/16; i++) vmsk_data_1a[i*16+:16]  = {'0, pop_sum_1a[i]};
	   2'b10: for(int i=0; i<VLEN/32; i++) vmsk_data_1a[i*32+:32]  = {'0, pop_sum_1a[i]};
	   2'b11: for(int i=0; i<VLEN/64; i++) vmsk_data_1a[i*64+:64]  = {'0, pop_sum_1a[i]};
	 endcase

       default:  vmsk_data_1a           = 'x;
     endcase
   end
   //////////////////////Permute Instructions////////////////////////
   logic vmv_x_s_1a,vmv_s_x_1a;
   logic vslideup_1a,vslidedwn_1a,vslide1up_1a,vslide1dwn_1a,vslideop_1a;
   logic vrgather_1a,vrgatherei16_1a;
   logic vzextby2_1a,vzextby4_1a,vzextby8_1a,vsextby2_1a,vsextby4_1a,vsextby8_1a,vext_1a,vextby2_1a,vextby4_1a,vsext_1a,vcompress_1a;
   logic [$clog2(VLEN/8)-1:0] sel_iterate_ele_1a;
   logic [XLEN-1:0] slide_shft_amt_1a; 
       
   assign vmv_x_s_1a     = (funct7_1a[6:1] == 6'b01_0000) & ((funct3_1a[2:0] == `OPMVV)/* | (funct3_1a[2:0] == `OPFVV)*/) & (i_vex_instrn_1a[19:15] == 5'b00000); 
   assign vmv_s_x_1a     = (funct7_1a[6:1] == 6'b01_0000) & ((funct3_1a[2:0] == `OPMVX) | (funct3_1a[2:0] == `OPFVF)) & (i_vex_instrn_1a[24:20] == 5'b00000);
   assign vslideup_1a    = (funct7_1a[6:1] == 6'b00_1110) & ((funct3_1a[2:0] == `OPIVX) | (funct3_1a[2:0] == `OPIVI));
   assign vslidedwn_1a   = (funct7_1a[6:1] == 6'b00_1111) & ((funct3_1a[2:0] == `OPIVX) | (funct3_1a[2:0] == `OPIVI));
   assign vslide1up_1a   = (funct7_1a[6:1] == 6'b00_1110) & ((funct3_1a[2:0] == `OPMVX) | (funct3_1a[2:0] == `OPFVF));
   assign vslide1dwn_1a  = (funct7_1a[6:1] == 6'b00_1111) & ((funct3_1a[2:0] == `OPMVX) | (funct3_1a[2:0] == `OPFVF));
   assign vslideop_1a    = vslideup_1a | vslidedwn_1a | vslide1dwn_1a | vslide1up_1a;
   assign vrgather_1a     = (funct7_1a[6:1] == 6'b00_1100) & ((funct3_1a[2:0] == `OPIVX) | (funct3_1a[2:0] == `OPIVI)  | (funct3_1a[2:0] == `OPIVV) );
   assign vrgatherei16_1a = (funct7_1a[6:1] == 6'b00_1110) & ((funct3_1a[2:0] == `OPIVV) );
   logic vrgatherall_1a; assign vrgatherall_1a = vrgather_1a | vrgatherei16_1a; //MM Nov 5 2021: Remove implicit wire.

   assign vzextby2_1a        = (funct7_1a[6:1] == 6'b01_0010) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b00110);
   assign vzextby4_1a        = (funct7_1a[6:1] == 6'b01_0010) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b00100);
   assign vzextby8_1a        = (funct7_1a[6:1] == 6'b01_0010) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b00010);
   assign vsextby2_1a        = (funct7_1a[6:1] == 6'b01_0010) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b00111);
   assign vsextby4_1a        = (funct7_1a[6:1] == 6'b01_0010) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b00101);
   assign vsextby8_1a        = (funct7_1a[6:1] == 6'b01_0010) & (funct3_1a[2:0] == `OPMVV) & (i_vex_instrn_1a[19:15] == 5'b00011);
   assign vext_1a            = vzextby2_1a | vzextby4_1a | vsextby2_1a | vsextby4_1a;
   assign vextby2_1a         = vzextby2_1a | vsextby2_1a;
   assign vextby4_1a         = vzextby4_1a | vsextby4_1a;
   assign vsext_1a           = vsextby2_1a | vsextby4_1a;
   assign vcompress_1a       = (funct7_1a[6:1] == 6'b01_0111) & (funct3_1a[2:0] == `OPMVV);
   //for slide1up shift amt is marked as 0, so we don't use old from Vd.
   assign o_slide_shft_amt_1a[XLEN-1:0] =  slide_shft_amt_1a;
   assign o_slide_dwn_1a           =  vslidedwn_1a | vslide1dwn_1a;
   //assign o_slide1_up_1a           =  vslide1up_1a;
   //assign o_slide1_dwn_1a          =  vslide1dwn_1a;
   assign slide_shft_amt_1a  = (vslide1up_1a | vslide1dwn_1a) ? {'0,1'b1} : i_scalar_imm_slide_1a; //A maximum of 128 elements are possible

   //For Vslide the iterate_cnt picks the element in VS2.
   //For Vgather the lower bits of vs1[iterate_cnt] picks the element in VS2, while the higher bits pick up the VS2.
   wire [1:0] sew_gather_1a  = vrgatherei16_1a ? 2'b01 : vsew_1a;  	
   always_comb begin
      sel_iterate_ele_1a = '0;
      case(1'b1)
       vslideop_1a:
	case(vsew_1a[1:0])
	  2'b00  :	sel_iterate_ele_1a[$clog2(VLEN/8)-1:0] = (o_slide_dwn_1a ? i_iterate_cnt_1a[$clog2(VLEN/8)-1:0] + slide_shft_amt_1a[$clog2(VLEN/8)-1:0] : i_iterate_cnt_1a[$clog2(VLEN/8)-1:0] - slide_shft_amt_1a[$clog2(VLEN/8)-1:0]);
	  2'b01  :	sel_iterate_ele_1a[$clog2(VLEN/8)-2:0] = (o_slide_dwn_1a ? i_iterate_cnt_1a[$clog2(VLEN/8)-2:0] + slide_shft_amt_1a[$clog2(VLEN/8)-2:0] : i_iterate_cnt_1a[$clog2(VLEN/8)-2:0] - slide_shft_amt_1a[$clog2(VLEN/8)-2:0]);
	  2'b10  :	sel_iterate_ele_1a[$clog2(VLEN/8)-3:0] = (o_slide_dwn_1a ? i_iterate_cnt_1a[$clog2(VLEN/8)-3:0] + slide_shft_amt_1a[$clog2(VLEN/8)-3:0] : i_iterate_cnt_1a[$clog2(VLEN/8)-3:0] - slide_shft_amt_1a[$clog2(VLEN/8)-3:0]);
	  2'b11  :	sel_iterate_ele_1a[$clog2(VLEN/8)-4:0] = (o_slide_dwn_1a ? i_iterate_cnt_1a[$clog2(VLEN/8)-4:0] + slide_shft_amt_1a[$clog2(VLEN/8)-4:0] : i_iterate_cnt_1a[$clog2(VLEN/8)-4:0] - slide_shft_amt_1a[$clog2(VLEN/8)-4:0]);
	endcase
	vrgatherall_1a:	sel_iterate_ele_1a = i_vs1_gather_indx_1a;
	vext_1a | vcompress_1a:
                 	sel_iterate_ele_1a = i_iterate_cnt_1a[$clog2(VLEN/8)-1:0]; //the source changes at a lower frequency than the destination.
	  
	default:	sel_iterate_ele_1a = 'x;
      endcase
   end

  
   //IMPROVE_Power: for iterate ops since only either 8,16,32 bits are generated in a cycle. THere is oppurunity to save power by not clocking in all 128 bits of idata_2a...
   always_comb begin
     vex_per_data_1a = 'x;
     case(1'b1)
       vmv_x_s_1a: 
	 begin
	    case(vsew_1a[1:0])
	      2'b00: vex_per_data_1a[63:0]   = {{58{final_src2_1a[7]}} ,final_src2_1a[ 7:0]};
	      2'b01: vex_per_data_1a[63:0]   = {{48{final_src2_1a[15]}},final_src2_1a[15:0]};
	      2'b10: vex_per_data_1a[63:0]   = {{32{final_src2_1a[31]}},final_src2_1a[31:0]};
	      2'b11: vex_per_data_1a[63:0]   = {                        final_src2_1a[63:0]};
	    endcase
	 end
       
       
       vmv_s_x_1a: vex_per_data_1a   = {'0,final_src1_1a[63:0]};
       vslideop_1a:
	 begin
	    case(vsew_1a[1:0])
	      2'b00:   vex_per_data_1a[7:0]  =  (  ~|i_iterate_cnt_1a & ~|lmul_cnt_1a & vslide1up_1a 
						 | ( ($clog2(VLEN+1)'(i_iterate_cnt_1a+1) == i_vl_cnt_1a)) & vslide1dwn_1a) ? i_scalar_imm_slide_1a[7 :0] : final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-1:0]* 8+: 8];
	      2'b01:   vex_per_data_1a[15:0] =  (  ~|i_iterate_cnt_1a & ~|lmul_cnt_1a & vslide1up_1a 
						 | ( ($clog2(VLEN+1)'(i_iterate_cnt_1a+1) == i_vl_cnt_1a)) & vslide1dwn_1a) ? i_scalar_imm_slide_1a[15:0] : final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-2:0]*16+:16];
	      2'b10:   vex_per_data_1a[31:0] =  (  ~|i_iterate_cnt_1a & ~|lmul_cnt_1a & vslide1up_1a 
                                                 | ( ($clog2(VLEN+1)'(i_iterate_cnt_1a+1) == i_vl_cnt_1a)) & vslide1dwn_1a) ? i_scalar_imm_slide_1a[31:0] : final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-3:0]*32+:32];
	      2'b11:   vex_per_data_1a[63:0] =  (  ~|i_iterate_cnt_1a & ~|lmul_cnt_1a & vslide1up_1a 
                                                 | ( ($clog2(VLEN+1)'(i_iterate_cnt_1a+1) == i_vl_cnt_1a)) & vslide1dwn_1a) ? i_scalar_imm_slide_1a[63:0] : final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-4:0]*64+:64];
	    endcase
	 end
       vrgatherall_1a | vcompress_1a:
	 case(vsew_1a[1:0])
	      2'b00:   vex_per_data_1a[7:0]  = final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-1:0]* 8+:8];
	      2'b01:   vex_per_data_1a[15:0] = final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-2:0]*16+:16];
	      2'b10:   vex_per_data_1a[31:0] = final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-3:0]*32+:32];
	      2'b11:   vex_per_data_1a[63:0] = final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-4:0]*64+:64];
	 endcase

       vext_1a:
	 case(vsew_1a[1:0])
	      2'b00:   vex_per_data_1a[7:0]  = 'x; //not possible
	      2'b01:   vex_per_data_1a[15:0] =              {{8 {vsext_1a & final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-1:0]*8  +7]}},final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-1:0]*8  +:8]};
					                  
	      2'b10:   vex_per_data_1a[31:0] = vextby2_1a ? {{16{vsext_1a & final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-2:0]*16+15]}},final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-2:0]*16+:16]} 
					                  : {{24{vsext_1a & final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-1:0]*8  +7]}},final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-1:0]*8  +:8]};
	      2'b11:   vex_per_data_1a[63:0] = vextby2_1a ? {{32{vsext_1a & final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-3:0]*32+31]}},final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-3:0]*32+:32]} :
                                               vextby4_1a ? {{48{vsext_1a & final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-2:0]*16+15]}},final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-2:0]*16+:16]} 
					                  : {{56{vsext_1a & final_src2_1a[sel_iterate_ele_1a[$clog2(VLEN/8)-1:0]*8  +7]}},final_src2_1a[ sel_iterate_ele_1a[$clog2(VLEN/8)-1:0]*8  +:8]};
	 endcase
        default: vex_per_data_1a =  'x;	
     endcase 
   end
   /////////////////////////
   always_ff @(posedge i_clk)  //not all of these may need resetting, but....I want this to be done...
     if(~i_reset_n) begin
	o_avg_1a                        <= '0;
	adden_1a			<= '0;
	wrmask_1a			<= '0;
	usemask_1a			<= '0;
	vsew_1a[1:0]			<= '0;
	ixv_tov_mv_1a			<= '0;
	vmvgrp_1a                       <= '0;
	o_v_tox_mv_1a			<= '0;
	vex_per_1a			<= '0;
	vex_shft_1a			<= '0;
	vex_bitwise_1a			<= '0;
	vex_maskbit_1a			<= '0;
	lmul_cnt_1a[2:0]		<= '0;
	sat_instrn_1a                   <= '0;
	wdeop_1a                        <= '0;
	reductop_1a                     <= '0;
	vmerge_1a                       <= '0;
     end else if(i_vex_en_0a  | i_vex_en_1a) begin 
	o_avg_1a                        <= i_avg_0a;
	adden_1a			<= i_adden_0a;
	wrmask_1a			<= i_wrmask_0a;
	usemask_1a			<= i_usemask_0a;
	vsew_1a[1:0]			<= i_vsew_0a[1:0];
	ixv_tov_mv_1a			<= i_ixv_tov_mv_0a & i_v_vm_0a;
	vmvgrp_1a                       <= i_vmvgrp_0a;
	o_v_tox_mv_1a			<= i_v_tox_mv_0a;
	vex_per_1a			<= i_vex_per_0a;
	vex_shft_1a			<= i_vex_shft_0a;
	vex_bitwise_1a			<= i_vex_bitwise_0a & ~(i_ixv_tov_mv_0a & i_v_vm_0a);
	vex_maskbit_1a			<= i_vex_vmaskbit_0a;
	lmul_cnt_1a[2:0]		<= i_lmul_cnt_0a[2:0];
	sat_instrn_1a                   <= i_sat_instrn_0a;
	wdeop_1a                        <= i_wdeop_0a;
	reductop_1a                     <= i_reductop_0a;
	vmerge_1a                       <= i_vmerge_0a;
     end
   
   always_ff@(posedge i_clk)       
      if(i_wrmask_0a | i_sat_instrn_0a | i_avg_0a | i_adden_0a)
	sgn_bits_differ_1a        <= sgn_bits_differ_0a;
   
   always_ff@(posedge i_clk) begin
      if(i_vex_vmaskbit_0a & ~i_id_replay_0a)
	last_lmul_iter_result_1a	<= '0;
      else if (vex_maskbit_1a)
	last_lmul_iter_result_1a	<= last_lmul_iter_result_0a;
   end
   
   always_ff@(posedge i_clk) begin
      if(i_vex_vmaskbit_0a & ~i_id_replay_0a)
	ffs_last_lmul_iter_1a		<= '0;
      else if (vex_maskbit_1a)
	ffs_last_lmul_iter_1a		<= ffs_last_lmul_iter_1a | ffs_last_lmul_iter_0a; //need to reciculate from 1st to last lmul iteration
   end

   always_ff@(posedge i_clk) begin
      if(vffs_1a & vex_maskbit_1a & ~ffs_last_lmul_iter_1a)
	last_ffs_ptr_1a            <= last_ffs_ptr_0a;
   end
   always_ff@(posedge i_clk)
     if(src1_en_0a) begin 
	final_src1_1a        	        <= final_src1_0a;  //overload non-inverted into higher bits, so that it could be saved for min/max comparision.
	issgn_src1_1a                   <= i_issgn_src1_0a;
     end
   
   always_ff @ (posedge i_clk) begin
      if(src2_en_0a) begin
	 final_src2_1a[VLEN-1:0]	<= final_src2_0a;
	 addorsub_1a		        <= i_addorsub_0a;
	 cryorbrw_mask_1a               <= cryorbrw_mask_0a;
	 lmul_1a[2:0]                   <= i_lmul_0a[2:0];
	 
      end 
   end
  
   
endmodule
