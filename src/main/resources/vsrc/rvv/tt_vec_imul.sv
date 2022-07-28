// See LICENSE.TT for license details.
module tt_vec_imul #(parameter
   VLEN=128
)
(
		    input [1:0] 	      i_vxrm_0a, 
		    input 		      i_clk,
		    input [VLEN-1:0] 	      i_src1_0a,
		    input [VLEN-1:0] 	      i_src2_0a,
		    input [VLEN-1:0] 	      i_src3_0a,
		    input 		      i_issgn_src1_0a,
		    input 		      i_issgn_src2_0a,
		    input 		      i_wdeop_0a, //~wdeop and ~high implies lo
		    input 		      i_mulh_0a,
		    input 		      i_mulen_0a,
		    input [1:0] 	      i_vsew_0a, 
		    input 		      i_macc_0a,
		    input 		      i_rnden_0a,
		    input 		      i_cmpmul_0a,
		    input [VLEN/8-1:0][128:0] i_mulsum_1a,
		    input [VLEN-1:0] 	      i_data_1a, //These are ops that need rounding, or iterated or narrowing
		    input 		      i_avg_1a,
		    input [VLEN/8-1:0] 	      i_avgbitd_1a,
		    input [VLEN/8-1:0] 	      i_avgbitdm1_1a,
		    input [VLEN/16-1:0]       i_sat_vnclip_pos_1a,
		    input [VLEN/16-1:0]       i_sat_vnclip_neg_1a, 
		    input 		      i_vnclipu_1a,
		    input 		      i_vnclip_1a,
		   
		    input 		      i_nrwop_0a,
		    input 		      i_iterate_1a,
		    input [VLEN/8 -1:0][ 7:0] i_src2_rs8_dm1to0_1a,
		    input [VLEN/16-1:0][15:0] i_src2_rs16_dm1to0_1a,
		    input [VLEN/32-1:0][31:0] i_src2_rs32_dm1to0_1a,
		    input [VLEN/64-1:0][63:0] i_src2_rs64_dm1to0_1a,
		    input [$clog2(VLEN/8)+2:0]i_iterate_cnt_0a,
		    
		    output logic [VLEN-1:0]      o_data_2a,
		    output logic 	      o_data_vld_2a,
		    output logic 	      o_sat_csr_2a,
		    
		    //to shared mul.
		    output logic [VLEN/8-1:0][63:0] o_sized_src2_0a,
		    output logic [VLEN/8-1:0][63:0] o_sized_src1_0a
		  //  output logic 	      o_mulissgn_0a
		    
		    );

      function automatic logic [VLEN/8-1:0][63:0] sized_src_0a;
      input [VLEN-1:0] i_src_0a;
      input [1:0]      i_vsew_0a;
      input 	       i_issgn_src_0a;
      
      case(i_vsew_0a[1:0])
	2'b00: 
	  begin
	     sized_src_0a = 'x;
	     for(int i=0; i<VLEN/64; i++) begin
		sized_src_0a[i][63:0] = {{56{i_src_0a[i*8+7] & i_issgn_src_0a}},i_src_0a[i*8+:8]};
	     end
	     for(int i=VLEN/64; i<VLEN/32; i++) begin
		sized_src_0a[i][31:0] = {{24{i_src_0a[i*8+7] & i_issgn_src_0a}},i_src_0a[i*8+:8]};
	     end
	     for(int i=VLEN/32; i<VLEN/16; i++) begin
		sized_src_0a[i][15:0] = {{8{i_src_0a[i*8+7] & i_issgn_src_0a}},i_src_0a[i*8+:8]};
	     end
	     for(int i=VLEN/16; i<VLEN/8; i++) begin
		sized_src_0a[i][7:0]  = i_src_0a[i*8+:8];
	     end
	  end
	
	2'b01: 
	  begin
	     sized_src_0a = 'x;
	     for(int i=0; i<VLEN/64; i++) begin
		sized_src_0a[i][63:0] = {{48{i_src_0a[i*16+15] & i_issgn_src_0a}},i_src_0a[i*16+:16]};
	     end
	     for(int i=VLEN/64; i<VLEN/32; i++) begin
		sized_src_0a[i][31:0] = {{16{i_src_0a[i*16+15] & i_issgn_src_0a}},i_src_0a[i*16+:16]};
	     end
	     for(int i=VLEN/32; i<VLEN/16; i++) begin
		sized_src_0a[i][15:0] = i_src_0a[i*16+:16];
	     end
	  end
	2'b10:
	  begin
	     sized_src_0a = 'x;
	     for(int i=0; i<VLEN/64; i++) begin
	       sized_src_0a[i][63:0] = {{32{i_src_0a[i*32+31] & i_issgn_src_0a}},i_src_0a[i*32+:32]}; 
	     end
	     for(int i=VLEN/64; i<VLEN/32; i++) begin
	       sized_src_0a[i][31:0] = i_src_0a[i*32+:32]; 
	     end
	  end
	2'b11:
	  begin
	     sized_src_0a = 'x;
	     for(int i=0; i<VLEN/64; i++) begin
	       sized_src_0a[i][63:0] = i_src_0a[i*64+:64]; 
	     end
	  end
	default: sized_src_0a = 'x;
      endcase
   endfunction // case


   logic 			   nrwop_1a;
   
   logic [1:0] 			   vsew_1a,vxrm_1a;
   logic [1:0] 			   vsew_2a,vxrm_2a;
   logic [VLEN-1:0] 		   src3_1a,src3_2a;
   logic [VLEN/8-1:0] 		   rnd_2a,sum_sized_dm1to0_2a,sum_sized_dm2to0_2a,sum_sized_d_2a,sum_sized_dm1_2a;
   logic                           mulh_1a,mulen_1a,rnden_1a,macc_1a,wdeop_1a,cmpmul_1a;
   logic                           mulh_2a,mulen_2a,rnden_2a,macc_2a,wdeop_2a,cmpmul_2a;
   logic [VLEN-1:0] 		   cmpmul_res_2a,mul_or_rnden_1a;
   logic [VLEN-1:0] 		   sum_hlf_2a,sum_rndacc_2a,prop_2a,gen_2a;
   logic [VLEN/8-1:0] 		   cin_2a,c_in_2a,cout_2a;
   logic 			   avg_2a,vnclip_2a,vnclipu_2a;
   logic [VLEN/16-1:0] 		   sat_vnclip_pos_2a,sat_vnclip_neg_2a,sized_sat_vnclip_pos_2a;			   
   logic [VLEN/2-1:0] 		   sat_vnclip_data_2a;		   
   logic [VLEN-1:0] 		   sum_sized_2a,sat_vsmul_data_2a;
   logic [VLEN-1:0] 		   src2_wde_0a,src1_wde_0a;
   logic [VLEN-1:0] 		   mul_or_add_rnd_datin_1a;
   logic [VLEN/8-1:0] 		   killrnd_avoid_ovfl_2a;		   
   logic [VLEN/8-1:0] 		   sgnbits_both_set_1a;		   
   logic [VLEN/8-1:0] 		   sized_sat_vsmul_pos_2a; 		   
   logic 			   sat_vnclip_csr_2a,sat_mul_csr_2a;

   
   wire   sel_upper_wdeop_0a = i_iterate_cnt_0a[0] & i_wdeop_0a;
   assign src2_wde_0a[VLEN/2-1:0     ] = sel_upper_wdeop_0a ? i_src2_0a[VLEN-1:VLEN/2] : i_src2_0a[VLEN/2-1:0];
   assign src2_wde_0a[VLEN-1  :VLEN/2] =                      i_src2_0a[VLEN-1:VLEN/2];      
   assign src1_wde_0a[VLEN/2-1:     0] = sel_upper_wdeop_0a ? i_src1_0a[VLEN-1:VLEN/2] : i_src1_0a[VLEN/2-1:0];
   assign src1_wde_0a[VLEN-1  :VLEN/2] =                      i_src1_0a[VLEN-1:VLEN/2]; 
   assign o_sized_src2_0a              = sized_src_0a(src2_wde_0a[VLEN-1:0],i_vsew_0a[1:0],i_issgn_src2_0a);
   assign o_sized_src1_0a              = sized_src_0a(src1_wde_0a[VLEN-1:0],i_vsew_0a[1:0],i_issgn_src1_0a);
   
  
   always_comb
     for(int i=0; i<VLEN/8; i++) 
       case(vxrm_2a) 
	 2'b00: rnd_2a[i] = sum_sized_dm1_2a[i];
	 2'b01: rnd_2a[i] = sum_sized_dm1_2a[i] & (sum_sized_dm2to0_2a[i] | sum_sized_d_2a[i]);
	 2'b10: rnd_2a[i] = 1'b0;
	 2'b11: rnd_2a[i] = ~sum_sized_d_2a[i] & sum_sized_dm1to0_2a[i];
       endcase

  // always_comb
  //   case(vsew_2a[1:0])
  //     2'b00: for(int i=0; i<16; i++) sum_hlf_2a[i*8 +: 8] = sum_sized_2a[i][7:0] ;
  //     2'b01: for(int i=0; i<8;  i++) sum_hlf_2a[i*16+:16] = sum_sized_2a[i][15:0];
  //     2'b10: for(int i=0; i<4;  i++) sum_hlf_2a[i*32+:32] = sum_sized_2a[i][31:0];
  //     2'b11:                         sum_hlf_2a[   127:0] = 'x;
  //   endcase
   
   
   assign cin_2a[15:0] = {16{vsew_2a[1:0] == 2'b00}} |  {8{1'b0,vsew_2a[1:0] == 2'b01}} | {4{3'b000,vsew_2a[1:0] == 2'b11}} & c_in_2a[15:0];

   always_comb
     case(vsew_2a[1:0])
       2'b00: for(int i=0; i<VLEN/16;  i++) sat_vnclip_data_2a[i*8 +: 8] = sized_sat_vnclip_pos_2a[i] ? {vnclipu_2a,{ 7{1'b1}}} : (sat_vnclip_neg_2a[i] ?  {1'b1,{ 7{1'b0}}} :  sum_rndacc_2a[i*8 +:8 ]);
       2'b01: for(int i=0; i<VLEN/32;  i++) sat_vnclip_data_2a[i*16+:16] = sized_sat_vnclip_pos_2a[i] ? {vnclipu_2a,{15{1'b1}}} : (sat_vnclip_neg_2a[i] ?  {1'b1,{15{1'b0}}} :  sum_rndacc_2a[i*16+:16]);
       2'b10: for(int i=0; i<VLEN/64;  i++) sat_vnclip_data_2a[i*32+:32] = sized_sat_vnclip_pos_2a[i] ? {vnclipu_2a,{31{1'b1}}} : (sat_vnclip_neg_2a[i] ?  {1'b1,{31{1'b0}}} :  sum_rndacc_2a[i*32+:32]);
       2'b11: for(int i=0; i<VLEN/128; i++) sat_vnclip_data_2a[i*64+:64] = sized_sat_vnclip_pos_2a[i] ? {vnclipu_2a,{63{1'b1}}} : (sat_vnclip_neg_2a[i] ?  {1'b1,{63{1'b0}}} :  sum_rndacc_2a[i*64+:64]);
     endcase

   always_comb
     case(vsew_2a[1:0])
       2'b00: for(int i=0; i<VLEN/8;  i++) sat_vsmul_data_2a[i*8 +: 8] = sized_sat_vsmul_pos_2a[i] ? {1'b0,{ 7{1'b1}}} : sum_rndacc_2a[i*8 +:8 ];
       2'b01: for(int i=0; i<VLEN/16; i++) sat_vsmul_data_2a[i*16+:16] = sized_sat_vsmul_pos_2a[i] ? {1'b0,{15{1'b1}}} : sum_rndacc_2a[i*16+:16];
       2'b10: for(int i=0; i<VLEN/32; i++) sat_vsmul_data_2a[i*32+:32] = sized_sat_vsmul_pos_2a[i] ? {1'b0,{31{1'b1}}} : sum_rndacc_2a[i*32+:32];
       2'b11: for(int i=0; i<VLEN/64; i++) sat_vsmul_data_2a[i*64+:64] = sized_sat_vsmul_pos_2a[i] ? {1'b0,{63{1'b1}}} : sum_rndacc_2a[i*64+:64];
     endcase 

   
   assign o_data_2a[VLEN-1:0] = (vnclipu_2a | vnclip_2a ) ? {'x,sat_vnclip_data_2a[VLEN/2-1:0]}
			                                  : (rnden_2a & mulen_2a) ? sat_vsmul_data_2a[VLEN-1:0]
                                                                                  : (rnden_2a | cmpmul_2a | macc_2a) ? sum_rndacc_2a[VLEN-1:0]  
                                                                                                                     : sum_sized_2a [VLEN-1:0];
    always_comb
     for(int i=0;i<VLEN/8;i++) begin 
	case(vsew_2a[1:0] + wdeop_2a)
	  2'b00: c_in_2a[i] =                 rnden_2a & rnd_2a[i]   & ~killrnd_avoid_ovfl_2a[i]    | cmpmul_2a;
	  2'b01: c_in_2a[i] = (i[0]   == 0) ? rnden_2a & rnd_2a[i/2] & ~killrnd_avoid_ovfl_2a[i/2]  | cmpmul_2a : cout_2a[i-1];
	  2'b10: c_in_2a[i] = (i[1:0] == 0) ? rnden_2a & rnd_2a[i/4] & ~killrnd_avoid_ovfl_2a[i/4]  | cmpmul_2a : cout_2a[i-1];
	  2'b11: c_in_2a[i] = (i[2:0] == 0) ? rnden_2a & rnd_2a[i/8] & ~killrnd_avoid_ovfl_2a[i/8]  | cmpmul_2a : cout_2a[i-1];
	endcase 
     end
   //Note: for signed vnclip a saturation via cout can not occur, because the number is sign extended and a cout=1 represents a 0. Cout saturation only applied to unsigned clip
   always_comb begin
      sized_sat_vnclip_pos_2a ='0;
      case(vsew_2a) 
	2'b00:  for(int i=0; i<VLEN/16;  i++) sized_sat_vnclip_pos_2a[i] = sat_vnclip_pos_2a[i]  | vnclipu_2a & cout_2a[i];
	2'b01:  for(int i=0; i<VLEN/32;  i++) sized_sat_vnclip_pos_2a[i] = sat_vnclip_pos_2a[i]  | vnclipu_2a & cout_2a[i*2+1];
	2'b10:  for(int i=0; i<VLEN/64;  i++) sized_sat_vnclip_pos_2a[i] = sat_vnclip_pos_2a[i]  | vnclipu_2a & cout_2a[i*4+3];
	2'b11:  for(int i=0; i<VLEN/128; i++) sized_sat_vnclip_pos_2a[i] = sat_vnclip_pos_2a[i]  | vnclipu_2a & cout_2a[i*8+7];
      endcase 
   end

   assign o_sat_csr_2a =   sat_vnclip_csr_2a  & (vnclip_2a | vnclipu_2a)
                         | sat_mul_csr_2a     & (mulen_2a & rnden_2a);

   
   always_comb begin
      sat_vnclip_csr_2a = '0;
      case(vsew_2a) 
	2'b00:  for(int i=0; i<VLEN/16;  i++) sat_vnclip_csr_2a |= sat_vnclip_neg_2a[i] | sat_vnclip_pos_2a[i]  | vnclipu_2a & cout_2a[i]    ;
	2'b01:  for(int i=0; i<VLEN/32;  i++) sat_vnclip_csr_2a |= sat_vnclip_neg_2a[i] | sat_vnclip_pos_2a[i]  | vnclipu_2a & cout_2a[i*2+1];
	2'b10:  for(int i=0; i<VLEN/64;  i++) sat_vnclip_csr_2a |= sat_vnclip_neg_2a[i] | sat_vnclip_pos_2a[i]  | vnclipu_2a & cout_2a[i*4+3];
	2'b11:  for(int i=0; i<VLEN/128; i++) sat_vnclip_csr_2a |= sat_vnclip_neg_2a[i] | sat_vnclip_pos_2a[i]  | vnclipu_2a & cout_2a[i*8+7];
      endcase 
   end

   always_comb begin
      sat_mul_csr_2a = '0;
      case(vsew_2a) 
	2'b00:  for(int i=0;i<VLEN/8; i++) sat_mul_csr_2a |= sized_sat_vsmul_pos_2a[i];
	2'b01:  for(int i=0;i<VLEN/16;i++) sat_mul_csr_2a |= sized_sat_vsmul_pos_2a[i];
	2'b10:  for(int i=0;i<VLEN/32;i++) sat_mul_csr_2a |= sized_sat_vsmul_pos_2a[i];
	2'b11:  for(int i=0;i<VLEN/64;i++) sat_mul_csr_2a |= sized_sat_vsmul_pos_2a[i];
      endcase 
   end
   

   always_comb begin
      cmpmul_res_2a = cmpmul_2a ? ~sum_sized_2a : sum_sized_2a;
      prop_2a       = cmpmul_res_2a | src3_2a & {VLEN{macc_2a}};
      gen_2a        = cmpmul_res_2a & src3_2a & {VLEN{macc_2a}};
      
      for(int i=0; i<VLEN/8; i++) {cout_2a[i], sum_rndacc_2a[i*8+:8]} = {1'b0,gen_2a[i*8+:8]} + {1'b0,prop_2a[i*8+:8]} + {8'd0,c_in_2a[i]};
   end
   
   always_ff @(posedge i_clk) begin
      mulen_1a						<= i_mulen_0a;
      o_data_vld_2a                                     <= mulen_1a | rnden_1a | nrwop_1a | i_iterate_1a;
      rnden_1a						<= i_rnden_0a;
      rnden_2a                                          <= rnden_1a;
      mulen_2a                                          <= mulen_1a;
      nrwop_1a                                          <= i_nrwop_0a;
   end
   
    always_ff @(posedge i_clk)  
     if(1'b1 | i_mulen_0a | i_rnden_0a | i_nrwop_0a | mulen_1a | rnden_1a | nrwop_1a) begin
      wdeop_1a						<= i_wdeop_0a;
      vsew_1a						<= i_vsew_0a;
      vxrm_1a						<= i_vxrm_0a;
      macc_1a						<= i_macc_0a;
      
      cmpmul_1a						<= i_cmpmul_0a;
       mulh_1a						<= i_mulh_0a;	
     end

   wire sel_idata_1a = rnden_1a | i_iterate_1a | nrwop_1a;
   //For narrow ops need to  correct dm1to0.
   logic [VLEN/8-1:0][63:0] src2_dm1to0_1a;
   always_comb begin
      src2_dm1to0_1a = '0;
      case(vsew_1a + nrwop_1a)
	2'b00: for(int i=0; i<VLEN/8;  i++) src2_dm1to0_1a[i][63:0] = {i_src2_rs8_dm1to0_1a [i][ 7:0],56'd0};
	2'b01: for(int i=0; i<VLEN/16; i++) src2_dm1to0_1a[i][63:0] = {i_src2_rs16_dm1to0_1a[i][15:0],48'd0};
	2'b10: for(int i=0; i<VLEN/32; i++) src2_dm1to0_1a[i][63:0] = {i_src2_rs32_dm1to0_1a[i][31:0],32'd0};
	2'b11: for(int i=0; i<VLEN/64; i++) src2_dm1to0_1a[i][63:0] = {i_src2_rs64_dm1to0_1a[i][63:0]};
      endcase
   end

   
   //always_comb
   //  case(vsew_1a[1:0])
   //    2'b00: for(int i=0; i<16;i++) mul_or_add_rnd_datin_1a[i*8 +:8 ] = (mulen_1a ? i_mulsum_1a[i][14:7]  : i_data_1a[i*8 +:8]);
   //    2'b01: for(int i=0; i<8 ;i++) mul_or_add_rnd_datin_1a[i*16+:16] = (mulen_1a ? i_mulsum_1a[i][30:15] : i_data_1a[i*16+:16]);
   //    2'b10: for(int i=0; i<4 ;i++) mul_or_add_rnd_datin_1a[i*32+:32] = (mulen_1a ? i_mulsum_1a[i][62:31] : i_data_1a[i*32+:32]);
   //    default: mul_or_add_rnd_datin_1a = 'x;
   //  endcase
   //Removing the mul input from the above equation, as for a multiplier 7f->80 or ff->00 case is not possible for  vsmul instruction. This is the only mul instruction that requires rounding.
   //I'm going to keep the old code and the old name , in case I am wrong.
   always_comb
     case(vsew_1a[1:0])
       2'b00: for(int i=0; i<VLEN/8; i++) mul_or_add_rnd_datin_1a[i*8 +:8 ] = i_data_1a[i* 8+: 8];
       2'b01: for(int i=0; i<VLEN/16;i++) mul_or_add_rnd_datin_1a[i*16+:16] = i_data_1a[i*16+:16];
       2'b10: for(int i=0; i<VLEN/32;i++) mul_or_add_rnd_datin_1a[i*32+:32] = i_data_1a[i*32+:32];
       2'b11: for(int i=0; i<VLEN/64;i++) mul_or_add_rnd_datin_1a[i*64+:64] = i_data_1a[i*64+:64];
       default: mul_or_add_rnd_datin_1a = 'x;
     endcase
   
   //kill rnd ovfl to prevent 7f -> 80 or ff->00.
   //This check is not required for mulitplier, as there will never be a case that rounding flips sign bit.
   //The avg add will also never see a flip, as when two max +ve numbers are added(both odds) the sum is even and the shifted out bit is a 0.
   always_ff @(posedge i_clk)
     if(sel_idata_1a) begin
	killrnd_avoid_ovfl_2a <= '0;
	case(vsew_1a[1:0]) 
	  2'b00: for(int i=0; i<VLEN/8; i++) killrnd_avoid_ovfl_2a[i]  <=  i_vnclipu_1a & (&mul_or_add_rnd_datin_1a[i*8 +: 8]) | i_vnclip_1a & (&{~mul_or_add_rnd_datin_1a[i*8+7]  ,mul_or_add_rnd_datin_1a[i*8 +: 7]});
	  2'b01: for(int i=0; i<VLEN/16;i++) killrnd_avoid_ovfl_2a[i]  <=  i_vnclipu_1a & (&mul_or_add_rnd_datin_1a[i*16+:16]) | i_vnclip_1a & (&{~mul_or_add_rnd_datin_1a[i*16+15],mul_or_add_rnd_datin_1a[i*16+:15]});
	  2'b10: for(int i=0; i<VLEN/32;i++) killrnd_avoid_ovfl_2a[i]  <=  i_vnclipu_1a & (&mul_or_add_rnd_datin_1a[i*32+:32]) | i_vnclip_1a & (&{~mul_or_add_rnd_datin_1a[i*32+31],mul_or_add_rnd_datin_1a[i*32+:31]});
	  2'b11: for(int i=0; i<VLEN/64;i++) killrnd_avoid_ovfl_2a[i]  <=  i_vnclipu_1a & (&mul_or_add_rnd_datin_1a[i*64+:64]) | i_vnclip_1a & (&{~mul_or_add_rnd_datin_1a[i*64+63],mul_or_add_rnd_datin_1a[i*64+:63]});
	endcase
     end

   //detect max -ve numbers resulting is a +ve number.
   always_ff @(posedge i_clk)
     if(mulen_1a & rnden_1a)
       case(vsew_1a)
	 2'b00: for(int i=0; i<VLEN/8; i++) sized_sat_vsmul_pos_2a[i] <= sgnbits_both_set_1a[i]     & (~i_mulsum_1a[i][ 15] & i_mulsum_1a[i][ 14]);
	 2'b01: for(int i=0; i<VLEN/16;i++) sized_sat_vsmul_pos_2a[i] <= sgnbits_both_set_1a[i*2+1] & (~i_mulsum_1a[i][ 31] & i_mulsum_1a[i][ 30]);
	 2'b10: for(int i=0; i<VLEN/32;i++) sized_sat_vsmul_pos_2a[i] <= sgnbits_both_set_1a[i*4+3] & (~i_mulsum_1a[i][ 63] & i_mulsum_1a[i][ 62]);
	 2'b11: for(int i=0; i<VLEN/64;i++) sized_sat_vsmul_pos_2a[i] <= sgnbits_both_set_1a[i*8+7] & (~i_mulsum_1a[i][127] & i_mulsum_1a[i][126]);
       endcase
	 
		 
   always_ff @(posedge i_clk)
      if(mulen_1a | rnden_1a | nrwop_1a | i_iterate_1a) begin
	 case(vsew_1a[1:0] + wdeop_1a)
	   2'b00: 
	     for(int i=0; i<VLEN/8; i++) 
	       begin 
		  sum_sized_2a[i*8+:8]   <= mulh_1a ? i_mulsum_1a[i][15:8] 
	                                            : sel_idata_1a ? (mulen_1a ? i_mulsum_1a[i][14:7] : i_data_1a[i*8+:8])
                                                                   : i_mulsum_1a[i][7:0]; 
		  sum_sized_dm1to0_2a[i] <= mulen_1a ? |i_mulsum_1a[i][6:0] : ~i_avg_1a ? |src2_dm1to0_1a[i][63:0] : i_avgbitdm1_1a[i];   
		  sum_sized_dm1_2a[i]    <= mulen_1a ? i_mulsum_1a [i][6 ]  : ~i_avg_1a ?  src2_dm1to0_1a[i][63]   : i_avgbitdm1_1a[i]; 
		  sum_sized_d_2a[i]      <= mulen_1a ? i_mulsum_1a[i] [7 ]  : ~i_avg_1a ?      i_data_1a[i*8+0]    : i_avgbitd_1a[i];  	
		  sum_sized_dm2to0_2a[i] <= mulen_1a ? |i_mulsum_1a[i][5:0] : ~i_avg_1a ? |src2_dm1to0_1a[i][62:0] : '0;    
		  
	       end
	   2'b01: 
	     for(int i=0; i<VLEN/16; i++) 
	       begin 
		  sum_sized_2a[i*16+:16] <= mulh_1a ? i_mulsum_1a[i][31:16] 
	                                            : sel_idata_1a  ? (mulen_1a ? i_mulsum_1a[i][30:15] : i_data_1a[i*16+:16])
                                                                    : i_mulsum_1a[i][15:0]; 
		  sum_sized_dm1to0_2a[i] <= mulen_1a ? |i_mulsum_1a[i][14:0] : ~i_avg_1a ? |src2_dm1to0_1a[i][63:0] : i_avgbitdm1_1a[i*2]; 
		  sum_sized_dm1_2a[i]    <= mulen_1a ? i_mulsum_1a[i][14]    : ~i_avg_1a ?  src2_dm1to0_1a[i][63]   : i_avgbitdm1_1a[i*2]; 
		  sum_sized_d_2a[i]      <= mulen_1a ? i_mulsum_1a[i][15]    : ~i_avg_1a ?      i_data_1a[i*16+0]   : i_avgbitd_1a[i*2]; 
		  sum_sized_dm2to0_2a[i] <= mulen_1a ? |i_mulsum_1a[i][13:0] : ~i_avg_1a ? |src2_dm1to0_1a[i][62:0] : '0;                 
	       end
	   2'b10: 
	     for(int i=0; i<VLEN/32; i++) 
	       begin 
		  sum_sized_2a[i*32+:32] <= mulh_1a ? i_mulsum_1a[i][63:32] 
	                                            : sel_idata_1a ? (mulen_1a ? i_mulsum_1a[i][62:31] : i_data_1a[i*32+:32])
	                                                           : i_mulsum_1a[i][31:0]; 
		  sum_sized_dm1to0_2a[i] <= mulen_1a ? |i_mulsum_1a[i][30:0] : ~i_avg_1a ? |src2_dm1to0_1a[i][63:0] : i_avgbitdm1_1a[i*4];
		  sum_sized_dm1_2a[i]    <= mulen_1a ? i_mulsum_1a[i][30]    : ~i_avg_1a ?  src2_dm1to0_1a[i][63]   : i_avgbitdm1_1a[i*4];
		  sum_sized_d_2a[i]      <= mulen_1a ? i_mulsum_1a[i][31]    : ~i_avg_1a ?      i_data_1a[i*32+0]   : i_avgbitd_1a[i*4]; 
		  sum_sized_dm2to0_2a[i] <= mulen_1a ? |i_mulsum_1a[i][29:0] : ~i_avg_1a ? |src2_dm1to0_1a[i][62:0] : '0;                 
	       end
	   2'b11: 
	     for(int i=0; i<VLEN/64; i++)
               begin
	          sum_sized_2a[i*64+:64]    <= mulh_1a ? i_mulsum_1a[i][127:64]
	                                            : sel_idata_1a ? (mulen_1a ? i_mulsum_1a[i][126:63] : i_data_1a[i*64+:64])
                                                                   : i_mulsum_1a[i][63:0]; 
		  sum_sized_dm1to0_2a[i] <= mulen_1a ? |i_mulsum_1a[i][62:0] : ~i_avg_1a ? |src2_dm1to0_1a[i][63:0] : i_avgbitdm1_1a[i*8];
		  sum_sized_dm1_2a[i]    <= mulen_1a ? i_mulsum_1a[i][62]    : ~i_avg_1a ?  src2_dm1to0_1a[i][63]   : i_avgbitdm1_1a[i*8];
		  sum_sized_d_2a[i]      <= mulen_1a ? i_mulsum_1a[i][63]    : ~i_avg_1a ?      i_data_1a[i*64+0]   : i_avgbitd_1a[i*8]; 
		  sum_sized_dm2to0_2a[i] <= mulen_1a ? |i_mulsum_1a[i][61:0] : ~i_avg_1a ? |src2_dm1to0_1a[i][62:0] : '0;                 
               end
      endcase
	 
      end

    always_ff @(posedge i_clk)
     if(1'b1 /*mulen_1a | rnden_1a | i_iterate_1a | i_nrwop_1a*/) begin
	wdeop_2a					<= wdeop_1a;
	vsew_2a					        <= vsew_1a;
	vxrm_2a					        <= vxrm_1a;
	avg_2a                                          <= i_avg_1a;
	vnclipu_2a                                      <= i_vnclipu_1a;
	vnclip_2a                                       <= i_vnclip_1a;
     end
   
   always_ff @(posedge i_clk)
     if(i_vnclipu_1a | i_vnclip_1a) begin
       sat_vnclip_pos_2a                                <= i_sat_vnclip_pos_1a;
       sat_vnclip_neg_2a                                <= i_sat_vnclip_neg_1a;
     end
   
   always_ff @(posedge i_clk)
     if(1'b1 | mulen_1a) begin
	macc_2a                                         <= macc_1a;
	cmpmul_2a                                       <= cmpmul_1a;
	mulh_2a                                         <= mulh_1a;
     end
 
   always_ff @(posedge i_clk)
     if(i_mulen_0a & i_macc_0a)
       src3_1a						<= i_src3_0a;
   
   always_ff @(posedge i_clk)
     if(mulen_1a & macc_1a)
       src3_2a						<= src3_1a;
   
   always_ff @(posedge i_clk)
     if(i_mulen_0a & i_rnden_0a)
       for(int i=0;i<VLEN/8;i++)
	 sgnbits_both_set_1a[i]                         <= i_src1_0a[i*8+7] & i_src2_0a[i*8+7];

endmodule // tt_vecmul_dp



// Local Variables:
// verilog-auto-inout-ignore-regexp: "^unused__"
// verilog-library-directories:("." "../../common/rtl/")
// verilog-library-extensions:(".v" ".h")
// End:

