// See LICENSE.TT for license details.
module tt_vec_idp #(parameter
   VLEN=256,
   XLEN=64
)
(
		  input 		    i_clk,
		  input 		    i_reset_n,
		  input 		    i_vex_en_0a,
                  input 		    i_vex_en_1a,
		  input [1:0] 		    i_vxrm_0a,
		  input [VLEN-1:0] 	    i_src1_0a,
		  input [VLEN-1:0] 	    i_src2_0a,
		  input [VLEN-1:0] 	    i_src3_0a,
		  input 		    i_issgn_src1_0a,
		  input 		    i_issgn_src2_0a,
		  //input 		    i_dbl_0a,
		  input 		    i_adden_0a,
		  input 		    i_avg_0a,
		  input [1:0] 		    i_vsew_0a,
		  input [2:0] 		    i_lmul_0a,
		  input 		    i_addorsub_0a,
		  input 		    i_reductop_0a,
		  input 		    i_inc_iterate_0a,
		  //input 	       i_cryorbrw_0a,
		  input 		    i_wdeop_0a,
		  input 		    i_src1hw_0a,
		  input 		    i_inversesub_0a,
		  input [VLEN/8-1:0] 	    i_mask_0a,
                  input [VLEN/8-1:0] 	    i_vm0_sized_0a,
		  input 		    i_vmerge_0a,
 		  input 		    i_usemask_0a,
		  input 		    i_wrmask_0a,
		  input 		    i_mulh_0a,
		  input 		    i_mulen_0a,
		  input 		    i_macc_0a,
		  input 		    i_rnden_0a,
		  input 		    i_cmpmul_0a,
		  input [VLEN/8-1:0][128:0] 	    i_mulsum_1a,
		  input 		    i_nrwop_0a,
		  input 		    i_iterate_0a,
		  input 		    i_iterate_1a,
		  input [$clog2(VLEN/8)+2:0] i_iterate_cnt_1a,
		  input [$clog2(VLEN+1)-1:0] i_vl_cnt_1a,
		  input [$clog2(VLEN/8)+2:0] i_iterate_cnt_0a,
		  input [$clog2(VLEN/8)-1:0] i_vs1_gather_indx_1a,
		  input 		    i_reduct_wdeop_1a,
		  input 		    i_ixv_tov_mv_0a,
		  input 		    i_vmvgrp_0a,
		  input 		    i_v_tox_mv_0a,
		  input 		    i_vex_per_0a,
		  input 		    i_vex_shft_0a,
		  input 		    i_vex_bitwise_0a,
		  input 		    i_vex_vmaskbit_0a,
		  input 		    i_id_replay_0a,
		  input 		    i_mask_only_instrn_1a,
		  input [31:0] 		    i_vex_instrn_1a,
	 	  input [2:0] 		    i_lmul_cnt_0a,
		  input [63:0] 	            i_reductop_data_0a,
	
		 
		  input [63:0] 		    i_scalar_imm_slide_1a,
		  input 		    i_v_vm_1a,
		  input 		    i_v_vm_0a,
		  input 		    i_sat_instrn_0a,
		  
	
	 	  output logic 		    o_v_tox_mv_1a, 
		  output logic 		    o_data_vld_2a,
		  output logic 		    o_sat_csr_2a,
		  output logic [VLEN-1:0] 	    o_data_2a,
		  output logic 		    o_data_vld_1a,
		  output logic [VLEN-1:0] 	    o_data_1a,
		  //output logic 		    o_slide1_up_1a,
		  //output logic 		    o_slide1_dwn_1a,
		  output logic 		    o_slide_dwn_1a, 
		  output logic [63:0] 	    o_slide_shft_amt_1a, 
		  output logic [VLEN/8-1:0][63:0] o_sized_src1_0a,
		  output logic [VLEN/8-1:0][63:0] o_sized_src2_0a
		  
		  );
   
   
   /*AUTOWIRE*/
   // Beginning of automatic wires (for undeclared instantiated-module outputs)
   logic		avg_1a;			// From add of tt_vec_iadd.v
   logic [VLEN/8-1:0]		avgbitd_1a;		// From add of tt_vec_iadd.v
   logic [VLEN/8-1:0]		avgbitdm1_1a;		// From add of tt_vec_iadd.v
   logic [VLEN/16-1:0]		sat_vnclip_neg_1a;	// From add of tt_vec_iadd.v
   logic [VLEN/16-1:0]		sat_vnclip_pos_1a;	// From add of tt_vec_iadd.v
   logic                        satval_2a;
   logic		vnclip_1a;		// From add of tt_vec_iadd.v
   logic		vnclipu_1a;		// From add of tt_vec_iadd.v
   // End of automatics
   
   logic [1:0] 				    vsew_1a;
   logic [2:0] 				    lmul_1a;
   logic [VLEN/8 -1:0][ 7:0] 		    src2_rs8_dm1to0_1a;
   logic [VLEN/16-1:0][15:0] 		    src2_rs16_dm1to0_1a;
   logic [VLEN/32-1:0][31:0] 		    src2_rs32_dm1to0_1a;
   logic [VLEN/64-1:0][63:0] 		    src2_rs64_dm1to0_1a;
   logic                                    sat_csr_2a;
   

   assign o_sat_csr_2a = sat_csr_2a | satval_2a; //satval_2a sets for saturating instructions in the xls sheet

	       
   always_ff @(posedge i_clk) begin
      vsew_1a[1:0]      <= i_vsew_0a[1:0];
      lmul_1a[2:0]      <= i_lmul_0a[2:0];   
   end
   /*tt_vec_iadd AUTO_TEMPLATE (
    //.\(.*\)_\(.*\)        (\1_@"(+ 1 @)"a[]),
    .o_src2_rs\(.*\)        (src2_rs\1),
    .o_avg\(.*_\)1a         (avg\11a[]),
    .o_vnclip\(.*_\)1a      (vnclip\11a[]),
    .o_sat_vnclip_pos_1a    (sat_vnclip_pos_1a[]),
    .o_sat_vnclip_neg_1a    (sat_vnclip_neg_1a[]),
    .o_satval_2a            (satval_2a),
    )*/
   tt_vec_iadd #(.VLEN(VLEN),
                 .XLEN(XLEN) )
   add(
		   /*AUTOINST*/
		   // Outputs
		   .o_slide_dwn_1a	(o_slide_dwn_1a),
		   .o_slide_shft_amt_1a	(o_slide_shft_amt_1a[63:0]),
		   .o_sat_vnclip_pos_1a	(sat_vnclip_pos_1a), // Templated
		   .o_sat_vnclip_neg_1a	(sat_vnclip_neg_1a), // Templated
		   .o_vnclipu_1a	(vnclipu_1a),		 // Templated
		   .o_vnclip_1a		(vnclip_1a),		 // Templated
		   .o_avg_1a		(avg_1a),		 // Templated
		   .o_avgbitd_1a	(avgbitd_1a),	 // Templated
		   .o_avgbitdm1_1a	(avgbitdm1_1a),	 // Templated
		   .o_src2_rs8_dm1to0_1a(src2_rs8_dm1to0_1a),	 // Templated
		   .o_src2_rs16_dm1to0_1a(src2_rs16_dm1to0_1a),	 // Templated
		   .o_src2_rs32_dm1to0_1a(src2_rs32_dm1to0_1a),	 // Templated
		   .o_src2_rs64_dm1to0_1a(src2_rs64_dm1to0_1a),	 // Templated
		   .o_v_tox_mv_1a	(o_v_tox_mv_1a),
		   .o_data_vld_1a	(o_data_vld_1a),
		   .o_data_1a		(o_data_1a[VLEN-1:0]),
                   .o_satval_2a         (satval_2a),             // Templated
		   // Inputs
		   .i_reset_n		(i_reset_n),
		   .i_vex_en_0a		(i_vex_en_0a),
		   .i_vex_en_1a		(i_vex_en_1a),
		   .i_vxrm_0a		(i_vxrm_0a[1:0]),
		   .i_clk		(i_clk),
		   .i_src1_0a		(i_src1_0a[VLEN-1:0]),
		   .i_src2_0a		(i_src2_0a[VLEN-1:0]),
		   .i_src3_0a		(i_src3_0a[VLEN-1:0]),
		   .i_issgn_src1_0a	(i_issgn_src1_0a),
		   .i_issgn_src2_0a	(i_issgn_src2_0a),
		   .i_adden_0a		(i_adden_0a),
		   .i_avg_0a		(i_avg_0a),
		   .i_vsew_0a		(i_vsew_0a[1:0]),
		   .i_lmul_0a		(i_lmul_0a[2:0]),
		   .i_addorsub_0a	(i_addorsub_0a),
		   .i_inc_iterate_0a	(i_inc_iterate_0a),
		   .i_reductop_0a	(i_reductop_0a),
		   .i_src1hw_0a		(i_src1hw_0a),
		   .i_wdeop_0a		(i_wdeop_0a),
		   .i_inversesub_0a	(i_inversesub_0a),
		   .i_mask_0a		(i_mask_0a),
		   .i_vm0_sized_0a	(i_vm0_sized_0a),
		   .i_vmerge_0a		(i_vmerge_0a),
		   .i_usemask_0a	(i_usemask_0a),
		   .i_wrmask_0a		(i_wrmask_0a),
		   .i_ixv_tov_mv_0a	(i_ixv_tov_mv_0a),
		   .i_vmvgrp_0a		(i_vmvgrp_0a),
		   .i_v_tox_mv_0a	(i_v_tox_mv_0a),
		   .i_vex_per_0a	(i_vex_per_0a),
		   .i_vex_shft_0a	(i_vex_shft_0a),
		   .i_vex_bitwise_0a	(i_vex_bitwise_0a),
		   .i_vex_vmaskbit_0a	(i_vex_vmaskbit_0a),
		   .i_id_replay_0a	(i_id_replay_0a),
		   .i_mask_only_instrn_1a(i_mask_only_instrn_1a),
		   .i_vex_instrn_1a	(i_vex_instrn_1a[31:0]),
		   .i_lmul_cnt_0a	(i_lmul_cnt_0a[2:0]),
		   .i_reductop_data_0a	(i_reductop_data_0a),
		   .i_v_vm_0a		(i_v_vm_0a),
		   .i_scalar_imm_slide_1a(i_scalar_imm_slide_1a[63:0]),
		   .i_v_vm_1a		(i_v_vm_1a),
		   .i_sat_instrn_0a	(i_sat_instrn_0a),
		   .i_iterate_0a	(i_iterate_0a),
		   .i_iterate_1a	(i_iterate_1a),
		   .i_iterate_cnt_1a	(i_iterate_cnt_1a),
		   .i_vl_cnt_1a		(i_vl_cnt_1a),
		   .i_iterate_cnt_0a	(i_iterate_cnt_0a),
		   .i_vs1_gather_indx_1a(i_vs1_gather_indx_1a),
		   .i_reduct_wdeop_1a	(i_reduct_wdeop_1a));
   
   /*tt_vec_imul AUTO_TEMPLATE (
    //.\(.*\)_\(.*\)        (\1_@"(+ 1 @)"a[]),
    .i_src2_rs\(.*\)        (src2_rs\1),
    .i_avg\(.*_\)1a         (avg\11a[]),
    .i_vnclip\(.*_\)1a      (vnclip\11a[]),
    .i_sat_vnclip_pos_1a    (sat_vnclip_pos_1a[]),
    .i_sat_vnclip_neg_1a    (sat_vnclip_neg_1a[]),
    .o_sat_csr_2a           (sat_csr_2a),
    )*/
   tt_vec_imul #(.VLEN(VLEN))
   mul(
		   .i_data_1a		(o_data_1a[VLEN-1:0]),
		   // Outputs
		   /*AUTOINST*/
		   // Outputs
		   .o_data_2a		(o_data_2a[VLEN-1:0]),
		   .o_data_vld_2a	(o_data_vld_2a),
		   .o_sat_csr_2a	(sat_csr_2a),
		   .o_sized_src2_0a	(o_sized_src2_0a/*[15:0][31:0]*/),
		   .o_sized_src1_0a	(o_sized_src1_0a/*[15:0][31:0]*/),
		   // Inputs
		   .i_vxrm_0a		(i_vxrm_0a[1:0]),
		   .i_clk		(i_clk),
		   .i_src1_0a		(i_src1_0a[VLEN-1:0]),
		   .i_src2_0a		(i_src2_0a[VLEN-1:0]),
		   .i_src3_0a		(i_src3_0a[VLEN-1:0]),
		   .i_issgn_src1_0a	(i_issgn_src1_0a),
		   .i_issgn_src2_0a	(i_issgn_src2_0a),
		   .i_wdeop_0a		(i_wdeop_0a),
		   .i_mulh_0a		(i_mulh_0a),
		   .i_mulen_0a		(i_mulen_0a),
		   .i_vsew_0a		(i_vsew_0a[1:0]),
		   .i_macc_0a		(i_macc_0a),
		   .i_rnden_0a		(i_rnden_0a),
		   .i_cmpmul_0a		(i_cmpmul_0a),
		   .i_mulsum_1a		(i_mulsum_1a/*[15:0][64:0]*/),
		   .i_avg_1a		(avg_1a),		 // Templated
		   .i_avgbitd_1a	(avgbitd_1a),	 // Templated
		   .i_avgbitdm1_1a	(avgbitdm1_1a),	 // Templated
		   .i_sat_vnclip_pos_1a	(sat_vnclip_pos_1a), // Templated
		   .i_sat_vnclip_neg_1a	(sat_vnclip_neg_1a), // Templated
		   .i_vnclipu_1a	(vnclipu_1a),		 // Templated
		   .i_vnclip_1a		(vnclip_1a),		 // Templated
		   .i_nrwop_0a		(i_nrwop_0a),
		   .i_iterate_1a	(i_iterate_1a),
		   .i_src2_rs8_dm1to0_1a(src2_rs8_dm1to0_1a),	 // Templated
		   .i_src2_rs16_dm1to0_1a(src2_rs16_dm1to0_1a),	 // Templated
		   .i_src2_rs32_dm1to0_1a(src2_rs32_dm1to0_1a),	 // Templated
		   .i_src2_rs64_dm1to0_1a(src2_rs64_dm1to0_1a),	 // Templated
		   .i_iterate_cnt_0a	(i_iterate_cnt_0a));
   
   
   
endmodule
// Local Variables:
// verilog-library-directories:(".")
// verilog-library-extensions:(".sv" ".h" ".v")
// End:
