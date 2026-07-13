// See LICENSE.TT for license details.
module tt_vec_mul_dp #(parameter
   VLEN=256
)
(
		     input [VLEN/8-1:0][63:0]  i_sized_src2_0a,
		     input [VLEN/8-1:0][63:0]  i_sized_src1_0a,
		     input 		       i_issgn_0a,
		     input 		       i_issgnsrc2_0a,
		     input 		       i_mulen_0a,
		     input i_clk ,
		 
		     output logic [VLEN/8-1:0][128:0] o_sum_1a
		     );
   
   always_ff @(posedge i_clk) begin
      if (i_mulen_0a) begin
         for(int i=VLEN/16;i<VLEN/8; i++) o_sum_1a[i] <= {{112'b0, {9{i_issgn_0a && i_sized_src1_0a[i][7]}},  i_sized_src1_0a[i][7:0]}  * {{9 {i_issgnsrc2_0a && i_sized_src2_0a[i][7]}},  i_sized_src2_0a[i][7:0]}};
         for(int i=VLEN/32;i<VLEN/16;i++) o_sum_1a[i] <= {{96'b0, {17{i_issgn_0a && i_sized_src1_0a[i][15]}}, i_sized_src1_0a[i][15:0]} * {{17{i_issgnsrc2_0a && i_sized_src2_0a[i][15]}}, i_sized_src2_0a[i][15:0]}};
         for(int i=VLEN/64;i<VLEN/32;i++) o_sum_1a[i] <= {{64'b0, {33{i_issgn_0a && i_sized_src1_0a[i][31]}}, i_sized_src1_0a[i][31:0]} * {{33{i_issgnsrc2_0a && i_sized_src2_0a[i][31]}}, i_sized_src2_0a[i][31:0]}};
         for(int i=0;      i<VLEN/64;i++) o_sum_1a[i] <= {{       {65{i_issgn_0a && i_sized_src1_0a[i][63]}}, i_sized_src1_0a[i][63:0]} * {{65{i_issgnsrc2_0a && i_sized_src2_0a[i][63]}}, i_sized_src2_0a[i][63:0]}};
      end
   end
   
endmodule

