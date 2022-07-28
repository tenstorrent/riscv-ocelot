// See LICENSE.TT for license details.
module tt_vec_regfile #(parameter
   VLEN=128
)
(
		      input [2:0] 		i_rden_0a,
		      input    		        i_wren_0a, 
		      input 			i_clk,
		      input 			i_reset_n,
		      input [2:0][4:0] 		i_rdaddr_0a, 
		      input [4:0] 		i_wraddr_0a, 
		      input [VLEN-1:0] 		i_wrdata_0a,
 		  
		      output logic [2:0][VLEN-1:0] o_rddata_0a,
		      output logic [VLEN-1:0] 	o_dstmask_0a,
		      output logic [VLEN-1:0] 	o_vm0_0a
		      );

   logic [31:0][VLEN-1:0] 				mem;
   logic [2:0][VLEN-1:0] 				rddata_0a;
 				
   always_comb
     for(int i=0; i<3; i++) begin
	rddata_0a[i]   = mem[i_rdaddr_0a[i][4:0]][VLEN-1:0];
     end
   

   logic [4:0] 	 wraddr; 
   assign wraddr = i_wraddr_0a;

   always_ff@(posedge i_clk, negedge i_reset_n) begin
      if (~i_reset_n) begin
           mem <= '0;
      end else begin
         if(i_wren_0a) begin
           mem[wraddr] <= i_wrdata_0a;
         end
      end
   end
   
   
   assign o_rddata_0a = rddata_0a;
   //mask is always v0.
   assign o_vm0_0a[VLEN-1:0]    = mem[0][VLEN-1:0];
   
   assign o_dstmask_0a[VLEN-1:0] = mem[i_wraddr_0a[4:0]];
   
endmodule
