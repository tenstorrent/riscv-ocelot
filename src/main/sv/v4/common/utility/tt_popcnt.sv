// See LICENSE.TT for license details.
//Find First Set, with integrated data muxing and encode.
//This needs to be a complete binary tree
module tt_popcnt #(                                //Direction of Priority L2H=1 finds lsb's, L2H=0 finds msb
		     parameter WIDTH      = 8,              //Number of inputs.
		     parameter SIZE       = $clog2(WIDTH)   //Log2 Number of inputs
		     )              //Width of data  
		  
   
   (input  [WIDTH-1:0]                 req_in, 
    output logic [SIZE:0]              req_sum);
   
   logic [SIZE:0][WIDTH-1:0][SIZE:0]   sum;
   
   assign req_sum  = sum[0][0];
      
      
   //Initialize leaf's of tree to data_in
   always_comb begin
      sum = '0;
      for(int i=0;i<WIDTH;i++) begin
         sum[SIZE][i][0] = req_in[i];
      end
      
      for(int LVL=SIZE-1; LVL>=0; LVL--)begin
         for(int NODE=0; NODE < 1 << LVL; NODE++) begin
	    
            sum[LVL][NODE] =  sum[LVL+1][2*NODE + 1] 
	                    + sum[LVL+1][2*NODE + 0];
         end
         //$display("LVL %d, NODE %d, MAX_BKT_AT_LVL %d, data_mux %h",LVL,2*NODE+!DIR_L2H,MAX_BKT_AT_LVL,req_mux[LVL][NODE][bkt]);
      end
   end
endmodule  
