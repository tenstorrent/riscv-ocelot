// See LICENSE.TT for license details.
//Find First Set, with integrated data muxing and encode.
//This needs to be a complete binary tree
  module tt_ffs #(parameter DIR_L2H    = 1,              //Direction of Priority L2H=1 finds lsb's, L2H=0 finds msb
		  parameter WIDTH      = 8,              //Number of inputs.
		  parameter SIZE       = $clog2(WIDTH),  //Log2 Number of inputs
		  parameter DATA_WIDTH = 4)              //Width of data  
		  
   
   (input  [WIDTH-1:0]                 req_in,
    input [WIDTH-1:0] [DATA_WIDTH-1:0] data_in,
    
    output logic 		       req_sum, 
    output logic [DATA_WIDTH-1:0]      data_out,
    output logic [WIDTH-1:0] 	       req_out,
    output logic [SIZE-1:0] 	       enc_req_out);
   
   logic [SIZE:0][WIDTH-1:0][DATA_WIDTH-1:0] data_mux;
   logic [SIZE:0][WIDTH-1:0][WIDTH-1:0]      req_mux;
   logic [SIZE:0][WIDTH-1:0][SIZE-1:0]       enc_req_mux;
   logic [SIZE:0][WIDTH-1:0] 		     sum;
    
   assign data_out    = data_mux   [0][0];
   assign req_out     = req_mux    [0][0];
   assign enc_req_out = enc_req_mux[0][0];
   assign req_sum     = sum        [0][0];  
   
   
   //Initialize leaf's of tree to data_in
   always_comb begin
      data_mux    = 'x;
      req_mux     = '0;
      sum         = '0;
      enc_req_mux = '0;
      
      for(int i=0;i<WIDTH;i++) begin
         data_mux   [SIZE][i]            = data_in[i];
         req_mux    [SIZE][i][0]         = req_in [i];
         sum        [SIZE][i]            = req_in [i];
      end
     
      for(int LVL=SIZE-1; LVL>=0; LVL--) begin
         for(int NODE=0; NODE < 1 << LVL; NODE++) begin
            //default: bring lower priority request up
            req_mux    [LVL][NODE] =  req_mux    [LVL+1][2*NODE + DIR_L2H] << (DIR_L2H << SIZE-LVL-1);
            data_mux   [LVL][NODE] =  data_mux   [LVL+1][2*NODE + DIR_L2H];
            enc_req_mux[LVL][NODE] =  enc_req_mux[LVL+1][2*NODE + DIR_L2H] | (DIR_L2H << (SIZE-1-LVL));
            sum        [LVL][NODE] =  sum        [LVL+1][2*NODE + DIR_L2H];
                     
            //If the priority side has elements, pull those up while shifting the default elements.
            if(sum[LVL+1][2*NODE+!DIR_L2H]) begin
               req_mux    [LVL][NODE] =  req_mux    [LVL+1][2*NODE + !DIR_L2H] << ((DIR_L2H ? 0 : 1) << (SIZE-LVL-1));
               data_mux   [LVL][NODE] =  data_mux   [LVL+1][2*NODE + !DIR_L2H];
               enc_req_mux[LVL][NODE] =  enc_req_mux[LVL+1][2*NODE + !DIR_L2H] | (!DIR_L2H << (SIZE-1-LVL));
               sum        [LVL][NODE] =  sum        [LVL+1][2*NODE + !DIR_L2H];
            end
         end
         //$display("LVL %d, NODE %d, MAX_BKT_AT_LVL %d, data_mux %h",LVL,2*NODE+!DIR_L2H,MAX_BKT_AT_LVL,req_mux[LVL][NODE][bkt]);
      end
   end
endmodule  
