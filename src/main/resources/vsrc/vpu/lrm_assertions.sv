module lrm_assertions
(
   input logic         clk,
   input logic         reset_n,
   input logic         load_valid,
   input logic [511:0] load_data,
   input logic [ 33:0] load_seq_id,
   input logic [  2:0] stride,     // 0:1, 1:2, 2:4, 3:RSVD, 4:-1, 5:-2, 6:-4, 7:RSVD
   input logic [  1:0] eew,        // 0:1B, 1:2B, 2:4B, 3:8B

   input logic [511:0] packed_data,
   input logic [ 63:0] byte_en
);

   logic [511:0] model_packed_data;
   logic [ 63:0] model_byte_en;
   lrm_model i_model
   (
     .clk(clk),
     .reset_n(reset_n),
     .load_valid(load_valid),
     .load_data(load_data),
     .load_seq_id(load_seq_id),
     .stride(stride),
     .eew(eew),
     .packed_data(model_packed_data),
     .byte_en(model_byte_en)
   );

   logic [63:0] chk_fail;

   always_comb begin
      for (int i=0; i<64; i++) begin
         chk_fail[i] = (model_byte_en[i] != byte_en[i]) ||
                       (model_byte_en[i] && (model_packed_data[i*8+:8] != packed_data[i*8+:8]));
      end
   end

   default disable iff (!reset_n);
   default clocking @(clk);
   endclocking
   //example_0: assert property ((load_data == 512'h03020100 && load_seq_id == {5'h0, 7'h4, 6'h0, 11'h0, 5'h0} && stride == 3'h0 && eew == 2'h0) |-> post_id_adj[31:0] == 512'h03020100);
   //example_1: assert property ((load_data[511:480] == 32'h00010203 && load_seq_id == {5'h0, 7'h4, 6'h0, 11'h0, 5'h0} && stride == 3'h4 && eew == 2'h0) |-> post_id_adj[31:0] == 512'h03020100);
   //example_2: assert property ((load_data == 512'hff03ff02ff01ff00 && load_seq_id == {5'h0, 7'h4, 6'h0, 11'h0, 5'h0} && stride == 3'h1 && eew == 2'h0) |-> post_id_adj[31:0] == 512'h03020100);
   //example_3: assert property ((load_data[511:448] == 64'h00ff01ff02ff03ff && load_seq_id == {5'h0, 7'h4, 6'h0, 11'h0, 5'h0} && stride == 3'h5 && eew == 2'h0) |-> post_id_adj[31:0] == 512'h03020100);
   //example_4: assert property ((load_data == 512'h03020100ffff && load_seq_id == {5'h0, 7'h4, 6'h2, 11'h0, 5'h0} && stride == 3'h0 && eew == 2'h0) |-> post_id_adj[31:0] == 512'h03020100);
   //example_5: assert property ((load_data == 512'h03020100 && load_seq_id == {5'h0, 7'h4, 6'h0, 11'h2, 5'h0} && stride == 3'h0 && eew == 2'h0) |-> post_id_adj[47:16] == 512'h03020100);
   //example_6: assert property ((load_data == 512'h0003ffff0002ffff0001ffff0000 && load_seq_id == {5'h0, 7'h4, 6'h0, 11'h0, 5'h0} && stride == 3'h1 && eew == 2'h1) |-> post_id_adj[63:0] == 512'h0003000200010000);
   //example_7: assert property ((load_data[511:368] == 144'hffff0000ffff0001ffff0002ffff0003ffff && load_seq_id == {5'h0, 7'h4, 6'h1, 11'h2, 5'h0} && stride == 3'h5 && eew == 2'h1) |-> post_id_adj[95:32] == 512'h0003000200010000);
   assert property (load_valid |-> ~|chk_fail);
endmodule

bind lrm lrm_assertions i_assertion (.*);
