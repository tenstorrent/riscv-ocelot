module lrm
(
   input  logic         clk,
   input  logic         reset_n,
   input  logic         load_valid,
   input  logic [511:0] load_data,
   input  logic [ 33:0] load_seq_id,
   input  logic [  2:0] stride,     // 0:1, 1:2, 2:4, 3:RSVD, 4:-1, 5:-2, 6:-4, 7:RSVD
   input  logic [  1:0] eew,        // 0:1B, 1:2B, 2:4B, 3:8B

   output logic [511:0] packed_data,
   output logic [ 63:0] byte_en
);
   localparam P1=0;
   localparam P2=1;
   localparam P4=2;
   localparam N1=4;
   localparam N2=5;
   localparam N4=6;

   logic [ 4:0] v_reg;
   logic [10:0] el_id;
   logic [ 5:0] el_off;
   logic [ 6:0] el_count;
   logic [ 4:0] sb_id;

   assign {sb_id, el_count, el_off, el_id, v_reg} = load_seq_id;

   logic [511:0] post_sign_adj;   // Reverse negative stride
   logic [511:0] post_off_adj;    // Aligh first element to byte 0
   logic [511:0] post_stride_adj; // Pack element together
   logic [511:0] post_id_adj;     // Shift to final position

   // Reverse negative stride
   always_comb begin
      if (stride inside {N1, N2, N4}) begin
         if (eew == 2'h0) begin
            for (int i=0; i<64; i++) begin
               post_sign_adj[i*8+:8] = load_data[(63-i)*8+:8];
            end
         end
         else if (eew == 2'h1) begin
            for (int i=0; i<32; i++) begin
               post_sign_adj[i*16+:16] = load_data[(31-i)*16+:16];
            end
         end
         else if (eew == 2'h2) begin
            for (int i=0; i<16; i++) begin
               post_sign_adj[i*32+:32] = load_data[(15-i)*32+:32];
            end
         end else begin
            for (int i=0; i<8; i++) begin
               post_sign_adj[i*64+:64] = load_data[(7-i)*64+:64];
            end
         end
      end else begin
         post_sign_adj = load_data;
      end
   end

   // Aligh first element to byte 0
   logic [11:0] off_shift_amt;
   assign off_shift_amt = {el_off, 3'h0} << eew;

   assign post_off_adj = post_sign_adj >> off_shift_amt;


   // Pack element together
   always_comb begin
      if (stride inside {P2, N2}) begin
         if (eew == 2'h0) begin
            for (int i=0; i<32; i++) begin
               post_stride_adj[i*8+:8] = post_off_adj[2*i*8+:8];
            end
            for (int i=32; i<64; i++) begin
               post_stride_adj[i*8+:8] = '0;
            end
         end
         else if (eew == 2'h1) begin
            for (int i=0; i<16; i++) begin
               post_stride_adj[i*16+:16] = post_off_adj[2*i*16+:16];
            end
            for (int i=16; i<32; i++) begin
               post_stride_adj[i*16+:16] = '0;
            end
         end
         else if (eew == 2'h2) begin
            for (int i=0; i<8; i++) begin
               post_stride_adj[i*32+:32] = post_off_adj[2*i*32+:32];
            end
            for (int i=8; i<16; i++) begin
               post_stride_adj[i*32+:32] = '0;
            end
         end else begin
            for (int i=0; i<4; i++) begin
               post_stride_adj[i*64+:64] = post_off_adj[2*i*64+:64];
            end
            for (int i=4; i<8; i++) begin
               post_stride_adj[i*64+:64] = '0;
            end
         end
      end else
      if (stride inside {P4, N4}) begin
         if (eew == 2'h0) begin
            for (int i=0; i<16; i++) begin
               post_stride_adj[i*8+:8] = post_off_adj[4*i*8+:8];
            end
            for (int i=16; i<64; i++) begin
               post_stride_adj[i*8+:8] = '0;
            end
         end
         else if (eew == 2'h1) begin
            for (int i=0; i<8; i++) begin
               post_stride_adj[i*16+:16] = post_off_adj[4*i*16+:16];
            end
            for (int i=8; i<32; i++) begin
               post_stride_adj[i*16+:16] = '0;
            end
         end
         else if (eew == 2'h2) begin
            for (int i=0; i<4; i++) begin
               post_stride_adj[i*32+:32] = post_off_adj[4*i*32+:32];
            end
            for (int i=4; i<16; i++) begin
               post_stride_adj[i*32+:32] = '0;
            end
         end else begin
            for (int i=0; i<2; i++) begin
               post_stride_adj[i*64+:64] = post_off_adj[4*i*64+:64];
            end
            for (int i=2; i<8; i++) begin
               post_stride_adj[i*64+:64] = '0;
            end
         end
      end else begin
         post_stride_adj = post_off_adj;
      end
   end

   // Shift to final position
   logic [11:0] id_shift_amt;
   assign id_shift_amt = {el_id, 3'h0} << eew;
   assign post_id_adj = post_stride_adj << id_shift_amt;

   assign packed_data = post_id_adj;

   always_comb begin
      byte_en = '0;
      if (eew == 2'h3) begin
         for (int i=0; i<8; i++) begin
            if (i < el_count) begin
               byte_en[(el_id+i)*8+:8] = 8'hff;
            end
         end
      end else
      if (eew == 2'h2) begin
         for (int i=0; i<16; i++) begin
            if (i < el_count) begin
               byte_en[(el_id+i)*4+:4] = 4'hf;
            end
         end
      end else
      if (eew == 2'h1) begin
         for (int i=0; i<32; i++) begin
            if (i < el_count) begin
               byte_en[(el_id+i)*2+:2] = 2'h3;
            end
         end
      end else begin
         for (int i=0; i<64; i++) begin
            if (i < el_count) begin
               byte_en[(el_id+i)*1+:1] = 1'b1;
            end
         end
      end
   end


endmodule
