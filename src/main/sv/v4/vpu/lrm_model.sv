module lrm_model
(
   input  logic         clk,
   input  logic         reset_n,
   input  logic         load_valid,
   input  logic [511:0] load_data,
   input  logic [ 33:0] load_seq_id,
   input  logic [  4:0] sb_vd,
   input  logic [  2:0] stride,     // 0:1, 1:2, 2:4, 3:RSVD, 4:-1, 5:-2, 6:-4, 7:RSVD
   input  logic [  1:0] eew,        // 0:1B, 1:2B, 2:4B, 3:8B
   input  logic [  2:0] emul,       // 0:1, 1:2, 2:4, 3:8
   input  logic [  2:0] load_seg,   // number of segments (0:1, 1:2, 2:3... n:n+1, 7:8)

   output logic [511:0] packed_data [7:0],
   output logic [ 63:0] byte_en     [7:0]
);

   localparam P1=0;
   localparam P2=1;
   localparam P4=2;
   localparam N1=4;
   localparam N2=5;
   localparam N4=6;

   // in segment packing mode, only elements of a segment are packed together (not multiple elements)
   logic segment_packing;

   logic [ 4:0] v_reg;
   logic [10:0] el_id;
   logic [ 5:0] el_off;
   logic [ 6:0] el_count;
   logic [ 4:0] sb_id;
   logic [ 2:0] seg_id;

   assign {sb_id, el_count, el_off, el_id, v_reg} = load_seq_id;
   // assign seg_id = ((v_reg - sb_vd) & ((5'h1 << emul) - 1'b1));
   assign seg_id = (v_reg - sb_vd) >> emul;
   assign segment_packing = (load_seg != 3'h0);

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


   // Pack element together (not required for segmented since their elements are not packed)
   always_comb begin
      if (stride inside {P2, N2} && !segment_packing) begin
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
      if (stride inside {P4, N4} && !segment_packing) begin
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
   logic [11:0] seg_shift_amt;
   assign id_shift_amt = {el_id, 3'h0} << eew;
   assign post_id_adj = post_stride_adj << id_shift_amt;

   always_comb begin
      for (int i=0; i<8; i++) begin
         packed_data[i] = '0;
      end

      for (int i=0; i<8; i++) begin
         if ((i >= seg_id) && (i <= load_seg)) begin
            seg_shift_amt = {i-seg_id, 3'h0} << eew;
            packed_data[i] = post_id_adj >> seg_shift_amt;
         end
      end
   end

   // Generate byte enable signals
   always_comb begin
      // initialize all byte_en to 0
      for (int i=0; i<8; i++) begin
         byte_en[i] = '0;
      end

      // segment packing mode: search vertically along column el_id
      if (segment_packing) begin
         for (int i=0; i<8; i++) begin
            if ((i >= seg_id) && (i-seg_id < el_count)) begin
               if (eew == 2'h3) begin
                  byte_en[i][(el_id)*8+:8] = 8'hff;
               end else
               if (eew == 2'h2) begin
                  byte_en[i][(el_id)*4+:4] = 4'hf;
               end else
               if (eew == 2'h1) begin
                  byte_en[i][(el_id)*2+:2] = 2'h3;
               end else begin
                  byte_en[i][(el_id)*1+:1] = 1'b1;
               end
            end
         end
      end

      // non-segment packing mode: search horizontally along row 0
      else begin
         if (eew == 2'h3) begin
            for (int i=0; i<8; i++) begin
               if (i < el_count) begin
                  byte_en[0][(el_id+i)*8+:8] = 8'hff;
               end
            end
         end else
         if (eew == 2'h2) begin
            for (int i=0; i<16; i++) begin
               if (i < el_count) begin
                  byte_en[0][(el_id+i)*4+:4] = 4'hf;
               end
            end
         end else
         if (eew == 2'h1) begin
            for (int i=0; i<32; i++) begin
               if (i < el_count) begin
                  byte_en[0][(el_id+i)*2+:2] = 2'h3;
               end
            end
         end else begin
            for (int i=0; i<64; i++) begin
               if (i < el_count) begin
                  byte_en[0][(el_id+i)*1+:1] = 1'b1;
               end
            end
         end
      end
   end

endmodule
