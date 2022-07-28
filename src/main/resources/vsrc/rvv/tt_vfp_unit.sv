// See LICENSE.TT for license details.
`include "briscv_defines.h"
`include "tt_briscv_pkg.vh"

module tt_vfp_unit 
# (parameter NUM_LANE=2)
(
    input i_clk, i_reset_n,

    // Handshaking
    input                                  i_id_vfp_ex0_rts,
    input tt_briscv_pkg::vec_autogen_s     i_id_vfp_autogen,

    input  logic [2:0][NUM_LANE-1:0][63:0] i_rddata,
    input  logic          [NUM_LANE*8-1:0] i_vm0,
    input  logic [1:0]                     i_sew,
    input  logic [1:0]                     i_xrm,
    input  logic [2:0]                     i_frm,
    input  logic                     [5:0] i_funct6, // Used for decoding compare instructions
    input  logic                     [4:0] i_vs1,    // Used for decoding convert instructions
    input  logic [2:0]                     i_lmul_cnt,

    output logic         [NUM_LANE*64-1:0] o_result,
    output logic                           o_result_valid,
    output tt_briscv_pkg::csr_fp_exc       o_result_exc,
    output logic                           o_result_hole_valid,
    output logic                           o_result_ooo_data_valid,
    output logic         [NUM_LANE*64-1:0] o_result_ooo_data,
    output tt_briscv_pkg::csr_fp_exc       o_result_ooo_exc

);

`include "HardFloat_localFuncs.vi"
`include "HardFloat_consts.vi"
import tt_briscv_pkg::*;

   // Stage 0
   // Encode Floating Numbers
   logic [2:0][NUM_LANE-1:0]     [63:0] encoder_data_in_s0;
   logic [2:0]                   [ 1:0] encoder_data_sel_s0;     // 0: inline ; 1: reserved; 2: widen_lo; 3: widen_hi
   logic [2:0][NUM_LANE-1:0][3:0][16:0] encoded_f16_data_s0;
   logic [2:0][NUM_LANE-1:0][1:0][32:0] encoded_f32_data_s0;
   logic [2:0][NUM_LANE-1:0][0:0][64:0] encoded_f64_data_s0;
   logic      [NUM_LANE-1:0]     [63:0] int_data_s0;
   logic                                ctl_wdeop_s0;

   // Stage 1
   // - Converter
   // - 
   logic                      ctl_valid_s1;
   vec_autogen_s              ctl_autogen_s1;
   logic                      ctl_wdeop_s1;
   logic               [ 1:0] ctl_mode_s1;   // 0: cvt, 1: add/sub, 2: mul, 3: fma
   logic               [ 1:0] ctl_sew_s1;
   logic     [NUM_LANE*8-1:0] ctl_vm0;
   logic [2:0]                ctl_lmul_cnt_s1;
   logic [ 2:0]               ctl_roundingMode_s1;
   logic               [ 5:0] ctl_funct6_s1; // Used for decoding compare instructions

   logic [2:0][NUM_LANE-1:0][3:0][16:0] lane_f16_data_s1;
   logic [2:0][NUM_LANE-1:0][1:0][32:0] lane_f32_data_s1;
   logic [2:0][NUM_LANE-1:0][0:0][64:0] lane_f64_data_s1;
   logic      [NUM_LANE-1:0]     [63:0] lane_int_data_s1;

   logic [NUM_LANE-1:0][ 3:0] lane_valid_s1;
   logic                      lane_signedIn_s1;
   logic                      lane_signedOut_s1;
   logic               [ 1:0] lane_fma_op_s1; // 0: madd_s1; 1: msub_s1; 2: nmsub_s1; 3: nmadd
   logic                      lane_cmp_sig_s1;
   logic                      lane_sel_op_s1; // 0: min, 1: max
   logic               [ 4:0] lane_vs1_s1;    // Used for decoding convert instructions

   logic [NUM_LANE-1:0][63:0] lane_res_s1;
   logic [NUM_LANE-1:0][ 4:0] lane_exc_s1;

   // Stage 2
   logic                      ctl_valid_s2;
   vec_autogen_s              ctl_autogen_s2;
   logic               [ 1:0] ctl_sew_s2;

   logic [NUM_LANE*64/2-1:0]  nrwop_res_s2;
   logic [NUM_LANE-1:0][ 4:0] nrwop_exc_s2;

   logic [NUM_LANE-1:0][63:0] lane_res_s2;
   logic [NUM_LANE-1:0][ 4:0] lane_exc_s2;

   generate
      for (genvar i=0; i<3; i++) begin: encoder
         tt_vfp_encoder #(.NUM_LANE(NUM_LANE))
         encoder
         (
            .data_in     (encoder_data_in_s0 [i]),
            .data_sel    (encoder_data_sel_s0[i]),
            .f16_data_out(encoded_f16_data_s0[i]),
            .f32_data_out(encoded_f32_data_s0[i]),
            .f64_data_out(encoded_f64_data_s0[i])
         );
   
         assign encoder_data_in_s0 [i] = i_rddata[i];
      end
   endgenerate

   always_comb begin
      case (i_funct6)
         // vfwadd.vv
         6'b110000,
         // vfwsub.vv
         6'b110010: begin
            encoder_data_sel_s0[0] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
            encoder_data_sel_s0[1] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
            encoder_data_sel_s0[2] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
            ctl_wdeop_s0           = 1'b1;
         end
         // vfwadd.wv
         6'b110100,
         // vfwsub.wv
         6'b110110: begin
            encoder_data_sel_s0[0] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
            encoder_data_sel_s0[1] = 2'h0;
            encoder_data_sel_s0[2] = 2'h0;
            ctl_wdeop_s0           = 1'b1;
         end
         // vfwmul.vv
         6'b111000: begin
            encoder_data_sel_s0[0] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
            encoder_data_sel_s0[1] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
            encoder_data_sel_s0[2] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
            ctl_wdeop_s0           = 1'b1;
         end
         // vfwmacc.vv
         6'b111100,
         // vfwnmacc.vv
         6'b111101,
         // vfwmsac.vv
         6'b111110,
         // vfwnmsac
         6'b111111: begin
            encoder_data_sel_s0[0] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
            encoder_data_sel_s0[1] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
            encoder_data_sel_s0[2] = 2'h0;
            ctl_wdeop_s0           = 1'b1;
         end
         // vfwredusum
         6'b110001,
         // vfwredosum
         6'b110011: begin
            encoder_data_sel_s0[0] = 2'h0;
            encoder_data_sel_s0[1] = 2'h0;
            encoder_data_sel_s0[2] = (i_sew == 2'h1) ? (((i_id_vfp_autogen.replay_cnt - 1) & (NUM_LANE*2)) ? 2'h2 : 2'h3) :
                                     (i_sew == 2'h2) ? (((i_id_vfp_autogen.replay_cnt - 1) & (NUM_LANE*1)) ? 2'h2 : 2'h3) :
                                                       (((i_id_vfp_autogen.replay_cnt - 1) & (NUM_LANE/2)) ? 2'h2 : 2'h3);
            ctl_wdeop_s0           = 1'b1;
         end
         // VFUNARY0
         6'b010010: begin
            case (i_vs1)
               5'b01000,
               5'b01001,
               5'b01010,
               5'b01011,
               5'b01100,
               5'b01110,
               5'b01111: begin
                  encoder_data_sel_s0[0] = 2'h0;
                  encoder_data_sel_s0[1] = i_id_vfp_autogen.replay_cnt[0] ? 2'h3 : 2'h2;
                  encoder_data_sel_s0[2] = 2'h0;
                  ctl_wdeop_s0           = 1'b1;
               end
               default: begin
                  encoder_data_sel_s0[0] = 2'h0;
                  encoder_data_sel_s0[1] = 2'h0;
                  encoder_data_sel_s0[2] = 2'h0;
                  ctl_wdeop_s0           = 1'b0;
               end
            endcase
         end
         default: begin
            encoder_data_sel_s0[0] = 2'h0;
            encoder_data_sel_s0[1] = 2'h0;
            encoder_data_sel_s0[2] = 2'h0;
            ctl_wdeop_s0           = 1'b0;
         end
      endcase 
   end

   always_comb begin
      for (int i=0; i<NUM_LANE; i++) begin
         // inline
         if (encoder_data_sel_s0[1] == 2'h0) begin
            int_data_s0[i] = i_rddata[1][i];
         end else
         // widen_lo
         if (encoder_data_sel_s0[1] == 2'h2) begin
            int_data_s0[i][31: 0] = i_rddata[1][i/2][32*(i%2)+:32];
            int_data_s0[i][63:32] = '0;
         end else
         // widen_hi
         if (encoder_data_sel_s0[1] == 2'h3) begin
            int_data_s0[i][31: 0] = i_rddata[1][(NUM_LANE+i)/2][32*(i%2)+:32];
            int_data_s0[i][63:32] = '0;
         end else begin
            int_data_s0[i] = '0;
         end
      end
   end

   // Stage 1
   always_ff @(posedge i_clk, negedge i_reset_n) begin
      if (~i_reset_n) begin
         ctl_valid_s1 <= '0;
      end else begin
         ctl_valid_s1 <= i_id_vfp_ex0_rts; // FIXME
      end
   end

   always_ff @(posedge i_clk) begin
      if (i_id_vfp_ex0_rts) begin
         ctl_autogen_s1       <=  i_id_vfp_autogen;
         ctl_sew_s1           <=  ctl_wdeop_s0 ? i_sew + 2'h1 :
                                                 i_sew;
         ctl_lmul_cnt_s1      <=  i_lmul_cnt;
         ctl_wdeop_s1         <=  ctl_wdeop_s0;
         case (i_funct6)
            6'b000000: ctl_mode_s1 <= 2'h1; // vfadd
            6'b000010: ctl_mode_s1 <= 2'h1; // vfsub
            6'b100100: ctl_mode_s1 <= 2'h2; // vfmul
            6'b100111: ctl_mode_s1 <= 2'h1; // vfrsub
            6'b101000: ctl_mode_s1 <= 2'h3; // vfmadd
            6'b101001: ctl_mode_s1 <= 2'h3; // vfnmadd
            6'b101010: ctl_mode_s1 <= 2'h3; // vfmsub
            6'b101011: ctl_mode_s1 <= 2'h3; // vfnmsub
            6'b101100: ctl_mode_s1 <= 2'h3; // vfmacc
            6'b101101: ctl_mode_s1 <= 2'h3; // vfnmacc
            6'b101110: ctl_mode_s1 <= 2'h3; // vfmsac
            6'b101111: ctl_mode_s1 <= 2'h3; // vfnmsac
            6'b110000: ctl_mode_s1 <= 2'h1; // vfwadd
            6'b110010: ctl_mode_s1 <= 2'h1; // vfwsub
            6'b110100: ctl_mode_s1 <= 2'h1; // vfwadd.w
            6'b110110: ctl_mode_s1 <= 2'h1; // vfwsub.w
            6'b111000: ctl_mode_s1 <= 2'h2; // vfwmul
            6'b111100: ctl_mode_s1 <= 2'h3; // vfwmacc
            6'b111101: ctl_mode_s1 <= 2'h3; // vfwnmacc
            6'b111110: ctl_mode_s1 <= 2'h3; // vfwmsac
            6'b111111: ctl_mode_s1 <= 2'h3; // vfwmsac
            default  : ctl_mode_s1 <= 2'h0;
         endcase
         // Override rounding mode
         if (i_funct6 == 6'b010010) begin
            // Round to zero
            if (i_vs1 inside {5'b00110, 5'b00111, 5'b01110, 5'b01111, 5'b10110, 5'b10111}) begin
               ctl_roundingMode_s1 <= `round_minMag;
            end else
            // Round to odd
            if (i_vs1 inside {5'b10101}) begin
               ctl_roundingMode_s1 <= `round_odd;
            end else begin
               ctl_roundingMode_s1 <= (i_frm == 3'h0) ? `round_near_even :
                                      (i_frm == 3'h1) ? `round_minMag    :
                                      (i_frm == 3'h2) ? `round_min       :
                                      (i_frm == 3'h3) ? `round_max       :
                                                        `round_near_maxMag;
            end
         end else begin
            ctl_roundingMode_s1 <= (i_frm == 3'h0) ? `round_near_even :
                                   (i_frm == 3'h1) ? `round_minMag    :
                                   (i_frm == 3'h2) ? `round_min       :
                                   (i_frm == 3'h3) ? `round_max       :
                                                     `round_near_maxMag;
         end
         ctl_funct6_s1        <=  i_funct6;
         if ( ctl_wdeop_s0                  &&
             !ctl_autogen_s1.reductop       &&
             i_id_vfp_autogen.replay_cnt[0]   ) begin
            ctl_vm0              <= (i_sew == 2'h0) ? i_vm0[NUM_LANE*4+:NUM_LANE*4] :
                                    (i_sew == 2'h1) ? i_vm0[NUM_LANE*2+:NUM_LANE*2] :
                                                      i_vm0[NUM_LANE*1+:NUM_LANE*1];
         end else begin
            ctl_vm0              <=  i_vm0;
         end

         lane_signedIn_s1     <=  i_vs1 inside {5'b00011, 5'b01011, 5'b10011};
         lane_signedOut_s1    <=  i_vs1 inside {5'b00001, 5'b00111, 5'b01001, 5'b01111, 5'b10001, 5'b10111};
         lane_fma_op_s1       <= {i_id_vfp_autogen.rd_neg0,
                                  i_id_vfp_autogen.rd_neg2};

         lane_cmp_sig_s1      <=  i_funct6 inside {6'b011001, 6'b011011, 6'b011101, 6'b011111};
         lane_sel_op_s1       <=  i_funct6 inside {6'b000110, 6'b000111};
         lane_vs1_s1          <=  i_vs1;
         lane_f16_data_s1     <=  encoded_f16_data_s0;
         lane_f32_data_s1     <=  encoded_f32_data_s0;
         lane_f64_data_s1     <=  encoded_f64_data_s0;
         lane_int_data_s1     <=  int_data_s0;
      end
   end

   generate
      for (genvar i=0; i<NUM_LANE; i++) begin: lane
         assign lane_valid_s1[i] = (ctl_sew_s1 <= 2'h1) ? {4{ctl_valid_s1}} &        ctl_vm0[4*i+:4]  :
                                   (ctl_sew_s1 == 2'h2) ? {4{ctl_valid_s1}} & {2'h0, ctl_vm0[2*i+:2]} :
                                                          {4{ctl_valid_s1}} & {3'h0, ctl_vm0[1*i+:1]};

         tt_vfp_lane lane
         (
            // Inputs
            .i_clk         (i_clk                 ),
            .i_reset_n     (i_reset_n             ),
            .i_valid       (lane_valid_s1      [i]),
            .i_mode        (ctl_mode_s1           ),
            .i_signedIn    (lane_signedIn_s1      ),
            .i_signedOut   (lane_signedOut_s1     ),
            .i_fma_op      (lane_fma_op_s1        ),
            .i_roundingMode(ctl_roundingMode_s1   ),
            .i_cmp_sig     (lane_cmp_sig_s1       ),
            .i_sel_op      (lane_sel_op_s1        ),
            .i_sew         (ctl_sew_s1            ),
            .i_funct6      (ctl_funct6_s1         ),
            .i_vs1         (lane_vs1_s1           ),
            .i_f16_a       (lane_f16_data_s1[0][i]),
            .i_f16_b       (lane_f16_data_s1[1][i]),
            .i_f16_c       (lane_f16_data_s1[2][i]),
            .i_f32_a       (lane_f32_data_s1[0][i]),
            .i_f32_b       (lane_f32_data_s1[1][i]),
            .i_f32_c       (lane_f32_data_s1[2][i]),
            .i_f64_a       (lane_f64_data_s1[0][i]),
            .i_f64_b       (lane_f64_data_s1[1][i]),
            .i_f64_c       (lane_f64_data_s1[2][i]),
            .i_int         (lane_int_data_s1   [i]),
            // Outputs
            .o_res_s1      (lane_res_s1        [i]),
            .o_exc_s1      (lane_exc_s1        [i]),
            .o_res_s2      (lane_res_s2        [i]),
            .o_exc_s2      (lane_exc_s2        [i])
         );
      end
   endgenerate

   always_comb begin
      o_result_exc = '0;
   
      for (int i=0; i<NUM_LANE; i++) begin
         o_result_exc.fpNV |= lane_exc_s1[i][4];
         o_result_exc.fpDZ |= lane_exc_s1[i][3];
         o_result_exc.fpOF |= lane_exc_s1[i][2];
         o_result_exc.fpUF |= lane_exc_s1[i][1];
         o_result_exc.fpNX |= lane_exc_s1[i][0];
      end
   end

   assign o_result_valid      = ctl_valid_s1 && ( ctl_mode_s1  == 2'h0 &&
                                                 !ctl_autogen_s1.nrwop   );
   assign o_result_hole_valid = ctl_valid_s1 && ( ctl_mode_s1  != 2'h0 ||
                                                  ctl_autogen_s1.nrwop ||
                                                  ctl_autogen_s1.reductop);
   
   always_comb begin
      o_result = '0;

      // vfm*
      if (ctl_autogen_s1.mask_only) begin
         for (int i=0; i<NUM_LANE; i++) begin
            // f16
            if (ctl_sew_s1 == 2'h1) begin
               o_result[ctl_lmul_cnt_s1*(NUM_LANE*4)+4*i+:4] = lane_res_s1[i][3:0];
            end else
            // f32
            if (ctl_sew_s1 == 2'h2) begin
               o_result[ctl_lmul_cnt_s1*(NUM_LANE*2)+2*i+:2] = lane_res_s1[i][1:0];
            end else begin
            // f64
               o_result[ctl_lmul_cnt_s1*(NUM_LANE*1)+1*i+:1] = lane_res_s1[i][  0];
            end
         end
      end else begin
         o_result = lane_res_s1;
      end
   end

   // Reduction Units
   logic        red_sel_s1;
   logic [ 1:0] red_op_s1;

   logic                        f16_red_valid_s1;
   logic                        f16_red_en_s1;
   logic [16:0]                 f16_red_a_s1;
   logic [$clog2(NUM_LANE)+1:0] f16_red_c_idx_s1;
   logic [16:0]                 f16_red_c_s1;
   logic [15:0]                 f16_red_res_s2;
   logic [ 4:0]                 f16_red_exc_s2;

   tt_vfp_red #(.expWidth( 5),
                .sigWidth(11) )
   red_unit_f16
   (
      .i_clk         (i_clk),
      .i_reset_n     (i_reset_n),
      .i_valid       (f16_red_valid_s1),
      .i_en          (f16_red_en_s1),
      .i_sel         (red_sel_s1),
      .i_op          (red_op_s1),
      .i_roundingMode(ctl_roundingMode_s1),
      .i_a           (f16_red_a_s1),
      .i_c           (f16_red_c_s1),
      .o_res         (f16_red_res_s2),
      .o_exc         (f16_red_exc_s2)
   );
   
   assign f16_red_valid_s1 = ctl_valid_s1            &&
                             ctl_sew_s1 == 2'h1      &&
                             ctl_autogen_s1.reductop;
   assign f16_red_en_s1    = ctl_valid_s1 &&
                             ctl_vm0[f16_red_c_idx_s1];
   assign f16_red_c_idx_s1 = ($clog2(NUM_LANE)+2)'(0-ctl_autogen_s1.replay_cnt[$clog2(NUM_LANE)+1:0]);
   assign f16_red_a_s1     = lane_f16_data_s1[0][0][0];
   assign f16_red_c_s1     = lane_f16_data_s1[2][f16_red_c_idx_s1/4][f16_red_c_idx_s1%4];

   logic                        f32_red_valid_s1;
   logic                        f32_red_en_s1;
   logic [32:0]                 f32_red_a_s1;
   logic [$clog2(NUM_LANE)+0:0] f32_red_c_idx_s1;
   logic [32:0]                 f32_red_c_s1;
   logic [31:0]                 f32_red_res_s2;
   logic [ 4:0]                 f32_red_exc_s2;

   tt_vfp_red #(.expWidth( 8),
                .sigWidth(24) )
   red_unit_f32
   (
      .i_clk         (i_clk),
      .i_reset_n     (i_reset_n),
      .i_valid       (f32_red_valid_s1),
      .i_en          (f32_red_en_s1),
      .i_sel         (red_sel_s1),
      .i_op          (red_op_s1),
      .i_roundingMode(ctl_roundingMode_s1),
      .i_a           (f32_red_a_s1),
      .i_c           (f32_red_c_s1),
      .o_res         (f32_red_res_s2),
      .o_exc         (f32_red_exc_s2)
   );

   assign f32_red_valid_s1 =   ctl_valid_s1                                    &&
                             ((ctl_sew_s1 == 2'h2                ) ||
                              (ctl_sew_s1 == 2'h1 && ctl_wdeop_s1)   ) &&
                               ctl_autogen_s1.reductop;
   assign f32_red_en_s1    =   ctl_valid_s1 &&
                              (ctl_wdeop_s1 ? ctl_vm0[f16_red_c_idx_s1] :
                                              ctl_vm0[f32_red_c_idx_s1]  );
   assign f32_red_c_idx_s1 = ($clog2(NUM_LANE)+1)'(0-ctl_autogen_s1.replay_cnt[$clog2(NUM_LANE)+0:0]);
   assign f32_red_a_s1     = lane_f32_data_s1[0][0][0];
   assign f32_red_c_s1     = lane_f32_data_s1[2][f32_red_c_idx_s1/2][f32_red_c_idx_s1%2];

   logic                        f64_red_valid_s1;
   logic                        f64_red_en_s1;
   logic [64:0]                 f64_red_a_s1;
   logic [$clog2(NUM_LANE)-1:0] f64_red_c_idx_s1;
   logic [64:0]                 f64_red_c_s1;
   logic [63:0]                 f64_red_res_s2;
   logic [ 4:0]                 f64_red_exc_s2;

   tt_vfp_red #(.expWidth(11),
                .sigWidth(53) )
   red_unit_f64
   (
      .i_clk         (i_clk),
      .i_reset_n     (i_reset_n),
      .i_valid       (f64_red_valid_s1),
      .i_en          (f64_red_en_s1),
      .i_sel         (red_sel_s1),
      .i_op          (red_op_s1),
      .i_roundingMode(ctl_roundingMode_s1),
      .i_a           (f64_red_a_s1),
      .i_c           (f64_red_c_s1),
      .o_res         (f64_red_res_s2),
      .o_exc         (f64_red_exc_s2)
   );

   assign f64_red_valid_s1 =   ctl_valid_s1                                    &&
                             ((ctl_sew_s1 == 2'h3                ) ||
                              (ctl_sew_s1 == 2'h2 && ctl_wdeop_s1)   ) &&
                               ctl_autogen_s1.reductop;
   assign f64_red_en_s1    =   ctl_valid_s1 &&
                              (ctl_wdeop_s1 ? ctl_vm0[f32_red_c_idx_s1] :
                                              ctl_vm0[f64_red_c_idx_s1]  );
   assign f64_red_c_idx_s1 = ($clog2(NUM_LANE)+0)'(0-ctl_autogen_s1.replay_cnt[$clog2(NUM_LANE)-1:0]);
   assign f64_red_a_s1     = lane_f64_data_s1[0][0][0];
   assign f64_red_c_s1     = lane_f64_data_s1[2][f64_red_c_idx_s1/1][f64_red_c_idx_s1%1];

   assign red_sel_s1       = ctl_valid_s1            &&
                             ctl_autogen_s1.replay_cnt != '0;
   always_comb begin
      case (ctl_funct6_s1)
         6'b000001: red_op_s1 = 2'h0;
         6'b000011: red_op_s1 = 2'h0;
         6'b000101: red_op_s1 = 2'h2;
         6'b000111: red_op_s1 = 2'h3;
         6'b110001: red_op_s1 = 2'h0;
         6'b110011: red_op_s1 = 2'h0;
         default  : red_op_s1 = 2'h1;
      endcase
   end

   // Stage 2
   always_ff @(posedge i_clk, negedge i_reset_n) begin
      if (~i_reset_n) begin
         ctl_valid_s2 <= '0;
      end else begin
         ctl_valid_s2 <= ctl_valid_s1;
      end
   end

   always_ff @(posedge i_clk) begin
      if (ctl_valid_s1) begin
         ctl_autogen_s2 <= ctl_autogen_s1;
         ctl_sew_s2     <= ctl_sew_s1;
      end

      if ( ctl_valid_s1                 &&
           ctl_autogen_s1.nrwop         &&
          !ctl_autogen_s1.replay_cnt[0]   ) begin
         for (int i=0; i<NUM_LANE; i++) begin
            nrwop_res_s2[32*i+:32] <= lane_res_s1[i][31:0];
            nrwop_exc_s2[       i] <= lane_exc_s1[i];
         end
      end
   end

   assign o_result_ooo_data_valid = (ctl_valid_s2 && ( ctl_autogen_s2.vfp_mad_type_inst         ||
                                                      (ctl_autogen_s2.onecycle_iterate      &&
                                                       ctl_autogen_s2.replay_cnt    == 8'h1   )   )) ||
                                    (ctl_valid_s2 && ( ctl_autogen_s2.nrwop                 &&          // For narrowing op, flush the result 
                                                       ctl_autogen_s2.replay_cnt[0] == 1'b0 &&          // With the first half when
                                                       ctl_autogen_s2.onecycle_nrwop          )    ) || // LMUL < 1
                                    (ctl_valid_s1 && ( ctl_autogen_s1.nrwop                  &&          // Otherwise, need to check
                                                       ctl_autogen_s1.replay_cnt[0] == 1'b1    )    );   // if the upper portion is ready

   always_comb begin
      if (ctl_autogen_s2.reductop) begin
         
         o_result_ooo_data = (ctl_sew_s2 == 2'h1) ? {'0, f16_red_res_s2} :
                             (ctl_sew_s2 == 2'h2) ? {'0, f32_red_res_s2} :
                                                    {'0, f64_red_res_s2};
      end else
      if (ctl_autogen_s2.nrwop) begin
         o_result_ooo_data[NUM_LANE*64/2-1:0] = nrwop_res_s2;
         for (int i=0; i<NUM_LANE; i++) begin
            o_result_ooo_data[NUM_LANE*64/2 + 32*i +:32] = lane_res_s1[i][31:0];
         end
      end else begin
         o_result_ooo_data = lane_res_s2;
      end
   end

   always_comb begin
      o_result_ooo_exc = '0;
   
      for (int i=0; i<NUM_LANE; i++) begin
         o_result_ooo_exc.fpNV |= lane_exc_s2[i][4];
         o_result_ooo_exc.fpDZ |= lane_exc_s2[i][3];
         o_result_ooo_exc.fpOF |= lane_exc_s2[i][2];
         o_result_ooo_exc.fpUF |= lane_exc_s2[i][1];
         o_result_ooo_exc.fpNX |= lane_exc_s2[i][0];
         if (ctl_autogen_s2.nrwop) begin
            o_result_ooo_exc.fpNV |= lane_exc_s1[i][4] || nrwop_exc_s2[i][4];
            o_result_ooo_exc.fpDZ |= lane_exc_s1[i][3] || nrwop_exc_s2[i][3];
            o_result_ooo_exc.fpOF |= lane_exc_s1[i][2] || nrwop_exc_s2[i][2];
            o_result_ooo_exc.fpUF |= lane_exc_s1[i][1] || nrwop_exc_s2[i][1];
            o_result_ooo_exc.fpNX |= lane_exc_s1[i][0] || nrwop_exc_s2[i][0];
         end
      end

      o_result_ooo_exc.fpNV |= f16_red_exc_s2[4] ||
                               f32_red_exc_s2[4] ||
                               f64_red_exc_s2[4];
      o_result_ooo_exc.fpDZ |= f16_red_exc_s2[3] ||
                               f32_red_exc_s2[3] ||
                               f64_red_exc_s2[3];
      o_result_ooo_exc.fpOF |= f16_red_exc_s2[2] ||
                               f32_red_exc_s2[2] ||
                               f64_red_exc_s2[2];
      o_result_ooo_exc.fpUF |= f16_red_exc_s2[1] ||
                               f32_red_exc_s2[1] ||
                               f64_red_exc_s2[1];
      o_result_ooo_exc.fpNX |= f16_red_exc_s2[0] ||
                               f32_red_exc_s2[0] ||
                               f64_red_exc_s2[0];
   end

endmodule
