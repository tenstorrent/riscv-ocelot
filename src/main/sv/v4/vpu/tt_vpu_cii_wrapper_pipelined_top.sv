/*************************************************************************
 *
 * Tenstorrent CONFIDENTIAL
 * __________________
 *
 *  Tenstorrent Inc.
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of Tenstorrent Inc.  The intellectual
 * and technical concepts contained
 * herein are proprietary to Tenstorrent Inc.
 * and may be covered by U.S., Canadian and Foreign Patents,
 * patents in process, and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Tenstorrent Inc.
 */
/*
  File Name: tt_vpu_cii_wrapper_pipelined_top.sv
  Author: ading
  Date Created: 7/2026
  Description:
    Top Level wrapper which implments the CII interface fifo and credit scheme and vector unit.

    Six stages, each with its OWN input pipeline register and a valid/ready
    handshake to the stage before it:

      S_FETCH  pop one issue beat out of u_iss_fifo                (f_*)
      S_DEC    decode it with u_decode_stage_id                    (d_*)
      S_REQ    walk the members, send src requests on the req chan  (q_*)
      S_DRAIN  pop the returned src beats into the staging operands (n_*)
      S_EX     u_ex_stage_id accepts the op; vecu iterates it       (x_*)
      S_WB     map a vecu result to a CII writeback beat

    Handshake convention, identical at every boundary:
      <PROD>_<CONS>_valid  the producer stage has finished its work on the op
      <CONS>_<PROD>_ready  the consumer stage register is free this cycle
      fire = valid & ready  the op moves into the consumer's register

    TWO tt_id INSTANCES. u_decode_stage_id has i_if_instrn_rts tied low, so it
    never issues and never replays: o_vec_autogen is always the fresh
    combinational decode of the S_DEC instruction, whatever u_ex_stage_id is
    doing. u_ex_stage_id is the real one -- it accepts the op, owns id_replay /
    the ldqid / the iterate addresses, and drives vecu. Without the split, the
    request stages would have to read a decode bundle that describes the
    EXECUTING op while it replays, which is what forced the old single-FSM
    wrapper to stall op N+1 until op N finished.
 */

`include "tt_cii_caracal_pkg.svh"

module tt_vpu_cii_wrapper_top
  import tt_cii_caracal_pkg::*;
#(
  parameter VLEN = 256
) (
  input  logic clk,
  input  logic reset_n,

  // CII coprocessor-side interface (four credit-metered channels; typed by
  // tt_cii_caracal_pkg). The relay drives this modport from the host side.
  tt_cii_interface.coprocessor cii_intf,

  // Debug signals for cosim checker
  output logic              debug_wb_vec_valid,
  output logic [VLEN*8-1:0] debug_wb_vec_wdata,
  output logic [7:0]        debug_wb_vec_wmask
);

  localparam INCL_VEC = 1;
  localparam LQ_DEPTH=8;
  localparam LQ_DEPTH_LOG2=$clog2(LQ_DEPTH);
  localparam INCL_FP = 1;
  localparam FLEN    = 32;
  localparam FP_RF_RD_PORTS  = 4;
  localparam EXP_WIDTH   = 8;
  localparam MAN_WIDTH   = 23;

  localparam int MAX_MEMBERS = 8;


  // Here are the stages of this pipeline.
  // The only possibility of pipeline stalls are S_REQ or S_WB not having enough credits
  // to accept more data from coproc, which should be a rare occurence given proper credit sizing.
  typedef enum logic [2:0] { S_FETCH, S_DEC, S_REQ, S_DRAIN, S_EX, S_WB } pipline_state_e;

  // The issue-packet payload that rides the whole pipeline unchanged.
  typedef struct packed {
    cii_caracal_tag_t    tag;
    cii_caracal_instr_t  instr;   // {insn, vtype, vl, vstart, vxrm, frm}
  } op_iss_s;

  // Everything S_REQ / S_DRAIN / S_EX need to know about the op, decoded ONCE in
  // S_DEC and then carried as data. No stage downstream of S_DEC looks at a
  // decoder output, so no stage can be aliased by a later instruction.
  typedef struct packed {
    logic [4:0]             max_src_beats;   // member walk length
    logic [4:0]             dst_nm;          // writeback beats (dest group size)
    logic [4:0]             vs1_beats;
    logic [4:0]             vs2_beats;
    logic [4:0]             vs3_beats;
    logic                   en_vs1;
    logic                   en_vs2;
    logic                   en_vs3;
    logic                   en_vm;
    logic                   rf_rden2;        // VS3 is a real 3rd source (not old-dest merge)
    logic                   scalar_src;      // vs1 comes from the INT/FP RF
    logic                   vec_single_reg;  // VPU writes member 0 only
    logic                   vs3_from_vs2;    // vmv<nr>r.v: no old-dest pull, VS3 staged from VS2
    logic                   scalar_dest;
    cii_caracal_dst_kind_e  dst_kind;
  } op_dec_s;


  // ---- Fetch Signals ---
  wire                              fetch_iss_pop;

  logic                             f_valid;
  op_iss_s                          f_op;

  logic                             FETCH_DEC_valid;
  logic                             DEC_FETCH_ready;
  wire                              fetch_fire;


  // ---- Decode Signals ---
  logic                             d_valid;
  op_iss_s                          d_op;

  op_dec_s                          dec_ctl;           // combinational decode of d_op
  logic [4:0]                       lmul_beats;
  logic                             vs2_wide;
  logic                             vmv_s_vec;
  logic                             vec_single_reg;
  logic                             scalar_src;
  logic                             need_old_dest;
  logic                             whole_reg_move;
  logic                             vm_needed;
  tt_briscv_pkg::vec_autogen_s      dec_uop;
  tt_briscv_pkg::csr_t              csr_dec;

  logic                             DEC_REQ_valid;
  logic                             REQ_DEC_ready;
  wire                              dec_fire;


  // ---- OP REQ Signals ---
  logic                             q_valid;
  op_iss_s                          q_op;
  op_dec_s                          q_ctl;
  logic [4:0]                       q_sent;            // src beats requested so far

  logic                             src_en_vs1, src_en_vs2, src_en_vs3, src_en_vm;
  logic                             req_send;

  logic                             REQ_DRAIN_valid;
  logic                             DRAIN_REQ_ready;
  wire                              req_fire;


  // ---- OP Drain Signals ---
  // REQ hands requested ops to pq so requests run ahead of the drain; pq feeds a
  // ring of NSLOT staging slots, filled in order by the in-order dat stream.
  localparam int PQ_DEPTH = 8;
  localparam int PQ_W     = $clog2(PQ_DEPTH);
  localparam int NSLOT    = 4;
  localparam int SLOT_W   = $clog2(NSLOT);

  typedef struct packed {
    op_iss_s op;
    op_dec_s ctl;
  } pq_ent_s;

  pq_ent_s                          pq_mem [0:PQ_DEPTH-1];
  logic [PQ_W-1:0]                  pq_rd, pq_wr;
  logic [PQ_W:0]                    pq_cnt;
  wire                              s_load;

  logic                             s_valid  [0:NSLOT-1];
  op_iss_s                          s_op     [0:NSLOT-1];
  op_dec_s                          s_ctl    [0:NSLOT-1];
  logic [4:0]                       s_popped [0:NSLOT-1];
  logic [4:0]                       s_rcvd   [0:NSLOT-1];
  logic [63:0]                      s_scalar [0:NSLOT-1];
  logic [VLEN-1:0]                  s_p0     [0:NSLOT-1][0:MAX_MEMBERS-1];
  logic [VLEN-1:0]                  s_p1     [0:NSLOT-1][0:MAX_MEMBERS-1];
  logic [VLEN-1:0]                  s_p2     [0:NSLOT-1][0:MAX_MEMBERS-1];
  logic [VLEN-1:0]                  s_vm     [0:NSLOT-1];
  logic [SLOT_W-1:0]                s_head;
  logic [SLOT_W:0]                  s_cnt;
  wire  [SLOT_W-1:0]                s_tail = s_head + s_cnt[SLOT_W-1:0];
  logic                             pop_found;
  logic [SLOT_W-1:0]                pop_slot;
  logic [SLOT_W-1:0]                rcv_slot;

  // The head slot: the op offered to S_EX.
  logic                             n_valid;
  op_iss_s                          n_op;
  op_dec_s                          n_ctl;
  logic [4:0]                       n_rcvd;
  logic [63:0]                      n_scalar;

  // Staging operand registers, written by S_DRAIN. Each vector source holds up to
  // MAX_MEMBERS member beats (VS2 wide -> up to 2*NM, bounded by the RVV LMUL<=4
  // widening rule => <=8 members), pulled via CII src-data and held until the
  // whole set is staged. The 4-lane src channel returns member `k` of
  // {VS1,VS2,VS3,VM} in one dat beat, so the member index equals the receive beat
  // count (n_rcvd).
  logic [VLEN-1:0]                  stg_p0 [0:MAX_MEMBERS-1]; // stg_p0 = VS1
  logic [VLEN-1:0]                  stg_p1 [0:MAX_MEMBERS-1]; // stg_p1 = VS2
  logic [VLEN-1:0]                  stg_p2 [0:MAX_MEMBERS-1]; // stg_p2 = VS3/old-dest
  logic [VLEN-1:0]                  stg_vm;                   // stg_vm = v0 mask

  logic                             DRAIN_EX_valid;
  logic                             ex_offer;          // DRAIN_EX_valid, held off while tt_id replays
  logic                             EX_DRAIN_ready;
  wire                              drain_fire;

  // The old-dest merge holds SINGLE-COPY per-op state (wbx_*) plus one snapshot
  // of the old dest, so a second multi-member vec_single_reg op waits at the
  // accept boundary until the outstanding merge retires.
  wire                              merge_block;


  // ---- Execute Signals ---
  logic                             x_valid;
  op_iss_s                          x_op;
  op_dec_s                          x_ctl;

  // S_EX's copy of the operands. The staging set is free to refill for the next
  // op the cycle after the handoff, so the executing op must read its own copy.
  logic [VLEN-1:0]                  ex_p0 [0:MAX_MEMBERS-1];
  logic [VLEN-1:0]                  ex_p1 [0:MAX_MEMBERS-1];
  logic [VLEN-1:0]                  ex_p2 [0:MAX_MEMBERS-1];
  logic [VLEN-1:0]                  ex_vm;
  logic [63:0]                      x_scalar;   // S_EX's copy of the .vx/.vf scalar

  // WHAT tt_id IS LOOKING AT THIS CYCLE. u_ex_stage_id decodes `i_if_instrn`
  // COMBINATIONALLY whenever id_replay is low (tt_id.sv:224 -- instrn_id =
  // id_replay ? instrn_id_replay : i_if_instrn), and o_vec_autogen IS that
  // decode. Driving it straight off the S_DRAIN register therefore hands the
  // EXECUTING op the NEXT op's decode bundle from the cycle after accept, since
  // S_REQ hands its op to S_DRAIN on that very cycle. tt_vec dispatches on that
  // bundle, and the wrapper subtracts its rf_addrp* from the executing op's
  // register fields to index the staged operands -- so both the functional unit
  // selection and the operand member indices flip mid-op.
  // A MULTI-ITERATION op HIDES THIS: id_replay is high for iterations 1..N-1,
  // so tt_id serves vec_autogen_replay and never looks at i_if_instrn again.
  // Only a SINGLE-ITERATION op (iterate=0: every plain .vv/.vx/.vi arithmetic
  // op) is exposed -- measured on jacobi2d_bare, where `vfadd.vv v27,v26,v29`
  // ran with the following `vmv1r.v v29,v25`'s bundle and wrote back its old
  // dest and vs1 interleaved instead of the sum.
  op_iss_s                          iss_op;
  logic [63:0]                      iss_scalar;

  // Combinational feed to tt_vec (assigned from the per-member operand arrays
  // below); tt_vec reads them at its iterate member address, matching the
  // tt_vec_regfile combinational read they replaced.
  logic [VLEN-1:0]                  vrf_p0_rddata;
  logic [VLEN-1:0]                  vrf_p1_rddata;
  logic [VLEN-1:0]                  vrf_p2_rddata;
  logic [VLEN-1:0]                  vrf_vm0_rddata;

  // present the operand-staged instruction to tt_id.
  logic [31:0]                      read_issue_inst;
  logic [63:0]                      read_issue_scalar_opnd;
  logic                             read_valid;
  logic                             id_ready_to_receive;
  logic [4:0]                       read_issue_sb_id;
  tt_briscv_pkg::inst_state_e       read_issue_state;

  tt_briscv_pkg::csr_t              csr_de0;
  tt_briscv_pkg::csr_t              csr_ex0;

  logic [63:0]                      rf_vex_p0, rf_vex_p1, fprf_vex_p0;
  logic [4:0]                       id_type;
  logic                             id_vex_rts, vex_id_rtr;
  tt_briscv_pkg::vec_autogen_s      id_vpu_uop_packet;
  logic                             vex_id_incr_addrp2;
  logic                             v_vm;
  logic                             vex_div_busy;
  logic [31:0]                      id_ex_instrn;
  logic                             id_replay;
  logic [4:0]                       iterate_addrp0, iterate_addrp1, iterate_addrp2;
  logic                             ignore_lmul, ignore_dstincr, ignore_srcincr;

  logic                             vpu_sat_csr;

  // ---- tt_vec result ports -> CII writeback (S_WB) ----
  logic                             vex_mem_lqvld_1c, vex_mem_lqvld_2c, vex_mem_lqvld_3c, vex_mem_lqvld_div;
  logic [VLEN-1:0]                  vex_mem_lqdata_1c, vex_mem_lqdata_2c, vex_mem_lqdata_3c, vex_mem_lqdata_div;
  tt_briscv_pkg::csr_fp_exc         vex_mem_lqexc_1c, vex_mem_lqexc_2c, vex_mem_lqexc_3c, vex_mem_lqexc_div;
  logic [LQ_DEPTH_LOG2-1:0]         vex_mem_lqid_1c, vex_mem_lqid_2c, vex_mem_lqid_3c, vex_mem_lqid_div;


  // ---- WriteBack Signals ---
  logic                       wb_result_valid;
  logic [VLEN-1:0]            wb_result_data;
  logic [LQ_DEPTH_LOG2-1:0]   wb_ldqid;
  tt_briscv_pkg::csr_fp_exc   wb_fp_flags;

  logic                       EX_WB_valid;
  logic                       WB_EX_ready;
  wire                        wb_fire;


  // ----- Credit Registers -----
  logic [$clog2(CII_N_REQ_CREDITS+1)-1:0] req_credit_cnt;
  logic [$clog2(CII_N_WB_CREDITS+1)-1 :0] wb_credit_cnt;


  // ----- VPU load-queue id allocator -----
  // tt_id derives every member's ldqid from i_mem_id_lqnxtid sampled at accept and
  // then increments it per member. Tying that input to 0 gives EVERY op the same
  // base, which is only safe while a single op is ever in flight -- the old
  // serialized wrapper. This wrapper overlaps ops: the next accept lands while the
  // previous op's 1c/2c/3c results are still draining, so a shared base overwrites
  // that op's *_by_lqid entries and its results are written back under the NEXT
  // op's tag, into the wrong architectural register. Allocate a disjoint lqid range
  // per op and hold the accept until the range has fully drained.
  // One lqid per dest-group member, exactly the span tt_id walks and the span the
  // *_by_lqid tables cover. Ops complete out of order: a range is marked done on its
  // op's final writeback beat and freed in allocation order once the head is done.
  logic [LQ_DEPTH_LOG2-1:0]  lq_alloc_ptr;
  logic [LQ_DEPTH_LOG2-1:0]  lq_free_ptr;
  logic [LQ_DEPTH_LOG2:0]    lq_outstanding;   // allocated-but-not-yet-freed lqids
  logic [4:0]                nm_at [0:LQ_DEPTH-1];  // range length, keyed by its base lqid
  logic                      range_done [0:LQ_DEPTH-1];  // keyed by base lqid
  logic [LQ_DEPTH_LOG2-1:0]  base_by_lqid [0:LQ_DEPTH-1];
  wire                       op_retire;        // final writeback beat of some op
  wire  [LQ_DEPTH_LOG2-1:0]  op_retire_base;   // that op's base lqid
  wire                       lq_free = (lq_outstanding != '0) && range_done[lq_free_ptr];
  wire  [4:0]                n_alloc_n   = n_ctl.dst_nm;
  wire  [5:0]                lq_after    = {2'd0, lq_outstanding} + {1'b0, n_alloc_n};
  wire                       lq_space_ok = (lq_after <= LQ_DEPTH);

  // ----- VPU result capture queue -----
  // tt_vec's four result ports are free-running: they assert whether or not the WB
  // channel has a credit, and with ops overlapping two different ops' results can
  // land in the same cycle. The old priority mux dropped the loser and dropped any
  // result produced with wb_credit_cnt==0. Capture every result strobe here and
  // drain one beat per cycle onto the single WB lane.
  localparam int RQ_DEPTH = 16;              // >= LQ_DEPTH, so it can never overflow
  localparam int RQ_PTR_W = $clog2(RQ_DEPTH);

  typedef struct packed {
    logic [LQ_DEPTH_LOG2-1:0] lqid;
    logic [VLEN-1:0]          data;
    tt_briscv_pkg::csr_fp_exc exc;
  } rq_ent_s;

  rq_ent_s             rq_mem [0:RQ_DEPTH-1];
  logic [RQ_PTR_W-1:0] rq_rd, rq_wr;
  logic [RQ_PTR_W:0]   rq_cnt;
  wire                 rq_pop;


  // ----- Old-dest merge writeback (vec_single_reg upper members 1..NM-1) -----
  // vec_single_reg ops emit one VPU result (member 0); the wrapper then writes the
  // remaining group members 1..NM-1 verbatim from the pulled old dest (undisturbed).
  logic [VLEN-1:0]          wb_olddest [0:MAX_MEMBERS-1];  // old dest group, latched at accept
  logic                     wbx_active;        // merging members 1..NM-1 this op
  logic                     wbx_pending;       // op accepted; awaiting its member-0 WB
  logic [4:0]               wbx_member;        // current merge member index
  logic [4:0]               wbx_nm;            // latched dst group size (NM)
  cii_caracal_tag_t         wbx_tag;           // latched op tag
  logic [LQ_DEPTH_LOG2-1:0] wbx_base_ldqid;    // op's member-0 ldqid (merge trigger)
  wire                      wb_send_merge = wbx_active;  // this cycle drives a merge beat

  // CII writeback lookups keyed by the VPU lqid (results carry lqid): tag,
  // dst-kind, dst member offset, and the final-member (last) marker. Filled for
  // ALL members at accept; S_WB indexes by the result lqid.
  cii_caracal_tag_t       tag_by_lqid     [0:LQ_DEPTH-1];
  cii_caracal_dst_kind_e  dstkind_by_lqid [0:LQ_DEPTH-1];
  cii_caracal_offset_t    dstoff_by_lqid  [0:LQ_DEPTH-1];
  logic                   last_by_lqid    [0:LQ_DEPTH-1];


  type(cii_intf.iss_data) iss_fifo_data;
  type(cii_intf.dat_data) dat_fifo_data;
  wire  iss_fifo_empty, dat_fifo_empty;
  logic dat_fifo_valid;   // RegNext(pop): pop_data valid this cycle

  wire  dat_pop;
  wire  dat_beat;

  /////////////////////////////////////////////////////////////////////////////
  // END Signal Declarations
  /////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////
  // Manage Src Request Credits
  /////////////////////////////////////////////////////////////////////////////
  always_ff @( posedge clk or negedge reset_n ) begin
    if (!reset_n) begin
      req_credit_cnt  <=  CII_N_REQ_CREDITS;
    end else begin
      // Send-credit counter: -1 per request sent, +1 per returned credit, up to max Request Credits.
      // Do not increment credit if req_send on same cycle as credit return
      if (  cii_intf.req_credit && !req_send && req_credit_cnt != CII_N_REQ_CREDITS ) begin
        req_credit_cnt <= req_credit_cnt + 1'b1;

      end else if (!cii_intf.req_credit && req_send) begin
        req_credit_cnt <= req_credit_cnt - 1'b1;

      end else begin
        req_credit_cnt <= req_credit_cnt;
      end
    end
  end

  /////////////////////////////////////////////////////////////////////////////
  // Manage WB Credits
  /////////////////////////////////////////////////////////////////////////////
  always_ff @(posedge clk or negedge reset_n) begin
    // Cap at the FIFO depth: the channel returns a credit every cycle wb_credit
    // is high, so an uncapped +1 wraps the counter to 0 and drops results.
    if (!reset_n) begin
      wb_credit_cnt <= CII_N_WB_CREDITS;
    end else begin
      if ( cii_intf.wb_credit && !wb_fire && wb_credit_cnt != CII_N_WB_CREDITS) begin
        wb_credit_cnt <= wb_credit_cnt + 1'b1;

      end else if (!cii_intf.wb_credit &&  wb_fire) begin
        wb_credit_cnt <= wb_credit_cnt - 1'b1;

      end else begin
        wb_credit_cnt <= wb_credit_cnt;
      end
    end
  end


  /////////////////////////////////////////////////////////////////////////////
  // Fetch Stage
  //   register: f_valid / f_op        consumer: S_DEC
  /////////////////////////////////////////////////////////////////////////////
  assign  FETCH_DEC_valid          = f_valid;
  assign  fetch_fire               = FETCH_DEC_valid && DEC_FETCH_ready;

  // tt_cii_fifo's pop_data register IS the S_FETCH register: it loads on pop and
  // holds otherwise, so popping whenever it is free or being consumed is 1 op/cycle.
  assign  fetch_iss_pop            = !iss_fifo_empty && (!f_valid || fetch_fire);
  assign  cii_intf.iss_credit      = fetch_iss_pop; // return 1 credit per FIFO pop.

  always_comb begin
    f_op.tag   = iss_fifo_data[0].tag;
    f_op.instr = iss_fifo_data[0].instr;
  end

  always_ff @(posedge clk or negedge reset_n) begin
    if ( !reset_n ) begin
      f_valid   <= 1'b0;
    end else begin
      if (fetch_iss_pop)   f_valid <= 1'b1;
      else if (fetch_fire) f_valid <= 1'b0;
    end
  end


  /////////////////////////////////////////////////////////////////////////////
  // Decode Stage
  //   register: d_valid / d_op        consumer: S_REQ
  ///////////////////////////////////////////////////////////////////////////
  assign  DEC_FETCH_ready = !d_valid || dec_fire;
  assign  DEC_REQ_valid   = d_valid;
  assign  dec_fire        = DEC_REQ_valid && REQ_DEC_ready;

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      d_valid <= 1'b0;
      d_op    <= '0;
    end else begin
      if (fetch_fire) begin
        d_valid <= 1'b1;
        d_op    <= f_op;
      end else if (dec_fire) begin
        d_valid <= 1'b0;
      end
    end
  end

  // u_decode_stage_id sees the same CSR bundle the real tt_id will see -- only
  // the LMUL/SEW-dependent ld/st autogen cells consume it, none of which this
  // wrapper reads, but wiring it keeps the two decodes identical.
  assign csr_dec.v_vsew   = d_op.instr.vtype.vsew;
  assign csr_dec.v_lmul   = d_op.instr.vtype.vlmul;
  assign csr_dec.v_vxrm   = d_op.instr.vxrm;
  assign csr_dec.v_vl     = d_op.instr.vl;
  assign csr_dec.v_vstart = d_op.instr.vstart;
  assign csr_dec.frm      = d_op.instr.frm;


  always_comb begin
    // Member count (EMUL) from LMUL: 1/2/4/8; fractional LMUL -> 1 register.
    lmul_beats = d_op.instr.vtype.vlmul[2] ? 5'd1 : (5'd1 << d_op.instr.vtype.vlmul[1:0]);

    vs2_wide   = dec_uop.nrwop || (dec_uop.wdeop && dec_uop.src1hw);

    vmv_s_vec  = (d_op.instr.insn[31:26] == 6'b010000) &&
                 ((d_op.instr.insn[14:12] == 3'b110) || (d_op.instr.insn[14:12] == 3'b101)) &&
                 (d_op.instr.insn[24:20] == 5'd0);

    vec_single_reg = dec_uop.reductop || dec_uop.mask_only || vmv_s_vec;

    // check is vs1 must read from int or fp regfile
    scalar_src = dec_uop.sel_scalar || dec_uop.fp_sel_scalar;

    vm_needed  = dec_uop.usemask || ~d_op.instr.insn[25];

    whole_reg_move = (d_op.instr.insn[6:0] == 7'b1010111) &&
                     (d_op.instr.insn[14:12] == 3'b011) &&
                     (d_op.instr.insn[31:26] == 6'b100111);

    // Pull the old dest group (VS3) for RMW/accumulate, tail-undisturbed (vta=0),
    // or mask-undisturbed masked ops -- AND for vec_single_reg ops, whose upper
    need_old_dest = ~whole_reg_move && (dec_uop.rf_rden2 ||
                    (~d_op.instr.vtype.vta) ||
                    (~d_op.instr.vtype.vma && ~d_op.instr.insn[25]) ||
                    vec_single_reg);

    // dst_nm = number of writeback beats = BOOM's dest group size.
    dec_ctl.dst_nm     = dec_uop.scalar_dest ? 5'd1 :
                         (dec_uop.wdeop ? (lmul_beats << 1) : lmul_beats);

    // members to fetch for the current source (scalar is a single beat).
    dec_ctl.vs1_beats  = scalar_src ? 5'd1              : lmul_beats;  // VS1 / scalar
    dec_ctl.vs2_beats  = vs2_wide   ? (lmul_beats << 1) : lmul_beats;  // VS2
    dec_ctl.vs3_beats  = dec_ctl.dst_nm;                               // VS3 = dest group

    dec_ctl.en_vs1     = dec_uop.rf_rden0 || scalar_src;
    dec_ctl.en_vs2     = dec_uop.rf_rden1;
    dec_ctl.en_vs3     = need_old_dest;
    dec_ctl.en_vm      = vm_needed;                                    // VM single register

    dec_ctl.rf_rden2       = dec_uop.rf_rden2;
    dec_ctl.scalar_src     = scalar_src;
    dec_ctl.vec_single_reg = vec_single_reg;
    dec_ctl.vs3_from_vs2   = whole_reg_move;
    dec_ctl.scalar_dest    = dec_uop.scalar_dest;

    // scalar-dest routes to INT RF, except vfmv.f.s (OPFVV, funct3=0b001) which
    // routes to the FP RF. autogen_v_scalar_dest fires for both OPMVV and OPFVV
    // funct6=0b010000, so distinguish on funct3 here.
    dec_ctl.dst_kind   = ~dec_uop.scalar_dest       ? CII_DST_VEC :
                         (d_op.instr.insn[14:12] == 3'b001) ? CII_DST_FP  : CII_DST_INT;

    // Member walk length = max member count over the ENABLED sources (a disabled
    // source must not extend the walk). VM contributes a single beat when needed.
    dec_ctl.max_src_beats = 5'd0;
    if (dec_ctl.en_vs1 && dec_ctl.vs1_beats > dec_ctl.max_src_beats)
      dec_ctl.max_src_beats = dec_ctl.vs1_beats;
    if (dec_ctl.en_vs2 && dec_ctl.vs2_beats > dec_ctl.max_src_beats)
      dec_ctl.max_src_beats = dec_ctl.vs2_beats;
    if (dec_ctl.en_vs3 && dec_ctl.vs3_beats > dec_ctl.max_src_beats)
      dec_ctl.max_src_beats = dec_ctl.vs3_beats;
    if (dec_ctl.en_vm  && 5'd1 > dec_ctl.max_src_beats)
      dec_ctl.max_src_beats = 5'd1;
  end


  /////////////////////////////////////////////////////////////////////////////
  // Operand Request Stage
  //   register: q_valid / q_op / q_ctl / q_sent      consumer: S_DRAIN
  /////////////////////////////////////////////////////////////////////////////
  assign  REQ_DEC_ready   = !q_valid || req_fire;
  assign  REQ_DRAIN_valid = q_valid && ((q_sent >= q_ctl.max_src_beats) ||
                                        (req_send && ({1'b0, q_sent} + 6'd1 >= {1'b0, q_ctl.max_src_beats})));
  assign  req_fire        = REQ_DRAIN_valid && DRAIN_REQ_ready;

  assign  req_send        = q_valid && (q_sent < q_ctl.max_src_beats) && (req_credit_cnt != 0);

  always_comb begin
    src_en_vs1  = q_ctl.en_vs1 && (q_sent < q_ctl.vs1_beats);  // VS1 or scalar rs1
    src_en_vs2  = q_ctl.en_vs2 && (q_sent < q_ctl.vs2_beats);  // VS2
    src_en_vs3  = q_ctl.en_vs3 && (q_sent < q_ctl.vs3_beats);  // VS3 (3rd src / RMW / undisturbed old-dest)
    src_en_vm   = q_ctl.en_vm  && (q_sent == 5'd0);            // VM (v0 mask)
  end

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      q_valid <= 1'b0;
      q_op    <= '0;
      q_ctl   <= '0;
      q_sent  <= 5'd0;
    end else begin
      if (dec_fire) begin
        q_valid <= 1'b1;
        q_op    <= d_op;
        q_ctl   <= dec_ctl;
        q_sent  <= 5'd0;
      end else begin
        if (req_fire) q_valid <= 1'b0;
        if (req_send) q_sent  <= q_sent + 1'b1;
      end
    end
  end


  /////////////////////////////////////////////////////////////////////////////
  // Operand Drain Stage
  //   register: pq_* / s_* slot ring (head slot = n_* / stg_*)
  //   consumer: S_EX (the accept handshake of u_ex_stage_id)
  /////////////////////////////////////////////////////////////////////////////
  assign  merge_block     = n_valid && n_ctl.vec_single_reg && (n_ctl.dst_nm > 5'd1) &&
                            (wbx_pending || wbx_active);

  assign  DRAIN_REQ_ready = (pq_cnt != PQ_DEPTH);
  // Every accept gate must sit on the VALID side: tt_id accepts on (rts & rtr) alone,
  // so a ready-only gate lets it swallow an op with no wrapper bookkeeping.
  assign  DRAIN_EX_valid  = n_valid && (n_rcvd >= n_ctl.max_src_beats) && !merge_block && lq_space_ok;
  // A replaying tt_id still reads iss_op/rd_mem_p*/vrf_p* for the op in S_EX.
  assign  ex_offer        = DRAIN_EX_valid && !id_replay;
  assign  drain_fire      = ex_offer && EX_DRAIN_ready;

  assign  s_load          = (pq_cnt != '0) && ((s_cnt != NSLOT) || drain_fire);

  always_comb begin
    n_valid  = (s_cnt != '0);
    n_op     = s_op[s_head];
    n_ctl    = s_ctl[s_head];
    n_rcvd   = s_rcvd[s_head];
    n_scalar = s_scalar[s_head];
    for (int m = 0; m < MAX_MEMBERS; m++) begin
      stg_p0[m] = s_p0[s_head][m];
      stg_p1[m] = s_p1[s_head][m];
      stg_p2[m] = s_p2[s_head][m];
    end
    stg_vm = s_vm[s_head];
  end

  // Oldest slot that still has src beats to pop; dat beats return in request order.
  always_comb begin
    pop_found = 1'b0;
    pop_slot  = s_head;
    for (int k = NSLOT-1; k >= 0; k--) begin
      automatic logic [SLOT_W-1:0] idx = s_head + k[SLOT_W-1:0];
      if ((k < s_cnt) && (s_popped[idx] < s_ctl[idx].max_src_beats)) begin
        pop_found = 1'b1;
        pop_slot  = idx;
      end
    end
  end

  assign  dat_pop         = pop_found && !dat_fifo_empty;
  assign  dat_beat        = dat_fifo_valid;
  assign  cii_intf.dat_credit = dat_pop;   // return 1 credit per src-data pop

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      dat_fifo_valid <= 1'b0;
      rcv_slot       <= '0;
    end else begin
      dat_fifo_valid <= dat_pop;
      if (dat_pop) rcv_slot <= pop_slot;
    end
  end

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      pq_rd  <= '0;
      pq_wr  <= '0;
      pq_cnt <= '0;
    end else begin
      if (req_fire) begin
        pq_mem[pq_wr] <= '{op: q_op, ctl: q_ctl};
        pq_wr         <= pq_wr + 1'b1;
      end
      if (s_load) pq_rd <= pq_rd + 1'b1;
      pq_cnt <= pq_cnt + {{PQ_W{1'b0}}, req_fire} - {{PQ_W{1'b0}}, s_load};
    end
  end

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      s_head <= '0;
      s_cnt  <= '0;
      for (int i = 0; i < NSLOT; i++) begin
        s_valid[i]  <= 1'b0;
        s_popped[i] <= 5'd0;
        s_rcvd[i]   <= 5'd0;
      end
    end else begin
      if (dat_pop)  s_popped[pop_slot] <= s_popped[pop_slot] + 1'b1;
      if (dat_beat) s_rcvd[rcv_slot]   <= s_rcvd[rcv_slot]   + 1'b1;
      if (drain_fire) begin
        s_valid[s_head] <= 1'b0;
        s_head          <= s_head + 1'b1;
      end
      if (s_load) begin
        s_valid[s_tail]  <= 1'b1;
        s_op[s_tail]     <= pq_mem[pq_rd].op;
        s_ctl[s_tail]    <= pq_mem[pq_rd].ctl;
        s_popped[s_tail] <= 5'd0;
        s_rcvd[s_tail]   <= 5'd0;
      end
      s_cnt <= s_cnt + {{SLOT_W{1'b0}}, s_load} - {{SLOT_W{1'b0}}, drain_fire};
    end
  end


  always_ff @( posedge clk ) begin
    // Clear the scalar latch as the op enters a slot (.vv/.vi never write it).
    if (s_load) s_scalar[s_tail] <= 64'd0;

    if (dat_beat) begin
      // lane 0: VS1 vector, OR the .vx/.vf scalar (a single beat -> member 0 only;
      //         later beats on lane 0 are NONE, so capture the scalar at beat 0).
      if (s_ctl[rcv_slot].scalar_src) begin
        if (s_rcvd[rcv_slot] == 5'd0) s_scalar[rcv_slot] <= dat_fifo_data[0].rsp_dat[63:0];
      end else begin
        s_p0[rcv_slot][s_rcvd[rcv_slot][CII_MEMBER_W-1:0]] <= dat_fifo_data[0].rsp_dat;
      end
      // lane 1: VS2  (wide -> up to 2*NM members)
      s_p1[rcv_slot][s_rcvd[rcv_slot][CII_MEMBER_W-1:0]] <= dat_fifo_data[1].rsp_dat;
      // lane 2: VS3 / old-dest
      s_p2[rcv_slot][s_rcvd[rcv_slot][CII_MEMBER_W-1:0]] <= s_ctl[rcv_slot].vs3_from_vs2 ?
                                                           dat_fifo_data[1].rsp_dat :
                                                           dat_fifo_data[2].rsp_dat;
      // lane 3: v0 mask -- single register (member 0 only)
      if (s_rcvd[rcv_slot] == 5'd0) s_vm[rcv_slot] <= dat_fifo_data[3].rsp_dat;
    end
  end


  /////////////////////////////////////////////////////////////////////////////
  // Execute Stage
  //   register: x_valid / x_op / x_ctl / ex_*        producer: S_DRAIN
  //
  // The S_DRAIN -> S_EX handshake IS u_ex_stage_id's instruction handshake, so
  // the instruction presented to it comes from the S_DRAIN register and the S_EX
  // register captures the same op on the accept cycle.
  /////////////////////////////////////////////////////////////////////////////

  assign  EX_DRAIN_ready = id_ready_to_receive;

  assign  read_valid              = ex_offer;
  // Present S_DRAIN's op only on the cycles it is actually being offered to
  // tt_id; otherwise hold the op resident in S_EX. See the iss_op declaration.
  assign  iss_op                  = ex_offer ? n_op     : x_op;
  assign  iss_scalar              = ex_offer ? n_scalar : x_scalar;

  assign  read_issue_inst         = iss_op.instr.insn;
  assign  read_issue_scalar_opnd  = iss_scalar;   // .vx/.vf scalar (0 for .vv/.vi)
  assign  read_issue_state        = tt_briscv_pkg::inst_state_e'(0);
  assign  read_issue_sb_id        = {1'b0, iss_op.tag};
  // decoded CSR config from the issue packet.
  assign  csr_de0.v_vsew          = iss_op.instr.vtype.vsew;
  assign  csr_de0.v_lmul          = iss_op.instr.vtype.vlmul;
  assign  csr_de0.v_vxrm          = iss_op.instr.vxrm;
  assign  csr_de0.v_vl            = iss_op.instr.vl;
  assign  csr_de0.v_vstart        = iss_op.instr.vstart;
  assign  csr_de0.frm             = iss_op.instr.frm;

  assign op_retire      = wb_fire && (wb_send_merge ? (wbx_member == wbx_nm - 1'b1)
                                                  : last_by_lqid[wb_ldqid]);
  assign op_retire_base = wb_send_merge ? wbx_base_ldqid : base_by_lqid[wb_ldqid];

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      x_valid <= 1'b0;
      x_op    <= '0;
      x_ctl   <= '0;
    end else begin
      if (drain_fire) begin
        x_valid <= 1'b1;
        x_op    <= n_op;
        x_ctl   <= n_ctl;
      end
    end
  end

  always_ff @( posedge clk ) begin
    if (drain_fire) begin
      for (int m = 0; m < MAX_MEMBERS; m++) begin
        ex_p0[m] <= stg_p0[m];
        ex_p1[m] <= stg_p1[m];
        ex_p2[m] <= stg_p2[m];
      end
      ex_vm    <= stg_vm;
      x_scalar <= n_scalar;
    end
  end

  // Per-lqid writeback tables. The ldqid the VPU assigns (vec_autogen.ldqid)
  // increments by 1 per member and echoes on each result, so fill all dst_nm
  // entries at accept. ldqid must come from u_ex_stage_id -- u_decode_stage_id
  // never issues, so its ldqid is meaningless.
  always_ff @( posedge clk ) begin
    if (drain_fire) begin
      for (int m = 0; m < MAX_MEMBERS; m++) begin
        if (m < n_ctl.dst_nm) begin                        // dst_nm result beats
          tag_by_lqid    [(id_vpu_uop_packet.ldqid + m) % LQ_DEPTH] <= n_op.tag;
          dstkind_by_lqid[(id_vpu_uop_packet.ldqid + m) % LQ_DEPTH] <= n_ctl.dst_kind;
          dstoff_by_lqid [(id_vpu_uop_packet.ldqid + m) % LQ_DEPTH] <= m[CII_MEMBER_W-1:0];
          last_by_lqid   [(id_vpu_uop_packet.ldqid + m) % LQ_DEPTH] <= (m == (n_ctl.dst_nm-1));
          base_by_lqid   [(id_vpu_uop_packet.ldqid + m) % LQ_DEPTH] <= id_vpu_uop_packet.ldqid;
        end
      end
    end
  end


  // Both of these must name the op whose decode tt_id is producing this cycle,
  // not the one that happens to be firing: rd_mem_p* is rf_addrp* MINUS the
  // register field, so a bundle and an instruction from two different ops give a
  // meaningless member index (measured: 25-26 = -1 -> member 7 of an LMUL=1
  // staging set). iss_op is that op by construction.
  wire [31:0]         cur_insn  = iss_op.instr.insn;
  cii_caracal_vtype_t cur_vtype;
  assign              cur_vtype = iss_op.instr.vtype;

  // READ PORT 2 IS NOT ALWAYS THE DESTINATION. tt_vec_top.sv:349 addresses it as
  //   reg_p2 = rf_rd_p2_is_rs2 ? rf_addrp1 : rf_addrp2
  // and the FMA's `c` input is that port verbatim (tt_vfp_ex_unit.sv:74). For
  // mode 1 -- the whole vfadd/vfsub/vfrsub family -- tt_vfp_ex_unit forces
  // `b` to 1.0 and computes a*1.0 + c, so THE SECOND ADDEND ARRIVES ON PORT 2,
  // not port 1. `autogen_v_rf_rd_p2_is_rs2` is exactly the set of ops for which
  // that is true.
  // This wrapper replaced tt_vec_regfile with the CII staging sets and only
  // ported the rf_addrp2 leg, so every one of those ops read the UNDISTURBED OLD
  // DESTINATION as its second addend. Measured on jacobi2d_bare:
  // `vfadd.vv v27,v26,v29` returned v29 + <stale v27 preg> element-by-element --
  // plausible-looking where the stale register happened to hold a denormal, wild
  // where it did not. vfmacc/vfmul (modes 3/2) genuinely want vd on port 2, which
  // is why axpy's vfmacc.vv passes and hides the family.
  wire       p2_is_rs2  = id_vpu_uop_packet.rf_rd_p2_is_rs2;
  wire [4:0] rd_mem_p0 = id_vpu_uop_packet.rf_addrp0 - cur_insn[19:15]; // VS1 base
  wire [4:0] rd_mem_p1 = id_vpu_uop_packet.rf_addrp1 - cur_insn[24:20]; // VS2 base
  // Same mux as reg_p2, and therefore the same base field: rs2 when port 2 is
  // reading rs2, vd otherwise.
  wire [4:0] rd_mem_p2 = p2_is_rs2 ? (id_vpu_uop_packet.rf_addrp1 - cur_insn[24:20])
                                   : (id_vpu_uop_packet.rf_addrp2 - cur_insn[11:7]);

  // Selected on ex_offer, the same term iss_op uses: the operands handed to
  // tt_vec always belong to the op whose instruction and decode bundle it is
  // looking at. (drain_fire implies ex_offer, so the accept cycle still
  // reads the staging set that is being copied into ex_* on this same edge.)
  assign vrf_p0_rddata  = ex_offer ? stg_p0[rd_mem_p0[CII_MEMBER_W-1:0]] : ex_p0[rd_mem_p0[CII_MEMBER_W-1:0]];
  assign vrf_p1_rddata  = ex_offer ? stg_p1[rd_mem_p1[CII_MEMBER_W-1:0]] : ex_p1[rd_mem_p1[CII_MEMBER_W-1:0]];
  // ...and the staging SET follows the mux too: rs2 lives in the VS2 set (lane 1),
  // which is already fetched, so this needs no extra source request.
  assign vrf_p2_rddata  = p2_is_rs2
    ? (ex_offer ? stg_p1[rd_mem_p2[CII_MEMBER_W-1:0]] : ex_p1[rd_mem_p2[CII_MEMBER_W-1:0]])
    : (ex_offer ? stg_p2[rd_mem_p2[CII_MEMBER_W-1:0]] : ex_p2[rd_mem_p2[CII_MEMBER_W-1:0]]);
  assign vrf_vm0_rddata = ex_offer ? stg_vm                              : ex_vm;


  /////////////////////////////////////////////////////////////////////////////
  // WriteBack Stage
  /////////////////////////////////////////////////////////////////////////////
  // wb = SENDER (drive wb_valid/wb_data + a credit counter fed by wb_credit).
  // Priority-select one of tt_vec's result ports (1c > 2c > 3c > div), map its
  // lqid -> CII tag + dst_kind + dst member offset + last, and emit a wb beat.
  // Each LMUL member produces one result beat (lqid = base_ldqid + member); the
  // per-lqid tables filled at accept give this beat's wb_dst_offset and the last
  // (final-member) marker. fflags come from the result exception.
  // debug_wb_vec_* mirror it for the cosim commit trace.
  //
  // !! PENDING (skeleton): if two result ports assert the same cycle (only across
  // different in-flight ops -- one op's members share a latency class), the
  // priority mux drops the loser; serializing concurrent results into the single
  // wb lane, and holding a result when wb_credit==0, need a small result buffer.
  /////////////////////////////////////////////////////////////////////////////
  // Capture every asserting result port this cycle (up to four, all from DIFFERENT
  // in-flight ops -- one op's members share a latency class), then present the
  // oldest entry to the WB lane.
  assign rq_pop = wb_fire && !wb_send_merge;

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      rq_rd  <= '0;
      rq_wr  <= '0;
      rq_cnt <= '0;
    end else begin
      automatic logic [RQ_PTR_W-1:0] wp = rq_wr;
      automatic logic [2:0]          np = 3'd0;
      if (vex_mem_lqvld_1c) begin
        rq_mem[wp] <= '{lqid: vex_mem_lqid_1c, data: vex_mem_lqdata_1c, exc: vex_mem_lqexc_1c};
        wp = wp + 1'b1; np = np + 1'b1;
      end
      if (vex_mem_lqvld_2c) begin
        rq_mem[wp] <= '{lqid: vex_mem_lqid_2c, data: vex_mem_lqdata_2c, exc: vex_mem_lqexc_2c};
        wp = wp + 1'b1; np = np + 1'b1;
      end
      if (vex_mem_lqvld_3c) begin
        rq_mem[wp] <= '{lqid: vex_mem_lqid_3c, data: vex_mem_lqdata_3c, exc: vex_mem_lqexc_3c};
        wp = wp + 1'b1; np = np + 1'b1;
      end
      if (vex_mem_lqvld_div) begin
        rq_mem[wp] <= '{lqid: vex_mem_lqid_div, data: vex_mem_lqdata_div, exc: vex_mem_lqexc_div};
        wp = wp + 1'b1; np = np + 1'b1;
      end
      rq_wr  <= wp;
      rq_cnt <= rq_cnt + {{RQ_PTR_W-2{1'b0}}, np} - {{RQ_PTR_W{1'b0}}, rq_pop};
      if (rq_pop) rq_rd <= rq_rd + 1'b1;
    end
  end

  always_comb begin
    wb_result_valid = (rq_cnt != '0);
    wb_result_data  = rq_mem[rq_rd].data;
    wb_ldqid        = rq_mem[rq_rd].lqid;
    wb_fp_flags     = rq_mem[rq_rd].exc;
  end

  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      lq_alloc_ptr   <= '0;
      lq_free_ptr    <= '0;
      lq_outstanding <= '0;
      for (int i = 0; i < LQ_DEPTH; i++) range_done[i] <= 1'b0;
    end else begin
      if (drain_fire) begin
        lq_alloc_ptr             <= lq_alloc_ptr + n_alloc_n[LQ_DEPTH_LOG2-1:0];
        nm_at[lq_alloc_ptr]      <= n_alloc_n;
        range_done[lq_alloc_ptr] <= 1'b0;
      end
      if (op_retire) range_done[op_retire_base] <= 1'b1;
      if (lq_free) begin
        lq_free_ptr             <= lq_free_ptr + nm_at[lq_free_ptr][LQ_DEPTH_LOG2-1:0];
        range_done[lq_free_ptr] <= 1'b0;
      end
      lq_outstanding <= lq_outstanding
                        + (drain_fire ? n_alloc_n[LQ_DEPTH_LOG2:0]          : '0)
                        - (lq_free    ? nm_at[lq_free_ptr][LQ_DEPTH_LOG2:0] : '0);
    end
  end

  // A beat is sent this cycle when either the VPU produced a result OR we are
  // merging old-dest members, and a WB credit is available. Merge takes priority
  // on the single WB lane (the vec_single_reg op's own VPU result already fired
  // member 0 before the merge starts, so there is no self-conflict).
  assign EX_WB_valid = wbx_active || wb_result_valid;
  assign WB_EX_ready = (wb_credit_cnt != 0);
  assign wb_fire     = EX_WB_valid && WB_EX_ready;

  assign cii_intf.wb_valid   = wb_fire;
  assign cii_intf.wb_data[0] = wb_send_merge ?
    // Merge beat: write undisturbed old-dest member `wbx_member` (last on NM-1).
    '{ inst_tag      : wbx_tag,
       wb_data       : wb_olddest[wbx_member[CII_MEMBER_W-1:0]],
       wb_dst_offset : wbx_member[CII_MEMBER_W-1:0],
       wb_wr_en      : 1'b1,
       wb_fp_flags   : '{ last     : (wbx_member == wbx_nm - 1'b1),
                          dst_kind : CII_DST_VEC,
                          vxsat    : 1'b0,
                          fflags   : '0 } }
    :
    // VPU-result beat (member 0 for vec_single_reg; every member for normal ops).
    '{ inst_tag      : tag_by_lqid[wb_ldqid],
       wb_data       : wb_result_data,
       wb_dst_offset : dstoff_by_lqid[wb_ldqid],
       wb_wr_en      : 1'b1,
       wb_fp_flags   : '{ last     : last_by_lqid[wb_ldqid],
                          dst_kind : dstkind_by_lqid[wb_ldqid],
                          vxsat    : 1'b0,
                          fflags   : {wb_fp_flags.fpNV,
                                      wb_fp_flags.fpDZ,
                                      wb_fp_flags.fpOF,
                                      wb_fp_flags.fpUF,
                                      wb_fp_flags.fpNX} } };


  /////////////////////////////////////////////////////////////////////////////
  // Old-dest merge FSM: for a vec_single_reg op (VPU writes only member 0), write
  // back the remaining group members 1..NM-1 from the undisturbed old dest.
  //   accept         -> latch NM/tag/base-ldqid + snapshot old dest (stg_p2).
  //   member-0 WB    -> start merging (wbx_active), member = 1.
  //   each merge beat (with credit) -> advance; last on member NM-1 completes the
  //                     group (its `last` bit drives the host group-done).
  // Emitting member 0 first then 1..NM-1 (last) guarantees all members are written
  // before the group-done fires. merge_block keeps this state single-owner.
  /////////////////////////////////////////////////////////////////////////////
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      wbx_pending <= 1'b0;
      wbx_active  <= 1'b0;
    end else begin
      // Snapshot the op + old dest when tt_id accepts a multi-member vec_single_reg.
      if (drain_fire && n_ctl.vec_single_reg && (n_ctl.dst_nm > 5'd1)) begin
        wbx_pending    <= 1'b1;
        wbx_nm         <= n_ctl.dst_nm;
        wbx_tag        <= n_op.tag;
        wbx_base_ldqid <= id_vpu_uop_packet.ldqid;
        for (int m = 0; m < MAX_MEMBERS; m++) wb_olddest[m] <= stg_p2[m];
      end
      // Kick off merging once this op's member-0 (VPU) result is emitted.
      if (wbx_pending && wb_fire && !wbx_active && (wb_ldqid == wbx_base_ldqid)) begin
        wbx_pending <= 1'b0;
        wbx_active  <= 1'b1;
        wbx_member  <= 5'd1;
      end
      // Advance one merge member per emitted beat.
      if (wbx_active && (wb_credit_cnt != 0)) begin
        if (wbx_member == wbx_nm - 1'b1) wbx_active <= 1'b0;
        wbx_member <= wbx_member + 1'b1;
      end
    end
  end

  /////////////////////////////////////////////////////////////////////////////
  // END WriteBack Control Logic
  /////////////////////////////////////////////////////////////////////////////


  /////////////////////////////////////////////////////////////////////////////
  // Drive the cii interface
  /////////////////////////////////////////////////////////////////////////////

  // req channel (SENDER): lane 0 to 3
  assign cii_intf.req_valid       = req_send;
  assign cii_intf.req_data[0]     = '{tag           : q_op.tag,
                                      rsp_src_id    : (src_en_vs1 ? (q_ctl.scalar_src ? CII_SRC_SCALAR : CII_SRC_VS1) : CII_SRC_NONE),
                                      rsp_src_offset: q_sent[CII_MEMBER_W-1:0]};

  assign cii_intf.req_data[1]     = '{tag           : q_op.tag,
                                      rsp_src_id    : (src_en_vs2 ? CII_SRC_VS2 : CII_SRC_NONE),
                                      rsp_src_offset: q_sent[CII_MEMBER_W-1:0]};

  // Slot 3 (VS3) and slot 6 (STALE_VD) name DIFFERENT groups on the host: VS3 is
  // the explicitly encoded third source (pvs3), STALE_VD is the old vd
  // (stale_pvdest). Only rf_rden2 means the datapath truly reads a third register
  // operand -- and there pvs3 and stale_pvdest coincide, so VS3 is right. Every
  // other need_old_dest case encodes NO third source and is a merge of the old
  // dest, so it must ask for STALE_VD.
  assign cii_intf.req_data[2]     = '{tag           : q_op.tag,
                                      rsp_src_id    : (~src_en_vs3     ? CII_SRC_NONE     :
                                                       q_ctl.rf_rden2  ? CII_SRC_VS3_VD   :
                                                                         CII_SRC_STALE_VD),
                                      rsp_src_offset: q_sent[CII_MEMBER_W-1:0]};

  assign cii_intf.req_data[3]     = '{tag           : q_op.tag,
                                      rsp_src_id    : (src_en_vm ? CII_SRC_VM  : CII_SRC_NONE),
                                      rsp_src_offset: q_sent[CII_MEMBER_W-1:0]};


  // Debug commit trace (cosim): member 0 of the dest group.
  assign debug_wb_vec_valid = wb_fire;
  assign debug_wb_vec_wdata = {{(VLEN*8-VLEN){1'b0}}, wb_result_data};
  assign debug_wb_vec_wmask = 8'h01;


  /////////////////////////////////////////////////////////////////////////////
  // Instatiate the Input CII FIFOs
  /////////////////////////////////////////////////////////////////////////////

  // ISSUE CHANNEL FIFO
  tt_cii_fifo #(
    .T            (type(cii_intf.iss_data)), 
    .DEPTH        (CII_N_ISS_CREDITS      )
  ) u_iss_fifo (
    .clk          (clk                    ),
    .rst_n        (reset_n                ),
    .push         (cii_intf.iss_valid     ), 
    .push_data    (cii_intf.iss_data      ),
    .pop          (fetch_iss_pop          ), 
    .pop_data     (iss_fifo_data          ), 
    .full         (                       ), // we dont check for this because Host should track credits  
    .empty        (iss_fifo_empty         )
  );
  
  // SRC DATA RETURN CHANNEL FIFO
  tt_cii_fifo #(
    .T            (type(cii_intf.dat_data)), 
    .DEPTH        (CII_N_DAT_CREDITS      )
  ) u_dat_fifo (
    .clk          (clk                    ), 
    .rst_n        (reset_n                ),
    .push         (cii_intf.dat_valid     ), 
    .push_data    (cii_intf.dat_data      ),
    .pop          (dat_pop                ), 
    .pop_data     (dat_fifo_data          ), 
    .full         (                       ), // we dont check for this because Host should track credits 
    .empty        (dat_fifo_empty         )
  );


  /////////////////////////////////////////////////////////////////////////////
  // Instatiate the VPU Top
  /////////////////////////////////////////////////////////////////////////////

  // DECODE-STAGE tt_id. i_if_instrn_rts is tied LOW, so id_rts never asserts,
  // id_replay never sets, and o_vec_autogen is unconditionally the fresh
  // combinational decode of d_op.instr.insn. DO NOT read dec_uop.ldqid: it tracks
  // this idle instance's own LQ pointer.
  tt_id #(
    .LQ_DEPTH(LQ_DEPTH),
    .LQ_DEPTH_LOG2(LQ_DEPTH_LOG2), 
    .EXP_WIDTH(EXP_WIDTH),
    .MAN_WIDTH(MAN_WIDTH),
    .FLEN(FLEN),
    .VLEN(VLEN),
    .FP_RF_RD_PORTS(FP_RF_RD_PORTS),
    .INCL_VEC(INCL_VEC),
    .INCL_FP(INCL_FP)
) u_decode_stage_id (
    .i_clk                                 (clk),    
    .i_reset_n                             (reset_n), 

    // Dispatch signals
    .dispatch_sb_id                        ('0),
    .dispatch_next_senior                  (1'b0),
    .dispatch_kill                         (1'b0),

    .i_csr                                 (csr_dec),             
    .o_csr                                 ( ),  
    .o_id_rf_vex_p0                        ( ),
    .o_id_rf_vex_p1                        ( ),
    .o_id_fprf_vex_p0                      ( ),

    .i_if_instrn                           (d_op.instr.insn),       
    .i_if_pc                               ('0),           
    .i_if_scalar_opnd                      ('0),
    .i_if_instrn_rts                       (1'b0),          // never issues -> never replays
    .o_id_instrn_rtr                       ( ),    

    .o_id_type                             ( ),          
    .o_id_immed_op                         ( ),     

    // Vector Interface
    .o_id_vex_rts                          ( ),
    .i_vex_id_rtr                          (1'b1),
    .o_vec_autogen                         (dec_uop),   
    .o_id_vex_lqid                         ( ), 
    .i_vex_id_incr_addrp2                  (1'b0),        
    .o_v_vm                                ( ),

    // EX Interface
    .i_div_resource_busy                   (1'b0),        
    .o_id_ex_rts                           ( ),
    .i_ex_rtr                              (1'b1 ),      
    .o_id_ex_pc                            ( ),
    .o_id_ex_instrn                        ( ),
    .o_id_ex_lqid                          ( ),
    .o_id_ex_vecldst                       ( ),
    .o_id_ex_Zb_instr                      ( ),
    .o_id_ex_units_rts                     ( ),
    .o_id_ex_instdisp                      ( ),
    .o_vecldst_autogen                     ( ),
    .o_id_ex_last                          ( ),
    .i_ex_dst_vld_1c                       (1'b0),         
    .i_ex_dst_lqid_1c                      ('0), 
    .i_ex_fwd_data_1c                      ('0),  
    .i_ex_dst_vld_2c                       (1'b0),         
    .i_ex_dst_lqid_2c                      ('0), 
    .i_ex_fwd_data_2c                      ('0),  

    // Integer RegFile Interface
    .o_rf_wr_flag                          ( ),
    .o_rf_wraddr                           ( ),

    // FP RegFile Interface
    .o_fp_rf_wr_flag                       ( ),
    .o_fp_rf_wraddr                        ( ),

    // Mem Interface
    .i_lq_broadside_info                   ('0),     
    .o_id_mem_lqinfo                       ( ),
    .o_id_replay                           ( ),
    .o_id_mem_lqalloc                      ( ),
    .o_id_mem_lq_done                      ( ),
    .i_mem_dst_vld                         ('0), 
    .i_mem_dst_lqid                        ('0), 
    .i_mem_fwd_data                        ('0), 
    .i_mem_lq_op                           ('0),        
    .i_mem_lq_commit                       ('0),         
    .i_lq_broadside_data                   ('0),     
    .i_lq_broadside_valid                  ('0), 
    .i_lq_broadside_data_valid             ('0), 

    // Misc
    .i_iterate_addrp0                      ('0),   
    .i_iterate_addrp1                      ('0),   
    .i_iterate_addrp2                      ('0),   
    .i_ignore_lmul                         (1'b0),           
    .i_ignore_dstincr                      (1'b0),        
    .i_ignore_srcincr                      (1'b0),        
    .i_mem_fe_lqfull                       (1'b0),         
    .i_mem_fe_lqempty                      ('0),
    .i_mem_fe_skidbuffull                  ('0),    
    .i_mem_id_lqnxtid                      ('0),

    .o_is_whole_memop                      ( ),
    .o_is_masked_memop                     ( ),
    .o_is_indexldst                        ( ),
    .o_is_maskldst                         ( ),
    .i_if_sb_id                            ('0),
    .o_id_sb_id                            ( ),

    .i_if_state                            (tt_briscv_pkg::inst_state_e'(0)),
    .o_id_state                            ( )
  );


  // EX-STAGE tt_id. The real one: it accepts the op out of S_DRAIN, owns the
  // ldqid and id_replay, and drives vecu's iteration.
  tt_id #(
    .LQ_DEPTH(LQ_DEPTH),
    .LQ_DEPTH_LOG2(LQ_DEPTH_LOG2), 
    .EXP_WIDTH(EXP_WIDTH),
    .MAN_WIDTH(MAN_WIDTH),
    .FLEN(FLEN),
    .VLEN(VLEN),
    .FP_RF_RD_PORTS(FP_RF_RD_PORTS),
    .INCL_VEC(INCL_VEC),
    .INCL_FP(INCL_FP)
  ) u_ex_stage_id (
    .i_clk                                 (clk),    
    .i_reset_n                             (reset_n), 

    // Dispatch signals
    .dispatch_sb_id                        ('0),
    .dispatch_next_senior                  (1'b0),
    .dispatch_kill                         (1'b0),

    .i_csr                                 (csr_de0),             
    .o_csr                                 (csr_ex0),  
    .o_id_rf_vex_p0                        (rf_vex_p0),
    .o_id_rf_vex_p1                        ( ),
    .o_id_fprf_vex_p0                      (fprf_vex_p0),

    .i_if_instrn                           (read_issue_inst),       
    .i_if_pc                               ('0),           
    .i_if_scalar_opnd                      (read_issue_scalar_opnd),
    .i_if_instrn_rts                       (read_valid),    
    .o_id_instrn_rtr                       (id_ready_to_receive),    

    .o_id_type                             (id_type),          
    .o_id_immed_op                         ( ),     

    // Vector Interface (ungated: operands are prefetched into staging before issue)
    .o_id_vex_rts                          (id_vex_rts),
    .i_vex_id_rtr                          (vex_id_rtr),
    .o_vec_autogen                         (id_vpu_uop_packet),   
    .o_id_vex_lqid                         ( ), 
    .i_vex_id_incr_addrp2                  (vex_id_incr_addrp2),    
    .o_v_vm                                (v_vm            ),    

    // EX Interface
    .i_div_resource_busy                   (vex_div_busy),
    .o_id_ex_rts                           ( ),             
    .i_ex_rtr                              (1'b1 ),    
    .o_id_ex_pc                            ( ),        
    .o_id_ex_instrn                        (id_ex_instrn),    
    .o_id_ex_lqid                          ( ), 
    .o_id_ex_vecldst                       ( ),         
    .o_id_ex_Zb_instr                      ( ),   
    .o_id_ex_units_rts                     ( ),       
    .o_id_ex_instdisp                      ( ),
    .o_vecldst_autogen                     ( ), 
    .o_id_ex_last                          ( ),  
    .i_ex_dst_vld_1c                       (1'b0),         
    .i_ex_dst_lqid_1c                      ('0), 
    .i_ex_fwd_data_1c                      ('0),  
    .i_ex_dst_vld_2c                       (1'b0),         
    .i_ex_dst_lqid_2c                      ('0), 
    .i_ex_fwd_data_2c                      ('0),  

    // Integer RegFile Interface  
    .o_rf_wr_flag                          ( ),    
    .o_rf_wraddr                           ( ),    

    // FP RegFile Interface    
    .o_fp_rf_wr_flag                       ( ),    
    .o_fp_rf_wraddr                        ( ),    

    // Mem Interface
    .i_lq_broadside_info                   ('0),     
    .o_id_mem_lqinfo                       ( ),         
    .o_id_replay                           (id_replay),             
    .o_id_mem_lqalloc                      ( ),        
    .o_id_mem_lq_done                      ( ),        
    .i_mem_dst_vld                         ('0), 
    .i_mem_dst_lqid                        ('0), 
    .i_mem_fwd_data                        ('0), 
    .i_mem_lq_op                           ('0),        
    .i_mem_lq_commit                       ('0),         
    .i_lq_broadside_data                   ('0),     
    .i_lq_broadside_valid                  ('0), 
    .i_lq_broadside_data_valid             ('0), 

    // Misc
    .i_iterate_addrp0                      (iterate_addrp0),   
    .i_iterate_addrp1                      (iterate_addrp1),   
    .i_iterate_addrp2                      (iterate_addrp2),   
    .i_ignore_lmul                         (ignore_lmul),           
    .i_ignore_dstincr                      (ignore_dstincr),        
    .i_ignore_srcincr                      (ignore_srcincr),        
    .i_mem_fe_lqfull                       (1'b0),         
    .i_mem_fe_lqempty                      ('0),
    .i_mem_fe_skidbuffull                  ('0),    
    .i_mem_id_lqnxtid                      (lq_alloc_ptr),

    .o_is_whole_memop                      ( ),
    .o_is_masked_memop                     ( ),
    .o_is_indexldst                        ( ),
    .o_is_maskldst                         ( ),
    .i_if_sb_id                            (read_issue_sb_id),
    .o_id_sb_id                            ( ),

    .i_if_state                            (read_issue_state),
    .o_id_state                            ( )
  );


  tt_vec_top #(
    .VLEN(VLEN),
    .XLEN(64  )
  ) vecu (
    .i_clk                 (clk                   ), 
    .i_reset_n             (reset_n               ), 
    .i_csr                 (csr_ex0               ), // vector CSR bundle sampled at EX0 (vtype/vl/vstart/frm/...)
    .i_v_vm                (v_vm                  ), // instr[25] mask-enable bit: 0 masked op, 1 unmasked
    .i_id_vec_autogen      (id_vpu_uop_packet     ), // decoded VPU uop packet (vec_autogen_s decode bundle)
    .o_sat_csr             (vpu_sat_csr           ), // sticky saturation flag back to CSR (vxsat)
    // ID Interface
    .o_vex_div_busy        (vex_div_busy          ), // vector divide unit is occupied; ID must not issue another divide
    .i_id_vex_rts          (id_vex_rts            ), // "ready-to-send": ID has a valid vector uop; transfer fires when qualified by o_vex_id_rtr
    .o_vex_id_rtr          (vex_id_rtr            ), // VEX ready-to-receive: back-pressure to ID (0 = stall issue)
    .i_id_ex_vecldst       ('0                    ), // this uop is a vector load/store (routes to the mem path)
    .i_id_ex_instrn        (id_ex_instrn          ), // the 32-bit vector instruction being issued
    .i_id_replay           (id_replay             ), // 0 first cycle of a fresh vector instruction. // 1 a replay/iteration cycle of the LMUL>1 op already in flight.
    .i_id_type             (id_type               ), // operand/instruction type encoding (OPIVV/OPIVX/OPIVI/OPMVV/...)
    .o_vex_id_incr_addrp2  (vex_id_incr_addrp2    ), // tell ID to bump read-port-2 reg address for the next iteration
    .o_iterate_addrp0      (iterate_addrp0        ), // per-iteration source reg address, read port 0
    .o_iterate_addrp1      (iterate_addrp1        ), // per-iteration source reg address, read port 1
    .o_iterate_addrp2      (iterate_addrp2        ), // per-iteration source reg address, read port 2
    .o_ignore_lmul         (ignore_lmul           ), // override the LMUL-driven iteration count (op needs a fixed count)
    .o_ignore_dstincr      (ignore_dstincr        ), // hold the destination reg address across iterations (no dst increment)
    .o_ignore_srcincr      (ignore_srcincr        ), // hold source reg address across iterations (mask-producing ops: vmand.mm, viota, vid, vmsof...)
    // RegFile Interface
    .i_rf_vex_p0           (rf_vex_p0             ), // integer RF read data (int->vec moves); aligns with 0a, read pre-flop
    .i_fprf_vex_p0         (fprf_vex_p0           ), // FP RF read data (fp->vec moves); aligns with 0a, read pre-flop
    .i_vrf_p0_rddata       (vrf_p0_rddata         ), // VRF read port 0 data (src operand)
    .i_vrf_p1_rddata       (vrf_p1_rddata         ), // VRF read port 1 data (src operand)
    .i_vrf_p2_rddata       (vrf_p2_rddata         ), // VRF read port 2 data (src operand / old destination for RMW)
    .i_vrf_vm0_rddata      (vrf_vm0_rddata        ), // VRF mask register v0 read data
    // Mem Interface Write Back
    .o_vex_mem_lqvld_1c    (vex_mem_lqvld_1c      ), // load-queue writeback valid, pipe stage 1c
    .o_vex_mem_lqdata_1c   (vex_mem_lqdata_1c     ), // load-queue writeback result data, 1c
    .o_vex_mem_lqexc_1c    (vex_mem_lqexc_1c      ), // load-queue writeback FP exception flags, 1c
    .o_vex_mem_lqid_1c     (vex_mem_lqid_1c       ), // load-queue entry id being written back, 1c
    .o_vex_mem_lqvld_2c    (vex_mem_lqvld_2c      ), // load-queue writeback valid, pipe stage 2c
    .o_vex_mem_lqdata_2c   (vex_mem_lqdata_2c     ), // load-queue writeback result data, 2c
    .o_vex_mem_lqexc_2c    (vex_mem_lqexc_2c      ), // load-queue writeback FP exception flags, 2c
    .o_vex_mem_lqid_2c     (vex_mem_lqid_2c       ), // load-queue entry id being written back, 2c
    .o_vex_mem_lqvld_3c    (vex_mem_lqvld_3c      ), // load-queue writeback valid, pipe stage 3c
    .o_vex_mem_lqdata_3c   (vex_mem_lqdata_3c     ), // load-queue writeback result data, 3c
    .o_vex_mem_lqexc_3c    (vex_mem_lqexc_3c      ), // load-queue writeback FP exception flags, 3c
    .o_vex_mem_lqid_3c     (vex_mem_lqid_3c       ), // load-queue entry id being written back, 3c
    .o_vex_mem_lqvld_div   (vex_mem_lqvld_div     ), // division writeback path: valid
    .o_vex_mem_lqdata_div  (vex_mem_lqdata_div    ), // division writeback path: result data
    .o_vex_mem_lqexc_div   (vex_mem_lqexc_div     ), // division writeback path: FP exception flags
    .o_vex_mem_lqid_div    (vex_mem_lqid_div      ), // division writeback path: load-queue entry id

    // Tail-/mask-agnostic policy from the CII issue vtype (the VPU applies it).
    // These bypass tt_id's replay latch and reach tt_vec directly, so they must
    // track the EXECUTING op, not the one being staged.
    .i_vta                 (cur_vtype.vta         ),
    .i_vma                 (cur_vtype.vma         )
  );


  /////////////////////////////////////////////////////////////////////////////
  // Debug: pipeline occupancy, what the VPU accepted, and every result strobe.
  /////////////////////////////////////////////////////////////////////////////
  // Plusarg-gated (+vecTrace, the same switch VecTrace uses on the Chisel side) so
  // it costs nothing when off. A wrong reduction is otherwise indistinguishable at
  // the CII boundary from a correctly-computed one over wrong operands.
  // ---- Runtime invariants (always on; cheap) --------------------------------
  // Every writeback beat must name a live lqid range, else it is a stale/aliased result.
  wire [LQ_DEPTH_LOG2-1:0] wb_lq_age = wb_ldqid - lq_free_ptr;
  always_ff @(posedge clk) begin
    if (reset_n && wb_fire && !wb_send_merge) begin
      if (!({1'b0, wb_lq_age} < lq_outstanding))
        $fdisplay(32'h8000_0002, "[vpu] ERROR wb beat to unallocated lqid: lqid=%0d free_ptr=%0d outstanding=%0d tag=%0d",
                  wb_ldqid, lq_free_ptr, lq_outstanding, tag_by_lqid[wb_ldqid]);
      else if (range_done[base_by_lqid[wb_ldqid]])
        $fdisplay(32'h8000_0002, "[vpu] ERROR wb beat to retired range: lqid=%0d base=%0d tag=%0d",
                  wb_ldqid, base_by_lqid[wb_ldqid], tag_by_lqid[wb_ldqid]);
    end
    if (reset_n && wb_fire && wb_send_merge && range_done[wbx_base_ldqid])
      $fdisplay(32'h8000_0002, "[vpu] ERROR merge beat to retired range: merge_tag=%0d base=%0d member=%0d",
                wbx_tag, wbx_base_ldqid, wbx_member);
  end

  bit vpu_dbg_en;
  initial vpu_dbg_en = $test$plusargs("vecTrace");
  always_ff @(posedge clk) begin
    if (vpu_dbg_en && reset_n) begin
      if (fetch_fire)
        $fdisplay(32'h8000_0002, "[vpu] FETCH->DEC   tag=%0d insn=%08x", f_op.tag, f_op.instr.insn);
      if (dec_fire)
        $fdisplay(32'h8000_0002, "[vpu] DEC->REQ     tag=%0d insn=%08x maxbeats=%0d dst_nm=%0d scalar=%0b redop=%0b",
                 d_op.tag, d_op.instr.insn, dec_ctl.max_src_beats, dec_ctl.dst_nm,
                 dec_ctl.scalar_src, dec_uop.reductop);
      if (req_fire)
        $fdisplay(32'h8000_0002, "[vpu] REQ->DRAIN   tag=%0d sent=%0d", q_op.tag, q_sent);
      if (drain_fire)
        $fdisplay(32'h8000_0002, "[vpu] DRAIN->EX    tag=%0d insn=%08x vl=%0d vsew=%0d vlmul=%0d vstart=%0d dst_nm=%0d maxbeats=%0d",
                 n_op.tag, n_op.instr.insn, n_op.instr.vl, n_op.instr.vtype.vsew,
                 n_op.instr.vtype.vlmul, n_op.instr.vstart, n_ctl.dst_nm, n_ctl.max_src_beats);
      // tt_id's replay setup for the accepted op. A reduction walks ONE ELEMENT
      // PER ITERATION (tt_vec_iadd.sv:168), so the element count it actually sums
      // IS this replay count -- the only place a "summed 3 of 16" can come from.
      if (drain_fire)
        $fdisplay(32'h8000_0002, "[vpu]   id: iterate=%0b onecyc=%0b cnt_start=%0d vsew=%0d lmul=%0d ign_lmul=%0b ign_dstincr=%0b ign_srcincr=%0b vl=%0d",
                 u_ex_stage_id.vec_autogen.iterate, u_ex_stage_id.vec_autogen.onecycle_iterate,
                 u_ex_stage_id.id_replay_cnt_start, u_ex_stage_id.v_vsew, u_ex_stage_id.v_lmul,
                 u_ex_stage_id.i_ignore_lmul, u_ex_stage_id.i_ignore_dstincr,
                 u_ex_stage_id.i_ignore_srcincr, u_ex_stage_id.o_csr.v_vl);
      // One line per DISPATCHED iteration: which member of which staged source the
      // wrapper is feeding tt_vec this iteration, and the data it sees. A reduction
      // consumes one element per iteration, so this is the sum's actual input list.
      if (id_vex_rts)
        $fdisplay(32'h8000_0002, "[vpu]   iter replay=%0b cnt=%0d addrp0=%0d addrp1=%0d addrp2=%0d memp0=%0d memp1=%0d memp2=%0d p0lo=%016x p1lo=%016x p2lo=%016x",
                 id_replay, u_ex_stage_id.vec_autogen_replay.replay_cnt,
                 id_vpu_uop_packet.rf_addrp0, id_vpu_uop_packet.rf_addrp1,
                 id_vpu_uop_packet.rf_addrp2, rd_mem_p0, rd_mem_p1, rd_mem_p2,
                 vrf_p0_rddata[63:0], vrf_p1_rddata[63:0], vrf_p2_rddata[63:0]);
      if (dat_beat)
        $fdisplay(32'h8000_0002, "[vpu]   dat  tag=%0d beat=%0d l0=%016x l1=%016x l2=%016x l3=%016x",
                 s_op[rcv_slot].tag, s_rcvd[rcv_slot], dat_fifo_data[0].rsp_dat[63:0], dat_fifo_data[1].rsp_dat[63:0],
                 dat_fifo_data[2].rsp_dat[63:0], dat_fifo_data[3].rsp_dat[63:0]);
      if (req_send)
        $fdisplay(32'h8000_0002, "[vpu]   req  tag=%0d beat=%0d s0=%0d s1=%0d s2=%0d s3=%0d",
                 q_op.tag, q_sent, cii_intf.req_data[0].rsp_src_id, cii_intf.req_data[1].rsp_src_id,
                 cii_intf.req_data[2].rsp_src_id, cii_intf.req_data[3].rsp_src_id);
      // Raw VPU result strobes, as produced (before the capture queue).
      if (vex_mem_lqvld_1c)
        $fdisplay(32'h8000_0002, "[vpu] res1c lqid=%0d data=%016x tag=%0d off=%0d last=%0b",
                 vex_mem_lqid_1c, vex_mem_lqdata_1c[63:0], tag_by_lqid[vex_mem_lqid_1c],
                 dstoff_by_lqid[vex_mem_lqid_1c], last_by_lqid[vex_mem_lqid_1c]);
      if (vex_mem_lqvld_2c)
        $fdisplay(32'h8000_0002, "[vpu] res2c lqid=%0d data=%016x tag=%0d off=%0d last=%0b",
                 vex_mem_lqid_2c, vex_mem_lqdata_2c[63:0], tag_by_lqid[vex_mem_lqid_2c],
                 dstoff_by_lqid[vex_mem_lqid_2c], last_by_lqid[vex_mem_lqid_2c]);
      if (vex_mem_lqvld_3c)
        $fdisplay(32'h8000_0002, "[vpu] res3c lqid=%0d data=%016x tag=%0d off=%0d last=%0b",
                 vex_mem_lqid_3c, vex_mem_lqdata_3c[63:0], tag_by_lqid[vex_mem_lqid_3c],
                 dstoff_by_lqid[vex_mem_lqid_3c], last_by_lqid[vex_mem_lqid_3c]);
      if (vex_mem_lqvld_div)
        $fdisplay(32'h8000_0002, "[vpu] resdv lqid=%0d data=%016x tag=%0d off=%0d last=%0b",
                 vex_mem_lqid_div, vex_mem_lqdata_div[63:0], tag_by_lqid[vex_mem_lqid_div],
                 dstoff_by_lqid[vex_mem_lqid_div], last_by_lqid[vex_mem_lqid_div]);
      // The beat actually put on the CII writeback lane.
      if (wb_fire)
        $fdisplay(32'h8000_0002, "[vpu] WB %s tag=%0d off=%0d last=%0b data=%016x lqid=%0d rqcnt=%0d",
                 wb_send_merge ? "merge " : "result",
                 cii_intf.wb_data[0].inst_tag, cii_intf.wb_data[0].wb_dst_offset,
                 cii_intf.wb_data[0].wb_fp_flags.last, cii_intf.wb_data[0].wb_data[63:0],
                 wb_ldqid, rq_cnt);
    end
  end


endmodule
