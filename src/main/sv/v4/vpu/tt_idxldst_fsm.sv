// see LICENSE.TT for license details.

// Vector load/store instructions need to read out the index for gather/scatter operations.
// however, the index has to be ignored by the ex stage since ex only cares about the data
// index is only for the CPU-OVI interface
// this FSM is to assert a signal to ignore during index replays from the id stage for ex
module tt_idxldst_fsm (
  input logic i_clk,
  input logic i_reset_n,
  input logic i_is_indexldst,
  input logic i_id_ex_rts,
  input logic i_ex_rtr,
  input logic i_last_uop,
  
  // Instruction decode inputs (needed for segment-aware logic)
  input logic [31:0] i_instrn,
  input logic [2:0]  i_vtype_vsew,   // from vconfig.vtype.vsew CSR  
  input logic [2:0]  i_vtype_vlmul,  // from vconfig.vtype.vlmul CSR
  
  // Output: should this iteration be squashed?
  output logic o_squash_id_ex_rts
);

  // ========= INSTRUCTION DECODE =========
  
  // Decoded signals from instruction (similar to store buffer)
  logic [2:0]   decoded_idx_emul;    // Decoded from instruction  
  logic [2:0]   decoded_inst_emul;   // EMUL of the instruction
  logic [3:0]   decoded_seg_count;   // Decoded from instruction
  logic [2:0]   decoded_idx_mul;     // Decoded/adjusted idx_mul for widening indexes

  // Simple instruction decoder
  always_comb begin
    // idx-offset's effective lmul
    decoded_idx_emul = i_vtype_vlmul + 3'(i_instrn[13:12]) - i_vtype_vsew;
    decoded_idx_emul = decoded_idx_emul[2] ? '0 : decoded_idx_emul; // positive only
    // EMUL of the instruction
    decoded_inst_emul = i_vtype_vlmul[2] ? '0 : i_vtype_vlmul;
    // For indexed instructions, calculate idx_mul for widening
    decoded_idx_mul = (decoded_idx_emul > decoded_inst_emul) ? decoded_idx_emul - decoded_inst_emul : '0;
    // Decode segment count from nf field for segment instructions
    // dont have to check whole or mask since FSM only activates for index ldst
    decoded_seg_count = i_instrn[31:29] + 4'b0001;
  end

  // ========= FSM LOGIC (replicated from storebuffer) =========
  
  logic [3:0] group_id, seg_id, idx_id;

  // update group_id and seg_id during fire condition (same logic as storebuffer)
  always_ff @(posedge i_clk) begin
    // reset
    if (!i_reset_n) begin
      idx_id   <= '0;
      group_id <= '0;
      seg_id   <= '0;
    end
    // updating "counter" values during fire condition
    else if (i_is_indexldst && i_id_ex_rts && i_ex_rtr) begin
      // reset group_id and seg_id
      // reset at last since we need the register values during first element
      if (i_last_uop) begin
        idx_id   <= '0;
        group_id <= '0;
        seg_id   <= '0;
      end
      // saturate increment idx_id first (decode replays this part first if present)
      else if (idx_id != ((4'b1 << decoded_idx_mul) - 1'b1)) begin
        idx_id <= idx_id + 1'b1;
      end
      // saturate increment group_id next (since decode does seg major)
      else if (group_id != ((4'b1 << decoded_inst_emul) - 1'b1)) begin
        idx_id <= '0;
        group_id <= group_id + 1'b1;
      end
      // carry over group_id into seg_id finally
      else if (seg_id != (decoded_seg_count - 1'b1)) begin
        idx_id <= '0;
        group_id <= '0;
        seg_id   <= seg_id + 1'b1;
      end
      // not required, but just in case..
      else begin
        group_id <= '0;
        seg_id   <= '0;
      end
    end
  end

  // ========= OUTPUT LOGIC =========
  
  // Generate squash signal based on FSM state (replaces tt_vpu_ovi.sv lines 454-472)
  // This properly handles segments unlike the original simple logic
  assign o_squash_id_ex_rts = (idx_id != '0);
  
endmodule