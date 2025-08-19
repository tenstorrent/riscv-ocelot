// See LICENSE.TT for license details.

// this module expects decode to send reg values as segment major
// eg for EMUL=2, SEG=2, V_REG=2: v2, v3, v4, v5 (NOT v2, v4, v3, v5)
module tt_store_buffer #(
   parameter VLEN = 256,
  parameter CREDITS = 1
) (
  // clock and reset
  input  logic                  clock,
  input  logic                  reset_n,

  // From VPU - (raw inputs: decoded internally) (uses ready-valid)
  output logic                   enq_ready,
  input  logic                   enq_valid,
  input  logic                   enq_first,
  input  logic                   enq_last,
  input  logic [4:0]             enq_sb_id,        // From scoreboard (pass-through)
  input  logic [31:0]            enq_instrn,       // Raw instruction (decoded internally)
  input  logic [11:0]            enq_vtype_vl,     // from vconfig.vl CSR
  input  logic [2:0]             enq_vtype_vsew,   // from vconfig.vtype.vsew CSR  
  input  logic [2:0]             enq_vtype_vlmul,  // from vconfig.vtype.vlmul_mag CSR
  input  logic [VLEN-1:0]        enq_data,

  // To OVI (uses credit system)
  input  logic                   return_credit,
  output logic                   store_valid,
  output logic [511:0]           store_data
);

  // ========= DECLARATIONS =========

  // --- decoded signals ---
  
  logic [2:0]   enq_eew;         // Decoded from instruction
  logic [2:0]   enq_emul;        // Decoded from instruction  
  logic [3:0]   enq_seg_count;   // Decoded from instruction
  logic [11:0]  enq_vl;          // Decoded/adjusted VL
  logic [2:0]   enq_idx_mul;     // Decoded/adjusted idx_mul (this is to sequencially ignore widening index width from decode replay logic)
  logic [4:0]   enq_base_v_reg;  // Decoded base register (not required)
  
  store_buffer_dec #(
    .VLEN(VLEN)
  ) internal_decode (
    .i_instrn(enq_instrn),
    .i_vtype_vl(enq_vtype_vl),
    .i_vtype_vsew(enq_vtype_vsew), 
    .i_vtype_vlmul(enq_vtype_vlmul),
    .o_eew_enc(enq_eew),
    .o_emul_enc(enq_emul),
    .o_seg_count(enq_seg_count),
    .o_idx_mul(enq_idx_mul),
    .o_vl(enq_vl),
    .o_base_v_reg(enq_base_v_reg)
  );

  // --- buffer declarations ---

  typedef struct packed {
    logic            valid;
    logic            dont_send; // think of this like a poison bit (should not be considered for DEQ but should be present for order tracking)
    logic [4:0]      sb_id;
    logic [2:0]      eew;
    logic [2:0]      emul;
    logic [3:0]      seg_count;
    logic [11:0]     vl;
    logic [VLEN-1:0] data;
    logic            last;
  } store_buffer_entry_t;
  store_buffer_entry_t buffer [7:0];

  logic [2:0] rd_ptr, wr_ptr; // buffer pointers
  logic [3:0] buffer_size;    // buffer size

  // "fake" a ready-valid system (actually using a credit system)
  logic [$clog2(CREDITS):0] credits;
  logic deq_ready, deq_valid;

  // --- packer declarations ---

  logic [VLEN-1:0] buffer_data_vec  [7:0]; // vector of buffer data
  logic            buffer_valid_vec [7:0]; // vector of buffer valid

  logic [511:0] el_packed_data,    // packed data from packers
                vreg_packed_data;
  logic         el_packed_valid,   // valid from packers
                vreg_packed_valid;

  logic [$clog2(VLEN/8):0] element_id; // need this for deq

  // --- packer module connections ---

  always_comb begin
    for (int i = 0; i < 8; i += 1) begin
      buffer_data_vec[i]  = buffer[i].data;
      buffer_valid_vec[i] = buffer[i].valid;
    end
  end

  store_element_packer #(
    .VLEN(VLEN)
  ) element_packer (
    .eew(buffer[rd_ptr].eew),
    .emul(buffer[rd_ptr].emul),
    .element_id(element_id),
    .seg_count(buffer[rd_ptr].seg_count),
    .start_entry(rd_ptr),
    .buffer_data(buffer_data_vec),
    .buffer_valid(buffer_valid_vec),
    .packed_data(el_packed_data),
    .packed_valid(el_packed_valid)
  );

  store_vreg_packer #(
    .VLEN(VLEN)
  ) vreg_packer (
    .emul(buffer[rd_ptr].emul),
    .start_entry(rd_ptr),
    .buffer_data(buffer_data_vec),
    .buffer_valid(buffer_valid_vec),
    .packed_data(vreg_packed_data),
    .packed_valid(vreg_packed_valid)
  );

  // ========= OUTPUTS and CONTROL =========

  // --- control signals ---

  logic deq_is_segment;
  logic enq_fire, deq_fire;

  assign deq_is_segment = (buffer[rd_ptr].seg_count > 4'b1);
  assign deq_ready = |credits;
  assign deq_valid = ((deq_is_segment) ? el_packed_valid : vreg_packed_valid) && !buffer[rd_ptr].dont_send;

  assign enq_ready = (buffer_size < 8); // not full

  assign enq_fire  = (enq_ready && enq_valid);
  assign deq_fire  = (deq_ready && deq_valid);
  
  // --- outputs ---

  assign store_data = (deq_is_segment) ? el_packed_data : vreg_packed_data;
  assign store_valid = deq_fire; // since we're using credits

  // ========= ENQ "FSM" =========

  logic [3:0] vl_group_id; // the group_id with the last element
  logic [3:0] group_id, seg_id, idx_id;
  logic       enq_past_vl; // this is used to determine if the enq doesnt matter since we are past vl element
  logic       ignore_idx_replay; // this is to avoid decoder replay for widening indexes

  assign vl_group_id = ((enq_vl - 12'h1) >> ($clog2(VLEN/8)-enq_eew));
  assign enq_past_vl = (group_id > vl_group_id) || (enq_vl == '0);
  assign ignore_idx_replay = (idx_id != '0);

  // update group_id and seg_id during enq
  always_ff @(posedge clock) begin
    // reset
    if (!reset_n) begin
      idx_id   <= '0;
      group_id <= '0;
      seg_id   <= '0;
    end
    // updating "counter" values during enq
    else if (enq_fire) begin
      // reset group_id and seg_id
      // reset at last since we need the register values during first element
      if (enq_last) begin
        idx_id   <= '0;
        group_id <= '0;
        seg_id   <= '0;
      end
      // saturate increment idx_id first (decode replays this part first if present)
      else if (idx_id != ((4'b1 << enq_idx_mul) - 1'b1)) begin
        idx_id <= idx_id + 1'b1;
      end
      // saturate increment group_id next (since decode does seg major)
      else if (group_id != ((4'b1 << enq_emul) - 1'b1)) begin
        idx_id <= '0;
        group_id <= group_id + 1'b1;
      end
      // carry over group_id into seg_id finally
      else if (seg_id != (enq_seg_count - 1'b1)) begin
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

  // ========= DEQ "FSM" (only matters for segment mode) =========

  logic [11:0] element_ctr; // element counter
  logic [11:0] max_element_id;
  logic        deq_max_el_id_reached; // "constraint" used to determine if the deq has to move to next register
  logic        deq_vl_reached;        // "constraint" used to determine if the deq has reached vl

  assign max_element_id        = ((VLEN/8) >> buffer[rd_ptr].eew) - 1'b1;
  assign element_id            = element_ctr & ((1'b1 << ($clog2(VLEN/8)-buffer[rd_ptr].eew)) - 1'b1);
  assign deq_max_el_id_reached = (element_id >= max_element_id);
  assign deq_vl_reached        = (element_ctr >= (buffer[rd_ptr].vl - 1'b1));

  // update element_ctr during deq
  always_ff @(posedge clock) begin
    // reset element_ctr
    if (!reset_n) begin
      element_ctr <= '0;
    end
    // increment element_ctr
    else if (deq_fire && deq_is_segment && !buffer[rd_ptr].last) begin
      // reset is on last (since need the value ready at next "first")
      if (deq_vl_reached) begin
        element_ctr <= '0;
      end else begin
        element_ctr <= element_ctr + 1'b1;
      end
    end
  end

  // invalidate all these entries (since there are entries of other segments where vl was reached)
  logic [2:0] deq_ptrs       [7:0];
  logic       deq_ptrs_valid [7:0]; // pointers of "related" entries (same vec group for segment, immediate for vreg)

  always_comb begin
    for (int idx = 0; idx < 8; idx += 1) begin
      // segment mode: this are the entries used in element packer (segments spaced by emul)
      if (deq_is_segment) begin
        deq_ptrs[idx]       = 3'(rd_ptr + (idx << buffer[rd_ptr].emul)); // wrap overflow
        deq_ptrs_valid[idx] = (idx < buffer[deq_ptrs[idx]].seg_count);
      // vreg mode: this are the entries used in vreg packer (immideate entries upto emul)
      end else begin
        deq_ptrs[idx]       = 3'(rd_ptr + idx); // wrap overflow
        deq_ptrs_valid[idx] = (idx < (512/VLEN)) && (idx < (4'b1 << buffer[rd_ptr].emul));
      end
    end
  end

  // ========= ENQ/DEQ CONTROL =========

  // read the comments below to understand these signals
  logic enq_trigger, deq_buffer_free, deq_trigger;
  assign enq_trigger     = (enq_fire) && (!ignore_idx_replay);
  assign deq_buffer_free = (buffer[rd_ptr].valid && buffer[rd_ptr].dont_send);
  assign deq_trigger     = (deq_fire) && (!deq_is_segment || (deq_max_el_id_reached || deq_vl_reached));

  // --- buffer ---

  always_ff @(posedge clock) begin
    // reset (0 pointers and valids)
    if (!reset_n) begin
      for (int i = 0; i < 8; i += 1) begin
        buffer[i] <= '0; // valid = 0
      end
      rd_ptr <= '0;
      wr_ptr <= '0;
    // enq and deq
    end else begin
      // enq logic (simply latch from input)
      if (enq_trigger) begin
        buffer[wr_ptr].valid <= 1'b1; // remember to only validate if not past vl
        buffer[wr_ptr].dont_send <= enq_past_vl;
        buffer[wr_ptr].sb_id <= enq_sb_id;
        buffer[wr_ptr].eew   <= enq_eew;
        buffer[wr_ptr].emul  <= enq_emul;
        buffer[wr_ptr].seg_count <= enq_seg_count;
        buffer[wr_ptr].vl    <= enq_vl;
        buffer[wr_ptr].data  <= enq_data;
        buffer[wr_ptr].last  <= enq_last;
        wr_ptr               <= wr_ptr + 1;
      end
      // deq case1 (free invalid buffer)
      if (deq_buffer_free) begin
        buffer[rd_ptr] <= '0; // valid = 0
        rd_ptr <= rd_ptr + 1'b1;
      end
      // deq case2 (free buffer and invalidate unrequired entries)
      else if (deq_trigger) begin
        for (int idx = 0; idx < 8; idx += 1) begin
          if (deq_ptrs_valid[idx]) begin
            buffer[deq_ptrs[idx]].dont_send <= 1'b1;
          end
        end
        buffer[rd_ptr] <= '0; // valid = 0
        rd_ptr <= rd_ptr + 1'b1;
      end
    end
  end

  // --- buffer size ---

  always_ff @(posedge clock) begin
    if (!reset_n) begin
      buffer_size <= '0;
    end else begin
      if (enq_trigger && !(deq_buffer_free || deq_trigger)) begin
        buffer_size <= buffer_size + 1'b1;
      end
      else if (!enq_trigger && (deq_buffer_free || deq_trigger)) begin
        buffer_size <= buffer_size - 1'b1;
      end
    end
  end

  // ========= CREDITS =========

  always_ff @(posedge clock) begin
    if (!reset_n) begin
      credits <= CREDITS;
    end else begin
      if (deq_fire && !return_credit) begin
        credits <= credits - 1'b1;
      end
      else if (!deq_fire && return_credit) begin
        credits <= credits + 1'b1;
      end
    end
  end

  // ========= DEBUG =========

  store_buffer_debug #(
    .VLEN(VLEN)
  ) debug_module (
    .clock(clock),
    .reset_n(reset_n),
    .enq_ready(enq_ready),
    .enq_valid(enq_valid),
    .enq_first(enq_first),
    .enq_last(enq_last),
    .enq_sb_id(enq_sb_id),
    .enq_eew(enq_eew),
    .enq_emul(enq_emul),
    .enq_seg_count(enq_seg_count),
    .enq_vl(enq_vl),
    .enq_data(enq_data),
    .return_credit(return_credit),
    .store_valid(store_valid),
    .store_data(store_data)
  );

endmodule



// Segment data extraction and packing module (LRM-style)
module store_element_packer #(
   parameter VLEN = 256
) (
  // Configuration inputs
  input  logic [2:0]              eew,         // Element width (log2)
  input  logic [2:0]              emul,        // EMUL (log2)
  input  logic [$clog2(VLEN/8):0] element_id,  // Which element to extract
  input  logic [3:0]              seg_count,
  input  logic [2:0]              start_entry, // Starting buffer entry
  // Buffer data inputs (8 entries max)
  input  logic [VLEN-1:0]         buffer_data  [7:0],
  input  logic                    buffer_valid [7:0],
   
  // data output
  output logic [511:0]            packed_data,
  output logic                    packed_valid
);

  // Internal signals
  logic [11:0] element_bit_offset;       // Bit offset for current element
  logic [2:0] segment_ptrs [7:0];        // Buffer pointers for each segment
   
  // Calculate element bit offset
  assign element_bit_offset = {element_id, 3'h0} << eew; // element_num * element_size_bits
   
  // Calculate buffer pointers for each segment
  always_comb begin
    for (int seg = 0; seg < 8; seg += 1) begin
      segment_ptrs[seg] = 3'(start_entry + (seg << emul)); // wrap overflow
    end
  end

  always_comb begin
    packed_valid = 1'b1;
    for (int seg = 0; seg < 8; seg += 1) begin
      if (seg < seg_count)
        packed_valid &= buffer_valid[segment_ptrs[seg]];
    end
  end
   
  // Extract elements from each segment
  logic [8:0] idx;
  always_comb begin
    // initialize packed_data to 0
    packed_data = '0;
    // mux bytes of data from the buffer into the packed data
    for (int seg = 0; seg < 8; seg++) begin
      idx = 9'({seg[2:0], 3'h0} << eew[1:0]);
      case (eew)
        3'h0: begin // 8-bit elements
          packed_data[idx +: 8]  = buffer_data[segment_ptrs[seg]][element_bit_offset +: 8];
        end
        3'h1: begin // 16-bit elements  
          packed_data[idx +: 16] = buffer_data[segment_ptrs[seg]][element_bit_offset +: 16];
        end
        3'h2: begin // 32-bit elements
          packed_data[idx +: 32] = buffer_data[segment_ptrs[seg]][element_bit_offset +: 32];
        end
        3'h3: begin // 64-bit elements
          packed_data[idx +: 64] = buffer_data[segment_ptrs[seg]][element_bit_offset +: 64];
        end
      endcase
    end
  end

endmodule



// Elements lie across multiple vregs, pack them into 512 bits
module store_vreg_packer #(
  parameter VLEN = 256
) (
  // Configuration inputs
  input  logic [2:0]      emul,
  input  logic [2:0]      start_entry,
  // Buffer data inputs
  input  logic [VLEN-1:0] buffer_data  [7:0],
  input  logic            buffer_valid [7:0],
  
  // data output
  output logic [511:0]    packed_data,
  output logic            packed_valid
);

  always_comb begin
    packed_valid = 1'b1;
    for (int i = 0; i < 512/VLEN; i += 1) begin
      if (i < (4'b1 << emul))
        packed_valid &= buffer_valid[3'(start_entry + i)];
    end
  end

  // just pack all the consecutive vregs into the packed data
  always_comb begin
    packed_data = '0;
    for (int i = 0; i < 512/VLEN; i += 1) begin
      packed_data[i*VLEN +: VLEN] = buffer_data[3'(start_entry + i)];
    end
  end

endmodule



// decoding module for the store buffer
module store_buffer_dec #(
  parameter VLEN = 256
) (
  // Raw instruction decode inputs (from ID stage)
  input  logic [31:0]            i_instrn,
  input  logic [11:0]            i_vtype_vl,       // from vconfig.vl CSR
  input  logic [2:0]             i_vtype_vsew,     // from vconfig.vtype.vsew CSR  
  input  logic [2:0]             i_vtype_vlmul,    // from vconfig.vtype.vlmul_mag CSR
  
  // Decoded outputs (matching ls_decode.scala naming)
  output logic [2:0]             o_eew_enc,        // Encoded Effective Element Width
  output logic [2:0]             o_emul_enc,       // Encoded Effective LMUL
  output logic [3:0]             o_seg_count,      // Segment count (nf+1)
  output logic [2:0]             o_idx_mul,        // Index width multiplier (for indexed stores)
  output logic [11:0]            o_vl,             // Vector Length (adjusted for mask)
  output logic [4:0]             o_base_v_reg      // Base vector register (vs3)
);

  // Extract instruction fields (matching ls_decode.scala naming)
  wire [6:0]  instOP        = i_instrn[6:0];      // opcode
  wire [2:0]  instElemSize  = i_instrn[14:12];    // func3
  wire [1:0]  instMop       = i_instrn[27:26];    // mop field
  wire [4:0]  instUMop      = i_instrn[24:20];    // lumop field
  wire [2:0]  instNf        = i_instrn[31:29];    // nf field
  wire        instMaskEnable = !i_instrn[25];     // vm field (inverted)
  wire [1:0]  instWidth     = instElemSize[1:0];  // EEW field
  wire [4:0]  instVldDest   = i_instrn[11:7];     // vs3 for stores
  
  // Basic instruction type detection (matching ls_decode.scala)
  wire isStore  = (instOP == 7'b0100111);  // V_ST opcode  
  wire isUnit   = (instMop == 2'b00);
  wire isStride = (instMop == 2'b10);
  wire isIndex  = (instMop == 2'b01) || (instMop == 2'b11);
  
  // Store type detection (matching ls_decode.scala logic)
  wire isWhole  = (instMop == 2'b00) && (instUMop == 5'b01000);  // whole register
  wire isMaskLS = (instMop == 2'b00) && (instUMop == 5'b01011);  // mask store
  wire isSeg    = !isWhole && !isMaskLS && (instNf != 3'b000);   // segmented
  
  // EEW calculation (matching ls_decode.scala logic)
  wire [2:0] eew_mask   = 3'b000;           // Mask: fixed at 8 bits (encoded as 0) 
  wire [2:0] eew_index  = i_vtype_vsew;     // Indexed: config sew value
  wire [2:0] eew_normal = instWidth;        // Others: data width from instruction
  
  // EMUL calculation (matching ls_decode.scala logic)
  wire [2:0] emul_normal = i_vtype_vlmul + instWidth - i_vtype_vsew;  // EMUL = LMUL * (EEW/SEW)
  wire [2:0] emul_mask   = 3'b000;                                    // Masked: EMUL = 0 (fixed)
  wire [2:0] emul_index  = i_vtype_vlmul;                             // Indexed: LMUL from vtype CSR
  
  // Whole register EMUL calculation (matching ls_decode.scala whole_vlmul)
  logic [2:0] whole_vlmul;
  always_comb begin
    case (instNf)
      3'b000:  whole_vlmul = 3'b000;  // 1 register  (log2(1) = 0)
      3'b001:  whole_vlmul = 3'b001;  // 2 registers (log2(2) = 1)
      3'b011:  whole_vlmul = 3'b010;  // 4 registers (log2(4) = 2)  
      3'b111:  whole_vlmul = 3'b011;  // 8 registers (log2(8) = 3)
      default: whole_vlmul = 3'b000;
    endcase
  end
  
  // VL calculation (matching ls_decode.scala logic)
  wire [11:0] vl_mask   = (i_vtype_vl + 12'd7) >> 3;  // Mask: ceil(vl/8)
  wire [11:0] vl_normal = i_vtype_vl;                 // Normal: vl CSR value
  
  // Whole register VL calculation: (nf+1) * VLEN / EEW
  logic [11:0] vl_whole;
  always_comb begin
    case ({instNf, instWidth})
      // nf=0 (1 register)
      5'b00000: vl_whole = VLEN >> 3;  // 8-bit:  1*VLEN/8
      5'b00001: vl_whole = VLEN >> 4;  // 16-bit: 1*VLEN/16
      5'b00010: vl_whole = VLEN >> 5;  // 32-bit: 1*VLEN/32
      5'b00011: vl_whole = VLEN >> 6;  // 64-bit: 1*VLEN/64
      // nf=1 (2 registers)  
      5'b00100: vl_whole = VLEN >> 2;  // 8-bit:  2*VLEN/8
      5'b00101: vl_whole = VLEN >> 3;  // 16-bit: 2*VLEN/16
      5'b00110: vl_whole = VLEN >> 4;  // 32-bit: 2*VLEN/32
      5'b00111: vl_whole = VLEN >> 5;  // 64-bit: 2*VLEN/64
      // nf=3 (4 registers)
      5'b01100: vl_whole = VLEN;       // 8-bit:  4*VLEN/8
      5'b01101: vl_whole = VLEN >> 1;  // 16-bit: 4*VLEN/16
      5'b01110: vl_whole = VLEN >> 2;  // 32-bit: 4*VLEN/32
      5'b01111: vl_whole = VLEN >> 3;  // 64-bit: 4*VLEN/64
      // nf=7 (8 registers)
      5'b11100: vl_whole = VLEN << 1;  // 8-bit:  8*VLEN/8
      5'b11101: vl_whole = VLEN;       // 16-bit: 8*VLEN/16
      5'b11110: vl_whole = VLEN >> 1;  // 32-bit: 8*VLEN/32
      5'b11111: vl_whole = VLEN >> 2;  // 64-bit: 8*VLEN/64
      default:  vl_whole = 12'd0;
    endcase
  end
  
  // Segment count calculation (matching ls_decode.scala)
  wire [3:0] seg_count = isSeg ? (instNf + 4'b0001) : 4'b0001;
  
  // Output assignments (matching ls_decode.scala priority order)
  assign o_base_v_reg = instVldDest;  // vs3 for stores
  
  // eew_enc (matching ls_decode.scala MuxLookup order)
  assign o_eew_enc = isMaskLS ? eew_mask : 
                     isIndex  ? eew_index : 
                     eew_normal;
                     
  // emul_enc (matching ls_decode.scala MuxLookup order)
  logic [2:0] emul_enc;
  assign emul_enc   = isWhole  ? whole_vlmul :
                      isIndex  ? emul_index :
                      isMaskLS ? emul_mask :
                      emul_normal;
  assign o_emul_enc = emul_enc[2] ? '0 : emul_enc; // override negative lmul with 0 since store buffer take the ceil of regs required
                      
  // vl (matching ls_decode.scala MuxLookup order)  
  assign o_vl = isWhole  ? vl_whole :
                isMaskLS ? vl_mask : 
                vl_normal;
  
  // Segment outputs (matching ls_decode.scala)
  assign o_seg_count = seg_count;

  // idx_mul
  assign o_idx_mul = isIndex && (instWidth > i_vtype_vsew) ? (instWidth - i_vtype_vsew) : 3'b000;

endmodule


// simply for assertions
module store_buffer_debug #(
  parameter VLEN = 256
) (
  input  logic                  clock,
  input  logic                  reset_n,

  input logic                    enq_ready, // OUTPUT
  input  logic                   enq_valid,
  input  logic                   enq_first,
  input  logic                   enq_last,
  input  logic [4:0]             enq_sb_id,
  input  logic [2:0]             enq_eew,
  input  logic [2:0]             enq_emul,
  input  logic [3:0]             enq_seg_count,
  input  logic [11:0]            enq_vl,
  input  logic [VLEN-1:0]        enq_data,

  input  logic                   return_credit,
  input  logic                   store_valid, // OUTPUT
  input  logic [511:0]           store_data // OUTPUT
);

  // ========= DECLARATIONS =========

  typedef struct packed {
    logic [4:0]  sb_id;
    logic [2:0]  eew;
    logic [2:0]  emul;
    logic [3:0]  seg_count;
    logic [11:0] vl;
  } enq_info_t;
  enq_info_t enq_info_reg;
  enq_info_t enq_info;

  typedef enum logic {
    WAIT_FOR_ENQ,
    ENQ_CHECK
  } state_t;
  state_t state;

  // ========= FSM LOGIC =========

  assign enq_info.sb_id = enq_sb_id;
  assign enq_info.eew = enq_eew;
  assign enq_info.emul = enq_emul;
  assign enq_info.seg_count = enq_seg_count;
  assign enq_info.vl = enq_vl;

  always_ff @(posedge clock) begin
    if (!reset_n) begin
      enq_info_reg <= '0;
      state <= WAIT_FOR_ENQ;
    end else begin
      if ((enq_ready && enq_valid) && enq_first) begin
        enq_info_reg <= enq_info;
        if (!enq_last) begin
          state <= ENQ_CHECK;
        end
      end
      else if ((enq_ready && enq_valid) && enq_last) begin
        state <= WAIT_FOR_ENQ;
      end
    end
  end

  // ========= DEBUG =========

  always_ff @(posedge clock) begin
    if ((enq_ready && enq_valid) && (state == WAIT_FOR_ENQ) && !enq_first) begin
      assert (enq_first)
        else $error("enq_first expected in WAIT_FOR_ENQ state (starting a transaction without the first signal)");
      // first and last together is fine
      assert (enq_first || !enq_last)
        else $error("enq_last is not allowed in WAIT_FOR_ENQ state (ending a transaction that never started)");
    end
    if ((enq_ready && enq_valid) && (state == ENQ_CHECK)) begin
      assert (enq_info_reg == enq_info)
        else $error("enq_info_reg != enq_info (enq_info changed in middle of an ongoing transaction)");
      assert (!enq_first)
        else $error("enq_first is not allowed in ENQ_CHECK state (starting a new transaction before ending the previous one)");
    end
  end

endmodule

