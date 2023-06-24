// See LICENSE.TT for license details.

`include "briscv_defines.h"

module tt_ascii_instrn_decode
(
  /* verilator lint_off UNUSED */
  input      [31:0] i_instrn,
  /* verilator lint_on UNUSED */
  output reg [63:0] o_ascii_instrn
);

wire [6:0] opcode = i_instrn[6:0];
wire [2:0] func = i_instrn[14:12];
wire       func_secondary = i_instrn[30];

/* verilator lint_off CASEINCOMPLETE */
//spyglass disable_block STARC05-2.10.3.2b_sa
//spyglass disable_block STARC05-2.10.3.2b_sb
//spyglass disable_block MatchWidthOnAssign
always @* begin
  o_ascii_instrn = "ILLEG";
  casez(opcode)
    7'b1101111: o_ascii_instrn = "JAL";
    7'b1100111: o_ascii_instrn = "JALR";
    7'b1100011: // branches
    begin
      casez(func)
        3'b000: o_ascii_instrn = "BEQ";
        3'b001: o_ascii_instrn = "BNE";
        3'b100: o_ascii_instrn = "BLT";
        3'b101: o_ascii_instrn = "BGE";
        3'b110: o_ascii_instrn = "BLTU";
        3'b111: o_ascii_instrn = "BGEU";
      endcase            
    end
    7'b0000011: // loads
    begin
      casez(func)
        3'b000: o_ascii_instrn = "LB";
        3'b001: o_ascii_instrn = "LH";
        3'b010: o_ascii_instrn = "LW";
        3'b100: o_ascii_instrn = "LBU";
        3'b101: o_ascii_instrn = "LHU";
      endcase      
    end
    7'b0100011: // stores
    begin
      casez(func)
        3'b000: o_ascii_instrn = "SB";
        3'b001: o_ascii_instrn = "SH";
        3'b010: o_ascii_instrn = "SW";
      endcase
    end 
    7'b0010011: // ALU with immediate 
    begin
      casez(func)
        3'b000: o_ascii_instrn = "ADDI";
        3'b010: o_ascii_instrn = "SLTI";
        3'b011: o_ascii_instrn = "SLTIU";
        3'b100: o_ascii_instrn = "XORI";
        3'b110: o_ascii_instrn = "ORI";
        3'b111: o_ascii_instrn = "ANDI";
        3'b001: o_ascii_instrn = "SLLI";
        3'b101:
        begin
          if(!func_secondary) o_ascii_instrn = "SRLI";
          else o_ascii_instrn = "SRAI";
        end
      endcase
    end
    7'b0110011: // ALU with both input ops registers
    begin
      casez(func)
        3'b000: 
        begin
          if(!func_secondary) o_ascii_instrn = "ADD";
          else o_ascii_instrn = "SUB";
        end
        3'b001: o_ascii_instrn = "SLL";
        3'b010: o_ascii_instrn = "SLT";
        3'b011: o_ascii_instrn = "SLTU";
        3'b100: o_ascii_instrn = "XOR";
        3'b101:
        begin
          if(!func_secondary) o_ascii_instrn = "SRL";
          else o_ascii_instrn = "SRA";
        end
        3'b110: o_ascii_instrn = "OR"; 
        3'b111: o_ascii_instrn = "AND";
      endcase
    end
    7'b0110111: o_ascii_instrn = "LUI";
    7'b0010111: o_ascii_instrn = "AUIPC";
    7'b1110011: // CSR
    begin
      casez(func)
        3'b001: o_ascii_instrn = "CSRRW";
        3'b010: o_ascii_instrn = "CSRRS";
        3'b011: o_ascii_instrn = "CSRRC";
        3'b101: o_ascii_instrn = "CSRRWI";
        3'b110: o_ascii_instrn = "CSRRSI";
        3'b111: o_ascii_instrn = "CSRRCI";
      endcase
    end 
  endcase
end
/* verilator lint_on CASEINCOMPLETE */
//spyglass enable_block STARC05-2.10.3.2b_sa
//spyglass enable_block MatchWidthOnAssign

endmodule
