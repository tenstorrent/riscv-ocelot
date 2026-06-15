// See LICENSE.TT for license details.

// Ext
`define I_Ext 2'b00
`define M_Ext 2'b01
`define F_Ext 2'b10
`define V_Ext 2'b11

// Instrn Enc Type Format
//   0-7 :    00??? 	    = I_Ext
//     8 :    01000 	    = M_Ext
//     9 :    01001 	    = A_Ext
// 10-11 :    01010 - 01011 = B_Ext
// 12-15 :    01100 - 01111 = F_Ext
// 16-30 :    10000 - 11110 = V_Ext
//    31 :    11111 	    = Illegal

// I_Ext Enc Type
`define E 5'b00000
`define R 5'b00001
`define I 5'b00010
`define S 5'b00011
`define B 5'b00100
`define U 5'b00101
`define J 5'b00110
`define F 5'b00111

// M_Ext only
`define RM 5'b01000

// A_Ext only
`define A 5'b01001

// B_Ext only
`define IB 5'b01010
`define RB 5'b01011

// F_Ext only
`define RF 5'b01100
`define R4 5'b01101
`define IF 5'b01110
`define SF 5'b01111

// V_Ext Enc Type
// vec-csr	
`define C1 5'b10000
`define C2 5'b10001
`define C3 5'b10010
// Loads
`define L1 5'b10011
`define L2 5'b10100
`define L3 5'b10101
//STORES	
`define S1 5'b10110
`define S2 5'b10111
`define S3 5'b11000
// AMO	
// `define A1	
// `define A2	
// ARITHMETIC	
`define Vvv 5'b11001
`define Vvx 5'b11010
`define Vvi 5'b11011
`define Vm  5'b11100
`define Vsx 5'b11101
`define Vsf 5'b11110

// dest usage
`define rd 2'b01
`define fd 2'b10
`define vd 2'b11

// src usage
`define rs1 2'b01
`define rs2 2'b01
`define rs3 2'b01

`define fs1 2'b10
`define fs2 2'b10
`define fs3 2'b10

`define vs1 2'b11
`define vs2 2'b11
`define vs3 2'b11
`define v0 2'b00

`define uimm5 2'b00 // Eventually need to create valids from values
`define simm5 2'b00 // Eventually need to create valids from values

// V_Ext - Opcodes
`define V_LD 7'b0000111
`define V_ST 7'b0100111
`define V_AMO 7'b0101111
`define V_OP 7'b1010111
`define V_SET 7'b1010111 // Same Enc as V_OP

// V_OP funct3 Category
`define OPIVV 3'b000
`define OPFVV 3'b001
`define OPMVV 3'b010
`define OPIVI 3'b011
`define OPIVX 3'b100
`define OPFVF 3'b101
`define OPMVX 3'b110
`define OPCFG 3'b111

// V_SEW 
`define e8	3'b000 // 8
`define e16	3'b001 // 16
`define e32	3'b010 // 32
`define e64	3'b011 // 64
`define e128	3'b100 // 128
`define e256	3'b101 // 256
`define e512	3'b110 // 512
`define e1024	3'b111 // 1024

// V_LMUL
`define m1	3'b000 // 1
`define m2	3'b001 // 2
`define m4	3'b010 // 4
`define m8	3'b011 // 8
//`define -	3'b100 // -
`define mf8	3'b101 //  1/8
`define mf4	3'b110 //  1/4
`define mf2	3'b111 //  1/2

`define nf1	3'b000 // 1
`define nf2	3'b001 // 2
`define nf3	3'b010 // 3
`define nf4	3'b011 // 4
`define nf5	3'b100 // 5
`define nf6	3'b101 // 6
`define nf7	3'b110 // 7
`define nf8	3'b111 // 8

// Dont cares
`define rm 3'b---
