module BoomCoreHarnessWrapper_3
#(parameter VLEN=256)
(
   input clock,
   input reset,
   input [ 7:0] hartid,

   input [ 2:0] csrwr_cmd,
   input [11:0] csrwr_addr,
   input [63:0] csrwr_wdata,
   input [63:0] csrwr_rdata,

   input commit_arch_valids_0,
   input commit_arch_valids_1,
   input commit_arch_valids_2,

   input [ 4:0] commit_uops_0_ldst,
   input [ 4:0] commit_uops_1_ldst,
   input [ 4:0] commit_uops_2_ldst,
   
   input [ 2:0] commit_uops_0_dst_rtype,
   input [ 2:0] commit_uops_1_dst_rtype,
   input [ 2:0] commit_uops_2_dst_rtype,
   
   input [39:0] commit_uops_0_debug_pc,
   input [39:0] commit_uops_1_debug_pc,
   input [39:0] commit_uops_2_debug_pc,
   
   input [63:0] commit_uops_0_debug_tag,
   input [63:0] commit_uops_1_debug_tag,
   input [63:0] commit_uops_2_debug_tag,
   
   input [63:0] commit_uops_0_debug_wdata,
   input [63:0] commit_uops_1_debug_wdata,
   input [63:0] commit_uops_2_debug_wdata,
   
   input [VLEN*8-1:0] commit_uops_0_debug_vec_wdata,
   input [VLEN*8-1:0] commit_uops_1_debug_vec_wdata,
   input [VLEN*8-1:0] commit_uops_2_debug_vec_wdata,
   
   input [   7:0] commit_uops_0_debug_vec_wmask,
   input [   7:0] commit_uops_1_debug_vec_wmask,
   input [   7:0] commit_uops_2_debug_vec_wmask,
   
   input [31:0] commit_uops_0_debug_inst,
   input [31:0] commit_uops_1_debug_inst,
   input [31:0] commit_uops_2_debug_inst

);

   BoomCoreHarnessIntf #(.coreMaxAddrBits(40),
                         .retireWidth    ( 3),
                         .xLen           (64),
                         .vLen           (VLEN),
                         .lregSz         ( 5) ) dut_harness();

   assign dut_harness.clock = clock;
   assign dut_harness.reset = reset;
   assign dut_harness.hartid = hartid;

   assign dut_harness.csrwr.cmd   = csrwr_cmd;
   assign dut_harness.csrwr.addr  = csrwr_addr;
   assign dut_harness.csrwr.wdata = csrwr_wdata;
   assign dut_harness.csrwr.rdata = csrwr_rdata;

   assign dut_harness.commit.arch_valids[0] = commit_arch_valids_0;
   assign dut_harness.commit.arch_valids[1] = commit_arch_valids_1;
   assign dut_harness.commit.arch_valids[2] = commit_arch_valids_2;
   
   assign dut_harness.commit.uops[0].ldst = commit_uops_0_ldst;
   assign dut_harness.commit.uops[1].ldst = commit_uops_1_ldst;
   assign dut_harness.commit.uops[2].ldst = commit_uops_2_ldst;
   
   assign dut_harness.commit.uops[0].dst_rtype = commit_uops_0_dst_rtype;
   assign dut_harness.commit.uops[1].dst_rtype = commit_uops_1_dst_rtype;
   assign dut_harness.commit.uops[2].dst_rtype = commit_uops_2_dst_rtype;
   
   assign dut_harness.commit.uops[0].debug_pc = commit_uops_0_debug_pc;
   assign dut_harness.commit.uops[1].debug_pc = commit_uops_1_debug_pc;
   assign dut_harness.commit.uops[2].debug_pc = commit_uops_2_debug_pc;
   
   assign dut_harness.commit.uops[0].debug_tag = commit_uops_0_debug_tag;
   assign dut_harness.commit.uops[1].debug_tag = commit_uops_1_debug_tag;
   assign dut_harness.commit.uops[2].debug_tag = commit_uops_2_debug_tag;
   
   assign dut_harness.commit.uops[0].debug_wdata = commit_uops_0_debug_wdata;
   assign dut_harness.commit.uops[1].debug_wdata = commit_uops_1_debug_wdata;
   assign dut_harness.commit.uops[2].debug_wdata = commit_uops_2_debug_wdata;
   
   assign dut_harness.commit.uops[0].debug_vec_wdata = commit_uops_0_debug_vec_wdata;
   assign dut_harness.commit.uops[1].debug_vec_wdata = commit_uops_1_debug_vec_wdata;
   assign dut_harness.commit.uops[2].debug_vec_wdata = commit_uops_2_debug_vec_wdata;
   
   assign dut_harness.commit.uops[0].debug_vec_wmask = commit_uops_0_debug_vec_wmask;
   assign dut_harness.commit.uops[1].debug_vec_wmask = commit_uops_1_debug_vec_wmask;
   assign dut_harness.commit.uops[2].debug_vec_wmask = commit_uops_2_debug_vec_wmask;
   
   assign dut_harness.commit.uops[0].debug_inst = commit_uops_0_debug_inst;
   assign dut_harness.commit.uops[1].debug_inst = commit_uops_1_debug_inst;
   assign dut_harness.commit.uops[2].debug_inst = commit_uops_2_debug_inst;
   
   BoomCoreHarness
   #(.coreMaxAddrBits(40),
     .retireWidth    (3 ),
     .xLen           (64),
     .vLen           (VLEN),
     .lregSz         (5 ) )
   SV_instance
   (dut_harness);

endmodule
