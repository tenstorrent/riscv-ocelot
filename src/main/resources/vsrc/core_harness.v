// Licensed under the Apache License, Version 2.0, see LICENSE.TT for details

module BoomCoreHarness
#(parameter
  coreMaxAddrBits = 40,
  retireWidth     = 3,
  xLen            = 64,
  vLen            = 256,
  lregSz          = 5,
  memWidth        = 1
  )
(
   BoomCoreHarnessIntf p0
);

import "DPI-C" function env_init();
import "DPI-C" function env_final();
import "DPI-C" function monitor_instr(input string name, input int hartid, input longint cycle, input longint tag, input longint pc, input int opcode, input int trap);   
import "DPI-C" function monitor_gpr(input string name, input int hartid, input longint cycle, input int rd_addr, input longint rd_wdata);
import "DPI-C" function monitor_fpr(input string name, input int hartid, input longint cycle, input int frd_addr, input longint frd_wdata);
import "DPI-C" function monitor_vr(input string name, input int hartid, input longint cycle, input int vrd_addr, input longint vrd_wdata[vLen/64]);
import "DPI-C" function monitor_csr(input string name, input int hartid, input longint cycle, input int csr_addr, input longint csr_wdata);

  // Info message logging
  string m    = $sformatf("%m");
  string prefix = m.substr(m.len() - 26, m.len() - 1);

  function automatic void info(string msg);
     $display({prefix, "-Harness: ", msg});
  endfunction
   
  // Init 
  int cosim = 0;
  int tracer = 0;
  initial begin
    #1; //wait one cycle for p0.hartid to get initialized
    if ($test$plusargs("cosim"))
      cosim = 1;
    if ($test$plusargs("harness_tracer"))
      tracer = 1;
    if (cosim && p0.hartid == 'h0)
      env_init();
  end

  // Cycle counting
  logic [63:0] cycle_cnt;
  always @(posedge p0.clock) begin
    if (p0.reset) begin
      cycle_cnt <= '0;
    end else begin
      cycle_cnt <= cycle_cnt + 1;
    end
  end

  // RTL harness connections for following events
  // Instruction retire
  // GPR write
  // FPR write
  // Vector write
  // CSR write
  int hart;
  longint cycle;
  longint tag;
  longint pc;
  int opcode;
  int trap;
  int rtype;
  int addr;
  longint data;
  int vec_addr;
  longint vec_wdata[vLen/64];
  int csr_addr;
  longint csr_wdata;

  always @(posedge p0.clock) begin
    for (int port_ix=0; port_ix<retireWidth; port_ix++) begin
      if (p0.commit.arch_valids[port_ix]) begin
        hart      = p0.hartid;
        cycle     = cycle_cnt;
        tag       = 0; //p0.commit.uops[port_ix].debug_tag;
        pc        = p0.commit.uops[port_ix].debug_pc[coreMaxAddrBits-1:0];
        opcode    = p0.commit.uops[port_ix].debug_inst;
        trap      = 0;
        rtype     = p0.commit.uops[port_ix].dst_rtype;
        addr      = p0.commit.uops[port_ix].ldst;
        data      = p0.commit.uops[port_ix].debug_wdata;

        // GPR
        if (rtype == 0) begin 
          if (tracer)
            info($sformatf("<%0d> GPR write: Hart=%0d, Addr=%0d, Data=0x%0x", cycle, hart, addr, data));
          if (cosim) 
            monitor_gpr("mon_instr", hart, cycle, addr, data);
        // FPR
        end else if (rtype == 1) begin
          if (tracer)
            info($sformatf("<%0d> FPR write: Hart=%0d, Addr=%0d, Data=0x%0x", cycle, hart, addr, data));
          if (cosim) 
            monitor_fpr("mon_instr", hart, cycle, addr, data);
        // Vector
        end else if (rtype == 4) begin
          for (int lmul=0; lmul<8; lmul++) begin
            if (p0.commit.uops[port_ix].debug_vec_wmask[lmul]) begin
              vec_addr = addr + lmul;
              if (tracer)
                info($sformatf("<%0d> VR write: Hart=%0d, Addr=%0d", cycle, hart, addr));
              for (int i=0; i<vLen/64; i++) begin
                case (lmul)
                  0: begin vec_wdata[i] = p0.commit.uops[port_ix].debug_vec_wdata[(vLen*lmul+64*i) +:64]; end
                  1: begin vec_wdata[i] = p0.commit.uops[port_ix].debug_vec_wdata[(vLen*lmul+64*i) +:64]; end
                  2: begin vec_wdata[i] = p0.commit.uops[port_ix].debug_vec_wdata[(vLen*lmul+64*i) +:64]; end
                  3: begin vec_wdata[i] = p0.commit.uops[port_ix].debug_vec_wdata[(vLen*lmul+64*i) +:64]; end
                  4: begin vec_wdata[i] = p0.commit.uops[port_ix].debug_vec_wdata[(vLen*lmul+64*i) +:64]; end
                  5: begin vec_wdata[i] = p0.commit.uops[port_ix].debug_vec_wdata[(vLen*lmul+64*i) +:64]; end
                  6: begin vec_wdata[i] = p0.commit.uops[port_ix].debug_vec_wdata[(vLen*lmul+64*i) +:64]; end
                  7: begin vec_wdata[i] = p0.commit.uops[port_ix].debug_vec_wdata[(vLen*lmul+64*i) +:64]; end
                endcase
              end
              if (cosim) 
                monitor_vr("mon_instr", hart, cycle, vec_addr, vec_wdata);
            end
          end
        end
        // Retire
        if (cosim) 
          monitor_instr("mon_instr", hart, cycle, tag, pc, opcode, trap);
      end
    end

    // CSR Write/Set/Clear events
    if (p0.csrwr.cmd == 3'h5 || p0.csrwr.cmd == 3'h6 || p0.csrwr.cmd == 3'h7) begin
      cycle = cycle_cnt;
      csr_addr = p0.csrwr.addr;
      if (p0.csrwr.cmd == 3'h5)
        csr_wdata = p0.csrwr.wdata;
      else if (p0.csrwr.cmd == 3'h6)
        csr_wdata = p0.csrwr.rdata | p0.csrwr.wdata;
      else if (p0.csrwr.cmd == 3'h7)
        csr_wdata = p0.csrwr.rdata & ~p0.csrwr.wdata;
      if (tracer)
        info($sformatf("<%0d> CSR write: Hart=%0d, CsrAddr=0x%0x, CsrData=0x%0x", cycle, hart, csr_addr, csr_wdata));
      if (cosim)
        monitor_csr("mon_instr", hart, cycle, csr_addr, csr_wdata);
    end
  end

  final begin
    if (cosim && p0.hartid == 0)
      env_final();
  end
  
endmodule
