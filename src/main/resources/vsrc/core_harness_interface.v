interface BoomCoreHarnessIntf
#(parameter
   coreMaxAddrBits = 40,
   retireWidth     = 3,
   xLen            = 64,
   vLen            = 256,
   lregSz          = 5,
   memWidth        = 1 );

   typedef struct packed {
       logic [         lregSz-1:0] ldst;
       logic [                2:0] dst_rtype; // 0:RT_FIX, 1:RT_FLT, 2:RT_X, 3: RT_PAS, 4: RT_VEC
       logic [coreMaxAddrBits-1:0] debug_pc;
       logic [               63:0] debug_tag;
       logic [           xLen-1:0] debug_wdata;
       logic [         vLen*8-1:0] debug_vec_wdata;
       logic [                7:0] debug_vec_wmask;
       logic [               31:0] debug_inst;
   } MicroOp_t;
   
   typedef struct packed {
       logic     [retireWidth-1:0] arch_valids;
       MicroOp_t [retireWidth-1:0] uops;
   } CommitSignals_t;

   typedef struct packed {
       logic [     2:0] cmd; // 0:Nop, 2:Read, 4:SystemInsn, 5:Write, 6:Set, 7:Clear
       logic [    11:0] addr;
       logic [xLen-1:0] wdata;
       logic [xLen-1:0] rdata;
   } CSRWrite_t;

   logic           clock;
   logic           reset;
   logic [7:0]     hartid;
   CommitSignals_t commit;   
   CSRWrite_t      csrwr;

endinterface
