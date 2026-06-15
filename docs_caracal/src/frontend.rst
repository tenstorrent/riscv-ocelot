Frontend
========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


The Frontend Stages
-------------------

The frontend includes 2 stages the Instruction Fetch and Decoder. Nominally the IF should consume
5 cycles and the Decoder 1 cycle, with support for super-scalar issue.

Instruction Fetch
-------------------

The Instruction fetch stage is unchanged from BOOMv4. The reader may review the BOOMv4 IFU here: 
`BOOM IFU docs <https://docs.boom-core.org/en/latest/sections/instruction-fetch-stage.html>`_.

.. _vector-rvv-decode:
Vector (RVV) Decode
-------------------

The Decoder is extended to support RVV1.0 instruction opcodes. The decoder itself is single cycle 
combinational module which produces the decoded uOP packet bundle. For |caracal|, parameterizable 
super-scalar decoder support is also maintained.

.. note::

   A uOP packet in BOOMv4 is simply a decoded instruction. The decoder does not perform any cracking 
   or micro-op expansion; all instructions, including vector instructions, are decoded into a single 
   uOP.

.. list-table::
   :header-rows: 1
   :widths: 1 2

   * - Parameter
     - Value
   * - RVV spec version
     - RVV1.0
   * - VLEN
     - 256
   * - ELEN
     - 64
   * - Supported SEW(s)
     - 4, 8, 16, 32, 64
   * - Supported LMUL(s)
     - ALL
   * - Vector regfile depth
     - 128

CII Shared Instruction Decoding
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Certain instructions may require shared resources between the vector load store units and
the vector co-processor. These instructions are marked as *is_shared* by the Decoder and
require special handling. 

Currently the only instruction type that will be marked *is_shared* is the segmented load store.
This is because segmented load stores must access memory using the LSU, and be transposed
using the co-processor transpose datapath.

The primary differentiator for shared instructions is their use of temporary vector registers
to hand off partial results between the the LSU and Co-processor via the bypass network. 
Shared instructions are only applicable to RVV1.0 instructions as only the CII attached co-processor
may want to read temporary results from the LSU and vice-versa.


VSET Special Handling
~~~~~~~~~~~~~~~~~~~~~~

Because we delay vector uOP cracking until the execution stage of the pipeline, 
the vector mapper must in advance allocate enough vector PRNs for a specific 
instruction based on LMUL/EMUL. Thus all uOP's emitted by the decoder must contain
the calculated EMUL of the instruction. This EMUL information is used by the 
Mapper to allocate PRN vector register groups.

We introduce a new unit called **Vector Config Unit**. The VCFG keeps a local copy 
of the VTYPE and VL values set by a vset instruction, so that dependent, subsequent 
instructions can know the most recent value of vset and be included in the uOP. 
The VCFG will also store performance counters and other related CSRs for vector 
instructions. The VCFG must also handle NxWide instruction decoder width, and 
handle the case where there are more than 1 vset instruction in a instruction bundle, 
the newest vset is used.

**vsetivli** is the best case instruction as the VTYPE and VL is an immediate value 
and can be immediately decoded and stored in the VCFG for decoding of subsequent vector instructions.

**vsetvli** is the most common case where VTYPE is an immediate but VL is supplied 
by a integer source register. In this case the decoder does not need to stall or wait 
for VL instead the uOP VL type is marked as a register, and the scalar value is 
resolved when the vector uOP enters the scalar scheduler.

**vsetvl** is a special case as both the VTYPE and VL is a scalar source operand instead
of an immediate encoded in the instruction. In this case the newer vector uOP cannot 
continue to be issued as the VTYPE value is needed by the next pipeline stage. 
To solve this issue we re-use the |boom| ``is_unique`` feature, and mark vsetvl as a unique
instruction. This forces all instructions in the BOOM pipeline to complete before vsetvl 
can be issued by the decoder. This allows the scalar instructions that calculate the vtype 
and VL value to complete, and the VCSRU reads this value and updates its local copy of VTYPE, 
VL and use it for subsequent vector uOP decoding.

This does result in poorer performance for this instruction, but this is acceptable as this vsetvl instruction type is not common.
