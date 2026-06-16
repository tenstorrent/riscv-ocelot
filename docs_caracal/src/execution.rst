Execution
=========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


The Execution Pipelines
-----------------------

|caracal| does not modify any |boom| execution pipeline, scalar execution is untouched.
The execution of vector instructions is split into 3 streams:

1. Vector Arithmetic, Reduction, Permutation
2. Vector Loads
3. Vector Stores
   
Vector instruction OP.v's may execute out-of-order relative to program order, but must always execute atomically
and in element order. Shared instructions require more than 1 EU to activate and share temporary results through 
vector bypass networks TVRB_RF.


Vector LS AGEN stage
--------------------

This stages sits between the ``IQ_V_LOAD``/``IQ_V_STORE`` issue queues and the Unified Load Store Unit.
Its primary purpose is to perform cracking of L/S OP.v instructions into nOP.v which access memory at the element
granularity. It also cracks OP.v instructions based on EMUL, so nOP.v also tracks to which PRN and offset within the PRN
the memory access should read or write to.

The vector LS AGEN stage outputs these nOP.v bundles which may be very large to dedicated queues with in the LSU.

.. _vector-agen:

Vector AGEN
~~~~~~~~~~~

Once vector load store instructions are issued from the ``IQ_V_LOAD``/``IQ_V_STORE`` queues, they enter the vector AGEN stage.
The vector AGEN stage contains the load vAGEN, store vAGEN/vDGEN. Vector OP.v's issued by 
the CII IQ are directly forwarded to the co-processor via the CII interface.

We reuse the Load/StorePacker, Load/StoreSkipper, Load/StoreWalker AGEN units from bobtail.

Packer
^^^^^^
Handles the fastest, densest case: contiguous/unit-stride loads with no 
indexing. It packs multiple segments and multiple elements into each DMEM-width packet, 
advancing a single EEW_CTR by whichever constraint (VL, vreg boundary, DMEM boundary, 
segment, or mask) is hit first. Supports masked loads (masked-off lanes are still fetched 
but flagged), but not indexed loads.

Skipper
^^^^^^^
Handles masked, non-indexed loads where masked-off elements should 
be skipped rather than fetched. It packs whole segments per element but uses the mask bits 
(via a priority encoder) to jump past runs of disabled elements in power-of-2 strides, 
emitting "fake" packets for skipped regions. Use this instead of the packer when a mask 
is present, and instead of the walker when there is no index.

Walker
^^^^^^
Handles indexed loads (and the general element-by-element fallback) 
by walking through elements one at a time, taking a per-element byte offset and mask bit 
from the index interface. Segments are still packed per element, but each element gets 
its own computed address and direction; to avoid stall/wait states it won't release the 
final segment or start until the next index arrives. Use this for any indexed load or 
indexed-masked load — it does not support non-indexed masked loads which the skipper will handle.

|caracal| splits the vAGEN into two stages. The first stage is after the issue stage (the
``IQ_V_LOAD``/``IQ_V_STORE`` queues) where load and store OP.v's are issued to ld_vAGEN_1 and
st_vagen_1, respectively.
The stage 1 vAGENs will only contain the Skipper and Walker generators. This choice is taken to
optimize the common unit-stride load/store case, and reduces the amount of pressure on the load and store
queues. Thus in stage 1 AGEN only strided, indexed, and segmented load stores will have effective addresses
calculated and expanded into nOP.v bundles to be sent to the load store unit. Unit strided OP.v load stores
will be encoded in a single nOP.v containing the effective base address, effective stride, and is_unit_stride 
flag set high. The stage 2 AGEN's Packer will see that this nOP.v is a unit stride operation and generate 
all of the effective nOP.v *just-in-time* at the load/store queues in the Unified Load Store Unit.


.. _vector-dgen:

Vector DGEN
~~~~~~~~~~~

st_vdgen for vector stores data generation works along side st_vagen_1. The st_vdgen will read from the
vector register file for source vector operands or the FP/INT register file for scalar source operands
, for instance vfmul.vf. 


For strided, indexed, and segmented (SSI) load/stores, st_vdgen may read an entire vPRN and alongside
the calculated effective address from the st_vagen_1 be dispatched to LSU SSI_Q's. SSI_Q's are ELEN
wide so the st_vdgen reads and buffers the entire vPRN read value, and generates a address and 
64 bit data bundle that will be issued to the LSU SSI address and data queues.

These nOP.v bundles must be dispatched to the LSU store queues in-order and atomically. Each nOP.v
must be processed in program and memory order, to maintain precise exception tracking via 
VSTART. As data within vPRN's is already packed each effective address corresponds to one 
consecutive element of the vPRN.

For unit-stride load/stores, the st_vdgen may read an entire vPRN and pass the entire VLEN
read data alongside the effective base address from st_vagen_1 to be dispatched to LSU.
The LSU will have seperate dedicated queues for the unit-stride load/store nOP.v's, and
generate per-element memeory accesses sequentially.



.. _vector-execution:

CII Vector Co-processor
-----------------------

The vector Coprocessor (VPU) will be issued instructions from the CII IQ. These instructions
include all vector arithmetic, reduction, permutation, and shared instructions such as segmented LS.
The VPU will reuse the Baby RISCV Vector Unit from bobtail, with modifications to implement the CII interface.
Improvements from the previous generation include a data transpose unit to support segmented LS. 



Tenstorrent Custom Instruction Interface (tt_CII)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This SPEC is Work In Progress. The main features should include:

1. Implement a PULL model of execution where the CoProcessor must request register operands.
2. In-order execution of dispatched instructions.
3. Usage of temporary registers, for intermediate values.
4. Extensible interface for custom instructions.
5. Ability of CII to access register file for reads and writes.