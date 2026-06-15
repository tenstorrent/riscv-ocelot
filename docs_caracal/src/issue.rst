Issue
=====

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


The Issue Units
---------------

Similar to the existing |boom| Issue Unit, |caracal| also uses split issue queues: Vector Load
IQ; Vector Store IQ; and Vector Arithmetic/CII IQ. Each vector IQ may handle any datatype.


We reuse |boom|'s Issue Queue implementation which provides queues with issue slots selected by a
priority encoder once the RF or Bypass network broadcasts operand availability. 


|caracal| extends the Issue Stage into 2 pipeline stages. The first stage which we wil call the
Scalar Scheduling stage will be nearly identically to the exiting Issue Unit, scalar and vector
issue queues will hold respective uOP's and wait for ONLY dependent scalar register operands to become available.
For Scalar Scheduling |caracal| will use |boom|'s Age Ordered Issue Queue policy.


Dispatch Stage
--------------

In BOOM, dispatch is the pipeline stage that sits between rename and the issue queues 
— it's the last in-order stage before instructions go out-of-order. 
This stage routes the instruction into the appropiate IQ and and reserves the LDQ/STQ
slot in program order, which only the in-order dispatch stage can do. 
The dispatch stage is essentially unchanged from BOOM except modified to support vector 
op-codes.


Scalar Scheduling/Issue
~~~~~~~~~~~~~~~~~~~~~~~

Scalar uOP's selected for issue will be sent to their respective ports to be executed by a suitable
functional unit. This behavior is unchanged from |boom|. Vector OP.v's from the Vector Load
IQ; Vector Store IQ; and Vector Arithmetic/CII IQ selected for issue will be sent to 
the second stage of Instruction Queues for vector instruction scheduling. 

Vector Scheduling/Issue
~~~~~~~~~~~~~~~~~~~~~~~

The second stage which we will call the Vector Scheduling stage, will only contain the three 
vector Issue Queues. The vector scheduler will hold vector OP.v's and wait for ONLY dependent vector register
operands to become available. Stage 2 Vector Scheduling instruction queues will also
use |boom|'s Age Ordered Issue Queue policy. 

Vector OPs from the stage 2 Vector Load IQ or Vector Store IQ selected for issue will be sent to the Vector
LS AGEN stage. See :ref:`vector-agen`. For vector load and store OPs the VAGEN will perform
nano-operation cracking which will generate per vector element nOP bundles for each Strided, Indexed, and 
Segmented load store OP. Under Age Ordered Issue Policy different load/store OP.v's may be selected for issue OOO,
but nOP.v bundles must be issued atomically to LSU as a single bundle and execute in-order.

Some minor modifications are made to the stage 1 Instruction Queues for vector instructions, such
as widening the operands, VL broadcast and update, wiring to int/fp register file, and slot specialization. 
Stage 1 vector IQs must add support for VL broadcast and update, when the dependent source integer register
containing the VL value becomes available the vector IQs must read the VL register and update the OP.v vl field.


CII Shared Instruction Scheduling
---------------------------------

Shared instructions may enter the stage 1 Scalar Scheduler, and be processed as any other instruction.
Once a shared instructions enter the stage 2 Vector Scheduling, it requires special handling.
When a segmented load or store instruction is issued from its respective vector IQ, the vector
scheduler will dispatch it to BOTH the CII IP and its own respective Load/Store IQ. 

We currently only support shared vector load store instructions, of those only Segmented instructions
are marked shared by the Decoder. A shared vector arithmetic instruction does not make sense as the 
co-processor fully manages its resources for execution.

Once a shared instruction has been issued to the vector load or store IQ and the CII IQ, the stage 2 
vector IQs must use different logic for wakeup. There are 2 cases:

Segmented Load
~~~~~~~~~~~~~~

1. A segmented load is a shared instruction where the load unit must use the vsrc PRN as the destination temporary register, that is broadcast and buffered only within the bypass network.
2. For segmented load the LSU does not update the VPRF.
3. A segmented load is selected for issue to the LSU when all of its vector source operands are available.


Segmented Store
~~~~~~~~~~~~~~~
1. A segmented store is a shared instruction where the store unit must use the vsrc PRN as the source temporary register, that is broadcast and buffered only within the bypass network.
2. For segmented store the LSU does not read from the VPRF.
3. A segmented store is selected for issue to the LSU when the bypass network wakes the instruction in the stage 2 vector store IQ and broadcasts that the vsrc is available in the Temporary Register Buffer.


Scalar and Vector Scheduling Key Features
-----------------------------------------

This dual stage approach for scalar and vector Issue Queues offers the following benefits:

1. The scalar datapath is not impacted by vector datapath.
2. The VL scalar value is resolved during scalar scheduling.
3. Enables a detached CII co-processor, with temporary registers and shared instructions.
4. Cracking of vector instructions in the frontend is no longer necessary. This is also made possible
   by the atomic LMUL vector mapper.




