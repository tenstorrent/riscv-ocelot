Loadstore
=========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC

The Unified Load/Store Unit (LSU)
---------------------------------

A major feature of |caracal| is that it extends the |boom| Load Store Unit to support
scalar and vector load stores. We call this the Unified Load Store Unit.

If the usingRVV paramaeter is disabled the underlying |boom| LSU is essentially untouched, when
enabled several vector address and data queues are initialized. Vector load stores OP.v's are still
allocated to the existing STQ and LDQ as a single entry, using largely the same dispatch stage 
logic. 

As vector memory accesses often produce many D$ accesses to read or write a whole vector,
we allocate separate vector address and data queues to buffer those effective nOP.v memory operations. 
This prevents the primary STG and LDQ from being polluted by many vector memory operations and impacting
scalar load store performance. When a vector LDQ or STQ entry becomes ready for execution, it reads the 
effective address or store data from separate address and data queues, rather than from a uOP as is the 
case with scalar load stores. 

The conditions that allow a vector LDQ or STQ entry to become eligible to drain are the same as scalar. 
Vector load stores also obey special conditions such as AMO and handle exceptions with the same behavior.
Critically, vector loads and stores must execute atomically, meaning once a STQ or LDQ entry is valid it
must drain the vector address queue of all addresses it owns in-order and then commit.

For LargeBoomV4Config or MegaBoomV4Config configurations we take advantage of the dual port L1 D$ interface,
and allow 2 vector memory operations to be issued per cycle. The vector address and data queues are 2xnOP.v
wide to allow 2 concurrent element memory accesses.


Strided, Segmented, Indexed Queue
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This class of vector (SSI) queues holds effective address calculated for strided, indexed, and segmented 
load stores. There exists a ld_SSI_ADDR_Q and a st_SSI_Q. 

Because the SSI queues hold essentially element wise memory addresses, its default parameterized size is 
quite big at 64 entries. The ld_SSI_ADDR_Q and a st_SSI_Q receives calculated effective addresses from the
stage 1 vAGENs.


Unit-Strided Queue
~~~~~~~~~~~~~~~~~~

We specially optimize unit-strided load store performance. Unit-Strided load stores have very simple 
address generation and only require a base address, VL, and EEW to generate all addresses. Thus we
have a separate ld_US_ADDR_Q and st_US_Q to hold these transactions. Unlike SSI queues a unit-stride load
store need only generate a single nOP.v from the stage 1 vAGENs.

A second Packer based AGEN unit will fire when a unit-strided STQ/LDQ entry is activated. 


Store Data Queue
~~~~~~~~~~~~~~~~
Instead of holding the store data in the STQ entry, which would not be feasible for vector data.
A separate st_SSI_DATA_Q and st_US_DATA_Q is initialized to buffer the store data. The st_SSI_DATA_Q
will have a data width of 64 bits, or ELEN. The st_US_DATA_Q will have a data width of 256 bits, or VLEN.


Vector Loads Algorithm
-----------------------
1. Dispatch — slot reserved in-order by dispatch stage.
2. Issue - AGen — when the load issues out of the issue queue, it drains from the ld_SSI_ADDR_Q or ls_US_Q.  
3. Fire — the same cycle, will_fire_load_agen_exec does TLB + D$ + LCAM together

Identical to scalar stores other than it drains address from dedicated vector queues.

Vector Stores Algorithm
-----------------------
1. Dispatch — slot reserved in-order by dispatch stage.
2. Execute — Does TLB + LCAM only, it translates the address and searches the load queue for ordering violations.  Draining from the st_SSI_Q or st_US_Q.
3. Commit — When the ROB retires the store, its committed flag is set. Only then does it become eligible to drain to the stq_execute_queue.
4. Fire —  stq_execute_queue drains and then the actual D$ write occurs.

Identical to scalar stores other than it drains address and data from dedicated vector queues.


Memory SubSystem
----------------

The memory subsystem remains unchanged from BOOMv4. However we recommend to use 
LargeBoomV4Config or MegaBoomV4Config configurations to enable the dual port L1 
DCache for best performance.