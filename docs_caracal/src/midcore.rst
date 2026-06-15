Midcore
=======

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC

The Rename Stage
----------------

The Rename Stage is extended to support vector register renaming. For vector instructions the rename stage is split into two pipeline stages **Scalar Mapping/Rename** and **Vector Mapping/Rename**.

In the scalar mapping the original |boom| implementation is unchanged, there still exists a integer rename_stage and fp_rename_stage. Scalar instructions may enter the rename stage and be dispatched normally to the scalar instruction queues.

Vector instructions will also enter the rename_stage and fp_rename_stage and have any scalar destination or source registers assigned a int or fp PRN. For vsetivl the VL register will require a integer PRN to be allocated for the VL value, which will be resolved in the scalar scheduling stage.

Rename Map Table (RMT)
~~~~~~~~~~~~~~~~~~~~~~~

|boom| maintains two sets of register rename map tables **Speculative RMT** and **Committed RMT**. The Committed RMT is an optional feature, which we we will always keep enabled in |caracal|.

1. Speculative RMT — ``map_table``

   This is the working table updated at rename time as physical destinations are allocated. It reflects in-flight, not-yet-committed instructions. Source operands are read from it (``map_resps``).

2. Architectural / Committed RMT — ``com_map_table``

   Updated only by committed uop's (driven from the ROB via ``com_remap_reqs``). It holds the known-good architectural state.

When an exception occurs, the ``com_map_table`` is copied to the speculative ``map_table``. Because the ``com_map_table`` only contains the latest committed ARN to PRN mappings it holds the most recent correct state before the trapping instruction, thus instead of a ROB 1 entry/cycle walk back a single cycle copy is all that is needed to roll back the ``map_table``.

The vector mapper extends the existing RenameStage implementation to support vector renaming. The vector mapper implements atomic LMUL/EMUL based vector register mapping. The mapper may rename and allocate up to LMUL=8 PRNs for a single vdest. This removes the need for vector uOP cracking after the decoder and reduces the number of ROB entries required for vector instructions.

.. note::

   The Vector Mapper should also get the old stale vdest reg and update the OP.v ``stale_pvdest`` field. This will assist in handling tail undisturbed instructions.

The vector mapper adds four features:

**1. LMUL TAG Whole Vector Group Checker**

We add an additional structure to the RMT for vector remapping. The LMUL tag checker adds a 2 bit tracking table for each of the ARN indexes to track which vector register group the ARN index belongs to. The tag checker table will consume 32x2 bits.

.. list-table::
   :header-rows: 1
   :widths: 1 1

   * - LMUL
     - Encoding
   * - LMUL=1
     - 2'b00
   * - LMUL=2
     - 2'b01
   * - LMUL=4
     - 2'b10
   * - LMUL=8
     - 2'b11

We can exploit the fact that vector register groups must be begin on ARN indexes that must be a multiple of the effective LMUL. So we build combinational checkers that check whether or not each valid ARN base index represents a whole vector group or not.

For example to check vector group Tags for LMUL = 8:

.. code-block:: python

   LMUL = 8
   vdest = [0, 8, 16, 24]              # valid vector-group base ARNs for LMUL=8
   is_whole_vg_8 = [0] * (32 // LMUL)  # one flag per group

   for i, vd in enumerate(vdest):
       is_whole_vg_8[i] = int(
           tag_table[vd]     == tag_table[vd + 1] == tag_table[vd + 2] ==
           tag_table[vd + 3] == tag_table[vd + 4] == tag_table[vd + 5] ==
           tag_table[vd + 6] == tag_table[vd + 7] == 0b11
       )

The same combinational check is performed for other LMULs and their valid vdest indexes. For LMUL=1 all ARN indexes would always return true.

If the check for a is_whole_vg_[1,2,4,8] returns TRUE, then the vector mapper can immediately read the RMT and get the most recent vsrc PRN mappings for an entire vgroup and update the OP.v with the PRN indexes.

If the check for a is_whole_vg_[1,2,4,8] returns FALSE, then the LMUL Tag Miss Handler must activate and step through the vector RMT Snaphots and find the ARN to PRN vector group before it was fragmented. This should be an extremely rare state.

**2. Periodic Vector RMT Snapshots**

We take a snapshot of the RMT every 32 OP.v instructions OR whenever there is a branch instrcution.

See Section :ref:`snapshots`

The periodic snapshoting handles 1 case where the tag is_whole_vg_[1,2,4,8] check returns false, and the Miss Handler needs to go find the correct PRN.

**3. LMUL Tag Miss Handler**

TODO. SKip this implmentation for now.

**4. Widened RMT Interfaces**

To support atomic LMUL based PRN allocation and ARN source/dest renaming the Vector RMT must be widened to read up to 8 PRNs per cycle. BOOM's scalar RMT has 4xW Read Ports and 1xW Write port, where W is the number of instructions wide issue configuration. Thus each Port must have a width of 8xlog2(NUM_PHYS_VREG).


CII Shared Instruction Mapping
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Up to this point both the scalar and vector mapper treat shared instructions as any 
other instruction. The differences in handling begin in the Issue Stage, Vector Scheduling.

Shared vector instructions are still allocated PRN's and ROB entries as normal instructions.


.. _free-list:

Free List
~~~~~~~~~


The Free List tracks the physical registers that are currently un-used and 
is used to allocate new physical registers to instructions passing through the Rename stage

Modification is required for the Vector Free List. Because we rename whole vector 
groups atomically, the vector free list mist be able to supply up to 8 PREG per request. 
Thus each port must have a width of 8, to change upto 8 elements in the bit vector per cycle.


Busy Table
~~~~~~~~~~

The Busy Table tracks the readiness status of each physical register. If all physical operands are 
ready, the instruction will be ready to be issued.

The Vector Busy Table will operate in a similar fashion to the scalar Busy Table. However, 
the Vector Busy Table will need additional ports to support 4 vector source reads.



Reorder Buffer (ROB)
~~~~~~~~~~~~~~~~~~~~~

|caracal| reuses |boom| v4's ROB structurally unchanged. The only field change is
widening the entry's ``dst_rtype`` to 3 bits (``exu/rob.scala``) to encode ``RT_VEC``,
so vector uop's allocate, commit, and roll back through the same
head-pointer/exception machinery as scalar ops.

Because the vector mapper allocates whole ``LMUL``/``EMUL`` register groups atomically
(see :ref:`snapshots`), a vector instruction is **not** cracked into one uop per
destination register. It therefore occupies **a single ROB entry**, regardless of
``EMUL``. This keeps the ROB small under wide vectors and lets vector instructions
retire in one commit slot.

Completion tracking
^^^^^^^^^^^^^^^^^^^^

A single ROB entry that owns an ``EMUL``-wide destination group needs to know when
**all** of the group's writes are done before it can clear its busy bit and commit.
How that is detected differs between the two vector paths:

Vector arithmetic (tt_CII)
   The vector ALU path executes **in program order on the in-order tt_CII
   coprocessor** (see :ref:`vector-execution`). An in-order unit knows when an entire
   ``op.v`` has retired, so it signals completion **once per instruction**. This maps
   directly onto the existing single-writeback busy-clear in ``rob.scala`` — **no
   per-destination counter is required** for CII ops. The ROB treats a CII completion
   exactly like a scalar writeback: one wakeup clears ``rob_bsy`` for the entry.

Out-of-order vector load/store (VLS)
   The vector memory path generates element/segment sub-accesses that complete
   **independently and out of order** through the LSU. Here a single writeback is not
   sufficient — the entry must remain busy until every sub-access has reported back.
   For this path the ROB entry carries a small **completion counter** (4 bits;
   ``EMUL ≤ 8``, and segment ``NF·EMUL ≤ 8``). The counter is initialized at dispatch to
   the number of expected destination writes — derived from ``v_emul`` and ``v_seg_nf``,
   **not** raw ``EMUL`` (widening/narrowing change the destination group size, and
   mask/reduction results are a single register). Each writeback (or ``lsu_clr_bsy``
   for stores) increments the counter by the number of ports matching the entry that
   cycle; ``rob_bsy`` is cleared only when the counter reaches its target. Scalar ops
   are the ``target = 1`` case and behave exactly as today.

The ``rob_unsafe`` (speculation-hazard) bit is gated the same way for VLS: a
multi-access vector load is not memory-safe until **all** of its sub-accesses have
disambiguated, so ``rob_unsafe`` is cleared only once the whole group reports safe,
not on the first sub-access.


Commit
~~~~~~

At retirement a vector entry frees its entire **stale destination group** — ``EMUL``
stale vector pregs, not one — so the ROB carries the stale group base alongside the
new ``pvdest``. This couples to the atomic free-list deallocation path described in
:ref:`free-list`.


.. _snapshots:

Checkpoint RMT Snapshots
~~~~~~~~~~~~~~~~~~~~~~~~

BOOM implements a branch snaphot mechnism to roll back the RMT upon branch mispredicts in a 
single cycle. A copy of the speculative scalar RMTs is taken per outstanding branch 
(upto maxBrCount entries).

We will slightly extend this mecahnism for the vector mapper.

The vector mapper will take a snapshot of the speculative RMT every 32 vector instructions 
OR whenever ther is a branch instruction that results in a scalar RMT branch snapshot.

.. list-table::
   :header-rows: 1
   :widths: 1 1 2

   * - Event
     - Recovery source
     - Cost
   * - Branch mispredict
     - ``br_snapshots(br_tag)``
     - 1 cycle, only flushes younger-than-branch
   * - Exception / pipeline flush
     - ``com_map_table``
     - 1 cycle, flushes everything in flight
   * - Normal
     - ``remap_table``
     - advance




Precise exceptions for vector ops
---------------------------------

A vector load/store can fault partway through its element stream (e.g. element *k* raises a page fault). 
Because the whole instruction is one ROB entry, the trap must be made restartable by recording ``vstart = k`` 
for the **oldest** faulting element. The single ``rob_exception`` bit is therefore augmented on the VLS path 
with the faulting element index (carried in on the LSU exception port).

On trap, ``vstart`` is written from this index so the instruction resumes mid-stream per the RVV 1.0 spec. 
CII arithmetic ops, which complete atomically, raise exceptions at instruction granularity and need no element index.

On vector loads if we load upto *k* elements and an exception occurs, we do not need to 
restore the remaining ELEN - *k* elements and can treat it as tail agnostic. 
This is thanks to the relaxation of rules for precise exception handling in section 17.1 RVV Specification.


Segmented Load/Store (Shared Instruction)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In addition, to the above procedures segmented load store instructions must also invalidate any TVRB entries which are
allocated for its execution. Segmented load stores should still commit data up until the point of exception. For segmented loads
this means the vsrc temporary register should be written to up until the point of execution and tail values may hold junk.
The Coprocessor should still read and free the vsrc temporary register and write the result to the VPRF. A segmented store will
always read and free the vsrc temporary register, but may only write data to memory up until the point of exception.


The Register Files and Bypass Network
-------------------------------------

|caracal| shall be configured to support an performant number of physical registers via
parameterization. 

.. list-table::
   :header-rows: 1
   :widths: 1 1 1

   * - Register file
     - Architectural regs
     - Physical regs (default)
   * - Integer (INT)
     - 32
     - 128
   * - Floating-point (FP)
     - 32
     - 128
   * - Vector (VEC)
     - 32
     - 128

.. note::

   |caracal|/|boom| has no dedicated mask **register file**; Vector mask is treated 
   like any other vector register, with masking semantics handled in the execution units.



Vector Bypass Network and Temporary Register Buffer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

|boom| supports a full operand forwarding bypass network for the integer and FP pipeline.
We also must implement the bypass network for the vector pipeline. However, the vector bypass 
network extends the scalar bypass networks with a **Temporary Vector Register Buffer** (TVRB).
The Temporary Vector Register Buffer serves an important role to support temporary vector registers
and shared instructions. The TVRB will be a relatively small register file parameterized to support a
default 16 temporary vector registers. TVRB register entires are dynamically allocated when an execution
unit has a value that they want stored in a temporary register. The TVRB maintains a PRN entry table
that uses the renamed PRN index from the OP.v to point to an index of the TVRB_RF. Free TVRB_RF indexes
are tracked with a free list. TVRB_RF entries are deallocated when an EU reads from the TVRB using
the vsrc PRN, and the index is marked free.


Segmented Load
~~~~~~~~~~~~~~

1. The LSU will begin executing the segmented load when vsrc operand are avalible in the vPRF.
2. The LSU reads from memory.
3. When executing a segmented load the LSU uses the vsrc PRN as the destination temporary register and will store the value in the TVRB.
4. The TVRB will notify the CII IQ that the temp register is available, and the Coprocessor can begin execution.
5. The Coprocessor will regard the vsrc as a temp register and only access the vector bypass network.
6. The Coprocessor will perform the necessary transform of the data and store the final result to the vdest PRN in the VPRF.  


Segmented Store
~~~~~~~~~~~~~~~

1. The Coprocessor will execute the segmented store when all source operands become available.
2. The Coprocessor will perform the necessary transforms on the data and write the result to the TVRB using the vsrc PRN.
3. 4. The TVRB will notify the vec store IQ that the temp register is available, and the LSU can begin execution.
4. When executing a segmented store the LSU uses the vsrc PRN as the source temporary register and will read the value in the TVRB.
5. The LSU will regard the vsrc as a temp register and only access the vector bypass network.
6. The LSU will store the temp vsrc data to memory once its ROB entry is committed.



Speculative Wakeups
~~~~~~~~~~~~~~~~~~~

The vector bypass network also supports speculative load wakeup, however speculative wakeup's are
only sent to the vector scheduler in the stage 2 IQ. Speculative bypass wakeup's to the scalar 
scheduler would not improve performance as it would require more than 1 cycle for the OP.v to reach
an vector execution unit.



The Vector Register File
~~~~~~~~~~~~~~~~~~~~~~~~~

The vector physical register file will be parameterizable with default support for
128 PRNs, 12 Read Ports, and 6 Write Ports. The vector register file (VRF) will be 
implemented using standard cell 3R1W FFs. The VRF is only initialized by the usingRVV switch.

.. list-table::
   :header-rows: 1
   :widths: 1 1 1

   * - Functional Unit
     - PRF Read Ports
     - PRF Write Ports
   * - CoProcessor
     - 4
     - 2
   * - Store Unit
     - 4
     - 2
   * - Load Unit
     - 4
     - 2
