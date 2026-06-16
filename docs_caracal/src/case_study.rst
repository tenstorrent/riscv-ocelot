Case Study
==========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


This document will review how edge-cases and complex instructions are handled by the architecture.



Segemented Load Stores (Shared Instruction)
-------------------------------------------

Segmented Load Stores is the only instruction that currently uses the temporary vector register
file. These instructions are marked is_shared, and split into two halves. One half is executed
by the load store unit to move data to/from memory, anf the other half by the co-processor
to transpose the data.



VL == 0
-------

If the VL is known at decode stage either as a immediate value or as a cached vl
value from a prior vset instruction. It can be immediately squashed, and not be issued to the
next stage.

If the vl is not known at decode stage and is reolved in the IQ_V_* because it was a scalar 
register source dependency. Then the EU must perform a copy fom the old stale vdest physical
register to the new vdest PRN.






