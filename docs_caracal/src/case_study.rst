Case Study
==========

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC


This document will review how edge-cases and complex instructions are handled by the architecture.



Segmented Load Stores (Shared Instruction)
-------------------------------------------

Segmented Load Stores is the only instruction that currently uses an intermediate temp vector group
(``pvtmp``) in the VRF. These instructions are marked is_shared, and split into two halves. One half
is executed by the load store unit to move data to/from memory, and the other half by the co-processor
to transpose the data; they hand off through ``pvtmp``.



VL == 0
-------

VL is only known at decode for the **immediate-AVL** form (``vsetivli`` with AVL ``= 0``): such a
``vsetivli``/dependent can be detected and the dependent squashed at decode without reaching issue.
For all other forms VL is not known at decode (it is renamed into the VL register file), so a
``VL = 0`` op reaches issue, where ``pvl`` resolves to 0.

When the ``OP.v`` reaches issue with ``VL = 0``, it cannot have been squashed at decode. With
``VL = 0`` **no element is active**, so nothing executes — but the freshly-allocated ``pvdest``
group still has to be made architecturally correct before the entry can commit, because a new
physical group was renamed for it. Two cases, gated on ``vta``:

- **Tail-undisturbed (``vta = 0``).** The new destination group must equal the **old** value of
  the architectural vreg group. The ``OP.v`` performs a **group copy** ``pvdest ← stale_pvdest`` —
  up to ``EMUL`` whole-register ``VLEN``-wide copies, one per group member, reading the stale group
  and writing the new group. This reuses the **Load Unit's VRF ports** (``R``/``W`` on the Load
  path, which are idle when no load is draining), so it adds **no new VRF ports** (see the Register
  Files section). The copy completes with a single **group-done** (:ref:`group-done-wb`), exactly
  like a real load, so the ROB/Busy-Table/wakeup path is unchanged.
- **Tail-agnostic (``vta = 1``).** The tail may hold any value, so no copy is required; the entry
  takes the squash / complete-without-execute path and emits its group-done immediately.

This is the degenerate (``elem_start ≥ vl``) corner of the **general** tail/mask-undisturbed
handling, which is *not* special to ``VL = 0``: on every masked or partial-tail vector op the
inactive element lanes of the destination must be preserved or filled per ``vta``/``vma``. For
**loads** the Load Coalescing Buffer already fills inactive byte lanes from the old group / per
``vta``/``vma`` policy before its single VRF write (:ref:`load-coalesce`), so no separate copy is
needed there. For **arithmetic** the CII coprocessor reads the old ``vd`` (one of its four VRF read
ports, see the Register Files section) and merges the undisturbed lanes itself. The standalone group
copy above is therefore needed only for the no-execution ``VL = 0`` / fully-inactive ``vta = 0``
case.






