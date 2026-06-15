=============================
The Caracal Microarchitecture
=============================

:Author: Tenstorrent

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC

.. =====================================================================
.. HOW TO USE THIS TEMPLATE
.. - Replace every TODO note and <placeholder> with real content.
.. - Sections mirror the upstream BOOM docs (docs/sections/*) plus
..   Caracal-specific additions (the vector / RVV pipeline).
.. - Delete sections that do not apply; add subsections as needed.
.. - Figures live under ./figures/ — reference with .. figure:: name.svg
.. - Build with:  rst2html5 caracal-architecture.rst caracal-architecture.html
..          or via Sphinx if integrated into a docs tree.
.. =====================================================================

.. contents:: Table of Contents
   :depth: 3

.. sectnum::
   :depth: 3

.. =====================================================================



.. =====================================================================
















.. =====================================================================


Usage
=====

Parameterization & Configs
--------------------------

TODO: List the |caracal| config classes and the config fragments that
compose them.

.. list-table::
   :header-rows: 1
   :widths: 1 1 2

   * - Config
     - Size
     - Notes
   * - ``SmallBoomV4Config``
     - 1 small core
     - <fill in>
   * - ``MediumBoomV4Config``
     - 1 medium core
     - <fill in>
   * - ``LargeBoomV4Config``
     - 1 large core
     - <fill in>
   * - ``MegaBoomV4Config``
     - 1 mega core
     - <fill in>
   * - <Caracal vector config>
     - <fill in>
     - <fill in>

The Caracal/Chipyard Ecosystem
------------------------------

TODO: How |caracal| plugs into Chipyard, FireSim, and ASIC flows.

Debugging
---------

TODO: Waveform flows (FSDB/Verdi, VPD/DVE, VCD/GTKWave, Xcelium IDA),
useful internal signals, and the trace interface.

Micro-architectural Performance Counters
----------------------------------------

TODO: Available HPM/uarch counters and how to read them.

Verification
------------

TODO: riscv-tests, torture/csmith, co-simulation, and any RVV-specific
verification (vector test suites).

Physical Realization (PPA)
--------------------------

TODO: Synthesis/area results, target frequency, and PPA goals.
Cross-reference the area analysis docs in ``../docs/``.

.. =====================================================================

