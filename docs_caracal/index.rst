Welcome to Tenstorrents Caracal documentation!
==============================================

:Author: Tenstorrent

.. Custom attributes — set once, reuse via |name|
.. |caracal| replace:: Caracal
.. |boom| replace:: BOOM
.. |isa| replace:: RV64GC



Abstract
========

|caracal| is Tenstorrent's fork of the Berkeley Out-of-Order Machine (|boom|), a synthesizable,
parameterizable |isa| out-of-order core. The |caracal| microarchitecture is a clean-slate
re-architecture of |boom| v4 implmenting RVV 1.0 instructions. The microarchitecture provides OOO
vector load/store support along with a In-Order Vector Unit for arithmetic operations. The vector
co-processor is integrated using the Tenstorrent Custom Instruction Interface (tt_CII).


Table of Contents
=================

The chapters below walk the |caracal| pipeline stage by stage, from the
front-end through the unified load/store unit. Each chapter calls out where the
vector path diverges from stock |boom| v4.

.. Make this welcome page itself a clickable item in the left sidebar.
.. The special `self` entry links the toctree back to its own document; a
.. caption makes the entry persist in the sidebar on every page (a bare,
.. caption-less toctree only shows while you are on this page).
.. toctree::
   :maxdepth: 2
   :caption: Home:

   self
   src/usage
   src/glossary

   
.. toctree::
   :maxdepth: 2
   :caption: Diagrams:

   src/diagrams


.. toctree::
   :maxdepth: 8
   :caption: Core Specification:
   :numbered:

   src/overview
   src/frontend
   src/midcore
   src/issue
   src/execution
   src/cii
   src/loadstore
   src/case_study




Useful Links
------------

The BOOM source code can be found here: https://github.com/riscv-boom/riscv-boom.

The main supported mechanism to use the core is to use the Chipyard framework: https://github.com/ucb-bar/chipyard.

The BOOM website can be found here: https://boom-core.org.

The BOOM mailing list can be found here: https://groups.google.com/forum/#!forum/riscv-boom.


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
