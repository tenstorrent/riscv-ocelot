/*
Tenstorrent CONFIDENTIAL
__________________
 Tenstorrent Inc.
 All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Tenstorrent Inc.  The intellectual
and technical concepts contained
herein are proprietary to Tenstorrent Inc.
and may be covered by U.S., Canadian and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Tenstorrent Inc.
*/

/*
  sync_fifo — a single-clock (synchronous) first-in first-out queue. The write
  side and read side share one clock domain. Words are read back in the order
  they were written. Output is registered (not show-ahead): read data is valid
  the cycle after a successful read.
*/

<|begin_module|>

  <|begin_parameters|>
  WIDTH is the bit width of each data word. Default 32, legal range 1 to 1024.

  DEPTH is the number of entries the FIFO can hold. Default 16. DEPTH must be a
  power of two so that pointer wrap-around is free; if a non-power-of-two value
  is given, round the internal storage up to the next power of two.

  ALMOST_FULL_THRESH sets when almost_full asserts: free entries at or below it.
  Default 2. ALMOST_EMPTY_THRESH sets when almost_empty asserts: used entries at
  or below it. Default 2.
  <|end_parameters|>

  <|begin_ports|>
  The FIFO operates in a single clock domain. clk is the clock and all state
  updates on its rising edge. rst_n is a synchronous active-low reset that
  clears the FIFO to empty.

  The write interface pushes data. wr_en requests a push of wr_data, which is
  WIDTH bits wide, on the current cycle. full indicates no free entry remains,
  and write requests are ignored while it is high. almost_full asserts when the
  number of free entries is at or below ALMOST_FULL_THRESH.

  The read interface pops data. rd_en requests a pop. rd_data is WIDTH bits and
  carries the popped word; it is registered and becomes valid the cycle after a
  successful read. empty indicates no data is available, and read requests are
  ignored while it is high. almost_empty asserts when the number of used entries
  is at or below ALMOST_EMPTY_THRESH.

  count reports the current number of valid entries and is ceil(log2(DEPTH+1))
  bits wide.
  <|end_ports|>

  <|begin_logic|>
  Storage is a memory of DEPTH entries by WIDTH bits, addressed by a read
  pointer and a write pointer that each advance modulo DEPTH.

  // wrap bit: extra MSB on each pointer distinguishes full from empty
  Each pointer carries one extra most-significant bit beyond the address. The
  FIFO is empty when the write and read pointers are fully equal, and full when
  their addresses match but their wrap bits differ.

  A write succeeds when wr_en is high and the FIFO is not full: store wr_data at
  the write pointer's address, then advance the write pointer. A read succeeds
  when rd_en is high and the FIFO is not empty: present the entry at the read
  pointer's address on rd_data, then advance the read pointer. A simultaneous
  read and write in the same cycle is allowed and leaves count unchanged.

  // overflow/underflow guard: these conditions must never corrupt state
  When full, ignore wr_en so unread data is never overwritten. When empty,
  ignore rd_en so the read pointer never passes valid data.

  count tracks the number of valid entries and stays consistent with the flags
  every cycle: count of zero means empty, count of DEPTH means full. almost_full
  and almost_empty are derived from count and their thresholds.
  <|end_logic|>

<|end_module|>

<|begin_perf|>
Sustain one write and one read per clock, back to back, with no bubble cycles.
A word written on cycle N can be read starting cycle N+1. Keep a single pipeline
stage on each interface: there must be no combinational path from wr_data to
rd_data.
<|end_perf|>

<|begin_dependencies|>
None. sync_fifo is a self-contained leaf module.
<|end_dependencies|>
