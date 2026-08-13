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

package boom.v4.vec.generated

import chisel3.util.{log2Ceil, isPow2}

// GENERATED from src/main/nlhdl/pkg/VectorParams.nlhdl.scala. Do not hand-edit;
// regenerate via the nlhdl gen-rtl flow instead.
case class VectorParams(
  // ---- Machine sizes ----

  //@req-spec-decode.a8
  vLen: Int = 256,

  //@req-spec-decode.a9
  eLen: Int = 64,

  // ---- Physical register files ----

  //@req-spec-vrf.a1
  //@req-spec-vrf.a4
  numVecPhysRegisters: Int = 128,

  //@req-spec-vrf.a5
  //@req-spec-rename.h1
  //@req-spec-rename.h2
  numVlPhysRegisters: Int = 64,

  // ---- Element queues (vector LSU) ----

  //@req-spec-lsu.b2
  ssiQueueEntries: Int = 512,

  usQueueEntries: Int = 16,

  lcbEntries: Int = 8,

  // Outstanding vector load beats whose response alignment is held in the
  // tag-keyed table; a beat may not fire without a free tag.
  ldRespTags: Int = 8,

  // Cycles a structure may sit without progress before the debug-layer watchdogs
  // dump their state. Below the core's boom_timeout so the dump precedes the hang.
  stallReportCycles: Int = 2000,

  // ---- Coprocessor interface figures (DERIVED, not chosen) ----

  //@req-spec-cii.b4
  ciiTagBits: Int = 4,

  //@req-spec-cii.a19
  maxMembers: Int = 8,

  ciiNumSrcSlots: Int = 4,

  // ---- Per-tier width knobs ----

  dcacheArbiterMode: String = "single",

  vecIssueGrantWidth: Int = 1,

  vecIssueEntries: Int = 16,

  // ---- Load reservation quantum (decision D9/D10) ----

  ldResvMembers: Int = 4,

  // ---- Port counts named by requirements but previously declared nowhere ----

  numVecWbPorts: Int = 3,

  numVecClrPorts: Int = 3
)
{
  // ---- Elaboration-time requires that depend only on this class's own fields ----

  require(numVecPhysRegisters >= 32 + maxMembers,
    s"numVecPhysRegisters ($numVecPhysRegisters) must be >= 32 + maxMembers ($maxMembers): " +
    "otherwise no full LMUL=8 group can ever be renamed and the machine deadlocks at rename")

  //@req-spec-lsu.b16
  //@req-spec-lsu.b17
  require(ssiQueueEntries >= vLen,
    s"ssiQueueEntries ($ssiQueueEntries) must be >= vLen ($vLen): every element-granular " +
    "queue must hold at least vLen elements, or the machine can deadlock on a wide masked store")

  require(dcacheArbiterMode == "single" || dcacheArbiterMode == "dual-dynamic",
    s"""dcacheArbiterMode ("$dcacheArbiterMode") must be "single" or "dual-dynamic"""")

  // The reservation tracks all six queue tails in ONE width (2*ssiQueueEntries); a US
  // queue's own pointer is that value truncated. Exact only under these two.
  require(isPow2(ssiQueueEntries) && isPow2(usQueueEntries),
    s"ssiQueueEntries ($ssiQueueEntries) and usQueueEntries ($usQueueEntries) must be powers of two")
  require((2 * ssiQueueEntries) % (2 * usQueueEntries) == 0,
    s"2*ssiQueueEntries (${2 * ssiQueueEntries}) must be a multiple of 2*usQueueEntries " +
    s"(${2 * usQueueEntries})")
}
trait HasVectorParams
{
  val vectorParams: VectorParams
  import vectorParams._

  // ---- Derived widths ----

  lazy val vecPregSz: Int = log2Ceil(numVecPhysRegisters)

  lazy val vlPregSz: Int = log2Ceil(numVlPhysRegisters)

  lazy val elenBytes: Int = eLen / 8

  lazy val ldRespTagSz: Int = log2Ceil(ldRespTags)

  // The +1 is VecElemQueue's full/empty-disambiguating carry bit. Every cross-module
  // element-queue pointer (reservation base, rollback tail, squash tail) uses THIS width;
  // dropping the bit makes "full" and "empty" the same value at the receiving queue.
  lazy val resvPtrSz: Int = log2Ceil(ssiQueueEntries) + 1

  //@req-spec-decode.a10

  lazy val maxVecVL: Int = vLen * maxMembers / 8

  lazy val vecVLSz: Int = {
    val w = log2Ceil(maxVecVL) + 1
    require(w >= log2Ceil(vLen * maxMembers / 8 + 1),
      s"vecVLSz ($w) is narrower than the architectural VL width " +
      s"(log2Ceil(vLen * maxMembers / 8 + 1) = ${log2Ceil(vLen * maxMembers / 8 + 1)}); " +
      "check the maxVecVL derivation for a truncation bug")
    w
  }

  //@req-spec-cii.a16
  lazy val ciiSrcDataBits: Int = vLen
  lazy val ciiWritebackBits: Int = vLen

  // ---- Vector PRN capacity: what numVecPhysRegisters actually buys ----

  //@req-spec-vrf.b2
  //@req-spec-vrf.b3
  //@req-spec-vrf.b4
  //@req-spec-vrf.b6
  lazy val maxRenamableGroups: Int = (numVecPhysRegisters - 32) / maxMembers
  lazy val maxRenamableSegGroups: Int = maxRenamableGroups / 2

  // ---- Element-queue depth is an architectural limit ----

  lazy val maxInflightWorstCaseStores: Int = ssiQueueEntries / vLen
}
