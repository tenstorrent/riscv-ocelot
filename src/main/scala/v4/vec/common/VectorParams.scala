//******************************************************************************
// Copyright (c) 2024 - 2024, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Caracal Vector Parameters
//------------------------------------------------------------------------------
//
// Configuration for the Caracal RVV 1.0 vector extension to BOOM v4. Every
// field is parametrizable; the defaults below are the Medium-tier Goal 1
// settings. The Mega tier overrides vecIssueGrantWidth, dcacheArbiterMode and
// vecScalarSnoopEnable (see WithVector usage in config-mixins.scala).

package boom.v4.vec.common

case class VectorParams(
  vLen: Int = 256,
  numVecPhysRegisters: Int = 128,            // bump after perf tuning (issue 7)
  numVlPhysRegisters: Int = 64,              // VL register file depth (own rename space)
  numVecLoadQueueEntries: Int = 64,          // single LMUL=8 NF=8 inst should fit (issue 8)
  numVecStoreQueueEntries: Int = 64,
  numVecTmpGroups: Int = 4,                  // pvtmp headroom: in-flight shared-inst temp vector groups
  ssiQueueEntries: Int = 512,                // worst-case single-store element count (VLEN/8 * LMUL=8)
  lcbEntries: Int = 8,                       // VLEN-wide load assembly (load-combine buffer) entries
  vecLoadIssueEntries:  Int = 8,             // vector issue-queue slot counts; distinct from the
  vecStoreIssueEntries: Int = 8,             // LSU numVec{Load,Store}QueueEntries (those are the
  vecAluIssueEntries:   Int = 8,             // post-issue address/data queues, these are pre-issue slots)
  vecIssueGrantWidth: Int = 1,               // FU-pipeline-in-order: 1 on Medium, lift to 2 on Mega
  dcacheArbiterMode: String = "single",      // "single" (Goal 1 default) | "dual-dynamic" (Mega)
  vecScalarSnoopEnable: Boolean = false,     // turn on with dual-dynamic arbiter (issue 9)
  mshrAllocPolicy: String = "fair-floor"     // "fair-floor" (default) | "hard-partition" | "fcfs" (issue 10)
)
