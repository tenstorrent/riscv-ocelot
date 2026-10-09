// See LICENSE.* for license details.
//
// WORKED EXAMPLE for /req-formal-chisel. Its ledger is examples/formal-skid.yaml.
//
// This file is NOT compiled by sbt (it lives outside src/main/scala/ on purpose).
// It was verified as a standalone Chisel project: it elaborates, firtool 1.75.0
// emits SkidBuffer_BoomSvaLayer.sv plus a `bind` in
// layers_SkidBuffer_BoomSvaLayer.sv, the DUT's own .sv contains zero assertions,
// and VCS compiles the result with -assert svaext.
//
// Read it for SHAPE: the apply signature mirroring the ledger's `signals:`, one
// //@formal-req- tag per requirement directly above its property, reachability
// covers with no tags, and labels prefixed with the DUT name.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for a 1-entry skid buffer.
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in SkidBuffer.scala.
  * Rows: examples/formal-skid.yaml.
  */
object SkidBufferChecks {
  def apply(
    enqFire:  Bool,
    enqValid: Bool,
    enqBits:  UInt,
    deqFire:  Bool,
    deqValid: Bool,
    deqReady: Bool,
    deqBits:  UInt,
    full:     Bool,
    data:     UInt
  ): Unit = {

    // -- group a: the full/empty invariant ---------------------------------

    //@formal-req-spec-skid.a1
    AssertProperty(
      !(enqFire && full),
      label = Some("skid_no_enq_when_full")
    )

    //@formal-req-spec-skid.a2
    AssertProperty(
      enqFire |=> deqValid,
      label = Some("skid_enq_then_deq_valid")
    )

    // Antecedent reachability for skid_enq_then_deq_valid (formal-skid.a6).
    CoverProperty(enqFire, label = Some("skid_enq_fire_seen"))

    // -- group b: the Decoupled contract ----------------------------------

    //@formal-req-spec-skid.b1
    AssertProperty(
      (deqValid && !deqReady) |-> deqValid.delay(1),
      label = Some("skid_deq_valid_stable")
    )

    //@formal-req-spec-skid.b2
    AssertProperty(
      deqValid |-> Sequence.BoolSequence(deqBits === data),
      label = Some("skid_deq_bits_matches_reg")
    )

    // Antecedent reachability for the two properties above (formal-skid.b5).
    CoverProperty(deqValid && !deqReady, label = Some("skid_deq_backpressured"))

    // -- group c: liveness -------------------------------------------------

    //@formal-req-spec-skid.c1
    AssertProperty(
      full |-> deqFire.delayAtLeast(0).eventually,
      label = Some("skid_drains")
    )

    // -- assumes: environment constraints, each licensed by a requirement ---
    // No //@formal-req- tag: an assume CHECKS nothing, so it discharges no
    // requirement. spec-skid.d1 is ledgered `unassertable:` and names this
    // assume in its reason. See schema.md §5.

    AssumeProperty(
      enqValid |-> Sequence.BoolSequence(enqBits =/= 0.U),
      label = Some("skid_no_zero_input")
    )

    // Proves skid_no_zero_input did not constrain enqueues out of existence.
    CoverProperty(enqValid, label = Some("skid_enq_valid_reachable"))

    // -- functional coverage: scenarios worth measuring, no reqs attached ---

    CoverProperty(full ### full, label = Some("skid_two_cycles_full"))
    CoverProperty(enqFire |=> deqFire, label = Some("skid_enq_then_deq_fire"))
  }
}

/* The anchor this checker is bound through, for reference. In the real flow it
 * lives in the DUT's own file and is the ONLY edit made there:
 *
 *   //@formal-anchor SkidBufferChecks
 *   layer.block(BoomSvaLayer) {
 *     SkidBufferChecks(
 *       enqFire  = io.enq.fire,  enqValid = io.enq.valid, enqBits = io.enq.bits,
 *       deqFire  = io.deq.fire,  deqValid = io.deq.valid, deqReady = io.deq.ready,
 *       deqBits  = io.deq.bits,  full     = full,         data    = data
 *     )
 *   }
 */
