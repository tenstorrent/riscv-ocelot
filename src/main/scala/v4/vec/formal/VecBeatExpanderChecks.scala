// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecBeatExpander]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in
  * VecBeatExpander.scala; firtool emits these into their own .sv and a
  * SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/formal/formal-lsu.yaml.
  */
object VecBeatExpanderChecks {
  def apply(
    reqValid:      Bool,
    reqByteEn:     UInt,
    usHeadValid:   Bool,
    ssiHeadValid:  Bool,
    cursorWrValid: Bool,
    cursorWrBits:  UInt,
    usCursor:      UInt,
    reqFire:       Bool
  ): Unit = {

    //@formal-req-spec-lsu.c5
    AssertProperty(
      reqValid |-> Sequence.BoolSequence(usHeadValid || ssiHeadValid),
      label = Some("vec_beat_expander_beat_needs_head")
    )

    // Antecedent reachability for vec_beat_expander_beat_needs_head (formal-lsu.c1)
    // AND vec_beat_expander_beat_has_enabled_bytes (formal-lsu.k1).
    CoverProperty(reqValid, label = Some("vec_beat_expander_beat_seen"))

    //@formal-req-spec-lsu.c6
    AssertProperty(
      cursorWrValid |-> Sequence.BoolSequence(cursorWrBits > usCursor),
      label = Some("vec_beat_expander_cursor_advances")
    )

    // Antecedent reachability for vec_beat_expander_cursor_advances (formal-lsu.c3).
    CoverProperty(cursorWrValid, label = Some("vec_beat_expander_cursor_write_seen"))

    //@formal-req-spec-lsu.k8
    AssertProperty(
      reqValid |-> Sequence.BoolSequence(reqByteEn =/= 0.U),
      label = Some("vec_beat_expander_beat_has_enabled_bytes")
    )

    //@formal-req-spec-lsu.j2
    AssertProperty(
      Sequence.BoolSequence(reqFire && usHeadValid && !ssiHeadValid) |-> cursorWrValid,
      label = Some("vec_beat_expander_us_beat_advances_cursor")
    )

    // Antecedent reachability for vec_beat_expander_us_beat_advances_cursor (formal-lsu.j18).
    CoverProperty(
      reqFire && usHeadValid && !ssiHeadValid,
      label = Some("vec_beat_expander_us_beat_granted_seen")
    )
  }
}
