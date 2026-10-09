// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.lsu.LSU]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in lsu.scala;
  * firtool emits these into their own .sv and a SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/reqs/formal-lsu.yaml.
  */
object LSUChecks {
  def apply(
    stqExecEnqValid:  Bool,
    stqEnqCommitted:  Bool,
    stqEnqIsAmo:      Bool,
    stqEnqAddrValid:  Bool,
    stqEnqDataValid:  Bool
  ): Unit = {

    //@formal-req-spec-lsu.a6
    AssertProperty(
      stqExecEnqValid |-> Sequence.BoolSequence(stqEnqAddrValid && stqEnqDataValid),
      label = Some("lsu_stq_execute_enq_has_addr_and_data")
    )

    // Antecedent reachability for lsu_stq_execute_enq_has_addr_and_data
    // (formal-lsu.a1) and lsu_stq_execute_enq_is_committed (formal-lsu.j17).
    CoverProperty(stqExecEnqValid, label = Some("lsu_stq_execute_enq_seen"))

    //@formal-req-spec-lsu.j9
    AssertProperty(
      stqExecEnqValid |-> Sequence.BoolSequence(stqEnqCommitted || stqEnqIsAmo),
      label = Some("lsu_stq_execute_enq_is_committed")
    )
  }
}
