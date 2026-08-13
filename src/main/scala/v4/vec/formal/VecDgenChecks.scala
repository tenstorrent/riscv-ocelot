// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecDgen]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in VecDgen.scala;
  * firtool emits these into their own .sv and a SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/reqs/formal-lsu.yaml.
  */
object VecDgenChecks {
  def apply(
    reqResident:  Bool,
    killed:       Bool,
    r3ReqValid:   Bool,
    outReqValid:  Bool,
    r3RespValid:  Bool,
    outReqMbr:    UInt,
    membersUsed:  UInt,
    haveData:     Bool,
    ssiEnqValid:  Bool,
    usEnqValid:   Bool,
    usEnqBytes:   UInt,
    vLenBytesLit: UInt,
    isLastMbr:    Bool
  ): Unit = {

    //@formal-req-spec-lsu.d9
    //@formal-req-spec-lsu.d10
    AssertProperty(
      r3ReqValid |-> Sequence.BoolSequence(reqResident && !killed),
      label = Some("vec_dgen_r3_read_only_while_resident")
    )

    // Antecedent reachability for vec_dgen_r3_read_only_while_resident (formal-lsu.d4).
    CoverProperty(r3ReqValid, label = Some("vec_dgen_r3_read_seen"))

    //@formal-req-spec-lsu.d7
    AssertProperty(
      (outReqValid && r3RespValid) |-> Sequence.BoolSequence(outReqMbr < membersUsed),
      label = Some("vec_dgen_r3_resp_member_in_range")
    )

    // Antecedent reachability for vec_dgen_r3_resp_member_in_range (formal-lsu.d6).
    CoverProperty(outReqValid && r3RespValid, label = Some("vec_dgen_r3_resp_seen"))

    //@formal-req-spec-lsu.d7
    AssertProperty(
      Sequence.BoolSequence(ssiEnqValid || usEnqValid) |-> haveData,
      label = Some("vec_dgen_enq_data_from_vrf_read")
    )

    // Antecedent reachability for vec_dgen_enq_data_from_vrf_read (formal-lsu.d8).
    CoverProperty(ssiEnqValid || usEnqValid, label = Some("vec_dgen_data_enq_seen"))

    //@formal-req-spec-lsu.d5
    AssertProperty(
      usEnqValid |-> Sequence.BoolSequence((usEnqBytes === vLenBytesLit) || isLastMbr),
      label = Some("vec_dgen_us_push_is_whole_member")
    )

    // Antecedent reachability for vec_dgen_us_push_is_whole_member (formal-lsu.d10).
    CoverProperty(usEnqValid, label = Some("vec_dgen_us_push_seen"))
  }
}
