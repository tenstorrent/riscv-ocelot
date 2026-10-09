// See LICENSE.* for license details.

package boom.v4.vec.formal

import chisel3._
import chisel3.ltl._
import chisel3.ltl.Sequence._

/** Properties for [[boom.v4.vec.generated.lsu.VecLsu]].
  *
  * Bound in via `layer.block(BoomSvaLayer)` at the anchor in VecLsu.scala;
  * firtool emits these into their own .sv and a SystemVerilog `bind`.
  *
  * Planned by /req-formal-chisel; rows in
  * src/main/nlhdl/formal/formal-lsu.yaml.
  */
object VecLsuChecks {
  def apply(
    stUsWritePass:     Bool,
    stSsiPass2Prev:    Bool,
    stUsCursorWrValid: Bool,
    stUsCursorWrBits:  UInt,
    stUsTotalElems:    UInt,
    stUsUpdateValid:   Bool,
    stBeatReqValid:    Bool,
    stBeatUsHead:      Bool,
    stBeatSsiHead:     Bool,
    stBeatWritePass:   Bool,
    stUsDataValid:     Bool,
    stUsDataPop:       Bool,
    stSsiDataValid:    Bool,
    dgenReqValid:      Bool,
    dgenReqRobIdx:     UInt,
    issStRobIdx:       UInt,
    issStAgen:         Bool,
    stRowTargetValid:  Bool,
    gcopyLaunchValid:  Bool,
    ldLcbTrigger:      Bool
  ): Unit = {

    //@formal-req-spec-lsu.d6
    //@formal-req-spec-lsu.j10
    AssertProperty(
      (stBeatReqValid && stBeatUsHead && stUsWritePass) |-> stUsDataValid,
      label = Some("vec_lsu_st_us_write_beat_has_data")
    )

    // Antecedent reachability for vec_lsu_st_us_write_beat_has_data (formal-lsu.d13).
    CoverProperty(
      stBeatReqValid && stBeatUsHead && stUsWritePass,
      label = Some("vec_lsu_st_us_write_beat_seen")
    )

    //@formal-req-spec-lsu.d8
    AssertProperty(
      stUsDataPop |-> Sequence.BoolSequence(stUsWritePass),
      label = Some("vec_lsu_st_us_data_pop_only_on_write_pass")
    )

    // Antecedent reachability for vec_lsu_st_us_data_pop_only_on_write_pass (formal-lsu.d15).
    CoverProperty(stUsDataPop, label = Some("vec_lsu_st_us_data_pop_seen"))

    //@formal-req-spec-lsu.d7
    AssertProperty(
      dgenReqValid |-> Sequence.BoolSequence(dgenReqRobIdx === issStRobIdx),
      label = Some("vec_lsu_dgen_request_matches_granted_store")
    )

    // Antecedent reachability for vec_lsu_dgen_request_matches_granted_store (formal-lsu.d17).
    CoverProperty(dgenReqValid, label = Some("vec_lsu_dgen_grant_seen"))

    //@formal-req-spec-lsu.j7
    //@formal-req-spec-lsu.j9
    AssertProperty(
      Sequence.BoolSequence(stBeatUsHead && stBeatSsiHead) |->
        Sequence.BoolSequence(stUsWritePass === stSsiPass2Prev),
      label = Some("vec_lsu_pass_state_agrees_across_classes")
    )

    // Antecedent reachability for vec_lsu_pass_state_agrees_across_classes (formal-lsu.j7).
    CoverProperty(
      stBeatUsHead && stBeatSsiHead,
      label = Some("vec_lsu_both_classes_staged_seen")
    )

    //@formal-req-spec-lsu.j8
    AssertProperty(
      stUsUpdateValid |-> Sequence.BoolSequence(!stUsWritePass),
      label = Some("vec_lsu_st_us_paddr_writeback_before_write_pass")
    )

    // Antecedent reachability for vec_lsu_st_us_paddr_writeback_before_write_pass (formal-lsu.j9).
    CoverProperty(stUsUpdateValid, label = Some("vec_lsu_st_us_paddr_writeback_seen"))

    //@formal-req-spec-lsu.j8
    //@formal-req-spec-lsu.j11
    AssertProperty(
      Sequence.BoolSequence(
        stUsCursorWrValid && !stUsWritePass && (stUsCursorWrBits >= stUsTotalElems)
      ) |=> stUsWritePass,
      label = Some("vec_lsu_st_us_write_pass_follows_translate")
    )

    // Antecedent reachability for vec_lsu_st_us_write_pass_follows_translate (formal-lsu.j11).
    CoverProperty(
      stUsCursorWrValid && !stUsWritePass && (stUsCursorWrBits >= stUsTotalElems),
      label = Some("vec_lsu_st_us_translate_complete_seen")
    )

    //@formal-req-spec-lsu.j10
    AssertProperty(
      Sequence.BoolSequence(
        stBeatReqValid && stBeatSsiHead && !stBeatUsHead && stBeatWritePass
      ) |-> stSsiDataValid,
      label = Some("vec_lsu_st_ssi_write_beat_has_data")
    )

    // Antecedent reachability for vec_lsu_st_ssi_write_beat_has_data (formal-lsu.j13).
    CoverProperty(
      stBeatReqValid && stBeatSsiHead && !stBeatUsHead && stBeatWritePass,
      label = Some("vec_lsu_st_ssi_write_beat_seen")
    )

    //@formal-req-spec-lsu.a8
    //@formal-req-spec-lsu.a9
    AssertProperty(
      issStAgen |-> Sequence.BoolSequence(!stRowTargetValid),
      label = Some("vec_lsu_store_grant_into_free_pending_row")
    )

    // Antecedent reachability for vec_lsu_store_grant_into_free_pending_row (formal-lsu.a4).
    CoverProperty(issStAgen, label = Some("vec_lsu_store_agen_grant_seen"))

    //@formal-req-spec-lsu.m2
    //@formal-req-spec-lsu.m1
    AssertProperty(
      gcopyLaunchValid |-> Sequence.BoolSequence(!ldLcbTrigger),
      label = Some("vec_lsu_group_copy_excludes_memory_path")
    )

    // Antecedent reachability for vec_lsu_group_copy_excludes_memory_path (formal-lsu.m16).
    CoverProperty(gcopyLaunchValid, label = Some("vec_lsu_group_copy_launch_seen"))
  }
}
