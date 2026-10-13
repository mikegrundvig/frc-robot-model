package com.michaelgrundvig.frc.world.record;

import static com.michaelgrundvig.frc.world.robot.RecordedRobot.PIVOT;
import static com.michaelgrundvig.frc.world.robot.RecordedRobot.ROBOT;
import static com.michaelgrundvig.frc.world.robot.RecordedRobot.ROLLER;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RecordedRobot;
import java.util.List;
import org.junit.jupiter.api.Test;

/** The wording of what a person reads about one joint. This is the one test of each sentence. */
class JointSentencesTest {
  private static final Joint PIVOT_JOINT = RecordedRobot.joint(PIVOT);
  private static final Joint ROLLER_JOINT = RecordedRobot.joint(ROLLER);

  private static String state(JointState state, String fault) {
    return JointSentences.state(ROBOT, PIVOT_JOINT, state, fault);
  }

  @Test
  void aStateIsItsStandingWithWhatHomingOrAFaultAdds() {
    assertThat(state(JointState.READY, "")).isEqualTo("READY");
    assertThat(state(JointState.UNHOMED, "")).isEqualTo("UNHOMED");
    assertThat(state(JointState.homing(JointState.Toward.LOWER_STOP, 9.5), ""))
        .isEqualTo("HOMING: toward its lower stop, for at most 9.5 s");
    assertThat(state(JointState.homing(JointState.Toward.UPPER_STOP, 2), ""))
        .isEqualTo("HOMING: toward its upper stop, for at most 2.0 s");
    assertThat(state(JointState.homing(JointState.Toward.LOWER_SWITCH, 4.25), ""))
        .isEqualTo("HOMING: toward pivot down, for at most 4.3 s");
    assertThat(
            state(
                JointState.faulted(JointState.Fault.HOMING_FOUND_NO_STOP),
                "pivot's homing found no stop in 2.0 s, pushing toward its lower one"))
        .isEqualTo("FAULTED: pivot's homing found no stop in 2.0 s, pushing toward its lower one");
  }

  @Test
  void whatHoldsAJointIsEachThingInItsOwnWords() {
    assertThat(JointSentences.heldBy(ROBOT, List.of())).isEmpty();
    assertThat(JointSentences.heldBy(ROBOT, List.of(Holder.interlock(0))))
        .isEqualTo("roller runs only when pivot at least run");
    assertThat(JointSentences.heldBy(ROBOT, List.of(Holder.interlock(1))))
        .isEqualTo("arm keeps out of -10° to 30° while elevator at most 0.35 m");
    assertThat(
            JointSentences.heldBy(ROBOT, List.of(Holder.noPosition(PIVOT), Holder.limitSwitch(0))))
        .isEqualTo("pivot's position isn't a number; pivot down tripped");
  }

  @Test
  void aHolderHoldsTheJointItIsAbout() {
    assertThat(JointSentences.heldJoint(ROBOT, Holder.interlock(0))).isEqualTo(ROLLER_JOINT);
    assertThat(JointSentences.heldJoint(ROBOT, Holder.limitSwitch(0))).isEqualTo(PIVOT_JOINT);
    assertThat(JointSentences.heldJoint(ROBOT, Holder.noPosition(ROLLER))).isEqualTo(ROLLER_JOINT);
  }

  @Test
  void whereAJointIsIsSaidInItsPlacesNames() {
    assertThat(place(PIVOT_JOINT, JointPlace.UNKNOWN)).isEqualTo("unknown");
    assertThat(place(PIVOT_JOINT, JointPlace.by(JointPlace.Where.AT, 2))).isEqualTo("at out");
    assertThat(place(PIVOT_JOINT, JointPlace.between(1, 2))).isEqualTo("between run and out");
    assertThat(place(PIVOT_JOINT, JointPlace.by(JointPlace.Where.BELOW, 0))).isEqualTo("below in");
    assertThat(place(PIVOT_JOINT, JointPlace.by(JointPlace.Where.ABOVE, 2))).isEqualTo("above out");
    assertThat(
            JointSentences.place(
                RecordedRobot.unnamedTurret(), JointPlace.UNNAMED, Math.toRadians(45)))
        .isEqualTo("45°");
  }

  @Test
  void aJointThatOnlyRunsIsSaidByWhatItRunsAt() {
    assertThat(place(ROLLER_JOINT, JointPlace.running(0))).isEqualTo("intaking");
    assertThat(place(ROLLER_JOINT, JointPlace.running(1))).isEqualTo("outtaking");
    assertThat(place(ROLLER_JOINT, JointPlace.STOPPED)).isEqualTo("stopped");
    assertThat(place(ROLLER_JOINT, JointPlace.IDLE)).isEqualTo("idle");
  }

  @Test
  void whoAskedIsTheCommandOrThatNoneDid() {
    assertThat(JointSentences.owner(JointOwner.COMMAND, "Intake")).isEqualTo("Intake");
    assertThat(JointSentences.owner(JointOwner.IDLE, "")).isEqualTo("idle");
    assertThat(JointSentences.owner(JointOwner.OUTSIDE_ANY_COMMAND, ""))
        .isEqualTo("outside any command");
  }

  private static String place(Joint joint, JointPlace place) {
    return JointSentences.place(joint, place, Double.NaN);
  }
}
