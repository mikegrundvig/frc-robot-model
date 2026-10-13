package com.michaelgrundvig.frc.world.record;

import static com.michaelgrundvig.frc.world.robot.RecordedRobot.PIVOT;
import static com.michaelgrundvig.frc.world.robot.RecordedRobot.ROBOT;
import static com.michaelgrundvig.frc.world.robot.RecordedRobot.ROLLER;
import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import org.junit.jupiter.api.Test;

/** The wording of a gate's terms and of a binding. This is the one test of each sentence. */
class ControlSentencesTest {
  private static final List<String> TEXT = List.of("aimed", "no target");

  private static String term(GateTerm.Kind kind, int index) {
    return ControlSentences.gateTerm(ROBOT, "ready to shoot", new GateTerm(kind, index), TEXT);
  }

  @Test
  void aTermSaysWhatKeepsTheGateFalse() {
    assertThat(ControlSentences.gateTerm(ROBOT, "ready to shoot", GateTerm.text(0), TEXT))
        .isEqualTo("aimed");
    assertThat(term(GateTerm.Kind.JOINT_POSITION_UNKNOWN, PIVOT))
        .isEqualTo("pivot doesn't know where it is");
    assertThat(term(GateTerm.Kind.JOINT_NOT_ANSWERING, ROLLER)).isEqualTo("roller isn't answering");
    assertThat(term(GateTerm.Kind.SENSOR_LOST, 1)).isEqualTo("claw beam lost");
    assertThat(ControlSentences.gateTerm(ROBOT, "ready to shoot", GateTerm.settling(), TEXT))
        .isEqualTo("ready to shoot settling");
    assertThat(term(GateTerm.Kind.THREW, 1)).isEqualTo("ready to shoot threw: no target");
  }

  @Test
  void whyAGateIsFalseListsItsTerms() {
    assertThat(ControlSentences.gateWhy("ready to shoot", List.of()))
        .isEqualTo("ready to shoot is false");
    assertThat(ControlSentences.gateWhy("ready to shoot", List.of("aimed", "shooter at goal")))
        .isEqualTo("ready to shoot is false: aimed, shooter at goal");
  }

  @Test
  void aBindingSaysItsButtonHowItStartsAndItsCommand() {
    assertThat(
            ControlSentences.binding(
                "operator", "right trigger", BindingStart.WHILE_HELD, "Intake"))
        .isEqualTo("operator right trigger: while held: Intake");
    assertThat(ControlSentences.binding("driver", "back", BindingStart.ON_PRESS, "Reset heading"))
        .isEqualTo("driver back: on press: Reset heading");
  }

  @Test
  void eachWayToBindSaysWhichEdgeStartsIt() {
    assertThat(BindingStart.values())
        .extracting(BindingStart::words)
        .containsExactly(
            "on press",
            "on release",
            "while held",
            "while released",
            "toggled on press",
            "toggled on release",
            "while held, again if it ends",
            "while released, again if it ends");
    assertThat(BindingStart.WHILE_HELD.startsOnPress()).isTrue();
    assertThat(BindingStart.ON_RELEASE.startsOnPress()).isFalse();
    assertThat(BindingStart.TOGGLE_ON_RELEASE.startsOn()).isEqualTo(BindingStart.Edge.RELEASE);
  }
}
