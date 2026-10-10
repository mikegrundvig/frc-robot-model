package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.api.Test;

/** The volts that hold a joint at its profile's point, from where its encoder says it is. */
class EncoderLoopTest {
  private static final Joint.Revolute PIVOT =
      StandardRobot.PIVOT
          .motors(Motor.KRAKEN_X60.through(MechanismFixtures.gearing(25, 1)))
          .control(
              MechanismControl.position(30).withHoming(MechanismControl.Homing.assumeAtRest()));

  /**
   * Its feedback gain's share of the error, the plan's feedforward at the profile's point, and the
   * rate's error and the program's own volts as they come.
   */
  @Test
  void itIsTheFeedbackPlusTheFeedforward() {
    ControllerPlan plan =
        ControllerPlan.of(
            StandardRobot.robotOf(PIVOT).math(), PIVOT, PIVOT.driven().control().orElseThrow());
    EncoderLoop loop = new EncoderLoop(plan);

    double volts = loop.volts(1.0, 0.5, 0.25, 0.9, 0.4);

    assertThat(volts)
        .isCloseTo(
            plan.kP() * 0.1 + plan.feedforward(1.0, 0.5, 0) + plan.kD() * 0.1 + 0.25,
            within(1e-12));
    assertThat(loop.volts(1.0, 0, 0, 1.0, 0)).isCloseTo(plan.feedforward(1.0, 0, 0), within(1e-12));
  }
}
