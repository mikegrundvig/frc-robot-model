package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Rotations;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * How far apart a joint's absolute encoder and its motor's count may be, still, and whether they
 * are.
 */
class EncoderGapTest {
  private static final double LOOP = 0.02;

  private static final Joint.Revolute PIVOT =
      StandardRobot.PIVOT
          .motor(Motor.KRAKEN_X60, MechanismFixtures.gearing(25, 1))
          .control(MechanismControl.position(30));

  private static final Joint.Prismatic ELEVATOR =
      StandardRobot.ELEVATOR
          .motors(Motor.KRAKEN_X60, MechanismFixtures.gearing(10, 1), Inches.of(1), 2)
          .control(MechanismControl.position(60));

  /** A turret that turns freely. */
  private static final Joint.Revolute TURRET =
      StandardRobot.named(
              StandardRobot.lightPivot("turret", Optional.empty()),
              List.of(Position.degrees("front", 0)),
              List.of())
          .motor(Motor.KRAKEN_X60, MechanismFixtures.gearing(50, 1))
          .control(MechanismControl.position(20));

  private static double margin(Joint joint) {
    return EncoderGap.margin(
        StandardRobot.robotOf(joint).math(), joint, joint.driven().control().orElseThrow());
  }

  /**
   * The least margin, 5° on a pivot, or 2 cm on an elevator, whose encoder's count at the joint is
   * finer; a ten-turn potentiometer on the pivot's axle, turning a tenth of its range a turn of the
   * pivot, has counts 0.88° apart there, and eight of them, 7.0°, is the margin; a control's own
   * wins.
   */
  @Test
  void itsMarginIsTheLeastOrEightCountsAtTheJoint() {
    Joint.Revolute onCancoder =
        PIVOT.motorsOn(TALON_FX.can(21)).absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0)));
    Joint.Revolute onPot =
        PIVOT.absoluteEncoder(MechanismWiring.PortEncoder.analog(2, 10).offset(Rotations.of(0)));
    Joint.Prismatic elevator =
        ELEVATOR.absoluteEncoder(
            Driven.AbsoluteEncoder.of()
                .ratio(1.0 / 9)
                .on(MechanismWiring.PortEncoder.dutyCycle(2).offset(Rotations.of(0.3))));

    assertThat(Math.toDegrees(margin(onCancoder))).isCloseTo(5, within(1e-9));
    assertThat(Math.toDegrees(margin(onPot))).isCloseTo(8 * 3600.0 / 4096, within(1e-9));
    assertThat(margin(elevator)).isCloseTo(0.02, within(1e-12));
    assertThat(
            Math.toDegrees(
                margin(onCancoder.retuned(control -> control.withEncoderGap(Degrees.of(8))))))
        .isCloseTo(8, within(1e-9));
  }

  /**
   * Apart past the margin and a loop's travel at the joint's speed, they're apart; a loop's travel
   * more is allowed moving, a reading missing says nothing, and inside the margin they agree.
   */
  @Test
  void theyreApartOnlyPastTheirMarginAndALoopsTravel() {
    EncoderGap gap = new EncoderGap(PIVOT, Math.toRadians(5), Double.POSITIVE_INFINITY);
    double off = Math.toRadians(6);

    assertThat(gap.apart(off, 0, 0, LOOP)).isTrue();
    assertThat(gap.shown(off, 0)).isEqualTo("6° apart, more than 5°");
    assertThat(gap.apart(off, 0, 4, LOOP)).as("6° inside 5° and a loop at 4 rad/s, 4.6°").isFalse();
    assertThat(gap.apart(Double.NaN, 0, 0, LOOP)).isFalse();
    assertThat(gap.agree(Math.toRadians(4.9), 0)).isTrue();
    assertThat(gap.agree(off, 0)).isFalse();
  }

  /**
   * A turret that turns freely, its encoder on its axle, travels past the turn its encoder tells
   * apart: its motor's count at 350° and its encoder's -10° agree, and an encoder slipped 20°
   * across its wrap, reading -170° with the count at 170°, is 20° apart. The pivot, whose range its
   * encoder tells apart, takes them as they read.
   */
  @Test
  void theyreApartTheNearestWayRoundAJointThatTravelsPastItsEncodersTurn() {
    Joint.Revolute turret =
        TURRET.absoluteEncoder(MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0)));
    Joint.Revolute pivot =
        PIVOT.absoluteEncoder(MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0)));
    double turn = EncoderGap.turn(StandardRobot.robotOf(turret).math(), turret);
    EncoderGap gap = new EncoderGap(turret, Math.toRadians(5), turn);

    assertThat(Math.toDegrees(turn)).isCloseTo(360, within(1e-9));
    assertThat(gap.agree(Math.toRadians(-10), Math.toRadians(350))).isTrue();
    assertThat(gap.agree(Math.toRadians(-10), Math.toRadians(710))).isTrue();
    assertThat(gap.apart(Math.toRadians(-170), Math.toRadians(170), 0, LOOP)).isTrue();
    assertThat(gap.shown(Math.toRadians(-170), Math.toRadians(170)))
        .isEqualTo("20° apart, more than 5°");
    assertThat(EncoderGap.turn(StandardRobot.robotOf(pivot).math(), pivot)).isInfinite();
  }
}
