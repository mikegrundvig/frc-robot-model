package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * Where a joint is driven, worked out from its block: its limits, a turret's turn of a heading, a
 * free joint's short way round, and where it stops braking. No simulator: each is a function of
 * what it's given.
 */
class TravelTest {
  private static final Position TURRET_FRONT = Position.degrees("front", 0);

  /** The intake's pivot (0° to 135°) on a Kraken at 25:1, profiled at 8 rad/s and 30 rad/s². */
  private static final Joint.Revolute PIVOT =
      StandardRobot.PIVOT
          .motors(Motor.KRAKEN_X60.through(MechanismFixtures.gearing(25, 1)))
          .control(
              MechanismControl.position(30)
                  .withProfile(RadiansPerSecond.of(8), RadiansPerSecondPerSecond.of(30))
                  .withHoming(MechanismControl.Homing.assumeAtRest()));

  /** The elevator (0 to 1.2 m), profiled at 1.2 m/s and 5 m/s². */
  private static final Joint.Prismatic ELEVATOR =
      StandardRobot.ELEVATOR
          .motors(
              Motor.KRAKEN_X60
                  .through(MechanismFixtures.gearing(10, 1))
                  .drum(Inches.of(1))
                  .count(2))
          .control(
              MechanismControl.position(60)
                  .withProfile(MetersPerSecond.of(1.2), MetersPerSecondPerSecond.of(5))
                  .withHoming(MechanismControl.Homing.assumeAtRest()));

  /** A turret that turns freely, profiled at 3 rad/s and 10 rad/s². */
  private static final Joint.Revolute TURRET =
      StandardRobot.named(
              StandardRobot.lightPivot("turret", Optional.empty()),
              List.of(TURRET_FRONT),
              List.of())
          .motors(Motor.KRAKEN_X60.through(MechanismFixtures.gearing(50, 1)))
          .control(
              MechanismControl.position(20)
                  .withProfile(RadiansPerSecond.of(3), RadiansPerSecondPerSecond.of(10))
                  .withHoming(MechanismControl.Homing.assumeAtRest()));

  /** A turret whose range spans more than a turn, -200° to 200°. */
  private static final Joint.Revolute WIDE =
      JointRevoluteBuilder.builder(TURRET)
          .range(Range.degrees(-200, 200))
          .positions(List.of(TURRET_FRONT, Position.degrees("back", -180)))
          .build();

  private static final RobotDefinition ROBOT = StandardRobot.robotOf(PIVOT, ELEVATOR, TURRET);

  private static MechanismControl control(Joint joint) {
    return joint.driven().control().orElseThrow();
  }

  private static Travel travel(Joint joint) {
    if (joint.equals(PIVOT) || joint.equals(ELEVATOR) || joint.equals(TURRET)) {
      return Travel.of(ROBOT.math(), joint, control(joint));
    }
    return Travel.of(StandardRobot.robotOf(PIVOT, joint).math(), joint, control(joint));
  }

  /** A robot's loop, 50 Hz: the robot program's to say. */
  private static final double LOOP = 0.02;

  private static double radians(double degrees) {
    return Math.toRadians(degrees);
  }

  @Test
  void itsLimitsAreItsSoftLimitsElseItsStopsElseNone() {
    MechanismControl soft = control(PIVOT).withSoftLimits(Range.of(Radians.of(0.1), Radians.of(2)));

    assertThat(Travel.of(ROBOT.math(), PIVOT, soft).lower()).isEqualTo(0.1);
    assertThat(Travel.of(ROBOT.math(), PIVOT, soft).upper()).isEqualTo(2);
    assertThat(travel(PIVOT).lower()).isEqualTo(0);
    assertThat(travel(PIVOT).upper()).isCloseTo(radians(135), within(1e-12));
    assertThat(travel(TURRET).lower()).isEqualTo(Double.NEGATIVE_INFINITY);
    assertThat(travel(TURRET).upper()).isEqualTo(Double.POSITIVE_INFINITY);
  }

  /**
   * The fixtures' profiles, and a quarter of its free speed, reached in a quarter second, without.
   */
  @Test
  void itCruisesAtItsProfileElseAQuarterOfItsFreeSpeed() {
    MechanismControl unprofiled =
        MechanismControlBuilder.builder(control(ELEVATOR)).profile(Optional.empty()).build();
    double free = ROBOT.math().freeSpeed(ELEVATOR);

    assertThat(travel(ELEVATOR).cruise()).isEqualTo(1.2);
    assertThat(travel(ELEVATOR).acceleration()).isEqualTo(5);
    assertThat(Travel.of(ROBOT.math(), ELEVATOR, unprofiled).cruise()).isEqualTo(0.25 * free);
    assertThat(Travel.of(ROBOT.math(), ELEVATOR, unprofiled).acceleration())
        .isEqualTo(0.25 * free / 0.25);
  }

  @Test
  void aPositionIsHeldInsideItsLimits() {
    assertThat(travel(PIVOT).clamp(-1)).isEqualTo(0);
    assertThat(travel(PIVOT).clamp(1)).isEqualTo(1);
    assertThat(travel(PIVOT).clamp(3)).isCloseTo(radians(135), within(1e-12));
    assertThat(travel(TURRET).clamp(100)).isEqualTo(100);
  }

  /** A joint that turns freely takes a goal the short way round; one with stops, as it is. */
  @Test
  void aFreeJointsGoalIsTheShortWayRound() {
    assertThat(travel(TURRET).taking()).isEqualTo(Travel.Taking.SHORT_WAY_ROUND);
    assertThat(travel(TURRET).goalNear(radians(-170), radians(170)))
        .isCloseTo(radians(190), within(1e-9));
    assertThat(travel(PIVOT).taking()).isEqualTo(Travel.Taking.AS_IT_IS);
    assertThat(travel(PIVOT).goalNear(radians(200), 0)).isCloseTo(radians(135), within(1e-12));
  }

  /** At 170°, -170° is 20° on, at 190°; from 0°, -170° as it is; 370° from 0° is 10°. */
  @Test
  void aTurretTakesAHeadingAtItsNearestTurn() {
    Travel wide = travel(WIDE);

    assertThat(wide.taking()).isEqualTo(Travel.Taking.AS_A_HEADING);
    assertThat(wide.turnNear(radians(-170), radians(170))).isCloseTo(radians(190), within(1e-9));
    assertThat(wide.turnNear(radians(-170), 0)).isCloseTo(radians(-170), within(1e-9));
    assertThat(wide.turnNear(radians(370), 0)).isCloseTo(radians(10), within(1e-9));
    assertThat(wide.whyUnreachable(radians(370), 0)).isNull();
  }

  /**
   * A range counts as a whole turn from a hair short of one, since a whole turn written in degrees
   * converts to a floating-point step under 2π. The next range narrower does not.
   */
  @Test
  void aRangeAHairShortOfATurnStillSpansOne() {
    double hairShortOfATurn = 2 * Math.PI - 1e-9;

    assertThat(travel(turretSpanning(hairShortOfATurn)).taking())
        .isEqualTo(Travel.Taking.AS_A_HEADING);
    assertThat(travel(turretSpanning(Math.nextDown(hairShortOfATurn))).taking())
        .isEqualTo(Travel.Taking.AS_IT_IS);
  }

  /**
   * A joint that slides takes a position as it is however long its travel: 8 m is more than a
   * turn's worth of radians, and is no turn of anything.
   */
  @Test
  void aJointThatSlidesTakesAPositionAsItIs() {
    Joint.Prismatic longSlide =
        JointPrismaticBuilder.builder(ELEVATOR).range(Range.meters(0, 8)).build();

    assertThat(travel(longSlide).taking()).isEqualTo(Travel.Taking.AS_IT_IS);
    assertThat(travel(longSlide).turnNear(7, 0)).isEqualTo(7);
    assertThat(travel(longSlide).goalNear(7, 0)).isEqualTo(7);
  }

  /** The turret with stops this far apart, in radians, from 0 up. */
  private static Joint.Revolute turretSpanning(double width) {
    Range range = Range.builder().kind(Position.Kind.ANGLE).lower(0).upper(width).build();
    return JointRevoluteBuilder.builder(TURRET).range(range).build();
  }

  /**
   * Soft limits narrower than a turn leave headings with no turn inside: one is refused, saying so,
   * and taken at the limit nearer it either way round.
   */
  @Test
  void aHeadingWithNoTurnInsideIsRefusedAndHeldAtTheNearerLimit() {
    Travel soft =
        Travel.of(
            StandardRobot.robotOf(PIVOT, WIDE).math(),
            WIDE,
            control(WIDE).withSoftLimits(Range.degrees(-190, 160)));

    assertThat(soft.turnNear(radians(163), 0)).isCloseTo(radians(160), within(1e-12));
    assertThat(soft.turnNear(radians(-193), 0)).isCloseTo(radians(-190), within(1e-12));
    assertThat(soft.whyUnreachable(radians(165), 0))
        .isEqualTo("165°, at every turn of it, is outside turret's limits, -190° to 160°");
  }

  /** A joint narrower than a turn takes an angle as it is: one off by whole turns is a slip. */
  @Test
  void aJointNarrowerThanATurnTakesItsAngleAsItIs() {
    assertThat(travel(PIVOT).taking()).isEqualTo(Travel.Taking.AS_IT_IS);
    assertThat(travel(PIVOT).turnNear(radians(370), 0)).isEqualTo(radians(370));
    assertThat(travel(PIVOT).whyUnreachable(radians(370), 0))
        .isEqualTo("370° is outside pivot's limits, 0° to 135°");
    assertThat(travel(PIVOT).whyUnreachable(radians(90), 0)).isNull();
  }

  /** An angle far too big to tell its turns apart still ends its search, inside the limits. */
  @Test
  void anAngleTooBigToTurnStillEnds() {
    assertThat(travel(WIDE).turnNear(1e300, 0)).isBetween(radians(-200), radians(200));
  }

  /** Braking at 30 rad/s², from 1 rad at 3 rad/s it stops 0.15 rad on; back the other way, too. */
  @Test
  void itStopsWhereItsAccelerationBrakesIt() {
    assertThat(travel(PIVOT).stoppingPoint(1, 3)).isCloseTo(1.15, within(1e-12));
    assertThat(travel(PIVOT).stoppingPoint(1, -3)).isCloseTo(0.85, within(1e-12));
    assertThat(travel(PIVOT).stoppingPoint(0.05, -3)).as("inside its limits").isEqualTo(0);
  }

  /**
   * Held a loop on, the pivot goes on a loop at its speed, and faster by what it's speeding up, and
   * then brakes at its 30 rad/s²: from 1 rad at 3 rad/s, 1.06 rad on and 0.15 more; speeding up at
   * 50 rad/s², 4 rad/s a loop on, 1.07 rad on and 0.267 more. Slowing down counts as going on.
   */
  @Test
  void heldALoopOnItStopsWhereItsSpeedAndSpeedingUpTakeIt() {
    Travel pivot = travel(PIVOT);

    assertThat(pivot.stoppingPointLoopOn(1, 3, 0, LOOP)).isCloseTo(1.21, within(1e-12));
    assertThat(pivot.stoppingPointLoopOn(1, 3, 50, LOOP))
        .isCloseTo(1.07 + 16 / 60.0, within(1e-12));
    assertThat(pivot.stoppingPointLoopOn(1, 3, -50, LOOP)).isCloseTo(1.21, within(1e-12));
    assertThat(pivot.stoppingPointLoopOn(1, -3, -50, LOOP))
        .isCloseTo(0.93 - 16 / 60.0, within(1e-12));
    assertThat(pivot.stoppingPointLoopOn(1, 0, 50, LOOP)).as("still, it stays").isEqualTo(1);
    assertThat(pivot.stoppingPointLoopOn(1, 0, -50, LOOP))
        .as("still, it stays, whichever way it is pushed")
        .isEqualTo(1);
    assertThat(pivot.stoppingPointLoopOn(1, -3, 50, LOOP))
        .as("moving down and slowing: it goes on at its speed")
        .isCloseTo(0.79, within(1e-12));
    assertThat(pivot.stoppingPointLoopOn(1, Double.NaN, 0, LOOP))
        .as("a speed that was not read gives no stopping point, not a made-up one")
        .isNaN();
  }
}
