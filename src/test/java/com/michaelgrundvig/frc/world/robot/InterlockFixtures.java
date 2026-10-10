package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.StandardRobot.ELEVATOR;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.ELEVATOR_CLEAR;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Optional;

/**
 * The joints and interlocks the interlock tests share. Every number is a small whole number or a
 * power of two, so that a row can put a joint exactly on an edge and say exactly where it stops:
 * the rail brakes at 2 m/s², so from 2 m/s it stops 1 m on, and a loop lasts half a second.
 */
final class InterlockFixtures {
  private InterlockFixtures() {}

  /** How long a loop lasts in these tests, in seconds. */
  static final double LOOP_SECONDS = 0.5;

  /** A rail 16 m long, which holds positions. */
  static final Joint.Prismatic RAIL =
      StandardRobot.lightSlide("rail", new Vector3(1, 0, 0), Range.meters(0, 16));

  /** The rail brakes at 2 m/s², and a full battery runs it at 8 m/s, so 3 V runs it at 2 m/s. */
  static final Travel RAIL_TRAVEL = new Travel(RAIL, 0, 16, false, false, 4, 2, 8);

  /** A wheel that turns freely: it has no range and no named positions, so it holds none. */
  static final Joint.Revolute WHEEL = StandardRobot.lightPivot("wheel", Optional.empty());

  /** The wheel brakes at 2 rad/s², and takes a goal the short way round. */
  static final Travel WHEEL_TRAVEL =
      new Travel(WHEEL, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, true, false, 4, 2, 8);

  // What each condition is about does not matter to these tests, which say what it reads.
  private static final Condition ELEVATOR_LOW = ELEVATOR.atMost(ELEVATOR_CLEAR);
  private static final Condition ELEVATOR_HIGH = ELEVATOR.atLeast(Inches.of(30));

  /** The rail kept out of 4 m to 8 m, with room on both sides. */
  static final Interlock.KeepsOut OUT_OF_THE_MIDDLE =
      Interlock.that(RAIL).keepsOut(Range.meters(4, 8)).when(ELEVATOR_LOW);

  /** The rail kept out of its first 4 m: the region reaches the low end of its range. */
  static final Interlock.KeepsOut OFF_THE_BOTTOM =
      Interlock.that(RAIL).keepsOut(Range.meters(0, 4)).when(ELEVATOR_LOW);

  /** The rail kept out of its last 4 m: the region reaches the high end of its range. */
  static final Interlock.KeepsOut OFF_THE_TOP =
      Interlock.that(RAIL).keepsOut(Range.meters(12, 16)).when(ELEVATOR_LOW);

  /** The wheel kept out of 1 rad to 2 rad. It has no range, so there is always room past both. */
  static final Interlock.KeepsOut WHEEL_OUT_OF_A_BAND =
      Interlock.that(WHEEL)
          .keepsOut(Range.builder().kind(Position.Kind.ANGLE).lower(1).upper(2).build())
          .when(ELEVATOR_LOW);

  static final Interlock.RunsOnlyWhen RAIL_RUNS_ONLY_WHEN_LOW =
      Interlock.that(RAIL).runsOnlyWhen(ELEVATOR_LOW);

  static final Interlock.RunsOnlyWhen RAIL_RUNS_ONLY_WHEN_HIGH =
      Interlock.that(RAIL).runsOnlyWhen(ELEVATOR_HIGH);

  static final Interlock.RunsOnlyWhen WHEEL_RUNS_ONLY_WHEN_LOW =
      Interlock.that(WHEEL).runsOnlyWhen(ELEVATOR_LOW);

  /** What enforces these interlocks, in this order, on the rail. */
  static InterlockEnforcement onTheRail(Interlock... interlocks) {
    return new InterlockEnforcement(RAIL_TRAVEL, List.of(interlocks), LOOP_SECONDS);
  }

  /** What enforces these interlocks, in this order, on the wheel. */
  static InterlockEnforcement onTheWheel(Interlock... interlocks) {
    return new InterlockEnforcement(WHEEL_TRAVEL, List.of(interlocks), LOOP_SECONDS);
  }
}
