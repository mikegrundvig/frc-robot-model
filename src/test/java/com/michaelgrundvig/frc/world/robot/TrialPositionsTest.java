package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.StandardRobot.ELEVATOR;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.lightPivot;
import static com.michaelgrundvig.frc.world.robot.TrialPositions.STEP;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.util.Collection;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The positions conditions are tried at, as a table: a stretch of a joint's travel and the
 * conditions on it, and every position that must be tried, in order. The elevator runs from 0 m to
 * 1.2 m; the wheel turns freely and has no range.
 */
class TrialPositionsTest {
  private static final double NO_END_BELOW = Double.NEGATIVE_INFINITY;
  private static final double NO_END_ABOVE = Double.POSITIVE_INFINITY;

  private static final Joint.Revolute WHEEL = lightPivot("wheel", Optional.empty());

  private static Condition.OnJoint elevatorAtLeast(double meters) {
    return ELEVATOR.atLeast(Meters.of(meters));
  }

  private static Condition.OnJoint elevatorBetween(double from, double to) {
    return ELEVATOR.between(Meters.of(from), Meters.of(to));
  }

  private static Condition.OnJoint wheelAtMost(double radians) {
    return WHEEL.atMost(Radians.of(radians));
  }

  private static double between(double one, double other) {
    return (one + other) / 2;
  }

  @ParameterizedTest
  @MethodSource({"withinAStretch", "acrossARange"})
  void itTries(Supplier<Collection<Double>> tried, List<Double> positions) {
    assertThat(tried.get()).containsExactlyElementsOf(positions);
  }

  private static Arguments tries(
      String label, Supplier<Collection<Double>> tried, Double... positions) {
    return argumentSet(label, tried, List.of(positions));
  }

  // spotless:off

  /** One condition, within the stretch a held joint may still be in. */
  static Stream<Arguments> withinAStretch() {
    return Stream.of(
        tries("an edge inside the stretch: its ends, the edge, a step either side, and between each",
            () -> TrialPositions.within(0, 1, elevatorAtLeast(0.5)),
            0.0, between(0, 0.5 - STEP), 0.5 - STEP, between(0.5 - STEP, 0.5), 0.5,
            between(0.5, 0.5 + STEP), 0.5 + STEP, between(0.5 + STEP, 1), 1.0),
        tries("an edge outside the stretch is not tried",
            () -> TrialPositions.within(0, 0.25, elevatorAtLeast(0.5)),
            0.0, 0.125, 0.25),
        tries("an edge on the stretch's low end: the step below it is outside",
            () -> TrialPositions.within(0.5, 1, elevatorAtLeast(0.5)),
            0.5, between(0.5, 0.5 + STEP), 0.5 + STEP, between(0.5 + STEP, 1), 1.0),
        tries("an edge on the stretch's high end: the step above it is outside",
            () -> TrialPositions.within(0, 0.5, elevatorAtLeast(0.5)),
            0.0, between(0, 0.5 - STEP), 0.5 - STEP, between(0.5 - STEP, 0.5), 0.5),
        tries("no end below: a unit below the lowest point tried",
            () -> TrialPositions.within(NO_END_BELOW, 1, wheelAtMost(0.5)),
            0.5 - STEP - 1, between(0.5 - STEP - 1, 0.5 - STEP), 0.5 - STEP,
            between(0.5 - STEP, 0.5), 0.5, between(0.5, 0.5 + STEP), 0.5 + STEP,
            between(0.5 + STEP, 1), 1.0),
        tries("no end below and no edge inside: a unit below the high end",
            () -> TrialPositions.within(NO_END_BELOW, 0.25, wheelAtMost(0.5)),
            -0.75, -0.25, 0.25),
        tries("no end above: a unit above the highest point tried",
            () -> TrialPositions.within(0.25, NO_END_ABOVE, wheelAtMost(0.5)),
            0.25, between(0.25, 0.5 - STEP), 0.5 - STEP, between(0.5 - STEP, 0.5), 0.5,
            between(0.5, 0.5 + STEP), 0.5 + STEP, between(0.5 + STEP, 0.5 + STEP + 1),
            0.5 + STEP + 1),
        tries("no end above and no edge inside: a unit above the low end",
            () -> TrialPositions.within(0.75, NO_END_ABOVE, wheelAtMost(0.5)),
            0.75, 1.25, 1.75),
        tries("a stretch that is one place is tried there",
            () -> TrialPositions.within(0.5, 0.5, elevatorAtLeast(0.5)),
            0.5));
  }

  /** Several conditions, across the whole of a joint's range. */
  static Stream<Arguments> acrossARange() {
    return Stream.of(
        tries("the range's ends, each edge, a step either side, and between the edges",
            () -> TrialPositions.acrossRange(ELEVATOR, List.of(elevatorAtLeast(0.25), elevatorAtLeast(0.75))),
            0.0, 0.25 - STEP, 0.25, 0.25 + STEP, 0.5, 0.75 - STEP, 0.75, 0.75 + STEP, 1.2),
        tries("a band's two edges are both tried",
            () -> TrialPositions.acrossRange(ELEVATOR, List.of(elevatorBetween(0.25, 0.75))),
            0.0, 0.25 - STEP, 0.25, 0.25 + STEP, 0.5, 0.75 - STEP, 0.75, 0.75 + STEP, 1.2),
        tries("a step past the range's end is outside it, and not tried",
            () -> TrialPositions.acrossRange(ELEVATOR, List.of(ELEVATOR.atMost(Meters.of(1.2 - STEP / 2)))),
            0.0, 1.2 - STEP / 2 - STEP, 1.2 - STEP / 2, 1.2),
        tries("a joint with no range: from a unit below the lowest edge to a unit above the highest",
            () -> TrialPositions.acrossRange(WHEEL, List.of(wheelAtMost(0.5), wheelAtMost(1.5))),
            -0.5, 0.5 - STEP, 0.5, 0.5 + STEP, 1.0, 1.5 - STEP, 1.5, 1.5 + STEP, 2.5));
  }

  // spotless:on
}
