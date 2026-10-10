package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.OFF_THE_BOTTOM;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.OFF_THE_TOP;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.OUT_OF_THE_MIDDLE;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.WHEEL_OUT_OF_A_BAND;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.ARM;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.ARM_CLEARS_BUMPER;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.ELEVATOR;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;

import com.michaelgrundvig.frc.world.robot.Interlock.KeepsOut.Side;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * What a keep-out says about a position, with no robot running: whether the position is inside its
 * region, which side of the region a joint there is kept to, and how far past an edge it is. Each
 * is a table of questions and answers. A row holds its question as a lambda, so the model runs
 * inside the test and not while the table is built.
 */
class InterlockTest {
  // spotless:off
  private static final Condition ARM_DOWN = ARM.atMost(Degrees.of(0));

  // The elevator runs from 0 m to 1.2 m.
  private static final Interlock.KeepsOut ELEVATOR_OUT_OF_THE_MIDDLE = Interlock.that(ELEVATOR).keepsOut(Range.meters(0.2, 0.6)).when(ARM_DOWN);
  private static final Interlock.KeepsOut ELEVATOR_OFF_THE_BOTTOM = Interlock.that(ELEVATOR).keepsOut(Range.meters(0, 0.3)).when(ARM_DOWN);
  private static final Interlock.KeepsOut ELEVATOR_OFF_THE_TOP = Interlock.that(ELEVATOR).keepsOut(Range.meters(0.9, 1.2)).when(ARM_DOWN);

  private static final Optional<Side> NEVER_OUTSIDE = Optional.empty();
  private static final Optional<Side> LAST_BELOW = Optional.of(Side.BELOW);
  private static final Optional<Side> LAST_ABOVE = Optional.of(Side.ABOVE);
  // spotless:on

  @ParameterizedTest
  @MethodSource({"sidesKeptTo", "sidesAtTheEndsOfTheRange", "positionsInside", "edges"})
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments asks(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  private static double inRadians(double degrees) {
    return Math.toRadians(degrees);
  }

  // spotless:off

  /** Which side a joint is kept to, in a region with room on both sides. */
  static Stream<Arguments> sidesKeptTo() {
    return Stream.of(
        asks("outside, below the region", () -> ELEVATOR_OUT_OF_THE_MIDDLE.sideKeptTo(0.1, LAST_ABOVE), Side.BELOW),
        asks("outside, above the region", () -> ELEVATOR_OUT_OF_THE_MIDDLE.sideKeptTo(0.7, LAST_BELOW), Side.ABOVE),
        asks("at the lower edge, still below", () -> ELEVATOR_OUT_OF_THE_MIDDLE.sideKeptTo(0.2, LAST_ABOVE), Side.BELOW),
        asks("at the upper edge, still above", () -> ELEVATOR_OUT_OF_THE_MIDDLE.sideKeptTo(0.6, LAST_BELOW), Side.ABOVE),
        asks("inside, last out below", () -> ELEVATOR_OUT_OF_THE_MIDDLE.sideKeptTo(0.5, LAST_BELOW), Side.BELOW),
        asks("inside, last out above", () -> ELEVATOR_OUT_OF_THE_MIDDLE.sideKeptTo(0.3, LAST_ABOVE), Side.ABOVE),
        asks("inside, never out, nearer the lower edge", () -> ELEVATOR_OUT_OF_THE_MIDDLE.sideKeptTo(0.3, NEVER_OUTSIDE), Side.BELOW),
        asks("inside, never out, nearer the upper edge", () -> ELEVATOR_OUT_OF_THE_MIDDLE.sideKeptTo(0.5, NEVER_OUTSIDE), Side.ABOVE),
        asks("inside, never out, exactly halfway: below", () -> OUT_OF_THE_MIDDLE.sideKeptTo(6), Side.BELOW),
        asks("inside, never out, a hair past halfway", () -> OUT_OF_THE_MIDDLE.sideKeptTo(Math.nextUp(6.0)), Side.ABOVE),
        asks("with no range, at the lower edge", () -> WHEEL_OUT_OF_A_BAND.sideKeptTo(1, LAST_ABOVE), Side.BELOW),
        asks("with no range, at the upper edge", () -> WHEEL_OUT_OF_A_BAND.sideKeptTo(2, LAST_BELOW), Side.ABOVE));
  }

  /** A region that reaches an end of the joint's range has no room on that side to leave by. */
  static Stream<Arguments> sidesAtTheEndsOfTheRange() {
    return Stream.of(
        asks("inside at the bottom, last out below: no room below", () -> ELEVATOR_OFF_THE_BOTTOM.sideKeptTo(0.1, LAST_BELOW), Side.ABOVE),
        asks("inside at the bottom, never out", () -> ELEVATOR_OFF_THE_BOTTOM.sideKeptTo(0.1, NEVER_OUTSIDE), Side.ABOVE),
        asks("inside at the top, last out above: no room above", () -> ELEVATOR_OFF_THE_TOP.sideKeptTo(1.1, LAST_ABOVE), Side.BELOW),
        asks("inside at the top, never out", () -> ELEVATOR_OFF_THE_TOP.sideKeptTo(1.1, NEVER_OUTSIDE), Side.BELOW),
        asks("standing on the low end of the range", () -> OFF_THE_BOTTOM.sideKeptTo(0, LAST_BELOW), Side.ABOVE),
        asks("standing on the high end of the range", () -> OFF_THE_TOP.sideKeptTo(16, LAST_ABOVE), Side.BELOW),
        asks("at the edge that does have room", () -> OFF_THE_BOTTOM.sideKeptTo(4, LAST_BELOW), Side.ABOVE),
        asks("reading past the low end of the range", () -> OFF_THE_BOTTOM.sideKeptTo(-1, LAST_BELOW), Side.ABOVE),
        asks("reading past the high end of the range", () -> OFF_THE_TOP.sideKeptTo(17, LAST_ABOVE), Side.BELOW));
  }

  /** An edge is outside the region, unless it is an end of the joint's range. */
  static Stream<Arguments> positionsInside() {
    return Stream.of(
        asks("a band's upper edge is out of it", () -> ARM_CLEARS_BUMPER.isInside(inRadians(30)), false),
        asks("just inside a band's upper edge", () -> ARM_CLEARS_BUMPER.isInside(inRadians(29)), true),
        asks("the lower edge is outside", () -> OUT_OF_THE_MIDDLE.isInside(4), false),
        asks("a hair past the lower edge is inside", () -> OUT_OF_THE_MIDDLE.isInside(Math.nextUp(4.0)), true),
        asks("a hair short of the upper edge is inside", () -> OUT_OF_THE_MIDDLE.isInside(Math.nextDown(8.0)), true),
        asks("the upper edge is outside", () -> OUT_OF_THE_MIDDLE.isInside(8), false),
        asks("below the region is outside", () -> OUT_OF_THE_MIDDLE.isInside(3), false),
        asks("above the region is outside", () -> OUT_OF_THE_MIDDLE.isInside(9), false),
        asks("the low end of the range is inside", () -> OFF_THE_BOTTOM.isInside(0), true),
        asks("that region's other edge is outside", () -> OFF_THE_BOTTOM.isInside(4), false),
        asks("the high end of the range is inside", () -> OFF_THE_TOP.isInside(16), true),
        asks("that region's other edge is outside too", () -> OFF_THE_TOP.isInside(12), false),
        asks("past the low end of the range counts as outside", () -> OFF_THE_BOTTOM.isInside(-1), false),
        asks("past the high end of the range counts as outside", () -> OFF_THE_TOP.isInside(17), false),
        asks("with no range, the lower edge is outside", () -> WHEEL_OUT_OF_A_BAND.isInside(1), false),
        asks("with no range, the upper edge is outside", () -> WHEEL_OUT_OF_A_BAND.isInside(2), false),
        asks("a position that is not a number is not inside", () -> OUT_OF_THE_MIDDLE.isInside(Double.NaN), false));
  }

  /** Where each side's edge is, and how far past it a position is, measured into the region. */
  static Stream<Arguments> edges() {
    return Stream.of(
        asks("the edge below is the region's lower", () -> OUT_OF_THE_MIDDLE.edge(Side.BELOW), 4.0),
        asks("the edge above is the region's upper", () -> OUT_OF_THE_MIDDLE.edge(Side.ABOVE), 8.0),
        asks("from below, 1 m into the region", () -> OUT_OF_THE_MIDDLE.distancePastEdge(Side.BELOW, 5), 1.0),
        asks("from below, 1 m short of it", () -> OUT_OF_THE_MIDDLE.distancePastEdge(Side.BELOW, 3), -1.0),
        asks("from above, 3 m into the region", () -> OUT_OF_THE_MIDDLE.distancePastEdge(Side.ABOVE, 5), 3.0),
        asks("from above, 1 m short of it", () -> OUT_OF_THE_MIDDLE.distancePastEdge(Side.ABOVE, 9), -1.0));
  }
  // spotless:on
}
