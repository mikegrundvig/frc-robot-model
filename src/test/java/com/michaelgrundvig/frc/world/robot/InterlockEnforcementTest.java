package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.OFF_THE_BOTTOM;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.OFF_THE_TOP;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.OUT_OF_THE_MIDDLE;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.RAIL_RUNS_ONLY_WHEN_HIGH;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.RAIL_RUNS_ONLY_WHEN_LOW;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.WHEEL_OUT_OF_A_BAND;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.WHEEL_RUNS_ONLY_WHEN_LOW;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.onTheRail;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.onTheWheel;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import com.michaelgrundvig.frc.world.robot.Interlock.KeepsOut.Side;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * What a joint's interlocks allow in one loop, as tables of scenarios on the joints of {@link
 * InterlockFixtures}. Each row gives what the interlocks remembered, what the joint is asked and
 * what was read, and the whole verdict that must come back: what the joint may do, which interlocks
 * hold it, what they remember now, and whether its profile restarts. A row holds its question as a
 * lambda, so the model runs inside the test and not while the table is built.
 *
 * <p>The rail's keep-out in most rows is 4 m to 8 m. The rail brakes at 2 m/s² and a loop lasts
 * half a second, so from 2 m/s it stops 1 m on if it is held now, and 2 m on if it runs one more
 * loop first.
 */
class InterlockEnforcementTest {
  // spotless:off
  private static final InterlockEnforcement MIDDLE = onTheRail(OUT_OF_THE_MIDDLE);
  private static final InterlockEnforcement BOTTOM = onTheRail(OFF_THE_BOTTOM);
  private static final InterlockEnforcement TOP = onTheRail(OFF_THE_TOP);
  private static final InterlockEnforcement WHEEL_BAND = onTheWheel(WHEEL_OUT_OF_A_BAND);
  private static final InterlockEnforcement RAIL_RUNS_ONLY = onTheRail(RAIL_RUNS_ONLY_WHEN_LOW);
  private static final InterlockEnforcement WHEEL_RUNS_ONLY = onTheWheel(WHEEL_RUNS_ONLY_WHEN_LOW);
  private static final InterlockEnforcement TWO_RUN_ONLYS = onTheRail(RAIL_RUNS_ONLY_WHEN_LOW, RAIL_RUNS_ONLY_WHEN_HIGH);
  private static final InterlockEnforcement KEEP_OUT_THEN_RUNS_ONLY = onTheRail(OUT_OF_THE_MIDDLE, RAIL_RUNS_ONLY_WHEN_LOW);
  private static final InterlockEnforcement RUNS_ONLY_THEN_KEEP_OUT = onTheRail(RAIL_RUNS_ONLY_WHEN_LOW, OUT_OF_THE_MIDDLE);
  private static final InterlockEnforcement MIDDLE_AND_TOP = onTheRail(OUT_OF_THE_MIDDLE, RAIL_RUNS_ONLY_WHEN_LOW, OFF_THE_TOP);

  // What one keep-out remembers.
  private static final KeepOutState NEVER_OUTSIDE = KeepOutState.INITIAL;
  private static final KeepOutState LAST_BELOW = KeepOutState.notHolding(Optional.of(Side.BELOW));
  private static final KeepOutState LAST_ABOVE = KeepOutState.notHolding(Optional.of(Side.ABOVE));
  private static final KeepOutState HELD_AT_LOWER_EDGE = KeepOutState.heldAtEdge(Optional.of(Side.BELOW));
  private static final KeepOutState HELD_AT_UPPER_EDGE = KeepOutState.heldAtEdge(Optional.of(Side.ABOVE));

  // What a run-only-when's condition reads, and a keep-out's: a keep-out is closed while its holds.
  private static final ConditionReading MAY_RUN = ConditionReading.HOLDS;
  private static final ConditionReading MAY_NOT_RUN = ConditionReading.DOES_NOT_HOLD;
  private static final ConditionReading CLOSED = ConditionReading.HOLDS;
  private static final ConditionReading OPEN = ConditionReading.DOES_NOT_HOLD;
  private static final ConditionReading LOST = ConditionReading.UNKNOWN;

  private static final JointAsk NEUTRAL = JointAsk.NEUTRAL;
  // spotless:on

  /** What a joint with one keep-out, held by no run-only-when, remembers. */
  private static InterlockState remembering(KeepOutState keepOut) {
    return new InterlockState(OptionalDouble.empty(), List.of(keepOut));
  }

  /** What a joint with only run-only-whens remembers: where it is held, if it is. */
  private static InterlockState stoppedAt(double position) {
    return new InterlockState(OptionalDouble.of(position), List.of());
  }

  private static final InterlockState RUNNING =
      new InterlockState(OptionalDouble.empty(), List.of());

  /** Held inside a region at a point, last outside on a side, or never. */
  private static KeepOutState heldInside(double at, KeepOutState before) {
    return KeepOutState.heldInside(before.lastOutside(), at);
  }

  /** Asked for a position at half the joint's cruise, as a command might. */
  private static JointAsk.ToPosition goTo(double position) {
    return new JointAsk.ToPosition(position, 0.5);
  }

  /** A position an interlock holds the joint at: always at its full cruise. */
  private static JointAsk.ToPosition holdAt(double position) {
    return new JointAsk.ToPosition(position, JointAsk.FULL_CRUISE);
  }

  private static JointAsk speed(double metersPerSecond) {
    return new JointAsk.AtVelocity(metersPerSecond);
  }

  private static JointAsk volts(double volts) {
    return new JointAsk.AtVoltage(volts);
  }

  private static JointAsk share(double share) {
    return new JointAsk.AtDutyCycle(share);
  }

  /** The joint at rest at a position, with these condition readings, one for each interlock. */
  private static InterlockReadings at(double position, ConditionReading... conditions) {
    return moving(position, 0, conditions);
  }

  /** The joint at a position, moving at a steady speed. */
  private static InterlockReadings moving(
      double position, double velocity, ConditionReading... conditions) {
    return new InterlockReadings(position, velocity, 0, List.of(conditions));
  }

  /** The joint at a position, moving, its speed changing. */
  private static InterlockReadings changingSpeed(
      double position, double velocity, double acceleration, ConditionReading... conditions) {
    return new InterlockReadings(position, velocity, acceleration, List.of(conditions));
  }

  /** The verdict that nothing holds the joint: it may do what it was asked. */
  private static InterlockVerdict leftAlone(JointAsk asked, InterlockState now) {
    return new InterlockVerdict(now, asked, List.of(), false);
  }

  /** The verdict that interlocks hold the joint, in a hold that was already under way. */
  private static InterlockVerdict held(JointAsk allowed, InterlockState now, Interlock... by) {
    return new InterlockVerdict(now, allowed, List.of(by), false);
  }

  /** The verdict that interlocks hold the joint in a hold that starts this loop: it restarts. */
  private static InterlockVerdict heldFromNow(
      JointAsk allowed, InterlockState now, Interlock... by) {
    return new InterlockVerdict(now, allowed, List.of(by), true);
  }

  @ParameterizedTest
  @MethodSource({
    "aPositionAskedFromOutside",
    "aPositionAskedFromInside",
    "anOpenRegionAndNeutral",
    "anOutputAskedFromOutside",
    "anOutputHeldAtTheEdge",
    "anOutputAskedFromInside",
    "aRegionReachingTheEndOfTheRange",
    "aJointThatTurnsFreely",
    "runsOnlyWhen",
    "severalInterlocks"
  })
  void itDecides(Supplier<InterlockVerdict> question, InterlockVerdict answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments decides(
      String label, Supplier<InterlockVerdict> question, InterlockVerdict answer) {
    return argumentSet(label, question, answer);
  }

  // spotless:off

  /** Outside a closed region, a goal beyond the edge on the joint's side is moved to that edge. */
  static Stream<Arguments> aPositionAskedFromOutside() {
    return Stream.of(
        decides("below, a goal above the region: stopped at the lower edge",
            () -> MIDDLE.decide(remembering(NEVER_OUTSIDE), goTo(10), at(2, CLOSED)),
            held(holdAt(4), remembering(LAST_BELOW), OUT_OF_THE_MIDDLE)),
        decides("below, a goal inside the region: stopped at the lower edge",
            () -> MIDDLE.decide(remembering(LAST_BELOW), goTo(5), at(2, CLOSED)),
            held(holdAt(4), remembering(LAST_BELOW), OUT_OF_THE_MIDDLE)),
        decides("below, a goal short of the region: left alone, at its own share of the cruise",
            () -> MIDDLE.decide(remembering(LAST_BELOW), goTo(3), at(2, CLOSED)),
            leftAlone(goTo(3), remembering(LAST_BELOW))),
        decides("below, a goal exactly at the lower edge: left alone",
            () -> MIDDLE.decide(remembering(LAST_BELOW), goTo(4), at(2, CLOSED)),
            leftAlone(goTo(4), remembering(LAST_BELOW))),
        decides("standing on the lower edge, a goal above: held there",
            () -> MIDDLE.decide(remembering(LAST_BELOW), goTo(10), at(4, CLOSED)),
            held(holdAt(4), remembering(LAST_BELOW), OUT_OF_THE_MIDDLE)),
        decides("above, a goal below the region: stopped at the upper edge",
            () -> MIDDLE.decide(remembering(LAST_BELOW), goTo(2), at(10, CLOSED)),
            held(holdAt(8), remembering(LAST_ABOVE), OUT_OF_THE_MIDDLE)),
        decides("above, a goal exactly at the upper edge: left alone",
            () -> MIDDLE.decide(remembering(LAST_ABOVE), goTo(8), at(10, CLOSED)),
            leftAlone(goTo(8), remembering(LAST_ABOVE))),
        decides("the condition's sensor lost: the region counts as closed",
            () -> MIDDLE.decide(remembering(LAST_BELOW), goTo(10), at(2, LOST)),
            held(holdAt(4), remembering(LAST_BELOW), OUT_OF_THE_MIDDLE)));
  }

  /**
   * Inside a closed region, the joint may go to a goal that takes it out by the side it is kept
   * to. Any other goal holds it at the point where it can stop, which is worked out once.
   */
  static Stream<Arguments> aPositionAskedFromInside() {
    return Stream.of(
        decides("came in from below at 2 m/s, a goal above: held where it stops, 1 m on",
            () -> MIDDLE.decide(remembering(LAST_BELOW), goTo(10), moving(5, 2, CLOSED)),
            heldFromNow(holdAt(6), remembering(heldInside(6, LAST_BELOW)), OUT_OF_THE_MIDDLE)),
        decides("a loop later, slower: still held at the same point",
            () -> MIDDLE.decide(remembering(heldInside(6, LAST_BELOW)), goTo(10), moving(5.5, 1, CLOSED)),
            held(holdAt(6), remembering(heldInside(6, LAST_BELOW)), OUT_OF_THE_MIDDLE)),
        decides("came in from below, a goal back below: it may leave, and the hold is forgotten",
            () -> MIDDLE.decide(remembering(heldInside(6, LAST_BELOW)), goTo(2), at(5, CLOSED)),
            leftAlone(goTo(2), remembering(LAST_BELOW))),
        decides("came in from below, a goal exactly at the lower edge: it may leave",
            () -> MIDDLE.decide(remembering(LAST_BELOW), goTo(4), at(5, CLOSED)),
            leftAlone(goTo(4), remembering(LAST_BELOW))),
        decides("came in from above, a goal below: it may not cross, and is held",
            () -> MIDDLE.decide(remembering(LAST_ABOVE), goTo(2), at(5, CLOSED)),
            heldFromNow(holdAt(5), remembering(heldInside(5, LAST_ABOVE)), OUT_OF_THE_MIDDLE)),
        decides("came in from above, a goal exactly at the upper edge: it may leave",
            () -> MIDDLE.decide(remembering(LAST_ABOVE), goTo(8), at(5, CLOSED)),
            leftAlone(goTo(8), remembering(LAST_ABOVE))),
        decides("started inside, nearer the lower edge, a goal below: it may leave",
            () -> MIDDLE.decide(remembering(NEVER_OUTSIDE), goTo(2), at(5, CLOSED)),
            leftAlone(goTo(2), remembering(NEVER_OUTSIDE))),
        decides("started inside, nearer the lower edge, a goal above: held",
            () -> MIDDLE.decide(remembering(NEVER_OUTSIDE), goTo(10), at(5, CLOSED)),
            heldFromNow(holdAt(5), remembering(heldInside(5, NEVER_OUTSIDE)), OUT_OF_THE_MIDDLE)),
        decides("started inside, nearer the upper edge, a goal above: it may leave",
            () -> MIDDLE.decide(remembering(NEVER_OUTSIDE), goTo(10), at(7, CLOSED)),
            leftAlone(goTo(10), remembering(NEVER_OUTSIDE))),
        decides("asked to stay exactly where it stops: its profile restarts, but nothing is changed",
            () -> MIDDLE.decide(remembering(NEVER_OUTSIDE), goTo(5), at(5, CLOSED)),
            new InterlockVerdict(remembering(heldInside(5, NEVER_OUTSIDE)), goTo(5), List.of(), true)),
        decides("closed on it at 4 m/s, too fast to stop inside: its stopping point is out the far side",
            () -> MIDDLE.decide(remembering(LAST_BELOW), goTo(10), moving(7, 4, CLOSED)),
            heldFromNow(holdAt(11), remembering(heldInside(11, LAST_BELOW)), OUT_OF_THE_MIDDLE)),
        decides("once out the far side, it is kept out from there",
            () -> MIDDLE.decide(remembering(heldInside(11, LAST_BELOW)), goTo(10), moving(9, 2, CLOSED)),
            leftAlone(goTo(10), remembering(LAST_ABOVE))));
  }

  /** An open region restricts nothing, and neither does neutral, but the side is still tracked. */
  static Stream<Arguments> anOpenRegionAndNeutral() {
    return Stream.of(
        decides("open, a goal across the region: left alone, and the side it is on is noted",
            () -> MIDDLE.decide(remembering(NEVER_OUTSIDE), goTo(10), at(2, OPEN)),
            leftAlone(goTo(10), remembering(LAST_BELOW))),
        decides("open, inside: the side it came in from is kept",
            () -> MIDDLE.decide(remembering(LAST_ABOVE), goTo(10), at(5, OPEN)),
            leftAlone(goTo(10), remembering(LAST_ABOVE))),
        decides("the region opens while it is held inside: let go, the hold forgotten",
            () -> MIDDLE.decide(remembering(heldInside(6, LAST_BELOW)), goTo(10), at(6, OPEN)),
            leftAlone(goTo(10), remembering(LAST_BELOW))),
        decides("the region opens while an output is held at its edge: let go",
            () -> MIDDLE.decide(remembering(HELD_AT_LOWER_EDGE), volts(3), at(4, OPEN)),
            leftAlone(volts(3), remembering(LAST_BELOW))),
        decides("neutral inside a closed region: left alone",
            () -> MIDDLE.decide(remembering(heldInside(6, LAST_BELOW)), NEUTRAL, at(6, CLOSED)),
            leftAlone(NEUTRAL, remembering(LAST_BELOW))),
        decides("neutral while coasting toward a closed region: left alone",
            () -> MIDDLE.decide(remembering(HELD_AT_LOWER_EDGE), NEUTRAL, moving(3, 2, CLOSED)),
            leftAlone(NEUTRAL, remembering(LAST_BELOW))));
  }

  /**
   * Run at an output outside a closed region, the joint is held at the edge from the loop in which
   * running one more loop and then braking would bring it to the edge or past it.
   */
  static Stream<Arguments> anOutputAskedFromOutside() {
    return Stream.of(
        decides("at 2 m/s from 1 m: a loop on it would stop at 3 m, short of the edge",
            () -> MIDDLE.decide(remembering(LAST_BELOW), speed(2), moving(1, 2, CLOSED)),
            leftAlone(speed(2), remembering(LAST_BELOW))),
        decides("at 2 m/s from 2 m: a loop on it would stop exactly at the edge, so it is held now",
            () -> MIDDLE.decide(remembering(LAST_BELOW), speed(2), moving(2, 2, CLOSED)),
            held(holdAt(4), remembering(HELD_AT_LOWER_EDGE), OUT_OF_THE_MIDDLE)),
        decides("speeding up at 4 m/s² from 1 m: it would stop past the edge",
            () -> MIDDLE.decide(remembering(LAST_BELOW), speed(2), changingSpeed(1, 2, 4, CLOSED)),
            held(holdAt(4), remembering(HELD_AT_LOWER_EDGE), OUT_OF_THE_MIDDLE)),
        decides("slowing down from 1 m: judged as if steady, never later",
            () -> MIDDLE.decide(remembering(LAST_BELOW), speed(2), changingSpeed(1, 2, -4, CLOSED)),
            leftAlone(speed(2), remembering(LAST_BELOW))),
        decides("at rest at 2 m, asked for 2 m/s: held, for it would reach that speed within a loop",
            () -> MIDDLE.decide(remembering(LAST_BELOW), speed(2), at(2, CLOSED)),
            held(holdAt(4), remembering(HELD_AT_LOWER_EDGE), OUT_OF_THE_MIDDLE)),
        decides("at rest at 1 m, asked for 2 m/s: left alone",
            () -> MIDDLE.decide(remembering(LAST_BELOW), speed(2), at(1, CLOSED)),
            leftAlone(speed(2), remembering(LAST_BELOW))),
        decides("at rest at 2 m, asked for 3 V, which runs it at 2 m/s: held",
            () -> MIDDLE.decide(remembering(LAST_BELOW), volts(3), at(2, CLOSED)),
            held(holdAt(4), remembering(HELD_AT_LOWER_EDGE), OUT_OF_THE_MIDDLE)),
        decides("at rest at 1 m, asked for 3 V: left alone",
            () -> MIDDLE.decide(remembering(LAST_BELOW), volts(3), at(1, CLOSED)),
            leftAlone(volts(3), remembering(LAST_BELOW))),
        decides("at rest at 2 m, asked for a quarter of the battery, 2 m/s: held",
            () -> MIDDLE.decide(remembering(LAST_BELOW), share(0.25), at(2, CLOSED)),
            held(holdAt(4), remembering(HELD_AT_LOWER_EDGE), OUT_OF_THE_MIDDLE)),
        decides("at rest at 1 m, asked for a quarter of the battery: left alone",
            () -> MIDDLE.decide(remembering(LAST_BELOW), share(0.25), at(1, CLOSED)),
            leftAlone(share(0.25), remembering(LAST_BELOW))),
        decides("moving toward the region at 2 m/s from 2 m, though asked to run away from it: held",
            () -> MIDDLE.decide(remembering(LAST_BELOW), speed(-1), moving(2, 2, CLOSED)),
            held(holdAt(4), remembering(HELD_AT_LOWER_EDGE), OUT_OF_THE_MIDDLE)),
        decides("above, at -2 m/s from 10 m: it would stop exactly at the upper edge, so it is held",
            () -> MIDDLE.decide(remembering(LAST_ABOVE), speed(-2), moving(10, -2, CLOSED)),
            held(holdAt(8), remembering(HELD_AT_UPPER_EDGE), OUT_OF_THE_MIDDLE)),
        decides("above, at -2 m/s from 11 m: it would stop at 9 m, short of the edge",
            () -> MIDDLE.decide(remembering(LAST_ABOVE), speed(-2), moving(11, -2, CLOSED)),
            leftAlone(speed(-2), remembering(LAST_ABOVE))),
        decides("at rest exactly on the edge, asked to run away from the region: held all the same",
            () -> MIDDLE.decide(remembering(LAST_BELOW), speed(-2), at(4, CLOSED)),
            held(holdAt(4), remembering(HELD_AT_LOWER_EDGE), OUT_OF_THE_MIDDLE)));
  }

  /**
   * Once an output is held at the edge, it stays held for as long as it pushes toward the region,
   * even at rest, where it no longer looks as though it would reach.
   */
  static Stream<Arguments> anOutputHeldAtTheEdge() {
    return Stream.of(
        decides("held, at rest just short, still pushed toward the region: stays held",
            () -> MIDDLE.decide(remembering(HELD_AT_LOWER_EDGE), volts(0.75), at(3.5, CLOSED)),
            held(holdAt(4), remembering(HELD_AT_LOWER_EDGE), OUT_OF_THE_MIDDLE)),
        decides("the same push, but it was not held: left alone",
            () -> MIDDLE.decide(remembering(LAST_BELOW), volts(0.75), at(3.5, CLOSED)),
            leftAlone(volts(0.75), remembering(LAST_BELOW))),
        decides("held, then pushed away from the region: let go",
            () -> MIDDLE.decide(remembering(HELD_AT_LOWER_EDGE), volts(-3), at(3.5, CLOSED)),
            leftAlone(volts(-3), remembering(LAST_BELOW))),
        decides("held, then asked for no output at all: let go",
            () -> MIDDLE.decide(remembering(HELD_AT_LOWER_EDGE), volts(0), at(3.5, CLOSED)),
            leftAlone(volts(0), remembering(LAST_BELOW))),
        decides("held at the upper edge, still pushed down toward the region: stays held",
            () -> MIDDLE.decide(remembering(HELD_AT_UPPER_EDGE), volts(-0.75), at(8.5, CLOSED)),
            held(holdAt(8), remembering(HELD_AT_UPPER_EDGE), OUT_OF_THE_MIDDLE)),
        decides("held at the upper edge, then pushed up and away: let go",
            () -> MIDDLE.decide(remembering(HELD_AT_UPPER_EDGE), volts(3), at(8.5, CLOSED)),
            leftAlone(volts(3), remembering(LAST_ABOVE))),
        decides("held at the upper edge, then asked for no output at all: let go",
            () -> MIDDLE.decide(remembering(HELD_AT_UPPER_EDGE), volts(0), at(8.5, CLOSED)),
            leftAlone(volts(0), remembering(LAST_ABOVE))));
  }

  /** Run at an output inside a closed region, the joint is held where it can stop, whichever way. */
  static Stream<Arguments> anOutputAskedFromInside() {
    return Stream.of(
        decides("overshot into the region at 2 m/s, still asked to run on: held where it stops",
            () -> MIDDLE.decide(remembering(HELD_AT_LOWER_EDGE), speed(2), moving(5, 2, CLOSED)),
            heldFromNow(holdAt(6), remembering(heldInside(6, LAST_BELOW)), OUT_OF_THE_MIDDLE)),
        decides("a loop later: still held at the same point",
            () -> MIDDLE.decide(remembering(heldInside(6, LAST_BELOW)), speed(2), moving(5.5, 1, CLOSED)),
            held(holdAt(6), remembering(heldInside(6, LAST_BELOW)), OUT_OF_THE_MIDDLE)),
        decides("inside, asked to run back out the way it came: held too, for only a position may leave",
            () -> MIDDLE.decide(remembering(LAST_BELOW), speed(-2), at(5, CLOSED)),
            heldFromNow(holdAt(5), remembering(heldInside(5, LAST_BELOW)), OUT_OF_THE_MIDDLE)));
  }

  /**
   * A region that reaches an end of the joint's range has no room past that end. A joint resting
   * on the end is inside the region, and may leave only by the region's other edge.
   */
  static Stream<Arguments> aRegionReachingTheEndOfTheRange() {
    return Stream.of(
        decides("resting on the low end, asked to stay: it stays, its profile restarted once",
            () -> BOTTOM.decide(remembering(NEVER_OUTSIDE), goTo(0), at(0, CLOSED)),
            new InterlockVerdict(remembering(heldInside(0, NEVER_OUTSIDE)), goTo(0), List.of(), true)),
        decides("a loop later, still asked to stay: nothing more to do",
            () -> BOTTOM.decide(remembering(heldInside(0, NEVER_OUTSIDE)), goTo(0), at(0, CLOSED)),
            leftAlone(goTo(0), remembering(heldInside(0, NEVER_OUTSIDE)))),
        decides("resting on the low end, a goal above the region: it may leave by the upper edge",
            () -> BOTTOM.decide(remembering(NEVER_OUTSIDE), goTo(10), at(0, CLOSED)),
            leftAlone(goTo(10), remembering(NEVER_OUTSIDE))),
        decides("resting on the low end, a goal deeper in the region: held where it is",
            () -> BOTTOM.decide(remembering(NEVER_OUTSIDE), goTo(2), at(0, CLOSED)),
            heldFromNow(holdAt(0), remembering(heldInside(0, NEVER_OUTSIDE)), OFF_THE_BOTTOM)),
        decides("above that region, a goal on the low end: stopped at the upper edge",
            () -> BOTTOM.decide(remembering(NEVER_OUTSIDE), goTo(0), at(10, CLOSED)),
            held(holdAt(4), remembering(LAST_ABOVE), OFF_THE_BOTTOM)),
        decides("resting on the high end, a goal below the region: it may leave by the lower edge",
            () -> TOP.decide(remembering(NEVER_OUTSIDE), goTo(2), at(16, CLOSED)),
            leftAlone(goTo(2), remembering(NEVER_OUTSIDE))),
        decides("resting on the high end, a goal deeper in the region: held where it is",
            () -> TOP.decide(remembering(NEVER_OUTSIDE), goTo(14), at(16, CLOSED)),
            heldFromNow(holdAt(16), remembering(heldInside(16, NEVER_OUTSIDE)), OFF_THE_TOP)),
        // As the code stands, and reported as a likely fault: a reading a little past the stop is
        // not "inside", so the joint is not held but driven through the region to its far edge.
        decides("reading a little past the low end, asked to stay: sent to the upper edge",
            () -> BOTTOM.decide(remembering(NEVER_OUTSIDE), goTo(0), at(-0.25, CLOSED)),
            held(holdAt(4), remembering(LAST_ABOVE), OFF_THE_BOTTOM)));
  }

  /**
   * A joint that turns freely takes a goal the short way round, and its goal is judged where that
   * takes it. The wheel's keep-out is 1 rad to 2 rad.
   */
  static Stream<Arguments> aJointThatTurnsFreely() {
    double behindTheRegionTheLongWay = 3 - 2 * Math.PI;
    return Stream.of(
        decides("a goal that is beyond the region the short way round: stopped at the edge",
            () -> WHEEL_BAND.decide(remembering(NEVER_OUTSIDE), goTo(behindTheRegionTheLongWay), at(0.5, CLOSED)),
            held(holdAt(1), remembering(LAST_BELOW), WHEEL_OUT_OF_A_BAND)),
        decides("a goal away from the region: left alone",
            () -> WHEEL_BAND.decide(remembering(NEVER_OUTSIDE), goTo(-1), at(0.5, CLOSED)),
            leftAlone(goTo(-1), remembering(LAST_BELOW))));
  }

  /**
   * A run-only-when stops its joint while its condition does not hold: a joint that holds positions
   * at the point where it can stop, worked out once, and a joint that does not, in neutral.
   */
  static Stream<Arguments> runsOnlyWhen() {
    return Stream.of(
        decides("the condition holds: left alone",
            () -> RAIL_RUNS_ONLY.decide(RUNNING, goTo(10), moving(5, 2, MAY_RUN)),
            leftAlone(goTo(10), RUNNING)),
        decides("the condition stops holding at 2 m/s: held where it stops, 1 m on",
            () -> RAIL_RUNS_ONLY.decide(RUNNING, goTo(10), moving(5, 2, MAY_NOT_RUN)),
            heldFromNow(holdAt(6), stoppedAt(6), RAIL_RUNS_ONLY_WHEN_LOW)),
        decides("the condition's sensor lost: held, as if it did not hold",
            () -> RAIL_RUNS_ONLY.decide(RUNNING, goTo(10), moving(5, 2, LOST)),
            heldFromNow(holdAt(6), stoppedAt(6), RAIL_RUNS_ONLY_WHEN_LOW)),
        decides("a loop later, slower: still held at the same point",
            () -> RAIL_RUNS_ONLY.decide(stoppedAt(6), goTo(10), moving(5.5, 1, MAY_NOT_RUN)),
            held(holdAt(6), stoppedAt(6), RAIL_RUNS_ONLY_WHEN_LOW)),
        decides("the condition holds again: let go, its profile restarted toward its goal",
            () -> RAIL_RUNS_ONLY.decide(stoppedAt(6), goTo(10), at(6, MAY_RUN)),
            new InterlockVerdict(RUNNING, goTo(10), List.of(), true)),
        decides("run at an output, a joint that holds positions is held at a position all the same",
            () -> RAIL_RUNS_ONLY.decide(RUNNING, volts(3), moving(5, 2, MAY_NOT_RUN)),
            heldFromNow(holdAt(6), stoppedAt(6), RAIL_RUNS_ONLY_WHEN_LOW)),
        decides("a joint that holds no positions: neutral, with nothing to remember or restart",
            () -> WHEEL_RUNS_ONLY.decide(RUNNING, speed(50), moving(5, 50, MAY_NOT_RUN)),
            held(NEUTRAL, RUNNING, WHEEL_RUNS_ONLY_WHEN_LOW)),
        decides("that joint, already in neutral: still reported as held",
            () -> WHEEL_RUNS_ONLY.decide(RUNNING, NEUTRAL, at(5, MAY_NOT_RUN)),
            held(NEUTRAL, RUNNING, WHEEL_RUNS_ONLY_WHEN_LOW)),
        decides("that joint, its condition holding: left alone",
            () -> WHEEL_RUNS_ONLY.decide(RUNNING, speed(50), moving(5, 50, MAY_RUN)),
            leftAlone(speed(50), RUNNING)));
  }

  /**
   * Several interlocks are applied in the order they are listed. Each sees what the ones before it
   * left of what was asked, and every one that holds the joint is named.
   */
  static Stream<Arguments> severalInterlocks() {
    InterlockState twoRegions = new InterlockState(OptionalDouble.empty(), List.of(LAST_ABOVE, LAST_BELOW));
    return Stream.of(
        decides("two run-only-whens, neither holding: held by both, at one point",
            () -> TWO_RUN_ONLYS.decide(RUNNING, goTo(10), moving(5, 2, MAY_NOT_RUN, MAY_NOT_RUN)),
            heldFromNow(holdAt(6), stoppedAt(6), RAIL_RUNS_ONLY_WHEN_LOW, RAIL_RUNS_ONLY_WHEN_HIGH)),
        decides("two run-only-whens, only the second not holding: held by the second",
            () -> TWO_RUN_ONLYS.decide(RUNNING, goTo(10), moving(5, 2, MAY_RUN, MAY_NOT_RUN)),
            heldFromNow(holdAt(6), stoppedAt(6), RAIL_RUNS_ONLY_WHEN_HIGH)),
        decides("two run-only-whens, one still not holding: not let go",
            () -> TWO_RUN_ONLYS.decide(stoppedAt(6), goTo(10), at(6, MAY_NOT_RUN, MAY_RUN)),
            held(holdAt(6), stoppedAt(6), RAIL_RUNS_ONLY_WHEN_LOW)),
        decides("a keep-out, then a run-only-when: the run-only-when's stop has the last word",
            () -> KEEP_OUT_THEN_RUNS_ONLY.decide(remembering(LAST_BELOW), goTo(10), moving(2, 2, CLOSED, MAY_NOT_RUN)),
            heldFromNow(holdAt(3), new InterlockState(OptionalDouble.of(3), List.of(LAST_BELOW)), OUT_OF_THE_MIDDLE, RAIL_RUNS_ONLY_WHEN_LOW)),
        decides("a run-only-when, then a keep-out: a stop inside the region is moved to its edge",
            () -> RUNS_ONLY_THEN_KEEP_OUT.decide(remembering(LAST_BELOW), goTo(10), moving(3.5, 2, MAY_NOT_RUN, CLOSED)),
            heldFromNow(holdAt(4), new InterlockState(OptionalDouble.of(4.5), List.of(LAST_BELOW)), RAIL_RUNS_ONLY_WHEN_LOW, OUT_OF_THE_MIDDLE)),
        decides("two keep-outs with a run-only-when between: each remembers its own side",
            () -> MIDDLE_AND_TOP.decide(MIDDLE_AND_TOP.initialState(), goTo(14), at(10, CLOSED, MAY_RUN, CLOSED)),
            held(holdAt(12), twoRegions, OFF_THE_TOP)),
        decides("between two closed regions, a goal below both: stopped by the nearer one",
            () -> MIDDLE_AND_TOP.decide(twoRegions, goTo(2), at(10, CLOSED, MAY_RUN, CLOSED)),
            held(holdAt(8), twoRegions, OUT_OF_THE_MIDDLE)));
  }
  // spotless:on

  /** As the robot starts, nothing is held, and each keep-out, and only they, has a memory. */
  @Test
  void itStartsRememberingNothing() {
    assertThat(MIDDLE_AND_TOP.initialState())
        .isEqualTo(
            new InterlockState(
                OptionalDouble.empty(), List.of(KeepOutState.INITIAL, KeepOutState.INITIAL)));
    assertThat(RAIL_RUNS_ONLY.initialState()).isEqualTo(RUNNING);
  }

  /** A joint left alone gets back the very ask it was given, not a copy. */
  @Test
  void aJointLeftAloneKeepsTheAskItWasGiven() {
    JointAsk asked = goTo(3);

    InterlockVerdict verdict = MIDDLE.decide(remembering(LAST_BELOW), asked, at(2, CLOSED));

    assertThat(verdict.allowed()).isSameAs(asked);
  }

  /** Each interlock needs its condition's reading, and one more or fewer is a mistake. */
  @Test
  void itRefusesReadingsThatDoNotMatchItsInterlocks() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> TWO_RUN_ONLYS.decide(RUNNING, goTo(10), at(5, MAY_RUN)))
        .withMessage(
            "rail has 2 interlocks, and was given 1 condition readings: one for each, in the same"
                + " order");
  }

  /** What each output is, and the speed it would run an unloaded joint at. */
  @Test
  void anOutputSaysWhichWayItPushesAndHowFastItWouldRun() {
    double freeSpeed = 8;

    assertThat(new JointAsk.AtVelocity(-2).value()).isEqualTo(-2);
    assertThat(new JointAsk.AtVelocity(-2).unloadedSpeed(freeSpeed)).isEqualTo(-2);
    assertThat(new JointAsk.AtVoltage(6).value()).isEqualTo(6);
    assertThat(new JointAsk.AtVoltage(6).unloadedSpeed(freeSpeed)).isEqualTo(4);
    assertThat(new JointAsk.AtDutyCycle(-0.5).value()).isEqualTo(-0.5);
    assertThat(new JointAsk.AtDutyCycle(-0.5).unloadedSpeed(freeSpeed)).isEqualTo(-4);
  }

  /** A condition that was read holds or does not; only one that could not be read is unknown. */
  @Test
  void aConditionThatWasReadHoldsOrDoesNot() {
    assertThat(ConditionReading.of(true)).isEqualTo(ConditionReading.HOLDS);
    assertThat(ConditionReading.of(false)).isEqualTo(ConditionReading.DOES_NOT_HOLD);
  }
}
