package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.OFF_THE_TOP;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.OUT_OF_THE_MIDDLE;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.RAIL_RUNS_ONLY_WHEN_HIGH;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.RAIL_RUNS_ONLY_WHEN_LOW;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.WHEEL_RUNS_ONLY_WHEN_LOW;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.onTheRail;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.onTheWheel;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import com.michaelgrundvig.frc.world.robot.Interlock.KeepsOut.Side;
import java.util.ArrayList;
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
 * What a joint's interlocks make new from one loop to the next, which is as little as they can:
 * {@link InterlockEnforcement#decide} runs for every joint with an interlock every loop, and a loop
 * in which nothing changed must make no new objects.
 *
 * <p>That shows from outside as which objects come back. A loop whose answer is the last loop's
 * gets the last loop's verdict itself. A loop that changes part of the answer gets a new verdict
 * whose unchanged parts are the last loop's own. What the answers are is {@link
 * InterlockEnforcementTest}'s job, so here every loop is only checked against the plain form of
 * {@code decide}, which that test pins down.
 *
 * <p>The rail's keep-out is 4 m to 8 m, and the rail stands still unless a row says otherwise.
 */
class InterlockReuseTest {
  private static final InterlockEnforcement UNRESTRICTED = onTheRail();
  private static final InterlockEnforcement MIDDLE = onTheRail(OUT_OF_THE_MIDDLE);
  private static final InterlockEnforcement RAIL_RUNS_ONLY = onTheRail(RAIL_RUNS_ONLY_WHEN_LOW);
  private static final InterlockEnforcement WHEEL_RUNS_ONLY = onTheWheel(WHEEL_RUNS_ONLY_WHEN_LOW);
  private static final InterlockEnforcement TWO_RUN_ONLYS =
      onTheRail(RAIL_RUNS_ONLY_WHEN_LOW, RAIL_RUNS_ONLY_WHEN_HIGH);
  private static final InterlockEnforcement KEEP_OUT_THEN_RUNS_ONLY =
      onTheRail(OUT_OF_THE_MIDDLE, RAIL_RUNS_ONLY_WHEN_LOW);
  private static final InterlockEnforcement MIDDLE_AND_TOP =
      onTheRail(OUT_OF_THE_MIDDLE, OFF_THE_TOP);

  // What a run-only-when's condition reads, and a keep-out's: a keep-out is closed while its holds.
  private static final ConditionReading MAY_RUN = ConditionReading.HOLDS;
  private static final ConditionReading MAY_NOT_RUN = ConditionReading.DOES_NOT_HOLD;
  private static final ConditionReading CLOSED = ConditionReading.HOLDS;
  private static final ConditionReading OPEN = ConditionReading.DOES_NOT_HOLD;

  /** Asked for a position at half the joint's cruise, as a command might. */
  private static JointAsk goTo(double position) {
    return new JointAsk.ToPosition(position, 0.5);
  }

  /** A position an interlock holds the joint at: always at its full cruise. */
  private static JointAsk holdAt(double position) {
    return new JointAsk.ToPosition(position, JointAsk.FULL_CRUISE);
  }

  private static JointAsk speed(double metersPerSecond) {
    return new JointAsk.AtVelocity(metersPerSecond);
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

  /**
   * One joint's interlocks run loop after loop as the robot runs them: each loop is handed the
   * verdict of the one before, and the same workspace.
   */
  private static final class Loops {
    private final InterlockEnforcement enforcement;
    private final InterlockWorkspace workspace = new InterlockWorkspace();
    private InterlockVerdict last;

    Loops(InterlockEnforcement enforcement) {
      this.enforcement = enforcement;
      last = InterlockVerdict.startingFrom(enforcement.initialState());
    }

    /**
     * Runs one loop, and checks that its verdict is the one the plain form of {@code decide} gives
     * from the same memory. The readings are handed over as the robot hands them: in a list of the
     * caller's own, not the readings' unchangeable one.
     */
    InterlockVerdict loop(JointAsk asked, InterlockReadings readings) {
      InterlockVerdict plain = enforcement.decide(last.state(), asked, readings);
      List<ConditionReading> callersOwn = new ArrayList<>(readings.conditions());
      InterlockVerdict verdict = enforcement.decide(last, asked, readings, callersOwn, workspace);
      assertThat(verdict).isEqualTo(plain);
      last = verdict;
      return verdict;
    }
  }

  @ParameterizedTest
  @MethodSource("steadyLoops")
  void aLoopInWhichNothingChangedGetsTheLastLoopsVerdictItself(
      InterlockEnforcement enforcement,
      Supplier<JointAsk> asked,
      Supplier<InterlockReadings> readings) {
    Loops loops = new Loops(enforcement);
    // A hold restarts the joint's profile in the loop it starts, so the second loop is the first
    // that can be like the one after it. Each loop is asked with equal values newly made, because
    // the same answer has to be found by what the values say, not by which objects they are.
    loops.loop(asked.get(), readings.get());
    InterlockVerdict second = loops.loop(asked.get(), readings.get());

    InterlockVerdict third = loops.loop(asked.get(), readings.get());

    assertThat(third).isSameAs(second);
  }

  private static Arguments steady(
      String label,
      InterlockEnforcement enforcement,
      Supplier<JointAsk> asked,
      Supplier<InterlockReadings> readings) {
    return argumentSet(label, enforcement, asked, readings);
  }

  // spotless:off
  static Stream<Arguments> steadyLoops() {
    return Stream.of(
        steady("no interlocks",
            UNRESTRICTED, () -> goTo(10), () -> at(2)),
        steady("left alone beside an open region",
            MIDDLE, () -> speed(2), () -> moving(3, 2, OPEN)),
        steady("stopped at the edge on the way to a position beyond it",
            MIDDLE, () -> goTo(10), () -> at(2, CLOSED)),
        steady("run at an output, held at the edge",
            MIDDLE, () -> speed(2), () -> at(3, CLOSED)),
        steady("held inside the region where it stopped",
            MIDDLE, () -> goTo(7), () -> at(5, CLOSED)),
        steady("held by a run-only-when where it stopped",
            RAIL_RUNS_ONLY, () -> goTo(10), () -> at(2, MAY_NOT_RUN)),
        steady("a joint that holds no positions, put in neutral by a run-only-when",
            WHEEL_RUNS_ONLY, () -> speed(2), () -> at(2, MAY_NOT_RUN)),
        steady("held by a keep-out and a run-only-when at once",
            KEEP_OUT_THEN_RUNS_ONLY, () -> goTo(10), () -> at(2, CLOSED, MAY_NOT_RUN)),
        steady("two keep-outs, one of them holding",
            MIDDLE_AND_TOP, () -> goTo(10), () -> at(2, CLOSED, CLOSED)));
  }
  // spotless:on

  /** While only what is asked changes, the interlocks remember nothing new. */
  @Test
  void aNewAskLeavesWhatTheInterlocksRememberAsItWas() {
    Loops loops = new Loops(MIDDLE);
    InterlockVerdict before = loops.loop(goTo(1), at(2, CLOSED));

    InterlockVerdict after = loops.loop(goTo(3), at(2, CLOSED));

    assertThat(after).isNotSameAs(before);
    assertThat(after.state()).isSameAs(before.state());
  }

  /** A keep-out with nothing new to remember keeps its memory while another interlock's changes. */
  @Test
  void aKeepOutWithNothingNewToRememberKeepsItsMemoryWhileAnothersChanges() {
    Loops loops = new Loops(KEEP_OUT_THEN_RUNS_ONLY);
    InterlockVerdict running = loops.loop(goTo(3), at(2, CLOSED, MAY_RUN));

    InterlockVerdict stopped = loops.loop(goTo(3), at(2, CLOSED, MAY_NOT_RUN));

    assertThat(stopped.state()).isNotSameAs(running.state());
    assertThat(stopped.state().keepOuts().get(0)).isSameAs(running.state().keepOuts().get(0));
  }

  /** A hold that lasts is one object, even through a loop in which the verdict changes. */
  @Test
  void aHoldThatLastsIsOneObjectWhileTheRestOfTheVerdictChanges() {
    Loops loops = new Loops(TWO_RUN_ONLYS);
    loops.loop(goTo(10), at(2, MAY_NOT_RUN, MAY_RUN));
    InterlockVerdict heldByOne = loops.loop(goTo(10), at(2, MAY_NOT_RUN, MAY_RUN));

    InterlockVerdict heldByBoth = loops.loop(goTo(10), at(2, MAY_NOT_RUN, MAY_NOT_RUN));

    assertThat(heldByBoth.heldBy())
        .containsExactly(RAIL_RUNS_ONLY_WHEN_LOW, RAIL_RUNS_ONLY_WHEN_HIGH);
    assertThat(heldByBoth.state()).isSameAs(heldByOne.state());
    assertThat(heldByBoth.allowed()).isSameAs(heldByOne.allowed());
  }

  /** The same number of interlocks holding is not the same interlocks holding. */
  @Test
  void aDifferentInterlockHoldingIsANewVerdict() {
    Loops loops = new Loops(TWO_RUN_ONLYS);
    loops.loop(goTo(10), at(2, MAY_NOT_RUN, MAY_RUN));
    InterlockVerdict heldByTheFirst = loops.loop(goTo(10), at(2, MAY_NOT_RUN, MAY_RUN));

    InterlockVerdict heldByTheSecond = loops.loop(goTo(10), at(2, MAY_RUN, MAY_NOT_RUN));

    assertThat(heldByTheFirst.heldBy()).containsExactly(RAIL_RUNS_ONLY_WHEN_LOW);
    assertThat(heldByTheSecond.heldBy()).containsExactly(RAIL_RUNS_ONLY_WHEN_HIGH);
  }

  /**
   * A hold is at the joint's full cruise. A command's own ask for the same position, at its own
   * share of the cruise, is not that hold, though it was what the joint was allowed last loop.
   */
  @Test
  void aCommandsAskForTheSamePositionIsNotTakenForTheHold() {
    Loops loops = new Loops(MIDDLE);
    InterlockVerdict leftAlone = loops.loop(goTo(4), at(2, CLOSED));

    InterlockVerdict held = loops.loop(goTo(10), at(2, CLOSED));

    assertThat(leftAlone.allowed()).isEqualTo(goTo(4));
    assertThat(held.allowed()).isEqualTo(holdAt(4));
  }

  /** A hold at a new position is a new hold, not the last loop's hold at another. */
  @Test
  void aHoldSomewhereElseIsANewHold() {
    Loops loops = new Loops(KEEP_OUT_THEN_RUNS_ONLY);
    InterlockVerdict heldWhereItStopped = loops.loop(goTo(3), at(2, CLOSED, MAY_NOT_RUN));

    InterlockVerdict heldAtTheEdge = loops.loop(goTo(10), at(2, CLOSED, MAY_RUN));

    assertThat(heldWhereItStopped.allowed()).isEqualTo(holdAt(2));
    assertThat(heldAtTheEdge.allowed()).isEqualTo(holdAt(4));
  }

  /** The loop after a hold starts no longer restarts the profile, though nothing else changed. */
  @Test
  void theLoopAfterAHoldStartsNoLongerRestartsTheProfile() {
    Loops loops = new Loops(RAIL_RUNS_ONLY);
    InterlockVerdict started = loops.loop(goTo(10), at(2, MAY_NOT_RUN));

    InterlockVerdict lasting = loops.loop(goTo(10), at(2, MAY_NOT_RUN));

    assertThat(started.restartProfile()).isTrue();
    assertThat(lasting.restartProfile()).isFalse();
    assertThat(lasting.state()).isSameAs(started.state());
    assertThat(lasting.allowed()).isSameAs(started.allowed());
  }

  /**
   * One workspace serves every loop, however the loops differ: each starts from an empty one, so
   * nothing a loop found is left over in the next.
   */
  @Test
  void oneWorkspaceServesLoopsThatDecideDifferently() {
    Loops loops = new Loops(onTheRail(OUT_OF_THE_MIDDLE, RAIL_RUNS_ONLY_WHEN_LOW));

    InterlockVerdict heldByBoth = loops.loop(goTo(10), at(2, CLOSED, MAY_NOT_RUN));
    InterlockVerdict heldByTheKeepOut = loops.loop(goTo(10), at(2, CLOSED, MAY_RUN));
    InterlockVerdict free = loops.loop(goTo(10), at(2, OPEN, MAY_RUN));
    InterlockVerdict above = loops.loop(goTo(10), at(9, OPEN, MAY_RUN));

    assertThat(heldByBoth.heldBy()).containsExactly(OUT_OF_THE_MIDDLE, RAIL_RUNS_ONLY_WHEN_LOW);
    assertThat(heldByTheKeepOut.heldBy()).containsExactly(OUT_OF_THE_MIDDLE);
    assertThat(free.heldBy()).isEmpty();
    assertThat(free.state().keepOuts()).hasSize(1);
    assertThat(above.state().keepOuts()).hasSize(1);
    assertThat(above.state()).isNotEqualTo(free.state());
  }

  /** A verdict to start from carries the memory it was given, and holds nothing. */
  @Test
  void aVerdictToStartFromHoldsNothing() {
    InterlockState remembered = MIDDLE.initialState();

    InterlockVerdict start = InterlockVerdict.startingFrom(remembered);

    assertThat(start)
        .isEqualTo(new InterlockVerdict(remembered, JointAsk.NEUTRAL, List.of(), false));
  }

  /** Each named memory of a keep-out says the one thing it holds, and nothing else. */
  @Test
  void aKeepOutsNamedMemoriesSayWhatItHolds() {
    Optional<Side> below = Optional.of(Side.BELOW);

    assertThat(KeepOutState.notHolding(below))
        .isEqualTo(new KeepOutState(below, OptionalDouble.empty(), false));
    assertThat(KeepOutState.heldInside(below, 5))
        .isEqualTo(new KeepOutState(below, OptionalDouble.of(5), false));
    assertThat(KeepOutState.heldAtEdge(below))
        .isEqualTo(new KeepOutState(below, OptionalDouble.empty(), true));
  }
}
