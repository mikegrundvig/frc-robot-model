package com.michaelgrundvig.frc.world.robot;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.OptionalDouble;

/**
 * Decides, one loop at a time, what the interlocks on one joint allow it to do.
 *
 * <p>It holds only facts that never change while the robot runs. Everything that does change comes
 * in as a value and goes out as a value: {@link #decide} takes what the interlocks remembered from
 * the last loop, what the joint is asked to do, and what was read this loop, and returns what the
 * joint may do and what the interlocks remember now. The same inputs always give the same answer,
 * so the robot, the simulator and a replay of a log all enforce the interlocks identically.
 *
 * <p>The interlocks are applied one after another, in the order they are listed, and each sees what
 * the ones before it left of what was asked. The order matters: when a keep-out moves a goal to its
 * region's edge, an interlock listed after it judges that edge, not the original goal.
 *
 * <p>The command that asked is not told and keeps its goal. Its ask is judged again every loop, so
 * the joint carries on by itself once the interlocks allow it.
 *
 * @param travel where the joint can be driven and how quickly it stops, which also names the joint
 * @param interlocks the interlocks that restrict the joint, in the order they are applied
 * @param loopSeconds how long one loop lasts, in seconds
 */
public record InterlockEnforcement(Travel travel, List<Interlock> interlocks, double loopSeconds) {
  public InterlockEnforcement {
    interlocks = List.copyOf(interlocks);
  }

  /**
   * What the interlocks remember as the robot starts: no joint held, none seen outside a region.
   */
  public InterlockState initialState() {
    List<KeepOutState> keepOuts = new ArrayList<>();
    for (Interlock interlock : interlocks) {
      if (interlock instanceof Interlock.KeepsOut) {
        keepOuts.add(KeepOutState.INITIAL);
      }
    }
    return new InterlockState(OptionalDouble.empty(), keepOuts);
  }

  /**
   * What the joint may do this loop, and what its interlocks remember afterwards.
   *
   * @param previous what the interlocks remembered from the last loop
   * @param asked what the joint is asked to do this loop: a command's request, or what the joint
   *     does by itself when no command asks anything
   * @param readings how the joint is moving, and what each interlock's condition reads
   * @throws IllegalArgumentException if there is not exactly one condition reading for each
   *     interlock
   */
  public InterlockVerdict decide(
      InterlockState previous, JointAsk asked, InterlockReadings readings) {
    requireAReadingForEachInterlock(readings);

    // The run-only-whens are settled first, all together, because they share one hold: whichever
    // of them stops the joint, it is stopped at the same point.
    OptionalDouble runsOnlyWhenHeldAt = runsOnlyWhenHold(previous, readings);
    JointAsk whileNotAllowedToRun = stopped(runsOnlyWhenHeldAt);

    JointAsk allowed = asked;
    List<Interlock> heldBy = new ArrayList<>();
    List<KeepOutState> keepOuts = new ArrayList<>();
    boolean holdInsideARegionStarted = false;
    Iterator<KeepOutState> previousKeepOuts = previous.keepOuts().iterator();
    for (int index = 0; index < interlocks.size(); index++) {
      ConditionReading condition = readings.conditions().get(index);
      switch (interlocks.get(index)) {
        case Interlock.RunsOnlyWhen runsOnlyWhen -> {
          if (!mayRun(condition)) {
            allowed = whileNotAllowedToRun;
            heldBy.add(runsOnlyWhen);
          }
        }
        case Interlock.KeepsOut keepOut -> {
          KeepOutDecision.Ruling ruling =
              KeepOutDecision.decide(
                  keepOut,
                  travel,
                  loopSeconds,
                  previousKeepOuts.next(),
                  isClosed(condition),
                  allowed,
                  readings);
          keepOuts.add(ruling.state());
          allowed = ruling.allowed();
          holdInsideARegionStarted |= ruling.startedHoldingInside();
          if (ruling.holds()) {
            heldBy.add(keepOut);
          }
        }
      }
    }

    boolean runsOnlyWhenHoldStarted =
        runsOnlyWhenHeldAt.isPresent() && previous.runsOnlyWhenHeldAt().isEmpty();
    boolean runsOnlyWhenLetGo =
        runsOnlyWhenHeldAt.isEmpty() && previous.runsOnlyWhenHeldAt().isPresent();
    boolean restartProfile =
        runsOnlyWhenHoldStarted || runsOnlyWhenLetGo || holdInsideARegionStarted;
    return new InterlockVerdict(
        new InterlockState(runsOnlyWhenHeldAt, keepOuts), allowed, heldBy, restartProfile);
  }

  /** Refuses readings that do not have exactly one condition reading for each interlock. */
  private void requireAReadingForEachInterlock(InterlockReadings readings) {
    if (readings.conditions().size() == interlocks.size()) {
      return;
    }
    throw new IllegalArgumentException(
        travel.joint().label()
            + " has "
            + interlocks.size()
            + " interlocks, and was given "
            + readings.conditions().size()
            + " condition readings: one for each, in the same order");
  }

  /**
   * Whether a run-only-when lets its joint move. Only a condition that was read and holds does: one
   * that could not be read holds the joint, to be safe.
   */
  private static boolean mayRun(ConditionReading condition) {
    return condition == ConditionReading.HOLDS;
  }

  /**
   * Whether a keep-out's region is closed. Only a condition that was read and does not hold opens
   * it: one that could not be read keeps the joint out, to be safe.
   */
  private static boolean isClosed(ConditionReading condition) {
    return condition != ConditionReading.DOES_NOT_HOLD;
  }

  /**
   * Where the joint is held this loop because a run-only-when does not let it move, or empty when
   * none holds it. A hold that started in an earlier loop keeps its point. A joint that does not
   * hold positions is never held at one: it is put in neutral instead ({@link #stopped}).
   */
  private OptionalDouble runsOnlyWhenHold(InterlockState previous, InterlockReadings readings) {
    if (!travel.joint().holdsPositions() || !aRunsOnlyWhenRestricts(readings)) {
      return OptionalDouble.empty();
    }
    if (previous.runsOnlyWhenHeldAt().isPresent()) {
      return previous.runsOnlyWhenHeldAt();
    }
    return OptionalDouble.of(travel.stoppingPoint(readings.position(), readings.velocity()));
  }

  /** Whether any run-only-when's condition does not let the joint move this loop. */
  private boolean aRunsOnlyWhenRestricts(InterlockReadings readings) {
    for (int index = 0; index < interlocks.size(); index++) {
      boolean isRunsOnlyWhen = interlocks.get(index) instanceof Interlock.RunsOnlyWhen;
      if (isRunsOnlyWhen && !mayRun(readings.conditions().get(index))) {
        return true;
      }
    }
    return false;
  }

  /**
   * What a joint does while a run-only-when does not let it move: it holds the point it was stopped
   * at, or, for a joint that does not hold positions and so has no such point, it goes to neutral.
   */
  private static JointAsk stopped(OptionalDouble heldAt) {
    if (heldAt.isPresent()) {
      return new JointAsk.ToPosition(heldAt.getAsDouble(), JointAsk.FULL_CRUISE);
    }
    return JointAsk.NEUTRAL;
  }
}
