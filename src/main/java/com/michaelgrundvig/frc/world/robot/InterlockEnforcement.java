package com.michaelgrundvig.frc.world.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;

/**
 * Decides, one loop at a time, what the interlocks on one joint allow it to do.
 *
 * <p>It holds only facts that never change while the robot runs. Everything that does change comes
 * in as a value and goes out as a value: {@link #decide} takes what the interlocks decided in the
 * last loop, what the joint is asked to do, and what was read this loop, and returns what the joint
 * may do and what the interlocks remember now. The same inputs always give the same answer, so the
 * robot, the simulator and a replay of a log all enforce the interlocks identically.
 *
 * <p>The interlocks are applied one after another, in the order they are listed, and each sees what
 * the ones before it left of what was asked. The order matters: when a keep-out moves a goal to its
 * region's edge, an interlock listed after it judges that edge, not the original goal.
 *
 * <p>The command that asked is not told and keeps its goal. Its ask is judged again every loop, so
 * the joint carries on by itself once the interlocks allow it.
 *
 * <p>A joint's interlocks are decided every loop, and in nearly every loop the answer is the same
 * as in the loop before. Such a loop makes no new objects. The values that come back never change
 * once made, so when the answer is the same, the same objects are handed back: the last loop's
 * verdict itself, or, in a loop where only part of the answer changed, the parts of it that did
 * not. Only what really changed is made new.
 */
public final class InterlockEnforcement {
  private final Travel travel;
  private final List<Interlock> interlocks;

  /**
   * What decides each keep-out among the interlocks: one for each, in the order they are listed.
   */
  private final List<KeepOutEnforcement> keepOuts;

  /**
   * @param travel where the joint can be driven and how quickly it stops, which also names the
   *     joint
   * @param interlocks the interlocks that restrict the joint, in the order they are applied
   * @param loopSeconds how long one loop lasts, in seconds
   */
  public InterlockEnforcement(Travel travel, List<Interlock> interlocks, double loopSeconds) {
    this.travel = travel;
    this.interlocks = List.copyOf(interlocks);
    List<KeepOutEnforcement> keepOuts = new ArrayList<>();
    for (Interlock interlock : interlocks) {
      if (interlock instanceof Interlock.KeepsOut keepOut) {
        keepOuts.add(new KeepOutEnforcement(keepOut, travel, loopSeconds));
      }
    }
    this.keepOuts = List.copyOf(keepOuts);
  }

  /**
   * What the interlocks remember as the robot starts: no joint held, none seen outside a region.
   */
  public InterlockState initialState() {
    List<KeepOutState> states = new ArrayList<>();
    for (int each = 0; each < keepOuts.size(); each++) {
      states.add(KeepOutState.INITIAL);
    }
    return new InterlockState(OptionalDouble.empty(), states);
  }

  /**
   * What the joint may do this loop, and what its interlocks remember afterwards, in the plain
   * form: every value is made new for the call. It suits a test, a tool, or a replay of a log. The
   * robot's own loop uses the other form, which gives the same answer and makes nothing new while
   * nothing changes.
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
    return decide(
        InterlockVerdict.startingFrom(previous),
        asked,
        readings,
        readings.conditions(),
        new InterlockWorkspace());
  }

  /**
   * What the joint may do this loop, and what its interlocks remember afterwards.
   *
   * <p>When that is exactly what the last loop's verdict says, the last loop's verdict itself is
   * handed back, and nothing new was made in finding that out.
   *
   * @param last the verdict of the last loop, which carries what the interlocks remembered, or
   *     {@link InterlockVerdict#startingFrom} for the first loop
   * @param asked what the joint is asked to do this loop: a command's request, or what the joint
   *     does by itself when no command asks anything
   * @param motion how the joint is moving. It is read during the call and not kept.
   * @param conditions what each interlock's condition reads this loop: one for each interlock, in
   *     the order they are listed. The list is the caller's: it is read during the call, and
   *     neither copied nor kept, so the caller may fill the same list again in the next loop.
   * @param workspace scratch for this decision, owned by the caller and handed in every loop
   * @throws IllegalArgumentException if there is not exactly one condition reading for each
   *     interlock
   */
  public InterlockVerdict decide(
      InterlockVerdict last,
      JointAsk asked,
      JointMotion motion,
      List<ConditionReading> conditions,
      InterlockWorkspace workspace) {
    requireAReadingForEachInterlock(conditions);
    InterlockState previous = last.state();

    // The run-only-whens are settled first, all together, because they share one hold: whichever
    // of them stops the joint, it is stopped at the same point.
    OptionalDouble runsOnlyWhenHeldAt = runsOnlyWhenHold(previous, motion, conditions);
    JointAsk whileNotAllowedToRun = stopped(runsOnlyWhenHeldAt, last.allowed());

    workspace.empty();
    JointAsk allowed = asked;
    List<Interlock> heldBy = workspace.heldBy;
    List<KeepOutState> keepOutsNow = workspace.keepOuts;
    boolean holdInsideARegionStarted = false;
    for (int index = 0; index < interlocks.size(); index++) {
      ConditionReading condition = conditions.get(index);
      switch (interlocks.get(index)) {
        case Interlock.RunsOnlyWhen runsOnlyWhen -> {
          if (!mayRun(condition)) {
            allowed = whileNotAllowedToRun;
            heldBy.add(runsOnlyWhen);
          }
        }
        case Interlock.KeepsOut keepOut -> {
          // The keep-outs are met in the order they are listed, so the next one to decide is the
          // one after those that already have.
          KeepOutEnforcement enforcement = keepOuts.get(keepOutsNow.size());
          KeepOutState before = previous.keepOuts().get(keepOutsNow.size());
          KeepOutEnforcement.Ruling ruling =
              enforcement.ruling(before, isClosed(condition), allowed, motion);
          KeepOutState now = enforcement.remembers(ruling, before, motion);
          keepOutsNow.add(now);
          holdInsideARegionStarted |= now.startedHoldingInside(before);
          if (enforcement.holds(ruling, now, allowed)) {
            allowed = hold(enforcement.heldAt(now), last.allowed());
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
    InterlockState state = previous.remembering(runsOnlyWhenHeldAt, keepOutsNow);
    return last.deciding(state, allowed, heldBy, restartProfile);
  }

  /** Refuses readings that do not have exactly one condition reading for each interlock. */
  private void requireAReadingForEachInterlock(List<ConditionReading> conditions) {
    if (conditions.size() == interlocks.size()) {
      return;
    }
    throw new IllegalArgumentException(
        travel.joint().label()
            + " has "
            + interlocks.size()
            + " interlocks, and was given "
            + conditions.size()
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
  private OptionalDouble runsOnlyWhenHold(
      InterlockState previous, JointMotion motion, List<ConditionReading> conditions) {
    if (!travel.joint().holdsPositions() || !aRunsOnlyWhenRestricts(conditions)) {
      return OptionalDouble.empty();
    }
    if (previous.runsOnlyWhenHeldAt().isPresent()) {
      return previous.runsOnlyWhenHeldAt();
    }
    return OptionalDouble.of(travel.stoppingPoint(motion.position(), motion.velocity()));
  }

  /** Whether any run-only-when's condition does not let the joint move this loop. */
  private boolean aRunsOnlyWhenRestricts(List<ConditionReading> conditions) {
    for (int index = 0; index < interlocks.size(); index++) {
      boolean isRunsOnlyWhen = interlocks.get(index) instanceof Interlock.RunsOnlyWhen;
      if (isRunsOnlyWhen && !mayRun(conditions.get(index))) {
        return true;
      }
    }
    return false;
  }

  /**
   * What a joint does while a run-only-when does not let it move: it holds the point it was stopped
   * at, or, for a joint that does not hold positions and so has no such point, it goes to neutral.
   *
   * @param lastAllowed what the joint was allowed in the last loop ({@link #hold} says why)
   */
  private static JointAsk stopped(OptionalDouble heldAt, JointAsk lastAllowed) {
    if (heldAt.isPresent()) {
      return hold(heldAt.getAsDouble(), lastAllowed);
    }
    return JointAsk.NEUTRAL;
  }

  /**
   * A position an interlock holds the joint at, moving there at its full cruise.
   *
   * <p>A hold lasts many loops, and in each of them it is the same position. So when the joint was
   * allowed exactly this hold in the last loop, that same object is handed back, and a hold makes
   * one object as it starts and not one every loop.
   *
   * @param lastAllowed what the joint was allowed in the last loop
   */
  private static JointAsk hold(double position, JointAsk lastAllowed) {
    boolean isTheSameHold =
        lastAllowed instanceof JointAsk.ToPosition held
            && Double.compare(held.position(), position) == 0
            && held.cruiseShare() == JointAsk.FULL_CRUISE;
    if (isTheSameHold) {
      return lastAllowed;
    }
    return new JointAsk.ToPosition(position, JointAsk.FULL_CRUISE);
  }
}
