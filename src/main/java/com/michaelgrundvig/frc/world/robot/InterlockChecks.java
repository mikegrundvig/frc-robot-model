package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

/**
 * The definition's checks of its interlocks against its joints and against each other.
 *
 * <p>Everything here is judged before the robot runs, so only a condition on a joint's position can
 * be judged at all: what a sensor reads and how fast a joint runs are not known yet. Positions are
 * judged with the robot placed some way ({@link Placement}): as it starts, or as a configuration
 * puts it.
 */
final class InterlockChecks {
  private InterlockChecks() {}

  /**
   * What is wrong with the interlocks. Each problem is about the joint the interlock restricts.
   *
   * <p>An interlock that names a joint, a sensor or a place that is not the robot's own is reported
   * first, and by itself: nothing else can be judged until every interlock is about this robot.
   * After that, a joint is refused if its conditions to run can never all hold, a named position is
   * refused if its joint can never reach it, and joints that hold each other in a deadlock as the
   * robot starts are refused ({@link Deadlock}).
   */
  static List<Finding> problems(RobotMath math, List<Interlock> interlocks) {
    List<Finding> notTheRobotsOwn = new ArrayList<>();
    for (Interlock interlock : interlocks) {
      notTheRobotsOwn.addAll(whatIsNotTheRobotsOwn(math, interlock));
    }
    if (!notTheRobotsOwn.isEmpty()) {
      return notTheRobotsOwn;
    }
    List<Finding> problems = new ArrayList<>();
    List<Interlock> onPositions = onPositions(interlocks);
    problems.addAll(jointsThatCanNeverRun(onPositions));
    problems.addAll(positionsNeverReached(math, onPositions));
    problems.addAll(deadlocksAsTheRobotStarts(math, interlocks));
    return problems;
  }

  /**
   * What is doubtful as the robot starts: a joint that rests inside a region it is kept out of. An
   * elevator resting on its bottom stop, with "keep out of the lowest 10 cm while the intake is in"
   * on it, is one. This is a doubt and not a refusal because the robot copes with it: it lets the
   * joint leave by the side it is kept to and do nothing else, until the condition clears. A
   * configuration that puts the joint there is still refused ({@link ConfigurationChecks}).
   */
  static List<Finding> startDoubts(RobotMath math, List<Interlock> interlocks) {
    List<Finding> doubts = new ArrayList<>();
    Placement start = Placement.asTheRobotStarts(math);
    for (Interlock interlock : checkable(math, interlocks)) {
      if (!(interlock instanceof Interlock.KeepsOut keepOut)) {
        continue;
      }
      Optional<String> broken = howKeepOutIsBroken(keepOut, start);
      if (broken.isEmpty()) {
        continue;
      }
      Joint joint = keepOut.joint();
      double edgeToLeaveBy = keepOut.edge(keepOut.sideKeptTo(start.positionOf(joint)));
      doubts.add(
          Finding.doubt(
              joint.label(),
              "as the robot starts, it "
                  + broken.get()
                  + ": "
                  + joint.label()
                  + " may only leave it, by its edge at "
                  + joint.shown(edgeToLeaveBy)
                  + ", until that changes"));
    }
    return doubts;
  }

  /**
   * The interlocks that can be judged against a placement: those on a joint's position, with every
   * joint and place they name the robot's own. The others are reported as problems with the
   * interlocks themselves ({@link #problems}).
   */
  static List<Interlock> checkable(RobotMath math, List<Interlock> interlocks) {
    return onPositions(interlocks).stream()
        .filter(interlock -> namesOnlyTheRobotsOwn(math, interlock))
        .toList();
  }

  /**
   * Whether every joint and place an interlock names is the robot's own, so that it can be judged.
   */
  static boolean namesOnlyTheRobotsOwn(RobotMath math, Interlock interlock) {
    if (!Places.movesOnItsOwn(math, interlock.joint())) {
      return false;
    }
    Optional<Joint> other = interlock.other();
    if (other.isPresent() && !Places.movesOnItsOwn(math, other.get())) {
      return false;
    }
    return Places.whyNotOwn(interlock.condition()).isEmpty();
  }

  /**
   * How a placement breaks an interlock on a joint's position, if it does: it runs a joint whose
   * condition to run does not hold, or it puts a joint inside a region that is closed.
   *
   * @return what the placement does, ready to follow what puts the robot there: "runs roller, but
   *     roller runs only when pivot at least 110°, and pivot is at 1.5°, where it rests"
   */
  static Optional<String> howBroken(Interlock interlock, Placement placement) {
    return switch (interlock) {
      case Interlock.RunsOnlyWhen runsOnlyWhen -> howRunsOnlyWhenIsBroken(runsOnlyWhen, placement);
      case Interlock.KeepsOut keepOut -> howKeepOutIsBroken(keepOut, placement);
    };
  }

  private static Optional<String> howRunsOnlyWhenIsBroken(
      Interlock.RunsOnlyWhen runsOnlyWhen, Placement placement) {
    Joint joint = runsOnlyWhen.joint();
    Joint other = onPosition(runsOnlyWhen).joint();
    boolean mayRun = onPosition(runsOnlyWhen).holdsAt(placement.positionOf(other));
    if (mayRun || !placement.runs(joint)) {
      return Optional.empty();
    }
    return Optional.of(
        "runs " + joint.label() + ", but " + runsOnlyWhen + ", and " + placement.where(other));
  }

  private static Optional<String> howKeepOutIsBroken(
      Interlock.KeepsOut keepOut, Placement placement) {
    Joint joint = keepOut.joint();
    Joint other = onPosition(keepOut).joint();
    double position = placement.positionOf(joint);
    boolean regionIsClosed = onPosition(keepOut).holdsAt(placement.positionOf(other));
    if (!regionIsClosed || !keepOut.isInside(position)) {
      return Optional.empty();
    }
    return Optional.of(
        "puts "
            + joint.label()
            + " at "
            + joint.shown(position)
            + ", but "
            + keepOut
            + ", and "
            + placement.where(other));
  }

  /** The findings that refuse an interlock for naming what is not the robot's own. */
  private static List<Finding> whatIsNotTheRobotsOwn(RobotMath math, Interlock interlock) {
    Joint restricted = interlock.joint();
    String named = "interlock \"" + interlock + "\"";
    List<Finding> found = new ArrayList<>();
    if (interlock.condition() instanceof Condition.OnSensor onSensor
        && !math.sensors().contains(onSensor.sensor())) {
      found.add(
          Places.refused(
              restricted,
              named
                  + "'s sensor, "
                  + onSensor.sensor().label()
                  + ", isn't one of the robot's sensors: list it"));
    }
    Places.whyNotMovingOnItsOwn(math, restricted, named)
        .ifPresent(why -> found.add(Places.refused(restricted, why)));
    interlock
        .other()
        .flatMap(other -> Places.whyNotMovingOnItsOwn(math, other, named))
        .ifPresent(why -> found.add(Places.refused(restricted, why)));
    Places.whyNotOwn(interlock.condition())
        .ifPresent(why -> found.add(Places.refused(restricted, named + ": " + why)));
    return found;
  }

  /** The interlocks on another joint's position, which are all that can be judged in advance. */
  private static List<Interlock> onPositions(List<Interlock> interlocks) {
    return interlocks.stream()
        .filter(interlock -> interlock.condition() instanceof Condition.OnJoint)
        .toList();
  }

  /** The condition of an interlock that is one of {@link #onPositions}. */
  private static Condition.OnJoint onPosition(Interlock interlock) {
    return (Condition.OnJoint) interlock.condition();
  }

  /**
   * The joints that can never run: a joint with run-only-whens on the same other joint whose
   * conditions hold together nowhere in that joint's range. It takes two or more: one condition
   * alone always holds somewhere, at its own edge, which it was refused for if that is outside the
   * range.
   */
  private static List<Finding> jointsThatCanNeverRun(List<Interlock> onPositions) {
    Map<Joint, Map<Joint, List<Condition.OnJoint>>> needed = new LinkedHashMap<>();
    for (Interlock interlock : onPositions) {
      if (interlock instanceof Interlock.RunsOnlyWhen runsOnlyWhen) {
        Condition.OnJoint condition = onPosition(runsOnlyWhen);
        needed
            .computeIfAbsent(runsOnlyWhen.joint(), joint -> new LinkedHashMap<>())
            .computeIfAbsent(condition.joint(), other -> new ArrayList<>())
            .add(condition);
      }
    }
    List<Finding> found = new ArrayList<>();
    needed.forEach(
        (joint, neededOfEachOther) ->
            neededOfEachOther.forEach(
                (other, conditions) -> {
                  if (!allHoldSomewhere(other, conditions)) {
                    found.add(Places.refused(joint, neverRuns(joint, other, conditions)));
                  }
                }));
    return found;
  }

  private static String neverRuns(Joint joint, Joint other, List<Condition.OnJoint> conditions) {
    List<String> each = conditions.stream().map(Condition.OnJoint::toString).toList();
    return joint.label()
        + " can never run: no position of "
        + other.label()
        + " has "
        + String.join(" and ", each);
  }

  /**
   * The named positions a joint can never reach: a position inside one or more of the joint's
   * keep-out regions, whose conditions on some other joint are all clear together nowhere in that
   * joint's range. At least one of the regions is then always closed.
   */
  private static List<Finding> positionsNeverReached(RobotMath math, List<Interlock> onPositions) {
    List<Finding> found = new ArrayList<>();
    for (Joint joint : math.joints()) {
      for (Position position : joint.positions()) {
        keepOutsAround(joint, position, onPositions)
            .forEach(
                (other, keepOuts) -> {
                  List<Condition.OnJoint> conditions =
                      keepOuts.stream().map(InterlockChecks::onPosition).toList();
                  if (!noneHoldsSomewhere(other, conditions)) {
                    found.add(
                        Places.refused(joint, neverReached(joint, position, other, keepOuts)));
                  }
                });
      }
    }
    return found;
  }

  /**
   * The keep-outs whose regions hold one of a joint's named positions, grouped by the other joint
   * each one's condition is about.
   */
  private static Map<Joint, List<Interlock.KeepsOut>> keepOutsAround(
      Joint joint, Position position, List<Interlock> onPositions) {
    Map<Joint, List<Interlock.KeepsOut>> around = new LinkedHashMap<>();
    for (Interlock interlock : onPositions) {
      if (interlock instanceof Interlock.KeepsOut keepOut
          && keepOut.joint().equals(joint)
          && keepOut.isInside(position.value())) {
        Joint other = onPosition(keepOut).joint();
        around.computeIfAbsent(other, each -> new ArrayList<>()).add(keepOut);
      }
    }
    return around;
  }

  private static String neverReached(
      Joint joint, Position position, Joint other, List<Interlock.KeepsOut> keepOuts) {
    List<String> each = keepOuts.stream().map(Interlock.KeepsOut::toString).toList();
    return joint.label()
        + " can never reach "
        + position.name()
        + ", "
        + joint.shown(position.value())
        + ": no position of "
        + other.label()
        + " opens it ("
        + String.join("; ", each)
        + ")";
  }

  /** Whether a joint has a position in its range where every one of some conditions holds. */
  private static boolean allHoldSomewhere(Joint joint, List<Condition.OnJoint> conditions) {
    for (double position : TrialPositions.acrossRange(joint, conditions)) {
      if (conditions.stream().allMatch(condition -> condition.holdsAt(position))) {
        return true;
      }
    }
    return false;
  }

  /** Whether a joint has a position in its range where none of some conditions holds. */
  private static boolean noneHoldsSomewhere(Joint joint, List<Condition.OnJoint> conditions) {
    for (double position : TrialPositions.acrossRange(joint, conditions)) {
      if (conditions.stream().noneMatch(condition -> condition.holdsAt(position))) {
        return true;
      }
    }
    return false;
  }

  /**
   * The joints that hold each other in a deadlock as the robot starts. A joint that merely rests
   * inside a region it is kept out of is not refused, because the robot lets it leave ({@link
   * #startDoubts}). Joints that hold each other there can never leave, so they are refused.
   */
  private static List<Finding> deadlocksAsTheRobotStarts(
      RobotMath math, List<Interlock> interlocks) {
    List<Finding> found = new ArrayList<>();
    Placement start = Placement.asTheRobotStarts(math);
    Set<Set<Joint>> reported = new HashSet<>();
    for (InterlockRing ring : InterlockRing.among(interlocks)) {
      Optional<String> deadlock = Deadlock.of(ring, start);
      if (deadlock.isPresent() && reported.add(ring.joints())) {
        Joint first = ring.interlocks().getFirst().joint();
        found.add(Places.refused(first, "as the robot starts, it " + deadlock.get()));
      }
    }
    return found;
  }
}
