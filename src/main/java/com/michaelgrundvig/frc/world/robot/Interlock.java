package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Optional;

/**
 * An interlock is a rule that restricts one joint according to where another joint is, how fast
 * another joint runs, or what a sensor reads. There are two kinds.
 *
 * <p>A run-only-when ({@link RunsOnlyWhen}) lets its joint move only while a condition holds. Here
 * a roller runs only once its pivot is out, written in the roller's block:
 *
 * {@snippet class=com.michaelgrundvig.frc.world.examples.ExampleJoints region=runs-only-when}
 *
 * <p>A keep-out ({@link KeepsOut}) keeps its joint out of a region of its travel while a condition
 * holds. Here an arm is kept out of the bumper's band while the elevator is too low, written in the
 * arm's block:
 *
 * {@snippet class=com.michaelgrundvig.frc.world.examples.ExampleJoints region=keeps-out}
 *
 * <p>An interlock is normally written in the block of the joint it restricts. When its condition
 * names a joint that is declared later, so the block cannot refer to it yet, it is written in the
 * robot's definition instead: {@code Interlock.that(ELEVATOR).keepsOut(...).when(...)}.
 *
 * <p>Interlocks are facts about the robot. They live in its definition and are enforced beneath
 * every command, so no command, binding, auto or button can skip one. They speak only of the
 * robot's own joints and sensors. A rule that depends on the field or the game (a trench, a zone, a
 * hub) is a gate in the robot's code instead, because the robot's code knows the field and this
 * model of the robot does not.
 *
 * <p>A condition that cannot be judged restricts. When a sensor is lost, or a joint does not know
 * where it is, a keep-out stays closed and a run-only-when holds its joint.
 *
 * <p>The definition checks its interlocks when it is built. Every joint and sensor must be one of
 * the robot's, and none may be a joint that follows another. No joint may be held where it cannot
 * move, or kept from a named position it could then never reach. Every configuration, and the robot
 * as it starts, must obey every interlock whose condition is on a joint's position (what a sensor
 * reads is not known until the robot runs). And no group of joints may hold each other so that none
 * can move to free another.
 *
 * <p>This type only declares the rules. What they allow a joint to do in any one loop is decided by
 * {@link InterlockEnforcement}.
 */
public sealed interface Interlock permits Interlock.RunsOnlyWhen, Interlock.KeepsOut {
  /** The joint it restricts. */
  Joint joint();

  /**
   * What must be true for the interlock to act. A run-only-when lets its joint move while this
   * holds; a keep-out closes its region while this holds.
   */
  Condition condition();

  /**
   * The other joint, the one the condition is about, whether it reads that joint's position or its
   * speed. A condition on a sensor has no other joint.
   */
  default Optional<Joint> other() {
    return switch (condition()) {
      case Condition.OnJoint onJoint -> Optional.of(onJoint.joint());
      case Condition.UpToSpeed upToSpeed -> Optional.of(upToSpeed.joint());
      case Condition.OnSensor onSensor -> Optional.empty();
    };
  }

  /** Starts an interlock about a joint: {@code Interlock.that(ROLLER).runsOnlyWhen(...)}. */
  static Subject that(Joint joint) {
    return new Subject(joint);
  }

  /**
   * A run-only-when: its joint may move only while a condition holds. While the condition does not
   * hold, a joint that holds positions stays where it comes to a stop, and a joint that only runs
   * at outputs stops. Either carries on with what it was asked once the condition holds again. For
   * example, an intake roller runs only with its pivot past 110°.
   *
   * @param joint the joint it restricts
   * @param condition where another joint must be, how fast it must run, or what a sensor must read,
   *     for this joint to move
   */
  @ConfigRecord
  record RunsOnlyWhen(Joint joint, Condition condition) implements Interlock {
    public RunsOnlyWhen {
      Places.refuse(whyNotAboutAnother(joint, condition));
    }

    /** Starts describing the interlock, one named value at a time. */
    public static InterlockRunsOnlyWhenBuilder.JointStage builder() {
      return InterlockRunsOnlyWhenBuilder.builder();
    }

    @Override
    public String toString() {
      return joint.label() + " runs only when " + condition;
    }
  }

  /**
   * A keep-out: its joint is kept out of a region of its travel while a condition holds. The region
   * is closed while the condition holds and open otherwise. While it is closed, a joint outside it
   * is stopped at the edge on the side it is on. For example, an arm is kept out of the bumper's
   * band while the elevator is too low to clear it.
   *
   * <p>A joint can be inside the region when it closes, or when the robot starts. It is then
   * allowed only to leave, by the side {@link #sideKeptTo} names.
   *
   * @param joint the joint it restricts
   * @param region where the joint may not be while the condition holds, in the joint's units. It
   *     may be written larger than the joint's travel: a region of -40° to 0° on an arm that
   *     travels from -30° to 120° is accepted. The part of it inside the travel is what counts, so
   *     that one reaches the low end of the arm's travel, as -30° to 0° does. It must cover some of
   *     the travel and never all of it.
   * @param condition where another joint is, how fast it runs, or what a sensor reads, while the
   *     region is closed
   */
  @ConfigRecord
  record KeepsOut(Joint joint, Range region, Condition condition) implements Interlock {
    /** A side of the region, along the joint's travel. */
    public enum Side {
      /** Toward the low end of the joint's travel, at or beyond the region's lower edge. */
      BELOW,
      /** Toward the high end of the joint's travel, at or beyond the region's upper edge. */
      ABOVE
    }

    public KeepsOut {
      Places.refuse(whyNotAboutAnother(joint, condition), whyNotPartOfRange(joint, region));
    }

    /** Starts describing the interlock, one named value at a time. */
    public static InterlockKeepsOutBuilder.JointStage builder() {
      return InterlockKeepsOutBuilder.builder();
    }

    /**
     * Whether a position is inside the region.
     *
     * <p>An edge of the region is normally outside it: the edge is where a joint is held, so it
     * must be somewhere the joint is allowed to be. The exception is an edge that sits at an end of
     * the joint's range. There is nowhere beyond that edge for the joint to be, so the region is
     * taken to carry on past it: a joint standing on that edge is inside, and so is one whose
     * encoder reads a little past the hard stop. Either has to leave by the other edge. A region
     * written larger than the joint's travel, with an edge beyond an end of the range, is treated
     * the same way as one whose edge sits at that end.
     */
    public boolean isInside(double position) {
      if (Double.isNaN(position)) {
        return false;
      }
      boolean aboveLowerEdge = !hasRoomBelow() || position > region.lower();
      boolean belowUpperEdge = !hasRoomAbove() || position < region.upper();
      return aboveLowerEdge && belowUpperEdge;
    }

    /**
     * Which side of the region the joint is kept to while the region is closed.
     *
     * <p>A joint outside the region is kept to the side it is on. A joint inside the region may
     * only leave it, and this says by which side. It leaves by the side it was last outside on,
     * because that is the way it came in. A joint that has never been outside (it started inside)
     * leaves by the nearer edge. In either case it never leaves by a side with no room beyond it.
     *
     * @param position where the joint is
     * @param lastOutside the side the joint was on when it was last outside the region, or empty if
     *     it has not been outside it yet
     */
    public Side sideKeptTo(double position, Optional<Side> lastOutside) {
      if (position <= region.lower() && hasRoomBelow()) {
        return Side.BELOW;
      }
      if (position >= region.upper() && hasRoomAbove()) {
        return Side.ABOVE;
      }
      // From here on the joint has to leave the region: it is inside it, or it reads past an end of
      // its range that the region reaches.
      if (!hasRoomAbove()) {
        return Side.BELOW;
      }
      if (!hasRoomBelow()) {
        return Side.ABOVE;
      }
      if (lastOutside.isPresent()) {
        return lastOutside.get();
      }
      return sideOfNearerEdge(position);
    }

    /** Which side the joint is kept to when it has not been outside the region yet. */
    public Side sideKeptTo(double position) {
      return sideKeptTo(position, Optional.empty());
    }

    /** The edge of the region on one side: where a joint kept to that side is stopped. */
    public double edge(Side side) {
      return switch (side) {
        case BELOW -> region.lower();
        case ABOVE -> region.upper();
      };
    }

    /**
     * How far a position is past the region's edge on one side, measured into the region. It is
     * zero at the edge and negative while the position is still outside the region on that side.
     * For the side a joint is kept to, this is how far the joint, or a goal it is asked for, has
     * gone beyond where it may be.
     */
    public double distancePastEdge(Side side, double position) {
      return switch (side) {
        case BELOW -> position - region.lower();
        case ABOVE -> region.upper() - position;
      };
    }

    /** The side whose edge is nearer a position. Halfway between the edges counts as below. */
    private Side sideOfNearerEdge(double position) {
      double toLowerEdge = position - region.lower();
      double toUpperEdge = region.upper() - position;
      if (toLowerEdge <= toUpperEdge) {
        return Side.BELOW;
      }
      return Side.ABOVE;
    }

    /**
     * Whether the joint has anywhere to be below the region. It has not when the region reaches the
     * low end of the joint's range. A joint with no range can always go further.
     */
    private boolean hasRoomBelow() {
      Optional<Range> range = joint.range();
      return range.isEmpty() || region.lower() > range.get().lower();
    }

    /** Whether the joint has anywhere to be above the region: see {@link #hasRoomBelow}. */
    private boolean hasRoomAbove() {
      Optional<Range> range = joint.range();
      return range.isEmpty() || region.upper() < range.get().upper();
    }

    @Override
    public String toString() {
      return joint.label()
          + " keeps out of "
          + joint.shown(region.lower())
          + " to "
          + joint.shown(region.upper())
          + " while "
          + condition;
    }
  }

  /**
   * The first step of writing an interlock: the joint it will restrict. It is not an interlock
   * until one of its methods says what the joint may do.
   *
   * @param joint the joint
   */
  record Subject(Joint joint) {
    /** The joint moves only while the condition holds. */
    public RunsOnlyWhen runsOnlyWhen(Condition condition) {
      return RunsOnlyWhen.builder().joint(joint).condition(condition).build();
    }

    /** The joint is kept out of a region of its travel. {@link KeepingOut#when} says while what. */
    public KeepingOut keepsOut(Range region) {
      return new KeepingOut(joint, region);
    }
  }

  /**
   * The second step of writing a keep-out: the joint and the region. It is not an interlock until
   * {@link #when} says while what.
   *
   * @param joint the joint
   * @param region where it may not be
   */
  record KeepingOut(Joint joint, Range region) {
    /** The region is closed while the condition holds. */
    public KeepsOut when(Condition condition) {
      return KeepsOut.builder().joint(joint).region(region).condition(condition).build();
    }
  }

  /**
   * Why a condition cannot restrict a joint, if it cannot. An interlock has to be about something
   * other than the joint it restricts, and has to be something the robot can judge while it runs. A
   * condition on a sensor always is.
   */
  private static Optional<String> whyNotAboutAnother(Joint joint, Condition condition) {
    return switch (condition) {
      case Condition.OnSensor onSensor -> Optional.empty();
      case Condition.UpToSpeed upToSpeed -> whyNotOnSpeed(joint, upToSpeed);
      case Condition.OnJoint onJoint -> whyNotOnPosition(joint, onJoint);
    };
  }

  /** Why a joint cannot be restricted by how fast a joint runs: only when it is the same joint. */
  private static Optional<String> whyNotOnSpeed(Joint joint, Condition.UpToSpeed condition) {
    if (condition.joint().equals(joint)) {
      return Optional.of(
          joint.label() + " can't be held by how fast it runs itself: an interlock is on another");
    }
    return Optional.empty();
  }

  /**
   * Why a joint cannot be restricted by where a joint is. The other joint must be a different one,
   * and the condition must be an edge or a band, not "at" a named place ({@link #whyNotAtAPlace}).
   */
  private static Optional<String> whyNotOnPosition(Joint joint, Condition.OnJoint condition) {
    if (condition.joint().equals(joint)) {
      return Optional.of(
          joint.label()
              + " can't be held by where it is itself: limit it by its range, or an interlock on"
              + " another joint");
    }
    if (condition instanceof Condition.At at) {
      return Optional.of(whyNotAtAPlace(at));
    }
    return Optional.empty();
  }

  /**
   * Why an interlock cannot be on a joint being "at" a named place. A named output says how the
   * joint runs, not where it is. A named position holds only with the joint exactly there, and the
   * robot judges the condition against a measured position, which is never exactly anywhere, so the
   * interlock would never act.
   */
  private static String whyNotAtAPlace(Condition.At at) {
    String other = at.joint().label();
    if (at.isOutput()) {
      return "an interlock speaks of where a joint is, or how fast it runs, and "
          + at.name()
          + " is "
          + possessive(other)
          + " output: say the speed it must reach, "
          + other
          + ".upToSpeed(...)";
    }
    String of =
        switch (Position.Kind.of(at.joint())) {
          case ANGLE -> "Degrees.of(...)";
          case LENGTH -> "Inches.of(...)";
        };
    return at
        + " holds only with "
        + other
        + " exactly at "
        + at.name()
        + ", "
        + at.joint().shown(at.position().orElseThrow().value())
        + ", which a measured position never is, so the interlock would never hold: say the"
        + " edge where the parts clear, "
        + other
        + ".atLeast("
        + of
        + ") or "
        + other
        + ".atMost("
        + of
        + "), or a band around "
        + at.name()
        + ", "
        + other
        + ".between("
        + of
        + ", "
        + of
        + ")";
  }

  /**
   * Why a region cannot be one a joint is kept out of, if it cannot. The region must be in the
   * joint's units, must overlap the joint's range, and must not cover all of it, which would leave
   * the joint nowhere to be.
   *
   * <p>A region that sticks out past one end of the range is accepted. Only the part of it inside
   * the range counts, so it is treated as reaching that end ({@link KeepsOut#isInside}).
   */
  private static Optional<String> whyNotPartOfRange(Joint joint, Range region) {
    Position.Kind jointsKind = Position.Kind.of(joint);
    if (region.kind() != jointsKind) {
      return Optional.of(JointChecks.wrongKind("the region is", jointsKind));
    }
    if (joint.range().isEmpty()) {
      return Optional.empty();
    }
    Range range = joint.range().get();
    String regionShown = joint.shown(region.lower()) + " to " + joint.shown(region.upper());
    String rangeShown = joint.shown(range.lower()) + " to " + joint.shown(range.upper());
    boolean outsideTheRange = region.upper() <= range.lower() || region.lower() >= range.upper();
    if (outsideTheRange) {
      return Optional.of(
          regionShown + " is outside " + possessive(joint.label()) + " range, " + rangeShown);
    }
    boolean allOfTheRange = region.lower() <= range.lower() && region.upper() >= range.upper();
    if (allOfTheRange) {
      return Optional.of(
          regionShown
              + " is all of "
              + possessive(joint.label())
              + " range, "
              + rangeShown
              + ": it would have nowhere to be");
    }
    return Optional.empty();
  }
}
