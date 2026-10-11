package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.robot.Sensor.Switch.Limit.End;
import java.util.Locale;
import java.util.Optional;
import org.wpilib.units.measure.Distance;

/**
 * Something that is true of the robot or is not: a joint is at one of its named positions, a joint
 * is at least or at most a position or between two, a joint runs at least a speed, or a sensor
 * reads something. An interlock restricts a joint while a condition holds ({@link Interlock}), and
 * a configuration is a set of joints each at a named position or output ({@link Configuration}).
 *
 * <p>A condition is written from the joint or the sensor it is about: {@code
 * PIVOT.atLeast(Degrees.of(110))}, {@code ELEVATOR.atMost(Inches.of(32))}, {@code
 * ELEVATOR.atMost(ELEVATOR_CLEAR)} and {@code SHOOTER.upToSpeed(RPM.of(3000))} for an interlock,
 * and {@code ARM.at(ARM_SCORING)} for a configuration.
 *
 * <p>A condition is a fixed fact about positions. It does not depend on which way the joint is
 * moving or on where the joint rests, so it means the same on every robot of the design. A
 * condition on a position is judged exactly ({@link OnJoint#holdsAt}): the definition judges it at
 * named positions, and the robot judges it at the position it measures. That is why an interlock
 * refuses "at" a named position: a measured position is never exactly there.
 *
 * <p>A condition keeps a place it names just as the code named it. Whether that place is the
 * joint's own is for whatever uses the condition to say ({@link
 * RobotDefinition#whyNotOwn(Condition)}): the definition refuses an interlock or a configuration on
 * another joint's place, and the running robot refuses a gate on one, as it refuses a move to one.
 * So {@code PIVOT.atLeast(ELEVATOR_CLEAR)} can be built while the robot runs; its gate then holds
 * false and says why.
 */
public sealed interface Condition
    permits Condition.OnJoint, Condition.OnSensor, Condition.UpToSpeed {
  /**
   * A condition on where a joint is. Interlocks and configurations are made of these, and they are
   * the only conditions the definition can judge before the robot runs.
   */
  sealed interface OnJoint extends Condition
      permits Condition.At, Condition.AtLeast, Condition.AtMost, Condition.Between {
    /** The joint whose position it is about. */
    Joint joint();

    /**
     * Whether it holds with its joint at a position, in the joint's units, judged exactly.
     *
     * @throws IllegalStateException for a joint at a named output, which says how the joint runs
     *     and not where it is
     */
    boolean holdsAt(double position);
  }

  /**
   * A condition on what a sensor reads: a switch tripped or clear, or something nearer or farther
   * from a rangefinder than a distance. What the reading means is for the team to name ("coral in
   * claw" is a beam tripped). Only the running robot can judge it, by reading the sensor.
   */
  sealed interface OnSensor extends Condition
      permits Condition.Tripped, Condition.Clear, Condition.Nearer, Condition.Farther {
    /** The sensor it reads. */
    Sensor sensor();
  }

  /** A switch tripped: pressed, its magnet there, its beam blocked. {@code clawBeam.tripped()}. */
  record Tripped(Sensor.Switch sensor) implements OnSensor {
    @Override
    public String toString() {
      return sensor.label() + " tripped";
    }
  }

  /** A switch clear: not pressed, its beam unbroken. {@code hopperBeam.clear()}. */
  record Clear(Sensor.Switch sensor) implements OnSensor {
    @Override
    public String toString() {
      return sensor.label() + " clear";
    }
  }

  /**
   * Something nearer a rangefinder than a distance: {@code hopperRange.nearerThan(Inches.of(6))}.
   *
   * @param distance more than none and short of the sensor's range. At the range or beyond it, the
   *     sensor would read everything as nearer, so the condition is refused.
   */
  record Nearer(Sensor.Rangefinder sensor, Distance distance) implements OnSensor {
    public Nearer {
      Places.refuse(whyNotInRange(sensor, distance));
    }

    @Override
    public String toString() {
      return sensor.label() + " nearer than " + shown(distance);
    }
  }

  /**
   * Nothing nearer a rangefinder than a distance: {@code hopperRange.fartherThan(...)}.
   *
   * @param distance more than none and short of the sensor's range. At the range or beyond it, the
   *     sensor would never read anything as farther, so the condition is refused.
   */
  record Farther(Sensor.Rangefinder sensor, Distance distance) implements OnSensor {
    public Farther {
      Places.refuse(whyNotInRange(sensor, distance));
    }

    @Override
    public String toString() {
      return sensor.label() + " farther than " + shown(distance);
    }
  }

  /**
   * Why a rangefinder cannot tell a distance, if it cannot. The distance must be more than none and
   * short of the farthest the sensor reads.
   *
   * <p>A distance at the sensor's range or beyond it is refused because the condition could never
   * change: nothing the sensor reads is farther than its range, and everything it reads is nearer.
   * A distance within a hair of the range counts as the range, as a position within a hair of a
   * joint's stop counts as the stop ({@link Range#EDGE}).
   */
  private static Optional<String> whyNotInRange(Sensor.Rangefinder sensor, Distance distance) {
    double meters = distance.in(Meters);
    double rangeMeters = sensor.range().in(Meters);
    boolean moreThanNone = meters > 0;
    if (!moreThanNone) {
      return Optional.of(
          shown(distance)
              + " isn't inside "
              + possessive(sensor.label())
              + " range, "
              + shown(sensor.range()));
    }
    boolean shortOfTheRange = meters < rangeMeters - Range.EDGE;
    if (shortOfTheRange) {
      return Optional.empty();
    }
    return Optional.of(
        shown(distance)
            + " is at or beyond "
            + possessive(sensor.label())
            + " range, "
            + shown(sensor.range())
            + ", so the condition could never change: either the distance or the sensor's stated"
            + " range is wrong");
  }

  private static String shown(Distance distance) {
    return String.format(Locale.ROOT, "%.3f m", distance.in(Meters));
  }

  /**
   * A joint at one of its named positions or outputs: {@code ARM.at(ARM_STOWED)}, {@code
   * FEEDER.at(FEEDER_FEEDING)}, {@code FEEDER.at(Output.STOPPED)}. A configuration is made of
   * these. An interlock refuses one. At a named position it would hold only with the joint exactly
   * there, and a measured position never is, so an interlock is written with an edge instead:
   * {@link AtLeast}, {@link AtMost} or {@link Between}.
   *
   * @param joint the joint
   * @param position one of its positions, as the code names it; empty for an output
   * @param output one of its outputs, as the code names it, or {@link Output#STOPPED}, which every
   *     joint has; empty for a position
   */
  @ConfigRecord
  record At(Joint joint, Optional<Position> position, Optional<Output> output) implements OnJoint {
    public At {
      if (position.isPresent() == output.isPresent()) {
        throw new IllegalArgumentException(
            joint.label() + " at a place is at one of its positions or one of its outputs");
      }
    }

    /** Starts describing a joint at a named place, one named value at a time. */
    public static ConditionAtBuilder.JointStage builder() {
      return ConditionAtBuilder.builder();
    }

    /** The name of its place: one of the joint's positions or outputs, or {@code "stopped"}. */
    public String name() {
      return position.map(Position::name).orElseGet(() -> output.orElseThrow().name());
    }

    /** Whether it names an output and not a position. */
    public boolean isOutput() {
      return output.isPresent();
    }

    @Override
    public boolean holdsAt(double at) {
      if (isOutput()) {
        throw new IllegalStateException(this + " is an output, not a position");
      }
      return at == joint.position(name()).value();
    }

    @Override
    public String toString() {
      return joint.label() + " at " + name();
    }
  }

  /**
   * A joint at a position or beyond it, toward the upper end of its range: {@code
   * PIVOT.atLeast(Degrees.of(110))}, or from one of its named positions on, {@code
   * PIVOT.atLeast(PIVOT_OUT)}. An interlock's edge is usually one of these: a clearance, which is
   * the same wherever the joint rests.
   *
   * @param joint the joint
   * @param position the edge, in the joint's units. It is inside the joint's range and above its
   *     lower end, where the condition would hold everywhere.
   * @param place the joint's named position at the edge, as the code names it, when the edge was
   *     written as one
   */
  @ConfigRecord
  record AtLeast(Joint joint, double position, Optional<Position> place) implements OnJoint {
    public AtLeast {
      if (isOwn(joint, place)) {
        Places.refuse(whyNotAnEdge(joint, position, place, End.LOWER));
      }
    }

    /** Starts describing a joint at least at a position, one named value at a time. */
    public static ConditionAtLeastBuilder.JointStage builder() {
      return ConditionAtLeastBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      return at >= position;
    }

    @Override
    public String toString() {
      return joint.label() + " at least " + shown(joint, position, place);
    }
  }

  /**
   * A joint at a position or short of it, toward the lower end of its range: {@code
   * ELEVATOR.atMost(Inches.of(32))}, or up to one of its named positions, {@code
   * ELEVATOR.atMost(ELEVATOR_CLEAR)}. Like {@link AtLeast}, it is what an interlock's edge usually
   * is.
   *
   * @param joint the joint
   * @param position the edge, in the joint's units. It is inside the joint's range and below its
   *     upper end, where the condition would hold everywhere.
   * @param place the joint's named position at the edge, as the code names it, when the edge was
   *     written as one
   */
  @ConfigRecord
  record AtMost(Joint joint, double position, Optional<Position> place) implements OnJoint {
    public AtMost {
      if (isOwn(joint, place)) {
        Places.refuse(whyNotAnEdge(joint, position, place, End.UPPER));
      }
    }

    /** Starts describing a joint at most at a position, one named value at a time. */
    public static ConditionAtMostBuilder.JointStage builder() {
      return ConditionAtMostBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      return at <= position;
    }

    @Override
    public String toString() {
      return joint.label() + " at most " + shown(joint, position, place);
    }
  }

  /**
   * A joint between two positions, both included, given in either order.
   *
   * @param joint the joint
   * @param from one end, in the joint's units, inside its range
   * @param to the other end
   * @param fromPlace the joint's named position at {@code from}, as the code names it, when that
   *     end was written as one
   * @param toPlace the same for {@code to}
   */
  @ConfigRecord
  record Between(
      Joint joint, double from, double to, Optional<Position> fromPlace, Optional<Position> toPlace)
      implements OnJoint {
    public Between {
      if (isOwn(joint, fromPlace) && isOwn(joint, toPlace)) {
        Places.refuse(
            whyNotWithin(joint, from, fromPlace),
            whyNotWithin(joint, to, toPlace),
            whyNotApart(joint, from, to, fromPlace));
      }
    }

    /** Starts describing a joint between two positions, one named value at a time. */
    public static ConditionBetweenBuilder.JointStage builder() {
      return ConditionBetweenBuilder.builder();
    }

    @Override
    public boolean holdsAt(double at) {
      return at >= Math.min(from, to) && at <= Math.max(from, to);
    }

    @Override
    public String toString() {
      return joint.label()
          + " between "
          + shown(joint, from, fromPlace)
          + " and "
          + shown(joint, to, toPlace);
    }
  }

  /**
   * A joint running forward at a speed or faster: a flywheel up to speed before anything feeds it.
   * Like a condition on a sensor, only the running robot can judge it, by reading the joint. It
   * does not hold for a joint that is not answering or does not know where it is.
   *
   * @param joint the joint
   * @param speed the speed to reach, in the joint's units a second: radians, or meters
   */
  @ConfigRecord
  record UpToSpeed(Joint joint, double speed) implements Condition {
    public UpToSpeed {
      Places.refuse(whyNotForward(joint, speed));
    }

    /** Starts describing a joint running at least a speed, one named value at a time. */
    public static ConditionUpToSpeedBuilder.JointStage builder() {
      return ConditionUpToSpeedBuilder.builder();
    }

    /** Whether it holds with the joint running at a speed, in its units a second. */
    public boolean holdsAt(double velocity) {
      return velocity >= speed;
    }

    @Override
    public String toString() {
      return joint.label() + " at least " + joint.shownSpeed(speed);
    }
  }

  /** Why a speed is not one a joint can be waited on to reach, if it is not. */
  private static Optional<String> whyNotForward(Joint joint, double speed) {
    boolean forward = Double.isFinite(speed) && speed > 0;
    if (forward) {
      return Optional.empty();
    }
    return Optional.of(
        "a speed to reach must be forward, and more than none: " + joint.shownSpeed(speed));
  }

  /** A position as a message shows it: by its name when the code named it, or else its value. */
  private static String shown(Joint joint, double position, Optional<Position> place) {
    return place.map(Position::name).orElseGet(() -> joint.shown(position));
  }

  /**
   * Whether the place a condition names, if it names one, is the joint's own. A condition checks
   * its position only when it is. A place that is another joint's is refused later, by whatever
   * uses the condition, which can say whose place it is.
   */
  private static boolean isOwn(Joint joint, Optional<Position> place) {
    return place.isEmpty() || joint.whyNotOwn(place.get()).isEmpty();
  }

  /**
   * Why a joint cannot be at a position, if it cannot. The position must be a number, it must be
   * where the joint has its named place if the code named one, and it must be inside the joint's
   * range.
   */
  private static Optional<String> whyNotWithin(
      Joint joint, double position, Optional<Position> place) {
    if (!Double.isFinite(position)) {
      return Optional.of(position + " isn't a position");
    }
    return whyNotAtItsPlace(joint, position, place)
        .or(() -> whyOutsideRange(joint, position, place));
  }

  /**
   * Why a position is not where the joint has the place it is named for, if it is not. This is only
   * so for a condition built by hand with a place and some other position. A position within a hair
   * of the place ({@link Range#EDGE}) is at it: the same angle converted from degrees two ways can
   * differ by a rounding step.
   */
  private static Optional<String> whyNotAtItsPlace(
      Joint joint, double position, Optional<Position> place) {
    if (place.isEmpty()) {
      return Optional.empty();
    }
    String name = place.get().name();
    double placedAt = joint.position(name).value();
    boolean atItsPlace = Math.abs(placedAt - position) <= Range.EDGE;
    if (atItsPlace) {
      return Optional.empty();
    }
    return Optional.of(name + " is " + joint.shown(placedAt) + ", not " + joint.shown(position));
  }

  /** Why a position is outside a joint's range, if the joint has a range and it is. */
  private static Optional<String> whyOutsideRange(
      Joint joint, double position, Optional<Position> place) {
    if (joint.range().isEmpty() || joint.range().get().contains(position)) {
      return Optional.empty();
    }
    Range range = joint.range().get();
    return Optional.of(
        shown(joint, position, place)
            + " is outside "
            + possessive(joint.label())
            + " range, "
            + joint.shown(range.lower())
            + " to "
            + joint.shown(range.upper()));
  }

  /**
   * Why a position cannot be the edge of an "at least" or an "at most", if it cannot. It must be
   * somewhere the joint can be ({@link #whyNotWithin}), and it must not be the end of the joint's
   * range at which the condition would hold everywhere and so restrict nothing.
   *
   * @param holdsEverywhereAt the end of the range where the condition would hold everywhere: the
   *     lower end for "at least", and the upper end for "at most"
   */
  private static Optional<String> whyNotAnEdge(
      Joint joint, double position, Optional<Position> place, End holdsEverywhereAt) {
    Optional<String> notWithin = whyNotWithin(joint, position, place);
    if (notWithin.isPresent() || joint.range().isEmpty()) {
      return notWithin;
    }
    Range range = joint.range().get();
    double end =
        switch (holdsEverywhereAt) {
          case LOWER -> range.lower();
          case UPPER -> range.upper();
        };
    boolean atThatEnd = Math.abs(position - end) <= Range.EDGE;
    if (!atThatEnd) {
      return Optional.empty();
    }
    return Optional.of(
        shown(joint, position, place)
            + " is "
            + possessive(joint.label())
            + " "
            + switch (holdsEverywhereAt) {
              case LOWER -> "lowest: it's at least that";
              case UPPER -> "highest: it's at most that";
            }
            + " everywhere");
  }

  /** Why two positions cannot be the ends of a "between", if they cannot: they are one place. */
  private static Optional<String> whyNotApart(
      Joint joint, double from, double to, Optional<Position> fromPlace) {
    boolean atOnePlace = Math.abs(from - to) <= Range.EDGE;
    if (!atOnePlace) {
      return Optional.empty();
    }
    return Optional.of("a joint between " + shown(joint, from, fromPlace) + " and itself is at it");
  }
}
