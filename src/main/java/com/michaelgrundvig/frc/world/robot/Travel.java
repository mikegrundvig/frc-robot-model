package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import java.util.Optional;
import java.util.OptionalDouble;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.util.MathUtil;

/**
 * Where a joint may be driven and how quickly it gets there and stops: its limits, the cruise speed
 * and acceleration of its motion profile, and its free speed. It is worked out once from the
 * joint's block. Every method is a function of what it is given, in the joint's units (radians or
 * meters), so the robot, the simulator and a replay of a log all get the same answers.
 *
 * <p>The limits are the control's soft limits if it has them, otherwise the joint's hard stops, and
 * otherwise there are none: the limits are then infinite.
 *
 * <p>A joint takes a position it is asked for in one of three ways, and the two flags say which:
 *
 * <ul>
 *   <li>Most joints take the position as it is, held inside the limits. Both flags are false.
 *   <li>A joint that turns with no stops at all ({@code wraps}) can reach any angle either way
 *       round, so it goes the short way ({@link #goalNear}).
 *   <li>A turret whose range spans a whole turn or more ({@code choosesTurn}) can face every
 *       direction, and some directions at more than one position. An angle that code computes for
 *       it, such as the bearing of a target, is therefore a heading, and the joint picks which turn
 *       of that heading to go to ({@link #turnNear}).
 * </ul>
 *
 * @param joint the joint, which says how a position in its units is shown
 * @param lower its lower limit, or negative infinity for none
 * @param upper its upper limit, or positive infinity for none
 * @param wraps whether it turns freely, with no stops, so that a goal is taken the short way round
 * @param choosesTurn whether its range spans a whole turn or more, as a turret's does, so that a
 *     computed angle is a heading ({@link #turnNear})
 * @param cruise the cruise speed of its profile, in its units a second
 * @param acceleration the acceleration of its profile, in its units a second squared. It brakes at
 *     this too.
 * @param freeSpeed the speed a full battery runs it at with nothing loading it, in its units a
 *     second: its motors' free speed through its gearing
 */
public record Travel(
    Joint joint,
    double lower,
    double upper,
    boolean wraps,
    boolean choosesTurn,
    double cruise,
    double acceleration,
    double freeSpeed) {
  /**
   * A joint whose control gives no profile cruises at this share of its free speed, which is slow
   * enough to be safe the first time it is enabled. Giving it a profile lets it go faster.
   */
  static final double DEFAULT_CRUISE_SHARE = 0.25;

  /** A joint whose control gives no profile reaches its cruise speed in this long, in seconds. */
  static final double DEFAULT_RAMP_SECONDS = 0.25;

  /** One whole turn, in radians. */
  private static final double TURN = 2 * Math.PI;

  /**
   * How far short of a whole turn, or past a limit, a position may be and still count as at it, in
   * the joint's units. A range written in degrees as a whole turn converts to a floating-point step
   * less than 2π, and a position that lies exactly on a limit, given in other units, can convert to
   * a step past it. It is the hair a range allows its own stops ({@link Range#EDGE}).
   */
  private static final double EDGE = Range.EDGE;

  /** The lowest and highest positions a joint is driven to. Either may be infinite. */
  private record Limits(double lower, double upper) {
    static final Limits NONE = new Limits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
  }

  /** A joint's travel, from its block and its control. */
  public static Travel of(RobotMath math, Joint joint, MechanismControl control) {
    Limits limits = limitsOf(joint, control);
    boolean turns = joint instanceof Joint.Revolute;
    boolean turnsFreely = turns && joint.range().isEmpty();
    boolean spansAWholeTurn =
        turns && joint.range().map(range -> range.width() >= TURN - EDGE).orElse(false);

    double freeSpeed = math.freeSpeed(joint);
    double defaultCruise = DEFAULT_CRUISE_SHARE * freeSpeed;
    double defaultAcceleration = defaultCruise / DEFAULT_RAMP_SECONDS;
    Optional<MechanismControl.Profile> profile = control.profile();
    double cruise = profile.map(MechanismControl.Profile::cruiseVelocity).orElse(defaultCruise);
    double acceleration =
        profile.map(MechanismControl.Profile::acceleration).orElse(defaultAcceleration);

    return new Travel(
        joint,
        limits.lower(),
        limits.upper(),
        turnsFreely,
        spansAWholeTurn,
        cruise,
        acceleration,
        freeSpeed);
  }

  /**
   * The control's soft limits if it has them, otherwise the joint's hard stops, if it has those.
   */
  private static Limits limitsOf(Joint joint, MechanismControl control) {
    if (control.softLimits().isPresent()) {
      MechanismControl.SoftLimits soft = control.softLimits().get();
      return new Limits(soft.lower(), soft.upper());
    }
    if (joint.range().isPresent()) {
      Range stops = joint.range().get();
      return new Limits(stops.lower(), stops.upper());
    }
    return Limits.NONE;
  }

  /** The position nearest the one given that is inside the limits. */
  public double clamp(double position) {
    return Math.max(lower, Math.min(upper, position));
  }

  /**
   * Where the joint's profile really heads when the joint is asked for a position. The position is
   * first held inside the limits. A joint that turns freely then goes to whichever turn of it is
   * nearest where the joint is, which is the short way round: asked for -170° from 170°, it heads
   * for 190°.
   *
   * @param position the position asked for
   * @param from where the joint is
   */
  public double goalNear(double position, double from) {
    double inside = clamp(position);
    if (!wraps) {
      return inside;
    }
    double shortWayRound = MathUtil.angleModulus(inside - from);
    return from + shortWayRound;
  }

  /**
   * Where an angle that code computed takes the joint.
   *
   * <p>For a turret whose range spans a whole turn or more, the angle is a heading, and the same
   * heading comes round again every whole turn. The joint goes to the turn of it that is inside its
   * limits and nearest where it is. So a turret at 170° asked for -170° carries on to 190°, and one
   * that can reach a heading only the long way round goes the long way.
   *
   * <p>Soft limits narrower than a turn can leave no turn of the heading inside them. The answer is
   * then the limit nearer the heading, measured the short way round. A move there is refused before
   * it starts ({@link #whyUnreachable}), and a joint that is following a moving target holds at
   * that limit.
   *
   * <p>Any other joint takes the angle as it is.
   *
   * @param angle the angle computed
   * @param from where the joint is
   */
  public double turnNear(double angle, double from) {
    if (!choosesTurn) {
      return angle;
    }
    OptionalDouble inside = nearestTurnInside(angle, from);
    if (inside.isPresent()) {
      return inside.getAsDouble();
    }
    return limitNearer(angle);
  }

  /**
   * The turn of a heading that is inside the limits and nearest a point, if any turn of it is
   * inside them.
   */
  private OptionalDouble nearestTurnInside(double heading, double from) {
    // The search starts at the lowest turn of the heading that is at or above the lower limit, and
    // steps up one turn at a time. The number of steps is counted from the width of the limits
    // rather than run until a turn passes the upper limit, so that a heading too large for adding
    // one turn to change it still ends the loop.
    double lowestTurn = heading + TURN * Math.ceil((lower - heading) / TURN - EDGE);
    int turnsTheLimitsSpan = (int) ((upper - lower) / TURN) + 1;
    OptionalDouble nearest = OptionalDouble.empty();
    for (int turns = 0; turns <= turnsTheLimitsSpan; turns++) {
      double candidate = lowestTurn + turns * TURN;
      boolean insideLimits = candidate >= lower - EDGE && candidate <= upper + EDGE;
      if (!insideLimits) {
        continue;
      }
      boolean nearerThanAnySoFar =
          nearest.isEmpty() || Math.abs(candidate - from) < Math.abs(nearest.getAsDouble() - from);
      if (nearerThanAnySoFar) {
        nearest = OptionalDouble.of(candidate);
      }
    }
    if (nearest.isEmpty()) {
      return nearest;
    }
    // A turn that counted as inside by a hair is put exactly on the limit.
    return OptionalDouble.of(clamp(nearest.getAsDouble()));
  }

  /**
   * The limit that is nearer a heading, measured the short way round. A heading equally far from
   * both counts as nearer the lower.
   */
  private double limitNearer(double heading) {
    double toLower = Math.abs(MathUtil.angleModulus(heading - lower));
    double toUpper = Math.abs(MathUtil.angleModulus(heading - upper));
    if (toLower <= toUpper) {
      return lower;
    }
    return upper;
  }

  /**
   * Why the joint cannot be sent to a position, or null when it can. It cannot when the position is
   * outside its limits by more than a hair ({@link #EDGE}): a position a rounding step past a limit
   * is at the limit, and the joint goes to the limit ({@link #clamp}). For a turret whose range
   * spans a whole turn or more, the position is a heading, and it cannot only when every turn of
   * that heading is outside its limits.
   *
   * @param position the position asked for
   * @param from where the joint is
   */
  public @Nullable String whyUnreachable(double position, double from) {
    boolean insideLimits = position >= lower - EDGE && position <= upper + EDGE;
    if (insideLimits) {
      return null;
    }
    if (!choosesTurn) {
      return joint.shown(position) + " is outside " + shownLimits();
    }
    if (nearestTurnInside(position, from).isPresent()) {
      return null;
    }
    return joint.shown(position) + ", at every turn of it, is outside " + shownLimits();
  }

  /** The limits as a refusal names them: "arm's limits, 5° to 130°". */
  private String shownLimits() {
    return possessive(joint.label())
        + " limits, "
        + joint.shown(lower)
        + " to "
        + joint.shown(upper);
  }

  /**
   * Where the joint comes to rest if it starts braking now, held inside its limits.
   *
   * <p>Braking at a steady rate a from a velocity v takes a distance of v² / 2a. It is written here
   * as v·|v| so that the distance keeps the sign of the velocity: it is added to where the joint
   * is, whichever way the joint is moving.
   *
   * @param at where the joint is
   * @param velocity how fast it is moving, in its units a second
   */
  public double stoppingPoint(double at, double velocity) {
    double brakingDistance = velocity * Math.abs(velocity) / (2 * acceleration);
    return clamp(at + brakingDistance);
  }

  /**
   * Where the joint would come to a stop if it carried on for one more loop as it is moving now,
   * and only then began to brake.
   *
   * <p>This is the stopping point that matters to anything that has to stop the joint short of a
   * place, because a hold decided in one loop only takes effect in the next. A hold that waited
   * until {@link #stoppingPoint} reached the place would be one loop late, and the joint would
   * overshoot by the distance it travels in a loop. At 5 rad/s and a 20 ms loop that is about 6°.
   *
   * <p>Over that one loop the joint is taken to go on as it is going: at its speed, and if it is
   * speeding up, speeding up at the same rate. A joint that is slowing down is taken to keep its
   * speed, so that this never promises a shorter stop than the joint may manage.
   *
   * @param at where the joint is
   * @param velocity how fast it is moving, a second
   * @param accelerating how fast its velocity is changing, measured over the last loop, a second
   *     squared
   * @param loopSeconds how long one loop lasts, in seconds
   */
  public double stoppingPointLoopOn(
      double at, double velocity, double accelerating, double loopSeconds) {
    double velocityALoopOn = velocityALoopOn(velocity, accelerating, loopSeconds);
    double averageVelocity = (velocity + velocityALoopOn) / 2;
    double positionALoopOn = at + averageVelocity * loopSeconds;
    return stoppingPoint(positionALoopOn, velocityALoopOn);
  }

  /**
   * The joint's velocity one loop on, if it goes on speeding up as it is. Only speeding up changes
   * it: for a joint moving up that is accelerating upward, and for one moving down, downward. A
   * joint at rest is moving neither way, so it is not speeding up.
   */
  private static double velocityALoopOn(double velocity, double accelerating, double loopSeconds) {
    if (velocity > 0) {
      double speedingUp = Math.max(0, accelerating);
      return velocity + speedingUp * loopSeconds;
    }
    if (velocity < 0) {
      double speedingUp = Math.min(0, accelerating);
      return velocity + speedingUp * loopSeconds;
    }
    return velocity;
  }
}
