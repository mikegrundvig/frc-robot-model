package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import org.jspecify.annotations.Nullable;
import org.wpilib.math.util.MathUtil;

/**
 * Where a joint is driven, and how fast: its limits (its soft limits, else its hard stops, else
 * anywhere), how it takes a goal inside them, its profile's cruise and acceleration, and its free
 * speed. A turret, whose range spans a whole turn or more, takes a computed angle as a heading, at
 * the turn of it nearest where it is; a joint that turns freely goes the short way round. Worked
 * out once from the joint's block, each answer a pure function of what it's given, in the joint's
 * units.
 *
 * @param joint the joint, which says a position in its units
 * @param lower its lower limit
 * @param upper its upper limit
 * @param wraps whether it turns freely, so a goal is taken the short way round
 * @param choosesTurn whether its range spans a whole turn or more, a turret's: it can face every
 *     way, at more than one position for some, so a computed angle is a heading ({@link #turnNear})
 * @param cruise its profile's cruise, a second
 * @param acceleration its profile's acceleration, a second squared: what it brakes at, too
 * @param freeSpeed its motors' free speed through its gearing, a second: what a full battery's
 *     voltage runs it at
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
   * A joint whose control gives no profile cruises at this share of its free speed: slow enough to
   * be safe on a first enable. Give it a profile to go faster.
   */
  static final double DEFAULT_CRUISE_SHARE = 0.25;

  /** ...and reaches its cruise in this long, in seconds. */
  static final double DEFAULT_RAMP_SECONDS = 0.25;

  /** One whole turn, in radians. */
  private static final double TURN = 2 * Math.PI;

  /**
   * A hair, in radians: a range given in degrees as a whole turn, or a heading on a limit's turn,
   * converts a floating-point step short of it or past it.
   */
  private static final double EDGE = 1e-9;

  /** A joint's travel, from its block and its control. */
  public static Travel of(RobotMath math, Joint joint, MechanismControl control) {
    double lower;
    double upper;
    if (control.softLimits().isPresent()) {
      lower = control.softLimits().get().lower();
      upper = control.softLimits().get().upper();
    } else if (joint.range().isPresent()) {
      lower = joint.range().get().lower();
      upper = joint.range().get().upper();
    } else {
      lower = Double.NEGATIVE_INFINITY;
      upper = Double.POSITIVE_INFINITY;
    }
    double freeSpeed = math.freeSpeed(joint);
    return new Travel(
        joint,
        lower,
        upper,
        joint instanceof Joint.Revolute && joint.range().isEmpty(),
        joint instanceof Joint.Revolute
            && joint.range().map(range -> range.width() >= TURN - EDGE).orElse(false),
        control
            .profile()
            .map(MechanismControl.Profile::cruiseVelocity)
            .orElse(DEFAULT_CRUISE_SHARE * freeSpeed),
        control
            .profile()
            .map(MechanismControl.Profile::acceleration)
            .orElse(DEFAULT_CRUISE_SHARE * freeSpeed / DEFAULT_RAMP_SECONDS),
        freeSpeed);
  }

  /** A position inside its limits. */
  public double clamp(double position) {
    return Math.max(lower, Math.min(upper, position));
  }

  /**
   * A position as a goal from a point: inside its limits, and for a joint that turns freely, the
   * nearest turn of it, the short way round.
   */
  public double goalNear(double position, double from) {
    double inside = clamp(position);
    return wraps ? from + MathUtil.angleModulus(inside - from) : inside;
  }

  /**
   * Where a computed angle takes the joint from a point. For a joint whose range spans a whole turn
   * or more, a turret's, the angle is a heading: of the angle plus or minus whole turns, the one
   * inside its limits nearest the point, so a turret at 170° asked for -170° goes on to 190°, and
   * one asked for a heading it can reach only the long way round goes the long way. With none
   * inside, which only soft limits narrower than a turn leave, whichever limit is nearer the
   * heading, either way round: a move there is refused first ({@link #whyUnreachable}), and a chase
   * holds there. Any other joint takes the angle as it is.
   */
  public double turnNear(double angle, double from) {
    if (!choosesTurn) {
      return angle;
    }
    double inside = turnInside(angle, from);
    if (!Double.isNaN(inside)) {
      return inside;
    }
    return Math.abs(MathUtil.angleModulus(angle - lower))
            <= Math.abs(MathUtil.angleModulus(angle - upper))
        ? lower
        : upper;
  }

  /** The turn of an angle inside its limits nearest a point, or NaN for none. */
  private double turnInside(double angle, double from) {
    // Its lowest turn at or above the lower limit, then one turn up at a time, as many as the
    // limits span: a count, so an angle far too big to turn by steps still ends.
    double lowest = angle + TURN * Math.ceil((lower - angle) / TURN - EDGE);
    int spanned = (int) ((upper - lower) / TURN) + 1;
    double nearest = Double.NaN;
    for (int turns = 0; turns <= spanned; turns++) {
      double turn = lowest + turns * TURN;
      if (turn >= lower - EDGE
          && turn <= upper + EDGE
          && (Double.isNaN(nearest) || Math.abs(turn - from) < Math.abs(nearest - from))) {
        nearest = turn;
      }
    }
    return Double.isNaN(nearest) ? nearest : clamp(nearest);
  }

  /**
   * Why the joint can't be sent to a position from a point, or null when it can: outside its
   * limits, at every turn of it for a joint whose range spans a turn or more.
   */
  public @Nullable String whyUnreachable(double position, double from) {
    if (position >= lower && position <= upper) {
      return null;
    }
    if (choosesTurn && !Double.isNaN(turnInside(position, from))) {
      return null;
    }
    return joint.shown(position)
        + (choosesTurn ? ", at every turn of it, is" : " is")
        + " outside "
        + possessive(joint.label())
        + " limits, "
        + joint.shown(lower)
        + " to "
        + joint.shown(upper);
  }

  /**
   * Where the joint comes to a stop from a point and a speed, braking at its acceleration, inside
   * its limits.
   */
  public double stoppingPoint(double at, double velocity) {
    return clamp(at + velocity * Math.abs(velocity) / (2 * acceleration));
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
