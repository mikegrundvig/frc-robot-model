package com.michaelgrundvig.frc.world.robot;

import org.jspecify.annotations.Nullable;

/**
 * Whether a joint's limit switch reads as it should, given where the joint is: tripped nowhere near
 * where it trips (a cut wire, if it's normally closed), or clear well past it (a cut wire, if it's
 * normally open). How long a switch must disagree before the robot says so is the robot program's,
 * which reads a clock; this judges one reading.
 */
public final class SwitchDoubt {
  private SwitchDoubt() {}

  /** The margin for a joint that turns freely, which has no range to take a tenth of: 15°. */
  private static final double MARGIN_TURNING_FREELY = Math.toRadians(15);

  /** The share of a joint's range that is its margin: a tenth. */
  private static final double MARGIN_SHARE_OF_RANGE = 0.1;

  /**
   * How far from where a limit switch trips its joint must be, past it or short of it, before the
   * switch's reading is doubted: a tenth of the joint's range, or 15° for one that turns freely. A
   * guess at what a switch bent a little, or flex in the mechanism, never reaches.
   *
   * <p>The robot asks this every loop for every joint with a limit switch, so it makes no objects.
   */
  static double margin(Joint joint) {
    if (joint.range().isEmpty()) {
      return MARGIN_TURNING_FREELY;
    }
    return MARGIN_SHARE_OF_RANGE * joint.range().get().width();
  }

  /**
   * What a limit switch's reading is doubted for with the joint where it is, in a sentence saying
   * what to check; null when it agrees, or when its position isn't known.
   *
   * @param joint the joint, which says where it is in its units
   * @param limit one of its limit switches
   * @param at where the joint is
   * @param tripped whether the switch reads tripped
   */
  public static @Nullable String of(Joint joint, Sensor.Switch limit, double at, boolean tripped) {
    Sensor.Switch.Limit kind = (Sensor.Switch.Limit) limit.kind();
    if (kind.position().isEmpty()) {
      return null;
    }
    double trips = kind.position().get();
    double toward = kind.end() == Sensor.Switch.Limit.End.LOWER ? -1 : 1;
    double past = (at - trips) * toward;
    double margin = margin(joint);
    if (tripped && past < -margin) {
      return limit.label()
          + " reads tripped with "
          + joint.label()
          + " at "
          + joint.shown(at)
          + ", nowhere near where it trips, "
          + joint.shown(trips)
          + ": a cut wire, if it's normally closed, or a switch out of place";
    }
    if (!tripped && past > margin) {
      return limit.label()
          + " reads clear with "
          + joint.label()
          + " at "
          + joint.shown(at)
          + ", past where it trips, "
          + joint.shown(trips)
          + ": a cut wire, if it's normally open, or a switch out of place";
    }
    return null;
  }
}
