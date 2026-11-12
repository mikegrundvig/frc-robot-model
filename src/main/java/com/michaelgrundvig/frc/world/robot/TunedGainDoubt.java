package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

/**
 * Whether a gain that a team tuned looks as if it belongs to the mechanism the definition
 * describes. The definition works out an ideal value for a joint's kV and kG from its motors, its
 * gearing and its mass, and for a drivetrain's drive kV from its modules. Friction and losses put a
 * real gain above the ideal, but rarely past twice it. A tuned gain further off than that is more
 * likely a mistake in a unit or a ratio than a property of the mechanism, and is noted.
 *
 * <p>For a kV, the commonest mistake is a gain worked out in one frame and entered in another: per
 * turn of the joint, say, and entered as if per turn of its rotor. When a tuned kV is off by about
 * the ratio between two such frames, the note names them.
 */
final class TunedGainDoubt {
  private TunedGainDoubt() {}

  /** Meters in an inch, for a gain worked out per inch. */
  private static final double INCH = 0.0254;

  /** Meters in a foot, for a drive gain worked out per foot. */
  private static final double FOOT = 0.3048;

  /**
   * A unit a gain may have been worked out per.
   *
   * @param name the unit, as the note says it: "degree", "turn of its rotor"
   * @param perJointUnit how many of the unit make one of the joint's own, a radian or a meter
   */
  private record Frame(String name, double perJointUnit) {}

  /**
   * The doubt about a tuned kV, if there is one.
   *
   * @param tuned the tuned kV, in volts per unit a second of the joint
   * @param ideal the definition's ideal kV, in the same terms
   * @param rotorRatio the rotor's turns for each turn (or meter) of the joint
   */
  static Optional<String> ofKv(
      Joint joint, double tuned, double ideal, MechanismRotations rotations, double rotorRatio) {
    String speed = unitOf(joint) + "/s";
    Optional<String> doubt =
        whyFarFromIdeal(joint.label(), "kV", tuned, ideal, " V/(" + speed + ")");
    if (doubt.isEmpty()) {
      return doubt;
    }
    double timesTheIdeal = tuned / ideal;
    String framesMixedUp =
        whichFramesWereMixedUp(timesTheIdeal, framesFor(joint, rotations, rotorRatio)).orElse("");
    return Optional.of(doubt.get() + framesMixedUp);
  }

  /**
   * The doubt about a tuned kG, if there is one. A kG is in volts whatever the joint is, so there
   * are no frames to mix up.
   */
  static Optional<String> ofKg(Joint joint, double tuned, double ideal) {
    return whyFarFromIdeal(joint.label(), "kG", tuned, ideal, " V");
  }

  /**
   * The doubt about a drivetrain's tuned drive kV, if there is one. Its ideal is per meter per
   * second of the robot's speed. Tuner X holds a drive motor's gains per turn a second of its
   * rotor, so a kV copied from it is off by about the ratio between the two, which the note names.
   *
   * @param tuned the tuned kV, in volts per meter per second
   * @param module the module whose ideal the kV is compared with: the drivetrain's slowest
   */
  static Optional<String> ofDriveKv(double tuned, Drivetrain.Module module) {
    double ideal = module.idealDriveKv();
    Optional<String> doubt = whyFarFromIdeal("drivetrain", "driveKv", tuned, ideal, " V/(m/s)");
    if (doubt.isEmpty()) {
      return doubt;
    }
    double timesTheIdeal = tuned / ideal;
    String framesMixedUp =
        whichFramesWereMixedUp(timesTheIdeal, framesForADrive(module)).orElse("");
    return Optional.of(doubt.get() + framesMixedUp);
  }

  /**
   * Why a tuned gain looks wrong beside the ideal one, if it does: it is under half the ideal or
   * over twice it. With no ideal to compare with (it is 0 for a joint whose links have no mass)
   * nothing is doubted.
   *
   * @param label what the gain drives, as the note names it: a joint's label, or "drivetrain"
   * @param unit the gain's unit as the note shows it after a value, with its leading space
   */
  private static Optional<String> whyFarFromIdeal(
      String label, String gain, double tuned, double ideal, String unit) {
    if (ideal == 0) {
      return Optional.empty();
    }
    double timesTheIdeal = tuned / ideal;
    boolean plausible =
        timesTheIdeal >= ControllerPlan.SANITY_LOW && timesTheIdeal <= ControllerPlan.SANITY_HIGH;
    if (plausible) {
      return Optional.empty();
    }
    return Optional.of(
        String.format(
            Locale.ROOT,
            "%s's tuned %s, %s, is %s times the ideal, %s, from its motors and what it moves",
            label,
            gain,
            shown(tuned) + unit,
            shown(timesTheIdeal),
            shown(ideal) + unit));
  }

  /** The unit of a joint's position, as a gain's unit names it. */
  private static String unitOf(Joint joint) {
    return switch (Position.Kind.of(joint)) {
      case ANGLE -> "rad";
      case LENGTH -> "m";
    };
  }

  /**
   * The pair of frames that best explains how far a tuned gain is from the ideal, as the end of a
   * note, if any pair explains it. A gain worked out per one frame and entered as if per another is
   * off by the ratio between the two. Every pair is tried, and the pair whose ratio comes nearest
   * is named, provided it comes within {@link ControllerPlan#FRAME_TOLERANCE}.
   */
  private static Optional<String> whichFramesWereMixedUp(double timesTheIdeal, List<Frame> frames) {
    double nearestSoFar = ControllerPlan.FRAME_TOLERANCE;
    Optional<String> named = Optional.empty();
    for (Frame meant : frames) {
      for (Frame entered : frames) {
        if (meant.equals(entered)) {
          continue;
        }
        double ratioBetweenFrames = entered.perJointUnit() / meant.perJointUnit();
        double off = Math.abs(timesTheIdeal / ratioBetweenFrames - 1);
        if (off < nearestSoFar) {
          nearestSoFar = off;
          int withinPercent = Math.max(1, (int) Math.ceil(off * 100));
          named =
              Optional.of(
                  String.format(
                      Locale.ROOT,
                      ": the ideal per %s, within %d%%, entered as if per %s, by the look of it",
                      meant.name(),
                      withinPercent,
                      entered.name()));
        }
      }
    }
    return named;
  }

  /**
   * The frames a drive gain is commonly worked out in: per meter or per foot of the robot's travel,
   * per turn of its wheel, or per turn of its drive motor's rotor, as Tuner X holds it.
   */
  private static List<Frame> framesForADrive(Drivetrain.Module module) {
    double wheelTurnsPerMeter = 1 / (2 * Math.PI * module.wheelRadius().in(Meters));
    return List.of(
        new Frame("meter", 1),
        new Frame("foot", 1 / FOOT),
        new Frame("turn of its wheel", wheelTurnsPerMeter),
        new Frame("turn of its rotor", module.driveGearing().ratio() * wheelTurnsPerMeter));
  }

  /** The frames a gain for a joint is commonly worked out in. */
  private static List<Frame> framesFor(
      Joint joint, MechanismRotations rotations, double rotorRatio) {
    if (joint instanceof RevoluteJoint) {
      return List.of(
          new Frame("radian", 1),
          new Frame("degree", 180 / Math.PI),
          new Frame("turn of " + joint.label(), 1 / (2 * Math.PI)),
          new Frame("turn of its rotor", rotorRatio / (2 * Math.PI)));
    }
    List<Frame> frames = new ArrayList<>();
    frames.add(new Frame("meter", 1));
    frames.add(new Frame("inch", 1 / INCH));
    frames.add(new Frame("turn of its rotor", rotorRatio));
    // A drum that winds exactly a meter each turn would only repeat the meter's frame.
    boolean drumTurnIsNotAMeter = rotations.perRotation() != 1;
    if (drumTurnIsNotAMeter) {
      frames.add(new Frame("turn of its drum", 1 / rotations.perRotation()));
    }
    return frames;
  }
}
