package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

/**
 * The definition's checks of what each joint is asked to run at, against what its motors can do:
 * its named speeds and its profile's cruise against its free speed, and its named voltages against
 * its motors' nominal voltage.
 *
 * <p>Two kinds of joint are set aside first. A joint that follows another is carried by its
 * leader's moves, so it may name no places of its own and has nothing else to check. A joint with
 * no motors has no free speed to check against, and {@link #unchecked} lists those that name a
 * speed or a voltage anyway.
 */
final class JointSpeedChecks {
  private JointSpeedChecks() {}

  /**
   * A named speed above this share of the joint's free speed is doubted: a velocity loop has little
   * or nothing left to hold it there under load.
   */
  static final double NOTED_SHARE = 0.9;

  /**
   * How far past a limit a value may be and still count as at it, as a share of the limit. A speed
   * given in rpm and a free speed worked out in radians a second can differ by a rounding step.
   */
  private static final double ROUNDING = 1e-9;

  /** Adds what is wrong with each joint's names, speeds and voltages to a list of problems. */
  static void problems(RobotMath math, List<Finding> problems) {
    for (Joint joint : math.joints()) {
      Joint leader = math.leaderOf(joint);
      if (leader != null) {
        whyAFollowerNamesNothing(joint, leader)
            .ifPresent(why -> problems.add(Places.refused(joint, why)));
        continue;
      }
      if (math.drivesOf(joint).isEmpty()) {
        continue;
      }
      double freeSpeed = math.freeSpeed(joint);
      double nominalVolts = nominalVolts(math, joint);
      whyCruiseIsTooFast(joint, freeSpeed)
          .ifPresent(why -> problems.add(Places.refused(joint, why)));
      whyBandsAreOfTheOtherLoop(joint).ifPresent(why -> problems.add(Places.refused(joint, why)));
      for (Output output : joint.outputs()) {
        whyOutputIsTooMuch(joint, output, freeSpeed, nominalVolts)
            .ifPresent(why -> problems.add(Places.refused(joint, why)));
      }
    }
  }

  /**
   * The doubts about each joint's named speeds: a speed so near the joint's free speed that its
   * velocity loop cannot hold it under load.
   */
  static List<Finding> doubts(RobotMath math) {
    List<Finding> doubts = new ArrayList<>();
    for (Joint joint : math.joints()) {
      boolean hasMotorsOfItsOwn = math.leaderOf(joint) == null && !math.drivesOf(joint).isEmpty();
      if (joint.outputs().isEmpty() || !hasMotorsOfItsOwn) {
        continue;
      }
      double freeSpeed = math.freeSpeed(joint);
      for (Output output : joint.outputs()) {
        whySpeedIsDoubted(joint, output, freeSpeed)
            .ifPresent(why -> doubts.add(Finding.doubt(joint.label(), why)));
      }
    }
    return doubts;
  }

  /**
   * The joints that name a speed or a voltage and have no motors to turn them, so nothing can check
   * what they name. A joint that names only duty cycles is not listed, because a duty cycle is a
   * share of whatever the battery gives and is checked where it is written.
   */
  static List<Joint> unchecked(RobotMath math) {
    List<Joint> unchecked = new ArrayList<>();
    for (Joint joint : math.joints()) {
      boolean namesASpeedOrVoltage =
          joint.outputs().stream().anyMatch(output -> output.kind() != Output.Kind.DUTY_CYCLE);
      boolean hasNoMotors = math.leaderOf(joint) == null && math.drivesOf(joint).isEmpty();
      if (namesASpeedOrVoltage && hasNoMotors) {
        unchecked.add(joint);
      }
    }
    return unchecked;
  }

  /** Whether a value is past a limit by more than rounding. */
  private static boolean isPast(double limit, double value) {
    return value > limit * (1 + ROUNDING);
  }

  /** Why a joint that follows another may not name positions or outputs, if it names any. */
  private static Optional<String> whyAFollowerNamesNothing(Joint follower, Joint leader) {
    if (follower.positions().isEmpty() && follower.outputs().isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(
        follower.label()
            + " follows "
            + leader.label()
            + ", whose moves carry it: give its places to "
            + leader.label()
            + ", and none to "
            + follower.label());
  }

  /** Why a joint's profile cruises faster than its motors can turn it, if it does. */
  private static Optional<String> whyCruiseIsTooFast(Joint joint, double freeSpeed) {
    Optional<MechanismControl.Profile> profile =
        joint.driven().control().flatMap(MechanismControl::profile);
    if (profile.isEmpty() || !isPast(freeSpeed, profile.get().cruiseVelocity())) {
      return Optional.empty();
    }
    double cruise = profile.get().cruiseVelocity();
    return Optional.of(
        String.format(
            Locale.ROOT,
            "%s's profile cruises at %s, %s times its free speed, %s: its motors can't turn it that"
                + " fast, so its moves would lag their profile; give it a cruise under its free"
                + " speed",
            joint.label(),
            shownCruise(joint, cruise),
            shown(cruise / freeSpeed),
            shownCruise(joint, freeSpeed)));
  }

  /**
   * A cruise speed in the terms the joint's own speeds are given in. A joint that is held at
   * positions has a profile, which is given in degrees a second or meters a second. A joint that
   * only runs, a roller or a flywheel, has outputs, which are given in rpm.
   */
  private static String shownCruise(Joint joint, double speed) {
    if (!joint.holdsPositions()) {
      return joint.shownSpeed(speed);
    }
    return switch (Position.Kind.of(joint)) {
      case ANGLE -> shown(Math.toDegrees(speed)) + "°/s";
      case LENGTH -> shown(speed) + " m/s";
    };
  }

  /**
   * Why a joint's bands are not the kind its loop holds, if they are not. A joint that only runs at
   * outputs has a velocity loop, whose bands are shares of the speed it is asked for. A joint held
   * at positions has a position loop, whose bands are angles or lengths.
   */
  private static Optional<String> whyBandsAreOfTheOtherLoop(Joint joint) {
    Optional<MechanismControl.Bands> bands =
        joint.driven().control().flatMap(MechanismControl::bands);
    if (bands.isEmpty()) {
      return Optional.empty();
    }
    Optional<Position.Kind> measuredIn = bands.get().kind();
    if (measuredIn.isPresent() && !joint.holdsPositions()) {
      return Optional.of(
          joint.label()
              + " only runs at outputs, so its bands are shares of the speed it's asked for, not "
              + plural(measuredIn.get())
              + ": withBands(Percent.of(3), Percent.of(6)) is 3% to arrive and 6% to stay");
    }
    if (measuredIn.isEmpty() && joint.holdsPositions()) {
      Position.Kind kind = Position.Kind.of(joint);
      return Optional.of(
          joint.label()
              + " is held at positions, so its bands are "
              + plural(kind)
              + ", not shares of a speed: "
              + switch (kind) {
                case ANGLE -> "withBands(Degrees.of(2), Degrees.of(4)) is 2° to arrive and 4°";
                case LENGTH ->
                    "withBands(Inches.of(0.5), Inches.of(1)) is half an inch to arrive and an inch";
              }
              + " to stay");
    }
    return Optional.empty();
  }

  private static String plural(Position.Kind kind) {
    return switch (kind) {
      case ANGLE -> "angles";
      case LENGTH -> "lengths";
    };
  }

  /**
   * Why a named output asks more than the joint's motors can give, if it does. A duty cycle never
   * does: it is a share of whatever the battery gives.
   */
  private static Optional<String> whyOutputIsTooMuch(
      Joint joint, Output output, double freeSpeed, double nominalVolts) {
    return switch (output.kind()) {
      case VELOCITY -> whySpeedIsTooFast(joint, output, freeSpeed);
      case VOLTAGE -> whyVoltageIsTooHigh(joint, output, nominalVolts);
      case DUTY_CYCLE -> Optional.empty();
    };
  }

  /** Why a named speed is past the joint's free speed, if it is. */
  private static Optional<String> whySpeedIsTooFast(Joint joint, Output speed, double freeSpeed) {
    double size = Math.abs(speed.value());
    if (!isPast(freeSpeed, size)) {
      return Optional.empty();
    }
    return Optional.of(
        String.format(
            Locale.ROOT,
            "%s %s, %s, is %s times its free speed, %s: its motors can't turn it that"
                + " fast; give it a speed under that",
            possessive(joint.label()),
            speed.name(),
            joint.shownSpeed(speed.value()),
            shown(size / freeSpeed),
            joint.shownSpeed(freeSpeed)));
  }

  /** Why a named voltage is past the nominal voltage of the joint's motors, if it is. */
  private static Optional<String> whyVoltageIsTooHigh(
      Joint joint, Output voltage, double nominalVolts) {
    double size = Math.abs(voltage.value());
    if (!isPast(nominalVolts, size)) {
      return Optional.empty();
    }
    return Optional.of(
        String.format(
            Locale.ROOT,
            "%s's %s, %s V, is past its motors' nominal %s V: the battery can't give it",
            joint.label(),
            voltage.name(),
            shown(voltage.value()),
            shown(nominalVolts)));
  }

  /**
   * Why a named speed is doubted, if it is: it is within the joint's free speed ({@link #isPast}),
   * and above the share of it past which a velocity loop cannot hold it ({@link #NOTED_SHARE}).
   */
  private static Optional<String> whySpeedIsDoubted(Joint joint, Output output, double freeSpeed) {
    if (output.kind() != Output.Kind.VELOCITY) {
      return Optional.empty();
    }
    double size = Math.abs(output.value());
    double share = size / freeSpeed;
    boolean nearFreeSpeed = share > NOTED_SHARE;
    // The same question the refusal asks (whySpeedIsTooFast), so a speed is never both refused
    // and doubted, nor let past both.
    boolean withinFreeSpeed = !isPast(freeSpeed, size);
    if (nearFreeSpeed && withinFreeSpeed) {
      return Optional.of(
          String.format(
              Locale.ROOT,
              "%s's %s, %s, is %.0f%% of its free speed, %s: a velocity loop has little or"
                  + " nothing left to hold it under load",
              joint.label(),
              output.name(),
              joint.shownSpeed(output.value()),
              share * 100,
              joint.shownSpeed(freeSpeed)));
    }
    return Optional.empty();
  }

  /**
   * The most volts a named voltage may ask of a joint: the least nominal voltage among the motors
   * that move it.
   */
  private static double nominalVolts(RobotMath math, Joint joint) {
    double nominal = Double.POSITIVE_INFINITY;
    for (Drive drive : math.drivesOf(joint)) {
      nominal = Math.min(nominal, ((Actuator.Motors) drive.actuator()).model().nominalVoltage);
    }
    return nominal;
  }
}
