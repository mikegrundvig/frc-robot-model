package com.michaelgrundvig.frc.world.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Whether the values of a joint's control that were given with units are of the joint's kind: an
 * angle for a joint that turns, a length for one that slides. A value given with its unit remembers
 * which it was ({@code Degrees.of(5)} is an angle), so an elevator's soft limits in degrees, or an
 * arm's kP per inch, can be refused where the joint is built.
 *
 * <p>A value given as a plain number has no kind and is taken to be in the joint's units, so it is
 * never refused here.
 */
final class ControlKindChecks {
  private ControlKindChecks() {}

  /**
   * Why a control does not fit a joint of one kind, if it does not: one reason for each of its
   * values that is of the other kind, joined into one message.
   *
   * @param joint the kind the joint is
   */
  static Optional<String> whyOfTheOtherKind(MechanismControl control, Position.Kind joint) {
    List<String> reasons = new ArrayList<>();
    if (isOfTheOtherKind(control.gains().kind(), joint)) {
      reasons.add(gainsAreOfTheOtherKind(joint));
    }
    if (isOfTheOtherKind(control.softLimits().flatMap(MechanismControl.SoftLimits::kind), joint)) {
      reasons.add(softLimitsAreOfTheOtherKind(joint));
    }
    if (isOfTheOtherKind(control.profile().flatMap(MechanismControl.Profile::kind), joint)) {
      reasons.add(profileIsOfTheOtherKind(joint));
    }
    if (isOfTheOtherKind(control.bands().flatMap(MechanismControl.Bands::kind), joint)) {
      reasons.add(bandsAreOfTheOtherKind(joint));
    }
    if (isOfTheOtherKind(control.encoderGap().map(MechanismControl.EncoderGap::kind), joint)) {
      reasons.add(encoderGapIsOfTheOtherKind(joint));
    }
    if (reasons.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(String.join("; ", reasons));
  }

  /** Whether a value was given with a unit, and the unit is not of the joint's kind. */
  private static boolean isOfTheOtherKind(Optional<Position.Kind> given, Position.Kind joint) {
    return given.isPresent() && given.get() != joint;
  }

  private static String gainsAreOfTheOtherKind(Position.Kind joint) {
    return switch (joint) {
      case ANGLE ->
          "its gains are per length, and the joint turns: give them per angle,"
              + " Volts.per(Rotation)";
      case LENGTH ->
          "its gains are per angle, and the joint slides: give them per length,"
              + " Volts.per(Meter) or Volts.per(Inch); a gain per turn of its drum is that per"
              + " the drum's circumference";
    };
  }

  private static String softLimitsAreOfTheOtherKind(Position.Kind joint) {
    return switch (joint) {
      case ANGLE ->
          "its soft limits are lengths, and the joint turns: give them in angles,"
              + " Range.degrees(...)";
      case LENGTH ->
          "its soft limits are angles, and the joint slides: give them in lengths,"
              + " Range.meters(...) or Range.of(Inches.of(...), Inches.of(...))";
    };
  }

  private static String profileIsOfTheOtherKind(Position.Kind joint) {
    return switch (joint) {
      case ANGLE ->
          "its profile is in lengths, and the joint turns: give it in angles,"
              + " DegreesPerSecond.of(...) and DegreesPerSecondPerSecond.of(...)";
      case LENGTH ->
          "its profile is in angles, and the joint slides: give it in lengths,"
              + " InchesPerSecond.of(...) and InchesPerSecondPerSecond.of(...)";
    };
  }

  private static String bandsAreOfTheOtherKind(Position.Kind joint) {
    return switch (joint) {
      case ANGLE ->
          "its bands are lengths, and the joint turns: give them in angles, Degrees.of(...)";
      case LENGTH ->
          "its bands are angles, and the joint slides: give them in lengths, Inches.of(...)";
    };
  }

  private static String encoderGapIsOfTheOtherKind(Position.Kind joint) {
    return switch (joint) {
      case ANGLE ->
          "its encoder gap is a length, and the joint turns: give it in an angle,"
              + " withEncoderGap(Degrees.of(...))";
      case LENGTH ->
          "its encoder gap is an angle, and the joint slides: give it in a length,"
              + " withEncoderGap(Inches.of(...))";
    };
  }
}
