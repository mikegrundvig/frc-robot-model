package com.michaelgrundvig.frc.world.robot;

import java.util.Optional;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.DistanceUnit;
import org.wpilib.units.LinearAccelerationUnit;
import org.wpilib.units.LinearVelocityUnit;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.Unit;
import org.wpilib.units.VoltageUnit;

/**
 * What a gain that was given with its unit is "per", and whether that is something the gain can be
 * per. {@code Volts.per(Rotation).of(188)} is volts per an angle, which a position loop's kP can
 * be; {@code Volts.per(Second)} is not something any gain is per.
 *
 * <p>Two things are read from the unit under the "per". One is what the gain is measured against
 * ({@link Per}): a position, a speed or an acceleration. Each gain takes exactly one of those, and
 * for kP and kD which one depends on the control's loop. The other is whether that position, speed
 * or acceleration is of an angle or of a length ({@link Position.Kind}), which has to be the kind
 * of joint the control ends up driving.
 */
final class GainUnits {
  private GainUnits() {}

  /** What a gain is measured against. */
  enum Per {
    /** A position: an angle or a length. */
    POSITION("an angle, Volts.per(Rotation), or a length, Volts.per(Meter)"),
    /** A speed of turning or of sliding. */
    SPEED("a speed, Volts.per(RotationsPerSecond) or Volts.per(MetersPerSecond)"),
    /** An acceleration of turning or of sliding. */
    ACCELERATION(
        "an acceleration, Volts.per(RotationsPerSecondPerSecond) or"
            + " Volts.per(MetersPerSecondPerSecond)");

    /** What it is and how to write a gain per it, as a refusal says. */
    private final String howToGiveIt;

    Per(String howToGiveIt) {
      this.howToGiveIt = howToGiveIt;
    }

    /**
     * What a loop's proportional gain is per: the error it acts on. A position loop's error is a
     * position, and a velocity loop's is a speed.
     */
    static Per ofKp(MechanismControl.Loop loop) {
      return switch (loop) {
        case POSITION -> POSITION;
        case VELOCITY -> SPEED;
      };
    }

    /**
     * What a loop's derivative gain is per: how fast its error changes. A position loop's error
     * changes at a speed, and a velocity loop's at an acceleration.
     */
    static Per ofKd(MechanismControl.Loop loop) {
      return switch (loop) {
        case POSITION -> SPEED;
        case VELOCITY -> ACCELERATION;
      };
    }
  }

  /**
   * What a unit measures, as far as a gain is concerned.
   *
   * @param kind whether it is of an angle or of a length
   * @param per whether it is a position, a speed or an acceleration
   */
  private record Measures(Position.Kind kind, Per per) {}

  /** What a unit measures, if it is something a gain can be per. A time or a current is not. */
  private static Optional<Measures> whatItMeasures(Unit unit) {
    return switch (unit) {
      case AngleUnit _ -> Optional.of(new Measures(Position.Kind.ANGLE, Per.POSITION));
      case DistanceUnit _ -> Optional.of(new Measures(Position.Kind.LENGTH, Per.POSITION));
      case AngularVelocityUnit _ -> Optional.of(new Measures(Position.Kind.ANGLE, Per.SPEED));
      case LinearVelocityUnit _ -> Optional.of(new Measures(Position.Kind.LENGTH, Per.SPEED));
      case AngularAccelerationUnit _ ->
          Optional.of(new Measures(Position.Kind.ANGLE, Per.ACCELERATION));
      case LinearAccelerationUnit _ ->
          Optional.of(new Measures(Position.Kind.LENGTH, Per.ACCELERATION));
      default -> Optional.empty();
    };
  }

  /**
   * Whether a gain given with its unit is per an angle or per a length.
   *
   * @param gain the gain's name, as the refusal says it: "kP"
   * @param takes what this gain has to be per
   * @param loopThatDecides the control's loop, when what the gain takes depends on it, so that the
   *     refusal can say so. It is empty for a gain that takes the same in either loop.
   * @throws IllegalArgumentException if the gain is per anything but what it takes
   */
  static Position.Kind kindOf(
      String gain,
      Measure<? extends PerUnit<VoltageUnit, ?>> value,
      Per takes,
      Optional<MechanismControl.Loop> loopThatDecides) {
    Unit unit = value.unit().denominator();
    Optional<Measures> measures = whatItMeasures(unit);
    if (measures.isPresent() && measures.get().per() == takes) {
      return measures.get().kind();
    }
    String inWhichControl =
        loopThatDecides.map(loop -> " in a control for " + loop.held()).orElse("");
    throw new IllegalArgumentException(
        "Gains."
            + gain
            + ": volts per "
            + unit.name()
            + " can't be one"
            + inWhichControl
            + ": give it per "
            + takes.howToGiveIt);
  }

  /**
   * The same, for a gain added to gains that may already have been given with units. A control
   * drives one joint, so its gains cannot be per an angle and per a length at once.
   *
   * @param otherGains the kind the control's other gains were given in, if any were given with
   *     units
   * @throws IllegalArgumentException if the gain is per anything but what it takes, or per the
   *     other kind than the other gains
   */
  static Position.Kind kindAgreeingWith(
      Optional<Position.Kind> otherGains,
      String gain,
      Measure<? extends PerUnit<VoltageUnit, ?>> value,
      Per takes,
      Optional<MechanismControl.Loop> loopThatDecides) {
    Position.Kind kind = kindOf(gain, value, takes, loopThatDecides);
    boolean agrees = otherGains.isEmpty() || otherGains.get() == kind;
    if (agrees) {
      return kind;
    }
    throw new IllegalArgumentException(
        "Gains."
            + gain
            + ": it's per "
            + aOrAn(kind)
            + ", and the other gains are per "
            + aOrAn(otherGains.get())
            + ": a control drives one joint, which turns or slides");
  }

  private static String aOrAn(Position.Kind kind) {
    return switch (kind) {
      case ANGLE -> "an angle";
      case LENGTH -> "a length";
    };
  }
}
