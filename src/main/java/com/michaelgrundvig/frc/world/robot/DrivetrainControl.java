package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.atMost;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.List;
import java.util.Optional;
import org.wpilib.units.Measure;
import org.wpilib.units.PerUnit;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.Time;

/**
 * How the swerve drivetrain is driven: its current and speed limits, and its control gains, all
 * tunable on a running robot (the robot program's {@code Drive.tuning}). What the drivetrain is
 * (its modules, its motors, where its wheels sit) is its geometry, in the robot's definition; this
 * is what the pit dials in on top of it.
 *
 * <p>It starts from {@link #defaults()}, and a robot says only what differs, one value a call:
 * {@code DRIVETRAIN.control(DrivetrainControl.defaults().withDriveStatorLimit(Amps.of(60)))}. The
 * drive's kV is seeded from its modules, as a joint's is from its definition, until it is tuned,
 * and so is its top speed, until it is given ({@link #TOP_SPEED_SHARE}).
 *
 * <p>The gains are held in the drivetrain's own units: volts per meter per second of the robot's
 * speed for the drive, volts per radian for the steering. A drive gain may be a plain number in its
 * unit, or given with its unit. A steering gain is always given with its unit, so a plain number
 * read per radian can't be mistaken for Tuner X's, per turn: {@code
 * withSteerKp(Volts.per(Rotation).of(100))} is Tuner X's stock steering kP, per turn of the module,
 * and is held as 100 / 2π volts per radian; Pitwall shows it per radian.
 *
 * @param limits current and speed limits
 * @param gains control gains
 */
@ConfigRecord
public record DrivetrainControl(Limits limits, SwerveGains gains) {
  /**
   * The share of its modules' free speed a drivetrain's top speed is seeded with, where its control
   * doesn't give one: four fifths. A drivetrain on carpet reaches about that, its free speed less
   * friction and the battery's sag under load (the design calculators' usual allowance), and a
   * velocity loop needs the rest to correct with.
   */
  public static final double TOP_SPEED_SHARE = 0.8;

  /**
   * The template's drivetrain control: its current limits, with Phoenix's own step-down written out
   * ({@link Limits#defaults()}); its top speeds, the linear one seeded from the modules; and its
   * gains, the drive's kV seeded from the modules and the drive moves' gains the template's ({@link
   * SwerveGains#defaults()}). Change any one value after.
   */
  public static DrivetrainControl defaults() {
    return DrivetrainControlBuilder.builder()
        .limits(Limits.defaults())
        .gains(SwerveGains.defaults())
        .build();
  }

  /** The same, each drive motor's torque capped at this current. */
  public DrivetrainControl withDriveStatorLimit(Current stator) {
    return with(limitsWith().driveStatorCurrent(stator).build());
  }

  /**
   * The same, what each drive motor draws from the battery capped at this current, for at most its
   * {@code driveSupplyLowerTime} at a time, then at its {@code driveSupplyLowerCurrent} ({@link
   * Limits} says how the step-down works).
   */
  public DrivetrainControl withDriveSupplyLimit(Current supply) {
    return with(limitsWith().driveSupplyCurrent(supply).build());
  }

  /** The same, the drive's supply limit stepping down to this current. */
  public DrivetrainControl withDriveSupplyLowerLimit(Current lower) {
    return with(limitsWith().driveSupplyLowerCurrent(lower).build());
  }

  /** The same, the drive's supply limit stepping down after this long at it. */
  public DrivetrainControl withDriveSupplyLowerTime(Time lowerTime) {
    return with(limitsWith().driveSupplyLowerTime(lowerTime).build());
  }

  /** The same, each steering motor's current capped at this. */
  public DrivetrainControl withSteerStatorLimit(Current stator) {
    return with(limitsWith().steerStatorCurrent(stator).build());
  }

  /** The same, the driver able to ask the robot to move at most this fast. */
  public DrivetrainControl withMaxSpeed(LinearVelocity linear) {
    return with(limitsWith().maxLinearSpeed(linear).build());
  }

  /** The same, the driver able to ask the robot to spin at most this fast. */
  public DrivetrainControl withMaxTurnRate(AngularVelocity angular) {
    return with(limitsWith().maxAngularSpeed(angular).build());
  }

  /** The same, with the drive's kS: the volts that overcome static friction. */
  public DrivetrainControl withDriveKs(double kS) {
    return with(gainsWith().driveKs(kS).build());
  }

  /**
   * The same, with the drive's kV in place of the modules' seed: volts per meter per second of the
   * robot's speed.
   */
  public DrivetrainControl withDriveKv(double kV) {
    return with(gainsWith().driveKv(kV).build());
  }

  /**
   * The same, the drive's kV given with its unit: per a speed along the ground, {@code
   * withDriveKv(Volts.per(FeetPerSecond).of(0.75))}.
   *
   * @throws IllegalArgumentException for a kV per anything but a speed along the ground
   */
  public DrivetrainControl withDriveKv(Measure<? extends PerUnit<VoltageUnit, ?>> kV) {
    return withDriveKv(perGroundSpeed("driveKv", kV));
  }

  /** The same, with the drive's kP: volts per meter per second of error in the robot's speed. */
  public DrivetrainControl withDriveKp(double kP) {
    return with(gainsWith().driveKp(kP).build());
  }

  /**
   * The same, the drive's kP given with its unit: per a speed along the ground, {@code
   * withDriveKp(Volts.per(FeetPerSecond).of(0.6))}.
   *
   * @throws IllegalArgumentException for a kP per anything but a speed along the ground
   */
  public DrivetrainControl withDriveKp(Measure<? extends PerUnit<VoltageUnit, ?>> kP) {
    return withDriveKp(perGroundSpeed("driveKp", kP));
  }

  /**
   * The same, with the steering's kP: volts per angle of error in the module's angle, given with
   * its unit. Tuner X shows it per turn of the module, {@code
   * withSteerKp(Volts.per(Rotation).of(100))}; {@code Volts.per(Radian)} is the drivetrain's own
   * unit. There is no bare-number form: a plain 100 would be per radian, 2π times as hot as Tuner
   * X's 100, so it can't be written.
   *
   * @throws IllegalArgumentException for a kP per anything but an angle
   */
  public DrivetrainControl withSteerKp(Measure<? extends PerUnit<VoltageUnit, ?>> kP) {
    return with(
        gainsWith().steerKp(perModuleTurning("steerKp", kP, GainUnits.Per.POSITION)).build());
  }

  /**
   * The same, with the steering's kD: volts per speed of the module's turning, given with its unit.
   * Tuner X shows it per turn per second of the module, {@code
   * withSteerKd(Volts.per(RotationsPerSecond).of(0.5))}; {@code Volts.per(RadiansPerSecond)} is the
   * drivetrain's own unit. As for the kP, there is no bare-number form.
   *
   * @throws IllegalArgumentException for a kD per anything but a speed of turning
   */
  public DrivetrainControl withSteerKd(Measure<? extends PerUnit<VoltageUnit, ?>> kD) {
    return with(gainsWith().steerKd(perModuleTurning("steerKd", kD, GainUnits.Per.SPEED)).build());
  }

  /** The same, field-relative driving aiming at the heading the robot will have this far ahead. */
  public DrivetrainControl withSkewLookahead(Time lookahead) {
    return with(gainsWith().skewLookahead(lookahead).build());
  }

  /**
   * The same, a drive move turning the robot back onto its heading this hard: radians a second for
   * each radian it's off.
   */
  public DrivetrainControl withHeadingKp(double kP) {
    return with(gainsWith().headingKp(kP).build());
  }

  /**
   * The same, a drive move driving the robot back onto its path this hard: meters a second for each
   * meter it's off.
   */
  public DrivetrainControl withTranslationKp(double kP) {
    return with(gainsWith().translationKp(kP).build());
  }

  /**
   * The same, its drive kV the modules' ideal when none was tuned, and its top speed the seed when
   * none was given. The drivetrain seeds them as it takes this control, so the program always
   * drives on both.
   *
   * @param idealDriveKv the modules' ideal, in volts per meter per second
   * @param topSpeed {@link #TOP_SPEED_SHARE} of the modules' free speed
   */
  DrivetrainControl seeded(double idealDriveKv, LinearVelocity topSpeed) {
    DrivetrainControl seeded = this;
    if (gains.driveKv().isEmpty()) {
      seeded = seeded.withDriveKv(idealDriveKv);
    }
    if (limits.maxLinearSpeed().isEmpty()) {
      seeded = seeded.withMaxSpeed(topSpeed);
    }
    return seeded;
  }

  /**
   * Why the top speed asked for is past what this drivetrain can reach, if it is. Velocity control
   * needs room under the free speed to correct with. The robot's definition checks the speed as it
   * is committed, and the range the drivetrain's limits can be tuned over in the pit ends at it.
   */
  public Optional<String> whyTooFastFor(Drivetrain drivetrain) {
    LinearVelocity freeSpeed = drivetrain.freeSpeed();
    if (limits.maxLinearSpeed().isEmpty()) {
      return Optional.empty();
    }
    LinearVelocity asked = limits.maxLinearSpeed().get();
    if (asked.lte(freeSpeed)) {
      return Optional.empty();
    }
    return Optional.of(
        shown(asked)
            + " is faster than the drivetrain's free speed of "
            + shown(freeSpeed)
            + ": give withMaxSpeed at most that");
  }

  private DrivetrainControlLimitsBuilder limitsWith() {
    return DrivetrainControlLimitsBuilder.builder(limits);
  }

  private DrivetrainControlSwerveGainsBuilder gainsWith() {
    return DrivetrainControlSwerveGainsBuilder.builder(gains);
  }

  private DrivetrainControl with(Limits changed) {
    return DrivetrainControlBuilder.builder(this).limits(changed).build();
  }

  private DrivetrainControl with(SwerveGains changed) {
    return DrivetrainControlBuilder.builder(this).gains(changed).build();
  }

  /**
   * A drive gain given with its unit, in volts per meter per second. A drive gain is per the
   * robot's speed along the ground. One per turn of a wheel or of a rotor, as Tuner X holds a drive
   * motor's, depends on the gearing and the wheel, which a unit can't say.
   *
   * @throws IllegalArgumentException for one per anything but a speed along the ground
   */
  private static double perGroundSpeed(
      String gain, Measure<? extends PerUnit<VoltageUnit, ?>> value) {
    Position.Kind kind =
        GainUnits.kindOf("SwerveGains." + gain, value, GainUnits.Per.SPEED, Optional.empty());
    if (kind == Position.Kind.LENGTH) {
      return value.baseUnitMagnitude();
    }
    throw new IllegalArgumentException(
        "SwerveGains."
            + gain
            + ": it's per a speed of turning, and a drive gain is per the robot's speed along the"
            + " ground: give it per Volts.per(MetersPerSecond), or as a plain number in volts per"
            + " meter per second");
  }

  /**
   * A steering gain given with its unit, in volts per radian (or per radian per second). A steering
   * gain is per the module's turning: Tuner X's are per turn of the module.
   *
   * @param takes a position for the kP, a speed for the kD
   * @throws IllegalArgumentException for one per anything but the module's turning
   */
  private static double perModuleTurning(
      String gain, Measure<? extends PerUnit<VoltageUnit, ?>> value, GainUnits.Per takes) {
    Position.Kind kind = GainUnits.kindOf("SwerveGains." + gain, value, takes, Optional.empty());
    if (kind == Position.Kind.ANGLE) {
      return value.baseUnitMagnitude();
    }
    throw new IllegalArgumentException(
        "SwerveGains."
            + gain
            + ": it's per a length, and the steering turns: give it per turn of the module, as"
            + " Tuner X shows it, Volts.per(Rotation)");
  }

  /**
   * Current limits are the main defense against wheel slip (more torque than the carpet can take)
   * and brownouts (more current than the battery can give). Speed limits cap what the driver can
   * ask for.
   *
   * <p>The drive's supply limit steps down. Phoenix 6 lets a drive motor draw up to {@code
   * driveSupplyCurrent}, but once it has been held at that limit for {@code driveSupplyLowerTime},
   * it lowers the limit to {@code driveSupplyLowerCurrent} until the motor draws less than that
   * again. The step-down keeps the motor's breaker from tripping in a long push: a 40 A breaker
   * carries 40 A for as long as you like, but 60 A for as little as 3.9 seconds. Phoenix steps down
   * even when the code never mentions it (to 40 A after 1 s), so the template's defaults are
   * Phoenix's, set where they can be seen. A robot that loses its push after a second of hard
   * driving is seeing the step-down.
   *
   * <p>All are tunable on a running robot (the drivetrain says within what): dial them in on the
   * robot, then commit what works.
   *
   * @param driveStatorCurrent the most current through each drive motor, which caps its torque
   * @param driveSupplyCurrent the most current each drive motor draws from the battery, for at most
   *     driveSupplyLowerTime at a time: then the limit steps down to driveSupplyLowerCurrent
   * @param driveSupplyLowerCurrent each drive motor's supply limit after the step-down, until it
   *     draws less than this again: no more than its breaker's rating. With driveSupplyCurrent at
   *     or below it, there's nothing to step down from, and it does nothing.
   * @param driveSupplyLowerTime how long a drive motor may draw driveSupplyCurrent before its limit
   *     steps down to driveSupplyLowerCurrent: longer than a sprint, and well short of the time its
   *     breaker takes to trip
   * @param steerStatorCurrent the most current through each steer motor
   * @param maxLinearSpeed the fastest the driver can ask the robot to move; none given, the
   *     drivetrain seeds it with {@link #TOP_SPEED_SHARE} of its modules' free speed as it takes
   *     this control, so a drivetrain's control always has one
   * @param maxAngularSpeed the fastest the driver can ask the robot to spin
   */
  @ConfigRecord
  public record Limits(
      Current driveStatorCurrent,
      Current driveSupplyCurrent,
      Current driveSupplyLowerCurrent,
      Time driveSupplyLowerTime,
      Current steerStatorCurrent,
      Optional<LinearVelocity> maxLinearSpeed,
      AngularVelocity maxAngularSpeed) {
    /**
     * What {@link DrivetrainControl#defaults()} starts from: 30 A through each drive motor, 60 A
     * from the battery stepping down to 40 A after 1 s (Phoenix's own step-down), 40 A through each
     * steering motor, and a turn a second at the driver's full stick. Its top speed is the
     * drivetrain's to seed.
     */
    public static Limits defaults() {
      return DrivetrainControlLimitsBuilder.builder()
          .driveStatorCurrent(Amps.of(30))
          .driveSupplyCurrent(Amps.of(60))
          .driveSupplyLowerCurrent(Amps.of(40))
          .driveSupplyLowerTime(Seconds.of(1))
          .steerStatorCurrent(Amps.of(40))
          .maxAngularSpeed(RotationsPerSecond.of(1))
          .build();
    }

    public Limits {
      Checks.on(Limits.class)
          .field("driveStatorCurrent", driveStatorCurrent, positive())
          .field("driveSupplyCurrent", driveSupplyCurrent, positive())
          .field("driveSupplyLowerCurrent", driveSupplyLowerCurrent, positive())
          // Phoenix reads a zero time as "never step down": to mean that, set the lower current to
          // the supply limit instead, where it shows. 5 s is the longest Phoenix takes.
          .field("driveSupplyLowerTime", driveSupplyLowerTime, positive())
          .field("driveSupplyLowerTime", driveSupplyLowerTime, atMost(Seconds.of(5)))
          .field("steerStatorCurrent", steerStatorCurrent, positive())
          .field("maxLinearSpeed", maxLinearSpeed, given(positive()))
          .field("maxAngularSpeed", maxAngularSpeed, positive())
          .orThrow();
    }
  }

  /**
   * Drive: velocity control on the motor controller, with feedforward from robot code. Steer:
   * position control on the motor controller. Drive moves: a path for the robot to follow, and
   * feedback that brings it back onto the path.
   *
   * <p>All are tunable on a running robot (the drivetrain says within what): dial them in on the
   * robot, then commit what works.
   *
   * @param driveKs volts to overcome static friction: measured, never seeded
   * @param driveKv volts per meter per second of the robot's speed, where tuned; empty for the
   *     modules' ideal, which the drivetrain fills in as it takes its control ({@code
   *     Module.idealDriveKv()}), so a drivetrain's control always has one
   * @param driveKp volts per meter per second of error
   * @param steerKp volts per radian of error: Tuner X's, per turn of the module, over 2π
   * @param steerKd volts per radian per second: Tuner X's, per turn a second, over 2π
   * @param skewLookahead field-relative commands use the heading the robot will have this far
   *     ahead, because the wheels take time to turn while the robot spins
   * @param headingKp how hard a drive move turns the robot back onto its profile's heading: radians
   *     a second for each radian it's off
   * @param translationKp how hard a drive move drives the robot back onto its path's position:
   *     meters a second for each meter it's off
   */
  @ConfigRecord
  public record SwerveGains(
      double driveKs,
      Optional<Double> driveKv,
      double driveKp,
      double steerKp,
      double steerKd,
      Time skewLookahead,
      double headingKp,
      double translationKp) {
    /**
     * What {@link DrivetrainControl#defaults()} starts from: no static friction, the drive's kV
     * seeded, a drive kP of 2, Tuner X's stock steering gains (100 V per turn and 0.5 V per turn a
     * second, to two figures: 16 and 0.08 per radian), and the drive moves' gains: a 0.03 s skew
     * lookahead, a heading kP of 5 and a translation kP of 3.
     */
    public static SwerveGains defaults() {
      return DrivetrainControlSwerveGainsBuilder.builder()
          .driveKs(0)
          .driveKp(2.0)
          .steerKp(16.0)
          .steerKd(0.08)
          .skewLookahead(Seconds.of(0.03))
          .headingKp(5.0)
          .translationKp(3.0)
          .build();
    }

    public SwerveGains {
      Checks.on(SwerveGains.class)
          .field("driveKs", driveKs, nonNegative())
          .field("driveKv", driveKv, given(nonNegative()))
          .field("driveKp", driveKp, nonNegative())
          .field("steerKp", steerKp, nonNegative())
          .field("steerKd", steerKd, nonNegative())
          .field("skewLookahead", skewLookahead, nonNegative())
          .field("headingKp", headingKp, nonNegative())
          .field("translationKp", translationKp, nonNegative())
          .orThrow();
    }
  }

  /**
   * How fast the robot can speed up, and spin up, by its own model: each wheel pushes with the
   * lesser of its motors at the drive's stator current limit, through its gearing, and its tread's
   * grip on its share of the robot's weight; that, over the robot's mass, and its yaw inertia about
   * the wheels, is what the drivetrain can do. Drive moves are profiled under it.
   *
   * @param linear in meters a second squared
   * @param angular in radians a second squared
   */
  public record Accelerations(double linear, double angular) {}

  /**
   * The robot's accelerations by its model, with this drivetrain's current limit.
   *
   * @param robotMass the whole robot's mass: as weighed, or until it is, the one taken in its place
   *     ({@code SeasonLimits.robotMass})
   */
  public static Accelerations accelerations(
      RobotDefinition robot, Mass robotMass, DrivetrainControl control) {
    List<Drivetrain.Corner> corners = robot.drivetrain().corners();
    double mass = robotMass.in(Kilograms);
    double amps = control.limits().driveStatorCurrent().in(Amps);
    double force = 0;
    double torque = 0;
    for (Drivetrain.Corner corner : corners) {
      Drivetrain.Module module = corner.module();
      Actuator.MotorGroup motors = module.driveMotors(robot.drivetrain().driveCommutation());
      double pushed =
          motors.count()
              * motors.curve().model().Kt
              * amps
              * module.driveGearing().ratio()
              * module.driveGearing().efficiency()
              / module.wheelRadius().in(Meters);
      double gripped =
          module.wheelCoefficientOfFrictionOrDefault() * mass * RobotMath.GRAVITY / corners.size();
      double wheel = Math.min(pushed, gripped);
      force += wheel;
      torque += wheel * corner.position().getNorm();
    }
    return new Accelerations(
        force / mass, torque / robot.body().yawMomentOfInertia(robotMass).in(KilogramSquareMeters));
  }
}
