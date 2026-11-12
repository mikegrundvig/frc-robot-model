package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degree;
import static org.wpilib.units.Units.FeetPerSecond;
import static org.wpilib.units.Units.Inch;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radian;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotation;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.List;
import java.util.Optional;
import java.util.function.ToDoubleFunction;
import java.util.function.UnaryOperator;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;

class DrivetrainControlTest {
  /** The template robot's drivetrain: SDS MK4i, L2, on Kraken X60s, 4 in wheels. */
  private static final Drivetrain TEMPLATES =
      Drivetrain.uniform(
          Drivetrain.Module.builder()
              .driveMotor(Motor.KRAKEN_X60)
              .steerMotor(Motor.KRAKEN_X60)
              .wheelRadius(Inches.of(2))
              .driveGearing(Gearing.of(6.75))
              .steerGearing(Gearing.of(150.0 / 7.0))
              .couplingRatio(50.0 / 14.0)
              .wheelCoefficientOfFriction(1.2)
              .driveInertia(KilogramSquareMeters.of(0.001))
              .steerInertia(KilogramSquareMeters.of(0.004))
              .build(),
          Inches.of(10.375),
          Inches.of(10.375));

  /**
   * The template's control, before a robot changes anything: the drive's kV and its top speed left
   * to the modules, Phoenix's own step-down written out, and the drive moves' gains.
   */
  @Test
  void theDefaultsAreTheTemplatesWithTheDrivesKvAndTopSpeedLeftToTheModules() {
    DrivetrainControl.Limits limits = DrivetrainControl.defaults().limits();
    assertThat(limits.driveStatorCurrent().in(Amps)).isCloseTo(30, within(1e-9));
    assertThat(limits.driveSupplyCurrent().in(Amps)).isCloseTo(60, within(1e-9));
    assertThat(limits.driveSupplyLowerCurrent().in(Amps)).isCloseTo(40, within(1e-9));
    assertThat(limits.driveSupplyLowerTime().in(Seconds)).isCloseTo(1, within(1e-9));
    assertThat(limits.steerStatorCurrent().in(Amps)).isCloseTo(40, within(1e-9));
    assertThat(limits.maxLinearSpeed()).isEmpty();
    assertThat(limits.maxAngularSpeed().in(RotationsPerSecond)).isCloseTo(1, within(1e-9));

    DrivetrainControl.SwerveGains gains = DrivetrainControl.defaults().gains();
    assertThat(gains.driveKs()).isZero();
    assertThat(gains.driveKv()).isEmpty();
    assertThat(gains.driveKp()).isEqualTo(2.0);
    assertThat(gains.steerKp()).isEqualTo(16.0);
    assertThat(gains.steerKd()).isEqualTo(0.08);
    assertThat(gains.skewLookahead().in(Seconds)).isCloseTo(0.03, within(1e-9));
    assertThat(gains.headingKp()).isEqualTo(5.0);
    assertThat(gains.translationKp()).isEqualTo(3.0);
  }

  /**
   * The drivetrain fills in the drive's kV as it takes its control: the modules' ideal, 12 V over
   * the 4.73 m/s free speed, until one is tuned, which it keeps. Its top speed likewise: four
   * fifths of the free speed, 3.78 m/s, until one is given, so a slower drivetrain than the
   * template's is never refused for a speed nobody chose.
   */
  @Test
  void theDrivetrainSeedsTheDrivesKvAndTopSpeedUntilTheyreGiven() {
    double ideal = TEMPLATES.module(0).idealDriveKv();

    Drivetrain seeded = TEMPLATES.control(DrivetrainControl.defaults());
    Drivetrain tuned =
        TEMPLATES.control(
            DrivetrainControl.defaults().withDriveKv(2.7).withMaxSpeed(MetersPerSecond.of(4.4)));

    assertThat(ideal).isCloseTo(2.538, within(0.001));
    assertThat(TEMPLATES.idealDriveKv()).isEqualTo(ideal);
    assertThat(seeded.control().orElseThrow().gains().driveKv()).contains(ideal);
    assertThat(seeded.control().orElseThrow().limits().maxLinearSpeed().orElseThrow())
        .isEqualTo(TEMPLATES.freeSpeed().times(0.8));
    assertThat(
            seeded
                .control()
                .orElseThrow()
                .limits()
                .maxLinearSpeed()
                .orElseThrow()
                .in(MetersPerSecond))
        .isCloseTo(3.783, within(0.001));
    assertThat(tuned.control().orElseThrow().gains().driveKv()).contains(2.7);
    assertThat(tuned.control().orElseThrow().limits().maxLinearSpeed())
        .contains(MetersPerSecond.of(4.4));
  }

  /**
   * A drivetrain slower than 4 m/s, a 7.5:1 drive on 1.5 in wheels (3.2 m/s free), takes the
   * template's control as it is: its top speed is seeded from it, where a fixed 4 m/s was refused.
   */
  @Test
  void aSlowDrivetrainTakesTheDefaultsAsTheyAre() {
    Drivetrain slow =
        Drivetrain.uniform(
            DrivetrainModuleBuilder.builder(TEMPLATES.module(0))
                .wheelRadius(Inches.of(1.5))
                .driveGearing(ConfigFixtures.gearing(7.5))
                .build(),
            Inches.of(10.375),
            Inches.of(10.375));

    DrivetrainControl control = slow.control(DrivetrainControl.defaults()).control().orElseThrow();

    assertThat(slow.freeSpeed().in(MetersPerSecond)).isCloseTo(3.19, within(0.01));
    assertThat(control.whyTooFastFor(slow)).isEmpty();
    assertThat(control.limits().maxLinearSpeed().orElseThrow().in(MetersPerSecond))
        .isCloseTo(0.8 * slow.freeSpeed().in(MetersPerSecond), within(1e-9));
  }

  /** One row a wither: each changes its own value, and nothing else. */
  @Test
  void eachWitherChangesItsOwnValueAndNothingElse() {
    record Row(
        String value,
        UnaryOperator<DrivetrainControl> changed,
        UnaryOperator<DrivetrainControlLimitsBuilder> limits,
        UnaryOperator<DrivetrainControlSwerveGainsBuilder> gains) {}
    List<Row> rows =
        List.of(
            new Row(
                "driveStatorCurrent",
                c -> c.withDriveStatorLimit(Amps.of(60)),
                b -> b.driveStatorCurrent(Amps.of(60)),
                b -> b),
            new Row(
                "driveSupplyCurrent",
                c -> c.withDriveSupplyLimit(Amps.of(20)),
                b -> b.driveSupplyCurrent(Amps.of(20)),
                b -> b),
            new Row(
                "driveSupplyLowerCurrent",
                c -> c.withDriveSupplyLowerLimit(Amps.of(10)),
                b -> b.driveSupplyLowerCurrent(Amps.of(10)),
                b -> b),
            new Row(
                "driveSupplyLowerTime",
                c -> c.withDriveSupplyLowerTime(Seconds.of(0.5)),
                b -> b.driveSupplyLowerTime(Seconds.of(0.5)),
                b -> b),
            new Row(
                "steerStatorCurrent",
                c -> c.withSteerStatorLimit(Amps.of(20)),
                b -> b.steerStatorCurrent(Amps.of(20)),
                b -> b),
            new Row(
                "maxLinearSpeed",
                c -> c.withMaxSpeed(MetersPerSecond.of(1.5)),
                b -> b.maxLinearSpeed(MetersPerSecond.of(1.5)),
                b -> b),
            new Row(
                "maxAngularSpeed",
                c -> c.withMaxTurnRate(RotationsPerSecond.of(0.5)),
                b -> b.maxAngularSpeed(RotationsPerSecond.of(0.5)),
                b -> b),
            new Row("driveKs", c -> c.withDriveKs(0.14), b -> b, b -> b.driveKs(0.14)),
            new Row("driveKv", c -> c.withDriveKv(2.6), b -> b, b -> b.driveKv(2.6)),
            new Row("driveKp", c -> c.withDriveKp(0.3), b -> b, b -> b.driveKp(0.3)),
            new Row(
                "steerKp",
                c -> c.withSteerKp(Volts.per(Radian).of(20.0)),
                b -> b,
                b -> b.steerKp(20.0)),
            new Row(
                "steerKd",
                c -> c.withSteerKd(Volts.per(RadiansPerSecond).of(0.1)),
                b -> b,
                b -> b.steerKd(0.1)),
            new Row(
                "skewLookahead",
                c -> c.withSkewLookahead(Seconds.of(0.02)),
                b -> b,
                b -> b.skewLookahead(Seconds.of(0.02))),
            new Row("headingKp", c -> c.withHeadingKp(4.0), b -> b, b -> b.headingKp(4.0)),
            new Row(
                "translationKp", c -> c.withTranslationKp(2.0), b -> b, b -> b.translationKp(2.0)));

    DrivetrainControl defaults = DrivetrainControl.defaults();
    for (Row row : rows) {
      DrivetrainControl expected =
          DrivetrainControlBuilder.builder(defaults)
              .limits(
                  row.limits()
                      .apply(DrivetrainControlLimitsBuilder.builder(defaults.limits()))
                      .build())
              .gains(
                  row.gains()
                      .apply(DrivetrainControlSwerveGainsBuilder.builder(defaults.gains()))
                      .build())
              .build();

      assertThat(row.changed().apply(defaults)).as(row.value()).isEqualTo(expected);
    }
  }

  /**
   * A gain given with its unit is held in the drivetrain's: volts per meter per second for the
   * drive, per radian for the steering. Tuner X's stock steering gains, per turn of the module, are
   * 100 and 0.5: over 2π, 15.9 and 0.0796.
   */
  @Test
  void aGainGivenWithItsUnitIsHeldInTheDrivetrainsUnits() {
    record Row(
        String call,
        UnaryOperator<DrivetrainControl> changed,
        ToDoubleFunction<DrivetrainControl.SwerveGains> read,
        double held) {}
    List<Row> rows =
        List.of(
            new Row(
                "withSteerKp(Volts.per(Rotation).of(100))",
                c -> c.withSteerKp(Volts.per(Rotation).of(100)),
                DrivetrainControl.SwerveGains::steerKp,
                100 / (2 * Math.PI)),
            new Row(
                "withSteerKp(Volts.per(Degree).of(0.5))",
                c -> c.withSteerKp(Volts.per(Degree).of(0.5)),
                DrivetrainControl.SwerveGains::steerKp,
                0.5 * 180 / Math.PI),
            new Row(
                "withSteerKd(Volts.per(RotationsPerSecond).of(0.5))",
                c -> c.withSteerKd(Volts.per(RotationsPerSecond).of(0.5)),
                DrivetrainControl.SwerveGains::steerKd,
                0.5 / (2 * Math.PI)),
            new Row(
                "withDriveKv(Volts.per(FeetPerSecond).of(0.75))",
                c -> c.withDriveKv(Volts.per(FeetPerSecond).of(0.75)),
                gains -> gains.driveKv().orElseThrow(),
                0.75 / 0.3048),
            new Row(
                "withDriveKp(Volts.per(MetersPerSecond).of(0.3))",
                c -> c.withDriveKp(Volts.per(MetersPerSecond).of(0.3)),
                DrivetrainControl.SwerveGains::driveKp,
                0.3));

    for (Row row : rows) {
      DrivetrainControl.SwerveGains gains =
          row.changed().apply(DrivetrainControl.defaults()).gains();
      double held = row.read().applyAsDouble(gains);

      assertThat(held).as(row.call()).isCloseTo(row.held(), within(1e-9));
    }
  }

  /**
   * A steering gain names its unit, so Tuner X's per turn and the drivetrain's per radian say the
   * same gain the same way: each row's two calls give the same control.
   */
  @Test
  void aSteeringGainPerTurnOrPerRadianIsTheSameGain() {
    record Row(
        String gain,
        UnaryOperator<DrivetrainControl> perTurn,
        UnaryOperator<DrivetrainControl> perRadian) {}
    List<Row> rows =
        List.of(
            new Row(
                "steerKp, Tuner X's stock 100 per turn",
                c -> c.withSteerKp(Volts.per(Rotation).of(100)),
                c -> c.withSteerKp(Volts.per(Radian).of(100 / (2 * Math.PI)))),
            new Row(
                "steerKd, Tuner X's stock 0.5 per turn a second",
                c -> c.withSteerKd(Volts.per(RotationsPerSecond).of(0.5)),
                c -> c.withSteerKd(Volts.per(RadiansPerSecond).of(0.5 / (2 * Math.PI)))));

    DrivetrainControl defaults = DrivetrainControl.defaults();
    for (Row row : rows) {
      DrivetrainControl.SwerveGains perTurn = row.perTurn().apply(defaults).gains();
      DrivetrainControl.SwerveGains perRadian = row.perRadian().apply(defaults).gains();

      assertThat(perTurn.steerKp()).as(row.gain()).isCloseTo(perRadian.steerKp(), within(1e-12));
      assertThat(perTurn.steerKd()).as(row.gain()).isCloseTo(perRadian.steerKd(), within(1e-12));
    }
  }

  /**
   * A gain given per what it can't be is refused, saying what to give: a steering gain per a
   * length, a drive gain per a turn (of the wheel or of the rotor, which a unit can't say), a kD
   * per an angle.
   */
  @Test
  void aGainGivenPerWhatItCantBeIsRefusedSayingWhatToGive() {
    record Row(String call, ThrowingCallable given, String message) {}
    DrivetrainControl defaults = DrivetrainControl.defaults();
    List<Row> rows =
        List.of(
            new Row(
                "steerKp per inch",
                () -> defaults.withSteerKp(Volts.per(Inch).of(1)),
                "SwerveGains.steerKp: it's per a length, and the steering turns: give it per turn of the"
                    + " module, as Tuner X shows it, Volts.per(Rotation)"),
            new Row(
                "driveKv per turn a second",
                () -> defaults.withDriveKv(Volts.per(RotationsPerSecond).of(0.12)),
                "SwerveGains.driveKv: it's per a speed of turning, and a drive gain is per the robot's"
                    + " speed along the ground: give it per Volts.per(MetersPerSecond), or as a"
                    + " plain number in volts per meter per second"),
            new Row(
                "steerKd per turn",
                () -> defaults.withSteerKd(Volts.per(Rotation).of(0.5)),
                "SwerveGains.steerKd: volts per Rotation can't be one: give it per a speed,"
                    + " Volts.per(RotationsPerSecond) or Volts.per(MetersPerSecond)"));

    for (Row row : rows) {
      assertThatIllegalArgumentException()
          .as(row.call())
          .isThrownBy(row.given())
          .withMessage(row.message());
    }
  }

  /**
   * A drive kV under half or over twice the modules' ideal, 2.54, is doubted, and one off by about
   * the ratio between two frames names them: Tuner X's 0.12, per turn a second of the rotor, typed
   * as a plain number.
   */
  @Test
  void aDriveKvFarFromTheIdealIsDoubtedNamingTheFramesItWasLikelyMixedUpIn() {
    record Row(String why, DrivetrainControl control, Optional<String> doubt) {}
    List<Row> rows =
        List.of(
            new Row("seeded", DrivetrainControl.defaults(), Optional.empty()),
            new Row("tuned near", DrivetrainControl.defaults().withDriveKv(2.8), Optional.empty()),
            new Row(
                "over twice",
                DrivetrainControl.defaults().withDriveKv(6.0),
                Optional.of(
                    "drivetrain's tuned driveKv, 6 V/(m/s), is 2.364 times the ideal, 2.538"
                        + " V/(m/s), from its motors and what it moves")),
            new Row(
                "Tuner X's, per rotor turn",
                DrivetrainControl.defaults().withDriveKv(0.12),
                Optional.of(
                    "drivetrain's tuned driveKv, 0.12 V/(m/s), is 0.04729 times the ideal, 2.538"
                        + " V/(m/s), from its motors and what it moves: the ideal per turn of its"
                        + " rotor, within 1%, entered as if per meter, by the look of it")));

    for (Row row : rows) {
      assertThat(TEMPLATES.control(row.control()).whyDriveKvIsDoubtful())
          .as(row.why())
          .isEqualTo(row.doubt());
    }
  }

  /** The template's drivetrain reaches 4.73 m/s free: 4 m/s leaves room, 5 m/s doesn't. */
  @Test
  void topSpeedCannotExceedTheFreeSpeed() {
    var drivetrain = TEMPLATES;
    var tooFast =
        DrivetrainControlBuilder.builder(control())
            .limits(changeLimits().maxLinearSpeed(MetersPerSecond.of(5)).build())
            .build();

    assertThat(control().whyTooFastFor(drivetrain)).isEmpty();
    assertThat(tooFast.whyTooFastFor(drivetrain))
        .contains(
            "5 m/s is faster than the drivetrain's free speed of 4.729 m/s: give withMaxSpeed at"
                + " most that");
  }

  /**
   * How fast a robot can speed up and spin up, by its own model, worked by hand for the fixture's:
   * 60 kg, a yaw inertia of 6 kg m², and at each corner, 0.374 m from its center, a Kraken X60
   * through 6.75:1 on a 2 in wheel whose tread grips at 1.2 of its quarter of the weight, 177 N.
   *
   * <ul>
   *   <li>At 40 A the motors give first: 0.775 N m each, 103 N at the floor. 412 N over 60 kg is
   *       6.86 m/s²; at 0.374 m, over 6 kg m², 25.7 rad/s².
   *   <li>At 80 A they'd push 206 N, and the tread gives first: 177 N a wheel, so the robot speeds
   *       up at 1.2 g, 11.77 m/s², whatever its mass, and spins up at 44.1 rad/s².
   * </ul>
   */
  @Test
  void aRobotSpeedsUpAsItsMotorsOrItsTreadAllowWhicheverIsLess() {
    RobotDefinition robot = ConfigFixtures.definition(List.of());
    double[][] rows = {
      // stator current (A), linear (m/s²), angular (rad/s²)
      {40, 6.86, 25.7},
      {80, 11.77, 44.1},
    };

    for (double[] row : rows) {
      DrivetrainControl control =
          DrivetrainControlBuilder.builder(control())
              .limits(changeLimits().driveStatorCurrent(Amps.of(row[0])).build())
              .build();

      DrivetrainControl.Accelerations most =
          DrivetrainControl.accelerations(robot, Kilograms.of(60), control);

      assertThat(most.linear()).as("%s A, linear", row[0]).isCloseTo(row[1], within(0.01));
      assertThat(most.angular()).as("%s A, angular", row[0]).isCloseTo(row[2], within(0.1));
    }
  }

  @Test
  void limitsMustBePositive() {
    assertRejected(
        "Limits.driveStatorCurrent", () -> changeLimits().driveStatorCurrent(Amps.of(0)).build());
    assertRejected(
        "Limits.driveSupplyCurrent", () -> changeLimits().driveSupplyCurrent(Amps.of(-60)).build());
    assertRejected(
        "Limits.driveSupplyLowerCurrent",
        () -> changeLimits().driveSupplyLowerCurrent(Amps.of(0)).build());
    assertRejected(
        "Limits.steerStatorCurrent",
        () -> changeLimits().steerStatorCurrent(Amps.of(Double.NaN)).build());
    assertRejected(
        "Limits.maxLinearSpeed",
        () -> changeLimits().maxLinearSpeed(MetersPerSecond.of(0)).build());
    assertRejected(
        "Limits.maxAngularSpeed",
        () ->
            changeLimits()
                .maxAngularSpeed(RotationsPerSecond.of(Double.POSITIVE_INFINITY))
                .build());
  }

  /**
   * Phoenix steps the drive's supply limit down after a time from just over zero to 5 s. At zero it
   * never steps down, which the template says by setting the lower current to the supply limit
   * instead, where it shows.
   */
  @Test
  void theStepDownTakesFromJustOverZeroToFiveSeconds() {
    assertRejected(
        "Limits.driveSupplyLowerTime",
        () -> changeLimits().driveSupplyLowerTime(Seconds.of(0)).build());
    assertRejected(
        "Limits.driveSupplyLowerTime",
        () -> changeLimits().driveSupplyLowerTime(Seconds.of(5.1)).build());
    assertThatNoException()
        .isThrownBy(() -> changeLimits().driveSupplyLowerTime(Seconds.of(5)).build());
    assertThatNoException()
        .as("no step-down, said where it shows")
        .isThrownBy(() -> changeLimits().driveSupplyLowerCurrent(Amps.of(60)).build());
  }

  @Test
  void gainsMayBeZeroButNotNegative() {
    assertThatNoException()
        .isThrownBy(
            () ->
                DrivetrainControlSwerveGainsBuilder.builder()
                    .driveKs(0)
                    .driveKp(0)
                    .steerKp(0)
                    .steerKd(0)
                    .skewLookahead(Seconds.of(0))
                    .headingKp(0)
                    .translationKp(0)
                    .driveKv(0.0)
                    .build());
    assertRejected("SwerveGains.driveKs", () -> changeGains().driveKs(-0.1).build());
    assertRejected("SwerveGains.driveKv", () -> changeGains().driveKv(Double.NaN).build());
    assertRejected("SwerveGains.driveKp", () -> changeGains().driveKp(-1).build());
    assertRejected("SwerveGains.steerKp", () -> changeGains().steerKp(-1).build());
    assertRejected("SwerveGains.headingKp", () -> changeGains().headingKp(-1).build());
    assertRejected("SwerveGains.translationKp", () -> changeGains().translationKp(-1).build());
    assertRejected("SwerveGains.steerKd", () -> changeGains().steerKd(-1).build());
    assertRejected(
        "SwerveGains.skewLookahead", () -> changeGains().skewLookahead(Seconds.of(-0.01)).build());
    assertRejected(
        "SwerveGains.skewLookahead",
        () -> changeGains().skewLookahead(Seconds.of(Double.NaN)).build());
  }

  static DrivetrainControl.Limits limits() {
    return DrivetrainControlLimitsBuilder.builder()
        .driveStatorCurrent(Amps.of(40))
        .driveSupplyCurrent(Amps.of(60))
        .driveSupplyLowerCurrent(Amps.of(35))
        .driveSupplyLowerTime(Seconds.of(0.5))
        .steerStatorCurrent(Amps.of(30))
        .maxAngularSpeed(RotationsPerSecond.of(1))
        .maxLinearSpeed(MetersPerSecond.of(4))
        .build();
  }

  static DrivetrainControl.SwerveGains gains() {
    return DrivetrainControlSwerveGainsBuilder.builder()
        .driveKs(0.1)
        .driveKp(2.0)
        .steerKp(16.0)
        .steerKd(0.08)
        .skewLookahead(Seconds.of(0.03))
        .headingKp(5.0)
        .translationKp(3.0)
        .driveKv(2.5)
        .build();
  }

  static DrivetrainControl control() {
    return DrivetrainControlBuilder.builder().limits(limits()).gains(gains()).build();
  }

  private static DrivetrainControlLimitsBuilder changeLimits() {
    return DrivetrainControlLimitsBuilder.builder(limits());
  }

  private static DrivetrainControlSwerveGainsBuilder changeGains() {
    return DrivetrainControlSwerveGainsBuilder.builder(gains());
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(field);
  }
}
