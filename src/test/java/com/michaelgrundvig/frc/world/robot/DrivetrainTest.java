package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.body;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.drivetrain;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.modules;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import com.michaelgrundvig.frc.world.physical.MassProperties;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

class DrivetrainTest {
  private static final MechanismWiring.Commutation PLAIN = MechanismWiring.Commutation.TRAPEZOIDAL;

  @Test
  void keepsItsSettings() {
    Drivetrain drivetrain = drivetrain();

    assertThat(drivetrain.corners()).extracting(Drivetrain.Corner::module).containsOnly(modules());

    Drivetrain.Module modules = drivetrain.module(0);
    assertThat(modules.driveMotor()).isEqualTo(Motor.KRAKEN_X60);
    assertThat(modules.steerMotor()).isEqualTo(Motor.KRAKEN_X44);
    assertThat(modules.wheelRadius().in(Inches)).isCloseTo(2, within(1e-9));
    assertThat(modules.driveGearing()).isEqualTo(gearing(6.75));
    assertThat(modules.steerGearing()).isEqualTo(gearing(12.8));
    assertThat(modules.wheelCoefficientOfFriction()).contains(1.2);
    assertThat(modules.driveInertia().orElseThrow().in(KilogramSquareMeters)).isEqualTo(0.001);
    assertThat(modules.steerInertia().orElseThrow().in(KilogramSquareMeters)).isEqualTo(0.004);
  }

  /**
   * A module type that leaves its tread's grip and its inertias out takes their defaults:
   * PathPlanner's 1.2, and the template's estimates, 0.001 and 0.004 kg m². Its joints' discs turn
   * as hard as those, and a measured tread beside it still slips first at its own grip.
   */
  @Test
  void aModuleLeftUnmeasuredTakesTheDefaults() {
    Drivetrain.Module unmeasured =
        changeModules()
            .wheelCoefficientOfFriction(Optional.empty())
            .driveInertia(Optional.empty())
            .steerInertia(Optional.empty())
            .build();
    Drivetrain.ModuleJoints corner =
        unmeasured.joints("here", new Translation2d(0.3, 0.2), PLAIN, PLAIN);
    Drivetrain worn =
        Drivetrain.builder()
            .corners(
                List.of(
                    corner(unmeasured, new Translation2d(0.3, 0.2)),
                    corner(unmeasured, new Translation2d(0.3, -0.2)),
                    corner(
                        changeModules().wheelCoefficientOfFriction(1.0).build(),
                        new Translation2d(-0.3, 0.2)),
                    corner(unmeasured, new Translation2d(-0.3, -0.2))))
            .build();

    assertThat(unmeasured.wheelCoefficientOfFrictionOrDefault()).isEqualTo(1.2);
    assertThat(unmeasured.driveInertiaOrDefault().in(KilogramSquareMeters)).isEqualTo(0.001);
    assertThat(unmeasured.steerInertiaOrDefault().in(KilogramSquareMeters)).isEqualTo(0.004);
    assertThat(corner.wheel().link().mass().inertia().iyy()).isCloseTo(0.001, within(1e-12));
    assertThat(corner.steer().link().mass().inertia().izz()).isCloseTo(0.004, within(1e-12));
    assertThat(Drivetrain.inset(unmeasured, body(), Inches.of(2.625)).wheelCoefficientOfFriction())
        .isEqualTo(1.2);
    assertThat(worn.wheelCoefficientOfFriction()).as("the measured tread's, least").isEqualTo(1.0);
    assertThat(modules().wheelCoefficientOfFrictionOrDefault()).as("as measured").isEqualTo(1.2);
  }

  @Test
  void freeSpeedComesFromTheMotorGearingAndWheel() {
    // A Kraken X60 turns 6000 rpm free: 100 rotations per second, divided by the gearing, times
    // the wheel's circumference.
    double expected = 6000.0 / 60 / 6.75 * 2 * Math.PI * Inches.of(2).in(Meters);

    assertThat(modules().freeSpeed().in(MetersPerSecond)).isCloseTo(expected, within(1e-9));
    assertThat(modules().idealDriveKv()).isCloseTo(12 / expected, within(1e-9));
    assertThat(drivetrain().freeSpeed().in(MetersPerSecond)).isCloseTo(expected, within(1e-9));
  }

  /** The drive and the steering are each one motor through their own gearing. */
  @Test
  void eachWheelIsDrivenAndTurnedByOneMotor() {
    assertThat(drivetrain().driveMotors(0)).isEqualTo(Actuator.MotorGroup.of(Motor.KRAKEN_X60, 1));
    assertThat(drivetrain().steerMotors(0).motor()).isEqualTo(Motor.KRAKEN_X44);
    assertThat(drivetrain().steerMotors(0).count()).isEqualTo(1);
  }

  @Test
  void wheelsSitInFromTheFrameCorners() {
    // A 28 x 24 in frame with wheels 2.625 in in from its edges.
    double x = Inches.of(14 - 2.625).in(Meters);
    double y = Inches.of(12 - 2.625).in(Meters);

    assertThat(drivetrain().moduleTranslations())
        .as("front left, front right, back left, back right")
        .containsExactly(
            new Translation2d(x, y),
            new Translation2d(x, -y),
            new Translation2d(-x, y),
            new Translation2d(-x, -y));
    assertThat(Drivetrain.uniform(modules(), Meters.of(x), Meters.of(y)).moduleTranslations())
        .as("the same, as Tuner X's swerve project gives them")
        .containsExactly(drivetrain().moduleTranslations());
  }

  /**
   * Each corner its own type, where its wheel sits: MK5i in front and MK5n behind, their steering
   * geared apart. The slowest module sets the drivetrain's free speed, the slipperiest tread its
   * grip.
   */
  @Test
  void eachCornerHasItsOwnModule() {
    Drivetrain.Module back =
        DrivetrainModuleBuilder.builder(modules())
            .steerGearing(gearing(26.09))
            .driveGearing(gearing(7.5))
            .wheelCoefficientOfFriction(1.0)
            .build();
    List<Drivetrain.Corner> corners = new ArrayList<>(drivetrain().corners());
    corners.set(2, corner(back, corners.get(2).position()));
    corners.set(3, corner(back, corners.get(3).position()));
    Drivetrain mixed = Drivetrain.builder().corners(corners).build();

    assertThat(mixed.module(0)).isEqualTo(modules());
    assertThat(mixed.module(3)).isEqualTo(back);
    assertThat(
            mixed
                .joints()
                .get(3)
                .steering()
                .transmission()
                .shaftRadiansPer(mixed.joints().get(3).steer()))
        .isEqualTo(26.09);
    assertThat(mixed.freeSpeed()).isEqualTo(back.freeSpeed());
    assertThat(mixed.wheelCoefficientOfFriction()).isEqualTo(1.0);
  }

  @Test
  void wheelsMustFitInsideTheFrameEachInItsCorner() {
    // The frame is 28 in long and 24 in wide: a 12 in inset meets in the middle side to side.
    Drivetrain wide = Drivetrain.uniform(modules(), Inches.of(11), Inches.of(13));

    assertThat(wide.whyWheelsDontFit(body()).orElseThrow())
        .contains("wheels outside the frame (28 in by 24 in): front-left's");
    assertThat(drivetrain().whyWheelsDontFit(body())).isEmpty();
    assertRejected("Drivetrain.corners", () -> Drivetrain.inset(modules(), body(), Inches.of(12)));
    List<Drivetrain.Corner> swapped = new ArrayList<>(drivetrain().corners());
    swapped.set(0, drivetrain().corners().get(1));
    swapped.set(1, drivetrain().corners().get(0));
    assertRejected("Drivetrain.corners", () -> Drivetrain.builder().corners(swapped).build());
    assertRejected(
        "Drivetrain.corners: needs four",
        () -> Drivetrain.builder().corners(swapped.subList(0, 3)).build());
  }

  /**
   * A wheel's center is inside the frame short of its outside, in both directions: one on the edge
   * of the frame, or beyond half its length, is outside it.
   */
  @Test
  void aWheelOnTheFramesEdgeOrBeyondItsHalfIsOutsideIt() {
    double halfLength = body().frameLength().in(Meters) / 2;
    double halfWidth = body().frameWidth().in(Meters) / 2;
    String outside = "wheels outside the frame (28 in by 24 in): front-left's, at ";

    assertThat(
            Drivetrain.uniform(modules(), Meters.of(halfLength), Meters.of(0.2))
                .whyWheelsDontFit(body())
                .orElseThrow())
        .as("on the front edge")
        .startsWith(outside);
    assertThat(
            Drivetrain.uniform(modules(), Meters.of(0.2), Meters.of(halfWidth))
                .whyWheelsDontFit(body())
                .orElseThrow())
        .as("on the side edge")
        .startsWith(outside);
    assertThat(
            Drivetrain.uniform(modules(), Meters.of(halfLength * 1.2), Meters.of(0.2))
                .whyWheelsDontFit(body())
                .orElseThrow())
        .as("past half its length, inside all of it")
        .startsWith(outside);
    assertThat(
            Drivetrain.uniform(modules(), Meters.of(halfLength - 0.01), Meters.of(halfWidth - 0.01))
                .whyWheelsDontFit(body()))
        .as("just inside both")
        .isEmpty();
  }

  /**
   * Each corner's wheel sits in its own quarter of the robot, off the lines through its center: one
   * on a line, or across it, is refused, saying which way it must be.
   */
  @Test
  void eachWheelMustSitInItsOwnQuarterOffTheCentersLines() {
    List<Drivetrain.Corner> corners = drivetrain().corners();
    List<Drivetrain.Corner> opposite = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      opposite.add(corner(modules(), corners.get(3 - i).position()));
    }
    List<Drivetrain.Corner> onLines = new ArrayList<>(corners);
    onLines.set(0, corner(modules(), new Translation2d(0, corners.get(0).position().getY())));
    onLines.set(3, corner(modules(), new Translation2d(corners.get(3).position().getX(), 0)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> Drivetrain.builder().corners(opposite).build())
        .withMessageContaining("front-left must be ahead of the center and left of it, not at ")
        .withMessageContaining("front-right must be ahead of the center and right of it, not at ")
        .withMessageContaining("back-left must be behind the center and left of it, not at ")
        .withMessageContaining("back-right must be behind the center and right of it, not at ");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Drivetrain.builder().corners(onLines).build())
        .withMessageContaining("front-left must be ahead of the center and left of it, not at ")
        .withMessageContaining("back-right must be behind the center and right of it, not at ");
  }

  /**
   * A module's moving parts are discs of its wheel's radius as hard to turn as its inertias say: a
   * solid disc's mass is twice its inertia over its radius squared, its inertia about a diameter
   * half of the one about its axis.
   */
  @Test
  void eachModulesPartsAreDiscsAsHardToTurnAsItsInertiasSay() {
    Drivetrain.ModuleJoints corner = drivetrain().joints().get(0);
    double radius = modules().wheelRadius().in(Meters);

    MassProperties wheel = corner.wheel().link().mass();
    assertThat(wheel.mass().in(Kilograms)).isCloseTo(2 * 0.001 / (radius * radius), within(1e-9));
    assertThat(wheel.inertia().ixx()).isCloseTo(0.0005, within(1e-12));
    assertThat(wheel.inertia().iyy()).isCloseTo(0.001, within(1e-12));
    assertThat(wheel.inertia().izz()).isCloseTo(0.0005, within(1e-12));
    MassProperties module = corner.steer().link().mass();
    assertThat(module.mass().in(Kilograms)).isCloseTo(2 * 0.004 / (radius * radius), within(1e-9));
    assertThat(module.inertia().ixx()).isCloseTo(0.002, within(1e-12));
    assertThat(module.inertia().iyy()).isCloseTo(0.002, within(1e-12));
    assertThat(module.inertia().izz()).isCloseTo(0.004, within(1e-12));
  }

  /**
   * How a drivetrain is driven and wired isn't what it is, nor is what only the simulator reads: a
   * re-tuned or re-wired drivetrain, or one whose modules' inertias are measured otherwise, has the
   * same hardware, and one whose modules differ has not. Equality is a record's, every value the
   * same, so the re-tuned and the re-wired ones aren't equal to it.
   */
  @Test
  void aDrivetrainIsTheSameHardwareHoweverItIsDrivenWiredOrWeighed() {
    Drivetrain plain = drivetrain();
    Drivetrain driven = plain.control(DrivetrainControlTest.control());
    Drivetrain wired = wired(plain, CanBus.SYSTEMCORE, TALON_FX, TALON_FX);
    Drivetrain heavier =
        Drivetrain.inset(
            changeModules().driveInertia(KilogramSquareMeters.of(0.002)).build(),
            body(),
            Inches.of(2.625));
    Drivetrain faster =
        Drivetrain.inset(
            changeModules().driveGearing(gearing(6)).build(), body(), Inches.of(2.625));

    assertThat(driven.control()).contains(DrivetrainControlTest.control());
    assertThat(List.of(driven, wired, heavier)).allMatch(same -> same.sameHardwareAs(plain));
    assertThat(faster.sameHardwareAs(plain)).isFalse();
    assertThat(List.of(driven, wired, heavier, faster)).noneMatch(plain::equals);
    assertThat(drivetrain()).isEqualTo(plain).hasSameHashCodeAs(plain);
  }

  @Test
  void rejectsImpossibleModuleValues() {
    assertRejected("Module.wheelRadius", () -> changeModules().wheelRadius(Inches.of(0)).build());
    assertRejected(
        "Module.wheelCoefficientOfFriction",
        () -> changeModules().wheelCoefficientOfFriction(Double.NaN).build());
    assertRejected(
        "Module.driveInertia",
        () -> changeModules().driveInertia(KilogramSquareMeters.of(0)).build());
    assertRejected(
        "Module.steerInertia",
        () -> changeModules().steerInertia(KilogramSquareMeters.of(-0.004)).build());
    assertRejected("Module.couplingRatio", () -> changeModules().couplingRatio(-1).build());
    assertRejected("Corner.position", () -> corner(modules(), new Translation2d(Double.NaN, 0.2)));
  }

  private static DrivetrainModuleBuilder changeModules() {
    return DrivetrainModuleBuilder.builder(modules());
  }

  private static Drivetrain.Corner corner(Drivetrain.Module module, Translation2d position) {
    return Drivetrain.Corner.builder().module(module).position(position).build();
  }

  /**
   * Without a coupling the drive motor turns only its wheel; with one, the module's steering too.
   * Each module is a disc as hard to turn as the module's inertia says.
   */
  @Test
  void eachCornersDriveMotorTurnsItsWheelAndThroughTheCouplingItsSteering() {
    Drivetrain.ModuleJoints uncoupled = drivetrain().joints().get(1);
    Drivetrain.ModuleJoints coupled =
        DrivetrainModuleBuilder.builder(modules())
            .couplingRatio(3.5)
            .build()
            .joints("here", new Translation2d(0.3, 0.2), PLAIN, PLAIN);

    assertThat(uncoupled.label()).isEqualTo("front-right");
    assertThat(uncoupled.drive().transmission().joints()).containsExactly(uncoupled.wheel());
    assertThat(uncoupled.steering().transmission().joints()).containsExactly(uncoupled.steer());
    assertThat(uncoupled.wheel().parent()).isSameAs(uncoupled.steer());
    assertThat(coupled.drive().transmission().joints())
        .containsExactly(coupled.wheel(), coupled.steer());
    assertThat(coupled.drive().transmission().shaftRadiansPer(coupled.steer())).isEqualTo(3.5);
    assertThat(coupled.wheel().link().mass().inertia().iyy())
        .isCloseTo(modules().driveInertiaOrDefault().in(KilogramSquareMeters), within(1e-12));
    assertThat(coupled.steer().link().mass().inertia().izz())
        .isCloseTo(modules().steerInertiaOrDefault().in(KilogramSquareMeters), within(1e-12));
  }

  /**
   * A drivetrain's wiring must drive its definition's motors: REV's MAXSwerve, a Vortex driving and
   * a NEO 550 steering, on SPARKs; the template's own Krakens on TalonFXs, and not the other way
   * round, saying what does. SPARKs are on Systemcore's own CAN, never a CANivore.
   */
  @Test
  void aSwervesWiringMustDriveItsMotors() {
    Drivetrain ours = drivetrain();
    Drivetrain.Module maxSwerve =
        DrivetrainModuleBuilder.builder(ours.corners().get(0).module())
            .driveMotor(Motor.NEO_VORTEX)
            .steerMotor(Motor.NEO_550)
            .build();
    Drivetrain rev =
        DrivetrainBuilder.builder(ours)
            .corners(
                ours.corners().stream()
                    .map(
                        corner -> DrivetrainCornerBuilder.builder(corner).module(maxSwerve).build())
                    .toList())
            .build();
    CanBus systemcore = CanBus.SYSTEMCORE;
    CanBus canivore = CanBus.canivore("canivore");

    assertThat(wired(ours, systemcore, TALON_FX, TALON_FX).wiring()).isPresent();
    assertThat(wired(rev, systemcore, SPARK_FLEX, SPARK_MAX).wiring()).isPresent();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> wired(rev, systemcore, TALON_FX, TALON_FX))
        .withMessageContaining(
            "its drive's NEO Vortex is on a TalonFX, which doesn't drive one: a TalonFXS or a"
                + " SPARK MAX or a SPARK Flex does; its steering's NEO 550 is on a TalonFX, which"
                + " doesn't drive one: a TalonFXS or a SPARK MAX or a SPARK Flex does");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> wired(rev, canivore, SPARK_FLEX, SPARK_MAX))
        .withMessageContaining("its SPARKs are on a CANivore: a SPARK is on Systemcore's own CAN");
  }

  /**
   * Controllers that drive the modules' motors but are of two kinds, a SPARK on one and a TalonFX
   * on the other, are SPARKs all the same: they're on Systemcore's own CAN, not a CANivore, which
   * TalonFXs may be on.
   */
  @Test
  void aSwerveWithASparkOnOneMotorAndATalonOnTheOtherIsOnSystemcoresCan() {
    CanBus systemcore = CanBus.SYSTEMCORE;
    CanBus canivore = CanBus.canivore("canivore");
    Drivetrain.Module sparkDrive =
        changeModules().driveMotor(Motor.NEO_VORTEX).steerMotor(Motor.KRAKEN_X44).build();
    Drivetrain.Module sparkSteer =
        changeModules().driveMotor(Motor.KRAKEN_X60).steerMotor(Motor.NEO_550).build();
    Drivetrain sparkDriven = Drivetrain.uniform(sparkDrive, Inches.of(11), Inches.of(9));
    Drivetrain sparkSteered = Drivetrain.uniform(sparkSteer, Inches.of(11), Inches.of(9));

    assertThat(wired(sparkDriven, systemcore, SPARK_FLEX, TALON_FX).wiring()).isPresent();
    assertThat(wired(sparkSteered, systemcore, TALON_FX, SPARK_MAX).wiring()).isPresent();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> wired(sparkDriven, canivore, SPARK_FLEX, TALON_FX))
        .withMessageContaining("its SPARKs are on a CANivore");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> wired(sparkSteered, canivore, TALON_FX, SPARK_MAX))
        .withMessageContaining("its SPARKs are on a CANivore");
    assertThat(wired(drivetrain(), canivore, TALON_FX, TALON_FX).wiring())
        .as("TalonFXs may be on a CANivore")
        .isPresent();
  }

  /** A drivetrain wired as the fixtures wire one: gyro 30, motors from 1, CANcoders from 20. */
  private static Drivetrain wired(
      Drivetrain drivetrain,
      CanBus bus,
      MechanismWiring.Controller drive,
      MechanismWiring.Controller steer) {
    return ConfigFixtures.wired(drivetrain, bus, drive, steer, 30, 20);
  }

  private static void assertRejected(String field, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageContaining(field);
  }
}
