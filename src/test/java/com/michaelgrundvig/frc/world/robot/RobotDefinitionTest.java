package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.camera;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.drivetrain;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.robot.Driven.LimitSwitch;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/** A robot's parts are checked against each other when the definition is written down. */
class RobotDefinitionTest {

  @Test
  void wheelPositionsComeFromTheDrivetrain() {
    assertThat(definition(List.of()).moduleTranslations())
        .containsExactly(drivetrain().moduleTranslations());
  }

  @Test
  void theWheelsMustFitTheFrame() {
    var tooWide = Drivetrain.uniform(ConfigFixtures.modules(), Inches.of(11), Inches.of(13));

    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            RobotDefinitionBuilder.builder(definition(List.of()))
                .drivetrain(tooWide)
                .build()
                .checked())
        .withMessageStartingWith("RobotDefinition[test].drivetrain.corners");
  }

  /** A robot has a name, which every message about its definition starts with. */
  @Test
  void aRobotNeedsAName() {
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> RobotDefinitionBuilder.builder(definition(List.of())).name(" ").build().checked())
        .withMessageStartingWith("RobotDefinition[ ].name");
  }

  @Test
  void cameraNamesMustBeUnique() {
    var cameras = List.of(camera("front", inches(10, 0, 8)), camera("front", inches(-10, 0, 8)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(cameras))
        .withMessageContaining("two cameras are named front");
  }

  @Test
  void cameraNamesMustBeUniqueAsIdsToo() {
    var cameras =
        List.of(camera("Front Left", inches(10, 0, 8)), camera("front-left", inches(-10, 0, 8)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(cameras))
        .withMessageContaining(
            "cameras \"Front Left\" and \"front-left\" differ only in case or punctuation, so"
                + " they're one id, \"front-left\": name them apart");
  }

  @Test
  void camerasMustBeOnTheRobot() {
    // 10 m ahead: the kind of mistake made by typing inches where meters belong.
    var cameras = List.of(camera("front", new Translation3d(10, 0, 0.2)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> definition(cameras))
        .withMessageContaining("camera front is outside the robot");
  }

  /**
   * An IMU is mounted inside the robot, as a camera is: the backup 2 m up, or the gyro 1 m behind,
   * is a mistake in units or sign. The gyro is the drivetrain's, and its mount with it; a
   * drivetrain not wired takes its gyro as every gyro starts.
   */
  @Test
  void imusMustBeOnTheRobot() {
    ImuMount high = ImuMountBuilder.builder(ImuMount.UNCHECKED).up(Inches.of(80)).build();
    ImuMount behind = ImuMountBuilder.builder(ImuMount.UNCHECKED).forward(Inches.of(-40)).build();
    RobotDefinition robot = definition(List.of());

    assertThatIllegalArgumentException()
        .isThrownBy(() -> RobotDefinitionBuilder.builder(robot).backupImu(high).build().checked())
        .withMessageContaining(
            "RobotDefinition[test].backupImu: the backup IMU is outside the robot");
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            RobotDefinitionBuilder.builder(robot)
                .drivetrain(
                    ConfigFixtures.wired(
                        drivetrain(),
                        SwerveWiring.PIGEON_2.can(1).at(behind),
                        wiredDrivetrain(1).wiring().orElseThrow().modules()))
                .build()
                .checked())
        .withMessageContaining(
            "RobotDefinition[test].drivetrain.wiring.gyro: the gyro is outside the robot");
    ImuMount aside = ImuMountBuilder.builder(ImuMount.UNCHECKED).left(Inches.of(4)).build();
    RobotDefinition besideItsBackup =
        RobotDefinitionBuilder.builder(robot).backupImu(aside).build();
    assertThat(besideItsBackup.gyroMount()).as("unwired, beside the backup").isEqualTo(aside);
    assertThat(
            RobotDefinitionBuilder.builder(robot).backupImu(Optional.empty()).build().gyroMount())
        .as("unwired, with no backup")
        .isEqualTo(ImuMount.UNCHECKED);
    assertThat(
            RobotDefinitionBuilder.builder(besideItsBackup)
                .drivetrain(wiredDrivetrain(1))
                .build()
                .gyroMount())
        .as("wired: its Pigeon's own")
        .isEqualTo(ImuMount.UNCHECKED);
    assertThat(
            RobotDefinitionBuilder.builder(robot).backupImu(Optional.empty()).build().backupImu())
        .as("a roboRIO robot has no backup IMU")
        .isEmpty();
  }

  @Test
  void anImusOrientationIsARotation() {
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            ImuMount.at(Meters.of(0), Meters.of(0), Meters.of(0))
                .turned(new Rotation3d(Double.NaN, 0, 0)))
        .withMessageContaining("ImuMount.orientation");
  }

  @Test
  void anImusMountSaysEachThingOnce() {
    ImuMount mount =
        ImuMount.at(Inches.of(2), Inches.of(-1), Inches.of(3))
            .turned(new Rotation3d(0, 0, Math.PI / 2))
            .accelerometerCheckPassed();

    assertThat(mount.position())
        .isEqualTo(new Translation3d(Inches.of(2), Inches.of(-1), Inches.of(3)));
    assertThat(mount.accelerometerChecked()).isTrue();
    assertThatIllegalArgumentException()
        .isThrownBy(() -> mount.turned(Rotation3d.ZERO))
        .withMessageStartingWith("ImuMount.orientation is given already");
    assertThatIllegalArgumentException()
        .isThrownBy(mount::accelerometerCheckPassed)
        .withMessage("ImuMount.accelerometerChecked is given already: give it once");
  }

  @Test
  void theCameraListCannotChangeAfterwards() {
    List<CameraConfig> cameras = new ArrayList<>(List.of(camera("front", inches(10, 0, 8))));
    RobotDefinition robot = definition(cameras);

    cameras.clear();

    assertThat(robot.cameras()).hasSize(1);
  }

  /**
   * A practice robot is the competition robot's joints, each changed where it's given and kept
   * otherwise: a CAD revision that regears the competition robot's pivot regears the practice
   * robot's, whose motors are only re-wired. Restating its motors, which would keep the old
   * gearing, is refused, saying how to re-wire them.
   */
  @Test
  void aRegearOfTheCompetitionRobotReachesThePracticeRobot() {
    for (double ratio : new double[] {25, 30}) {
      RevoluteJoint pivot = pivot(Gearing.of(ratio));
      RobotDefinition practice =
          robot(pivot).with(pivot.motors(motors -> motors.on(TALON_FX.can(33))));

      Driven.Motors motors = practice.joints().getFirst().driven().motors().orElseThrow();
      assertThat(motors.gearing()).isEqualTo(Gearing.of(ratio));
      assertThat(motors.controllers()).containsExactly(TALON_FX.can(33));
    }
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            pivot(Gearing.of(30))
                .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(33))))
        .withMessageStartingWith(
            "pivot already has its motors: to re-wire them, motors(motors -> motors.on(...))");
  }

  /**
   * A practice robot is the same robot with its parts re-wired: each joint, sensor or drivetrain
   * given replaces the robot's own it's equal to, the same but for how it's driven or wired. One
   * the robot hasn't is refused; so is one given twice, whose second would drop the first's change;
   * a joint's own switch or encoder, re-wired with its joint; and a drivetrain of other modules.
   */
  @Test
  void aPracticeRobotIsTheSameRobotWithItsPartsRewired() {
    RevoluteJoint pivot =
        pivot(Gearing.of(25))
            .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.2)))
            .limitSwitch(
                LimitSwitch.lower().at(Degrees.of(0)).on(SensorWiring.port(1).normallyClosed()));
    Sensor.Switch beam =
        Sensor.Switch.beam("claw beam", pivot).on(SensorWiring.port(2).normallyOpen());
    RobotDefinition competition =
        RobotDefinitionBuilder.builder(robot(pivot)).sensors(List.of(beam)).build();
    Drivetrain rewired = wiredDrivetrain(1);

    RobotDefinition practice =
        competition
            .with(
                pivot
                    .motors(motors -> motors.on(TALON_FX.can(33)))
                    .limitSwitch(
                        Sensor.Switch.Limit.End.LOWER,
                        limit -> limit.on(SensorWiring.port(3).normallyClosed())))
            .with(beam.on(CANDI.can(36).s1().normallyOpen()))
            .with(rewired);

    assertThat(practice.joints()).containsExactly(pivot);
    assertThat(
            RobotMath.switchAt(practice.joints().getFirst(), Sensor.Switch.Limit.End.LOWER)
                .wiring())
        .contains(SensorWiring.port(3).normallyClosed());
    assertThat(practice.sensors()).containsExactly(beam);
    assertThat(((Sensor.Switch) practice.sensors().getFirst()).wiring())
        .contains(CANDI.can(36).s1().normallyOpen());
    assertThat(practice.drivetrain().sameHardwareAs(competition.drivetrain())).isTrue();
    assertThat(practice.drivetrain()).isEqualTo(rewired);
    assertThat(practice.devices())
        .extracting(CanDevice::name)
        .contains("pivot motor 33", "CANdi 36", "the gyro");
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            competition.with(
                Joint.roller("roller", Part.CHASSIS)
                    .at(Meters.of(0.25), Meters.of(0), Meters.of(0.32))))
        .withMessage(
            "test has no joint roller the same but for how it's driven, its stops, where it rests"
                + " and where its places are");
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            competition.with(
                pivot.motors(motors -> motors.on(TALON_FX.can(33))),
                pivot.retuned(control -> control.withKd(1))))
        .withMessage(
            "pivot is given twice, and the second would drop the first's change: give it once,"
                + " its changes one after another");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(Sensor.Switch.beam("hopper beam", Part.CHASSIS)))
        .withMessage("test has no sensor hopper beam the same but for its wiring");
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            competition.with(
                RobotMath.switchAt(pivot, Sensor.Switch.Limit.End.LOWER)
                    .on(SensorWiring.port(4).normallyClosed())))
        .withMessage(
            "pivot lower limit is pivot's own: re-wire it with its joint,"
                + " limitSwitch(End.LOWER, limit -> limit.on(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(JointParts.sensors(pivot).getFirst()))
        .withMessage(
            "pivot absolute encoder is pivot's own: re-wire it with its joint,"
                + " absoluteEncoder(encoder -> encoder.on(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            competition.with(
                Drivetrain.uniform(ConfigFixtures.modules(), Inches.of(10), Inches.of(10))))
        .withMessage(
            "test's drivetrain isn't this one: their modules, or where they sit, differ. Give its"
                + " own, re-wired, .gyro(...).frontLeft(...)...backRight(...), or re-tuned,"
                + " .control(...)");
  }

  /**
   * A practice robot whose encoders differ only in their offsets says the offsets alone, so the
   * competition robot's CANcoders moved to other IDs, its joint's and its modules', reach it: the
   * silent divergence a whole CANcoder said again would leave. A drivetrain that isn't wired has no
   * offsets to change.
   */
  @Test
  void aCompetitionRobotsMovedCanIdReachesItsPracticeRobot() {
    for (int id : new int[] {34, 44}) {
      RevoluteJoint pivot =
          pivot(Gearing.of(25)).absoluteEncoder(CANCODER.can(id).offset(Rotations.of(0.2)));
      RobotDefinition competition =
          RobotDefinitionBuilder.builder(robot(pivot)).drivetrain(wiredDrivetrain(id + 1)).build();

      RobotDefinition practice =
          competition
              .with(pivot.absoluteEncoder(encoder -> encoder.offset(Rotations.of(0.47))))
              .with(
                  competition
                      .drivetrain()
                      .encoderOffsets(
                          Rotations.of(0.1),
                          Rotations.of(0.2),
                          Rotations.of(0.3),
                          Rotations.of(0.4)));

      assertThat(
              practice
                  .joints()
                  .getFirst()
                  .driven()
                  .absoluteEncoder()
                  .flatMap(Driven.AbsoluteEncoder::cancoder))
          .contains(CANCODER.can(id).offset(Rotations.of(0.47)));
      List<SwerveWiring.ModuleWiring> modules =
          practice.drivetrain().wiring().orElseThrow().modules();
      assertThat(modules)
          .extracting(module -> ((MechanismWiring.Cancoder) module.encoder()).id())
          .containsExactly(id + 1, id + 2, id + 3, id + 4);
      assertThat(modules)
          .extracting(module -> module.encoder().offset())
          .containsExactly(
              Rotations.of(0.1), Rotations.of(0.2), Rotations.of(0.3), Rotations.of(0.4));
      assertThat(modules.getFirst().drive().id()).isEqualTo(1);
    }
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            drivetrain()
                .encoderOffsets(Rotations.of(0), Rotations.of(0), Rotations.of(0), Rotations.of(0)))
        .withMessage(
            "the drivetrain isn't wired, so its modules have no offsets: wire it,"
                + " .gyro(...).frontLeft(...)...backRight(...)");
  }

  /** A practice robot is named on its definition, as on its configuration, the rest kept. */
  @Test
  void aPracticeRobotIsTheRobotUnderAnotherName() {
    RobotDefinition competition = robot(pivot(Gearing.of(25)));

    RobotDefinition practice = competition.named("practice");

    assertThat(practice.name()).isEqualTo("practice");
    assertThat(practice.joints()).isEqualTo(competition.joints());
    assertThat(practice.drivetrain()).isEqualTo(competition.drivetrain());
  }

  /**
   * A camera mounted otherwise is given whole, and replaces the robot's camera of its name: a
   * practice robot's back camera, a fraction of an inch lower. One the robot has no camera of the
   * name of is refused, and so is one given twice.
   */
  @Test
  void aPracticeRobotsCameraIsMountedOtherwise() {
    CameraConfig back = camera("back", inches(-10, 0, 20));
    CameraConfig front = camera("front", inches(10, 0, 20));
    RobotDefinition competition = definition(List.of(front, back));
    CameraConfig lower = CameraConfigBuilder.builder(back).up(Inches.of(19.81)).build();

    RobotDefinition practice = competition.with(lower);

    assertThat(practice.cameras()).containsExactly(front, lower);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(camera("side", inches(0, 10, 20))))
        .withMessage(
            "test has no camera named side: it has front, back; to add one, withCameras(...)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> competition.with(lower, back))
        .withMessage(
            "back is given twice, and the second would drop the first's change: give it once,"
                + " its changes one after another");
  }

  /**
   * A drivetrain whose modules differ only in what the simulator alone reads, their inertias, as a
   * practice robot's Tuner X simulation values can, is the robot's own, and the robot takes them.
   * One whose tread grips otherwise, which the robot plans by, is another.
   */
  @Test
  void aDrivetrainDifferingOnlyInItsSimulatorsValuesIsTheRobotsOwn() {
    RobotDefinition competition = definition(List.of());
    Drivetrain.Module heavier =
        DrivetrainModuleBuilder.builder(ConfigFixtures.modules())
            .driveInertia(KilogramSquareMeters.of(0.01))
            .steerInertia(KilogramSquareMeters.of(0.035))
            .build();
    Drivetrain practiceModules = Drivetrain.inset(heavier, ConfigFixtures.body(), Inches.of(2.625));

    RobotDefinition practice = competition.with(practiceModules);

    assertThat(practice.drivetrain().module(0).steerInertia())
        .contains(KilogramSquareMeters.of(0.035));
    Drivetrain.Module grippier =
        DrivetrainModuleBuilder.builder(ConfigFixtures.modules())
            .wheelCoefficientOfFriction(1.0)
            .build();
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            competition.with(Drivetrain.inset(grippier, ConfigFixtures.body(), Inches.of(2.625))))
        .withMessageStartingWith("test's drivetrain isn't this one");
  }

  /** A pivot of the robot's, as its block has it, geared as CAD has it. */
  private static RevoluteJoint pivot(Gearing gearing) {
    return Joint.pivot("pivot", Part.CHASSIS, Joint.PITCH_DOWN)
        .at(Meters.of(0.25), Meters.of(0), Meters.of(0.32))
        .range(Range.degrees(0, 135))
        .rest(Degrees.of(1.5))
        .motors(Motor.KRAKEN_X60.through(gearing).on(TALON_FX.can(32)))
        .control(
            MechanismControl.position(30)
                .withHoming(MechanismControl.Homing.toHardStop(Sensor.Switch.Limit.End.LOWER)));
  }

  /** The test robot with these joints. */
  private static RobotDefinition robot(Joint... joints) {
    return RobotDefinitionBuilder.builder(definition(List.of()))
        .joints(List.of(joints))
        .build()
        .checked();
  }

  /**
   * The test drivetrain wired on Systemcore's CAN: TalonFXs 1 to 8, its CANcoders numbered from the
   * one given, and its gyro 1.
   */
  private static Drivetrain wiredDrivetrain(int firstCancoder) {
    return ConfigFixtures.wired(
        drivetrain(), CanBus.SYSTEMCORE, TALON_FX, TALON_FX, 1, firstCancoder);
  }

  private static Translation3d inches(double x, double y, double z) {
    return new Translation3d(Inches.of(x), Inches.of(y), Inches.of(z));
  }
}
