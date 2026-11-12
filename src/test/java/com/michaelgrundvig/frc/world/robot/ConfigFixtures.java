package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.physical.Inertia;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Translation3d;

/**
 * Valid pieces of a robot's definition for tests to start from, independent of the team's robots.
 * To change a value, copy one: {@code BodyBuilder.builder(body()).frameLength(...).build()}.
 */
public final class ConfigFixtures {
  private ConfigFixtures() {}

  /** A 28 x 24 in frame with 3.5 in bumpers (a 35 x 31 in outline), 30 in tall. */
  public static Body body() {
    return Body.frame(Inches.of(28), Inches.of(24), Inches.of(30))
        .bumpers(Inches.of(3.5), Inches.of(2.5), Inches.of(5.75))
        .weight(Kilograms.of(60))
        .inertia(
            Body.Turning.tensor(
                Inertia.builder().ixx(6).iyy(6).izz(6).ixy(0).ixz(0).iyz(0).build()))
        .cg(new Translation3d(0, 0, 0.2));
  }

  /** A reduction that loses nothing. */
  public static Gearing gearing(double ratio) {
    return Gearing.of(ratio);
  }

  /** Kraken X60 drive at 6.75:1 on 2 in radius wheels: 4.73 m/s free speed. */
  public static Drivetrain.Module modules() {
    return Drivetrain.Module.builder()
        .driveMotor(Motor.KRAKEN_X60)
        .steerMotor(Motor.KRAKEN_X44)
        .wheelRadius(Inches.of(2))
        .driveGearing(gearing(6.75))
        .steerGearing(gearing(12.8))
        .couplingRatio(0)
        .wheelCoefficientOfFriction(1.2)
        .driveInertia(KilogramSquareMeters.of(0.001))
        .steerInertia(KilogramSquareMeters.of(0.004))
        .build();
  }

  /** Wheels 2.625 in in from the frame's edges. */
  public static Drivetrain drivetrain() {
    return Drivetrain.inset(modules(), body(), Inches.of(2.625));
  }

  /** A 1280 x 800 camera with an 80 degree field of view. */
  public static CameraConfig.ImageSensor sensor() {
    return CameraConfig.ImageSensor.of(1280, 800, Hertz.of(30))
        .fieldOfView(Degrees.of(80), CameraConfig.Across.DIAGONAL)
        .latency(Milliseconds.of(30))
        .sightRange(Meters.of(7));
  }

  /** A level, forward-facing camera at the given position. */
  public static CameraConfig camera(String name, Translation3d position) {
    return CameraConfig.of(name, sensor())
        .at(position.getMeasureX(), position.getMeasureY(), position.getMeasureZ())
        .facing(Degrees.of(0), Degrees.of(0))
        .roll(Degrees.of(0))
        .noiseScale(1.0);
  }

  /**
   * The body and drivetrain above, with these cameras, and a backup IMU beside the gyro, neither
   * measured yet.
   */
  public static RobotDefinition definition(List<CameraConfig> cameras) {
    return RobotDefinition.of("test", body(), drivetrain())
        .withBackupImu(ImuMount.UNCHECKED)
        .withCameras(cameras.toArray(CameraConfig[]::new))
        .checked();
  }

  /**
   * A drivetrain wired on one bus: its gyro at an ID, and each corner's drive and steering on
   * controllers of the kinds given, numbered from 1, the front left's drive 1 and steering 2, the
   * front right's 3 and 4, and on. A TalonFX's steering reads a CANcoder, numbered from {@code
   * firstCancoder}; a SPARK's reads its data port. Every encoder is offset 0, every device turning
   * counterclockwise for positive.
   */
  public static Drivetrain wired(
      Drivetrain drivetrain,
      CanBus bus,
      MechanismWiring.Controller drive,
      MechanismWiring.Controller steer,
      int gyro,
      int firstCancoder) {
    List<SwerveWiring.ModuleWiring> corners = new ArrayList<>();
    for (int i = 0; i < 4; i++) {
      SwerveWiring.ModuleEncoder encoder =
          steer == MechanismWiring.Controller.TALON_FX
              ? MechanismWiring.CANCODER.can(bus, firstCancoder + i).offset(Rotations.of(0))
              : MechanismWiring.DATA_PORT.offset(Rotations.of(0));
      corners.add(
          SwerveWiringModuleWiringBuilder.builder()
              .drive(drive.can(bus, 1 + 2 * i))
              .steering(steer.can(bus, 2 + 2 * i))
              .encoder(encoder)
              .build());
    }
    return wired(drivetrain, SwerveWiring.PIGEON_2.can(bus, gyro), corners);
  }

  /**
   * A drivetrain wired with a gyro and four corners: front left, front right, back left, back
   * right.
   */
  public static Drivetrain wired(
      Drivetrain drivetrain, SwerveWiring.Gyro gyro, List<SwerveWiring.ModuleWiring> corners) {
    return drivetrain
        .gyro(gyro)
        .frontLeft(corners.get(0).drive(), corners.get(0).steering(), corners.get(0).encoder())
        .frontRight(corners.get(1).drive(), corners.get(1).steering(), corners.get(1).encoder())
        .backLeft(corners.get(2).drive(), corners.get(2).steering(), corners.get(2).encoder())
        .backRight(corners.get(3).drive(), corners.get(3).steering(), corners.get(3).encoder());
  }
}
