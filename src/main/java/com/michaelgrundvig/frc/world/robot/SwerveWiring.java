package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.between;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.units.measure.Angle;

/**
 * Where a swerve drivetrain's devices are on the CAN bus, which way each turns, and how each
 * module's absolute encoder reads: what the robot code needs to drive real hardware. It's written
 * in the drivetrain's block as a joint writes its devices, the gyro first and then each corner's
 * drive, steering and encoder:
 *
 * {@snippet :
 * Drivetrain.uniform(MODULE, Inches.of(10.375), Inches.of(10.375))
 *     .gyro(PIGEON_2.can(CANIVORE, 1))
 *     .frontLeft(
 *         TALON_FX.can(CANIVORE, 23),
 *         TALON_FX.can(CANIVORE, 24),
 *         CANCODER.can(CANIVORE, 12).offset(Rotations.of(0.107)))
 *     // ... then frontRight, backLeft and backRight the same way
 * }
 *
 * <p>It differs robot to robot even with identical drivetrains, and the encoder offsets change
 * whenever a module is rebuilt, so measure them again then. Phoenix Tuner X's swerve generator
 * reports every value here for a CTRE drivetrain; copy them across as it gives them. Its coupling
 * ratio is the module's gearing, so it goes in the robot's definition ({@code
 * Drivetrain.Module.couplingRatio}). A REV drivetrain (MAXSwerve, say) has SPARKs, and each
 * module's absolute encoder on its steering SPARK's data port ({@code DATA_PORT.offset(...)}).
 *
 * <p>Directions are the module's, as a joint's are its own: a device turns counterclockwise for
 * positive, seen facing a motor's shaft or a CANcoder's LED, until {@code .clockwise()} says
 * otherwise. Positive is the wheel rolling forward for the drive, and the module turning
 * counterclockwise, seen from above, for the steering and the CANcoder. A data port's encoder says
 * {@code .inverted()} for one whose reading falls as the module turns counterclockwise.
 *
 * <p>The kind of controller and the bus aren't written: they follow from the devices, as a joint's
 * do. So a drivetrain's drives are all one kind, its steerings all one kind, and every device, the
 * gyro too, is on one bus.
 *
 * @param gyro the Pigeon 2: its bus and ID, its yaw scale, and where it's mounted
 * @param frontLeft the front left module's devices
 * @param frontRight the front right module's
 * @param backLeft the back left module's
 * @param backRight the back right module's
 */
@ConfigRecord
public record SwerveWiring(
    Gyro gyro,
    ModuleWiring frontLeft,
    ModuleWiring frontRight,
    ModuleWiring backLeft,
    ModuleWiring backRight) {
  /**
   * A Pigeon 2 at its bus and ID, the drivetrain's gyro: {@code PIGEON_2.can(CANIVORE, 1)}. It
   * reads turns at their true size, and sits at the robot's center, level, its accelerometer not
   * yet checked, until {@code .yawScale(...)} and {@code .at(...)} say otherwise.
   */
  public static final CanDeviceKind<Gyro> PIGEON_2 =
      new CanDeviceKind<>((bus, id) ->
          SwerveWiringGyroBuilder.builder()
              .bus(bus)
              .id(id)
              .yawScale(1)
              .mount(ImuMount.UNCHECKED)
              .build());

  public SwerveWiring {
    List<ModuleWiring> corners = List.of(frontLeft, frontRight, backLeft, backRight);
    for (int i = 0; i < corners.size(); i++) {
      ModuleChecks.ownValues(Drivetrain.LABELS.get(i), corners.get(i));
    }
    CanBus bus = gyro.bus();
    Checks.on(SwerveWiring.class)
        .whyNot("modules", () -> ModuleChecks.whyNotOneKind(corners))
        .whyNot("modules", () -> ModuleChecks.whyNotOneCommutation(corners))
        .whyNot("modules", () -> ModuleChecks.whyNotOneBus(bus, corners))
        .whyNot("modules", () -> ModuleChecks.whyItRunsNoModule(corners))
        .whyNot("modules", () -> ModuleChecks.whyEncodersDoNotFit(corners))
        .whyNot("modules", () -> ModuleChecks.whyIdsAreShared(corners))
        .orThrow();
  }

  /** The bus every one of its devices is on: the gyro's, which every module's is too. */
  public CanBus canBus() {
    return gyro.bus();
  }

  /** What its drive motors are on: a TalonFX, or a SPARK MAX or SPARK Flex, the same at each. */
  public MechanismWiring.Controller driveController() {
    return frontLeft.drive().controller();
  }

  /** What its steering motors are on, the same at each. */
  public MechanismWiring.Controller steerController() {
    return frontLeft.steering().controller();
  }

  /** Its modules in corner order: front left, front right, back left, back right. */
  public List<ModuleWiring> modules() {
    return List.of(frontLeft, frontRight, backLeft, backRight);
  }

  /**
   * The same wiring, its modules' absolute encoders offset otherwise, front left, front right, back
   * left, back right.
   */
  SwerveWiring encoderOffsets(Angle frontLeft, Angle frontRight, Angle backLeft, Angle backRight) {
    return SwerveWiringBuilder.builder(this)
        .frontLeft(this.frontLeft.encoderOffset(frontLeft))
        .frontRight(this.frontRight.encoderOffset(frontRight))
        .backLeft(this.backLeft.encoderOffset(backLeft))
        .backRight(this.backRight.encoderOffset(backRight))
        .build();
  }

  /**
   * Its devices on the CAN bus: the gyro, then each module's drive, steering and CANcoder, in
   * corner order, "front-left drive", "front-left steering", "front-left CANcoder", each motor
   * controller with the PDH channel that feeds it where its module's wiring says.
   */
  List<CanDevice> devices() {
    List<CanDevice> devices = new ArrayList<>();
    devices.add(
        new CanDevice(
            "the gyro", CanDevice.Kind.PIGEON_2, gyro.bus(), gyro.id(), Optional.empty()));
    List<ModuleWiring> modules = modules();
    for (int i = 0; i < modules.size(); i++) {
      String corner = Drivetrain.LABELS.get(i);
      ModuleWiring module = modules.get(i);
      devices.add(controller(corner + " drive", module.drive()));
      devices.add(controller(corner + " steering", module.steering()));
      if (module.encoder() instanceof MechanismWiring.Cancoder cancoder) {
        devices.add(
            new CanDevice(
                corner + " CANcoder",
                CanDevice.Kind.CANCODER,
                cancoder.bus(),
                cancoder.id(),
                Optional.empty()));
      }
    }
    return devices;
  }

  /** A module's motor controller as a device on the bus, fed where its wiring says. */
  private static CanDevice controller(String name, MechanismWiring.Device device) {
    return new CanDevice(
        name, device.controller().kind(), device.bus(), device.id(), device.feed());
  }

  /**
   * The drivetrain's gyro, a Pigeon 2: {@code PIGEON_2.can(CANIVORE, 1)}, then {@code
   * .yawScale(1.0012)} once the gyro scale procedure has measured it, and {@code .at(mount)} once
   * where it sits is measured.
   *
   * @param bus the CAN bus it's on: its drivetrain's modules', where Phoenix reads them together
   * @param id its CAN ID
   * @param yawScale its readings times this are the true rotation: 1 until the gyro scale procedure
   *     measures it (docs/calibration.md)
   * @param mount where it sits on the robot, how it's turned, and whether its accelerometer has
   *     been checked: localization compares the wheels with what it feels there
   */
  @ConfigRecord
  public record Gyro(CanBus bus, int id, double yawScale, ImuMount mount) {
    public Gyro {
      Checks.on(Gyro.class)
          .field("id", id, between(0, 62)) // a CAN ID
          .field("yawScale", yawScale, between(0.9, 1.1)) // off by more is a broken gyro
          .orThrow();
    }

    /**
     * The same gyro, its readings times this the true rotation, as the gyro scale procedure
     * measures it (docs/calibration.md): {@code .yawScale(1.0012)}.
     */
    public Gyro yawScale(double scale) {
      return SwerveWiringGyroBuilder.builder(this).yawScale(scale).build();
    }

    /** The same gyro, mounted where it's measured to sit, turned as it is. */
    public Gyro at(ImuMount mount) {
      return SwerveWiringGyroBuilder.builder(this).mount(mount).build();
    }
  }

  /**
   * A swerve module's absolute encoder, which its steering reads: a CANcoder, for a TalonFX's
   * steering, or an encoder on a SPARK's data port, for a SPARK's. A module's steering always runs
   * its loop on it, in its own way, so one told how a joint's loop would read it ({@code
   * .remote()}, {@code .fused()}, {@code .sync()}, {@code .closesLoop()}) is refused.
   */
  public sealed interface ModuleEncoder
      permits MechanismWiring.Cancoder, MechanismWiring.DataPortEncoder {
    /**
     * Added to its reading so that it reads zero with the wheel pointing straight ahead: Phoenix
     * Tuner X's swerve generator reports it for each module.
     */
    Angle offset();

    /** The same encoder, offset otherwise. */
    ModuleEncoder offset(Angle offset);
  }

  /**
   * One module's devices, as its corner writes them: {@code .frontLeft(TALON_FX.can(CANIVORE, 23),
   * TALON_FX.can(CANIVORE, 24), CANCODER.can(CANIVORE, 12).offset(Rotations.of(0.107)))}. A CAN ID
   * may be shared by devices of different types, but never by two motor controllers or two encoders
   * on one bus.
   *
   * @param drive the drive motor's controller: {@code .clockwise()} if it turns clockwise, seen
   *     facing its shaft, to roll the wheel forward, and {@code .pdh(...)} for the PDH channel that
   *     feeds it
   * @param steering the steering motor's: {@code .clockwise()} if it turns clockwise to turn the
   *     module counterclockwise, seen from above
   * @param encoder its absolute encoder: a CANcoder, {@code .clockwise()} if it counts clockwise,
   *     seen facing its LED, as the module turns counterclockwise; or the steering SPARK's data
   *     port
   */
  @ConfigRecord
  public record ModuleWiring(
      MechanismWiring.Device drive, MechanismWiring.Device steering, ModuleEncoder encoder) {
    /** A module's devices, as its corner names them. */
    static ModuleWiring of(
        MechanismWiring.Device drive, MechanismWiring.Device steering, ModuleEncoder encoder) {
      return SwerveWiringModuleWiringBuilder.builder()
          .drive(drive)
          .steering(steering)
          .encoder(encoder)
          .build();
    }

    /** The same module, its absolute encoder offset otherwise. */
    ModuleWiring encoderOffset(Angle offset) {
      return SwerveWiringModuleWiringBuilder.builder(this).encoder(encoder.offset(offset)).build();
    }
  }

  /**
   * A drivetrain's wiring as its block writes it, a corner at a time after its gyro: each stage
   * takes the next corner, and the last gives the drivetrain back, wired.
   */
  private record Written(Drivetrain drivetrain, Gyro gyro, List<ModuleWiring> corners) {
    /** The same, with one more corner's devices. */
    Written and(
        MechanismWiring.Device drive, MechanismWiring.Device steering, ModuleEncoder encoder) {
      List<ModuleWiring> more = new ArrayList<>(corners);
      more.add(ModuleWiring.of(drive, steering, encoder));
      return new Written(drivetrain, gyro, List.copyOf(more));
    }

    /** The drivetrain, wired as written, its wiring checked as a whole. */
    Drivetrain wired() {
      return drivetrain.withWiring(
          SwerveWiringBuilder.builder()
              .gyro(gyro)
              .frontLeft(corners.get(0))
              .frontRight(corners.get(1))
              .backLeft(corners.get(2))
              .backRight(corners.get(3))
              .build());
    }
  }

  /** The drivetrain's wiring under way, its gyro given: its front left module next. */
  public static final class FrontLeft {
    private final Written written;

    FrontLeft(Drivetrain drivetrain, Gyro gyro) {
      written = new Written(drivetrain, gyro, List.of());
    }

    /** The front left module's drive, steering and absolute encoder. */
    public FrontRight frontLeft(
        MechanismWiring.Device drive, MechanismWiring.Device steering, ModuleEncoder encoder) {
      return new FrontRight(written.and(drive, steering, encoder));
    }
  }

  /** The drivetrain's wiring under way: its front right module next. */
  public static final class FrontRight {
    private final Written written;

    private FrontRight(Written written) {
      this.written = written;
    }

    /** The front right module's drive, steering and absolute encoder. */
    public BackLeft frontRight(
        MechanismWiring.Device drive, MechanismWiring.Device steering, ModuleEncoder encoder) {
      return new BackLeft(written.and(drive, steering, encoder));
    }
  }

  /** The drivetrain's wiring under way: its back left module next. */
  public static final class BackLeft {
    private final Written written;

    private BackLeft(Written written) {
      this.written = written;
    }

    /** The back left module's drive, steering and absolute encoder. */
    public BackRight backLeft(
        MechanismWiring.Device drive, MechanismWiring.Device steering, ModuleEncoder encoder) {
      return new BackRight(written.and(drive, steering, encoder));
    }
  }

  /** The drivetrain's wiring under way: its back right module last. */
  public static final class BackRight {
    private final Written written;

    private BackRight(Written written) {
      this.written = written;
    }

    /**
     * The back right module's drive, steering and absolute encoder: the drivetrain, wired, its
     * wiring checked as a whole.
     */
    public Drivetrain backRight(
        MechanismWiring.Device drive, MechanismWiring.Device steering, ModuleEncoder encoder) {
      return written.and(drive, steering, encoder).wired();
    }
  }
}
