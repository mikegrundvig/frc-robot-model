package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FXS;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static com.michaelgrundvig.frc.world.robot.SwerveWiring.PIGEON_2;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Rotations;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.UnaryOperator;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

/**
 * A drivetrain's wiring is written as a joint writes its devices, a corner at a time after its
 * gyro, and checked when it's written down, not when the robot won't drive: each refusal names the
 * corner it's about.
 */
class SwerveWiringTest {
  private static final CanBus CANIVORE = CanBus.canivore("canivore");

  /** The template's own drivetrain, which TalonFXs and CANcoders drive. */
  private static final Drivetrain KRAKENS = ConfigFixtures.drivetrain();

  /** REV's MAXSwerve: a NEO Vortex driving and a NEO 550 steering, on SPARKs. */
  private static final Drivetrain MAX_SWERVE =
      Drivetrain.uniform(
          DrivetrainModuleBuilder.builder(ConfigFixtures.modules())
              .driveMotor(Motor.NEO_VORTEX)
              .steerMotor(Motor.NEO_550)
              .build(),
          Inches.of(11),
          Inches.of(9));

  /**
   * The drivetrain wired as the team's chassis is: TalonFXs on the CANivore, the front left's drive
   * 23, its steering 24 and its CANcoder 12, and so on, the right side's drives clockwise.
   */
  private static Drivetrain chassis() {
    return KRAKENS
        .gyro(PIGEON_2.can(CANIVORE, 1))
        .frontLeft(
            TALON_FX.can(CANIVORE, 23),
            TALON_FX.can(CANIVORE, 24),
            CANCODER.can(CANIVORE, 12).offset(Rotations.of(0.107)))
        .frontRight(
            TALON_FX.can(CANIVORE, 21).clockwise(),
            TALON_FX.can(CANIVORE, 22),
            CANCODER.can(CANIVORE, 11).offset(Rotations.of(-0.031)))
        .backLeft(
            TALON_FX.can(CANIVORE, 25),
            TALON_FX.can(CANIVORE, 26),
            CANCODER.can(CANIVORE, 13).offset(Rotations.of(-0.129)))
        .backRight(
            TALON_FX.can(CANIVORE, 27).clockwise(),
            TALON_FX.can(CANIVORE, 28),
            CANCODER.can(CANIVORE, 14).offset(Rotations.of(-0.384)));
  }

  private static SwerveWiring chassisWiring() {
    return chassis().wiring().orElseThrow();
  }

  /** The chassis's wiring with one corner's devices changed. */
  private static SwerveWiring changed(
      String corner, UnaryOperator<SwerveWiringModuleWiringBuilder> change) {
    return changed(chassis(), corner, change);
  }

  /** A drivetrain's wiring with one corner's devices changed, wired again. */
  private static SwerveWiring changed(
      Drivetrain wired, String corner, UnaryOperator<SwerveWiringModuleWiringBuilder> change) {
    SwerveWiring wiring = wired.wiring().orElseThrow();
    List<SwerveWiring.ModuleWiring> corners = new ArrayList<>(wiring.modules());
    int at = Drivetrain.LABELS.indexOf(corner);
    corners.set(at, change.apply(SwerveWiringModuleWiringBuilder.builder(corners.get(at))).build());
    return ConfigFixtures.wired(wired, wiring.gyro(), corners).wiring().orElseThrow();
  }

  /**
   * Each corner holds what its block wrote, in corner order; the bus and the kinds of controller
   * follow from the devices, and the gyro reads true and sits at the center until it's measured.
   */
  @Test
  void eachCornerHoldsWhatItsBlockWrote() {
    SwerveWiring wiring = chassisWiring();

    assertThat(wiring.modules())
        .extracting(module -> module.drive().id())
        .containsExactly(23, 21, 25, 27);
    assertThat(wiring.modules())
        .extracting(module -> module.drive().turnsClockwise())
        .containsExactly(false, true, false, true);
    assertThat(wiring.backRight().steering().id()).isEqualTo(28);
    assertThat(wiring.frontRight().encoder().offset()).isEqualTo(Rotations.of(-0.031));
    assertThat(wiring.canBus()).isEqualTo(CANIVORE);
    assertThat(wiring.driveController()).isEqualTo(TALON_FX);
    assertThat(wiring.steerController()).isEqualTo(TALON_FX);
    assertThat(wiring.gyro().id()).isEqualTo(1);
    assertThat(wiring.gyro().yawScale()).isEqualTo(1);
    assertThat(wiring.gyro().mount()).isEqualTo(ImuMount.UNCHECKED);
    assertThat(PIGEON_2.can(CANIVORE, 1).yawScale(1.0012).yawScale()).isEqualTo(1.0012);
  }

  /** A REV drivetrain's modules read their absolute encoders on their steering SPARKs' ports. */
  @Test
  void aRevDrivetrainSteersByItsDataPorts() {
    Drivetrain rev =
        ConfigFixtures.wired(MAX_SWERVE, CanBus.SYSTEMCORE, SPARK_FLEX, SPARK_MAX, 30, 20);
    SwerveWiring wiring = rev.wiring().orElseThrow();

    assertThat(wiring.driveController()).isEqualTo(SPARK_FLEX);
    assertThat(wiring.steerController()).isEqualTo(SPARK_MAX);
    assertThat(wiring.modules())
        .extracting(SwerveWiring.ModuleWiring::encoder)
        .allMatch(encoder -> encoder instanceof MechanismWiring.DataPortEncoder);
  }

  /** The chassis wired again with these corners' steerings saying {@code .foc()}. */
  private static Drivetrain steeringsWithFoc(String... corners) {
    SwerveWiring wiring = chassisWiring();
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>(wiring.modules());
    for (String corner : corners) {
      int at = Drivetrain.LABELS.indexOf(corner);
      SwerveWiring.ModuleWiring was = modules.get(at);
      modules.set(
          at, SwerveWiringModuleWiringBuilder.builder(was).steering(was.steering().foc()).build());
    }
    return ConfigFixtures.wired(chassis(), wiring.gyro(), modules);
  }

  /**
   * A drivetrain whose four drives say {@code .foc()} runs them on the Kraken X60's FOC curve, and
   * its steerings, which don't, on the Kraken X44's own.
   */
  @Test
  void drivesSaidFocRunOnTheirMotorsFocCurve() {
    SwerveWiring wiring = chassisWiring();
    List<SwerveWiring.ModuleWiring> modules = new ArrayList<>();
    for (SwerveWiring.ModuleWiring corner : wiring.modules()) {
      modules.add(
          SwerveWiringModuleWiringBuilder.builder(corner).drive(corner.drive().foc()).build());
    }
    Drivetrain foc = ConfigFixtures.wired(chassis(), wiring.gyro(), modules);

    assertThat(chassis().driveCommutation()).isEqualTo(MechanismWiring.Commutation.TRAPEZOIDAL);
    assertThat(foc.driveCommutation()).isEqualTo(MechanismWiring.Commutation.FOC);
    assertThat(foc.steerCommutation()).isEqualTo(MechanismWiring.Commutation.TRAPEZOIDAL);
    for (int corner = 0; corner < 4; corner++) {
      assertThat(foc.driveMotors(corner).model().freeSpeed)
          .isEqualTo(DCMotor.getKrakenX60Foc(1).freeSpeed);
      assertThat(chassis().driveMotors(corner).model().freeSpeed)
          .isEqualTo(DCMotor.getKrakenX60(1).freeSpeed);
      assertThat(foc.steerMotors(corner).model().freeSpeed)
          .isEqualTo(DCMotor.getKrakenX44(1).freeSpeed);
    }
  }

  /** A wiring refused, and how its refusal starts, word for word. */
  private record Refused(String scenario, Runnable wiring, String message) {}

  /**
   * Each refusal names what it's about: a corner's own value, its corner; a rule across the
   * corners, the drivetrain's wiring; the gyro's own value, the gyro.
   */
  @Test
  void eachRefusalNamesWhatItsAbout() {
    List<Refused> rows =
        List.of(
            new Refused(
                "a steering's CAN ID off the bus",
                () -> changed("back-right", c -> c.steering(TALON_FX.can(CANIVORE, 70))),
                "ModuleWiring[back-right].steering: CAN ID 70 must be from 0 to 62"),
            new Refused(
                "a drive's CAN ID off the bus",
                () -> changed("front-left", c -> c.drive(TALON_FX.can(CANIVORE, 63))),
                "ModuleWiring[front-left].drive: CAN ID 63 must be from 0 to 62"),
            new Refused(
                "a CANcoder's CAN ID off the bus",
                () ->
                    changed(
                        "back-left",
                        c -> c.encoder(CANCODER.can(CANIVORE, 63).offset(Rotations.of(0)))),
                "ModuleWiring[back-left].encoder: CAN ID 63 must be from 0 to 62"),
            new Refused(
                "an offset of more than a turn",
                () ->
                    changed(
                        "front-right",
                        c -> c.encoder(CANCODER.can(CANIVORE, 11).offset(Rotations.of(1.5)))),
                "ModuleWiring[front-right].encoder: offset 1.5 R must be from -1 R to 1 R"),
            new Refused(
                "a CANcoder read .remote()",
                () ->
                    changed(
                        "front-right",
                        c ->
                            c.encoder(CANCODER.can(CANIVORE, 11).offset(Rotations.of(0)).remote())),
                "ModuleWiring[front-right].encoder: is read .remote(), and a module chooses how its"
                    + " steering reads its encoder itself, closing its loop on it: leave .remote()"
                    + " out"),
            new Refused(
                "a CANcoder read .fused()",
                () ->
                    changed(
                        "front-right",
                        c -> c.encoder(CANCODER.can(CANIVORE, 11).offset(Rotations.of(0)).fused())),
                "ModuleWiring[front-right].encoder: is read .fused(), and a module chooses how its"
                    + " steering reads its encoder itself, closing its loop on it: leave .fused()"
                    + " out"),
            new Refused(
                "a CANcoder read .sync()",
                () ->
                    changed(
                        "front-right",
                        c -> c.encoder(CANCODER.can(CANIVORE, 11).offset(Rotations.of(0)).sync())),
                "ModuleWiring[front-right].encoder: is read .sync(), and a module chooses how its"
                    + " steering reads its encoder itself, closing its loop on it: leave .sync()"
                    + " out"),
            new Refused(
                "a data port encoder that .closesLoop()",
                () ->
                    changed(
                        ConfigFixtures.wired(
                            MAX_SWERVE, CanBus.SYSTEMCORE, SPARK_FLEX, SPARK_MAX, 30, 20),
                        "back-left",
                        c -> c.encoder(DATA_PORT.offset(Rotations.of(0)).closesLoop())),
                "ModuleWiring[back-left].encoder: is told .closesLoop(), and a module chooses how its"
                    + " steering reads its encoder itself, closing its loop on it: leave"
                    + " .closesLoop() out"),
            new Refused(
                "two TalonFXs at one ID",
                () -> changed("back-right", c -> c.steering(TALON_FX.can(CANIVORE, 22))),
                "SwerveWiring.modules: CAN ID 22 is used by two TalonFXs"),
            new Refused(
                "a device off the gyro's bus",
                () -> changed("back-left", c -> c.drive(TALON_FX.can(25))),
                "SwerveWiring.modules: a drivetrain's devices are on one bus, the gyro's, the"
                    + " CANivore canivore: back-left's aren't"),
            new Refused(
                "drives on two kinds of controller",
                () -> changed("front-right", c -> c.drive(SPARK_MAX.can(CANIVORE, 21))),
                "SwerveWiring.modules: a drivetrain's drive motors are on one kind of controller:"
                    + " front-left's is on a TalonFX, and front-right's on a SPARK MAX"),
            new Refused(
                "one drive with FOC",
                () -> changed("front-right", c -> c.drive(TALON_FX.can(CANIVORE, 21).foc())),
                "SwerveWiring.modules: a drivetrain's drives are all driven with FOC or none is:"
                    + " front-right's say .foc(): say it on all four or on none"),
            new Refused(
                "two steerings with FOC",
                () -> steeringsWithFoc("front-left", "back-left"),
                "SwerveWiring.modules: a drivetrain's steerings are all driven with FOC or none is:"
                    + " front-left's and back-left's say .foc(): say it on all four or on none"),
            new Refused(
                "a TalonFX steering by a data port",
                () -> changed("back-left", c -> c.encoder(DATA_PORT.offset(Rotations.of(0)))),
                "SwerveWiring.modules: a TalonFX steers by a CANcoder: give each module its own,"
                    + " CANCODER.can(bus, id).offset(...)"),
            new Refused(
                "a SPARK steering by a CANcoder",
                () ->
                    changed(
                        ConfigFixtures.wired(
                            MAX_SWERVE, CanBus.SYSTEMCORE, SPARK_FLEX, SPARK_MAX, 30, 20),
                        "front-left",
                        c -> c.encoder(CANCODER.can(20).offset(Rotations.of(0)))),
                "SwerveWiring.modules: a SPARK steers by the absolute encoder on its data port"),
            new Refused(
                "a TalonFXS",
                () -> ConfigFixtures.wired(KRAKENS, CanBus.SYSTEMCORE, TALON_FXS, TALON_FX, 30, 20),
                "SwerveWiring.modules: a TalonFXS doesn't run a swerve module in the template yet"),
            new Refused(
                "a gyro off the bus",
                () -> PIGEON_2.can(CANIVORE, 63),
                "Gyro.id: 63 must be from 0 to 62"),
            new Refused(
                "a gyro whose scale says it's broken",
                () -> PIGEON_2.can(CANIVORE, 1).yawScale(1.2),
                "Gyro.yawScale: 1.2 must be from 0.9 to 1.1"));

    for (Refused row : rows) {
      assertThatThrownBy(row.wiring()::run)
          .as(row.scenario())
          .isInstanceOf(IllegalArgumentException.class)
          .message()
          .startsWith(row.message());
    }
  }

  /**
   * Devices of different kinds may share an ID: the gyro, the front left's drive and its CANcoder
   * are each 1.
   */
  @Test
  void devicesOfDifferentKindsMayShareAnId() {
    Drivetrain wired = ConfigFixtures.wired(KRAKENS, CanBus.SYSTEMCORE, TALON_FX, TALON_FX, 1, 1);

    assertThat(wired.wiring().orElseThrow().devices())
        .filteredOn(device -> device.id() == 1)
        .extracting(CanDevice::name)
        .containsExactly("the gyro", "front-left drive", "front-left CANcoder");
  }

  /** A module's motor controllers say the PDH channel feeding them as a joint's do. */
  @Test
  void aModulesControllersSayTheirFeeds() {
    SwerveWiring wiring =
        changed("front-left", c -> c.drive(TALON_FX.can(CANIVORE, 23).pdh(0, Amps.of(40))));

    assertThat(wiring.devices())
        .filteredOn(device -> device.name().equals("front-left drive"))
        .extracting(CanDevice::feed)
        .containsExactly(Optional.of(Pdh.feed(0, Amps.of(40))));
  }

  /**
   * A practice robot's offsets replace each corner's, front left to back right, and keep the rest
   * of its wiring.
   */
  @Test
  void newOffsetsKeepTheRestOfTheWiring() {
    Drivetrain practice =
        chassis()
            .encoderOffsets(
                Rotations.of(0.1), Rotations.of(0.2), Rotations.of(0.3), Rotations.of(0.4));
    SwerveWiring wiring = practice.wiring().orElseThrow();

    assertThat(wiring.modules())
        .extracting(module -> module.encoder().offset().in(Rotations))
        .containsExactly(0.1, 0.2, 0.3, 0.4);
    assertThat(wiring.modules())
        .extracting(module -> module.drive().turnsClockwise())
        .containsExactly(false, true, false, true);
    assertThat(wiring.gyro()).isEqualTo(chassisWiring().gyro());
  }

  @Test
  void aCanivoreHasAName() {
    assertThatThrownBy(() -> CanBus.canivore(" ")).hasMessage("Canivore.name: must not be blank");
  }
}
