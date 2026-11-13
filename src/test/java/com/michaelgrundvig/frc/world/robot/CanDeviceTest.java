package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_FLEX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.wpilib.units.Units.Meters;

import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Two parts of the program on one motor: the survey found an intake and a roller each building CAN
 * ID 3 and both driving it every loop, which no command's requirements could sort out. The
 * definition refuses two devices of a kind on one bus at one ID, over its one list of them.
 */
class CanDeviceTest {
  private static final Output RUNNING = Output.volts("running", 6);
  private static final CanBus CANIVORE = CanBus.canivore("drive");

  /** A roller on one controller: a Kraken on a TalonFX, a NEO Vortex on a SPARK. */
  private static RevoluteJoint roller(String label, MechanismWiring.Device controller) {
    Motor motor = controller.controller() == TALON_FX ? Motor.KRAKEN_X60 : Motor.NEO_VORTEX;
    return Joint.roller(label, Part.CHASSIS)
        .at(Meters.of(0.25), Meters.of(0), Meters.of(0.32))
        .outputs(RUNNING)
        .motors(motor.through(Gearing.of(3)).on(controller));
  }

  /** The test robot's drivetrain on Systemcore's CAN: TalonFXs 1 to 8, CANcoders and gyro 1. */
  private static RobotDefinition robot(Joint... joints) {
    return RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of()))
        .drivetrain(
            ConfigFixtures.wired(
                ConfigFixtures.drivetrain(), CanBus.SYSTEMCORE, TALON_FX, TALON_FX, 1, 1))
        .joints(List.of(joints))
        .build()
        .checked();
  }

  @Test
  void twoMechanismsOnOneMotorAreNamed() {
    assertThatIllegalArgumentException()
        .isThrownBy(() ->
            robot(
                roller("coral intake", SPARK_MAX.can(13)),
                roller("algae intake", SPARK_FLEX.can(13))))
        .withMessage(
            "RobotDefinition[test].devices: two SPARKs on Systemcore's CAN_S0 port share CAN ID 13,"
                + " coral intake motor 13 and algae intake motor 13: give each its own");
  }

  @Test
  void aMechanismOnADrivetrainsIdIsNamed() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(roller("pivot", TALON_FX.can(5))))
        .withMessage(
            "RobotDefinition[test].devices: two TalonFXs on Systemcore's CAN_S0 port share CAN ID"
                + " 5, back-left drive and pivot motor 5: give each its own");
  }

  /**
   * Devices of other kinds, or on other buses, may share an ID: the drivetrain's own CANcoder 1,
   * drive motor 1 and gyro 1 do, and a SPARK at 1 beside them.
   */
  @Test
  void devicesOfOtherKindsOrOnOtherBusesMayShareAnId() {
    RobotDefinition robot =
        robot(roller("pivot", SPARK_MAX.can(1)), roller("roller", TALON_FX.can(CANIVORE, 1)));

    assertThat(robot.devices())
        .filteredOn(device -> device.id() == 1)
        .extracting(CanDevice::name)
        .containsExactly(
            "the gyro",
            "front-left drive",
            "front-left CANcoder",
            "pivot motor 1",
            "roller motor 1");
  }
}
