package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.SensorWiring.CANDI;
import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/** What a joint's motor controller reads on its limits, from the wiring its switches carry. */
class ControllerPinsTest {
  private static final CanBus BUS = CanBus.port(CanBus.Systemcore.CAN_S0);

  private static final Joint.Revolute PIVOT = StandardRobot.PIVOT;

  /** A limit switch in the robot's list of sensors, by a label of its own. */
  private static Sensor.Switch limit(String label, Joint joint, Sensor.Switch.Limit.End end) {
    return Sensor.Switch.builder()
        .label(label)
        .part(joint.parent())
        .kind(new Sensor.Switch.Limit(joint, Optional.empty(), end))
        .build();
  }

  /**
   * A joint's controller's limits come from the wiring its switches carry: its own limit switch on
   * its pin, stopped on; a beam on its other pin, read only; and on a TalonFX, its limit switch on
   * a CANdi on its bus, read over CAN as the limit for the switch's end. A SPARK can't read a
   * CANdi, and a joint with nothing on it has nothing.
   */
  @Test
  void eachJointsControllerLimitsComeFromTheWiring() {
    Joint.Revolute onSpark =
        PIVOT.motors(Motor.NEO.through(MechanismFixtures.gearing(25, 1)).on(SPARK_MAX.can(21)));
    Joint.Revolute onTalon =
        PIVOT.motors(
            Motor.FALCON_500.through(MechanismFixtures.gearing(25, 1)).on(TALON_FX.can(21)));
    Joint.Revolute roller =
        StandardRobot.ROLLER.motors(
            Motor.KRAKEN_X60.through(MechanismFixtures.gearing(3, 1)).on(TALON_FX.can(22)));
    Sensor.Switch up =
        limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER)
            .on(CANDI.can(BUS, 31).s2().normallyClosed());
    List<Sensor> wired =
        List.of(
            limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER)
                .on(
                    SensorWiring.ControllerLimit.of(PIVOT, SensorWiring.Direction.REVERSE)
                        .normallyClosed()),
            Sensor.Switch.beam("claw beam", PIVOT)
                .on(
                    SensorWiring.ControllerLimit.of(PIVOT, SensorWiring.Direction.FORWARD)
                        .normallyOpen()),
            up);
    RobotDefinition onSparks = robot(onSpark, roller, wired);
    RobotDefinition onTalons = robot(onTalon, roller, wired);

    ControllerPins spark = ControllerPins.of(onSparks, onSpark);
    ControllerPins talon = ControllerPins.of(onTalons, onTalon);

    assertThat(spark.reverse())
        .contains(new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_CLOSED, true));
    assertThat(spark.forward())
        .as("a beam, and a SPARK can't read the CANdi")
        .contains(new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false));
    assertThat(talon.forward())
        .as(
            "its pin's beam: the CANdi's upper switch would be its forward too, and the pin keeps it")
        .contains(new ControllerPins.OnPin(SensorWiring.Polarity.NORMALLY_OPEN, false));
    assertThat(ControllerPins.of(robot(onTalon, roller, List.of(up)), onTalon).forward())
        .contains(new ControllerPins.OnCandi(31, 2));
    assertThat(ControllerPins.of(onSparks, roller)).isEqualTo(ControllerPins.NONE);
  }

  /** The standard robot with its pivot and roller as given, and these sensors. */
  private static RobotDefinition robot(Joint pivot, Joint roller, List<Sensor> sensors) {
    return RobotDefinitionBuilder.builder(StandardRobot.robotOf(pivot, roller))
        .sensors(sensors)
        .build();
  }
}
