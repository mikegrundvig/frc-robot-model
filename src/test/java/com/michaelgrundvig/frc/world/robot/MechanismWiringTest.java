package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;

/** A mechanism's wiring is checked when it's written down, not when the mechanism won't move. */
class MechanismWiringTest {
  /** A joint's motors on TalonFXs at these IDs, the first leading. */
  private static RevoluteJoint onTalons(MechanismWiring.Device... controllers) {
    return Joint.pivot("arm", Part.CHASSIS, Joint.PITCH_UP)
        .at(Meters.of(0), Meters.of(0), Meters.of(0))
        .motors(Motor.KRAKEN_X60.through(Gearing.of(60)).on(controllers));
  }

  @Test
  void theFirstMotorLeads() {
    RevoluteJoint arm =
        onTalons(
            MechanismWiring.Controller.TALON_FX.can(11),
            MechanismWiring.Controller.TALON_FX.can(12));

    assertThat(arm.driven().motors().orElseThrow().controllers().getFirst().id()).isEqualTo(11);
    assertThat(arm.driven().motors().orElseThrow().count()).isEqualTo(2);
  }

  /**
   * A motor its block wires to nothing runs on the controller it plugs into: a NEO's a SPARK MAX, a
   * NEO Vortex's a SPARK Flex, a Minion's a TalonFXS, any other's a TalonFX.
   */
  @Test
  void anUnwiredMotorStandsInOnItsOwnController() {
    assertThat(MechanismWiring.Controller.standIn(Motor.NEO))
        .isEqualTo(MechanismWiring.Controller.SPARK_MAX);
    assertThat(MechanismWiring.Controller.standIn(Motor.NEO_550))
        .isEqualTo(MechanismWiring.Controller.SPARK_MAX);
    assertThat(MechanismWiring.Controller.standIn(Motor.NEO_VORTEX))
        .isEqualTo(MechanismWiring.Controller.SPARK_FLEX);
    assertThat(MechanismWiring.Controller.standIn(Motor.MINION))
        .isEqualTo(MechanismWiring.Controller.TALON_FXS);
    assertThat(MechanismWiring.Controller.standIn(Motor.KRAKEN_X60))
        .isEqualTo(MechanismWiring.Controller.TALON_FX);
  }

  /**
   * A joint's motors are at least one, and no two of its controllers share an ID: the definition
   * says so as it lists every device on the bus once.
   */
  @Test
  void itTakesAMotorAndNoIdTwice() {
    assertThatThrownBy(() -> Motor.KRAKEN_X60.through(Gearing.of(60)).count(0))
        .hasMessage("Motors.count: 0 must be positive");
    assertThatThrownBy(() ->
            RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of()))
                .joints(
                    List.of(
                        onTalons(
                            MechanismWiring.Controller.TALON_FX.can(11),
                            MechanismWiring.Controller.TALON_FX.can(11))))
                .build()
                .checked())
        .hasMessageContaining(
            "two TalonFXs on Systemcore's CAN_S0 port share CAN ID 11, arm motor 11 and arm motor"
                + " 11: give each its own");
  }

  /** A joint's device refused, and how, word for word. */
  private record Refused(String scenario, Runnable joint, String message) {}

  /**
   * IDs are on the bus and an offset within a turn, and a refusal names the joint the device is in:
   * a device doesn't check them as it's made, so that the joint holding it can.
   */
  @Test
  void idsAreOnTheBusAndAnOffsetWithinATurnRefusedUnderTheirJoint() {
    RevoluteJoint arm = onTalons(MechanismWiring.Controller.TALON_FX.can(11));
    List<Refused> rows =
        List.of(
            new Refused(
                "a motor controller off the bus",
                () ->
                    onTalons(
                        MechanismWiring.Controller.TALON_FX.can(11),
                        MechanismWiring.Controller.TALON_FX.can(63)),
                "Motors[arm].controllers: CAN ID 63 must be from 0 to 62"),
            new Refused(
                "a CANcoder's offset of more than a turn",
                () -> arm.absoluteEncoder(CANCODER.can(5).offset(Rotations.of(1.5))),
                "AbsoluteEncoder[arm].cancoder: offset 1.5 R must be from -1 R to 1 R"),
            new Refused(
                "a CANcoder off the bus",
                () -> arm.absoluteEncoder(CANCODER.can(63).offset(Rotations.of(0))),
                "AbsoluteEncoder[arm].cancoder: CAN ID 63 must be from 0 to 62"),
            new Refused(
                "a data port's offset of more than a turn",
                () -> arm.absoluteEncoder(DATA_PORT.offset(Rotations.of(-1.5))),
                "AbsoluteEncoder[arm].dataPort: offset -1.5 R must be from -1 R to 1 R"));

    for (Refused row : rows) {
      assertThatThrownBy(row.joint()::run).as(row.scenario()).hasMessage(row.message());
    }
  }

  /**
   * FOC is said on a controller, once, and only where it can be: on CTRE's controllers, on every
   * controller of a joint or none, and for a motor WPILib has an FOC curve of.
   */
  @Test
  void focIsSaidOnceOnEveryControllerOfAJointThatCanRunIt() {
    List<Refused> rows =
        List.of(
            new Refused(
                "the leader with FOC and a follower without",
                () ->
                    onTalons(
                        MechanismWiring.Controller.TALON_FX.can(11).foc(),
                        MechanismWiring.Controller.TALON_FX.can(12)),
                "Motors.controllers: a joint's motors are all driven with FOC or none is: its"
                    + " leader says .foc(), and CAN ID 12 doesn't: say .foc() on every controller"
                    + " or on none"),
            new Refused(
                "a follower with FOC and the leader without",
                () ->
                    onTalons(
                        MechanismWiring.Controller.TALON_FX.can(11),
                        MechanismWiring.Controller.TALON_FX.can(12).foc()),
                "Motors.controllers: a joint's motors are all driven with FOC or none is: its"
                    + " leader doesn't, and CAN ID 12 says .foc(): say .foc() on every controller"
                    + " or on none"),
            new Refused(
                "a Minion, which has no FOC curve",
                () ->
                    Motor.MINION
                        .through(Gearing.of(60))
                        .on(MechanismWiring.Controller.TALON_FXS.can(11).foc()),
                "Motors.controllers: a Minion has no FOC curve to run it on: .foc() is for a"
                    + " Kraken X60, a Kraken X44 or a Falcon 500"),
            new Refused(
                "a NEO on a TalonFXS, which has no FOC curve",
                () ->
                    Motor.NEO
                        .through(Gearing.of(60))
                        .on(MechanismWiring.Controller.TALON_FXS.can(11).foc()),
                "Motors.controllers: a NEO has no FOC curve to run it on: .foc() is for a Kraken"
                    + " X60, a Kraken X44 or a Falcon 500"),
            new Refused(
                "a SPARK MAX, which has no FOC",
                () -> MechanismWiring.Controller.SPARK_MAX.can(11).foc(),
                "a SPARK MAX at CAN ID 11 has no FOC, which is CTRE's: drop .foc(), or put the"
                    + " motor on a TalonFX"),
            new Refused(
                "a SPARK Flex, which has no FOC",
                () -> MechanismWiring.Controller.SPARK_FLEX.can(11).foc(),
                "a SPARK Flex at CAN ID 11 has no FOC, which is CTRE's: drop .foc(), or put the"
                    + " motor on a TalonFX"),
            new Refused(
                "FOC said twice",
                () -> MechanismWiring.Controller.TALON_FX.can(11).foc().foc(),
                "a TalonFX at CAN ID 11 already says .foc(): say it once"));

    for (Refused row : rows) {
      assertThatThrownBy(row.joint()::run).as(row.scenario()).hasMessage(row.message());
    }
  }

  /**
   * A joint whose controllers say {@code .foc()} runs on its motor's FOC curve, which the simulator
   * and the model's ideal gains take: more torque, a little less speed. One that doesn't, on the
   * plain curve.
   */
  @Test
  void aJointSaidFocRunsOnItsMotorsFocCurve() {
    RevoluteJoint plain =
        onTalons(
            MechanismWiring.Controller.TALON_FX.can(11),
            MechanismWiring.Controller.TALON_FX.can(12));
    RevoluteJoint foc =
        onTalons(
            MechanismWiring.Controller.TALON_FX.can(11).foc(),
            MechanismWiring.Controller.TALON_FX.can(12).foc());

    Actuator.MotorGroup plainMotors =
        (Actuator.MotorGroup) JointParts.drives(plain).get(0).actuator();
    Actuator.MotorGroup focMotors = (Actuator.MotorGroup) JointParts.drives(foc).get(0).actuator();

    assertThat(plainMotors.commutation()).isEqualTo(MechanismWiring.Commutation.TRAPEZOIDAL);
    assertThat(plainMotors.model().stallTorque).isEqualTo(DCMotor.getKrakenX60(2).stallTorque);
    assertThat(focMotors.commutation()).isEqualTo(MechanismWiring.Commutation.FOC);
    assertThat(focMotors.motor()).isEqualTo(Motor.KRAKEN_X60);
    assertThat(focMotors.model().stallTorque).isEqualTo(DCMotor.getKrakenX60Foc(2).stallTorque);
    assertThat(focMotors.model().freeSpeed).isEqualTo(DCMotor.getKrakenX60Foc(2).freeSpeed);
  }

  /**
   * A data-port encoder says what its controller reads raw where it reads a number of turns, its
   * offset and direction undone, from 0 to 1, a turn on or back the same: what a SPARK is told to
   * read as zero. The joint has one absolute encoder, so the wiring reads it one way.
   */
  @Test
  void aDataPortEncoderGivesItsRawReadingAndIsTheOnlyReader() {
    MechanismWiring.DataPortEncoder port = DATA_PORT.offset(Rotations.of(-0.25)).inverted();
    MechanismWiring.Cancoder cancoder = CANCODER.can(5).offset(Rotations.of(0));

    // Reversed, offset a quarter turn back: reading -0.35 turns, it reads 0.1 raw.
    assertThat(port.raw(-0.35)).isCloseTo(0.1, within(1e-12));
    assertThat(port.raw(0.65)).isCloseTo(0.1, within(1e-12));
    assertThatThrownBy(() ->
            DrivenAbsoluteEncoderBuilder.builder()
                .ratio(1)
                .cancoder(cancoder)
                .dataPort(port)
                .build())
        .hasMessage(
            "AbsoluteEncoder.port: names more than one of a CANcoder, a data port and a Systemcore"
                + " port: it's read one way");
  }

  /**
   * A port encoder, or a data-port one, tells its motor where the joint is, once, unless its wiring
   * says the joint's loop closes on it; nothing else about it changes.
   */
  @Test
  void anEncodersLoopClosesOnItOnlyWhenItsWiringSays() {
    MechanismWiring.PortEncoder once =
        MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.82)).inverted();
    MechanismWiring.PortEncoder closing = once.closesLoop();

    assertThat(once.closedLoop()).isFalse();
    assertThat(MechanismWiring.PortEncoder.analog(2, 10).closedLoop()).isFalse();
    assertThat(closing.closedLoop()).isTrue();
    assertThat(closing.read(0.25)).isEqualTo(once.read(0.25));
    assertThat(Driven.AbsoluteEncoder.of().on(once).closedLoop()).isFalse();
    assertThat(Driven.AbsoluteEncoder.of().on(closing).closedLoop()).isTrue();
    MechanismWiring.DataPortEncoder dataPort = DATA_PORT.offset(Rotations.of(-0.25)).inverted();
    assertThat(dataPort.closesLoop().closedLoop()).isTrue();
    assertThat(dataPort.closesLoop().raw(0.4)).isEqualTo(dataPort.raw(0.4));
    assertThat(Driven.AbsoluteEncoder.of().on(dataPort).closedLoop()).isFalse();
    assertThat(Driven.AbsoluteEncoder.of().on(dataPort.closesLoop()).closedLoop()).isTrue();
  }

  /**
   * A CANcoder is named as a motor's controller is, by its bus and ID, with its offset: read once
   * at startup unless it says the TalonFX's loop runs on it, counterclockwise unless it says
   * otherwise.
   */
  @Test
  void aCancoderIsNamedByItsBusAndIdWithItsOffset() {
    CanBus canivore = CanBus.canivore("canivore");
    MechanismWiring.Cancoder seeded = CANCODER.can(canivore, 33).offset(Rotations.of(0.82));

    assertThat(seeded)
        .isEqualTo(
            MechanismWiring.Cancoder.builder()
                .bus(canivore)
                .id(33)
                .offset(Rotations.of(0.82))
                .turnsClockwise(false)
                .feedback(MechanismWiring.Feedback.ROTOR)
                .build());
    assertThat(CANCODER.can(33).offset(Rotations.of(0)).bus()).isEqualTo(CanBus.SYSTEMCORE);
    assertThat(CANCODER.can(canivore, 33).offset(Rotations.of(0.82)).clockwise().turnsClockwise())
        .isTrue();
    assertThat(seeded.remote().feedback()).isEqualTo(MechanismWiring.Feedback.REMOTE);
    assertThat(seeded.fused().feedback()).isEqualTo(MechanismWiring.Feedback.FUSED);
    assertThat(seeded.sync().feedback()).isEqualTo(MechanismWiring.Feedback.SYNC);
    assertThat(seeded.fused().offset()).isEqualTo(seeded.offset());
  }

  /** A data-port encoder is named by its offset: read once, its reading growing with the joint. */
  @Test
  void aDataPortEncoderIsNamedByItsOffset() {
    MechanismWiring.DataPortEncoder once = DATA_PORT.offset(Rotations.of(0.3));

    assertThat(once)
        .isEqualTo(
            MechanismWiring.DataPortEncoder.builder()
                .offset(Rotations.of(0.3))
                .reversed(false)
                .closedLoop(false)
                .build());
    assertThat(once.inverted().reversed()).isTrue();
    assertThat(once.inverted().closesLoop())
        .isEqualTo(
            MechanismWiring.DataPortEncoder.builder()
                .offset(Rotations.of(0.3))
                .reversed(true)
                .closedLoop(true)
                .build());
  }
}
