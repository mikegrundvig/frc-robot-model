package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.CANCODER;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.SPARK_MAX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static com.michaelgrundvig.frc.world.robot.MechanismWiring.DATA_PORT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Value;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Shape;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Driven.AbsoluteEncoder;
import com.michaelgrundvig.frc.world.robot.Driven.LimitSwitch;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * A joint's block: everything about one mechanism in one place, its shape and places, its motors
 * and their wiring, its own sensors, and how the program drives it.
 */
class JointBlockTest {
  private static final Translation3d AXLE = new Translation3d(0.25, 0, 0.32);
  private static final Position IN = Position.degrees("in", 1.5);
  private static final Position OUT = Position.degrees("out", 130);
  private static final Output INTAKING = Output.rpm("intaking", 2000);
  private static final CanBus CANIVORE = CanBus.canivore("canivore");

  @Test
  void aPivotTurnsBetweenItsStopsFromWhereItRests() {
    Link arm = Link.box("arm", new Translation3d(0, 0, 0.2), new Translation3d(0.05, 0.5, 0.36));

    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135))
            .rest(IN)
            .carries(arm)
            .positions(IN, OUT);

    assertThat(pivot.origin().getTranslation()).isEqualTo(AXLE);
    assertThat(pivot.axis()).isEqualTo(Joint.PITCH_DOWN);
    assertThat(pivot.range()).contains(Range.degrees(0, 135));
    assertThat(pivot.restPosition()).isCloseTo(Math.toRadians(1.5), within(1e-12));
    assertThat(pivot.link()).isEqualTo(arm);
    assertThat(pivot.positions()).containsExactly(IN, OUT);
    assertThat(pivot.outputs()).isEmpty();
    assertThat(pivot.driven()).isEqualTo(Driven.NONE);
    assertThat(pivot.ownDrives()).isEmpty();
  }

  @Test
  void aRollerTurnsFreelyFromZeroAtItsOutputs() {
    Joint.Revolute roller = Joint.roller("intake", Part.CHASSIS, AXLE).outputs(INTAKING);

    assertThat(roller.range()).isEmpty();
    assertThat(roller.restPosition()).isZero();
    assertThat(roller.outputs()).containsExactly(INTAKING);
    assertThat(roller.link()).isEqualTo(Link.unseen("intake"));
    assertThat(roller.rest(Position.degrees("home", 10)).restPosition())
        .as("a place it hasn't named, where it is")
        .isCloseTo(Math.toRadians(10), within(1e-12));
  }

  @Test
  void anElevatorSlidesStraightUpOntoItsDrum() {
    Position clear = Position.of("clear", Inches.of(20));

    Joint.Prismatic elevator =
        Joint.elevator("elevator", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 1.2))
            .rest(Meters.of(0))
            .positions(clear)
            .motors(
                Motor.KRAKEN_X60
                    .through(Gearing.of(10))
                    .drum(Inches.of(1))
                    .on(TALON_FX.can(5), TALON_FX.can(6)))
            .limitSwitch(LimitSwitch.lower().at(Meters.of(0)));

    assertThat(elevator.axis()).isEqualTo(Vector3.UP);
    assertThat(elevator.range()).contains(Range.meters(0, 1.2));
    assertThat(elevator.position("clear").value()).isCloseTo(0.508, within(1e-12));
    Drive drive = elevator.ownDrives().getFirst();
    assertThat(drive.transmission())
        .isEqualTo(
            Transmission.Drum.builder()
                .joint(elevator)
                .gearing(Gearing.of(10))
                .pitchRadius(Inches.of(1))
                .build());
    assertThat(((Actuator.Motors) drive.actuator()).count()).isEqualTo(2);
    assertThat(elevator.switchAt(Sensor.Switch.Limit.End.LOWER).label())
        .isEqualTo("elevator lower limit");
    assertThat(elevator.driven().motors().orElseThrow().controllers())
        .containsExactly(TALON_FX.can(5), TALON_FX.can(6));
  }

  /**
   * Its motors, unwired, run in the simulator: a drive under its label, and nothing to wire. On
   * controllers, the leader first, its absolute encoder read through them.
   */
  @Test
  void itsMotorsAreItsOwnDrive() {
    Joint.Revolute sim =
        Joint.pivot("arm", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(-30, 120))
            .rest(Degrees.of(-20))
            .motors(Motor.KRAKEN_X60.through(Gearing.of(60)).count(2));
    Joint.Revolute wired =
        sim.motors(
                motors ->
                    motors.on(TALON_FX.can(CANIVORE, 20), TALON_FX.can(CANIVORE, 21).clockwise()))
            .absoluteEncoder(CANCODER.can(CANIVORE, 22).offset(Rotations.of(0.12)).fused());

    Drive drive = sim.ownDrives().getFirst();
    assertThat(drive.label()).isEqualTo("arm");
    assertThat(drive.actuator()).isEqualTo(new Actuator.Motors(Motor.KRAKEN_X60, 2));
    assertThat(drive.transmission())
        .isEqualTo(Transmission.Geared.builder().joint(sim).gearing(Gearing.of(60)).build());
    assertThat(sim.driven().motors().orElseThrow().controllers()).isEmpty();
    assertThat(wired.driven().motors().orElseThrow().controllers())
        .containsExactly(TALON_FX.can(CANIVORE, 20), TALON_FX.can(CANIVORE, 21).clockwise());
    assertThat(
            wired
                .driven()
                .absoluteEncoder()
                .flatMap(Driven.AbsoluteEncoder::cancoder)
                .orElseThrow())
        .isEqualTo(CANCODER.can(CANIVORE, 22).offset(Rotations.of(0.12)).fused());
    Sensor.Encoder encoder = (Sensor.Encoder) wired.ownSensors().getFirst();
    assertThat(encoder.label()).isEqualTo("arm absolute encoder");
    assertThat(encoder.absolute()).isTrue();
    assertThat(encoder.joint()).isEqualTo(wired);
  }

  /** An absolute encoder in its block, each way it's read, is the one written in full. */
  @Test
  void anAbsoluteEncoderInItsBlockIsTheOneWrittenInFull() {
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(-30, 120))
            .rest(Degrees.of(-20))
            .motors(Motor.KRAKEN_X60.through(Gearing.of(60)).on(TALON_FX.can(20)));
    MechanismWiring.Cancoder cancoder = CANCODER.can(22).offset(Rotations.of(0.12)).fused();
    MechanismWiring.DataPortEncoder dataPort = DATA_PORT.offset(Rotations.of(0.3));
    MechanismWiring.PortEncoder pot =
        MechanismWiring.PortEncoder.analog(2, 10).offset(Rotations.of(-2.5));

    assertThat(arm.absoluteEncoder(AbsoluteEncoder.of()).driven().absoluteEncoder())
        .contains(Driven.AbsoluteEncoder.of());
    assertThat(arm.absoluteEncoder(cancoder).driven().absoluteEncoder())
        .contains(Driven.AbsoluteEncoder.of().on(cancoder));
    assertThat(arm.absoluteEncoder(dataPort).driven().absoluteEncoder())
        .contains(Driven.AbsoluteEncoder.of().on(dataPort));
    assertThat(arm.absoluteEncoder(pot).driven().absoluteEncoder())
        .contains(Driven.AbsoluteEncoder.of().on(pot));
    assertThat(Driven.AbsoluteEncoder.of().wired()).isFalse();
    Sensor.Encoder read = (Sensor.Encoder) arm.absoluteEncoder(pot).ownSensors().getFirst();
    assertThat(read.absolute()).isTrue();
    assertThat(read.terms().getFirst().ratio()).isCloseTo(0.1, within(1e-12));
    assertThat(arm.absoluteEncoder(cancoder).driven().absoluteEncoder().orElseThrow().cancoder())
        .contains(cancoder);
  }

  /**
   * An absolute encoder is on the axle or geared, and read any of three ways: a CANcoder through
   * the joint's TalonFX, its SPARK's data port, or a Systemcore port the robot reads, which needs
   * no motors wired.
   */
  @Test
  void anAbsoluteEncoderIsGearedAndReadAnyOfThreeWays() {
    Joint.Revolute turret =
        Joint.roller("turret", Part.CHASSIS, AXLE, Vector3.UP)
            .absoluteEncoder(Driven.AbsoluteEncoder.of().ratio(92.0 / 18));
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(0, 90))
            .rest(Degrees.of(0))
            .absoluteEncoder(
                MechanismWiring.PortEncoder.dutyCycle(1).offset(Rotations.of(0.82)).inverted());
    MechanismWiring.PortEncoder pot =
        MechanismWiring.PortEncoder.analog(2, 10).offset(Rotations.of(-2.5));
    Joint.Revolute winch =
        Joint.roller("winch", Part.CHASSIS, AXLE)
            .absoluteEncoder(Driven.AbsoluteEncoder.of().ratio(4).on(pot));

    Sensor.Encoder geared = (Sensor.Encoder) turret.ownSensors().getFirst();
    assertThat(geared.terms().getFirst().ratio()).isEqualTo(92.0 / 18);
    assertThat(geared.absolute()).isTrue();
    MechanismWiring.PortEncoder port =
        pivot.driven().absoluteEncoder().orElseThrow().port().orElseThrow();
    assertThat(port.signal()).isEqualTo(MechanismWiring.PortEncoder.Signal.DUTY_CYCLE);
    assertThat(port.read(0.25)).isCloseTo(0.57, within(1e-12));
    assertThat(port.raw(port.read(0.25))).isCloseTo(0.25, within(1e-12));
    // A ten-turn pot reads in tenths of its range: its sensor is geared a tenth as much.
    assertThat(pot.read(0.5)).isCloseTo(0.25, within(1e-12));
    assertThat(pot.raw(1.1)).isCloseTo(0.35, within(1e-12));
    Sensor.Encoder slow = (Sensor.Encoder) winch.ownSensors().getFirst();
    assertThat(slow.terms().getFirst().ratio()).isCloseTo(0.4, within(1e-12));
    assertThat(pivot.driven().motors()).isEmpty();
    assertThat(pivot.driven().absoluteEncoder().orElseThrow().readByController()).isFalse();
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                DrivenAbsoluteEncoderBuilder.builder()
                    .ratio(1)
                    .port(MechanismWiring.PortEncoder.dutyCycle(0))
                    .dataPort(DATA_PORT.offset(Rotations.of(0)))
                    .build())
        .withMessageContaining("it's read one way");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Driven.AbsoluteEncoder.of().ratio(0))
        .withMessageContaining("must be a ratio");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Driven.AbsoluteEncoder.of().ratio(-2))
        .withMessageContaining("which way it turns is its wiring's");
    assertThatIllegalArgumentException().isThrownBy(() -> MechanismWiring.PortEncoder.dutyCycle(6));
    assertThatIllegalArgumentException().isThrownBy(() -> MechanismWiring.PortEncoder.analog(0, 0));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> MechanismWiring.PortEncoder.dutyCycle(0).offset(Rotations.of(1.5)))
        .withMessageContaining("more than its range");
  }

  /**
   * An elevator's absolute encoder turns with its drum, the one its motors' pitch radius names: its
   * ratio is its turns for each of the drum's, so its sensor reads turns per meter through the
   * drum, and a regear of its motors leaves it be. Read any of the three ways, said once, and
   * re-wired or offset on a finished joint, its ratio kept, as a pivot's is. Without motors there's
   * no drum to read it through: it's no sensor, and the definition refuses it (BlocksTest).
   */
  @Test
  void anElevatorsAbsoluteEncoderTurnsWithItsDrum() {
    MechanismWiring.Cancoder cancoder = CANCODER.can(CANIVORE, 7).offset(Rotations.of(0.31));
    Joint.Prismatic elevator =
        Joint.elevator("elevator", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 1.2))
            .rest(Meters.of(0))
            .motors(
                Motor.KRAKEN_X60
                    .through(Gearing.of(10))
                    .drum(Inches.of(1))
                    .on(TALON_FX.can(CANIVORE, 5), TALON_FX.can(CANIVORE, 6)))
            .absoluteEncoder(Driven.AbsoluteEncoder.of().ratio(1.0 / 9).on(cancoder));
    double perMeter = 1.0 / 9 / (2 * Math.PI * 0.0254); // a ninth of a turn per 2π inches

    Sensor.Encoder encoder = (Sensor.Encoder) elevator.ownSensors().getFirst();
    assertThat(encoder.label()).isEqualTo("elevator absolute encoder");
    assertThat(encoder.absolute()).isTrue();
    assertThat(encoder.joint()).isEqualTo(elevator);
    assertThat(encoder.terms().getFirst().pitchRadius()).contains(Inches.of(1));
    assertThat(encoder.ratio(elevator)).isCloseTo(perMeter, within(1e-9));
    assertThat(
            ((Sensor.Encoder)
                    elevator.motors(had -> had.gearing(Gearing.of(12))).ownSensors().getFirst())
                .ratio(elevator))
        .as("regeared motors turn the same drum")
        .isCloseTo(perMeter, within(1e-9));
    assertThat(elevator.devices())
        .extracting(CanDevice::name)
        .containsExactly("elevator motor 5", "elevator motor 6", "elevator CANcoder");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> elevator.absoluteEncoder(cancoder))
        .withMessage(
            "elevator already has an absolute encoder: to re-wire it, absoluteEncoder(encoder ->"
                + " encoder.on(...)); to offset it, absoluteEncoder(encoder ->"
                + " encoder.offset(...)); a decoded pair isn't supported yet");

    Joint.Prismatic practice = elevator.absoluteEncoder(wired -> wired.offset(Rotations.of(0.64)));
    assertThat(practice).isEqualTo(elevator);
    assertThat(practice.driven().absoluteEncoder().orElseThrow().cancoder())
        .contains(CANCODER.can(CANIVORE, 7).offset(Rotations.of(0.64)));
    assertThat(practice.driven().absoluteEncoder().orElseThrow().ratio()).isEqualTo(1.0 / 9);
    MechanismWiring.DataPortEncoder dataPort = DATA_PORT.offset(Rotations.of(0.2)).closesLoop();
    Driven.AbsoluteEncoder onSpark =
        elevator
            .absoluteEncoder(wired -> wired.on(dataPort))
            .driven()
            .absoluteEncoder()
            .orElseThrow();
    assertThat(onSpark.cancoder()).as("read one way").isEmpty();
    assertThat(onSpark.dataPort()).contains(dataPort);
    assertThat(onSpark.ratio()).isEqualTo(1.0 / 9);
    MechanismWiring.PortEncoder port = MechanismWiring.PortEncoder.dutyCycle(1);
    Driven.AbsoluteEncoder onPort =
        elevator.absoluteEncoder(wired -> wired.on(port)).driven().absoluteEncoder().orElseThrow();
    assertThat(onPort.port()).contains(port);
    assertThat(onPort.ratio()).isEqualTo(1.0 / 9);
    assertThat(
            elevator
                .absoluteEncoder(wired -> wired.on(port))
                .absoluteEncoder(wired -> wired.offset(Rotations.of(0.4)))
                .driven()
                .absoluteEncoder()
                .orElseThrow()
                .port())
        .contains(port.offset(Rotations.of(0.4)));

    // On the drum's shaft: a ten-turn potentiometer reads a tenth of its signal per drum turn.
    MechanismWiring.PortEncoder pot = MechanismWiring.PortEncoder.analog(2, 10);
    Joint.Prismatic bare = Joint.elevator("lift", Part.CHASSIS, AXLE);
    assertThat(bare.absoluteEncoder(AbsoluteEncoder.of()).driven().absoluteEncoder())
        .contains(Driven.AbsoluteEncoder.of());
    assertThat(bare.absoluteEncoder(cancoder).driven().absoluteEncoder())
        .contains(Driven.AbsoluteEncoder.of().on(cancoder));
    assertThat(bare.absoluteEncoder(dataPort).driven().absoluteEncoder())
        .contains(Driven.AbsoluteEncoder.of().on(dataPort));
    assertThat(bare.absoluteEncoder(pot).ownSensors()).as("no motors, no drum").isEmpty();
    Sensor.Encoder onShaft =
        (Sensor.Encoder)
            bare.motors(Motor.KRAKEN_X60.through(Gearing.of(10)).drum(Inches.of(1)).count(2))
                .absoluteEncoder(pot)
                .ownSensors()
                .getFirst();
    assertThat(onShaft.terms().getFirst().ratio()).isCloseTo(0.1, within(1e-12));
    assertThat(onShaft.ratio(onShaft.joint()))
        .isCloseTo(0.1 / (2 * Math.PI * 0.0254), within(1e-9));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> bare.absoluteEncoder(wired -> wired.on(cancoder)))
        .withMessage("lift has no absolute encoder to change: give it one, .absoluteEncoder(...)");
  }

  /**
   * Its limit switches are its own sensors, named for its ends, each carrying its wiring: on a
   * port, or on its own controller's pin, which names the joint.
   */
  @Test
  void itsLimitSwitchesAreItsOwnSensors() {
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135))
            .rest(IN)
            .limitSwitch(LimitSwitch.lower().at(Degrees.of(0)))
            .limitSwitch(
                LimitSwitch.upper()
                    .at(Degrees.of(135))
                    .on(SensorWiring.Direction.FORWARD.normallyClosed()));

    Sensor.Switch lower = pivot.switchAt(Sensor.Switch.Limit.End.LOWER);
    Sensor.Switch upper = pivot.switchAt(Sensor.Switch.Limit.End.UPPER);
    assertThat(lower.label()).isEqualTo("pivot lower limit");
    assertThat(lower.wiring()).isEmpty();
    assertThat(((Sensor.Switch.Limit) lower.kind()).position()).contains(0.0);
    assertThat(upper.wiring())
        .contains(
            new SensorWiring.Connection(
                new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.FORWARD),
                SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                pivot.limitSwitch(
                    LimitSwitch.lower().at(Degrees.of(0)).on(SensorWiring.port(1).normallyOpen())))
        .withMessage(
            "pivot already has a limit switch at its lower end: to re-wire it,"
                + " limitSwitch(End.LOWER, limit -> limit.on(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Joint.roller("roller", Part.CHASSIS, AXLE).switchAt(Sensor.Switch.Limit.End.LOWER))
        .withMessage("roller has no limit switch at its lower end");
    Joint.Prismatic unknown =
        Joint.elevator("climber", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 0.5))
            .rest(Meters.of(0))
            .limitSwitch(LimitSwitch.lower())
            .limitSwitch(LimitSwitch.upper().on(SensorWiring.port(2).normallyClosed()));
    Sensor.Switch.Limit lowest =
        (Sensor.Switch.Limit) unknown.switchAt(Sensor.Switch.Limit.End.LOWER).kind();
    assertThat(lowest.position()).as("where it trips isn't known").isEmpty();
    assertThat(unknown.switchAt(Sensor.Switch.Limit.End.UPPER).wiring()).isPresent();
  }

  /** A hood's servos push between the chassis and its link, each a drive under its own label. */
  @Test
  void itsServosPushBetweenItsParentAndItsLink() {
    Actuator.Servo linear =
        Actuator.Servo.builder().from(0.168).to(0.268).force(50).speed(0.032).build();
    Joint.Revolute hood =
        Joint.pivot("hood", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(0, 40))
            .rest(Degrees.of(0))
            .servo(
                "hood left",
                linear,
                new Translation3d(0.054, 0.1, -0.233),
                new Translation3d(0.054, 0.1, 0));

    Drive left = hood.ownDrives().getFirst();
    assertThat(left.label()).isEqualTo("hood left");
    assertThat(left.transmission())
        .isEqualTo(
            Transmission.Between.builder()
                .a(Part.CHASSIS)
                .onA(AXLE.plus(new Translation3d(0.054, 0.1, -0.233)))
                .b(hood)
                .onB(new Translation3d(0.054, 0.1, 0))
                .build());
  }

  /**
   * How it's driven isn't part of what it is, nor where its named places are: a re-tuned or
   * re-wired joint, or one whose places are placed otherwise, is the same joint.
   */
  @Test
  void howItsDrivenIsntWhatItIs() {
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135))
            .rest(IN)
            .positions(IN, OUT)
            .motors(Motor.KRAKEN_X60.through(Gearing.of(25)))
            .control(MechanismControl.position(30));
    Joint.Revolute practice =
        pivot
            .motors(had -> had.on(TALON_FX.can(33)))
            .retuned(control -> MechanismControl.position(25))
            .replaced(OUT, Degrees.of(128));

    assertThat(practice).isEqualTo(pivot).hasSameHashCodeAs(pivot);
    assertThat(JointRevoluteBuilder.builder(pivot).label("wrist").build())
        .as("another name: another joint, with another hash")
        .isNotEqualTo(pivot)
        .doesNotHaveSameHashCodeAs(pivot);
    assertThat(practice.position("out")).isEqualTo(Position.degrees("out", 128));
    assertThat(JointRevoluteBuilder.builder(pivot).positions(List.of(IN)).build())
        .as("a place it hasn't: another joint")
        .isNotEqualTo(pivot);
    assertThat(practice.driven()).isNotEqualTo(pivot.driven());
    assertThat(practice.driven().control().orElseThrow().gains().kP()).isEqualTo(25);
    assertThat(pivot.range(Range.degrees(0, 140)))
        .as("its stops elsewhere: the same joint")
        .isEqualTo(pivot)
        .hasSameHashCodeAs(pivot);
    assertThat(JointRevoluteBuilder.builder(pivot).range(Optional.empty()).build())
        .as("no stops at all: another joint")
        .isNotEqualTo(pivot);
    assertThat(JointRevoluteBuilder.builder(pivot).link(Link.unseen("other arm")).build())
        .as("another link: another joint")
        .isNotEqualTo(pivot);
  }

  /**
   * A block says each thing once: what a joint has one of is refused said again, in its block or on
   * the finished joint, saying how to change it, so a second encoder meant as a pair, or a practice
   * robot's motor restating its gearing, never silently takes the first one's place. A rest is said
   * whatever it is, 0 included. What it has several of adds up.
   */
  @Test
  void aSecondSayOfWhatAJointHasOneOfIsRefused() {
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135))
            .rest(IN)
            .carries(
                Link.box("arm", new Translation3d(0, 0, 0.2), new Translation3d(0.05, 0.5, 0.36)))
            .positions(IN, OUT)
            .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(32)))
            .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))
            .limitSwitch(LimitSwitch.lower().at(Degrees.of(0)))
            .control(MechanismControl.position(30));
    Joint.Revolute roller =
        Joint.roller("roller", Part.CHASSIS, AXLE).outputs(INTAKING).rest(Degrees.of(10));
    Joint.Prismatic elevator =
        Joint.elevator("elevator", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 1.2))
            .rest(Meters.of(0))
            .motors(Motor.KRAKEN_X60.through(Gearing.of(10)).drum(Inches.of(1)).count(2))
            .control(MechanismControl.position(60));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> pivot.motors(Motor.KRAKEN_X60.through(Gearing.of(30)).on(TALON_FX.can(33))))
        .withMessage(
            "pivot already has its motors: to re-wire them, motors(motors -> motors.on(...)); to"
                + " re-gear them, motors(motors -> motors.gearing(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> pivot.absoluteEncoder(CANCODER.can(35).offset(Rotations.of(0.1))))
        .withMessage(
            "pivot already has an absolute encoder: to re-wire it, absoluteEncoder(encoder ->"
                + " encoder.on(...)); to offset it, absoluteEncoder(encoder ->"
                + " encoder.offset(...)); a decoded pair isn't supported yet");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> pivot.control(MechanismControl.position(25)))
        .withMessage(
            "pivot already has its control: to re-tune it, retuned(control ->"
                + " control.withKg(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> pivot.limitSwitch(LimitSwitch.lower().on(SensorWiring.port(5).normallyClosed())))
        .withMessage(
            "pivot already has a limit switch at its lower end: to re-wire it,"
                + " limitSwitch(End.LOWER, limit -> limit.on(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> pivot.carries(Link.unseen("roller")))
        .withMessage(
            "pivot already carries arm: it moves one link, so give it once, with everything it"
                + " carries");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> pivot.positions(Position.degrees("run", 110)))
        .withMessage(
            "pivot already has its positions, in, out: name them all in one positions(...); to"
                + " put one elsewhere, replaced(...)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> roller.outputs(Output.volts("outtaking", -4)))
        .withMessage(
            "roller already has its outputs, intaking: name them all in one outputs(...); to run"
                + " one otherwise, replaced(...)");
    assertThat(pivot.range(Range.degrees(0, 120)).rest(OUT).rest())
        .as("its stops and its rest each one verb, said again on a finished joint to change it")
        .contains(Math.toRadians(130));
    Joint.Revolute atZero = Joint.roller("spindexer", Part.CHASSIS, AXLE).rest(Degrees.of(0));
    assertThat(atZero.rest()).as("said, at 0").contains(0.0);
    assertThat(Joint.roller("unsaid", Part.CHASSIS, AXLE).rest())
        .as("a roller's block says none: it starts at 0")
        .isEmpty();
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                elevator.motors(
                    Motor.KRAKEN_X60.through(Gearing.of(12)).drum(Inches.of(1)).count(2)))
        .withMessageStartingWith("elevator already has its motors");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> elevator.control(MechanismControl.position(50)))
        .withMessageStartingWith("elevator already has its control");
    assertThat(pivot.limitSwitch(LimitSwitch.upper().at(Degrees.of(135))).driven().limitSwitches())
        .as("a switch at its other end adds up")
        .hasSize(2);
  }

  /**
   * Its absolute encoder re-wired, its loop reads it as it read the one before, where it's the same
   * kind of input: a practice robot whose CANcoder only moved ID stays fused, as a fresh CANcoder
   * is read once; one that says its own way, remote, has it. A loop closed on a data port or a
   * Systemcore port closes on the new one. One from another kind of input is as written. Until
   * 2026-10-07 a fresh CANcoder read the practice robot's once while the competition robot's was
   * fused.
   */
  @Test
  void reWiredItsLoopReadsItsEncoderAsBefore() {
    Joint.Revolute fused =
        Joint.roller("turret", Part.CHASSIS, AXLE, Vector3.UP)
            .motors(Motor.KRAKEN_X60.through(Gearing.of(50)).on(TALON_FX.can(20)))
            .absoluteEncoder(CANCODER.can(22).offset(Rotations.of(0.1)).fused());
    Joint.Revolute onDataPort =
        Joint.roller("turret", Part.CHASSIS, AXLE, Vector3.UP)
            .motors(Motor.NEO.through(Gearing.of(50)).on(SPARK_MAX.can(20)))
            .absoluteEncoder(DATA_PORT.offset(Rotations.of(0.1)).closesLoop());
    MechanismWiring.PortEncoder port = MechanismWiring.PortEncoder.dutyCycle(1);
    Joint.Revolute onPort =
        Joint.roller("turret", Part.CHASSIS, AXLE, Vector3.UP)
            .motors(Motor.KRAKEN_X60.through(Gearing.of(50)).on(TALON_FX.can(20)))
            .absoluteEncoder(port.closesLoop());

    assertThat(
            cancoder(
                fused.absoluteEncoder(
                    wired -> wired.on(CANCODER.can(35).offset(Rotations.of(0.4))))))
        .isEqualTo(CANCODER.can(35).offset(Rotations.of(0.4)).fused());
    assertThat(
            cancoder(
                fused.absoluteEncoder(
                    wired -> wired.on(CANCODER.can(35).offset(Rotations.of(0.4)).remote()))))
        .isEqualTo(CANCODER.can(35).offset(Rotations.of(0.4)).remote());
    assertThat(
            onDataPort
                .absoluteEncoder(wired -> wired.on(DATA_PORT.offset(Rotations.of(0.3))))
                .driven()
                .absoluteEncoder()
                .orElseThrow()
                .dataPort())
        .contains(DATA_PORT.offset(Rotations.of(0.3)).closesLoop());
    assertThat(
            onPort
                .absoluteEncoder(wired -> wired.on(MechanismWiring.PortEncoder.dutyCycle(2)))
                .driven()
                .absoluteEncoder()
                .orElseThrow()
                .port())
        .contains(MechanismWiring.PortEncoder.dutyCycle(2).closesLoop());
    assertThat(
            fused
                .absoluteEncoder(wired -> wired.on(DATA_PORT.offset(Rotations.of(0.3))))
                .driven()
                .absoluteEncoder()
                .orElseThrow()
                .dataPort())
        .as("another kind of input, as written")
        .contains(DATA_PORT.offset(Rotations.of(0.3)));
  }

  private static MechanismWiring.Cancoder cancoder(Joint joint) {
    return joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder).orElseThrow();
  }

  /**
   * An absolute encoder says whether its joint's motor controller runs its own loop on it: a
   * CANcoder read as remote, fused or synced, or a data port that closes the loop. A Systemcore
   * port's loop is the robot's, not the controller's.
   */
  @Test
  void anEncoderSaysWhetherItsControllersLoopRunsOnIt() {
    assertThat(Driven.AbsoluteEncoder.of().runsControllerLoop()).isFalse();
    assertThat(on(CANCODER.can(7).offset(Rotations.of(0))).runsControllerLoop()).isFalse();
    assertThat(on(CANCODER.can(7).offset(Rotations.of(0)).remote()).runsControllerLoop()).isTrue();
    assertThat(on(DATA_PORT.offset(Rotations.of(0))).runsControllerLoop()).isFalse();
    assertThat(on(DATA_PORT.offset(Rotations.of(0)).closesLoop()).runsControllerLoop()).isTrue();
    assertThat(on(MechanismWiring.PortEncoder.dutyCycle(1).closesLoop()).runsControllerLoop())
        .isFalse();
  }

  private static Driven.AbsoluteEncoder on(MechanismWiring.EncoderWiring wiring) {
    return Driven.AbsoluteEncoder.of().on(wiring);
  }

  /**
   * A limit switch says where it trips as its joint is measured: an angle on a joint that turns, a
   * distance on one that slides. The other kind is refused where the block says it, and where a
   * practice robot changes it.
   */
  @Test
  void aLimitSwitchTripsWhereItsJointIsMeasured() {
    Joint.Revolute pivot = Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_UP);
    Joint.Prismatic lift = Joint.elevator("lift", Part.CHASSIS, AXLE);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> pivot.limitSwitch(LimitSwitch.lower().at(Inches.of(1))))
        .withMessage(
            "pivot's lower limit switch trips at a length, and the joint turns: give it an angle,"
                + " .at(Degrees.of(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> lift.limitSwitch(LimitSwitch.upper().at(Degrees.of(90))))
        .withMessage(
            "lift's upper limit switch trips at an angle, and the joint slides: give it a length,"
                + " .at(Inches.of(...))");
    Joint.Prismatic stopped = lift.limitSwitch(LimitSwitch.upper().at(Inches.of(40)));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                stopped.limitSwitch(
                    Sensor.Switch.Limit.End.UPPER, limit -> limit.at(Degrees.of(90))))
        .withMessageStartingWith("lift's upper limit switch trips at an angle");
    assertThat(
            ((Sensor.Switch.Limit)
                    stopped
                        .limitSwitch(
                            Sensor.Switch.Limit.End.UPPER, limit -> limit.at(Inches.of(39)))
                        .switchAt(Sensor.Switch.Limit.End.UPPER)
                        .kind())
                .position())
        .hasValueSatisfying(at -> assertThat(at).isCloseTo(39 * 0.0254, within(1e-12)));
    assertThat(pivot.limitSwitch(LimitSwitch.lower()).driven().limitSwitches())
        .containsExactly(LimitSwitch.lower());
  }

  /**
   * A finished joint is changed one thing at a time, the rest of it kept, as a practice robot's is:
   * its motors on other controllers, keeping their motor, count and gearing; its absolute encoder
   * read elsewhere, keeping its ratio; a limit switch on another input in the old one's place,
   * keeping where it trips; its control, a wither at a time. Each refuses a joint without what it
   * changes.
   */
  @Test
  void aFinishedJointIsChangedOneThingAtATimeTheRestKept() {
    Joint.Revolute turret =
        Joint.roller("turret", Part.CHASSIS, AXLE, Vector3.UP)
            .motors(
                Motor.KRAKEN_X60
                    .through(Gearing.of(50))
                    .on(TALON_FX.can(20), TALON_FX.can(21).clockwise()))
            .absoluteEncoder(
                Driven.AbsoluteEncoder.of()
                    .ratio(92.0 / 18)
                    .on(CANCODER.can(22).offset(Rotations.of(0.1))))
            .limitSwitch(LimitSwitch.lower().on(SensorWiring.port(1).normallyClosed()))
            .control(MechanismControl.position(20).withKd(1));
    Joint.Prismatic elevator =
        Joint.elevator("elevator", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 1.2))
            .rest(Meters.of(0))
            .motors(
                Motor.KRAKEN_X60
                    .through(Gearing.of(10))
                    .drum(Inches.of(1))
                    .on(TALON_FX.can(5), TALON_FX.can(6)))
            .limitSwitch(
                LimitSwitch.lower().at(Meters.of(0)).on(SensorWiring.port(1).normallyClosed()))
            .control(MechanismControl.position(60));

    Joint.Revolute practice =
        turret
            .motors(had -> had.on(TALON_FX.can(30), TALON_FX.can(31).clockwise()))
            .absoluteEncoder(wired -> wired.on(CANCODER.can(32).offset(Rotations.of(0.4))))
            .limitSwitch(
                Sensor.Switch.Limit.End.LOWER,
                limit -> limit.on(SensorWiring.port(2).normallyClosed()))
            .retuned(control -> control.withKd(2));

    assertThat(practice).isEqualTo(turret);
    Driven.Motors motors = practice.driven().motors().orElseThrow();
    assertThat(motors.motor()).isEqualTo(Motor.KRAKEN_X60);
    assertThat(motors.count()).isEqualTo(2);
    assertThat(motors.gearing()).isEqualTo(Gearing.of(50));
    assertThat(motors.controllers())
        .containsExactly(TALON_FX.can(30), TALON_FX.can(31).clockwise());
    Driven.AbsoluteEncoder encoder = practice.driven().absoluteEncoder().orElseThrow();
    assertThat(encoder.ratio()).isEqualTo(92.0 / 18);
    assertThat(encoder.cancoder()).contains(CANCODER.can(32).offset(Rotations.of(0.4)));
    Driven.AbsoluteEncoder onPort =
        practice
            .absoluteEncoder(wired -> wired.on(MechanismWiring.PortEncoder.dutyCycle(3)))
            .driven()
            .absoluteEncoder()
            .orElseThrow();
    assertThat(onPort.cancoder()).as("read one way").isEmpty();
    assertThat(onPort.port()).contains(MechanismWiring.PortEncoder.dutyCycle(3));
    assertThat(onPort.ratio()).isEqualTo(92.0 / 18);
    assertThat(
            practice
                .absoluteEncoder(wired -> wired.on(DATA_PORT.offset(Rotations.of(0.2))))
                .driven()
                .absoluteEncoder()
                .orElseThrow()
                .dataPort())
        .contains(DATA_PORT.offset(Rotations.of(0.2)));
    assertThat(practice.driven().limitSwitches())
        .singleElement()
        .isEqualTo(
            DrivenLimitSwitchBuilder.builder()
                .end(Sensor.Switch.Limit.End.LOWER)
                .wiring(SensorWiring.port(2).normallyClosed())
                .build());
    assertThat(
            practice
                .limitSwitch(
                    Sensor.Switch.Limit.End.LOWER,
                    limit -> limit.on(SensorWiring.Direction.REVERSE.normallyClosed()))
                .switchAt(Sensor.Switch.Limit.End.LOWER)
                .wiring())
        .contains(
            new SensorWiring.Connection(
                SensorWiring.ControllerLimit.of(turret, SensorWiring.Direction.REVERSE),
                SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThat(practice.driven().control().orElseThrow().gains().kD()).isEqualTo(2);
    assertThat(practice.driven().control().orElseThrow().gains().kP()).isEqualTo(20);
    assertThat(turret.motors(had -> had.on()).driven().motors().orElseThrow().controllers())
        .as("none: in the simulator alone")
        .isEmpty();

    Joint.Prismatic lifted =
        elevator
            .motors(had -> had.on(TALON_FX.can(7), TALON_FX.can(8).clockwise()))
            .limitSwitch(
                Sensor.Switch.Limit.End.LOWER,
                limit -> limit.on(SensorWiring.Direction.REVERSE.normallyClosed()))
            .retuned(control -> control.withKg(0.5));
    assertThat(lifted.driven().motors().orElseThrow().pitchRadius()).contains(Inches.of(1));
    assertThat(lifted.driven().motors().orElseThrow().controllers())
        .containsExactly(TALON_FX.can(7), TALON_FX.can(8).clockwise());
    assertThat(
            ((Sensor.Switch.Limit) lifted.switchAt(Sensor.Switch.Limit.End.LOWER).kind())
                .position())
        .as("where it trips kept")
        .contains(0.0);
    assertThat(lifted.driven().control().orElseThrow().gains().kG()).contains(0.5);

    Joint.Revolute bare = Joint.roller("bare", Part.CHASSIS, AXLE);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> bare.motors(had -> had.on(TALON_FX.can(30))))
        .withMessage("bare has no motors to change: give it .motors(...) first");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> turret.motors(had -> had.on(TALON_FX.can(30))))
        .withMessage(
            "turret has 2 motors, and motors.on(...) names 1: name a controller for each, the"
                + " leader first");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> bare.absoluteEncoder(wired -> wired.on(CANCODER.can(32).offset(Rotations.of(0)))))
        .withMessage("bare has no absolute encoder to change: give it one, .absoluteEncoder(...)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                turret.limitSwitch(
                    Sensor.Switch.Limit.End.UPPER,
                    limit -> limit.on(SensorWiring.port(2).normallyClosed())))
        .withMessage(
            "turret has no limit switch at its upper end to change: give it one,"
                + " .limitSwitch(LimitSwitch.upper()...)");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> bare.retuned(control -> control.withKd(2)))
        .withMessage("bare has no control to re-tune: give it one, .control(...)");
  }

  /**
   * A finished joint's hard stops move, where it rests kept, as a practice turret's that turns
   * -355° to 5° where the competition robot's turns -390° to 40°. Where it rests and its positions
   * are checked inside them by the robot that has it, so a rest left outside, or a position, is
   * refused there; a joint with no stops to move is refused at once.
   */
  @Test
  void aFinishedJointsStopsMoveItsRestKept() {
    Joint.Revolute turret =
        Joint.pivot("turret", Part.CHASSIS, AXLE, Vector3.UP)
            .range(Range.degrees(-390, 40))
            .rest(Degrees.of(0))
            .positions(Position.degrees("home", 0), Position.degrees("back", -180));

    Joint.Revolute practice = turret.range(Range.degrees(-355, 5));

    assertThat(practice.range()).contains(Range.degrees(-355, 5));
    assertThat(practice.restPosition()).isEqualTo(0);
    assertThat(practice).isEqualTo(turret);
    assertThat(
            Joint.elevator("elevator", Part.CHASSIS, AXLE)
                .range(Range.meters(0, 1.2))
                .rest(Meters.of(0))
                .range(Range.meters(0, 1.1))
                .range())
        .contains(Range.meters(0, 1.1));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(turret.range(Range.degrees(-355, -5))))
        .withMessageContaining(
            "RobotDefinition[test].joints: turret rests at 0°, outside its hard stops, -355° to"
                + " -5°: it can't start there");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(turret.range(Range.degrees(-170, 40))))
        .withMessage(
            "RobotDefinition[test].joints: turret's back, -180°, is outside its hard stops, -170°"
                + " to 40°: it can't get there");
    assertThat(Joint.pivot("tilt", Part.CHASSIS, AXLE, Joint.PITCH_UP).range(Range.degrees(0, 90)))
        .as("with no rest said, it rests at its zero, at its stop")
        .satisfies(tilt -> assertThat(robot(tilt).joints().getFirst().restPosition()).isZero());
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    Joint.pivot("tilt", Part.CHASSIS, AXLE, Joint.PITCH_UP)
                        .range(Range.degrees(10, 90))))
        .withMessage(
            "RobotDefinition[test].joints: tilt has no rest, so it starts at its zero, 0°, outside"
                + " its hard stops, 10° to 90°: it can't start there; give where it rests,"
                + " .rest(...)");
  }

  /**
   * A finished joint rests elsewhere, its stops kept, as a practice robot's arm stowed lower: given
   * an angle, or one of its positions, at its own, re-placed. It's the same joint, so the robot
   * takes it in the competition robot's one's place; a rest outside its stops is refused there, and
   * another joint's position as its rest at once.
   */
  @Test
  void aFinishedJointRestsElsewhereItsStopsKept() {
    Position stowed = Position.degrees("stowed", -45);
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(-60, 150))
            .rest(stowed)
            .positions(stowed, Position.degrees("level", 0));

    Joint.Revolute practice = arm.rest(Degrees.of(-30));

    assertThat(practice.restPosition()).isCloseTo(Math.toRadians(-30), within(1e-12));
    assertThat(practice.range()).isEqualTo(arm.range());
    assertThat(practice).isEqualTo(arm).hasSameHashCodeAs(arm);
    assertThat(arm.replaced(stowed, Degrees.of(-50)).rest(stowed).restPosition())
        .as("its own stowed, re-placed")
        .isCloseTo(Math.toRadians(-50), within(1e-12));
    assertThat(robot(practice).joints().getFirst().restPosition())
        .isCloseTo(Math.toRadians(-30), within(1e-12));
    assertThat(
            Joint.elevator("elevator", Part.CHASSIS, AXLE)
                .range(Range.meters(0, 1.2))
                .rest(Meters.of(0))
                .rest(Inches.of(1))
                .restPosition())
        .isCloseTo(0.0254, within(1e-12));
    assertThat(Joint.roller("turret", Part.CHASSIS, AXLE, Vector3.UP).rest(Degrees.of(90)).rest())
        .as("one that turns freely rests anywhere")
        .contains(Math.toRadians(90));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(arm.rest(Degrees.of(160))))
        .withMessage(
            "RobotDefinition[test].joints: arm rests at 160°, outside its hard stops, -60° to"
                + " 150°: it can't start there");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> arm.rest(Position.degrees("stowed", -20)))
        .withMessage(
            "arm's stowed is at -45°, and this stowed, at -20°, isn't one it has had: another"
                + " joint's or another robot's; give arm's own");
  }

  /**
   * A practice wrist whose stops and places both move, each past the other's, changes them in
   * either order: its score, at 80°, is past the competition wrist's stops, and the competition
   * wrist's, at -80°, past its own. The robot that has it checks them together, once it's whole,
   * and refuses one left behind.
   */
  @Test
  void aJointsStopsAndPlacesChangeInEitherOrder() {
    Position score = Position.degrees("score", -80);
    Joint.Revolute wrist =
        Joint.pivot("wrist", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(-90, 10))
            .rest(Degrees.of(0))
            .positions(Position.degrees("stow", 0), score);

    Joint.Revolute stopsFirst = wrist.range(Range.degrees(-10, 90)).replaced(score, Degrees.of(80));
    Joint.Revolute placesFirst =
        wrist.replaced(score, Degrees.of(80)).range(Range.degrees(-10, 90));

    assertThat(robot(stopsFirst).joints()).containsExactly(wrist);
    assertThat(robot(placesFirst).joints().getFirst().position("score"))
        .isEqualTo(Position.of("score", Degrees.of(80)));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(wrist.range(Range.degrees(-10, 90))))
        .withMessage(
            "RobotDefinition[test].joints: wrist's score, -80°, is outside its hard stops, -10° to"
                + " 90°: it can't get there");
  }

  /**
   * A practice robot's encoder usually differs from the competition robot's in its offset alone:
   * changed alone, its device and the rest of its wiring are kept, whichever of the three reads it.
   * One with no encoder, or none wired, has no offset to change.
   */
  @Test
  void anEncodersOffsetIsChangedAloneItsWiringKept() {
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(-60, 150))
            .rest(Degrees.of(-45))
            .absoluteEncoder(
                Driven.AbsoluteEncoder.of()
                    .ratio(2)
                    .on(
                        CANCODER
                            .can(CanBus.canivore("drive"), 23)
                            .offset(Rotations.of(0.12))
                            .clockwise()
                            .remote()));

    Driven.AbsoluteEncoder offset =
        arm.absoluteEncoder(wired -> wired.offset(Rotations.of(0.47)))
            .driven()
            .absoluteEncoder()
            .orElseThrow();

    assertThat(offset.cancoder())
        .contains(
            CANCODER
                .can(CanBus.canivore("drive"), 23)
                .offset(Rotations.of(0.47))
                .clockwise()
                .remote());
    assertThat(offset.ratio()).isEqualTo(2);
    assertThat(arm.absoluteEncoder(wired -> wired.offset(Rotations.of(0.47)))).isEqualTo(arm);
    assertThat(
            arm.absoluteEncoder(
                    encoder ->
                        encoder.on(DATA_PORT.offset(Rotations.of(0.1)).inverted().closesLoop()))
                .absoluteEncoder(wired -> wired.offset(Rotations.of(-0.2)))
                .driven()
                .absoluteEncoder()
                .orElseThrow()
                .dataPort())
        .contains(DATA_PORT.offset(Rotations.of(-0.2)).inverted().closesLoop());
    assertThat(
            arm.absoluteEncoder(
                    encoder ->
                        encoder.on(
                            MechanismWiring.PortEncoder.analog(2, 10)
                                .offset(Rotations.of(1))
                                .inverted()))
                .absoluteEncoder(wired -> wired.offset(Rotations.of(3.127)))
                .driven()
                .absoluteEncoder()
                .orElseThrow()
                .port())
        .contains(MechanismWiring.PortEncoder.analog(2, 10).offset(Rotations.of(3.127)).inverted());
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Joint.roller("bare", Part.CHASSIS, AXLE)
                    .absoluteEncoder(wired -> wired.offset(Rotations.of(0.1))))
        .withMessage("bare has no absolute encoder to change: give it one, .absoluteEncoder(...)");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                Joint.roller("unwired", Part.CHASSIS, AXLE)
                    .absoluteEncoder(AbsoluteEncoder.of())
                    .absoluteEncoder(wired -> wired.offset(Rotations.of(0.1))))
        .withMessage(
            "an absolute encoder that isn't wired has no offset: the simulator alone reads it."
                + " Wire it, on(...)");
  }

  @Test
  void itRefusesWhatCantBeWired() {
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .range(Range.degrees(0, 90))
            .rest(Degrees.of(0));

    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                arm.motors(
                    Motor.KRAKEN_X60
                        .through(Gearing.of(60))
                        .on(TALON_FX.can(20), SPARK_MAX.can(21))))
        .withMessageContaining("a joint's motors are on one kind of controller, on one bus");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> arm.motors(Motor.NEO.through(Gearing.of(60)).on(TALON_FX.can(20))))
        .withMessageContaining(
            "its NEO is on a TalonFX, which doesn't drive one: a TalonFXS or a SPARK MAX or a SPARK"
                + " Flex does");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> robot(arm.absoluteEncoder(DATA_PORT.offset(Rotations.of(0)))))
        .withMessage(
            "RobotDefinition[test].joints: arm's absolute encoder is wired, and its motors aren't:"
                + " it's read through their controller, so wire them too; a joint your own code"
                + " drives reads one on a Systemcore port instead, MechanismWiring.PortEncoder");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                robot(
                    arm.motors(
                            Motor.KRAKEN_X60.through(Gearing.of(60)).on(TALON_FX.can(CANIVORE, 20)))
                        .absoluteEncoder(CANCODER.can(21).offset(Rotations.of(0.1)))))
        .withMessage(
            "RobotDefinition[test].joints: arm's CANcoder is on Systemcore's CAN_S0 port, and its"
                + " motors on the CANivore canivore: their TalonFX reads it over its own bus, so"
                + " put the CANcoder there");
    assertThat(
            robot(
                    arm.absoluteEncoder(CANCODER.can(CANIVORE, 21).offset(Rotations.of(0.1)))
                        .motors(
                            Motor.KRAKEN_X60
                                .through(Gearing.of(60))
                                .on(TALON_FX.can(CANIVORE, 20))))
                .joints())
        .as("its encoder written before its motors: the definition judges the whole block")
        .hasSize(1);
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                arm.driven(
                    DrivenBuilder.builder(Driven.NONE)
                        .motors(
                            new Driven.Motors(
                                Motor.KRAKEN_X60,
                                1,
                                Gearing.of(60),
                                Optional.of(Inches.of(1)),
                                List.of()))
                        .build()))
        .withMessageContaining("a joint that turns has no drum");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                new Driven.Motors(
                    Motor.KRAKEN_X60, 2, Gearing.of(1), Optional.empty(), List.of(TALON_FX.can(1))))
        .withMessageContaining("names 1 controllers for 2 motors");
  }

  /**
   * A tuned kG needs no mass, and an arm's may say where its weight pulls hardest; a slide's holds
   * it the same all along, so it has no angle.
   */
  @Test
  void aSlidesKgHasNoAngle() {
    Joint.Revolute arm =
        Joint.pivot("arm", Part.CHASSIS, AXLE, Joint.PITCH_UP)
            .control(MechanismControl.position(30).withKg(0.4, Degrees.of(-90)));
    Joint.Prismatic elevator =
        Joint.elevator("elevator", Part.CHASSIS, AXLE)
            .control(MechanismControl.position(60).withKg(0.5));

    assertThat(arm.link().mass().isMassless()).isTrue();
    assertThat(arm.driven().control().orElseThrow().gains().gravityAngle())
        .contains(Degrees.of(-90));
    assertThat(elevator.driven().control().orElseThrow().gains().kG()).contains(0.5);
    assertThatIllegalArgumentException()
        .isThrownBy(() -> elevator.retuned(control -> control.withKg(0.5, Degrees.of(0))))
        .withMessageContaining(
            "a joint that slides is held against gravity the same all along it, so its kG has no"
                + " angle: give kG alone, withKg(kG)");
  }

  /**
   * What real robots' code needed: a roller and a slide on any axis, a rest with no stops, a second
   * robot's gearing with its controllers kept, a switch of unknown position on the joint's own pin,
   * and a rest given in degrees landing on a stop given in degrees.
   */
  @Test
  void theBlockTakesWhatRealRobotsNeeded() {
    Joint.Revolute spindexer = Joint.roller("spindexer", Part.CHASSIS, AXLE, Vector3.UP);
    Joint.Prismatic hopper = Joint.slide("hopper", Part.CHASSIS, AXLE, new Vector3(1, 0, 0));
    Joint.Revolute shoulder =
        Joint.pivot("shoulder", Part.CHASSIS, AXLE, Joint.PITCH_UP).rest(Degrees.of(104.4));
    Joint.Prismatic held = hopper.rest(Meters.of(0.1));
    Joint.Revolute shooter =
        Joint.roller("shooter", Part.CHASSIS, AXLE)
            .motors(
                Motor.KRAKEN_X60
                    .through(Gearing.of(1))
                    .on(TALON_FX.can(1), TALON_FX.can(2).clockwise()));
    Joint.Revolute regeared = shooter.motors(had -> had.gearing(Gearing.of(1.5)));
    Joint.Prismatic topped =
        Joint.elevator("elevator", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 1.2))
            .rest(Meters.of(0))
            .limitSwitch(LimitSwitch.upper().on(SensorWiring.Direction.FORWARD.normallyClosed()));

    assertThat(spindexer.axis()).isEqualTo(Vector3.UP);
    assertThat(hopper.axis()).isEqualTo(new Vector3(1, 0, 0));
    assertThat(shoulder.restPosition()).isCloseTo(Math.toRadians(104.4), within(1e-12));
    assertThat(held.restPosition()).isEqualTo(0.1);
    assertThat(regeared.driven().motors().orElseThrow().gearing()).isEqualTo(Gearing.of(1.5));
    assertThat(regeared.driven().motors().orElseThrow().controllers())
        .isEqualTo(shooter.driven().motors().orElseThrow().controllers());
    assertThat(topped.switchAt(Sensor.Switch.Limit.End.UPPER).wiring())
        .contains(
            new SensorWiring.Connection(
                SensorWiring.ControllerLimit.of(topped, SensorWiring.Direction.FORWARD),
                SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> spindexer.motors(had -> had.gearing(Gearing.of(2))))
        .withMessage("spindexer has no motors to change: give it .motors(...) first");
    assertThat(
            Joint.pivot("tilt", Part.CHASSIS, AXLE, Joint.PITCH_UP)
                .range(Range.degrees(12, 90))
                .rest(Degrees.of(12))
                .restPosition())
        .as("at its stop, though 12° converts a step apart two ways")
        .isCloseTo(Math.toRadians(12), within(1e-12));
  }

  /**
   * A place is its joint's by its name and where it is, or was: a practice robot's climber, its
   * climb re-placed higher, takes the competition robot's constant as its own, at its own height,
   * and keeps every height it has had, invisibly to what it is. A climb at a height it has never
   * had is another joint's or another robot's, and refused, saying where its own is; a name it
   * hasn't is refused, saying what it has, and one of its names of the other kind, an angle, is
   * another joint's.
   */
  @Test
  void aPlaceIsItsJointsByItsNameAndWhereItIsOrWas() {
    Position climb = Position.of("climb", Meters.of(0.05));
    Joint.Prismatic climber =
        Joint.elevator("climber", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 0.3))
            .rest(Meters.of(0))
            .positions(climb);
    Joint.Prismatic practice = climber.replaced(climb, Meters.of(0.075));
    Position practices = Position.of("climb", Meters.of(0.075));

    assertThat(practice.whyNotOwn(climb)).isEmpty();
    assertThat(practice.own(climb)).isEqualTo(practices);
    assertThat(practice.formerPositions()).containsExactly(climb);
    assertThat(practice).isEqualTo(climber).hasSameHashCodeAs(climber);
    assertThat(practice.replaced(climb, Meters.of(0.1)).formerPositions())
        .as("every height it has had, once each")
        .containsExactly(climb, practices);
    assertThat(practice.replaced(climb, Meters.of(0.075)).formerPositions())
        .as("re-placed where it is: nothing had")
        .containsExactly(climb);
    assertThat(climber.whyNotOwn(Position.of("climb", Inches.of(0.05 / 0.0254))))
        .as("the same height, written another way")
        .isEmpty();
    assertThat(climber.whyNotOwn(practices))
        .contains(
            "climber's climb is at 0.05 m, and this climb, at 0.075 m, isn't one it has had:"
                + " another joint's or another robot's; give climber's own");
    assertThat(climber.whyNotOwn(Position.of("stow", Meters.of(0))))
        .contains("climber has no position stow at 0 m: it has climb");
    assertThat(climber.whyNotOwn(Position.degrees("climb", 30)))
        .contains("climber's climb is a length, and this climb an angle: another joint's");
  }

  /**
   * A place or output is the joint's own to a nanometer, or a nanoradian or a nanovolt: so a place
   * written in degrees and one in radians, a bit apart, are one, and two nanometers are two places.
   */
  @Test
  void aPlaceIsItsJointsToTheNanometer() {
    Joint.Prismatic climber =
        Joint.elevator("climber", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 0.3))
            .rest(Meters.of(0))
            .positions(Position.of("stow", Meters.of(0)));
    Joint.Revolute roller =
        Joint.roller("roller", Part.CHASSIS, AXLE).outputs(Output.volts("idle", 0));

    assertThat(climber.whyNotOwn(Position.of("stow", Meters.of(1e-9)))).isEmpty();
    assertThat(climber.whyNotOwn(Position.of("stow", Meters.of(2e-9)))).isPresent();
    assertThat(roller.whyNotOwn(Output.volts("idle", 1e-9))).isEmpty();
    assertThat(roller.whyNotOwn(Output.volts("idle", 2e-9))).isPresent();
  }

  /**
   * A joint rests at a place of its kind: an angle for one that turns, a length for one that
   * slides. One named by a place it hasn't, in its block, rests where that place is.
   */
  @Test
  void aJointRestsAtAPlaceOfItsKindWhereverItIs() {
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135))
            .positions(IN, OUT);
    Joint.Prismatic climber = Joint.elevator("climber", Part.CHASSIS, AXLE);

    assertThat(pivot.rest(Position.degrees("home", 10)).restPosition())
        .as("a place it hasn't named, where it is, though it has others")
        .isCloseTo(Math.toRadians(10), within(1e-12));
    assertThat(pivot.rest(OUT).restPosition()).isCloseTo(Math.toRadians(130), within(1e-12));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> pivot.rest(Position.of("home", Meters.of(1))))
        .withMessage(
            "pivot can't rest at home, a length, since it turns: give it an angle,"
                + " rest(Degrees.of(...))");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> climber.rest(Position.degrees("home", 10)))
        .withMessage(
            "climber can't rest at home, an angle, since it slides: give it a length,"
                + " rest(Inches.of(...))");
  }

  /**
   * A slide's block says what a pivot's does, each of its things once: its link, places and
   * outputs, its rest at one of its places, its motor on a controller, its encoder, its limit
   * switch on its controller's pin, its servo, and its interlocks. Other joints' interlocks name it
   * by where it is and how fast it runs.
   */
  @Test
  void aSlidesBlockSaysWhatAPivotsDoes() {
    Position low = Position.of("low", Inches.of(4));
    Output raising = Output.of("raising", MetersPerSecond.of(0.5));
    Link carriage =
        Link.box("carriage", new Translation3d(0, 0, 0.1), new Translation3d(0.3, 0.3, 0.2));
    Actuator.Servo linear =
        Actuator.Servo.builder().from(0.168).to(0.268).force(50).speed(0.032).build();
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN).range(Range.degrees(0, 135));
    MechanismWiring.Cancoder cancoder = CANCODER.can(7).offset(Rotations.of(0.31));

    Joint.Prismatic lift =
        Joint.elevator("lift", Part.CHASSIS, AXLE)
            .range(Range.meters(0, 1.2))
            .carries(carriage)
            .positions(low)
            .outputs(raising)
            .rest(low)
            .motors(Motor.KRAKEN_X60.through(Gearing.of(10)).drum(Inches.of(1)).on(TALON_FX.can(5)))
            .absoluteEncoder(cancoder)
            .limitSwitch(
                LimitSwitch.lower()
                    .at(Meters.of(0))
                    .on(SensorWiring.Direction.REVERSE.normallyClosed()))
            .servo(
                "lift brake",
                linear,
                new Translation3d(0.05, 0, 0),
                new Translation3d(0.05, 0, 0.1))
            .runsOnlyWhen(pivot.atLeast(Degrees.of(110)))
            .keepsOut(Range.meters(0.5, 0.7))
            .when(pivot.atMost(Degrees.of(20)));

    assertThat(lift.link()).isEqualTo(carriage);
    assertThat(lift.positions()).containsExactly(low);
    assertThat(lift.outputs()).containsExactly(raising);
    assertThat(lift.restPosition()).isCloseTo(low.value(), within(1e-12));
    assertThat(lift.driven().motors().orElseThrow().controllers()).containsExactly(TALON_FX.can(5));
    assertThat(lift.switchAt(Sensor.Switch.Limit.End.LOWER).wiring())
        .contains(
            new SensorWiring.Connection(
                new SensorWiring.ControllerLimit("lift", SensorWiring.Direction.REVERSE),
                SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThat(lift.ownDrives()).extracting(Drive::label).containsExactly("lift", "lift brake");
    assertThat(lift.driven().interlocks())
        .extracting(Object::toString)
        .containsExactly(
            "lift runs only when pivot at least 110°",
            "lift keeps out of 0.5 m to 0.7 m while pivot at most 20°");
    assertThat(
            lift.absoluteEncoder(wired -> wired.on(CANCODER.can(8).offset(Rotations.of(0.4))))
                .driven()
                .absoluteEncoder()
                .orElseThrow()
                .cancoder())
        .contains(CANCODER.can(8).offset(Rotations.of(0.4)));
    assertThat(lift.replaced(raising, Volts.of(3)).output("raising"))
        .isEqualTo(Output.volts("raising", 3));
    assertThat(lift.replaced(raising, Value.of(0.4)).output("raising"))
        .isEqualTo(Output.dutyCycle("raising", 0.4));
    assertThat(lift.atLeast(Inches.of(10))).hasToString("lift at least 0.254 m");
    assertThat(lift.between(Inches.of(10), Inches.of(20)))
        .hasToString("lift between 0.254 m and 0.508 m");
    assertThat(lift.upToSpeed(MetersPerSecond.of(0.5))).hasToString("lift at least 0.5 m/s");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> lift.carries(Link.unseen("other")))
        .withMessageStartingWith("lift already carries carriage");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> lift.positions(Position.of("high", Inches.of(30))))
        .withMessageStartingWith("lift already has its positions, low");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> lift.outputs(Output.of("lowering", MetersPerSecond.of(-0.5))))
        .withMessageStartingWith("lift already has its outputs, raising");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                lift.motors(
                    Motor.KRAKEN_X60
                        .through(Gearing.of(12))
                        .drum(Inches.of(1))
                        .on(TALON_FX.can(9))))
        .withMessageStartingWith("lift already has its motors");
  }

  /**
   * A pivot's motors are said once whichever way they're counted, its limit switch can sit on its
   * controller's pin with no position known, and it can run only when another joint says.
   */
  @Test
  void aPivotsMotorsAreSaidOnceItsSwitchOnAPinNeedsNoPositionAndItRunsOnlyWhenAnotherSays() {
    Joint.Prismatic lift = Joint.elevator("lift", Part.CHASSIS, AXLE).range(Range.meters(0, 1.2));
    Joint.Revolute pivot =
        Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_DOWN)
            .range(Range.degrees(0, 135))
            .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).count(2))
            .limitSwitch(LimitSwitch.upper().on(SensorWiring.Direction.FORWARD.normallyClosed()))
            .runsOnlyWhen(lift.atLeast(Inches.of(10)));

    assertThatIllegalArgumentException()
        .isThrownBy(() -> pivot.motors(Motor.KRAKEN_X60.through(Gearing.of(30)).count(2)))
        .withMessageStartingWith("pivot already has its motors");
    Sensor.Switch upper = pivot.switchAt(Sensor.Switch.Limit.End.UPPER);
    assertThat(((Sensor.Switch.Limit) upper.kind()).position()).isEmpty();
    assertThat(upper.wiring())
        .contains(
            new SensorWiring.Connection(
                new SensorWiring.ControllerLimit("pivot", SensorWiring.Direction.FORWARD),
                SensorWiring.Polarity.NORMALLY_CLOSED));
    assertThat(pivot.driven().interlocks())
        .extracting(Object::toString)
        .containsExactly("pivot runs only when lift at least 0.254 m");
  }

  @Test
  void aLinkIsUnweighedUnlessItsMassIsGiven() {
    Translation3d center = new Translation3d(0.1, 0, 0.2);
    Translation3d size = new Translation3d(0.2, 0.4, 0.05);

    Link drawn = Link.box("arm", center, size);
    Link weighed = Link.weighed("arm", Kilograms.of(3), center, size);

    assertThat(drawn.mass()).isEqualTo(MassProperties.NONE);
    assertThat(drawn.shapes()).hasSize(1);
    assertThat(((Shape.Box) drawn.shapes().get(0)).material()).isEqualTo(Material.ALUMINUM);
    assertThat(Link.unseen("arm").shapes()).isEmpty();
    assertThat(weighed.mass().mass().in(Kilograms)).isEqualTo(3);
    assertThat(weighed.mass().cg()).isEqualTo(center);
    assertThat(weighed.shapes()).isEqualTo(drawn.shapes());
  }

  @Test
  void aGearingLosesNothingUntilMeasured() {
    assertThat(Gearing.of(25)).isEqualTo(new Gearing(25, 1));
    assertThat(List.of(Joint.PITCH_UP, Joint.PITCH_DOWN)).doesNotContain(Vector3.UP);
  }

  /** The test robot with one joint, its block as given. */
  private static RobotDefinition robot(Joint joint) {
    return RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of()))
        .joints(List.of(joint))
        .build();
  }
}
