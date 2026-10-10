package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Value;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Vector3;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

/**
 * A mechanism's joint: one way a link moves on what it hangs from, turning about an axis ({@link
 * Revolute}, in radians) or sliding along one ({@link Prismatic}, in meters). Joints hang from the
 * chassis or from each other, each written after its parent, so they form a tree, and an arm on an
 * elevator is a joint whose parent is the elevator's.
 *
 * <p>Position 0 is the pose its {@link #origin} gives: an arm's zero is wherever CAD's frame put
 * it. Where its weight pulls hardest comes from its links' mass ({@code RobotMath.gravityOffset}),
 * or from its control, beside its tuned kG ({@code MechanismControl.withKg(kG, angle)}); with
 * neither, it's its zero, horizontal as WPILib takes an arm's. A joint has no state: where it is
 * now comes from its sensors, as a position the derived functions take.
 *
 * <p>It names its own places and outputs ({@link #positions}, {@link #outputs}), and from those,
 * what configurations and interlocks say about it: {@link #at}, {@link #atLeast(Position)}, {@link
 * #atMost(Position)} and {@link #between}.
 *
 * <p>Its block ({@link Block}) declares it one call at a time. A joint has one link, one set of
 * named positions and of outputs, one set of motors, one absolute encoder, one limit switch at each
 * end and one control: a block that says any of these twice is refused, with how to change it. It
 * may have several servos and interlocks, which add up.
 *
 * <p>The parts describe themselves. Motors, an absolute encoder and a limit switch are values that
 * carry their own options ({@link Driven.Motors}, {@link Driven.AbsoluteEncoder}, {@link
 * Driven.LimitSwitch}), and the block takes each whole: {@code
 * .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(32)))}.
 *
 * <p>A practice robot is written as the competition robot's joints with a few things changed, so a
 * later change to the competition robot reaches it. A part is changed by naming it and saying what
 * differs, in the part's own words: {@code PIVOT.motors(motors -> motors.on(TALON_FX.can(33)))},
 * {@code ARM.absoluteEncoder(encoder -> encoder.offset(Rotations.of(0.47)))}, {@code
 * ARM.retuned(control -> control.withKg(0.3))}. A named position or output is put elsewhere with
 * {@code replaced(...)}, and its stops and where it rests with {@code range(...)} and {@code
 * rest(...)}, which a finished joint takes as its block does.
 *
 * <p>Its positions and outputs are names on it, and their values the robot's: code that names one,
 * a move, a configuration or an interlock, finds it where the robot running it puts it ({@link
 * #own(Position)}), so the same code drives a practice robot to its own places. A constant of one
 * of its names is its own at a value the joint has, or had before it was re-placed: the competition
 * robot's, which the team's constants hold. At any other value it's another joint's, or another
 * robot's, and refused, wherever it's given.
 */
public sealed interface Joint extends Part permits Joint.Block {
  /**
   * An axis about which a link whose zero points forward swings up as its position grows: about the
   * robot's y, the other way round.
   */
  Vector3 PITCH_UP = new Vector3(0, -1, 0);

  /** An axis about which a link whose zero points forward swings down as its position grows. */
  Vector3 PITCH_DOWN = new Vector3(0, 1, 0);

  /**
   * A pivot: an arm, an intake's swing, a wrist, turning about an axis. Its block says the rest,
   * one call at a time: {@code Joint.pivot("pivot", Part.CHASSIS, at,
   * Joint.PITCH_DOWN).range(Range.degrees(0, 135)).rest(PIVOT_IN).carries(link).positions(PIVOT_IN,
   * PIVOT_OUT).motor(Motor.KRAKEN_X60, Gearing.of(25)).control(...)}.
   *
   * @param at where its axle is, in its parent's frame
   */
  static Revolute pivot(String label, Part parent, Translation3d at, Vector3 axis) {
    return Revolute.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(axis)
        .link(Link.unseen(label))
        .build();
  }

  /**
   * A roller, a belt or a flywheel: a joint that turns freely, from 0, run at its outputs. {@code
   * Joint.roller("intake", PIVOT, at).outputs(INTAKING).motor(Motor.KRAKEN_X60, Gearing.of(3))}.
   *
   * @param at where its axle is, in its parent's frame
   */
  static Revolute roller(String label, Part parent, Translation3d at) {
    return pivot(label, parent, at, PITCH_DOWN);
  }

  /**
   * A roller turning about another axis: a spindexer about the vertical, {@code
   * Joint.roller("spindexer", Part.CHASSIS, at, Vector3.UP)}.
   */
  static Revolute roller(String label, Part parent, Translation3d at, Vector3 axis) {
    return pivot(label, parent, at, axis);
  }

  /**
   * An elevator: a carriage sliding straight up. {@code Joint.elevator("elevator", Part.CHASSIS,
   * at).range(Range.meters(0, 1.2)).rest(ELEVATOR_DOWN).motors(Motor.KRAKEN_X60, gearing, drum,
   * 2)}.
   *
   * @param at where the slide starts, in its parent's frame
   */
  static Prismatic elevator(String label, Part parent, Translation3d at) {
    return slide(label, parent, at, Vector3.UP);
  }

  /**
   * A joint sliding along any axis: a hopper that extends forward, {@code Joint.slide("hopper",
   * Part.CHASSIS, at, new Vector3(1, 0, 0))}.
   *
   * @param at where the slide starts, in its parent's frame
   */
  static Prismatic slide(String label, Part parent, Translation3d at, Vector3 axis) {
    return Prismatic.builder()
        .label(label)
        .parent(parent)
        .origin(new Transform3d(at, Rotation3d.ZERO))
        .axis(axis)
        .link(Link.unseen(label))
        .build();
  }

  /** What logs and writers call it: unique among the robot's joints. */
  @Override
  String label();

  /** What it hangs from: the chassis, or another joint's link. */
  Part parent();

  /** Where its frame is in its parent's frame at position 0: the pivot, or the slide's start. */
  Transform3d origin();

  /** The axis it turns about or slides along, in its own frame; any length but zero. */
  Vector3 axis();

  /** Its hard stops, unwrapped; empty for one that turns freely, a roller or a flywheel. */
  Optional<Range> range();

  /**
   * Where it rests as the robot starts, as its block says it ({@code rest(...)}); none for one
   * whose block doesn't, which starts at its zero. A practice robot's moves it ({@code rest(...)}).
   */
  Optional<Double> rest();

  /**
   * Where it starts: where the robot is weighed and modeled (stowed), and where a mechanism that
   * zeroes itself at boot is taken to be. Where it rests ({@link #rest}), or 0. Inside its range.
   */
  default double restPosition() {
    return rest().orElse(0.0);
  }

  /** What it moves. */
  Link link();

  /**
   * Its places by name, where code moves it to: an intake's "in", "run" and "out". Inside its
   * range; a joint that follows another has none, since its leader's moves carry it. Code names
   * each by its name, and each robot's joint puts it where that robot's is ({@code replaced}).
   */
  List<Position> positions();

  /**
   * What code runs it at, by name: a roller's "intaking" at a speed, its "outtaking" at a voltage,
   * each within what its motors can do. Not {@link Output#STOPPED}, which every joint has.
   */
  List<Output> outputs();

  /**
   * Where its positions were before they were re-placed ({@code replaced}), each value one had: on
   * a practice robot, the competition robot's, which the team's constants hold, so a constant at
   * one is still this joint's own. None on a joint whose places are where its block put them.
   */
  List<Position> formerPositions();

  /** What its outputs were before they were re-valued ({@code replaced}), as its positions. */
  List<Output> formerOutputs();

  /**
   * Whether it's held at positions: it has a range or named positions. One without either, a
   * roller, only runs at outputs, and stops when idle.
   */
  default boolean holdsPositions() {
    return range().isPresent() || !positions().isEmpty();
  }

  /**
   * How it's driven and wired: its motors or servos, its own sensors, their wiring, and its
   * control. Not part of its identity: joints that differ only here are equal.
   */
  Driven driven();

  /** How its frame moves from its origin's at a position: turned about, or slid along, its axis. */
  Transform3d motion(double position);

  /**
   * How the program drives it, said once in its block: {@code
   * .control(MechanismControl.position(30))}.
   *
   * @throws IllegalArgumentException if it has a control already: {@link #retuned} changes it
   */
  Joint control(MechanismControl control);

  /**
   * The same joint with its control changed: a practice robot's lighter arm, {@code
   * ARM.retuned(control -> control.withKg(0.3))}.
   *
   * @throws IllegalArgumentException if it has no control to change
   */
  Joint retuned(UnaryOperator<MechanismControl> change);

  /**
   * The same joint with its motors changed: a practice robot's on another controller, {@code
   * PIVOT.motors(motors -> motors.on(TALON_FX.can(33).pdh(4, Amps.of(40))))}, or geared otherwise,
   * {@code PIVOT.motors(motors -> motors.gearing(Gearing.of(20)))}.
   *
   * @throws IllegalArgumentException if it has no motors, or the change leaves another number
   */
  Joint motors(UnaryOperator<Driven.Motors> change);

  /**
   * The same joint with its absolute encoder changed: a practice robot's magnet, {@code
   * ARM.absoluteEncoder(encoder -> encoder.offset(Rotations.of(0.47)))}, or its CANcoder at another
   * ID, {@code TURRET.absoluteEncoder(encoder ->
   * encoder.on(CANCODER.can(35).offset(Rotations.of(0.11))))}.
   *
   * @throws IllegalArgumentException if it has no absolute encoder
   */
  Joint absoluteEncoder(UnaryOperator<Driven.AbsoluteEncoder> change);

  /**
   * The same joint with its limit switch at an end changed: a practice robot's on another input,
   * {@code ELEVATOR.limitSwitch(End.LOWER, limit ->
   * limit.on(SensorWiring.port(5).normallyClosed()))}.
   *
   * @throws IllegalArgumentException if it has no limit switch at that end
   */
  Joint limitSwitch(Sensor.Switch.Limit.End end, UnaryOperator<Driven.LimitSwitch> change);

  /**
   * A position on it, or a distance along it, as every message shows one: degrees for a joint that
   * turns, meters for one that slides, to four significant figures ("130°", "0.4 m"), and no finer
   * than a hundredth of a degree or a tenth of a millimeter.
   *
   * @param value in its units: radians, or meters
   */
  default String shown(double value) {
    return Places.shownAt(this instanceof Revolute, value);
  }

  /**
   * Its own drives, as its block says them: its motors, under its label, and each servo, under its
   * own. A drive between joints, a differential's, is the robot's ({@code RobotDefinition.drives}).
   */
  default List<Drive> ownDrives() {
    List<Drive> drives = new ArrayList<>();
    driven()
        .motors()
        .ifPresent(
            motors ->
                drives.add(
                    Drive.builder()
                        .label(label())
                        .actuator(
                            Actuator.Motors.builder()
                                .motor(motors.motor())
                                .count(motors.count())
                                .build())
                        .transmission(through(motors))
                        .build()));
    for (Driven.Servo servo : driven().servos()) {
      drives.add(
          Drive.builder()
              .label(servo.label())
              .actuator(servo.servo())
              .transmission(
                  Transmission.Between.builder()
                      .a(parent())
                      .onA(
                          origin()
                              .getTranslation()
                              .plus(servo.onParent().rotateBy(origin().getRotation())))
                      .b(this)
                      .onB(servo.onLink())
                      .build())
              .build());
    }
    return drives;
  }

  /**
   * What carries its motors' turning to it: a gearing alone for a joint that turns, and a gearing
   * onto a drum for one that slides.
   */
  private Transmission through(Driven.Motors motors) {
    return switch (this) {
      case Revolute turning ->
          Transmission.Geared.builder().joint(turning).gearing(motors.gearing()).build();
      case Prismatic sliding ->
          Transmission.Drum.builder()
              .joint(sliding)
              .gearing(motors.gearing())
              .pitchRadius(motors.pitchRadius().orElseThrow())
              .build();
    };
  }

  /**
   * Its own sensors, as its block says them: its absolute encoder ("arm absolute encoder"), and its
   * limit switches ("arm lower limit", "arm upper limit"), each carrying its wiring. A sliding
   * joint's absolute encoder turns with its drum, so it's one of its sensors once its motors say
   * the drum's pitch radius; without them, the robot's definition refuses it ({@code Blocks}).
   */
  default List<Sensor> ownSensors() {
    List<Sensor> sensors = new ArrayList<>();
    Optional<Distance> drum =
        this instanceof Prismatic
            ? driven().motors().flatMap(Driven.Motors::pitchRadius)
            : Optional.empty();
    driven()
        .absoluteEncoder()
        .filter(encoder -> this instanceof Revolute || drum.isPresent())
        .ifPresent(
            encoder ->
                sensors.add(
                    Sensor.Encoder.builder()
                        .label(label() + " absolute encoder")
                        .terms(
                            List.of(
                                Transmission.Term.builder()
                                    .joint(this)
                                    .ratio(encoder.ratio() / encoder.turnsPerSignal())
                                    .pitchRadius(drum)
                                    .build()))
                        .absolute(true)
                        .build()));
    for (Driven.LimitSwitch each : driven().limitSwitches()) {
      String label = limitLabel(each.end());
      Sensor.Switch made =
          each.position()
              .map(at -> Sensor.Switch.limit(label, this, each.end(), inJointUnits(at)))
              .orElseGet(() -> Sensor.Switch.limit(label, this, each.end()));
      sensors.add(each.wiring().map(wiring -> made.on(wiring.of(this))).orElse(made));
    }
    return sensors;
  }

  /**
   * Its devices on the CAN bus, as its block wires them: its motor controllers, the leader first,
   * each "pivot motor 32" with the PDH channel that feeds it where its block says ({@code
   * TALON_FX.can(32).pdh(4, Amps.of(40))}); then its CANcoder, "pivot CANcoder", on its motors'
   * bus. None for a joint that runs in the simulator alone.
   */
  default List<CanDevice> devices() {
    List<CanDevice> devices = new ArrayList<>();
    List<MechanismWiring.Device> controllers =
        driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    for (MechanismWiring.Device device : controllers) {
      devices.add(
          new CanDevice(
              label() + " motor " + device.id(),
              device.controller().kind(),
              device.bus(),
              device.id(),
              device.feed()));
    }
    Optional<MechanismWiring.Cancoder> cancoder =
        driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder);
    if (cancoder.isPresent() && !controllers.isEmpty()) {
      devices.add(
          new CanDevice(
              label() + " CANcoder",
              CanDevice.Kind.CANCODER,
              cancoder.get().bus(),
              cancoder.get().id(),
              Optional.empty()));
    }
    return devices;
  }

  /**
   * Its own interlocks, as its block says them ({@code .keepsOut(...).when(...)}, {@code
   * .runsOnlyWhen(...)}), each about this joint as the robot has it.
   */
  default List<Interlock> ownInterlocks() {
    List<Interlock> own = new ArrayList<>();
    for (Interlock interlock : driven().interlocks()) {
      own.add(
          switch (interlock) {
            case Interlock.RunsOnlyWhen runs -> Interlock.that(this).runsOnlyWhen(runs.condition());
            case Interlock.KeepsOut keeps ->
                Interlock.that(this).keepsOut(keeps.region()).when(keeps.condition());
          });
    }
    return own;
  }

  /**
   * A stretch of a joint's travel being kept out, in its block: say while what. Not an interlock
   * until it's said.
   *
   * @param joint the joint
   * @param region where it may not be
   * @param adding the joint with the interlock added
   */
  record KeepingOut<J extends Joint>(Joint joint, Range region, Function<Interlock, J> adding) {
    /** While the condition holds: the joint, kept out meanwhile. */
    public J when(Condition condition) {
      return adding.apply(Interlock.that(joint).keepsOut(region).when(condition));
    }
  }

  /**
   * Its limit switch at an end, from its block: what a condition on it is made from, {@code
   * PIVOT.switchAt(End.LOWER).tripped()}.
   *
   * @throws IllegalArgumentException if it has none there
   */
  default Sensor.Switch switchAt(Sensor.Switch.Limit.End end) {
    for (Sensor sensor : ownSensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.end() == end) {
        return onSwitch;
      }
    }
    throw new IllegalArgumentException(
        label() + " has no limit switch at its " + lowerOrUpper(end) + " end");
  }

  /** A limit switch's label, after its joint and its end: "arm lower limit". */
  private String limitLabel(Sensor.Switch.Limit.End end) {
    return label() + " " + lowerOrUpper(end) + " limit";
  }

  /** An end of a joint's travel as a sentence names it: "lower" or "upper". */
  private static String lowerOrUpper(Sensor.Switch.Limit.End end) {
    return end.name().toLowerCase(Locale.ROOT);
  }

  /**
   * Its position by a name.
   *
   * @throws IllegalArgumentException if it has none by the name, naming those it has
   */
  default Position position(String name) {
    for (Position position : positions()) {
      if (position.name().equals(name)) {
        return position;
      }
    }
    throw new IllegalArgumentException(
        label() + " has no position named \"" + name + "\": " + has(names(positions(), List.of())));
  }

  /**
   * Its output by a name: one of its own, or {@link Output#STOPPED}, which every joint has.
   *
   * @throws IllegalArgumentException if it has none by the name, naming those it has
   */
  default Output output(String name) {
    for (Output output : outputs()) {
      if (output.name().equals(name)) {
        return output;
      }
    }
    if (name.equals(Output.STOPPED_NAME)) {
      return Output.STOPPED;
    }
    throw new IllegalArgumentException(
        label()
            + " has no output named \""
            + name
            + "\": "
            + has(names(List.of(), outputs()))
            + ", and "
            + Output.STOPPED_NAME);
  }

  /**
   * It at one of its positions: what a configuration is made of ({@code Configuration.of("amp",
   * ELEVATOR.at(ELEVATOR_CLEAR), ARM.at(ARM_SCORING))}). It's exact, and a measured position never
   * is, so an interlock refuses it: its edge is {@code atLeast}, {@code atMost} or {@code between}.
   * A position not its own is refused where the condition is used ({@link Condition}).
   */
  default Condition.At at(Position position) {
    return ConditionAtBuilder.builder().joint(this).position(position).build();
  }

  /**
   * It at one of its outputs, or {@link Output#STOPPED}: what a configuration is made of ({@code
   * Configuration.of("feeding", FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING))}). An
   * output not its own is refused where the condition is used ({@link Condition}).
   */
  default Condition.At at(Output output) {
    return ConditionAtBuilder.builder().joint(this).output(output).build();
  }

  /**
   * It at one of its positions or beyond it, toward its range's upper end: {@code
   * PIVOT.atLeast(PIVOT_OUT)}, from "out" on. An edge that's a clearance rather than a place it
   * goes is written as a number ({@link Block#atLeast(Measure)}). A position not its own is refused
   * where the condition is used ({@link Condition}).
   *
   * @throws IllegalArgumentException for its lower end
   */
  default Condition.AtLeast atLeast(Position position) {
    return ConditionAtLeastBuilder.builder()
        .joint(this)
        .position(where(position))
        .place(position)
        .build();
  }

  /**
   * It at one of its positions or short of it, toward its range's lower end: {@code
   * ELEVATOR.atMost(ELEVATOR_CLEAR)}, up to "clear". A position not its own is refused where the
   * condition is used ({@link Condition}).
   *
   * @throws IllegalArgumentException for its upper end
   */
  default Condition.AtMost atMost(Position position) {
    return ConditionAtMostBuilder.builder()
        .joint(this)
        .position(where(position))
        .place(position)
        .build();
  }

  /**
   * It between two of its positions, both included, whichever way round they're given. A position
   * not its own is refused where the condition is used ({@link Condition}).
   *
   * @throws IllegalArgumentException if they're one
   */
  default Condition.Between between(Position from, Position to) {
    return ConditionBetweenBuilder.builder()
        .joint(this)
        .from(where(from))
        .to(where(to))
        .fromPlace(from)
        .toPlace(to)
        .build();
  }

  /**
   * Where it puts a position of its own; one not its own, where that says, for what uses the
   * condition naming it to refuse.
   */
  private double where(Position position) {
    return whyNotOwn(position) == null ? own(position).value() : position.value();
  }

  /**
   * Its own position of a position's name, where this joint puts it: the constant a team's code
   * names, as the robot running it places it. A practice robot's joint, its position re-placed
   * ({@code replaced}), gives its own.
   *
   * @throws IllegalArgumentException if it isn't its own ({@link #whyNotOwn(Position)})
   */
  default Position own(Position position) {
    String why = whyNotOwn(position);
    if (why != null) {
      throw new IllegalArgumentException(why);
    }
    return position(position.name());
  }

  /**
   * Its own output of an output's name, at this joint's value, or {@link Output#STOPPED}: a
   * practice robot's joint, its output re-valued ({@code replaced}), gives its own.
   *
   * @throws IllegalArgumentException if it isn't its own ({@link #whyNotOwn(Output)})
   */
  default Output own(Output output) {
    String why = whyNotOwn(output);
    if (why != null) {
      throw new IllegalArgumentException(why);
    }
    return output(output.name());
  }

  /**
   * Why a position isn't one of this joint's own, or null when it is. It's its own when the joint
   * has a position of its name and kind, and it's where that one is, or was before it was re-placed
   * ({@link #formerPositions}): the competition robot's constant, on its practice robot. Another
   * joint's of the same name, elsewhere, is refused, saying where this joint's is, and so is one of
   * a name the joint hasn't, naming those it has. Two joints' places of one name and value are each
   * one's: nothing tells them apart, and either drives each to its own.
   */
  default @Nullable String whyNotOwn(Position position) {
    boolean revolute = position.kind() == Position.Kind.ANGLE;
    for (Position own : positions()) {
      if (!own.name().equals(position.name())) {
        continue;
      }
      if (own.kind() != position.kind()) {
        return possessive(label())
            + " "
            + own.name()
            + " is "
            + (revolute ? "a length" : "an angle")
            + ", and this "
            + own.name()
            + " "
            + (revolute ? "an angle" : "a length")
            + ": another joint's";
      }
      if (had(own, position) || formerPositions().stream().anyMatch(was -> had(was, position))) {
        return null;
      }
      return notHad(own.name(), "is at " + shown(own.value()), shown(position.value()));
    }
    return label()
        + " has no position "
        + position.name()
        + " at "
        + Places.shownAt(revolute, position.value())
        + ": "
        + has(names(positions(), List.of()));
  }

  /**
   * Why an output isn't one of this joint's own, or null when it is: {@link Output#STOPPED}, which
   * every joint has, or one of its name at what it runs at, or ran at before it was re-valued
   * ({@link #formerOutputs}), as for a position ({@link #whyNotOwn(Position)}).
   */
  default @Nullable String whyNotOwn(Output output) {
    if (output.name().equals(Output.STOPPED_NAME)) {
      return null;
    }
    for (Output own : outputs()) {
      if (!own.name().equals(output.name())) {
        continue;
      }
      if (had(own, output) || formerOutputs().stream().anyMatch(was -> had(was, output))) {
        return null;
      }
      boolean revolute = this instanceof Revolute;
      return notHad(
          own.name(),
          "runs at " + Places.shownOutput(revolute, own),
          Places.shownOutput(revolute, output));
    }
    return label()
        + " has no output "
        + output.name()
        + ": "
        + has(names(List.of(), outputs()))
        + ", and "
        + Output.STOPPED_NAME;
  }

  /**
   * A place of one of its names, at a value it hasn't had: "elevator's L1 is at 0.5 m, and this L1,
   * at 0.25 m, isn't one it has had: another joint's or another robot's; give elevator's own".
   *
   * @param own how its own is: "is at 0.5 m"
   * @param given where the one given is: "0.25 m"
   */
  private String notHad(String name, String own, String given) {
    return possessive(label())
        + " "
        + name
        + " "
        + own
        + ", and this "
        + name
        + ", at "
        + given
        + ", isn't one it has had: another joint's or another robot's; give "
        + possessive(label())
        + " own";
  }

  /**
   * Whether a position is one a joint had, of its name and kind, there: to a nanometer, or a
   * nanoradian, so a place written in degrees and one in radians, a bit apart, are one.
   */
  private static boolean had(Position had, Position position) {
    return had.name().equals(position.name())
        && had.kind() == position.kind()
        && Math.abs(had.value() - position.value()) <= Range.EDGE;
  }

  /** Whether an output is one a joint had, of its name and kind, at its value, as a position. */
  private static boolean had(Output had, Output output) {
    return had.name().equals(output.name())
        && had.kind() == output.kind()
        && had.motion().equals(output.motion())
        && Math.abs(had.value() - output.value()) <= Range.EDGE;
  }

  /**
   * A joint's block: everything its declaration can say, and everything a practice robot can change
   * on the finished joint, written once for both kinds of joint. Each call returns a new joint, so
   * a block is a chain of them.
   *
   * <p>The two kinds differ only in what they are measured in, which the type parameters carry, so
   * the compiler refuses a length given to a joint that turns. Each kind supplies the six copies of
   * itself that the rest of the block is written with.
   *
   * @param <J> the kind of joint the block declares, which every call returns
   * @param <P> what its positions are measured in: an angle or a distance
   * @param <S> what its speeds are measured in: an angular or a linear velocity
   */
  sealed interface Block<J extends Block<J, P, S>, P extends Measure<?>, S extends Measure<?>>
      extends Joint permits Revolute, Prismatic {
    /**
     * Its hard stops: {@code .range(Range.degrees(0, 135))}. On a finished joint this moves its
     * stops and keeps where it rests: a practice robot's turret, {@code
     * TURRET.range(Range.degrees(-355, 5))}. The robot's definition checks everything else against
     * them when it is built, so a practice robot may move its stops and its places in either order.
     */
    J range(Range range);

    /** The same joint, driven otherwise. */
    J driven(Driven driven);

    /** The same joint, resting elsewhere, in its own units. */
    J withRest(double rest);

    /** The same joint, moving another link. */
    J withLink(Link link);

    /** The same joint with other named positions, and those it had before they were re-placed. */
    J withPositions(List<Position> positions, List<Position> former);

    /** The same joint with other named outputs, and those it had before they were re-valued. */
    J withOutputs(List<Output> outputs, List<Output> former);

    /**
     * Where it rests as the robot starts, stowed: {@code .rest(Degrees.of(-30))}. Without one it
     * rests at its zero. On a finished joint this moves where it rests and keeps its stops. The
     * robot's definition checks that it is inside them.
     */
    default J rest(P rest) {
      return withRest(inJointUnits(rest));
    }

    /**
     * Where it rests as the robot starts, at one of its positions: {@code .rest(ARM_STOWED)}. The
     * position is this joint's own of that name, so on a practice robot that re-placed it, the
     * joint rests where the practice robot put it. A block may say this before it names its
     * positions.
     *
     * @throws IllegalArgumentException if the position is another joint's, or of the other kind
     */
    default J rest(Position rest) {
      return withRest(restAt(this, rest));
    }

    /**
     * It at a position or beyond it, toward its upper stop: {@code PIVOT.atLeast(Degrees.of(110))}.
     * An interlock's edge is usually a clearance like this one, not a place the joint goes, so it
     * is written as a number.
     *
     * @throws IllegalArgumentException if it is outside its range, or its lower end
     */
    default Condition.AtLeast atLeast(P position) {
      return ConditionAtLeastBuilder.builder().joint(this).position(inJointUnits(position)).build();
    }

    /**
     * It at a position or short of it, toward its lower stop: {@code
     * ELEVATOR.atMost(Inches.of(32))}.
     *
     * @throws IllegalArgumentException if it is outside its range, or its upper end
     */
    default Condition.AtMost atMost(P position) {
      return ConditionAtMostBuilder.builder().joint(this).position(inJointUnits(position)).build();
    }

    /**
     * It between two positions, both included, given in either order.
     *
     * @throws IllegalArgumentException if either is outside its range, or they are the same
     */
    default Condition.Between between(P from, P to) {
      return ConditionBetweenBuilder.builder()
          .joint(this)
          .from(inJointUnits(from))
          .to(inJointUnits(to))
          .build();
    }

    /**
     * It running forward at a speed or faster: {@code SHOOTER.upToSpeed(RPM.of(3000))}.
     *
     * @throws IllegalArgumentException for a speed that is not forward
     */
    default Condition.UpToSpeed upToSpeed(S speed) {
      return ConditionUpToSpeedBuilder.builder().joint(this).speed(inJointUnits(speed)).build();
    }

    /** What it moves. */
    default J carries(Link link) {
      saidOnce(
          !link().equals(Link.unseen(label())),
          "carries " + link().label(),
          "it moves one link, so give it once, with everything it carries");
      return withLink(link);
    }

    /** Its places by name: {@code .positions(ARM_STOWED, ARM_SCORING)}. */
    default J positions(Position... positions) {
      saidOnce(
          !positions().isEmpty(),
          "has its positions, " + String.join(", ", names(positions(), List.of())),
          "name them all in one positions(...); to put one elsewhere, replaced(...)");
      return withPositions(List.of(positions), formerPositions());
    }

    /** What it runs at, by name: {@code .outputs(INTAKING, OUTTAKING)}. */
    default J outputs(Output... outputs) {
      saidOnce(
          !outputs().isEmpty(),
          "has its outputs, " + String.join(", ", names(List.of(), outputs())),
          "name them all in one outputs(...); to run one otherwise, replaced(...)");
      return withOutputs(List.of(outputs), formerOutputs());
    }

    /**
     * The same joint with one of its named positions somewhere else: a practice robot's, {@code
     * PIVOT.replaced(PIVOT_OUT, Degrees.of(128))}. Code that names the position finds it there on
     * the robot that has this joint, so the same code drives each robot to its own.
     *
     * @throws IllegalArgumentException if the position is not this joint's own
     */
    default J replaced(Position position, P at) {
      Position replacing =
          Position.builder()
              .name(position.name())
              .kind(Position.Kind.of(this))
              .value(inJointUnits(at))
              .build();
      return withPositions(
          replacedPosition(this, replacing), former(formerPositions(), own(position), replacing));
    }

    /**
     * The same joint with one of its named outputs at another speed: {@code
     * FEEDER.replaced(FEEDER_FEEDING, RPM.of(1200))}.
     *
     * @throws IllegalArgumentException if it has no output of the name
     */
    default J replaced(Output output, S speed) {
      return revalued(
          output, Output.speed(output.name(), inJointUnits(speed), Position.Kind.of(this)));
    }

    /**
     * The same joint with one of its named outputs at another voltage: {@code
     * ROLLER.replaced(OUTTAKING, Volts.of(-3))}.
     *
     * @throws IllegalArgumentException if it has no output of the name
     */
    default J replaced(Output output, Voltage voltage) {
      return revalued(output, Output.of(output.name(), voltage));
    }

    /**
     * The same joint with one of its named outputs at another share of the battery's voltage:
     * {@code ROLLER.replaced(OUTTAKING, Percent.of(-30))}.
     *
     * @throws IllegalArgumentException if it has no output of the name
     */
    default J replaced(Output output, Dimensionless share) {
      return revalued(output, Output.dutyCycle(output.name(), share.in(Value)));
    }

    private J revalued(Output output, Output replacing) {
      return withOutputs(
          replacedOutput(this, replacing), former(formerOutputs(), own(output), replacing));
    }

    /**
     * The motors that drive it: {@code
     * .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(32)))}. A joint that slides
     * says its drum as well: {@code .through(Gearing.of(5)).drum(Inches.of(0.75))}.
     */
    default J motors(Driven.Motors motors) {
      saidOnce(
          driven().motors().isPresent(),
          "has its motors",
          "to re-wire them, motors(motors -> motors.on(...)); to re-gear them, motors(motors ->"
              + " motors.gearing(...))");
      return driven(drivenWith().motors(motors).build());
    }

    @Override
    default J motors(UnaryOperator<Driven.Motors> change) {
      Driven.Motors had = toChange(driven().motors(), "motors to change", ".motors(...) first");
      Driven.Motors changed = change.apply(had);
      if (changed.count() != had.count()) {
        throw new IllegalArgumentException(
            label()
                + " has "
                + had.count()
                + " motors, and motors.on(...) names "
                + changed.count()
                + ": name a controller for each, the leader first");
      }
      return driven(drivenWith().motors(changed).build());
    }

    /**
     * A servo pushing between its parent and its link, under its own label. A joint may have
     * several.
     *
     * @param onParent where its body is fixed, from the joint's origin, in the parent's frame
     * @param onLink where its rod meets the link, in the joint's frame
     */
    default J servo(
        String label, Actuator.Servo servo, Translation3d onParent, Translation3d onLink) {
      List<Driven.Servo> servos = new ArrayList<>(driven().servos());
      servos.add(
          DrivenServoBuilder.builder()
              .label(label)
              .servo(servo)
              .onParent(onParent)
              .onLink(onLink)
              .build());
      return driven(drivenWith().servos(servos).build());
    }

    /**
     * An absolute encoder turning with it, written in full: geared, {@code
     * .absoluteEncoder(AbsoluteEncoder.of().ratio(0.536).on(cancoder))}, or one only the simulator
     * reads, {@code .absoluteEncoder(AbsoluteEncoder.of())}. On a joint that turns it is on the
     * axle. On one that slides it is on the drum's shaft, and its ratio is its turns for each turn
     * of the drum.
     */
    default J absoluteEncoder(Driven.AbsoluteEncoder encoder) {
      saidOnce(
          driven().absoluteEncoder().isPresent(),
          "has an absolute encoder",
          "to re-wire it, absoluteEncoder(encoder -> encoder.on(...)); to offset it,"
              + " absoluteEncoder(encoder -> encoder.offset(...)); a decoded pair isn't supported"
              + " yet");
      return driven(drivenWith().absoluteEncoder(encoder).build());
    }

    /**
     * An absolute encoder turning with it one to one, read by a CANcoder, its SPARK's data port or
     * one of Systemcore's ports: {@code
     * .absoluteEncoder(CANCODER.can(34).offset(Rotations.of(0.82)))}.
     */
    default J absoluteEncoder(MechanismWiring.EncoderWiring wiring) {
      return absoluteEncoder(Driven.AbsoluteEncoder.of().on(wiring));
    }

    @Override
    default J absoluteEncoder(UnaryOperator<Driven.AbsoluteEncoder> change) {
      Driven.AbsoluteEncoder had =
          toChange(
              driven().absoluteEncoder(),
              "absolute encoder to change",
              "one, .absoluteEncoder(...)");
      return driven(drivenWith().absoluteEncoder(change.apply(had)).build());
    }

    /**
     * A limit switch at one end of its travel: {@code
     * .limitSwitch(LimitSwitch.lower().at(Degrees.of(0)).on(SensorWiring.port(1).normallyClosed()))}.
     * One that does not say where it trips can stop the joint but cannot home it.
     *
     * @throws IllegalArgumentException if it trips at a distance and the joint turns, or at an
     *     angle and the joint slides
     */
    default J limitSwitch(Driven.LimitSwitch limit) {
      String end = lowerOrUpper(limit.end());
      saidOnce(
          limitAt(limit.end()).isPresent(),
          "has a limit switch at its " + end + " end",
          "to re-wire it, limitSwitch(End." + limit.end().name() + ", limit -> limit.on(...))");
      List<Driven.LimitSwitch> limits = new ArrayList<>(driven().limitSwitches());
      limits.add(ofThisKind(limit));
      return driven(drivenWith().limitSwitches(limits).build());
    }

    @Override
    default J limitSwitch(Sensor.Switch.Limit.End end, UnaryOperator<Driven.LimitSwitch> change) {
      Driven.LimitSwitch had =
          toChange(
              limitAt(end),
              "limit switch at its " + lowerOrUpper(end) + " end to change",
              "one, .limitSwitch(LimitSwitch." + lowerOrUpper(end) + "()...)");
      List<Driven.LimitSwitch> limits = new ArrayList<>(driven().limitSwitches());
      limits.set(limits.indexOf(had), ofThisKind(change.apply(had)));
      return driven(drivenWith().limitSwitches(limits).build());
    }

    private Optional<Driven.LimitSwitch> limitAt(Sensor.Switch.Limit.End end) {
      return driven().limitSwitches().stream().filter(each -> each.end() == end).findFirst();
    }

    /** A limit switch, refused if where it trips is not measured as this joint's positions are. */
    private Driven.LimitSwitch ofThisKind(Driven.LimitSwitch limit) {
      if (limit.position().isEmpty()) {
        return limit;
      }
      Position.Kind tripsAt =
          limit.position().get() instanceof Angle ? Position.Kind.ANGLE : Position.Kind.LENGTH;
      if (tripsAt == Position.Kind.of(this)) {
        return limit;
      }
      throw new IllegalArgumentException(
          possessive(label())
              + " "
              + lowerOrUpper(limit.end())
              + " limit switch trips at "
              + switch (tripsAt) {
                case LENGTH ->
                    "a length, and the joint turns: give it an angle, .at(Degrees.of(...))";
                case ANGLE ->
                    "an angle, and the joint slides: give it a length, .at(Inches.of(...))";
              });
    }

    /**
     * It kept out of a stretch of its travel while a condition holds, enforced beneath every
     * command: {@code .keepsOut(Range.degrees(-180, -101)).when(ELEVATOR.atMost(Inches.of(32)))}.
     * Its target is held at the stretch's edge, on the side it is on, until the condition clears.
     */
    default KeepingOut<J> keepsOut(Range region) {
      return new KeepingOut<>(this, region, this::interlocked);
    }

    /**
     * It moving only while a condition holds, enforced beneath every command: {@code
     * .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))}. Meanwhile it holds where it is, or stops.
     */
    default J runsOnlyWhen(Condition condition) {
      return interlocked(Interlock.that(this).runsOnlyWhen(condition));
    }

    private J interlocked(Interlock interlock) {
      List<Interlock> interlocks = new ArrayList<>(driven().interlocks());
      interlocks.add(interlock);
      return driven(drivenWith().interlocks(interlocks).build());
    }

    @Override
    default J control(MechanismControl control) {
      saidOnce(
          driven().control().isPresent(),
          "has its control",
          "to re-tune it, retuned(control -> control.withKg(...))");
      return driven(drivenWith().control(control).build());
    }

    @Override
    default J retuned(UnaryOperator<MechanismControl> change) {
      MechanismControl had =
          toChange(driven().control(), "control to re-tune", "one, .control(...)");
      return driven(drivenWith().control(change.apply(had)).build());
    }

    /**
     * The rule that a joint has one of each part its block says once: one link, one set of
     * positions, of outputs and of motors, one absolute encoder, one limit switch at each end, and
     * one control. A block that says one again is refused, with how to change the one it has.
     *
     * @param has what the joint already has, as the refusal says it: "has its motors"
     * @param instead what to write instead
     */
    private void saidOnce(boolean alreadySaid, String has, String instead) {
      if (alreadySaid) {
        throw new IllegalArgumentException(label() + " already " + has + ": " + instead);
      }
    }

    /**
     * A part of this joint that a practice robot is changing, refused if the joint has none.
     *
     * @param what the part, and what was to be done with it: "motors to change"
     * @param give how its block gives the joint one: ".motors(...) first"
     */
    private <T> T toChange(Optional<T> part, String what, String give) {
      return part.orElseThrow(
          () -> new IllegalArgumentException(label() + " has no " + what + ": give it " + give));
    }

    private DrivenBuilder drivenWith() {
      return DrivenBuilder.builder(driven());
    }
  }

  /**
   * A joint that turns about its axis, in radians: an arm, a wrist, a turret, a roller.
   *
   * @param label what logs and writers call it: unique among the robot's joints
   * @param parent what it hangs from: the chassis, or another joint's link
   * @param origin where its frame is in its parent's frame at position 0: the pivot
   * @param axis what it turns about, in its own frame, positive by the right-hand rule
   * @param range its hard stops, in radians, unwrapped; empty for one that turns freely
   * @param rest where it rests as the robot starts, in radians, if its block says: stowed, inside
   *     its range; none starts it at 0
   * @param link what it moves
   * @param positions its places by name, in radians, inside its range: {@code
   *     Position.degrees("out", 130)}
   * @param outputs what code runs it at, by name: {@code Output.rpm("intaking", 2000)}, {@code
   *     Output.volts("push", 6)}
   * @param formerPositions where its positions were before they were re-placed ({@code replaced}):
   *     the competition robot's, on a practice robot, still its own
   * @param formerOutputs what its outputs were before they were re-valued, still its own
   * @param driven how it's driven and wired, its own sensors, and its control: its block's {@code
   *     .motor(...)} and the rest; not part of its identity
   */
  @ConfigRecord
  record Revolute(
      String label,
      Part parent,
      Transform3d origin,
      Vector3 axis,
      Optional<Range> range,
      Optional<Double> rest,
      Link link,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> positions,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> outputs,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> formerPositions,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> formerOutputs,
      @RecordBuilder.Initializer(value = "NONE", source = Driven.class) Driven driven)
      implements Block<Revolute, Angle, AngularVelocity> {
    public Revolute {
      positions = List.copyOf(positions);
      outputs = List.copyOf(outputs);
      formerPositions = List.copyOf(formerPositions);
      formerOutputs = List.copyOf(formerOutputs);
      check(Revolute.class, label, origin, axis, range, rest, positions, outputs);
      Optional<Driven.Motors> motors = driven.motors();
      Optional<MechanismControl> control = driven.control();
      Checks.on(Revolute.class, label)
          .check(
              "driven.motors",
              () ->
                  motors.isPresent() && motors.get().pitchRadius().isPresent()
                      ? "a joint that turns has no drum: give its motors a gearing alone"
                      : null)
          .check(
              "driven.control",
              () -> control.map(c -> c.ofTheOtherKind(Position.Kind.ANGLE)).orElse(null))
          .orThrow();
    }

    /** Starts describing a revolute joint, one named value at a time. */
    public static JointRevoluteBuilder.LabelStage builder() {
      return JointRevoluteBuilder.builder();
    }

    @Override
    public Transform3d motion(double position) {
      return new Transform3d(
          Translation3d.ZERO,
          new Rotation3d(VecBuilder.fill(axis.x(), axis.y(), axis.z()), position));
    }

    @Override
    public boolean equals(@Nullable Object other) {
      return sameJoint(this, other);
    }

    @Override
    public int hashCode() {
      return identityHash(this);
    }

    @Override
    public Revolute range(Range range) {
      return with().range(range).build();
    }

    @Override
    public Revolute driven(Driven driven) {
      return with().driven(driven).build();
    }

    @Override
    public Revolute withRest(double rest) {
      return with().rest(rest).build();
    }

    @Override
    public Revolute withLink(Link link) {
      return with().link(link).build();
    }

    @Override
    public Revolute withPositions(List<Position> positions, List<Position> former) {
      return with().positions(positions).formerPositions(former).build();
    }

    @Override
    public Revolute withOutputs(List<Output> outputs, List<Output> former) {
      return with().outputs(outputs).formerOutputs(former).build();
    }

    private JointRevoluteBuilder with() {
      return JointRevoluteBuilder.builder(this);
    }
  }

  /**
   * A joint that slides along its axis, in meters: an elevator's carriage or stage, a telescoping
   * arm, a linear deploy.
   *
   * @param label what logs and writers call it: unique among the robot's joints
   * @param parent what it hangs from: the chassis, or another joint's link
   * @param origin where its frame is in its parent's frame at position 0: the slide's start
   * @param axis what it slides along, in its own frame
   * @param range its hard stops, in meters
   * @param rest where it rests as the robot starts, in meters, if its block says: stowed, inside
   *     its range; none starts it at 0
   * @param link what it moves
   * @param positions its places by name, in meters, inside its range: {@code Position.of("clear",
   *     Inches.of(20))}
   * @param outputs what code runs it at, by name: a speed in meters per second, a voltage, or a
   *     duty cycle
   * @param formerPositions where its positions were before they were re-placed ({@code replaced}):
   *     the competition robot's, on a practice robot, still its own
   * @param formerOutputs what its outputs were before they were re-valued, still its own
   * @param driven how it's driven and wired, its own sensors, and its control: its block's {@code
   *     .motors(...)} and the rest; not part of its identity
   */
  @ConfigRecord
  record Prismatic(
      String label,
      Part parent,
      Transform3d origin,
      Vector3 axis,
      Optional<Range> range,
      Optional<Double> rest,
      Link link,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> positions,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> outputs,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> formerPositions,
      @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> formerOutputs,
      @RecordBuilder.Initializer(value = "NONE", source = Driven.class) Driven driven)
      implements Block<Prismatic, Distance, LinearVelocity> {
    public Prismatic {
      positions = List.copyOf(positions);
      outputs = List.copyOf(outputs);
      formerPositions = List.copyOf(formerPositions);
      formerOutputs = List.copyOf(formerOutputs);
      check(Prismatic.class, label, origin, axis, range, rest, positions, outputs);
      Optional<Driven.Motors> motors = driven.motors();
      Optional<MechanismControl> control = driven.control();
      Checks.on(Prismatic.class, label)
          .check(
              "driven.motors",
              () ->
                  motors.isPresent() && motors.get().pitchRadius().isEmpty()
                      ? "a joint that slides needs its drum: give its motors the pitch radius"
                      : null)
          .check(
              "driven.control",
              () ->
                  control.isPresent() && control.get().gains().gravityAngle().isPresent()
                      ? "a joint that slides is held against gravity the same all along it, so"
                          + " its kG has no angle: give kG alone, withKg(kG)"
                      : null)
          .check(
              "driven.control",
              () -> control.map(c -> c.ofTheOtherKind(Position.Kind.LENGTH)).orElse(null))
          .orThrow();
    }

    /** Starts describing a prismatic joint, one named value at a time. */
    public static JointPrismaticBuilder.LabelStage builder() {
      return JointPrismaticBuilder.builder();
    }

    @Override
    public Transform3d motion(double position) {
      Vector3 along = axis.unit().times(position);
      return new Transform3d(new Translation3d(along.x(), along.y(), along.z()), Rotation3d.ZERO);
    }

    @Override
    public boolean equals(@Nullable Object other) {
      return sameJoint(this, other);
    }

    @Override
    public int hashCode() {
      return identityHash(this);
    }

    @Override
    public Prismatic range(Range range) {
      return with().range(range).build();
    }

    @Override
    public Prismatic driven(Driven driven) {
      return with().driven(driven).build();
    }

    @Override
    public Prismatic withRest(double rest) {
      return with().rest(rest).build();
    }

    @Override
    public Prismatic withLink(Link link) {
      return with().link(link).build();
    }

    @Override
    public Prismatic withPositions(List<Position> positions, List<Position> former) {
      return with().positions(positions).formerPositions(former).build();
    }

    @Override
    public Prismatic withOutputs(List<Output> outputs, List<Output> former) {
      return with().outputs(outputs).formerOutputs(former).build();
    }

    private JointPrismaticBuilder with() {
      return JointPrismaticBuilder.builder(this);
    }
  }

  /**
   * Whether a joint and another object are the same joint. Two joints are the same when they are of
   * one kind and agree on everything but how they are driven, where their stops are, where they
   * rest, and where their named places are. A practice robot's joint is therefore equal to the
   * competition robot's, and stands in its place.
   */
  private static boolean sameJoint(Joint joint, @Nullable Object other) {
    return other instanceof Joint that
        && joint.getClass() == that.getClass()
        && joint.label().equals(that.label())
        && joint.parent().equals(that.parent())
        && joint.origin().equals(that.origin())
        && joint.axis().equals(that.axis())
        && joint.range().isPresent() == that.range().isPresent()
        && joint.link().equals(that.link())
        && names(joint.positions(), List.of()).equals(names(that.positions(), List.of()))
        && names(List.of(), joint.outputs()).equals(names(List.of(), that.outputs()));
  }

  /** A hash of what makes two joints the same ({@link #sameJoint}). */
  private static int identityHash(Joint joint) {
    return Objects.hash(
        joint.label(),
        joint.parent(),
        joint.origin(),
        joint.axis(),
        joint.range().isPresent(),
        joint.link(),
        names(joint.positions(), List.of()),
        names(List.of(), joint.outputs()));
  }

  /**
   * A measure of a joint's position or speed as the model keeps it: radians, meters, radians per
   * second or meters per second. A block's type parameters let nothing else reach this.
   */
  private static double inJointUnits(Measure<?> measure) {
    return switch (measure) {
      case Angle angle -> angle.in(Radians);
      case Distance distance -> distance.in(Meters);
      case AngularVelocity turning -> turning.in(RadiansPerSecond);
      case LinearVelocity sliding -> sliding.in(MetersPerSecond);
      default -> throw new IllegalArgumentException(measure + " is not a position or a speed");
    };
  }

  /**
   * Where a joint rests at one of its positions: its own of the position's name, where this joint
   * puts it; or, a name it hasn't (in its block, before its positions are named), where the
   * position is. Whether that's inside its stops is the robot's definition's to judge.
   */
  private static double restAt(Joint joint, Position rest) {
    boolean named = joint.positions().stream().anyMatch(each -> each.name().equals(rest.name()));
    if (named) {
      return joint.own(rest).value();
    }
    boolean revolute = joint instanceof Revolute;
    if (rest.kind() != Position.Kind.of(revolute)) {
      throw new IllegalArgumentException(
          joint.label()
              + " can't rest at "
              + rest.name()
              + ", "
              + (revolute
                  ? "a length, since it turns: give it an angle, rest(Degrees.of(...))"
                  : "an angle, since it slides: give it a length, rest(Inches.of(...))"));
    }
    return rest.value();
  }

  /** A joint's positions, the one of a position's name in its place, there instead. */
  private static List<Position> replacedPosition(Joint joint, Position replacing) {
    List<Position> positions = new ArrayList<>();
    for (Position each : joint.positions()) {
      positions.add(each.name().equals(replacing.name()) ? replacing : each);
    }
    if (!positions.contains(replacing)) {
      throw new IllegalArgumentException(
          joint.label()
              + " has no position "
              + replacing.name()
              + " to re-place: "
              + has(names(joint.positions(), List.of()))
              + ". Its positions are what its code names, the same on every robot of the design");
    }
    return positions;
  }

  /** A joint's outputs, the one of an output's name in its place, at its value instead. */
  private static List<Output> replacedOutput(Joint joint, Output replacing) {
    List<Output> outputs = new ArrayList<>();
    for (Output each : joint.outputs()) {
      outputs.add(each.name().equals(replacing.name()) ? replacing : each);
    }
    if (!outputs.contains(replacing)) {
      throw new IllegalArgumentException(
          joint.label()
              + " has no output "
              + replacing.name()
              + " to re-value: "
              + has(names(List.of(), joint.outputs()))
              + ". Its outputs are what its code names, the same on every robot of the design");
    }
    return outputs;
  }

  /**
   * A place's former values with the one it has now, as it's re-placed elsewhere: each once, so a
   * constant at any is still the joint's own.
   */
  private static <P> List<P> former(List<P> former, P now, P replacing) {
    List<P> had = new ArrayList<>(former);
    if (!now.equals(replacing) && !had.contains(now)) {
      had.add(now);
    }
    return had;
  }

  /**
   * A joint's own checks, the same for both kinds. Whether where it rests and its places are inside
   * its stops is its robot's definition's to judge, on the finished joint ({@code Blocks}), so a
   * practice robot changes them in either order.
   */
  private static void check(
      Class<? extends Record> kind,
      String label,
      Transform3d origin,
      Vector3 axis,
      Optional<Range> range,
      Optional<Double> rest,
      List<Position> positions,
      List<Output> outputs) {
    boolean revolute = kind == Revolute.class;
    Checks.on(kind, label)
        .field("label", label, notBlank())
        .field("origin", origin, Joint::notATransform)
        .field(
            "axis",
            axis,
            a ->
                Double.isFinite(a.length()) && a.length() > 0
                    ? null
                    : a + " must point some way: an axis can't be zero")
        .field("rest", rest, given(finite()))
        .check(
            "range",
            () ->
                range.isPresent() && range.get().kind() != Position.Kind.of(revolute)
                    ? wrongKind("its range is", revolute)
                    : null)
        .check("positions", () -> ofTheOtherKind(revolute, positions))
        .check("outputs", () -> twice(positions, outputs))
        .check("outputs", () -> speedsOfTheOtherKind(label, revolute, outputs))
        .orThrow();
  }

  /**
   * A stretch of a joint's travel given as the other kind, lengths for a joint that turns or angles
   * for one that slides, and how to give it: {@code "its range is lengths, and the joint turns:
   * give it in angles, Range.degrees(...)"}.
   *
   * @param what what it is, and the verb: {@code "its range is"}
   */
  static String wrongKind(String what, boolean revolute) {
    return what
        + (revolute
            ? " lengths, and the joint turns: give it in angles, Range.degrees(...)"
            : " angles, and the joint slides: give it in lengths, Range.meters(...) or"
                + " Range.of(Inches.of(...), Inches.of(...))");
  }

  /**
   * The named positions of the wrong kind, a length for a joint that turns or an angle for one that
   * slides, or null when they're all of its kind. Whether they're inside its stops is the
   * definition's to judge, on the finished joint, so a practice robot's stops and places change in
   * either order.
   */
  private static @Nullable String ofTheOtherKind(boolean revolute, List<Position> positions) {
    Position.Kind kind = revolute ? Position.Kind.ANGLE : Position.Kind.LENGTH;
    List<String> wrong = new ArrayList<>();
    for (Position position : positions) {
      if (position.kind() != kind) {
        wrong.add(
            position.name()
                + " is "
                + (revolute
                    ? "a length, and the joint turns: give it an angle, Position.degrees(...)"
                    : "an angle, and the joint slides: give it a length, Position.of(name,"
                        + " Inches.of(...))"));
      }
    }
    return wrong.isEmpty() ? null : String.join("; ", wrong);
  }

  /**
   * The named speeds of the wrong kind, of turning for a joint that slides or of sliding for one
   * that turns, or null when they're all of its kind: an rpm on an elevator would run in meters a
   * second.
   */
  private static @Nullable String speedsOfTheOtherKind(
      String label, boolean revolute, List<Output> outputs) {
    Position.Kind kind = Position.Kind.of(revolute);
    List<String> wrong = new ArrayList<>();
    for (Output output : outputs) {
      if (output.motion().isPresent() && output.motion().get() != kind) {
        wrong.add(
            output.name()
                + ", "
                + Places.shownSpeed(!revolute, output.value())
                + ", is "
                + (revolute
                    ? "a speed along a slide, and "
                        + label
                        + " turns: give it a speed of turning, Output.rpm(\""
                        + output.name()
                        + "\", ...)"
                    : "a speed of turning, and "
                        + label
                        + " slides: give it a speed along it, Output.of(\""
                        + output.name()
                        + "\", InchesPerSecond.of(...))"));
      }
    }
    return wrong.isEmpty() ? null : String.join("; ", wrong);
  }

  /**
   * The names used twice among its positions and outputs, or the built-in output's, or null when
   * none are.
   */
  private static @Nullable String twice(List<Position> positions, List<Output> outputs) {
    Set<String> seen = new HashSet<>();
    Set<String> twice = new LinkedHashSet<>();
    for (String name : names(positions, outputs)) {
      if (!seen.add(name)) {
        twice.add(name);
      }
    }
    if (seen.contains(Output.STOPPED_NAME)) {
      return "it names "
          + Output.STOPPED_NAME
          + ", which every joint has built in (its motors at neutral): name it something else";
    }
    return twice.isEmpty()
        ? null
        : "it names "
            + String.join(", ", twice)
            + " twice: each name is one position or output of the joint";
  }

  /** What a joint has by name, for a message: "it has in, run, out", or "it has none". */
  private static String has(List<String> names) {
    return "it has " + (names.isEmpty() ? "none" : String.join(", ", names));
  }

  /** Its positions' names, then its outputs'. */
  private static List<String> names(List<Position> positions, List<Output> outputs) {
    List<String> names = new ArrayList<>();
    positions.forEach(position -> names.add(position.name()));
    outputs.forEach(output -> names.add(output.name()));
    return names;
  }

  private static @Nullable String notATransform(Transform3d transform) {
    Quaternion turn = transform.getRotation().getQuaternion();
    return Double.isFinite(
            transform.getX()
                + transform.getY()
                + transform.getZ()
                + turn.getW()
                + turn.getX()
                + turn.getY()
                + turn.getZ())
        ? null
        : transform + " must be a transform, numbers all";
  }
}
