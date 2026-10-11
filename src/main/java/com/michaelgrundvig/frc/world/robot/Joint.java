package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.robot.JointParts.inJointUnits;
import static com.michaelgrundvig.frc.world.robot.JointParts.lowerOrUpper;
import static org.wpilib.units.Units.Value;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Vector3;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;
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
 * A mechanism's joint: one way a link moves on what it hangs from. A joint either turns about an
 * axis ({@link Revolute}, measured in radians) or slides along one ({@link Prismatic}, measured in
 * meters). Joints hang from the chassis or from each other, and each is written after its parent,
 * so they form a tree: an arm on an elevator is a joint whose parent is the elevator's joint.
 *
 * <p>Position 0 is the pose the joint's {@link #origin} gives, so an arm's zero is wherever CAD's
 * frame put it. A joint has no state. Where it is at any moment comes from the robot's sensors, and
 * the functions that work something out from a joint take that position as an argument.
 *
 * <p>A joint names its own positions and outputs ({@link #positions}, {@link #outputs}), and
 * conditions on it are written from it: {@link #at}, {@link #atLeast(Position)}, {@link
 * #atMost(Position)} and {@link #between}.
 *
 * <p>A joint is declared by its block ({@link Block}), one call at a time. A joint has one link,
 * one set of named positions and one of outputs, one set of motors, one absolute encoder, one limit
 * switch at each end and one control. A block that says any of these twice is refused, with how to
 * change it. It may have several servos and several interlocks, which add up.
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
 * <p>A position or an output is a name on the joint, and its value belongs to the robot. Code that
 * names one, in a move, a configuration or an interlock, finds it where the robot running that code
 * puts it ({@link #own(Position)}), so the same code drives a practice robot to the practice
 * robot's own places. What makes a constant the joint's own, and what is refused, is in {@link
 * #whyNotOwn(Position)}.
 *
 * <p>This file holds what a joint is and what its block can say. What a block makes for the robot
 * is in {@link JointParts}, how a joint's names are looked up and judged is in {@link JointNames},
 * and the checks a joint makes of its own values are in {@link JointChecks}.
 */
public sealed interface Joint extends Part permits Joint.Block {
  /**
   * The axis of a pivot whose link points forward at zero and swings up as its position grows. It
   * is the robot's y axis, reversed.
   */
  Vector3 PITCH_UP = new Vector3(0, -1, 0);

  /**
   * The axis of a pivot whose link points forward at zero and swings down as its position grows.
   */
  Vector3 PITCH_DOWN = new Vector3(0, 1, 0);

  /**
   * A pivot: an arm, an intake's swing or a wrist, turning about an axis. Its block says the rest,
   * one call at a time: {@code Joint.pivot("pivot", Part.CHASSIS, at,
   * Joint.PITCH_DOWN).range(Range.degrees(0, 135)).rest(PIVOT_IN).carries(link).positions(PIVOT_IN,
   * PIVOT_OUT).motors(...).control(...)}.
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
   * A roller, a belt or a flywheel: a joint that turns freely and is run at its outputs. {@code
   * Joint.roller("intake", PIVOT, at).outputs(INTAKING).motors(...)}.
   *
   * @param at where its axle is, in its parent's frame
   */
  static Revolute roller(String label, Part parent, Translation3d at) {
    return pivot(label, parent, at, PITCH_DOWN);
  }

  /**
   * A roller that turns about some other axis: a spindexer about the vertical is {@code
   * Joint.roller("spindexer", Part.CHASSIS, at, Vector3.UP)}.
   */
  static Revolute roller(String label, Part parent, Translation3d at, Vector3 axis) {
    return pivot(label, parent, at, axis);
  }

  /**
   * An elevator: a carriage sliding straight up. {@code Joint.elevator("elevator", Part.CHASSIS,
   * at).range(Range.meters(0, 1.2)).rest(ELEVATOR_DOWN).motors(...)}.
   *
   * @param at where the slide starts, in its parent's frame
   */
  static Prismatic elevator(String label, Part parent, Translation3d at) {
    return slide(label, parent, at, Vector3.UP);
  }

  /**
   * A joint sliding along any axis: a hopper that extends forward is {@code Joint.slide("hopper",
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

  /** What logs and writers call it. It is unique among the robot's joints. */
  @Override
  String label();

  /** What it hangs from: the chassis, or another joint's link. */
  Part parent();

  /**
   * Where its frame is in its parent's frame when it is at position 0: where the pivot is, or where
   * the slide starts.
   */
  Transform3d origin();

  /** The axis it turns about or slides along, in its own frame. Any length but zero will do. */
  Vector3 axis();

  /**
   * Its hard stops. The range is not wrapped to one turn, so a turret may span more than a turn. A
   * joint that turns freely, a roller or a flywheel, has none.
   */
  Optional<Range> range();

  /**
   * Where it rests as the robot starts, if its block says ({@code rest(...)}). A practice robot may
   * move it. A joint whose block does not say starts at its zero ({@link #restPosition}).
   */
  Optional<Double> rest();

  /**
   * Where it is as the robot starts: where it rests ({@link #rest}), or 0. This is where the robot
   * is weighed and modeled, stowed, and where a mechanism that zeroes itself at boot is taken to
   * be.
   */
  default double restPosition() {
    return rest().orElse(0.0);
  }

  /** What it moves. */
  Link link();

  /**
   * Its named positions, where code moves it to: an intake's "in", "run" and "out". Code names each
   * by its name, and each robot's joint puts it where that robot has it ({@code replaced}). A joint
   * that follows another has none, because its leader's moves carry it.
   */
  List<Position> positions();

  /**
   * Its named outputs, what code runs it at: a roller's "intaking" at a speed, its "outtaking" at a
   * voltage. {@link Output#STOPPED} is not among them, because every joint has it.
   */
  List<Output> outputs();

  /**
   * The values its positions had before they were re-placed ({@code replaced}). On a practice robot
   * these are the competition robot's values, which the team's constants hold, so a constant at one
   * of them still counts as this joint's own. A joint whose positions are where its block put them
   * has none.
   */
  List<Position> formerPositions();

  /** The values its outputs had before they were re-valued: see {@link #formerPositions}. */
  List<Output> formerOutputs();

  /**
   * Whether it is held at positions: it is if it has a range or any named position. A joint with
   * neither, a roller, only runs at outputs, and stops when nothing asks it to run.
   */
  default boolean holdsPositions() {
    return range().isPresent() || !positions().isEmpty();
  }

  /**
   * How it is driven and wired: its motors or servos, its own sensors, their wiring, and its
   * control. This is not part of what makes a joint the same joint: two joints that differ only
   * here are equal.
   */
  Driven driven();

  /**
   * How far its frame has moved from its origin when it is at a position: turned about its axis, or
   * slid along it.
   */
  Transform3d motion(double position);

  /**
   * How the program drives it, said once in its block: {@code
   * .control(MechanismControl.position(30))}.
   *
   * @throws IllegalArgumentException if it has a control already: {@link #retuned} changes it
   */
  Joint control(MechanismControl control);

  /**
   * The same joint with its control changed: a practice robot's lighter arm is {@code
   * ARM.retuned(control -> control.withKg(0.3))}.
   *
   * @throws IllegalArgumentException if it has no control to change
   */
  Joint retuned(UnaryOperator<MechanismControl> change);

  /**
   * The same joint with its motors changed: on another controller, {@code PIVOT.motors(motors ->
   * motors.on(TALON_FX.can(33).pdh(4, Amps.of(40))))}, or geared differently, {@code
   * PIVOT.motors(motors -> motors.gearing(Gearing.of(20)))}.
   *
   * @throws IllegalArgumentException if it has no motors, or the change leaves a different number
   *     of them
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
   * The same joint with its limit switch at one end changed: on another input, {@code
   * ELEVATOR.limitSwitch(End.LOWER, limit -> limit.on(SensorWiring.port(5).normallyClosed()))}.
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
    return Position.Kind.of(this).shown(value);
  }

  /**
   * A speed of it as people read one: rpm for a joint that turns, meters a second for one that
   * slides ("2000 rpm", "1.5 m/s").
   *
   * @param value in its units a second: radians, or meters
   */
  default String shownSpeed(double value) {
    return Position.Kind.of(this).shownSpeed(value);
  }

  /**
   * Its own drives, as its block says them: its motors, labeled as the joint is, and each servo
   * under the servo's own label. A drive between two joints, such as a differential's, belongs to
   * the robot instead ({@code RobotDefinition.drives}).
   */
  default List<Drive> ownDrives() {
    return JointParts.drives(this);
  }

  /**
   * Its own sensors, as its block says them: its absolute encoder ("arm absolute encoder") and its
   * limit switches ("arm lower limit", "arm upper limit"), each with its wiring.
   */
  default List<Sensor> ownSensors() {
    return JointParts.sensors(this);
  }

  /**
   * Its devices on the CAN bus, as its block wires them: its motor controllers, the leader first,
   * and then its CANcoder. A joint that runs in the simulator alone has none.
   */
  default List<CanDevice> devices() {
    return JointParts.devices(this);
  }

  /**
   * Its own interlocks, as its block says them ({@code .keepsOut(...).when(...)}, {@code
   * .runsOnlyWhen(...)}), each about this joint as the robot has it.
   */
  default List<Interlock> ownInterlocks() {
    return JointParts.interlocks(this);
  }

  /**
   * The second step of writing a keep-out in a joint's block: the joint and the region. It is not
   * an interlock until {@link #when} says while what.
   *
   * @param joint the joint
   * @param region where it may not be
   * @param adding what gives back the joint with the finished interlock added to its block
   * @param <J> the kind of joint the block declares
   */
  record KeepingOut<J extends Joint>(Joint joint, Range region, Function<Interlock, J> adding) {
    /** The region is closed while the condition holds. This gives back the joint, kept out. */
    public J when(Condition condition) {
      return adding.apply(Interlock.that(joint).keepsOut(region).when(condition));
    }
  }

  /**
   * Its limit switch at one end, as its block says it. A condition on the switch is made from it:
   * {@code PIVOT.switchAt(End.LOWER).tripped()}.
   *
   * @throws IllegalArgumentException if it has none at that end
   */
  default Sensor.Switch switchAt(Sensor.Switch.Limit.End end) {
    return JointParts.switchAt(this, end);
  }

  /**
   * Its position by a name.
   *
   * @throws IllegalArgumentException if it has none by that name, naming those it has
   */
  default Position position(String name) {
    return JointNames.position(this, name);
  }

  /**
   * Its output by a name: one of its own, or {@link Output#STOPPED}, which every joint has.
   *
   * @throws IllegalArgumentException if it has none by that name, naming those it has
   */
  default Output output(String name) {
    return JointNames.output(this, name);
  }

  /**
   * It at one of its positions, which is what a configuration is made of: {@code
   * Configuration.of("amp", ELEVATOR.at(ELEVATOR_CLEAR), ARM.at(ARM_SCORING))}. An interlock
   * refuses this condition, because it holds only with the joint exactly there and a measured
   * position never is. An interlock's edge is {@code atLeast}, {@code atMost} or {@code between}.
   *
   * <p>A position that is not this joint's own is still taken here. It is refused where the
   * condition is used ({@link Condition}).
   */
  default Condition.At at(Position position) {
    return ConditionAtBuilder.builder().joint(this).position(position).build();
  }

  /**
   * It at one of its outputs, or {@link Output#STOPPED}, which is what a configuration is made of:
   * {@code Configuration.of("feeding", FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING))}. An
   * output that is not this joint's own is refused where the condition is used ({@link Condition}).
   */
  default Condition.At at(Output output) {
    return ConditionAtBuilder.builder().joint(this).output(output).build();
  }

  /**
   * It at one of its positions or beyond it, toward the upper end of its range: {@code
   * PIVOT.atLeast(PIVOT_OUT)} holds from "out" on. An edge that is a clearance, and not a place the
   * joint goes, is written as a number instead ({@link Block#atLeast(Measure)}). A position that is
   * not this joint's own is refused where the condition is used ({@link Condition}).
   *
   * @throws IllegalArgumentException for the lower end of its range, where it would always hold
   */
  default Condition.AtLeast atLeast(Position position) {
    return ConditionAtLeastBuilder.builder()
        .joint(this)
        .position(whereItPuts(position))
        .place(position)
        .build();
  }

  /**
   * It at one of its positions or short of it, toward the lower end of its range: {@code
   * ELEVATOR.atMost(ELEVATOR_CLEAR)} holds up to "clear". A position that is not this joint's own
   * is refused where the condition is used ({@link Condition}).
   *
   * @throws IllegalArgumentException for the upper end of its range, where it would always hold
   */
  default Condition.AtMost atMost(Position position) {
    return ConditionAtMostBuilder.builder()
        .joint(this)
        .position(whereItPuts(position))
        .place(position)
        .build();
  }

  /**
   * It between two of its positions, both included, given in either order. A position that is not
   * this joint's own is refused where the condition is used ({@link Condition}).
   *
   * @throws IllegalArgumentException if the two are the same place
   */
  default Condition.Between between(Position from, Position to) {
    return ConditionBetweenBuilder.builder()
        .joint(this)
        .from(whereItPuts(from))
        .to(whereItPuts(to))
        .fromPlace(from)
        .toPlace(to)
        .build();
  }

  /**
   * Where this joint puts a position: its own value for a position that is its own. A position that
   * is not its own keeps the value it was given with, and whatever uses the condition that names it
   * refuses it.
   */
  private double whereItPuts(Position position) {
    if (whyNotOwn(position).isPresent()) {
      return position.value();
    }
    return own(position).value();
  }

  /**
   * This joint's own position of the name a position has, where this joint puts it. Code passes the
   * team's constant, and gets the position as the robot running the code has it: a practice robot
   * whose position was re-placed ({@code replaced}) gives its own.
   *
   * @throws IllegalArgumentException if the position is not this joint's own ({@link
   *     #whyNotOwn(Position)})
   */
  default Position own(Position position) {
    Optional<String> whyNotOwn = whyNotOwn(position);
    if (whyNotOwn.isPresent()) {
      throw new IllegalArgumentException(whyNotOwn.get());
    }
    return position(position.name());
  }

  /**
   * This joint's own output of the name an output has, at this joint's value, or {@link
   * Output#STOPPED}. A practice robot whose output was re-valued ({@code replaced}) gives its own.
   *
   * @throws IllegalArgumentException if the output is not this joint's own ({@link
   *     #whyNotOwn(Output)})
   */
  default Output own(Output output) {
    Optional<String> whyNotOwn = whyNotOwn(output);
    if (whyNotOwn.isPresent()) {
      throw new IllegalArgumentException(whyNotOwn.get());
    }
    return output(output.name());
  }

  /**
   * Why a position is not this joint's own, if it is not.
   *
   * <p>A position is the joint's own when the joint has a position of that name and kind, and the
   * one given is where the joint's is now, or where it was before it was re-placed ({@link
   * #formerPositions}). That second case is the competition robot's constant, used on its practice
   * robot. Anything else is refused: another joint's position of the same name somewhere else,
   * saying where this joint's is, or a name this joint does not have, saying which it has.
   *
   * @return the refusal, in plain words, or empty when the position is this joint's own
   */
  default Optional<String> whyNotOwn(Position position) {
    return JointNames.whyNotOwn(this, position);
  }

  /**
   * Why an output is not this joint's own, if it is not. {@link Output#STOPPED} is every joint's
   * own. Any other is judged as a position is ({@link #whyNotOwn(Position)}), against what the
   * joint's output of that name runs at now and what it ran at before it was re-valued ({@link
   * #formerOutputs}).
   *
   * @return the refusal, in plain words, or empty when the output is this joint's own
   */
  default Optional<String> whyNotOwn(Output output) {
    return JointNames.whyNotOwn(this, output);
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
      return withRest(JointNames.whereToRestAt(this, rest));
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
          "has its positions, " + String.join(", ", JointNames.ofPositions(positions())),
          "name them all in one positions(...); to put one elsewhere, replaced(...)");
      return withPositions(List.of(positions), formerPositions());
    }

    /** What it runs at, by name: {@code .outputs(INTAKING, OUTTAKING)}. */
    default J outputs(Output... outputs) {
      saidOnce(
          !outputs().isEmpty(),
          "has its outputs, " + String.join(", ", JointNames.ofOutputs(outputs())),
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
          JointNames.withReplaced(this, replacing),
          JointNames.formerWith(formerPositions(), own(position), replacing));
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
          JointNames.withReplaced(this, replacing),
          JointNames.formerWith(formerOutputs(), own(output), replacing));
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
   * A joint that turns about its axis, measured in radians: an arm, a wrist, a turret, a roller.
   *
   * @param label what logs and writers call it, unique among the robot's joints
   * @param parent what it hangs from: the chassis, or another joint's link
   * @param origin where its frame is in its parent's frame at position 0: where it pivots
   * @param axis what it turns about, in its own frame. A positive position turns it by the
   *     right-hand rule.
   * @param range its hard stops, in radians and not wrapped to one turn. A joint that turns freely
   *     has none.
   * @param rest where it rests as the robot starts, stowed, in radians, if its block says. Without
   *     one it starts at 0.
   * @param link what it moves
   * @param positions its named positions, in radians: {@code Position.degrees("out", 130)}
   * @param outputs its named outputs: {@code Output.rpm("intaking", 2000)}, {@code
   *     Output.volts("push", 6)}
   * @param formerPositions the values its positions had before they were re-placed ({@code
   *     replaced}), which still count as its own: on a practice robot, the competition robot's
   * @param formerOutputs the values its outputs had before they were re-valued, which still count
   *     as its own
   * @param driven how it is driven and wired, its own sensors, and its control, as its block says
   *     them. This is not part of what makes it the same joint.
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
      JointChecks.shared(
          Revolute.class,
          Position.Kind.ANGLE,
          label,
          origin,
          axis,
          range,
          rest,
          positions,
          outputs);
      JointChecks.turning(label, driven);
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
   * A joint that slides along its axis, measured in meters: an elevator's carriage or stage, a
   * telescoping arm, a linear deploy.
   *
   * @param label what logs and writers call it, unique among the robot's joints
   * @param parent what it hangs from: the chassis, or another joint's link
   * @param origin where its frame is in its parent's frame at position 0: where the slide starts
   * @param axis what it slides along, in its own frame
   * @param range its hard stops, in meters
   * @param rest where it rests as the robot starts, stowed, in meters, if its block says. Without
   *     one it starts at 0.
   * @param link what it moves
   * @param positions its named positions, in meters: {@code Position.of("clear", Inches.of(20))}
   * @param outputs its named outputs: a speed in meters a second, a voltage, or a duty cycle
   * @param formerPositions the values its positions had before they were re-placed ({@code
   *     replaced}), which still count as its own: on a practice robot, the competition robot's
   * @param formerOutputs the values its outputs had before they were re-valued, which still count
   *     as its own
   * @param driven how it is driven and wired, its own sensors, and its control, as its block says
   *     them. This is not part of what makes it the same joint.
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
      JointChecks.shared(
          Prismatic.class,
          Position.Kind.LENGTH,
          label,
          origin,
          axis,
          range,
          rest,
          positions,
          outputs);
      JointChecks.sliding(label, driven);
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
        && JointNames.ofPositions(joint.positions())
            .equals(JointNames.ofPositions(that.positions()))
        && JointNames.ofOutputs(joint.outputs()).equals(JointNames.ofOutputs(that.outputs()));
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
        JointNames.ofPositions(joint.positions()),
        JointNames.ofOutputs(joint.outputs()));
  }
}
