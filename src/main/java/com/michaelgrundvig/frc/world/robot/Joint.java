package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.robot.JointParts.inJointUnits;
import static com.michaelgrundvig.frc.world.robot.JointParts.lowerOrUpper;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

/**
 * A mechanism's joint: one way a link moves on what it hangs from. A joint either turns about an
 * axis ({@link RevoluteJoint}, measured in radians) or slides along one ({@link PrismaticJoint},
 * measured in meters). Joints hang from the chassis or from each other, and each is written after
 * its parent, so they form a tree: an arm on an elevator is a joint whose parent is the elevator's
 * joint.
 *
 * <p>Position 0 is the pose the joint's {@link #place} gives, so an arm's zero is wherever CAD's
 * frame put it. Where the joint is on its parent is said in its block, {@code .at(Inches.of(-8),
 * Inches.of(0), Inches.of(18.33))}; left out, it's at its parent's origin, and the robot says so at
 * startup. A joint has no state. Where it is at any moment comes from the robot's sensors, and the
 * functions that work something out from a joint take that position as an argument.
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
 * names one, in a move, a preset or an interlock, finds it where the robot running that code puts
 * it ({@link RobotDefinition#own(Joint, Position)}), so the same code drives a practice robot to
 * the practice robot's own places. What makes a constant the joint's own, and what is refused, is
 * in {@link RobotDefinition#whyNotOwn(Joint, Position)}.
 *
 * <p>This file holds what a joint is and what its block can say. What the program asks of a joint,
 * where it rests, how it moves, its devices and its named places, is on {@link RobotMath}. What a
 * block makes for the robot is in {@link JointParts}, how a joint's names are looked up and judged
 * is in {@link JointNames}, the copies its block makes are in {@link JointCopies}, and the checks a
 * joint makes of its own values are in {@link JointChecks}.
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
   * one call at a time: {@code Joint.pivot("pivot", Part.CHASSIS, Joint.PITCH_DOWN).at(forward,
   * left, up).range(Range.degrees(0, 135)).rest(PIVOT_IN).carries(link).positions(PIVOT_IN,
   * PIVOT_OUT).motors(...).control(...)}.
   */
  static RevoluteJoint pivot(String label, Part parent, Vector3 axis) {
    return RevoluteJoint.builder()
        .label(label)
        .parent(parent)
        .axis(axis)
        .link(Link.unseen(label))
        .build();
  }

  /**
   * A roller, a belt or a flywheel: a joint that turns freely and is run at its outputs. {@code
   * Joint.roller("intake", PIVOT).at(forward, left, up).outputs(INTAKING).motors(...)}.
   */
  static RevoluteJoint roller(String label, Part parent) {
    return pivot(label, parent, PITCH_DOWN);
  }

  /**
   * A roller that turns about some other axis: a spindexer about the vertical is {@code
   * Joint.roller("spindexer", Part.CHASSIS, Vector3.UP)}.
   */
  static RevoluteJoint roller(String label, Part parent, Vector3 axis) {
    return pivot(label, parent, axis);
  }

  /**
   * An elevator: a carriage sliding straight up. {@code Joint.elevator("elevator",
   * Part.CHASSIS).at(forward, left, up).range(Range.meters(0, 1.2)).rest(ELEVATOR_DOWN)
   * .motors(...)}, placed where the slide starts.
   */
  static PrismaticJoint elevator(String label, Part parent) {
    return slide(label, parent, Vector3.UP);
  }

  /**
   * A joint sliding along any axis: a hopper that extends forward is {@code Joint.slide("hopper",
   * Part.CHASSIS, new Vector3(1, 0, 0))}, placed where the slide starts.
   */
  static PrismaticJoint slide(String label, Part parent, Vector3 axis) {
    return PrismaticJoint.builder()
        .label(label)
        .parent(parent)
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
   * Where its frame is in its parent's frame when it is at position 0, as its block places it
   * ({@code .at(...)}): where the pivot is, or where the slide starts. None given, the robot says
   * so at startup, and takes it at its parent's origin ({@link RobotMath#origin}).
   */
  Optional<Transform3d> place();

  /** The axis it turns about or slides along, in its own frame. Any length but zero will do. */
  Vector3 axis();

  /**
   * Its hard stops. The range is not wrapped to one turn, so a turret may span more than a turn. A
   * joint that turns freely, a roller or a flywheel, has none.
   */
  Optional<Range> range();

  /**
   * Where it rests as the robot starts, if its block says ({@code rest(...)}). A practice robot may
   * move it. A joint whose block does not say starts at its zero ({@link RobotMath#restPosition}).
   */
  Optional<Double> rest();

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
   * How it is driven and wired: its motors or servos, its own sensors, their wiring, and its
   * control. This is not part of what makes a joint the same joint: two joints that differ only
   * here are equal.
   */
  Driven driven();

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
   * It at one of its positions, which is what a preset is made of: {@code Preset.of("amp",
   * ELEVATOR.at(ELEVATOR_CLEAR), ARM.at(ARM_SCORING))}. An interlock refuses this condition,
   * because it holds only with the joint exactly there and a measured position never is. An
   * interlock's edge is {@code atLeast}, {@code atMost} or {@code between}.
   *
   * <p>A position that is not this joint's own is still taken here. It is refused where the
   * condition is used ({@link Condition}).
   */
  default Condition.At at(Position position) {
    return ConditionAtBuilder.builder().joint(this).position(position).build();
  }

  /**
   * It at one of its outputs, or {@link Output#STOPPED}, which is what a preset is made of: {@code
   * Preset.of("feeding", FEEDER.at(FEEDER_FEEDING), INDEXER.at(INDEXER_FEEDING))}. An output that
   * is not this joint's own is refused where the condition is used ({@link Condition}).
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
    if (JointNames.whyNotOwn(this, position).isPresent()) {
      return position.value();
    }
    return JointNames.own(this, position).value();
  }

  /**
   * A joint's block: everything its declaration can say, and everything a practice robot can change
   * on the finished joint, written once for both kinds of joint. Each call returns a new joint, so
   * a block is a chain of them.
   *
   * <p>The two kinds differ only in what they are measured in, which the type parameters carry, so
   * the compiler refuses a length given to a joint that turns. The copies the block makes of a
   * joint are in {@link JointCopies}, so that nothing can skip the block's said-once checks.
   *
   * @param <J> the kind of joint the block declares, which every call returns
   * @param <P> what its positions are measured in: an angle or a distance
   * @param <S> what its speeds are measured in: an angular or a linear velocity
   */
  sealed interface Block<J extends Block<J, P, S>, P extends Measure<?>, S extends Measure<?>>
      extends Joint permits RevoluteJoint, PrismaticJoint {
    /**
     * Its hard stops: {@code .range(Range.degrees(0, 135))}. On a finished joint this moves its
     * stops and keeps where it rests: a practice robot's turret, {@code
     * TURRET.range(Range.degrees(-355, 5))}. The robot's definition checks everything else against
     * them when it is built, so a practice robot may move its stops and its places in either order.
     */
    default J range(Range range) {
      return JointCopies.withRange(this, range);
    }

    /**
     * Where it rests as the robot starts, stowed: {@code .rest(Degrees.of(-30))}. Without one it
     * rests at its zero. On a finished joint this moves where it rests and keeps its stops. The
     * robot's definition checks that it is inside them.
     */
    default J rest(P rest) {
      return JointCopies.withRest(this, inJointUnits(rest));
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
      return JointCopies.withRest(this, JointNames.whereToRestAt(this, rest));
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

    /**
     * Where it is on its parent, in its parent's frame, the camera's order: how far forward of the
     * parent's origin, how far left, how far up: {@code .at(Inches.of(-8), Inches.of(0),
     * Inches.of(18.33))}. For a pivot or a roller, its axle; for a slide, where it starts. Left
     * out, it's at its parent's origin, and the robot says so at startup.
     */
    default J at(Distance forward, Distance left, Distance up) {
      saidOnce(place().isPresent(), "has its place", "give it once, in one .at(...)");
      return JointCopies.withPlace(
          this, new Transform3d(new Translation3d(forward, left, up), Rotation3d.ZERO));
    }

    /** What it moves. */
    default J carries(Link link) {
      saidOnce(
          !link().equals(Link.unseen(label())),
          "carries " + link().label(),
          "it moves one link, so give it once, with everything it carries");
      return JointCopies.withLink(this, link);
    }

    /** Its places by name: {@code .positions(ARM_STOWED, ARM_SCORING)}. */
    default J positions(Position... positions) {
      saidOnce(
          !positions().isEmpty(),
          "has its positions, " + String.join(", ", JointNames.ofPositions(positions())),
          "name them all in one positions(...); to put one elsewhere, replaced(...)");
      return JointCopies.withPositions(this, List.of(positions), formerPositions());
    }

    /** What it runs at, by name: {@code .outputs(INTAKING, OUTTAKING)}. */
    default J outputs(Output... outputs) {
      saidOnce(
          !outputs().isEmpty(),
          "has its outputs, " + String.join(", ", JointNames.ofOutputs(outputs())),
          "name them all in one outputs(...); to run one otherwise, replaced(...)");
      return JointCopies.withOutputs(this, List.of(outputs), formerOutputs());
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
      return JointCopies.withPositions(
          this,
          JointNames.withReplaced(this, replacing),
          JointNames.formerWith(formerPositions(), JointNames.own(this, position), replacing));
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
      return revalued(output, Output.dutyCycle(output.name(), share));
    }

    private J revalued(Output output, Output replacing) {
      return JointCopies.withOutputs(
          this,
          JointNames.withReplaced(this, replacing),
          JointNames.formerWith(formerOutputs(), JointNames.own(this, output), replacing));
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
      return JointCopies.driven(this, drivenWith().motors(motors).build());
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
      return JointCopies.driven(this, drivenWith().motors(changed).build());
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
      List<Driven.ServoMount> servos = new ArrayList<>(driven().servos());
      servos.add(
          DrivenServoMountBuilder.builder()
              .label(label)
              .servo(servo)
              .onParent(onParent)
              .onLink(onLink)
              .build());
      return JointCopies.driven(this, drivenWith().servos(servos).build());
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
      return JointCopies.driven(this, drivenWith().absoluteEncoder(encoder).build());
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
      return JointCopies.driven(this, drivenWith().absoluteEncoder(change.apply(had)).build());
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
      return JointCopies.driven(this, drivenWith().limitSwitches(limits).build());
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
      return JointCopies.driven(this, drivenWith().limitSwitches(limits).build());
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
     * The stretch may be written larger than the joint's travel. The part inside the travel is what
     * counts.
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
      return JointCopies.driven(this, drivenWith().interlocks(interlocks).build());
    }

    @Override
    default J control(MechanismControl control) {
      saidOnce(
          driven().control().isPresent(),
          "has its control",
          "to re-tune it, retuned(control -> control.withKg(...))");
      return JointCopies.driven(this, drivenWith().control(control).build());
    }

    @Override
    default J retuned(UnaryOperator<MechanismControl> change) {
      MechanismControl had =
          toChange(driven().control(), "control to re-tune", "one, .control(...)");
      return JointCopies.driven(this, drivenWith().control(change.apply(had)).build());
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
}
