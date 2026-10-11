package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

/**
 * The definition's checks of each joint's block against itself and against the robot it is on:
 *
 * <ul>
 *   <li>where the joint rests, and its named positions, against its hard stops;
 *   <li>its control against the kind of joint it is, its range, its places and its sensors;
 *   <li>its absolute encoder against the motor controllers that would read it;
 *   <li>a sliding joint's absolute encoder against its drum and its stops.
 * </ul>
 *
 * <p>These are judged on the finished definition and never on a block that is half written. So the
 * order of a block's calls does not matter, and a sensor listed among the robot's sensors counts
 * for a joint just as one written in the joint's block does.
 *
 * <p>What the program cannot drive yet (a joint with motors and no control) is for the program to
 * say, not the definition, and is listed separately ({@link #uncontrolled}).
 */
final class Blocks {
  private Blocks() {}

  /** What is wrong with the joints' blocks, each problem reported against its joint. */
  static List<Finding> problems(RobotMath math) {
    List<Finding> problems = new ArrayList<>();
    for (Joint joint : math.joints()) {
      List<String> found = new ArrayList<>();
      found.addAll(whatIsPastItsStops(joint));
      joint
          .driven()
          .control()
          .ifPresent(control -> found.addAll(controlProblems(math, joint, control)));
      whyEncoderNeedsMotorsWired(joint).ifPresent(found::add);
      whyCancoderIsOffTheMotorsBus(joint).ifPresent(found::add);
      whySlidingEncoderCannotRead(joint).ifPresent(found::add);
      whyLoopCannotCloseOnEncoder(joint).ifPresent(found::add);
      whyThereIsNoDataPort(joint).ifPresent(found::add);
      whySparksCannotBeWiredSo(joint).ifPresent(found::add);
      for (String problem : found) {
        problems.add(Finding.refused(joint.label(), problem));
      }
    }
    return problems;
  }

  /** What is wrong with a joint's control, for the joint it is on. */
  private static List<String> controlProblems(
      RobotMath math, Joint joint, MechanismControl control) {
    List<String> found = new ArrayList<>();
    whyControlIsForTheOtherLoop(joint, control).ifPresent(found::add);
    whySoftLimitsPassTheStops(joint, control).ifPresent(found::add);
    found.addAll(placesOutsideSoftLimits(joint, control));
    whyHomingCannotWork(joint, control).ifPresent(found::add);
    if (math.motorsAlone(joint)) {
      whyItCannotFindWhereItIs(math, joint, control).ifPresent(found::add);
      whyItCannotHomeToItsSwitch(math, joint, control).ifPresent(found::add);
    }
    return found;
  }

  /**
   * The joints that have motors and no control, which the program cannot drive: a joint whose block
   * gives it motors, or one that a drive of the robot's turns by motors alone and that follows no
   * other.
   */
  static List<Finding> uncontrolled(RobotMath math) {
    List<Finding> uncontrolled = new ArrayList<>();
    for (Joint joint : math.joints()) {
      boolean hasNoControl = joint.driven().control().isEmpty();
      boolean hasMotors = joint.driven().motors().isPresent() || math.motorsAlone(joint);
      if (hasNoControl && hasMotors) {
        uncontrolled.add(
            Finding.refused(
                joint.label(),
                joint.label()
                    + " has motors and no control: give it .control(MechanismControl.position(kP)),"
                    + " or .velocity(kP) for one that only runs"));
      }
    }
    return uncontrolled;
  }

  /** A joint's motor controllers, the leader first. The list is empty when none are wired. */
  private static List<MechanismWiring.Device> controllersOf(Joint joint) {
    return joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
  }

  /**
   * Why a joint's control is written for the other loop than the joint runs, if it is. A joint that
   * is held at positions (it has a range or named positions) runs a position loop, and one that
   * only runs at outputs, such as a roller, runs a velocity loop. A kP is volts for each unit of
   * its own loop's error, so a control written for the other loop would run, and be wrong by orders
   * of magnitude without any sign of it.
   */
  private static Optional<String> whyControlIsForTheOtherLoop(
      Joint joint, MechanismControl control) {
    boolean jointHoldsPositions = joint.holdsPositions();
    boolean controlHoldsPositions = control.loop() == MechanismControl.Loop.POSITION;
    if (controlHoldsPositions == jointHoldsPositions) {
      return Optional.empty();
    }
    if (jointHoldsPositions) {
      return Optional.of(
          joint.label()
              + " is held at positions (it has a range or named positions), and its control is"
              + " MechanismControl.velocity(...), for a roller run at speeds: write"
              + " MechanismControl.position(kP), its kP in volts per "
              + whatAPositionLoopsKpIsPer(joint));
    }
    return Optional.of(
        joint.label()
            + " only runs at outputs (it has no range and no named positions), and its control is"
            + " MechanismControl.position(...), for a joint held at positions: write"
            + " MechanismControl.velocity(kP), its kP in volts per "
            + whatAVelocityLoopsKpIsPer(joint));
  }

  private static String whatAPositionLoopsKpIsPer(Joint joint) {
    return switch (Position.Kind.of(joint)) {
      case ANGLE -> "radian of error, or per turn, Volts.per(Rotation).of(...)";
      case LENGTH -> "meter of error, or per inch, Volts.per(Inch).of(...)";
    };
  }

  private static String whatAVelocityLoopsKpIsPer(Joint joint) {
    return switch (Position.Kind.of(joint)) {
      case ANGLE ->
          "radian per second of error, or per turn per second,"
              + " Volts.per(RotationsPerSecond).of(...)";
      case LENGTH ->
          "meter per second of error, or per inch per second,"
              + " Volts.per(InchesPerSecond).of(...)";
    };
  }

  /** Why the control's soft limits reach past the joint's hard stops, if they do. */
  private static Optional<String> whySoftLimitsPassTheStops(Joint joint, MechanismControl control) {
    if (control.softLimits().isEmpty() || joint.range().isEmpty()) {
      return Optional.empty();
    }
    MechanismControl.SoftLimits soft = control.softLimits().get();
    Range stops = joint.range().get();
    boolean insideTheStops = soft.lower() >= stops.lower() && soft.upper() <= stops.upper();
    if (insideTheStops) {
      return Optional.empty();
    }
    return Optional.of(
        String.format(
            Locale.ROOT,
            "%s's soft limits, %s to %s, must be inside its hard stops, %s to %s",
            joint.label(),
            joint.shown(soft.lower()),
            joint.shown(soft.upper()),
            joint.shown(stops.lower()),
            joint.shown(stops.upper())));
  }

  /**
   * Where a joint rests, and each of its named positions, that is outside its hard stops, where the
   * joint cannot be. It is judged on the finished joint, so a practice robot that moves both its
   * stops and its places, each past where the other was, can change them in either order.
   */
  private static List<String> whatIsPastItsStops(Joint joint) {
    if (joint.range().isEmpty()) {
      return List.of();
    }
    Range stops = joint.range().get();
    String shownStops = joint.shown(stops.lower()) + " to " + joint.shown(stops.upper());
    List<String> past = new ArrayList<>();
    if (!stops.contains(joint.restPosition())) {
      past.add(restIsPastItsStops(joint, shownStops));
    }
    for (Position position : joint.positions()) {
      if (!stops.contains(position.value())) {
        past.add(
            possessive(joint.label())
                + " "
                + position.name()
                + ", "
                + joint.shown(position.value())
                + ", is outside its hard stops, "
                + shownStops
                + ": it can't get there");
      }
    }
    return past;
  }

  /**
   * The refusal for a joint that rests outside its stops. A joint whose block gives no rest starts
   * at its zero, and the refusal then says so, because the team never wrote the position it is
   * being refused for.
   */
  private static String restIsPastItsStops(Joint joint, String shownStops) {
    if (joint.rest().isPresent()) {
      return joint.label()
          + " rests at "
          + joint.shown(joint.restPosition())
          + ", outside its hard stops, "
          + shownStops
          + ": it can't start there";
    }
    return joint.label()
        + " has no rest, so it starts at its zero, "
        + joint.shown(0)
        + ", outside its hard stops, "
        + shownStops
        + ": it can't start there; give where it rests, .rest(...)";
  }

  /**
   * A refusal for each of the joint's named positions that is inside its hard stops but outside its
   * control's soft limits, where the joint would never be driven. A position outside the hard stops
   * is refused for that instead ({@link #whatIsPastItsStops}).
   */
  private static List<String> placesOutsideSoftLimits(Joint joint, MechanismControl control) {
    if (control.softLimits().isEmpty()) {
      return List.of();
    }
    MechanismControl.SoftLimits soft = control.softLimits().get();
    List<String> outside = new ArrayList<>();
    for (Position position : joint.positions()) {
      boolean outsideSoftLimits =
          position.value() < soft.lower() || position.value() > soft.upper();
      boolean insideHardStops =
          joint.range().map(stops -> stops.contains(position.value())).orElse(true);
      if (outsideSoftLimits && insideHardStops) {
        outside.add(
            String.format(
                Locale.ROOT,
                "%s's %s, %s, is outside its soft limits, %s to %s: it's never driven there",
                joint.label(),
                position.name(),
                joint.shown(position.value()),
                joint.shown(soft.lower()),
                joint.shown(soft.upper())));
      }
    }
    return outside;
  }

  /**
   * Why a joint's homing recipe cannot work, if it has one that cannot. A joint that only runs at
   * outputs has no position to find, and a joint can only be pushed into a hard stop it has.
   */
  private static Optional<String> whyHomingCannotWork(Joint joint, MechanismControl control) {
    if (control.homing().isEmpty()) {
      return Optional.empty();
    }
    if (!joint.holdsPositions()) {
      return Optional.of(joint.label() + " only runs at outputs: it has no position to home");
    }
    boolean homesToAHardStop = control.homing().get() instanceof MechanismControl.Homing.ToHardStop;
    if (homesToAHardStop && joint.range().isEmpty()) {
      return Optional.of(joint.label() + " turns freely: it has no hard stop to home to");
    }
    return Optional.empty();
  }

  /**
   * Why a joint cannot find where it is at power on, if it cannot. A joint held at positions either
   * names a homing recipe in its control or reads an absolute encoder, and on a robot the encoder
   * has to be wired to something that reads it: a CANcoder, its SPARK's data port, or a Systemcore
   * port.
   */
  private static Optional<String> whyItCannotFindWhereItIs(
      RobotMath math, Joint joint, MechanismControl control) {
    if (!joint.holdsPositions() || control.homing().isPresent()) {
      return Optional.empty();
    }
    if (math.absoluteEncoder(joint) == null) {
      return Optional.of(
          joint.label()
              + " has no absolute encoder, so where it is at power on is a guess: give its control"
              + " a homing recipe, Homing.toHardStop, Homing.toSwitch or Homing.assumeAtRest()");
    }
    List<MechanismWiring.Device> controllers = controllersOf(joint);
    boolean encoderIsWired =
        joint.driven().absoluteEncoder().map(Driven.AbsoluteEncoder::wired).orElse(false);
    // With no controllers wired the joint runs in the simulator alone, which reads the encoder
    // without any wiring.
    if (controllers.isEmpty() || encoderIsWired) {
      return Optional.empty();
    }
    return Optional.of(
        joint.label()
            + " homes by its absolute encoder, and its wiring names no CANcoder, data port or"
            + " Systemcore port to read it on: wire it in its block, "
            + howItsControllerReadsAnEncoder(controllers.get(0).controller())
            + ", or give its control a homing recipe");
  }

  /**
   * How a joint's block wires an absolute encoder that its leading controller can read. A SPARK
   * reads one on its data port and reads no CANcoder. A TalonFX or a TalonFXS reads a CANcoder on
   * its bus.
   */
  private static String howItsControllerReadsAnEncoder(MechanismWiring.Controller controller) {
    String dataPort = "'s data port, .absoluteEncoder(DATA_PORT.offset(...))";
    String cancoder = "'s bus, .absoluteEncoder(CANCODER.can(...).offset(...))";
    return switch (controller) {
      case SPARK_MAX -> "on its SPARK MAX" + dataPort;
      case SPARK_FLEX -> "on its SPARK Flex" + dataPort;
      case TALON_FX -> "as a CANcoder on its TalonFX" + cancoder;
      case TALON_FXS -> "as a CANcoder on its TalonFXS" + cancoder;
    };
  }

  /**
   * Why a joint whose control homes it to a limit switch cannot do so, if it cannot: it has no
   * switch at that end, or nobody has said where that switch trips.
   */
  private static Optional<String> whyItCannotHomeToItsSwitch(
      RobotMath math, Joint joint, MechanismControl control) {
    Optional<MechanismControl.Homing> homing = control.homing();
    if (homing.isEmpty() || !(homing.get() instanceof MechanismControl.Homing.ToSwitch toSwitch)) {
      return Optional.empty();
    }
    String end =
        switch (toSwitch.end()) {
          case LOWER -> "lower";
          case UPPER -> "upper";
        };
    String whereItTrips =
        switch (Position.Kind.of(joint)) {
          case ANGLE -> "Degrees.of(...)";
          case LENGTH -> "Inches.of(...)";
        };
    Sensor.Switch onSwitch = math.limitSwitch(joint, toSwitch.end());
    if (onSwitch == null) {
      return Optional.of(
          joint.label()
              + " homes to its "
              + end
              + " limit switch, and its block has none: give it one where it trips,"
              + " .limitSwitch(LimitSwitch."
              + end
              + "().at("
              + whereItTrips
              + ")...)");
    }
    boolean positionIsKnown = ((Sensor.Switch.Limit) onSwitch.kind()).position().isPresent();
    if (positionIsKnown) {
      return Optional.empty();
    }
    return Optional.of(
        joint.label()
            + " homes to "
            + onSwitch.label()
            + ", whose position isn't known: measure where it trips and give it, LimitSwitch."
            + end
            + "().at("
            + whereItTrips
            + "), or home another way");
  }

  /**
   * Why a sliding joint's absolute encoder cannot say where the joint is, if it has one that
   * cannot. The encoder turns with the joint's drum, so the block's motors have to give the drum's
   * pitch radius, and one of the encoder's turns must cover the joint's travel, which only hard
   * stops bound.
   */
  private static Optional<String> whySlidingEncoderCannotRead(Joint joint) {
    if (!(joint instanceof Joint.Prismatic) || joint.driven().absoluteEncoder().isEmpty()) {
      return Optional.empty();
    }
    if (joint.driven().motors().flatMap(Driven.Motors::pitchRadius).isEmpty()) {
      return Optional.of(
          possessive(joint.label())
              + " absolute encoder turns with its drum, and its block has no motors to say the"
              + " drum's pitch radius: give them,"
              + " .motors(motor.through(gearing).drum(pitchRadius))");
    }
    if (joint.range().isEmpty()) {
      return Optional.of(
          joint.label()
              + " has an absolute encoder and no hard stops, so nothing says one of the encoder's"
              + " turns covers its travel: give its range, range(Range.meters(...))");
    }
    return Optional.empty();
  }

  /**
   * Why a joint's control loop cannot run on its absolute encoder, when its block says it does
   * ({@code .closesLoop()}), if it cannot. The joint must be held at positions. It must learn where
   * it is from that encoder and not from a homing recipe. And it must have a range, which the
   * definition checks one turn of the encoder holds: a joint that turns freely has a reading that
   * wraps inside its travel. A sliding joint with no range is refused already ({@link
   * #whySlidingEncoderCannotRead}).
   */
  private static Optional<String> whyLoopCannotCloseOnEncoder(Joint joint) {
    boolean closesLoopOnEncoder =
        joint.driven().absoluteEncoder().map(Driven.AbsoluteEncoder::closedLoop).orElse(false);
    if (!closesLoopOnEncoder) {
      return Optional.empty();
    }
    if (!joint.holdsPositions()) {
      return Optional.of(
          joint.label()
              + " only runs at outputs, so its loop has no position to close on its absolute"
              + " encoder: drop .closesLoop() from its encoder's wiring");
    }
    if (joint.driven().control().flatMap(MechanismControl::homing).isPresent()) {
      return Optional.of(
          possessive(joint.label())
              + " loop closes on its absolute encoder, which tells it where it is: drop its"
              + " control's homing recipe");
    }
    if (joint.range().isEmpty() && joint instanceof Joint.Revolute) {
      return Optional.of(
          joint.label()
              + " turns freely, and its absolute encoder's reading wraps each turn: its loop can't"
              + " close on it yet, so let the encoder tell its motor where it is instead");
    }
    return Optional.empty();
  }

  /**
   * Why a joint's absolute encoder cannot be on its controller's data port, if its block puts it
   * there: a TalonFX has no data port.
   */
  private static Optional<String> whyThereIsNoDataPort(Joint joint) {
    List<MechanismWiring.Device> controllers = controllersOf(joint);
    boolean encoderIsOnDataPort =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::dataPort).isPresent();
    if (!encoderIsOnDataPort || controllers.isEmpty()) {
      return Optional.empty();
    }
    if (controllers.get(0).controller() != MechanismWiring.Controller.TALON_FX) {
      return Optional.empty();
    }
    return Optional.of(
        possessive(joint.label())
            + " wiring names an encoder on its controller's data port, and a TalonFX has none:"
            + " wire it as a CANcoder");
  }

  /**
   * Why a joint's SPARKs cannot be wired as its block says, if they cannot. A SPARK is always on
   * Systemcore's own CAN bus, never on a CANivore, and it reads no CANcoder.
   */
  private static Optional<String> whySparksCannotBeWiredSo(Joint joint) {
    List<MechanismWiring.Device> controllers = controllersOf(joint);
    if (controllers.isEmpty()) {
      return Optional.empty();
    }
    MechanismWiring.Device leader = controllers.get(0);
    boolean isASpark =
        leader.controller() == MechanismWiring.Controller.SPARK_MAX
            || leader.controller() == MechanismWiring.Controller.SPARK_FLEX;
    if (!isASpark) {
      return Optional.empty();
    }
    if (!(leader.bus() instanceof CanBus.Port)) {
      return Optional.of(
          possessive(joint.label())
              + " SPARKs are on a CANivore: a SPARK is on Systemcore's own CAN");
    }
    if (joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder).isPresent()) {
      return Optional.of(
          possessive(joint.label())
              + " wiring names a CANcoder, which a SPARK doesn't read: plug the encoder into the"
              + " SPARK's data port, or give its control a homing recipe");
    }
    return Optional.empty();
  }

  /**
   * Why a joint's absolute encoder cannot be read, if it is wired to be read through the joint's
   * motor controller and the motors themselves are not wired.
   */
  private static Optional<String> whyEncoderNeedsMotorsWired(Joint joint) {
    boolean readThroughController =
        joint
            .driven()
            .absoluteEncoder()
            .map(Driven.AbsoluteEncoder::readByController)
            .orElse(false);
    boolean motorsAreWired = !controllersOf(joint).isEmpty();
    if (!readThroughController || motorsAreWired) {
      return Optional.empty();
    }
    return Optional.of(
        possessive(joint.label())
            + " absolute encoder is wired, and its motors aren't: it's read through their"
            + " controller, so wire them too; a joint your own code drives reads one on a"
            + " Systemcore port instead, MechanismWiring.PortEncoder");
  }

  /**
   * Why a joint's CANcoder is not where its TalonFX can read it, if it is not: the TalonFX reads a
   * CANcoder over its own bus, so the two have to be on the same one.
   */
  private static Optional<String> whyCancoderIsOffTheMotorsBus(Joint joint) {
    Optional<MechanismWiring.Cancoder> cancoder =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder);
    List<MechanismWiring.Device> controllers = controllersOf(joint);
    if (cancoder.isEmpty() || controllers.isEmpty()) {
      return Optional.empty();
    }
    CanBus motorsBus = controllers.get(0).bus();
    if (cancoder.get().bus().equals(motorsBus)) {
      return Optional.empty();
    }
    return Optional.of(
        possessive(joint.label())
            + " CANcoder is on "
            + cancoder.get().bus().shown()
            + ", and its motors on "
            + motorsBus.shown()
            + ": their TalonFX reads it over its own bus, so put the CANcoder there");
  }
}
