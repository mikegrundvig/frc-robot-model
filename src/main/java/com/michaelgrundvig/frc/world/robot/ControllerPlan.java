package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * How the program drives one joint, worked out once from what the joint is and how its block says
 * to drive it, in the joint's own units, radians or meters: its gains, each tuned where its control
 * gives it and seeded from the definition where not ({@link RobotMath#idealGains}); gravity's kind,
 * its kG and where it pulls hardest; its soft limits, current limits and neutral mode; its motors'
 * controllers, and which way each turns; and the one feedforward law ({@link #feedforward}) that
 * every loop driving the joint evaluates: a TalonFX's slot, set from these gains, a SPARK's
 * feedforward, which the robot works out each loop, a loop the robot closes itself, and the
 * simulator's motor controllers. A joint whose block wires no controllers is planned on the one its
 * motor runs on ({@link MechanismWiring.Controller#standIn}), which the simulator runs it as.
 *
 * <p>Vendor-neutral and pure: a motor controller's settings are worked out from it in that vendor's
 * units and terms, and what only one vendor can't take (a ratio past Phoenix's, a kI on a SPARK) is
 * that vendor's to refuse. What no motor controller could do is refused here, by an {@link
 * IllegalArgumentException} saying what and why: motors that don't move the joint, a slide with no
 * drum to count its rotations in ({@link MechanismRotations}), a tuned kG on a joint gravity can't
 * move. What's doubtful is noted beside it ({@link #notes}): a tuned gain far from what the
 * definition derives, a gravity one cosine fits badly. A joint with no kG written is the
 * definition's doubt ({@code RobotDefinition.findings}), not noted here.
 */
public final class ControllerPlan {
  /**
   * How far one cosine may miss gravity's torque, as a share of its peak, before it's noted: beyond
   * a few percent, the joint's feedforward wants {@link RobotMath#gravityTorque} instead.
   */
  public static final double MISFIT = 0.05;

  /**
   * How far a tuned gain may be from the definition's ideal one and still be noted as the same
   * mechanism: from half to twice. Friction and losses put real gains above the ideal, rarely past
   * twice it; past that, a unit or a ratio is likelier than the mechanism.
   */
  static final double SANITY_LOW = 0.5;

  static final double SANITY_HIGH = 2.0;

  /** How close a tuned gain's ratio to the ideal must come to a frame's to be named as one. */
  static final double FRAME_TOLERANCE = 0.1;

  /**
   * Gravity's share of the feedforward: none for a joint gravity can't move (a turret, a slide that
   * runs level), kG steady all along a slide, kG along one cosine for an arm, as Phoenix's {@code
   * Arm_Cosine} and WPILib's {@code ArmFeedforward} take it.
   *
   * @param kind which of the three
   * @param kG the volts that hold the joint: everywhere along a slide, where its weight pulls
   *     hardest on an arm; 0 for none
   * @param offset for an arm, what's added to its position for its angle from where its weight
   *     pulls hardest, in radians: Phoenix's {@code GravityArmPositionOffset}, in rotations; 0
   *     otherwise
   */
  public record Gravity(Kind kind, double kG, double offset) {
    /** Which way gravity's volts follow the joint. */
    public enum Kind {
      /** Gravity can't move the joint. */
      NONE,
      /** The same all along: a slide's. */
      STEADY,
      /** Along one cosine of the joint's angle: an arm's. */
      COSINE
    }

    /** The volts that hold the joint against gravity at a position. */
    public double volts(double position) {
      return switch (kind) {
        case NONE -> 0;
        case STEADY -> kG;
        case COSINE -> kG * Math.cos(position + offset);
      };
    }
  }

  private final Joint joint;
  private final Drive drive;
  private final MechanismControl control;
  private final List<MechanismWiring.Device> controllers;
  private final int motors;
  private final MechanismRotations rotations;
  private final double rotorRatio;
  private final double kV;
  private final double kA;
  private final Gravity gravity;
  private final List<String> notes;

  private ControllerPlan(
      RobotMath math,
      Joint joint,
      Drive drive,
      List<MechanismWiring.Device> controllers,
      MechanismControl control) {
    List<String> noted = new ArrayList<>();
    if (!(drive.actuator() instanceof Actuator.Motors driving)) {
      throw new IllegalArgumentException(
          drive.label() + " isn't motors: a motor controller drives motors");
    }
    boolean aControllerForEachMotor =
        controllers.isEmpty() || controllers.size() == driving.count();
    if (!aControllerForEachMotor) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s is %d motors, and its wiring names %d",
              drive.label(),
              driving.count(),
              controllers.size()));
    }
    Sensor.Encoder rotor = drive.rotor();
    rotorRatio = Math.abs(rotor.ratio(joint));
    if (rotorRatio == 0) {
      throw new IllegalArgumentException(drive.label() + " doesn't move " + joint.label());
    }
    noteOtherJointsTheRotorTurnsWith(joint, drive, rotor, noted);
    this.joint = joint;
    this.drive = drive;
    this.control = control;
    if (controllers.isEmpty()) {
      this.controllers = standIns(driving);
    } else {
      this.controllers = List.copyOf(controllers);
    }
    motors = driving.count();
    rotations = MechanismRotations.of(joint, drive);
    RobotMath.Gains seed = math.idealGains(joint);
    MechanismControl.Gains gains = control.gains();
    kV = gains.kV().orElse(seed.kV());
    if (gains.kV().isPresent()) {
      TunedGainDoubt.ofKv(joint, kV, seed.kV(), rotations, rotorRatio).ifPresent(noted::add);
    }
    kA = gains.kA().orElse(seed.kA());
    gravity = gravity(math, gains, seed.kG(), noted);
    notes = List.copyOf(noted);
  }

  /**
   * Notes each other joint that a drive's rotor turns with. A motor controller counts its rotor's
   * turns and cannot tell which joint they came from, so when another joint moves it reads this
   * joint as moving. A swerve module's drive motor is the usual case: it turns as the module
   * steers.
   */
  private static void noteOtherJointsTheRotorTurnsWith(
      Joint joint, Drive drive, Sensor.Encoder rotor, List<String> noted) {
    for (Transmission.Term term : rotor.terms()) {
      Joint other = term.joint();
      if (other.equals(joint)) {
        continue;
      }
      String perWhat =
          switch (Position.Kind.of(other)) {
            case ANGLE -> "turn";
            case LENGTH -> "meter";
          };
      noted.add(
          String.format(
              Locale.ROOT,
              "%s's rotor turns %s times for each %s of %s too, which the controller reads as %s"
                  + " moving: take it out in code, as a swerve module's coupling is",
              drive.label(),
              shown(Math.abs(rotor.ratio(other))),
              perWhat,
              other.label(),
              joint.label()));
    }
  }

  /**
   * The plan for a joint as its block says: its control, its motors' controllers.
   *
   * @param joint one the template drives, with a control
   * @throws IllegalArgumentException for a joint with no control, and what no motor controller
   *     could do
   */
  public static ControllerPlan of(RobotMath math, Joint joint) {
    return of(
        math,
        joint,
        joint
            .driven()
            .control()
            .orElseThrow(() -> new IllegalArgumentException(joint.label() + " has no control")));
  }

  /**
   * The plan for a joint driven by another control than its block's: as the pit tunes it. Its drive
   * is the definition's ({@link RobotMath#drivesOf}), its controllers its block's, or its motor's
   * stand-in where they aren't wired ({@link #controllers}).
   *
   * @throws IllegalArgumentException for what no motor controller could do
   */
  public static ControllerPlan of(RobotMath math, Joint joint, MechanismControl control) {
    return of(
        math,
        joint,
        math.drivesOf(joint).get(0),
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of()),
        control);
  }

  /**
   * The plan for a joint from its parts given apart: what a vendor's helper is given, by its tests.
   *
   * @param drive the motors that move it
   * @param controllers each motor's controller, the leader first, as many as its motors; or none,
   *     for its motor's stand-in ({@link #controllers})
   * @throws IllegalArgumentException for what no motor controller could do
   */
  public static ControllerPlan of(
      RobotMath math,
      Joint joint,
      Drive drive,
      List<MechanismWiring.Device> controllers,
      MechanismControl control) {
    return new ControllerPlan(math, joint, drive, controllers, control);
  }

  /** The joint. */
  public Joint joint() {
    return joint;
  }

  /** The motors that move it. */
  public Drive drive() {
    return drive;
  }

  /** How it's driven, as given: its block's, or as the pit tunes it. */
  public MechanismControl control() {
    return control;
  }

  /**
   * Each motor's controller, the leader first: its block's, or, where its block wires none, the
   * controller its motor runs on ({@link MechanismWiring.Controller#standIn}), on Systemcore's CAN
   * at IDs from 0, each turning counterclockwise for positive. A stand-in's ID only tells its
   * motors apart: nothing refused or doubted of the joint reads its ID, its bus or its direction.
   */
  public List<MechanismWiring.Device> controllers() {
    return controllers;
  }

  /** The stand-in controller for each of a joint's motors, its block wiring none. */
  private static List<MechanismWiring.Device> standIns(Actuator.Motors motors) {
    MechanismWiring.Controller controller = MechanismWiring.Controller.standIn(motors.motor());
    List<MechanismWiring.Device> standIns = new ArrayList<>();
    for (int id = 0; id < motors.count(); id++) {
      standIns.add(controller.can(id));
    }
    return List.copyOf(standIns);
  }

  /**
   * Whether a following motor's controller turns the other way from the leader's as the joint moves
   * forward: one opposed to it, rather than aligned.
   */
  public boolean opposes(MechanismWiring.Device follower) {
    return controllers.get(0).turnsClockwise() != follower.turnsClockwise();
  }

  /** How many motors move it together. */
  public int motors() {
    return motors;
  }

  /** Its rotations, as its motor controllers count them. */
  public MechanismRotations rotations() {
    return rotations;
  }

  /**
   * Its motors' rotor turns for each turn (or meter) of the joint, as the definition writes it: a
   * magnitude.
   */
  public double rotorRatio() {
    return rotorRatio;
  }

  /** Volts per radian (or meter) of error. */
  public double kP() {
    return control.gains().kP();
  }

  /** Volts per radian (or meter) of error, per second it lasts. */
  public double kI() {
    return control.gains().kI();
  }

  /** Volts per radian (or meter) per second of error. */
  public double kD() {
    return control.gains().kD();
  }

  /** Volts to overcome friction, in the direction of travel: measured, never seeded. */
  public double kS() {
    return control.gains().kS();
  }

  /** Volts per radian (or meter) per second: tuned, else the definition's seed. */
  public double kV() {
    return kV;
  }

  /** Volts per radian (or meter) per second squared: tuned, else the definition's seed. */
  public double kA() {
    return kA;
  }

  /** Gravity's share of its feedforward: tuned, else the definition's seed from its mass. */
  public Gravity gravity() {
    return gravity;
  }

  /** Where its motor controllers stop driving it, if its control says. */
  public Optional<MechanismControl.SoftLimits> softLimits() {
    return control.softLimits();
  }

  /** Its motors' current limits. */
  public MechanismControl.CurrentLimits currentLimits() {
    return control.currentLimits();
  }

  /** Whether its motors brake when told nothing, rather than coast. */
  public boolean brakes() {
    return control.neutralMode() == MechanismControl.NeutralMode.BRAKE;
  }

  /** Whether it turns freely, so a position is reached the short way round, a turn being a turn. */
  public boolean turnsFreely() {
    return joint instanceof Joint.Revolute && joint.range().isEmpty();
  }

  /** What's doubtful about it, each a sentence for a log or a test. */
  public List<String> notes() {
    return notes;
  }

  /**
   * The feedforward: the volts that move the joint as asked, before any feedback. Friction in the
   * direction of travel, kV at the speed, kA at the acceleration, and gravity at the position.
   *
   * @param position where it is, or where it's asked to be, in its units
   * @param velocity how fast it's asked to move, in its units a second
   * @param acceleration how fast it's asked to speed up, in its units a second squared
   */
  public double feedforward(double position, double velocity, double acceleration) {
    return kS() * Math.signum(velocity)
        + kV * velocity
        + kA * acceleration
        + gravity.volts(position);
  }

  /**
   * Where an absolute encoder its motor controller reads wraps, in the encoder's turns, as the
   * controller reads it: told its direction, so it grows as the joint moves forward, and its
   * offset, so it reads zero with the joint at zero. The definition's discontinuity point ({@link
   * RobotMath#discontinuityPoint}), which reads it as its ratio's sign says, turned round for an
   * encoder geared to turn back: Phoenix's {@code AbsoluteSensorDiscontinuityPoint}, and where a
   * SPARK's reading of its data port wraps.
   *
   * @throws IllegalArgumentException as the definition's does
   */
  public static double wrapPoint(RobotMath math, Sensor.Encoder encoder) {
    return turnedRoundIfGearedBack(encoder, math.discontinuityPoint(encoder));
  }

  /**
   * A point in an encoder's turn, changed between the two ways of reading the encoder. The
   * definition reads an encoder as the sign of its ratio says. A motor controller reads it as its
   * wiring sets it, so that it grows as the joint moves forward. For an encoder geared to turn
   * backward the two run opposite ways, and a point at one end of the turn for one is at the other
   * end for the other. The same change turns it back.
   */
  private static double turnedRoundIfGearedBack(Sensor.Encoder encoder, double point) {
    boolean gearedBack = encoder.ratio(encoder.joint()) < 0;
    if (gearedBack) {
      return 1 - point;
    }
    return point;
  }

  /**
   * How far inside an absolute encoder's turn its joint's range sits when the encoder wraps at a
   * point, in the encoder's turns, at the nearer end: negative when the range crosses the wrap. The
   * encoder reads as its motor controller reads it, growing with the joint ({@link #wrapPoint}).
   * Infinite for a joint that turns freely, which has no range to keep clear.
   */
  public static double wrapMargin(Sensor.Encoder encoder, double wrapPoint) {
    return RobotMath.wrapMargin(encoder, turnedRoundIfGearedBack(encoder, wrapPoint));
  }

  /**
   * What's wrong with an absolute encoder wrapping at a point, or null when nothing is: its joint's
   * range crossing the wrap, or within the definition's margin of it ({@link
   * RobotMath#WRAP_MARGIN}, a fortieth of a turn). A team's own point is checked as the template's
   * is.
   */
  public static @Nullable String wrapProblem(Sensor.Encoder encoder, double wrapPoint) {
    double margin = wrapMargin(encoder, wrapPoint);
    if (margin >= RobotMath.WRAP_MARGIN) {
      return null;
    }
    Joint joint = encoder.joint();
    Range range = joint.range().orElseThrow();
    double readingAtStow = reading(encoder, joint.restPosition());
    double bottomOfTurn = wrapPoint - 1;
    double stowFromWrap = Math.min(readingAtStow - bottomOfTurn, wrapPoint - readingAtStow);
    boolean rangeCrossesTheWrap = margin < 0;
    String whereTheWrapIs = joint.shown(alongJoint(encoder, margin)) + " from";
    if (rangeCrossesTheWrap) {
      whereTheWrapIs = "inside";
    }
    return String.format(
        Locale.ROOT,
        "%s wraps at %s of its turn, %s %s's range (%s to %s), and %s from where it stows (%s): a"
            + " reading a hair past the wrap reads a turn away",
        encoder.label(),
        shown(wrapPoint),
        whereTheWrapIs,
        joint.label(),
        joint.shown(range.lower()),
        joint.shown(range.upper()),
        joint.shown(alongJoint(encoder, stowFromWrap)),
        joint.shown(joint.restPosition()));
  }

  /** The encoder's turns for each of its joint's turns (or meters): a magnitude. */
  private static double sensorRatio(Sensor.Encoder encoder) {
    return Math.abs(encoder.ratio(encoder.joint()));
  }

  /** How far its joint moves, in the joint's units, as an encoder turns this much. */
  private static double alongJoint(Sensor.Encoder encoder, double turns) {
    return turns / sensorRatio(encoder) * perRatioUnit(encoder.joint());
  }

  /** What an encoder reads, in its turns, with its joint at a position. */
  private static double reading(Sensor.Encoder encoder, double position) {
    return sensorRatio(encoder) * position / perRatioUnit(encoder.joint());
  }

  /**
   * The joint's units in each of the units the definition's ratios are per: 2π radians in a turn of
   * a revolute joint, a meter in a meter of a prismatic one.
   */
  private static double perRatioUnit(Joint joint) {
    if (joint instanceof Joint.Revolute) {
      return 2 * Math.PI;
    }
    return 1;
  }

  /**
   * Gravity's share of the joint's feedforward, with what is doubtful about it added to the notes.
   *
   * <p>The kG is the tuned one where the control gives it, and otherwise the definition's seed from
   * the links' mass, which is 0 for links with no mass. A joint that gravity cannot move has none,
   * and a tuned kG on such a joint is refused.
   *
   * @param seed the definition's ideal kG
   */
  private Gravity gravity(
      RobotMath math, MechanismControl.Gains gains, double seed, List<String> noted) {
    Optional<Double> tuned = gains.kG();
    if (!math.gravityActsOn(joint)) {
      refuseATunedKg(tuned);
      return new Gravity(Gravity.Kind.NONE, 0, 0);
    }
    double kG = tuned.orElse(seed);
    Gravity found;
    if (joint instanceof Joint.Revolute) {
      found = new Gravity(Gravity.Kind.COSINE, kG, offsetOfGravitysCosine(math, gains));
      whyOneCosineMissesGravity(math).ifPresent(noted::add);
    } else {
      found = new Gravity(Gravity.Kind.STEADY, kG, 0);
    }
    if (tuned.isPresent()) {
      TunedGainDoubt.ofKg(joint, kG, seed).ifPresent(noted::add);
    }
    if (kG != 0) {
      whyAJointBelowTiltsGravity(math).ifPresent(noted::add);
    }
    return found;
  }

  /**
   * Refuses a kG that was tuned to anything but 0 for a joint gravity cannot move. Such a kG would
   * push a turret or a level slide one way for no reason.
   */
  private void refuseATunedKg(Optional<Double> tuned) {
    if (tuned.isEmpty() || tuned.get() == 0) {
      return;
    }
    String whyGravityCannotMoveIt =
        switch (Position.Kind.of(joint)) {
          case ANGLE -> "turns about the vertical";
          case LENGTH -> "slides level";
        };
    throw new IllegalArgumentException(
        String.format(
            Locale.ROOT,
            "%s has a tuned kG of %s V, but gravity can't move it: it %s. Leave its kG out, or"
                + " write 0",
            joint.label(),
            shown(tuned.get()),
            whyGravityCannotMoveIt));
  }

  /**
   * What is added to an arm's position to give its angle from where its weight pulls hardest, in
   * radians. The control may say where that is ({@code gravityAngle}), and the offset is then minus
   * that angle, brought to within half a turn of zero. Otherwise it is where the links' mass puts
   * it, or the joint's zero for links with no mass ({@link RobotMath#gravityOffset}).
   */
  private double offsetOfGravitysCosine(RobotMath math, MechanismControl.Gains gains) {
    if (gains.gravityAngle().isPresent()) {
      double pullsHardestAt = gains.gravityAngle().get().in(Radians);
      return -Math.IEEEremainder(pullsHardestAt, 2 * Math.PI);
    }
    return math.gravityOffset(joint);
  }

  /** The note for an arm whose gravity one cosine fits badly, if it does ({@link #MISFIT}). */
  private Optional<String> whyOneCosineMissesGravity(RobotMath math) {
    RobotMath.GravityFit fit = math.gravityFit(joint);
    if (!(fit.misfit() > MISFIT)) {
      return Optional.empty();
    }
    return Optional.of(
        String.format(
            Locale.ROOT,
            "one cosine misses %s's gravity by %.0f%% of its peak, at %s: a controller's arm"
                + " cosine can't follow it, so feed RobotMath.gravityTorque at the joint's"
                + " angle as the request's feedforward",
            joint.label(),
            fit.misfit() * 100,
            joint.shown(fit.worstAt())));
  }

  /**
   * The note for a joint that rides on one whose turning tilts it, if it rides on one. A motor
   * controller's gravity term knows only this joint's own position, so it holds the joint only
   * while the joint below is where it rests.
   */
  private Optional<String> whyAJointBelowTiltsGravity(RobotMath math) {
    Optional<Joint> tilting = firstJointBelowThatTiltsIt(math);
    if (tilting.isEmpty()) {
      return Optional.empty();
    }
    String below = tilting.get().label();
    return Optional.of(
        String.format(
            Locale.ROOT,
            "%s rides %s, whose turning tilts its gravity: a controller's gravity term holds it"
                + " only with %s at rest, so feed RobotMath.gravityTorque at the joints'"
                + " positions as the request's feedforward",
            joint.label(),
            below,
            below));
  }

  /**
   * The nearest joint between this one and the chassis whose turning tilts gravity at this one, if
   * there is one: a joint that turns about an axis that is not vertical, with the joints where they
   * rest.
   */
  private Optional<Joint> firstJointBelowThatTiltsIt(RobotMath math) {
    Part below = joint.parent();
    while (below instanceof Joint parent) {
      if (parent instanceof Joint.Revolute && math.gravityActsOn(parent)) {
        return Optional.of(parent);
      }
      below = parent.parent();
    }
    return Optional.empty();
  }
}
