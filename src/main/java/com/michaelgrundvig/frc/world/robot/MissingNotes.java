package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.units.measure.Mass;

/**
 * What a robot's definition does not know: the values nobody has measured or written yet. None of
 * them is wrong. Each is a note, said once as the program starts, that names what is missing, what
 * stands in for it, and what is less accurate because of it. {@link RobotDefinition#findings} lists
 * what the notes cover.
 *
 * <p>A note is about the robot as a whole, a joint or a sensor, and says what reads the missing
 * value: the robot itself, or only the simulator. Mass is optional and only the simulator reads
 * most of it, so most notes about mass are the simulator's.
 *
 * <p>An instance exists for the length of one listing.
 */
final class MissingNotes {
  /** Below this, in newton meters or newtons, a joint's links are taken to give it no gravity. */
  private static final double NO_GRAVITY = 1e-12;

  /** Below this, in seconds, a time is told in milliseconds. */
  private static final double TENTH_OF_A_SECOND = 0.1;

  private final RobotDefinition robot;
  private final RobotMath math;
  private final Mass robotMass;
  private final List<Finding> notes = new ArrayList<>();

  private MissingNotes(RobotDefinition robot, RobotMath math, Mass robotMass) {
    this.robot = robot;
    this.math = math;
    this.robotMass = robotMass;
  }

  /**
   * The notes about a robot: the robot's own first, then each joint's in order, then sensors'.
   *
   * @param robotMass the whole robot's mass as the program takes it: as weighed, or until it is,
   *     the most the season allows
   */
  static List<Finding> of(RobotDefinition robot, RobotMath math, Mass robotMass) {
    MissingNotes missing = new MissingNotes(robot, math, robotMass);
    missing.addTheBodys();
    missing.addTheDrivetrains();
    for (Joint joint : robot.joints()) {
      missing.addAJoints(joint);
    }
    missing.addUncheckedOutputs();
    for (Sensor sensor : math.sensors()) {
      missing.addASensors(sensor);
    }
    return missing.notes;
  }

  /**
   * Adds one note.
   *
   * @param subject what it is about: the robot, by its name, a joint, or a sensor
   * @param what what is not known: {@code "pivot has no kG"}
   * @param costs what the robot's code, its derived values or the simulator lose for it
   * @param scope what reads the missing value: the robot, or only the simulator
   */
  private void note(String subject, String what, String costs, Finding.Scope scope) {
    notes.add(Finding.note(scope, subject, what + ": " + costs));
  }

  /** The body's mass, how hard it is to turn, and how much of it is in no link. */
  private void addTheBodys() {
    Body body = robot.body();
    if (body.weight().isEmpty()) {
      note(
          robot.name(),
          "the robot isn't weighed: it's taken at the season's maximum, "
              + shown(robotMass)
              + " with battery",
          "the drive's moves and feedforward, the simulator's acceleration and the links' mass"
              + " budget rest on the rules, not the robot",
          Finding.Scope.ROBOT);
    }
    if (body.inertia().isEmpty()) {
      note(
          robot.name(),
          "the body's inertia is a box's, its mass spread through its bumpers' outline and"
              + " overall height",
          "how fast the drive's moves turn, its turning feedforward, and how the simulator"
              + " tips and turns the robot are approximate",
          Finding.Scope.ROBOT);
    } else if (body.inertia().get() instanceof Body.Turning.Yaw) {
      note(
          robot.name(),
          "the body's pitch and roll are a box's, beside its given yaw",
          "how the simulator tips the robot is approximate",
          Finding.Scope.SIMULATOR);
    }
    // A center of mass that is not given gets no note: nothing reads it but the mass model's own
    // check.
    double robotKilograms = robotMass.in(Kilograms);
    double inNoLink = robotKilograms - math.linksKilograms();
    note(
        robot.name(),
        String.format(
            Locale.ROOT, "%.2f kg of the robot's %.2f kg is in no link", inNoLink, robotKilograms),
        "the chassis, its electronics, bumpers and battery: carried as one rigid body",
        Finding.Scope.SIMULATOR);
  }

  /**
   * What the drivetrain's modules leave to their defaults, and which of its motor controllers do
   * not say what feeds them.
   */
  private void addTheDrivetrains() {
    addModulesDefaults();
    addModulesWithNoFeed();
  }

  /**
   * The values the modules leave to their defaults: the tread's grip, which the robot plans and
   * judges by, and the inertia of the drive and of the steering, which only the simulator reads.
   * Each is noted once if any corner's module leaves it out.
   */
  private void addModulesDefaults() {
    List<DrivetrainGeometry.ModuleType> modules =
        robot.drivetrain().corners().stream().map(DrivetrainGeometry.Corner::module).toList();
    if (modules.stream().anyMatch(module -> module.wheelCoefficientOfFriction().isEmpty())) {
      note(
          robot.name(),
          "the tread's grip isn't measured: it's "
              + shown(DrivetrainGeometry.ModuleType.DEFAULT_WHEEL_COEFFICIENT_OF_FRICTION)
              + ", PathPlanner's default",
          "drive moves plan their acceleration on it, and the slip check judges by it: measure"
              + " it with a pull test (docs/characterization.md, tread friction)",
          Finding.Scope.ROBOT);
    }
    if (modules.stream().anyMatch(module -> module.driveInertia().isEmpty())) {
      note(
          robot.name(),
          "the drive's own inertia isn't measured: it's "
              + shown(DrivetrainGeometry.ModuleType.DEFAULT_DRIVE_INERTIA.in(KilogramSquareMeters))
              + " kg m², an estimate for a 4 in wheel and a Kraken X60's rotor",
          "how fast the simulator's wheels spin up is a guess, though a small one beside the"
              + " robot's mass they carry",
          Finding.Scope.SIMULATOR);
    }
    if (modules.stream().anyMatch(module -> module.steerInertia().isEmpty())) {
      note(
          robot.name(),
          "the steering's inertia isn't measured: it's "
              + shown(DrivetrainGeometry.ModuleType.DEFAULT_STEER_INERTIA.in(KilogramSquareMeters))
              + " kg m², Phoenix Tuner X's simulation value",
          "how fast the simulator's modules steer at their current limit is a guess",
          Finding.Scope.SIMULATOR);
    }
  }

  /**
   * The drivetrain's motor controllers whose wiring does not say which PDH channel feeds them,
   * noted once. With no PDH declared there is nothing to check a feed against, and no note.
   */
  private void addModulesWithNoFeed() {
    List<CanDevice> wired =
        robot.drivetrain().wiring().map(SwerveWiring::devices).orElse(List.of());
    List<String> withNoFeed =
        motorControllersWithNoFeed(wired).stream().map(CanDevice::name).toList();
    if (robot.pdh().isEmpty() || withNoFeed.isEmpty()) {
      return;
    }
    // Each corner has a drive controller and a steering controller.
    int allOfThem = 2 * DrivetrainGeometry.LABELS.size();
    String what = "the drivetrain's " + String.join(", ", withNoFeed) + " have no PDH channel";
    if (withNoFeed.size() == allOfThem) {
      what = "none of the drivetrain's motor controllers has a PDH channel";
    }
    note(
        robot.name(),
        what,
        "which channel feeds each, and through what breaker, isn't checked: give each module's"
            + " wiring its feeds, .driveFeed(Pdh.feed(channel, breaker)) and .steerFeed(...)",
        Finding.Scope.ROBOT);
  }

  /** The motor controllers among some devices whose wiring does not say their PDH channel. */
  private static List<CanDevice> motorControllersWithNoFeed(List<CanDevice> devices) {
    return devices.stream()
        .filter(device -> device.kind().motorController() && device.feed().isEmpty())
        .toList();
  }

  /** One joint's notes: its link's mass and shape, what moves it, its kG and its motors' feeds. */
  private void addAJoints(Joint joint) {
    // A joint that is held at positions and that gravity acts on sags without a kG.
    boolean heldAgainstGravity = joint.holdsPositions() && math.gravityActsOn(joint);
    addLinksMass(joint, heldAgainstGravity);
    if (joint.link().shapes().isEmpty()) {
      note(
          joint.label(),
          joint.link().label() + " has no shape",
          "it isn't drawn, nor checked against the starting height and the extension limit",
          Finding.Scope.ROBOT);
    }
    if (!isMoved(joint)) {
      note(
          joint.label(),
          joint.label() + " has nothing moving it",
          "no drive turns it and no coupling leads it: no ideal gains, and the simulator"
              + " leaves it where it is",
          Finding.Scope.ROBOT);
    }
    addKg(joint, heldAgainstGravity);
    addMotorsWithNoFeed(joint);
  }

  /**
   * A link with no mass, or with mass and no inertia. The simulator works out the joint's inertia,
   * and its gravity, from its links, so what a link leaves out the joint goes without.
   */
  private void addLinksMass(Joint joint, boolean heldAgainstGravity) {
    Link link = joint.link();
    boolean hasInertia = !link.mass().inertia().isZero();
    if (!link.mass().isMassless()) {
      if (!hasInertia) {
        note(
            joint.label(),
            link.label() + " has mass but no inertia",
            "its own turning adds nothing to " + possessive(joint.label()) + " inertia",
            Finding.Scope.SIMULATOR);
      }
      return;
    }
    String what = link.label() + " has no mass";
    String addsToInertia = " adds nothing to ";
    if (hasInertia) {
      what = link.label() + " has no mass, only its inertia";
      addsToInertia = " adds only its own turning to ";
    }
    String gravityComesFromKg = "";
    if (heldAgainstGravity) {
      gravityComesFromKg = possessive(joint.label()) + " gravity comes from its kG, and ";
    }
    String rotorsSpinItUp = "";
    if (!hasInertia) {
      rotorsSpinItUp = rotorsSpinItUp(joint);
    }
    note(
        joint.label(),
        what,
        gravityComesFromKg
            + link.label()
            + addsToInertia
            + possessive(joint.label())
            + " inertia"
            + rotorsSpinItUp,
        Finding.Scope.SIMULATOR);
  }

  /**
   * What a joint with no mass gets from nothing but its motors' rotors, if it only runs at speeds:
   * how long they take to bring it to its fastest named speed, at their stator limit ({@link
   * RobotMath#secondsFromRestAtStatorLimit}). The simulator has it there that fast, which hides a
   * move that waited too little for it. Nothing for a joint held at positions, one whose outputs
   * name no speed, or one whose time can't be worked out.
   */
  private String rotorsSpinItUp(Joint joint) {
    if (joint.holdsPositions()) {
      return "";
    }
    Optional<Output> fastest =
        joint.outputs().stream()
            .filter(output -> output.kind() == Output.Kind.VELOCITY)
            .max(Comparator.comparingDouble(output -> Math.abs(output.value())));
    if (fastest.isEmpty()) {
      return "";
    }
    Output speed = fastest.get();
    OptionalDouble seconds = math.secondsFromRestAtStatorLimit(joint, speed.value());
    if (seconds.isEmpty()) {
      return "";
    }
    return ", so its rotors alone bring it to "
        + speed.name()
        + ", "
        + joint.shownSpeed(speed.value())
        + ", in "
        + shownTime(seconds.getAsDouble());
  }

  /**
   * A time as a note says it, as an estimate: whole milliseconds below a tenth of a second,
   * otherwise tenths of a second.
   */
  private static String shownTime(double seconds) {
    if (seconds < TENTH_OF_A_SECOND) {
      long milliseconds = Math.round(seconds * 1000);
      if (milliseconds == 0) {
        return "under a millisecond";
      }
      return "about " + milliseconds + " ms";
    }
    return "about " + shown(Math.round(seconds * 10) / 10.0) + " s";
  }

  /** Whether anything moves a joint: a coupling that leads it, or a drive that turns it. */
  private boolean isMoved(Joint joint) {
    for (Coupling coupling : robot.couplings()) {
      if (coupling.follower().equals(joint)) {
        return true;
      }
    }
    for (Drive drive : math.drives()) {
      if (drive.transmission().joints().contains(joint)) {
        return true;
      }
    }
    return false;
  }

  /**
   * A kG that is needed and not written. It is needed by a joint that its own control drives, that
   * is held at positions, and that gravity acts on. A roller, a turret and a joint that follows
   * another need none.
   *
   * <p>The links' mass may seed a kG for such a joint, but only a tuned kG is known to hold it, so
   * a seeded one is noted too. A joint that a counterbalance holds writes a kG of 0.
   */
  private void addKg(Joint joint, boolean heldAgainstGravity) {
    Optional<MechanismControl> control = joint.driven().control();
    boolean needsAKg = control.isPresent() && heldAgainstGravity && math.leaderOf(joint) == null;
    if (!needsAKg || control.get().gains().kG().isPresent()) {
      return;
    }
    boolean seededFromItsMass = math.gravityHeld(joint) > NO_GRAVITY;
    if (seededFromItsMass) {
      note(
          joint.label(),
          possessive(joint.label()) + " kG is seeded from its mass, not tuned",
          "it holds only as well as its links' mass and where it sits are right: tune its kG and"
              + " write it, withKg(...)",
          Finding.Scope.ROBOT);
      return;
    }
    note(
        joint.label(),
        joint.label() + " has no kG",
        "it holds against gravity by kP alone, so it sits below its targets until its kG is tuned"
            + " and written, withKg(...), or written as 0 for a counterbalanced joint",
        Finding.Scope.ROBOT);
  }

  /**
   * A joint's motor controllers whose wiring does not say which PDH channel feeds them. There is no
   * note when the robot declares no PDH, or the joint's motors are not wired.
   */
  private void addMotorsWithNoFeed(Joint joint) {
    List<String> withNoFeed =
        motorControllersWithNoFeed(joint.devices()).stream()
            .map(device -> String.valueOf(device.id()))
            .toList();
    if (robot.pdh().isEmpty() || withNoFeed.isEmpty()) {
      return;
    }
    String which = " motor controllers " + String.join(", ", withNoFeed) + " have";
    if (withNoFeed.size() == 1) {
      which = " motor controller " + withNoFeed.get(0) + " has";
    }
    note(
        joint.label(),
        possessive(joint.label()) + which + " no PDH channel",
        "which channel feeds each, and through what breaker, isn't checked: give each its own where"
            + " it's wired, .pdh(channel, breaker)",
        Finding.Scope.ROBOT);
  }

  /** The joints that name speeds or voltages and have no motors to check them against. */
  private void addUncheckedOutputs() {
    for (Joint joint : JointSpeedChecks.unchecked(math)) {
      note(
          joint.label(),
          joint.label() + " names outputs, and no motors turn it",
          "its speeds and voltages aren't checked against what its motors can do",
          Finding.Scope.ROBOT);
    }
  }

  /** A sensor that is not placed on its part, and a limit switch whose position is not known. */
  private void addASensors(Sensor sensor) {
    if (sensor instanceof Sensor.Placed placed && placed.pose().isEmpty()) {
      note(
          sensor.label(),
          sensor.label() + " isn't placed on its part",
          "it's drawn, and simulated, at its part's origin",
          Finding.Scope.ROBOT);
    }
    if (sensor instanceof Sensor.Switch onSwitch
        && onSwitch.kind() instanceof Sensor.Switch.Limit limit
        && limit.position().isEmpty()) {
      note(
          sensor.label(),
          possessive(sensor.label()) + " position on " + limit.joint().label() + " isn't known",
          "it can stop " + limit.joint().label() + " but not home it",
          Finding.Scope.ROBOT);
    }
  }
}
