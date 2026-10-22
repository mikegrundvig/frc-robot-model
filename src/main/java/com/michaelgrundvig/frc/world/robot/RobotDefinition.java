package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.config.Rule;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.Set;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Mass;

/**
 * What one robot is: its body, its drivetrain, where its cameras and IMUs are mounted, and its
 * mechanisms: their joints, how the joints are coupled, what drives them, and their sensors. Every
 * reader of the robot reads this one description: localization (the wheels' positions, the mounts),
 * the simulator (all of it), the mechanisms' own code (their ratios, ranges and ideal gains, {@link
 * #math}), and the views. How each joint is driven (gains, limits) and wired (CAN IDs, an encoder's
 * offset) is in its block too ({@link Driven}), as the drivetrain's is in its geometry, but isn't
 * part of what it is: a joint re-wired or re-tuned is the same joint, so a practice robot is the
 * competition robot with its differences ({@link #with}). The PDH that feeds them is the robot's
 * ({@link #pdh}).
 *
 * <p>It also names the robot's places: each joint's named positions and outputs, the configurations
 * the robot takes ({@link Configuration}), and the interlocks between joints ({@link Interlock}).
 * They're facts about the robot, read by everything that reads it; what to do with them is the
 * robot program's. A place is a name on its joint, its value this robot's: the configurations and
 * interlocks, written of the joints' constants, are taken as about this robot's own joints, each
 * named place where its joint puts it, so a practice robot's joint, its places re-placed, is
 * checked and driven at its own.
 *
 * <p>Each part checks its own values when built; the checks here are those across parts, each
 * saying what it finds ({@link Finding}): a camera mounted inside the body, a drive's joint among
 * the robot's joints, the chassis left once the links are taken off the body still a body, an
 * absolute encoder telling its joint's whole range apart, a named output within what its joint's
 * motors can do; each joint's block against itself (where it rests and its places inside its hard
 * stops, judged here on the finished joint so a practice robot changes them in any order, its
 * control's soft limits inside its hard stops and around its places, a homing recipe it can carry
 * out, a loop on its absolute encoder only where the encoder tells its whole range); the
 * drivetrain's top speed within its free speed; one sensor on each input, and a switch on a motor
 * controller's limit input only where there is one; every configuration obeying every interlock, no
 * interlocks holding joints by each other, no two devices of a kind on one CAN bus at one ID, and
 * every device fed from one of the PDH's channels, a channel to each, through a breaker no bigger
 * than it takes. What the program needs beyond what the robot is, a control for each joint it
 * drives, it asks of {@link #findings}. Every reference between parts is the part itself, never a
 * name, so a joint built after its parent can't be its own ancestor; labels are for logs and
 * writers, and unique.
 *
 * @param name which robot this is, as its logs and its checks' messages name it
 * @param body the frame, bumpers, and mass
 * @param drivetrain the swerve drivetrain, as built
 * @param cameras the AprilTag cameras
 * @param imus where each IMU is mounted, and whether its accelerometer has been checked to read
 *     along the robot's axes
 * @param pdh its power distribution hub, whose channels feed its motor controllers, each fed as its
 *     wiring says ({@code TALON_FX.can(32).pdh(4, Amps.of(40))}); with none, no device says one
 * @param joints the mechanisms' joints, each after its parent, each with its own motors, sensors,
 *     wiring and control in its block
 * @param couplings the joints that move as functions of others: an elevator's stages, a linkage
 * @param drives what moves the mechanisms beyond each joint's own: a differential's motors, turning
 *     two joints at once; each through its transmission
 * @param sensors the sensors beyond each joint's own: beam breaks, contact switches and
 *     rangefinders, each on its part and carrying its wiring
 * @param configurations the poses the robot takes by name, each some joints at their named
 *     positions or outputs: "stowed", "amp", "feeding"
 * @param interlocks what one joint may do by where another is, enforced beneath every command
 */
@ConfigRecord
public record RobotDefinition(
    String name,
    Body body,
    DrivetrainGeometry drivetrain,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<CameraConfig> cameras,
    Imus imus,
    Optional<Pdh> pdh,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Joint> joints,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Coupling> couplings,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Drive> drives,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Sensor> sensors,
    @RecordBuilder.Initializer(value = "of", source = List.class)
        List<Configuration> configurations,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Interlock> interlocks) {
  public RobotDefinition {
    cameras = List.copyOf(cameras);
    joints = List.copyOf(joints);
    couplings = List.copyOf(couplings);
    drives = List.copyOf(drives);
    sensors = List.copyOf(sensors);
    List<Joint> robots = joints;
    configurations = configurations.stream().map(each -> OwnJoints.onOwn(robots, each)).toList();
    interlocks = interlocks.stream().map(each -> OwnJoints.onOwn(robots, each)).toList();
    List<CameraConfig> allCameras = cameras;
    List<Joint> allJoints = joints;
    List<Coupling> allCouplings = couplings;
    List<Configuration> allConfigurations = configurations;
    List<Drive> everyDrive = everyDrive(joints, drives);
    List<Sensor> everySensor = everySensor(joints, sensors);
    List<Interlock> everyInterlock = everyInterlock(joints, interlocks);
    List<CanDevice> everyDevice = RobotDevices.of(pdh, joints, drivetrain, everySensor);
    Optional<RobotMath> whole = RobotMath.whole(body, joints, couplings, everyDrive, everySensor);
    Checks.on(RobotDefinition.class, name)
        .field("name", name, notBlank())
        .field("cameras", cameras, Rule.of(RobotDefinition::whyNamesAreShared))
        .whyNot("drivetrain.corners", () -> drivetrain.whyWheelsDontFit(body))
        .whyNot(
            "drivetrain.control.limits.maxLinearSpeed",
            () -> drivetrain.control().flatMap(control -> control.whyTooFastFor(drivetrain)))
        .findings("cameras", () -> outside(allCameras, body))
        .findings("imus", () -> outside(imus, body))
        .findings(
            "joints",
            () ->
                RobotMath.problems(
                    name, body, allJoints, allCouplings, everyDrive, everySensor, allCameras))
        .findings("joints", () -> whole.map(Blocks::problems).orElse(List.of()))
        .findings("sensors", () -> Inputs.problems(allJoints, everySensor))
        .findings(
            "configurations",
            () ->
                whole
                    .map(
                        math ->
                            ConfigurationChecks.problems(math, allConfigurations, everyInterlock))
                    .orElse(List.of()))
        .findings("interlocks", () -> twice(everyInterlock))
        .findings(
            "interlocks",
            () ->
                whole.map(math -> InterlockChecks.problems(math, everyInterlock)).orElse(List.of()))
        .findings("devices", () -> CanDevice.sharedIds(everyDevice))
        .findings("pdh", () -> Pdh.problems(pdh, everyDevice))
        .orThrow();
  }

  /** Starts describing a robot, one named value at a time. */
  public static RobotDefinitionBuilder.NameStage builder() {
    return RobotDefinitionBuilder.builder();
  }

  /** Where each wheel touches the floor: front left, front right, back left, back right. */
  public Translation2d[] moduleTranslations() {
    return drivetrain.moduleTranslations();
  }

  /**
   * The same robot under another name: a practice robot of its design, {@code
   * COMPETITION.named("practice").with(...)}.
   */
  public RobotDefinition named(String name) {
    return RobotDefinitionBuilder.builder(this).name(name).build();
  }

  /**
   * The same robot with some of its joints wired, tuned or placed otherwise: a practice robot of
   * its design, {@code COMPETITION.with(PIVOT.motors(motors -> motors.on(TALON_FX.can(33))))}. Each
   * joint given replaces the robot's joint it's equal to, the same in all but how it's driven, its
   * stops, where it rests and where its places are, so everything that names that joint or its
   * places, behavior included, finds this one's. Each is the robot's joint changed one thing at a
   * time ({@code motors}, {@code absoluteEncoder} and {@code limitSwitch} with what changes on the
   * part, {@code retuned}, {@code replaced}, {@code range}, {@code rest}), the rest kept, so a
   * change to the robot's own block reaches this one.
   *
   * @throws IllegalArgumentException for a joint the robot has no equal of, or one given twice
   */
  public RobotDefinition with(Joint... changed) {
    return RobotDefinitionBuilder.builder(this)
        .joints(
            replaced(
                joints,
                List.of(changed),
                joint -> joint,
                Joint::label,
                joint ->
                    name
                        + " has no joint "
                        + joint.label()
                        + " the same but for how it's driven, its stops, where it rests and"
                        + " where its places are"))
        .build();
  }

  /**
   * The same robot with some of its sensors wired otherwise: a beam on another CANdi, {@code
   * COMPETITION.with(CLAW_BEAM.on(CANDI.can(36).s1().normallyOpen()))}. Each replaces the robot's
   * sensor it's equal to, the same but for its wiring. A joint's own limit switch or absolute
   * encoder is re-wired with its joint ({@code limitSwitch(end, limit -> ...)}, {@code
   * absoluteEncoder(encoder -> ...)}).
   *
   * @throws IllegalArgumentException for a sensor the robot has no equal of, or one given twice
   */
  public RobotDefinition with(Sensor... changed) {
    for (Sensor sensor : changed) {
      for (Joint joint : joints) {
        if (joint.ownSensors().contains(sensor)) {
          throw new IllegalArgumentException(
              sensor.label()
                  + " is "
                  + possessive(joint.label())
                  + " own: re-wire it with its joint, "
                  + howAJointRewires(sensor));
        }
      }
    }
    return RobotDefinitionBuilder.builder(this)
        .sensors(
            replaced(
                sensors,
                List.of(changed),
                sensor -> sensor,
                Sensor::label,
                sensor ->
                    name + " has no sensor " + sensor.label() + " the same but for its wiring"))
        .build();
  }

  /**
   * The call on a joint that re-wires one of the joint's own sensors, as a refusal suggests it. A
   * joint's own sensors are its limit switches and its absolute encoder.
   */
  private static String howAJointRewires(Sensor own) {
    if (own instanceof Sensor.Switch onSwitch
        && onSwitch.kind() instanceof Sensor.Switch.Limit limit) {
      return "limitSwitch(End." + limit.end().name() + ", limit -> limit.on(...))";
    }
    return "absoluteEncoder(encoder -> encoder.on(...))";
  }

  /**
   * The same robot with some of its cameras mounted otherwise: a practice robot's back camera,
   * lower, {@code COMPETITION.with(CameraConfigBuilder.builder(BACK).up(Inches.of(20.8)).build())}.
   * Each replaces the robot's camera of its name, all of it: where it sits and points, the camera
   * itself, and how far its results are trusted.
   *
   * @throws IllegalArgumentException for a camera the robot has none of by its name, or one given
   *     twice
   */
  public RobotDefinition with(CameraConfig... changed) {
    return RobotDefinitionBuilder.builder(this)
        .cameras(
            replaced(
                cameras,
                List.of(changed),
                CameraConfig::name,
                CameraConfig::name,
                camera ->
                    name
                        + " has no camera named "
                        + camera.name()
                        + ": it has "
                        + namesOfItsCameras()))
        .build();
  }

  /** The cameras' names as a refusal lists them, or "none" for a robot with no cameras. */
  private String namesOfItsCameras() {
    if (cameras.isEmpty()) {
      return "none";
    }
    return String.join(", ", cameras.stream().map(CameraConfig::name).toList());
  }

  /**
   * The same robot, its drivetrain wired or driven otherwise: a practice robot's, its modules'
   * CANcoder offsets its own, {@code COMPETITION.with(DRIVETRAIN.wired(PRACTICE_SWERVE))}. Its
   * modules may differ in what only the simulator reads, their inertias, which it takes too.
   *
   * @throws IllegalArgumentException for a drivetrain that isn't this robot's: other modules, or
   *     placed otherwise
   */
  public RobotDefinition with(DrivetrainGeometry changed) {
    if (!changed.equals(drivetrain)) {
      throw new IllegalArgumentException(
          possessive(name)
              + " drivetrain isn't this one: their modules, or where they sit, differ. Give its"
              + " own, re-wired, .wired(...), or re-tuned, .control(...)");
    }
    return RobotDefinitionBuilder.builder(this).drivetrain(changed).build();
  }

  /**
   * Parts, some replaced by those the same as them, each given once: a second would drop the
   * first's change.
   *
   * @param same what two parts are the same by: the part itself, its equality, or its name
   * @param label what a part is called, for a message
   * @param none why the robot has no part the same as one given
   */
  private static <T> List<T> replaced(
      List<T> parts,
      List<? extends T> changed,
      Function<? super T, ?> same,
      Function<? super T, String> label,
      Function<? super T, String> none) {
    List<T> each = new ArrayList<>(parts);
    Set<Object> given = new HashSet<>();
    for (T part : changed) {
      Object sameBy = same.apply(part);
      OptionalInt at = indexOfSame(each, sameBy, same);
      if (at.isEmpty()) {
        throw new IllegalArgumentException(none.apply(part));
      }
      boolean alreadyGiven = !given.add(sameBy);
      if (alreadyGiven) {
        throw new IllegalArgumentException(
            label.apply(part)
                + " is given twice, and the second would drop the first's change: give it once,"
                + " its changes one after another");
      }
      each.set(at.getAsInt(), part);
    }
    return each;
  }

  /** Where the first part that is the same as another is in a list, if any is. */
  private static <T> OptionalInt indexOfSame(
      List<T> parts, Object sameBy, Function<? super T, ?> same) {
    for (int index = 0; index < parts.size(); index++) {
      if (same.apply(parts.get(index)).equals(sameBy)) {
        return OptionalInt.of(index);
      }
    }
    return OptionalInt.empty();
  }

  /**
   * What the mechanisms derive: their poses, ratios, ideal gains, gravity and reach, as pure
   * functions of their joints' positions.
   */
  public RobotMath math() {
    return RobotMath.of(
        body, joints, couplings, everyDrive(joints, drives), everySensor(joints, sensors));
  }

  /**
   * Every device on the robot's CAN buses, each once, as the definition wires it ({@link
   * CanDevice}): the PDH; the drivetrain's gyro, and each module's drive, steering and CANcoder;
   * each joint's motor controllers and CANcoder, in its block; and the CANdis and CANranges its
   * sensors are on. Each motor controller carries the PDH channel that feeds it, where its wiring
   * says. The one list the checks across the robot's wiring read: no two devices of a kind on one
   * bus at one ID, the drivetrain's included, and no channel feeding two devices.
   */
  public List<CanDevice> devices() {
    return RobotDevices.of(pdh, joints, drivetrain, everySensor(joints, sensors));
  }

  /**
   * What the definition says about itself to the program that drives the robot ({@link Finding}):
   * what the program can't drive, which the definition itself allows; what's doubtful, though
   * allowed; and what it doesn't know, with what that costs and what reads it, the robot first and
   * then each joint in order. The program refuses the first, and says the rest at startup.
   *
   * <p>What it can't drive ({@link Finding.Level#REFUSED}): a joint with motors and no control, and
   * a drivetrain with no control. Localization, the simulator's physics and the views need neither.
   *
   * <p>Doubtful ({@link Finding.Level#DOUBT}): a named speed above nine tenths of what its joint's
   * motors turn it at, which a velocity loop can't hold under load ({@code RobotMath.freeSpeed});
   * one above it all is refused. A joint resting inside a region it's kept out of as the robot
   * starts, which it may only leave until the region opens; a configuration there is refused.
   *
   * <p>Not known ({@link Finding.Level#NOTE}): nothing here is wrong, only unknown. A value nobody
   * has is zero or the model's own stand-in, never a guess, and this says where. On the robot
   * ({@link Finding.Scope#ROBOT}):
   *
   * <ul>
   *   <li>the robot not weighed, taken at the mass given here, the season's most, or its inertia a
   *       box's: the drive's moves and feedforward read them;
   *   <li>the tread's grip not measured, PathPlanner's default: the drive's moves plan on it, and
   *       the slip check judges by it;
   *   <li>a joint gravity acts on, held at positions, with no kG written: none, or one seeded from
   *       its links' mass, never tuned;
   *   <li>a link with no shape;
   *   <li>a joint nothing drives or couples;
   *   <li>a joint with named speeds or voltages no motors turn it at, which can't be checked;
   *   <li>a sensor not placed on its part, or a limit switch whose position isn't known;
   *   <li>with a PDH declared, a motor controller whose wiring doesn't say which of its channels
   *       feeds it: the drivetrain's, then each joint's.
   * </ul>
   *
   * <p>In the simulator alone ({@link Finding.Scope#SIMULATOR}), since mass is optional and only
   * the simulator reads it: the body's pitch and roll a box's beside a given yaw; its center of
   * mass not given; the share of its mass in no link; the modules' drive or steering inertia not
   * measured; a link with no mass, or mass but no inertia.
   *
   * @param robotMass the whole robot's mass as the program takes it: as weighed, or until it is,
   *     the most the season allows ({@code SeasonLimits.robotMass}), which the notes then name
   */
  public List<Finding> findings(Mass robotMass) {
    RobotMath math = math();
    List<Finding> found = judged(math);
    found.addAll(MissingNotes.of(this, math, robotMass));
    return found;
  }

  /**
   * What the program can't drive, of what the definition says about itself ({@link #findings}):
   * none of it reads the robot's mass, so a program refuses it before it knows its season.
   */
  public List<Finding> refusals() {
    return judged(math()).stream().filter(Finding::refused).toList();
  }

  /**
   * Everything wrong with the robot's mass at a mass, each in words; empty when it adds up: a given
   * yaw that can't stand beside a box's pitch and roll ({@code Body.whyInertiaDoesNotFit}), links
   * that leave nothing of it for the chassis, a chassis that turns more easily than any body. The
   * definition refuses these itself at the weight given; until the robot is weighed, whoever takes
   * a mass for it asks at that ({@code SeasonLimits.problems}).
   */
  public List<String> massProblems(Mass robotMass) {
    List<Optional<String>> found = new ArrayList<>();
    found.add(body.whyInertiaDoesNotFit(robotMass));
    found.add(AssemblyChecks.whyMassDoesNotAddUp(math(), body, robotMass));
    return found.stream().flatMap(Optional::stream).toList();
  }

  /** What can't be driven and what's doubtful: the findings that aren't notes of what's missing. */
  private List<Finding> judged(RobotMath math) {
    List<Finding> found = new ArrayList<>();
    if (drivetrain.control().isEmpty()) {
      found.add(
          Finding.refused(
              "drivetrain",
              "the drivetrain has no control: give it its limits and gains,"
                  + " .control(DrivetrainControl.builder()...)"));
    }
    found.addAll(Blocks.uncontrolled(math));
    found.addAll(JointSpeedChecks.doubts(math));
    found.addAll(InterlockChecks.startDoubts(math, everyInterlock()));
    return found;
  }

  /** Each joint's own drives, then the robot's. */
  private static List<Drive> everyDrive(List<Joint> joints, List<Drive> drives) {
    List<Drive> every = new ArrayList<>();
    joints.forEach(joint -> every.addAll(joint.ownDrives()));
    every.addAll(drives);
    return every;
  }

  /**
   * This robot's own joint, the one equal to a joint: the competition robot's {@code HOOD}, on its
   * practice robot, with the practice robot's stops and places. A subsystem of the team's own reads
   * its joint from it rather than restating it.
   *
   * @throws IllegalArgumentException for a joint the robot hasn't
   */
  public Joint own(Joint joint) {
    int at = joints.indexOf(joint);
    if (at < 0) {
      throw new IllegalArgumentException(
          name + " has no joint " + joint.label() + ": add it to the definition's joints");
    }
    return joints.get(at);
  }

  /**
   * A condition about this robot's own joint, each place it names where this robot puts it: the
   * competition robot's {@code PIVOT.atLeast(PIVOT_OUT)}, on its practice robot, from the practice
   * robot's out. One about a sensor, as it is; one about a joint the robot hasn't, as it is.
   */
  public Condition own(Condition condition) {
    return OwnJoints.onOwn(joints, condition);
  }

  /**
   * Why a place a condition names isn't where this robot's joint has it, or null when each is: a
   * gate on it is refused as a move there is, and an interlock or a configuration on it is refused
   * here, as the definition is built. The competition robot's constant is its practice robot's own.
   */
  public @Nullable String whyNotOwn(Condition condition) {
    return Places.whyNotOwn(OwnJoints.onOwn(joints, condition)).orElse(null);
  }

  /**
   * Every interlock: each joint's own, from its block, then the robot's list. What the robot
   * enforces, and what the definition checks.
   */
  public List<Interlock> everyInterlock() {
    return everyInterlock(joints, interlocks);
  }

  /** Each joint's own interlocks, then the robot's, each about the robot's own joints. */
  private static List<Interlock> everyInterlock(List<Joint> joints, List<Interlock> interlocks) {
    List<Interlock> every = new ArrayList<>();
    joints.forEach(
        joint ->
            joint
                .ownInterlocks()
                .forEach(interlock -> every.add(OwnJoints.onOwn(joints, interlock))));
    every.addAll(interlocks);
    return every;
  }

  /** An interlock written twice, in a block and in the robot's list, or twice in one. */
  private static List<Finding> twice(List<Interlock> interlocks) {
    Set<Interlock> seen = new HashSet<>();
    for (Interlock interlock : interlocks) {
      if (!seen.add(interlock)) {
        return List.of(
            Finding.refused(
                interlock.joint().label(),
                "\""
                    + interlock
                    + "\" is written twice: once is enough, in "
                    + possessive(interlock.joint().label())
                    + " block or the robot's interlocks"));
      }
    }
    return List.of();
  }

  /**
   * Every sensor: each joint's own, from its block, then the robot's list. A record of what the
   * robot did names a sensor by where it is in this list.
   */
  public List<Sensor> everySensor() {
    return everySensor(joints, sensors);
  }

  /** Each joint's own sensors, then the robot's. */
  private static List<Sensor> everySensor(List<Joint> joints, List<Sensor> sensors) {
    List<Sensor> every = new ArrayList<>();
    joints.forEach(joint -> every.addAll(joint.ownSensors()));
    every.addAll(sensors);
    return every;
  }

  /**
   * Why cameras' names cannot all be theirs, if they cannot: two have the same name, or names that
   * differ only in case or punctuation. Those become the same id ({@link CameraConfig#id()}), so
   * "Front Left" and "front-left" are one camera to everything that reads ids.
   */
  private static Optional<String> whyNamesAreShared(List<CameraConfig> cameras) {
    Map<String, String> nameById = new HashMap<>();
    List<String> shared = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      String earlier = nameById.putIfAbsent(camera.id(), camera.name());
      if (earlier == null) {
        continue;
      }
      if (earlier.equals(camera.name())) {
        shared.add("two cameras are named " + camera.name());
        continue;
      }
      shared.add(
          "cameras \""
              + earlier
              + "\" and \""
              + camera.name()
              + "\" differ only in case or punctuation, so they're one id, \""
              + camera.id()
              + "\": name them apart");
    }
    if (shared.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(String.join("; ", shared));
  }

  /**
   * The cameras on the chassis mounted outside the robot. A camera on a joint is checked with its
   * joint stowed ({@link RobotMath}).
   */
  private static List<Finding> outside(List<CameraConfig> cameras, Body body) {
    List<Finding> outside = new ArrayList<>();
    for (CameraConfig camera : cameras) {
      if (camera.joint().isEmpty() && !body.contains(camera.position())) {
        outside.add(
            Finding.refused(
                camera.name(),
                "camera " + camera.name() + " is outside the robot, at " + camera.position()));
      }
    }
    return outside;
  }

  /** The IMUs mounted outside the robot. */
  private static List<Finding> outside(Imus imus, Body body) {
    List<Finding> outside = new ArrayList<>();
    outside(outside, "the gyro", imus.gyro(), body);
    imus.backup().ifPresent(backup -> outside(outside, "the backup IMU", backup, body));
    return outside;
  }

  private static void outside(List<Finding> outside, String name, ImuMount imu, Body body) {
    if (!body.contains(imu.position())) {
      outside.add(Finding.refused(name, name + " is outside the robot, at " + imu.position()));
    }
  }
}
