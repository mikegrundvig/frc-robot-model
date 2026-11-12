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
 * <p>It also names the robot's places: each joint's named positions and outputs, the presets the
 * robot takes ({@link Preset}), and the interlocks between joints ({@link Interlock}). They're
 * facts about the robot, read by everything that reads it; what to do with them is the robot
 * program's. A place is a name on its joint, its value this robot's: the presets and interlocks,
 * written of the joints' constants, are taken as about this robot's own joints, each named place
 * where its joint puts it, so a practice robot's joint, its places re-placed, is checked and driven
 * at its own.
 *
 * <p>Each part checks its own values when built. The checks here are those across parts, run once
 * on the finished robot ({@link #checked}), so its {@code with...} verbs come in any order. Each
 * says what it finds ({@link Finding}): a camera mounted inside the body, a drive's joint among the
 * robot's joints, the chassis left once the links are taken off the body still a body, an absolute
 * encoder telling its joint's whole range apart, a named output within what its joint's motors can
 * do; each joint's block against itself (where it rests and its places inside its hard stops,
 * judged here on the finished joint so a practice robot changes them in any order, its control's
 * soft limits inside its hard stops and around its places, a homing recipe it can carry out, a loop
 * on its absolute encoder only where the encoder tells its whole range); the drivetrain's top speed
 * within its free speed; one sensor on each input, and a switch on a motor controller's limit input
 * only where there is one; every preset obeying every interlock, no interlocks holding joints by
 * each other, no two devices of a kind on one CAN bus at one ID, and every device fed from one of
 * the PDH's channels, a channel to each, through a breaker no bigger than it takes. What the
 * program needs beyond what the robot is, a control for each joint it drives, it asks of {@link
 * #findings}. Every reference between parts is the part itself, never a name, so a joint built
 * after its parent can't be its own ancestor; labels are for logs and writers, and unique.
 *
 * @param name which robot this is, as its logs and its checks' messages name it
 * @param body the frame, bumpers, and mass
 * @param drivetrain the swerve drivetrain, as built
 * @param cameras the AprilTag cameras
 * @param backupImu where the backup IMU is mounted, and whether its accelerometer has been checked
 *     to read along the robot's axes, if the robot has one: Systemcore's own, which stands in while
 *     the gyro is lost; or a second Pigeon. Empty for a robot with none: robot code then reads no
 *     backup, and with the gyro lost, the heading rests on the wheels. The gyro is the drivetrain's
 *     ({@link #gyroMount})
 * @param pdh its power distribution hub, whose channels feed its motor controllers, each fed as its
 *     wiring says ({@code TALON_FX.can(32).pdh(4, Amps.of(40))}); with none, no device says one
 * @param joints the mechanisms' joints, each after its parent, each with its own motors, sensors,
 *     wiring and control in its block
 * @param couplings the joints that move as functions of others: an elevator's stages, a linkage
 * @param drives what moves the mechanisms beyond each joint's own: a differential's motors, turning
 *     two joints at once; each through its transmission
 * @param sensors the sensors beyond each joint's own: beam breaks, contact switches and
 *     rangefinders, each on its part and carrying its wiring
 * @param presets the poses the robot takes by name, each some joints at their named positions or
 *     outputs: "stowed", "amp", "feeding"
 * @param interlocks what one joint may do by where another is, enforced beneath every command
 */
@ConfigRecord
public record RobotDefinition(
    String name,
    Body body,
    Drivetrain drivetrain,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<CameraConfig> cameras,
    Optional<ImuMount> backupImu,
    Optional<Pdh> pdh,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Joint> joints,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Coupling> couplings,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Drive> drives,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Sensor> sensors,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Preset> presets,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Interlock> interlocks) {
  public RobotDefinition {
    cameras = List.copyOf(cameras);
    joints = List.copyOf(joints);
    couplings = List.copyOf(couplings);
    drives = List.copyOf(drives);
    sensors = List.copyOf(sensors);
    List<Joint> robots = joints;
    presets = presets.stream().map(each -> OwnJoints.onOwn(robots, each)).toList();
    interlocks = interlocks.stream().map(each -> OwnJoints.onOwn(robots, each)).toList();
    // Only each part on its own is checked here, so the with... verbs can be called in any order.
    // The checks of one part against another wait for the finished robot (checked()).
    Checks.on(RobotDefinition.class, name)
        .field("name", name, notBlank())
        .field("cameras", cameras, Rule.of(RobotDefinition::whyACameraIsNotMounted))
        .orThrow();
  }

  /**
   * This robot, once its parts are checked against each other: a camera mounted inside the body and
   * named apart from the others, a drive's joint among the robot's joints, and the rest the class
   * comment lists. Each part is checked on its own as it's given, but what one part says about
   * another waits for the finished robot, so its {@code with...} verbs can come in any order.
   * {@code RobotConfig.of} calls this, as does everything that takes a definition to the robot or
   * the simulator.
   *
   * @return this robot, unchanged
   * @throws com.michaelgrundvig.frc.world.config.ConfigException naming every part that doesn't fit
   *     the others
   */
  public RobotDefinition checked() {
    List<Drive> everyDrive = everyDrive(joints, drives);
    List<Sensor> everySensor = everySensor(joints, sensors);
    List<Interlock> everyInterlock = everyInterlock(joints, interlocks);
    List<CanDevice> everyDevice = RobotDevices.of(pdh, joints, drivetrain, everySensor);
    Optional<RobotMath> whole = RobotMath.whole(body, joints, couplings, everyDrive, everySensor);
    Checks.on(RobotDefinition.class, name)
        .field("cameras", cameras, Rule.of(RobotDefinition::whyNamesAreShared))
        .whyNot("drivetrain.corners", () -> drivetrain.whyWheelsDontFit(body))
        .whyNot(
            "drivetrain.control.limits.maxLinearSpeed",
            () -> drivetrain.control().flatMap(control -> control.whyTooFastFor(drivetrain)))
        .findings("cameras", () -> outside(cameras, body))
        .findings(
            "drivetrain.wiring.gyro",
            () -> outsideAt("the gyro", gyroMountOf(drivetrain, backupImu), body))
        .findings("backupImu", () -> outside(backupImu, body))
        .findings(
            "joints",
            () ->
                RobotMath.problems(name, body, joints, couplings, everyDrive, everySensor, cameras))
        .findings("joints", () -> whole.map(Blocks::problems).orElse(List.of()))
        .findings("sensors", () -> Inputs.problems(joints, everySensor))
        .findings(
            "presets",
            () ->
                whole
                    .map(math -> PresetChecks.problems(math, presets, everyInterlock))
                    .orElse(List.of()))
        .findings("interlocks", () -> twice(everyInterlock))
        .findings(
            "interlocks",
            () ->
                whole.map(math -> InterlockChecks.problems(math, everyInterlock)).orElse(List.of()))
        .findings("devices", () -> CanDevice.sharedIds(everyDevice))
        .findings("pdh", () -> Pdh.problems(pdh, everyDevice))
        .orThrow();
    return this;
  }

  /**
   * A robot by its name, with its body and drivetrain, the rest added by its {@code with...} verbs:
   * {@code RobotDefinition.of("delta", BODY, DRIVETRAIN).withCameras(LEFT, RIGHT).withJoints(PIVOT,
   * INTAKE).withPresets(FEEDING)}.
   *
   * @param name which robot this is, as its logs and its checks' messages name it
   * @param body the frame, bumpers, and mass
   * @param drivetrain the swerve drivetrain, as built
   */
  public static RobotDefinition of(String name, Body body, Drivetrain drivetrain) {
    return builder().name(name).body(body).drivetrain(drivetrain).build();
  }

  /** Starts describing a robot, one named value at a time. */
  static RobotDefinitionBuilder.NameStage builder() {
    return RobotDefinitionBuilder.builder();
  }

  /**
   * The same robot with its backup IMU here: Systemcore's own, or a second Pigeon. One it had is
   * replaced.
   */
  public RobotDefinition withBackupImu(ImuMount backupImu) {
    return RobotDefinitionBuilder.builder(this).backupImu(backupImu).build();
  }

  /**
   * The same robot with its power distribution hub, whose channels feed its motor controllers. One
   * it had is replaced.
   */
  public RobotDefinition withPdh(Pdh pdh) {
    return RobotDefinitionBuilder.builder(this).pdh(pdh).build();
  }

  /**
   * The same robot with more joints, each after its parent, after its own: {@code
   * .withJoints(PIVOT, INTAKE, SHOOTER)}. To wire or tune one of its joints otherwise, {@link
   * #with(Joint...)}.
   */
  public RobotDefinition withJoints(Joint... added) {
    return RobotDefinitionBuilder.builder(this).joints(adding(joints, added)).build();
  }

  /** The same robot with more joints that move as functions of others: an elevator's stages. */
  public RobotDefinition withCouplings(Coupling... added) {
    return RobotDefinitionBuilder.builder(this).couplings(adding(couplings, added)).build();
  }

  /** The same robot with more drives beyond each joint's own: a differential's motors. */
  public RobotDefinition withDrives(Drive... added) {
    return RobotDefinitionBuilder.builder(this).drives(adding(drives, added)).build();
  }

  /**
   * The same robot with more sensors beyond each joint's own: beam breaks, contact switches and
   * rangefinders. To wire one of its sensors otherwise, {@link #with(Sensor...)}.
   */
  public RobotDefinition withSensors(Sensor... added) {
    return RobotDefinitionBuilder.builder(this).sensors(adding(sensors, added)).build();
  }

  /** The same robot with more poses it takes by name: "stowed", "amp", "feeding". */
  public RobotDefinition withPresets(Preset... added) {
    return RobotDefinitionBuilder.builder(this).presets(adding(presets, added)).build();
  }

  /** The same robot with more rules for what one joint may do by where another is. */
  public RobotDefinition withInterlocks(Interlock... added) {
    return RobotDefinitionBuilder.builder(this).interlocks(adding(interlocks, added)).build();
  }

  /** A list with more after what it has. */
  @SafeVarargs
  private static <T> List<T> adding(List<T> has, T... added) {
    List<T> all = new ArrayList<>(has);
    all.addAll(List.of(added));
    return all;
  }

  /**
   * Where the gyro is mounted, and whether its accelerometer has been checked: its drivetrain's
   * gyro's ({@code PIGEON_2.can(CANIVORE, 1).at(mount)}). A drivetrain not wired runs in the
   * simulator alone and has no Pigeon of its own, so the simulator stands one in beside the backup
   * IMU, where the robot has one, or as every gyro starts, {@link ImuMount#UNCHECKED}.
   */
  public ImuMount gyroMount() {
    return gyroMountOf(drivetrain, backupImu);
  }

  private static ImuMount gyroMountOf(Drivetrain drivetrain, Optional<ImuMount> backup) {
    if (drivetrain.wiring().isPresent()) {
      return drivetrain.wiring().get().gyro().mount();
    }
    return backup.orElse(ImuMount.UNCHECKED);
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
        if (JointParts.sensors(joint).contains(sensor)) {
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
                        + namesOfItsCameras()
                        + "; to add one, withCameras(...)"))
        .build();
  }

  /**
   * The same robot with more cameras: a bench robot's, {@code
   * DRIVING.named("vision-bench").withCameras(CAMERA_1, CAMERA_2)}. Each is added after the robot's
   * own, and a camera of a name it has already is refused once the robot is finished ({@link
   * #checked}), as two of one name are: to mount one of its cameras otherwise, {@link
   * #with(CameraConfig...)}. A camera on a joint may come before or after its joint ({@link
   * #withJoints}).
   */
  public RobotDefinition withCameras(CameraConfig... added) {
    List<CameraConfig> all = new ArrayList<>(cameras);
    all.addAll(List.of(added));
    return RobotDefinitionBuilder.builder(this).cameras(all).build();
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
   * CANcoder offsets its own, {@code COMPETITION.with(DRIVETRAIN.encoderOffsets(...))}, or its
   * devices its own, {@code
   * COMPETITION.with(DRIVETRAIN.gyro(...).frontLeft(...)...backRight(...))}. Its modules may differ
   * in what only the simulator reads, their inertias, which it takes too ({@link
   * Drivetrain#sameHardwareAs}).
   *
   * @throws IllegalArgumentException for a drivetrain that isn't this robot's: other modules, or
   *     placed otherwise
   */
  public RobotDefinition with(Drivetrain changed) {
    if (!changed.sameHardwareAs(drivetrain)) {
      throw new IllegalArgumentException(
          possessive(name)
              + " drivetrain isn't this one: their modules, or where they sit, differ. Give its"
              + " own, re-wired, .gyro(...).frontLeft(...)...backRight(...), or re-tuned,"
              + " .control(...)");
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
   * <p>Doubtful ({@link Finding.Level#DOUBT}): a drive kV under half or over twice the modules'
   * ideal ({@code Drivetrain.whyDriveKvIsDoubtful}); a camera's field of view more than 2° from its
   * lens's ({@code CameraConfig.whyFieldOfViewDisagreesWithLens}); a named speed above nine tenths
   * of what its joint's motors turn it at, which a velocity loop can't hold under load ({@code
   * RobotMath.freeSpeed}); one above it all is refused. A joint resting inside a region it's kept
   * out of as the robot starts, which it may only leave until the region opens; a preset there is
   * refused.
   *
   * <p>Not known ({@link Finding.Level#NOTE}): nothing here is wrong, only unknown. A value nobody
   * has is zero or the model's own stand-in, never a guess, and this says where. On the robot
   * ({@link Finding.Scope#ROBOT}):
   *
   * <ul>
   *   <li>the robot not weighed, taken at the mass given here, the season's most, or its inertia a
   *       box's: the drive's moves and feedforward read them;
   *   <li>the bumpers' thickness not given, the usual 3 in, or their height, which leaves the
   *       season's bumper zone unchecked;
   *   <li>no backup IMU, so a lost gyro leaves the heading to the wheels;
   *   <li>a camera's roll, noise scale or latency not given, or its mount not checked, in one note
   *       for the camera that also names its field of view or sight range where those are left out
   *       too;
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
   * measured; a link with no mass, or mass but no inertia; a camera's field of view or sight range
   * not given, where the camera leaves out nothing the robot reads.
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
                  + " .control(DrivetrainControl.defaults())"));
    }
    drivetrain.whyDriveKvIsDoubtful().ifPresent(why -> found.add(Finding.doubt("drivetrain", why)));
    for (CameraConfig camera : cameras) {
      camera
          .whyFieldOfViewDisagreesWithLens()
          .ifPresent(why -> found.add(Finding.doubt(camera.name(), why)));
    }
    found.addAll(Blocks.uncontrolled(math));
    found.addAll(JointSpeedChecks.doubts(math));
    found.addAll(InterlockChecks.startDoubts(math, everyInterlock()));
    return found;
  }

  /** Each joint's own drives, then the robot's. */
  private static List<Drive> everyDrive(List<Joint> joints, List<Drive> drives) {
    List<Drive> every = new ArrayList<>();
    joints.forEach(joint -> every.addAll(JointParts.drives(joint)));
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
   * A joint's own position of the name a position has, where the joint puts it. Code passes the
   * team's constant, and gets the position as the robot running the code has it: a practice robot
   * whose position was re-placed ({@code replaced}) gives its own.
   *
   * @throws IllegalArgumentException if the position is not the joint's own ({@link
   *     #whyNotOwn(Joint, Position)})
   */
  public Position own(Joint joint, Position position) {
    return JointNames.own(joint, position);
  }

  /**
   * A joint's own output of the name an output has, at the joint's value, or {@link
   * Output#STOPPED}. A practice robot whose output was re-valued ({@code replaced}) gives its own.
   *
   * @throws IllegalArgumentException if the output is not the joint's own ({@link #whyNotOwn(Joint,
   *     Output)})
   */
  public Output own(Joint joint, Output output) {
    return JointNames.own(joint, output);
  }

  /**
   * Why a position is not a joint's own, if it is not.
   *
   * <p>A position is the joint's own when the joint has a position of that name and kind, and the
   * one given is where the joint's is now, or where it was before it was re-placed ({@link
   * Joint#formerPositions}). That second case is the competition robot's constant, used on its
   * practice robot. Anything else is refused: another joint's position of the same name somewhere
   * else, saying where this joint's is, or a name the joint does not have, saying which it has.
   *
   * @return the refusal, in plain words, or empty when the position is the joint's own
   */
  public Optional<String> whyNotOwn(Joint joint, Position position) {
    return JointNames.whyNotOwn(joint, position);
  }

  /**
   * Why an output is not a joint's own, if it is not. {@link Output#STOPPED} is every joint's own.
   * Any other is judged as a position is ({@link #whyNotOwn(Joint, Position)}), against what the
   * joint's output of that name runs at now and what it ran at before it was re-valued ({@link
   * Joint#formerOutputs}).
   *
   * @return the refusal, in plain words, or empty when the output is the joint's own
   */
  public Optional<String> whyNotOwn(Joint joint, Output output) {
    return JointNames.whyNotOwn(joint, output);
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
   * gate on it is refused as a move there is, and an interlock or a preset on it is refused here,
   * as the definition is checked ({@link #checked}). The competition robot's constant is its
   * practice robot's own.
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
            JointParts.interlocks(joint)
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
    joints.forEach(joint -> every.addAll(JointParts.sensors(joint)));
    every.addAll(sensors);
    return every;
  }

  /**
   * Why cameras' names cannot all be theirs, if they cannot: two have the same name, or names that
   * differ only in case or punctuation. Those become the same id ({@link CameraConfig#id()}), so
   * "Front Left" and "front-left" are one camera to everything that reads ids.
   */
  /**
   * Why a camera can't be used, if one can't: it isn't placed or pointed. Every check of the
   * cameras reads where they are, so this is the first.
   */
  private static Optional<String> whyACameraIsNotMounted(List<CameraConfig> cameras) {
    for (CameraConfig camera : cameras) {
      Optional<String> whyNot = camera.whyNotMounted();
      if (whyNot.isPresent()) {
        return whyNot;
      }
    }
    return Optional.empty();
  }

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

  /** The backup IMU, if the robot has one and it's mounted outside the robot. */
  private static List<Finding> outside(Optional<ImuMount> backupImu, Body body) {
    if (backupImu.isEmpty()) {
      return List.of();
    }
    return outsideAt("the backup IMU", backupImu.get(), body);
  }

  /** An IMU, if it's mounted outside the robot. */
  private static List<Finding> outsideAt(String name, ImuMount imu, Body body) {
    if (body.contains(imu.position())) {
      return List.of();
    }
    return List.of(Finding.refused(name, name + " is outside the robot, at " + imu.position()));
  }
}
