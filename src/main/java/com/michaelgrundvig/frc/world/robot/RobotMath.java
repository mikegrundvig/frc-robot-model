package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Shape;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * What can be worked out about a robot's mechanisms from their definition. There are two kinds of
 * answer. One is the numbers that robot code needs to drive a joint: Phoenix's ratios, where an
 * absolute encoder's reading should wrap, and the ideal gains that seed tuning. The other is the
 * geometry that the definition is checked against: where each link is, how far the robot reaches,
 * and what gravity does at each joint. Every answer is a function of the definition and of the
 * joints' positions alone.
 *
 * <p>Positions are given by joint, in the joint's own units, radians or meters. A joint that is not
 * given is where it rests. A joint that follows another through a coupling is wherever its leader
 * puts it, so a position cannot be given for it. Gravity pulls along the robot's -z: the robot is
 * taken to be level.
 *
 * <p>The ideal gains are never the gains. They seed a mechanism's feedforward and Phoenix's slots
 * until SysId measures the real ones, and they are what SysId's results are compared with.
 *
 * <p>This type is the way in, and says what each answer means. The working is in the types it hands
 * to: {@link JointTree} for where the joints are and how they move together, {@link JointLoads} for
 * gravity, inertia and the gains, and {@link AbsoluteEncoders} for what absolute encoders tell
 * apart. The definition's checks of the parts together are in {@link AssemblyChecks}.
 */
public final class RobotMath {
  /** Standard gravity, in m/s². */
  public static final double GRAVITY = 9.80665;

  /**
   * How far a camera on a link may sit from the link's shapes: on it, give or take a bracket and
   * shapes simplified from CAD; farther is a typo or the wrong joint.
   */
  static final double CAMERA_ON_LINK_METERS = 0.1;

  /**
   * How near an absolute encoder's wrap may come to its joint's range, as a share of the turn that
   * wraps: a fortieth, 9° of an encoder on the joint's shaft. Nearer, and backlash, a hard stop's
   * give, or a range measured a little short carries a reading across the wrap, which then reads a
   * turn away. The definition refuses a joint whose encoders wrap this near its range ({@code
   * RobotDefinition}), and the Phoenix helper notes a discontinuity point that does, a team's own
   * included ({@link #wrapMargin}).
   */
  public static final double WRAP_MARGIN = 1.0 / 40;

  /**
   * How near vertical, or how near level, a joint's axis must be to count as vertical or level:
   * within this of 1 or of 0, as the share of the axis that points up.
   */
  private static final double AXIS_TOLERANCE = 1e-9;

  /**
   * A joint's ideal feedforward gains: what its motors would need with nothing lost but what the
   * gearing says, and no friction. In the joint's units: radians for a revolute joint, meters for a
   * prismatic one.
   *
   * @param kV volts per unit per second: WPILib's and Phoenix's kV, the motors' nominal voltage
   *     over their free speed through the gearing
   * @param kA volts per unit per second squared, with every link it moves where it rests and the
   *     motors' rotors through their gearing
   * @param kG volts to hold it against gravity, from its links' mass, so 0 for links without it:
   *     for a revolute joint, at its worst, which {@code ArmFeedforward} scales by the cosine of
   *     its angle from horizontal ({@link #gravityOffset}), and how well that cosine fits is {@link
   *     #gravityFit}; for a prismatic one, everywhere
   */
  public record Gains(double kV, double kA, double kG) {}

  /**
   * How well one cosine fits gravity's torque at a revolute joint across its range: Phoenix's
   * {@code Arm_Cosine} and WPILib's {@code ArmFeedforward} take gravity as {@code kG cos(angle +
   * offset)}, which is exact for one link on a level parent, but not for links that move with the
   * joint along their own curves (a hopper floor a linkage swings at a fraction of the intake's
   * angle), nor for a joint whose parent tilts. Past a few percent, feed {@link #gravityTorque} at
   * the joint's angle instead.
   *
   * @param offset the cosine's offset, {@link #gravityOffset}: Phoenix's {@code
   *     GravityArmPositionOffset}
   * @param peak the cosine's amplitude, gravity's torque at its worst, in newton meters: what the
   *     ideal kG holds, through the motors ({@link Gains#kG})
   * @param misfit the widest gap between the cosine and gravity's real torque over the range, as a
   *     share of the peak: 0 when one cosine is gravity, and when there's no gravity at all
   * @param worstAt where in the range the gap is widest, in radians
   */
  public record GravityFit(double offset, double peak, double misfit, double worstAt) {}

  private final Body body;
  private final List<Drive> drives;
  private final List<Sensor> sensors;
  private final JointTree tree;
  private final JointLoads loads;

  private RobotMath(
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors) {
    this.body = body;
    this.drives = List.copyOf(drives);
    this.sensors = List.copyOf(sensors);
    tree = new JointTree(joints, couplings);
    loads = new JointLoads(tree, drives);
  }

  /**
   * A robot's mechanisms, from its parts: the joints, how they're coupled, and what drives and
   * senses them. A robot's definition gives its own ({@code RobotDefinition.math()}).
   *
   * @throws IllegalArgumentException if the parts don't make mechanisms: a label used twice, a
   *     joint or a part referred to that isn't among them, a joint following two others or itself
   */
  public static RobotMath of(
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors) {
    List<String> problems = StructureChecks.problems(joints, couplings, drives, sensors, List.of());
    if (!problems.isEmpty()) {
      throw new IllegalArgumentException(String.join("; ", problems));
    }
    return new RobotMath(body, joints, couplings, drives, sensors);
  }

  /**
   * A robot's mechanisms, if its parts make them. The definition's checks across its named places
   * read this once the parts' own checks have said what is wrong with them, and skip those checks
   * when the parts do not make mechanisms at all.
   */
  static Optional<RobotMath> whole(
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors) {
    boolean sound =
        StructureChecks.problems(joints, couplings, drives, sensors, List.of()).isEmpty();
    if (!sound) {
      return Optional.empty();
    }
    return Optional.of(new RobotMath(body, joints, couplings, drives, sensors));
  }

  /**
   * A robot's mechanisms, from parts that {@link StructureChecks} has already found sound, so that
   * they are not checked a second time.
   */
  static RobotMath ofSoundParts(
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors) {
    return new RobotMath(body, joints, couplings, drives, sensors);
  }

  /** The joints, in the order the definition lists them. */
  public List<Joint> joints() {
    return tree.joints();
  }

  /** Whether a joint is one of the robot's. */
  boolean has(Joint joint) {
    return tree.has(joint);
  }

  /**
   * The joint a joint follows through a coupling, or null for one that moves on its own.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's
   */
  public @Nullable Joint leaderOf(Joint joint) {
    return tree.leaderOf(joint).orElse(null);
  }

  /**
   * Whether a joint is turned by motors alone: it follows no other joint, and exactly one drive of
   * motors turns it. Its homing, a control loop that runs on its encoder, and the program's
   * mechanism for it are all worked out for a joint like this.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's
   */
  public boolean motorsAlone(Joint joint) {
    if (leaderOf(joint) != null) {
      return false;
    }
    // The drives that move a joint are only ever motors turning a shaft ({@link #drivesOf}), so
    // counting them is the whole question.
    return drivesOf(joint).size() == 1;
  }

  /** The drives. */
  public List<Drive> drives() {
    return drives;
  }

  /** The sensors. */
  public List<Sensor> sensors() {
    return sensors;
  }

  /**
   * The absolute encoder that reads a joint and nothing else, or null when the joint has none. It
   * may be written in the joint's block or listed among the robot's sensors. It is what tells the
   * joint where it is at power on. When there are several, this is the first listed.
   */
  public Sensor.@Nullable Encoder absoluteEncoder(Joint joint) {
    List<Sensor.Encoder> found = AbsoluteEncoders.readingOnly(joint, sensors);
    if (found.isEmpty()) {
      return null;
    }
    return found.get(0);
  }

  /**
   * A joint's limit switch at one end of its travel, or null when it has none there. It may be
   * written in the joint's block or listed among the robot's sensors. It is the switch that homing
   * to a switch drives the joint onto.
   */
  public Sensor.@Nullable Switch limitSwitch(Joint joint, Sensor.Switch.Limit.End end) {
    for (Sensor sensor : sensors) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.joint().equals(joint)
          && limit.end() == end) {
        return onSwitch;
      }
    }
    return null;
  }

  /**
   * Every joint that moves on its own, where it rests: the robot stowed, as it is weighed. The
   * joints that follow others are left out, because their positions are their leaders' to give.
   * That makes this map something every method that takes positions accepts as it is. {@link
   * #positions} with nothing given has every joint, followers included.
   */
  public Map<Joint, Double> rests() {
    Map<Joint, Double> rests = new LinkedHashMap<>();
    List<Joint> joints = tree.joints();
    for (int number = 0; number < joints.size(); number++) {
      if (!tree.follows(number)) {
        rests.put(joints.get(number), joints.get(number).restPosition());
      }
    }
    return rests;
  }

  /**
   * Every joint's position: the ones given, the rest where they rest, and each coupled joint where
   * its leader puts it.
   *
   * @throws IllegalArgumentException for a joint that isn't the robot's, or one that follows
   *     another, whose position is its leader's to give
   */
  public Map<Joint, Double> positions(Map<Joint, Double> given) {
    double[] byNumber = tree.positions(given);
    Map<Joint, Double> positions = new LinkedHashMap<>();
    List<Joint> joints = tree.joints();
    for (int number = 0; number < joints.size(); number++) {
      positions.put(joints.get(number), byNumber[number]);
    }
    return positions;
  }

  /**
   * Where a part's frame is, in the robot's frame. The chassis's frame is the robot's own. A
   * joint's frame is its parent's, moved to where the joint is mounted and then by the joint's
   * position.
   */
  public Pose3d pose(Part part, Map<Joint, Double> positions) {
    if (!(part instanceof Joint joint)) {
      return Pose3d.ZERO;
    }
    Pose3d[] frames = tree.frames(tree.positions(positions));
    return frames[tree.numberOf(joint)];
  }

  /**
   * Where a camera is and which way it looks, in the robot's frame: its mount, on its joint's link
   * where the joint's position puts it, or on the chassis.
   */
  public Pose3d cameraPose(CameraConfig camera, Map<Joint, Double> positions) {
    Pose3d mountedOn = camera.joint().map(joint -> pose(joint, positions)).orElse(Pose3d.ZERO);
    return mountedOn.transformBy(camera.robotToCamera());
  }

  /**
   * Where a switch or a rangefinder is, in the robot's frame: at its pose on its part, with the
   * part where its joint's position puts it. A sensor whose pose is not known is at its part's
   * origin.
   */
  public Pose3d sensorPose(Sensor.Placed sensor, Map<Joint, Double> positions) {
    Pose3d mountedOn = pose(sensor.part(), positions);
    if (sensor.pose().isEmpty()) {
      return mountedOn;
    }
    Pose3d onPart = sensor.pose().get();
    return mountedOn.transformBy(new Transform3d(onPart.getTranslation(), onPart.getRotation()));
  }

  /**
   * What the robot's mass is without its links: the body as weighed, less every link where its
   * joint rests. The simulator's chassis.
   *
   * @throws IllegalArgumentException if nothing's left, or what's left turns more easily than any
   *     body could
   */
  public MassProperties chassis() {
    MassProperties chassis = body.massProperties();
    Pose3d[] frames = tree.frames(tree.positions(Map.of()));
    for (int number = 0; number < tree.size(); number++) {
      chassis = chassis.minus(tree.linkInRobotFrame(number, frames));
    }
    return chassis;
  }

  /**
   * The torque (or force) a joint must make to hold still against gravity, at a set of positions:
   * the weight of everything it carries, through the tree and through every joint coupled to it.
   * Positive pushes the joint forward. This is the feedforward for a nested joint, such as a wrist
   * whose arm turns under it, which Phoenix's onboard gravity term cannot give.
   *
   * @return newton meters for a revolute joint, newtons for a prismatic one
   */
  public double gravityTorque(Joint joint, Map<Joint, Double> positions) {
    int number = tree.numberOfIndependent(joint);
    return loads.gravity(number, tree.positions(positions));
  }

  /**
   * How far a revolute joint's zero is from horizontal: the angle to add to its position to give
   * the angle {@code ArmFeedforward} takes, and Phoenix's {@code GravityArmPositionOffset}, with
   * the joint's parent level and every other joint where it rests. Zero for a joint gravity doesn't
   * turn, such as a turret, and for one whose links have no mass: its weight is then taken to pull
   * hardest at its zero, unless its control says where ({@code
   * MechanismControl.Gains.gravityAngle}).
   *
   * @throws IllegalArgumentException for a prismatic joint, which doesn't turn against gravity
   */
  public double gravityOffset(Joint joint) {
    return loads.gravityOffset(numberOfJointThatTurns(joint));
  }

  /**
   * The number of a joint that gravity can be a cosine for: one that moves on its own, and turns.
   *
   * @throws IllegalArgumentException for a joint that follows another, or one that slides
   */
  private int numberOfJointThatTurns(Joint joint) {
    int number = tree.numberOfIndependent(joint);
    if (!(joint instanceof Joint.Revolute)) {
      throw new IllegalArgumentException(
          joint.label() + " slides: only a revolute joint turns against gravity");
    }
    return number;
  }

  /**
   * How well one cosine, the ideal kG and the gravity offset, fits gravity at a revolute joint
   * across its range (a turn about where it rests, for one that turns freely), its parent level and
   * every other joint where it rests.
   *
   * @throws IllegalArgumentException for a prismatic joint, or one that follows another
   */
  public GravityFit gravityFit(Joint joint) {
    int number = numberOfJointThatTurns(joint);
    double from = joint.range().map(Range::lower).orElse(joint.restPosition() - Math.PI);
    double to = joint.range().map(Range::upper).orElse(joint.restPosition() + Math.PI);
    return loads.cosineFit(number, from, to);
  }

  /**
   * Whether gravity can move a joint, whatever it carries, with the joints where they rest. It can
   * turn a revolute joint whose axis is not vertical, and slide a prismatic one whose axis is not
   * level. It cannot move a turret or a spindexer, which turn about the vertical, or a slide that
   * runs level. This needs no mass: it says what a tuned kG may hold against.
   */
  public boolean gravityActsOn(Joint joint) {
    Vector3 axis = joint.axis().unit();
    Translation3d axisInRobotFrame =
        new Translation3d(axis.x(), axis.y(), axis.z())
            .rotateBy(pose(joint, Map.of()).getRotation());
    double verticalShareOfAxis = Math.abs(axisInRobotFrame.getZ());
    if (joint instanceof Joint.Revolute) {
      boolean axisIsOffVertical = verticalShareOfAxis < 1 - AXIS_TOLERANCE;
      return axisIsOffVertical;
    }
    boolean axisIsOffLevel = verticalShareOfAxis > AXIS_TOLERANCE;
    return axisIsOffLevel;
  }

  /**
   * The drives of motors meant to move a joint, which its seeds and free speed come from: those
   * whose every joint moves with it (the joint itself, or one that follows it), where any do;
   * otherwise every drive whose motors turn as it moves. A swerve module's steering is its steering
   * motor's, though the drive motor turns with it through the coupling (the drive motor's other
   * joint, the wheel, stays still as the module steers); a wrist chained through the shoulder's
   * axle leaves the shoulder to the shoulder's motors; and a differential's two axes are both
   * motors', since neither moves either alone.
   *
   * @throws IllegalArgumentException if it follows another joint
   */
  public List<Drive> drivesOf(Joint joint) {
    int number = tree.numberOfIndependent(joint);
    double[] atRest = tree.positions(Map.of());
    return loads.motorsThatMove(tree.rates(number, atRest));
  }

  /**
   * The joint's ideal gains from its motors ({@link #drivesOf}), their gearing and what it moves,
   * pushing together. They seed a mechanism's feedforward until SysId measures it; they're never
   * the gains.
   *
   * @throws IllegalArgumentException if no motors move it, or it follows another joint
   */
  public Gains idealGains(Joint joint) {
    return loads.idealGains(joint, tree.numberOfIndependent(joint));
  }

  /**
   * What the ideal kG holds against, through the joint's motors: gravity's torque at the joint from
   * its links' mass, at its worst for a revolute joint, everywhere for a prismatic one, with its
   * parent level. Zero for links without mass.
   *
   * @return newton meters for a revolute joint, newtons for a prismatic one
   * @throws IllegalArgumentException if it follows another joint
   */
  double gravityHeld(Joint joint) {
    return loads.gravityHeld(joint, tree.numberOfIndependent(joint));
  }

  /**
   * How fast the joint can move at its motors' nominal voltage ({@link #drivesOf}), with nothing on
   * it: the ceiling for its profile's cruise velocity, and for its named speeds. In its units per
   * second.
   *
   * @throws IllegalArgumentException if no motors move it, or it follows another joint
   */
  public double freeSpeed(Joint joint) {
    double lowestNominalVolts = Double.POSITIVE_INFINITY;
    for (Drive drive : drivesOf(joint)) {
      double nominalVolts = ((Actuator.Motors) drive.actuator()).model().nominalVoltage;
      lowestNominalVolts = Math.min(lowestNominalVolts, nominalVolts);
    }
    // kV is volts for each unit a second, so the volts there are, over kV, is the speed they hold.
    return lowestNominalVolts / idealGains(joint).kV();
  }

  /**
   * How hard the joint is to accelerate at a set of positions: everything it moves, through the
   * tree and its couplings, and every motor's rotor that turns with it, through its gearing.
   *
   * @return kg·m² for a revolute joint, kg for a prismatic one
   */
  public double inertia(Joint joint, Map<Joint, Double> positions) {
    return loads.inertia(tree.numberOfIndependent(joint), tree.positions(positions));
  }

  /**
   * A drive's coordinate at a set of positions: for a drive that pushes along a line between two
   * parts, the length of that line, in meters; for any other, its shaft's angle, in radians, which
   * is the sum over the joints it turns.
   */
  public double coordinate(Drive drive, Map<Joint, Double> positions) {
    double[] byNumber = tree.positions(positions);
    if (drive.transmission() instanceof Transmission.Between line) {
      Pose3d[] frames = tree.frames(byNumber);
      Translation3d endOnA = tree.pointOn(line.a(), line.onA(), frames);
      Translation3d endOnB = tree.pointOn(line.b(), line.onB(), frames);
      return endOnB.getDistance(endOnA);
    }
    double shaftAngle = 0;
    for (Joint each : drive.transmission().joints()) {
      shaftAngle += drive.transmission().shaftRadiansPer(each) * byNumber[tree.numberOf(each)];
    }
    return shaftAngle;
  }

  /**
   * How fast a drive's coordinate changes for a joint's speed, at a set of positions. For a gearing
   * that is its ratio. For a drive that pushes along a line it is the line's lever arm about the
   * joint, which changes as the joint turns.
   */
  public double leverage(Drive drive, Joint joint, Map<Joint, Double> positions) {
    int number = tree.numberOfIndependent(joint);
    double[] byNumber = tree.positions(positions);
    double[] rates = tree.rates(number, byNumber);
    if (!(drive.transmission() instanceof Transmission.Between line)) {
      return loads.shaftRate(drive, rates);
    }
    // A line grows as fast as its two ends move apart, measured along the line itself.
    Pose3d[] frames = tree.frames(byNumber);
    Translation3d endOnA = tree.pointOn(line.a(), line.onA(), frames);
    Translation3d endOnB = tree.pointOn(line.b(), line.onB(), frames);
    Translation3d alongTheLine = endOnB.minus(endOnA).div(endOnB.getDistance(endOnA));
    Translation3d velocityOfB = tree.velocityOf(line.b(), endOnB, frames, rates);
    Translation3d velocityOfA = tree.velocityOf(line.a(), endOnA, frames, rates);
    Translation3d movingApart = velocityOfB.minus(velocityOfA);
    return alongTheLine.dot(movingApart);
  }

  /**
   * The torque (or force) a drive makes at a joint, everything held still: a counterbalance's at an
   * angle, which its feedforward takes off the motors' work; a cylinder's, extending.
   *
   * @param command what the drive's told ({@link Actuator#forceOrTorque})
   */
  public double torque(Drive drive, Joint joint, Map<Joint, Double> positions, double command) {
    double atTheDrive = drive.actuator().forceOrTorque(command, coordinate(drive, positions), 0);
    return atTheDrive * leverage(drive, joint, positions) * drive.transmission().efficiency();
  }

  /**
   * Phoenix's {@code SensorToMechanismRatio} for an encoder that reads one joint: its turns for
   * each of its joint's turns, or for each meter of a prismatic joint, so the mechanism's position
   * reads in turns, or in meters. A drive's rotor is an encoder too ({@link Drive#rotor}).
   *
   * @throws IllegalStateException if it reads several joints: ask about one
   */
  public double sensorToMechanismRatio(Sensor.Encoder encoder) {
    return encoder.ratio(encoder.joint());
  }

  /**
   * Phoenix's {@code SensorToMechanismRatio} for an encoder reading several joints, in one of them:
   * a swerve drive motor's rotor in its wheel's turns, a differential's in its average axis.
   *
   * @throws IllegalArgumentException if it doesn't read the joint
   */
  public double sensorToMechanismRatio(Sensor.Encoder encoder, Joint joint) {
    double ratio = encoder.ratio(joint);
    if (ratio == 0) {
      throw new IllegalArgumentException(encoder.label() + " doesn't read " + joint.label());
    }
    return ratio;
  }

  /**
   * Phoenix's {@code RotorToSensorRatio} for a drive's motor fused with an encoder: the rotor's
   * turns for each of the encoder's.
   *
   * @throws IllegalArgumentException if the drive isn't motors, or doesn't turn the encoder's joint
   * @throws IllegalStateException if the encoder reads several joints
   */
  public double rotorToSensorRatio(Drive drive, Sensor.Encoder encoder) {
    if (!(drive.actuator() instanceof Actuator.Motors)) {
      throw new IllegalArgumentException(drive.label() + " has no rotor: it isn't motors");
    }
    Joint joint = encoder.joint();
    double rotorRadiansPerJointUnit = drive.transmission().shaftRadiansPer(joint);
    if (rotorRadiansPerJointUnit == 0) {
      throw new IllegalArgumentException(
          drive.label()
              + " doesn't turn "
              + joint.label()
              + ", which "
              + encoder.label()
              + " reads");
    }
    // The encoder's ratio is its turns for each turn of a joint that turns, or for each meter of
    // one that slides, so the rotor's is put in the same terms before the two are divided. Radians
    // of rotor for each radian of the joint are already turns for each turn.
    double rotorTurnsPerJointTurn = rotorRadiansPerJointUnit;
    if (!(joint instanceof Joint.Revolute)) {
      rotorTurnsPerJointTurn = rotorRadiansPerJointUnit / (2 * Math.PI);
    }
    return rotorTurnsPerJointTurn / encoder.ratio(joint);
  }

  /**
   * Phoenix's {@code AbsoluteSensorDiscontinuityPoint} for an absolute encoder, in turns: where its
   * reading wraps, put as far from the joint's range as it can be, with the range centered in the
   * encoder's turn. The encoder's offset, measured per install, makes it read 0 with the joint at
   * 0. A joint that turns freely is centered on where it rests.
   *
   * @throws IllegalArgumentException if it isn't absolute, or the joint's range is more of its turn
   *     than one turn can hold
   */
  public double discontinuityPoint(Sensor.Encoder encoder) {
    return AbsoluteEncoders.discontinuityPoint(encoder);
  }

  /**
   * How far inside an absolute encoder's turn its joint's range sits when the encoder wraps at a
   * discontinuity point, in the encoder's turns, at the nearer end: negative when the range crosses
   * the wrap, and too near below {@link #WRAP_MARGIN}. The encoder reads 0 with the joint at 0, and
   * grows as its ratio's sign says, as {@link #discontinuityPoint} takes it. Infinite for a joint
   * that turns freely, which has no range to keep clear.
   *
   * @throws IllegalStateException if it reads several joints
   */
  public static double wrapMargin(Sensor.Encoder encoder, double discontinuityPoint) {
    return AbsoluteEncoders.wrapMargin(encoder, discontinuityPoint);
  }

  /**
   * How far a joint can move with its absolute encoders still telling every position apart, as a
   * decoder reads them, at their resolution. In the joint's units, radians or meters.
   *
   * <p>One encoder tells its own turn apart, through its ratio. Two decode one of two ways:
   *
   * <ul>
   *   <li>Coarse then fine, when one turns at least twice as fast as the other: the coarse one
   *       picks which of the fine one's turns the joint is in, and the fine one says where in it.
   *       The span is the coarse one's alone; the fine one adds precision, never reach. Readings
   *       repeating together farther out don't count: one count off, or the backlash between them,
   *       picks the wrong turn.
   *   <li>A pair geared nearly alike, decoded by their difference (a vernier): it turns as slowly
   *       as their ratios differ, so it tells apart far more than either, as two encoders on 17 and
   *       18 teeth against a turret's ring tell more than three of its turns apart. Only if their
   *       resolution lets the difference pick the right turn of one of them: a count off on each
   *       must move it less than half that turn.
   * </ul>
   *
   * With more than two, the best one or pair is what a decoder would read.
   *
   * @throws IllegalArgumentException if there are none, they aren't all absolute and on one joint,
   *     or a pair geared nearly alike doesn't give each one's counts per turn
   */
  public double absoluteSpan(List<Sensor.Encoder> encoders) {
    return AbsoluteEncoders.decoding(encoders).span();
  }

  /**
   * Where a joint is from one absolute encoder's reading: in the joint's units, wrapped into the
   * window the encoder tells apart ({@link #absoluteWindow}). What a motor controller is told at
   * startup, from an encoder read once, as a turret's coarse one is.
   *
   * @param turns the encoder's reading in its turns, its offset applied so it reads zero with the
   *     joint at zero, and growing as the joint moves forward
   * @throws IllegalArgumentException as {@link #absoluteWindow} does
   */
  public double fromAbsolute(Sensor.Encoder encoder, double turns) {
    return AbsoluteEncoders.positionFor(encoder, turns);
  }

  /**
   * Where a decoder reading these absolute encoders has its window: the joint's positions it tells
   * apart, from lower to upper, in the joint's units, as wide as their span ({@link
   * #absoluteSpan}). It wraps everything it reads into this window. For one encoder, its
   * discontinuity point puts it ({@link #discontinuityPoint}): its turn centered on the joint's
   * range, as far as Phoenix's point, from 0 to 1, can go. For a pair, centered on the range,
   * wherever that is: where their difference wraps is the decoder's own to choose. A joint that
   * turns freely is centered on where it rests.
   *
   * @throws IllegalArgumentException as {@link #absoluteSpan} does
   */
  public Range absoluteWindow(List<Sensor.Encoder> encoders) {
    return AbsoluteEncoders.window(encoders);
  }

  /**
   * How far the robot's links reach beyond its frame perimeter at a set of positions, seen from
   * above: what the game's extension limit is checked against. Zero within it.
   */
  public double extension(Map<Joint, Double> positions) {
    double halfLength = body.frameLength().in(Meters) / 2;
    double halfWidth = body.frameWidth().in(Meters) / 2;
    double farthest = 0;
    for (Translation3d point : linkPoints(positions)) {
      double pastTheFrontOrBack = Math.max(0, Math.abs(point.getX()) - halfLength);
      double pastASide = Math.max(0, Math.abs(point.getY()) - halfWidth);
      farthest = Math.max(farthest, Math.hypot(pastTheFrontOrBack, pastASide));
    }
    return farthest;
  }

  /**
   * The highest any link reaches at a set of positions, above the floor; zero without links. The
   * starting height limit is checked against it with the joints where they rest, and against the
   * body's height.
   */
  public double top(Map<Joint, Double> positions) {
    double top = 0;
    for (Translation3d point : linkPoints(positions)) {
      top = Math.max(top, point.getZ());
    }
    return top;
  }

  /** Every point of every link's shapes, in the robot's frame. */
  private List<Translation3d> linkPoints(Map<Joint, Double> positions) {
    Pose3d[] frames = tree.frames(tree.positions(positions));
    List<Translation3d> points = new ArrayList<>();
    List<Joint> joints = tree.joints();
    for (int number = 0; number < joints.size(); number++) {
      Pose3d frame = frames[number];
      for (Shape shape : joints.get(number).link().shapes()) {
        for (Translation3d point : shape.points()) {
          points.add(frame.getTranslation().plus(point.rotateBy(frame.getRotation())));
        }
      }
    }
    return points;
  }

  /**
   * What is wrong with what the definition's parts make together. The list is empty when they make
   * one robot ({@link AssemblyChecks}).
   *
   * @param robot the robot's name, which a problem about the whole of it is reported against
   */
  static List<Finding> problems(
      String robot,
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors,
      List<CameraConfig> cameras) {
    return AssemblyChecks.problems(robot, body, joints, couplings, drives, sensors, cameras);
  }

  /** What the links weigh together, in kilograms. A link whose mass is not given weighs nothing. */
  double linksKilograms() {
    double total = 0;
    for (Joint joint : tree.joints()) {
      total += joint.link().mass().mass().in(Kilograms);
    }
    return total;
  }
}
