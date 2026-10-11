package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The robot's joints as a tree, together with the couplings that make some joints follow others. It
 * answers the geometric questions the rest of the robot's maths is built from: where every joint is
 * once each follower is put where its leader takes it, where each joint's frame is in the robot's
 * frame, and how fast each joint, and any point riding on one, moves when a single joint moves.
 *
 * <p>Each joint has a number, its place in the order the definition lists the joints. The working
 * values of a calculation are plain arrays in that order: a position for every joint, a frame for
 * every joint, or a rate for every joint. An array is made for one calculation, handed from one
 * method here to the next, and never kept. Everything that adds up over the joints does so in that
 * same order, so a result is the same every time it is worked out.
 *
 * <p>A joint's "rate" is how fast it moves for each unit of speed of the one joint a calculation is
 * about: 1 for that joint itself, the slope of its coupling for a joint that follows it, and 0 for
 * every joint that does not move with it.
 */
final class JointTree {
  /**
   * Stands where a joint's number would be when there is no joint: for the parent of a joint that
   * is on the chassis, and for the leader of a joint that follows none.
   */
  private static final int NONE = -1;

  private final List<Joint> joints;
  private final Map<Joint, Integer> numbers = new HashMap<>();

  /** Each joint's parent, by number, or {@link #NONE} for a joint on the chassis. */
  private final int[] parent;

  /** The joint each joint follows, by number, or {@link #NONE} for one that moves on its own. */
  private final int[] leader;

  /** The coupling each joint follows its leader through, or null for one that moves on its own. */
  private final @Nullable Coupling[] coupling;

  /**
   * @param joints the robot's joints, in the definition's order. Every parent, leader and follower
   *     must be among them, which {@link StructureChecks} has checked before a tree is made.
   * @param couplings which joints follow which
   */
  JointTree(List<Joint> joints, List<Coupling> couplings) {
    this.joints = List.copyOf(joints);
    for (int number = 0; number < this.joints.size(); number++) {
      numbers.put(this.joints.get(number), number);
    }
    int count = this.joints.size();
    parent = new int[count];
    leader = new int[count];
    coupling = new Coupling[count];
    for (int number = 0; number < count; number++) {
      parent[number] = numberOfParent(this.joints.get(number));
      leader[number] = NONE;
    }
    for (Coupling each : couplings) {
      int follower = numberOf(each.follower());
      leader[follower] = numberOf(each.leader());
      coupling[follower] = each;
    }
  }

  private int numberOfParent(Joint joint) {
    if (joint.parent() instanceof Joint parentJoint) {
      return numberOf(parentJoint);
    }
    return NONE;
  }

  /** The joints, in the order the definition lists them, which is the order of their numbers. */
  List<Joint> joints() {
    return joints;
  }

  /** How many joints there are. */
  int size() {
    return joints.size();
  }

  /** Whether a joint is one of the robot's. */
  boolean has(Joint joint) {
    return numbers.containsKey(joint);
  }

  /**
   * A joint's number.
   *
   * @throws IllegalArgumentException for a joint that is not one of the robot's
   */
  int numberOf(Joint joint) {
    Integer number = numbers.get(joint);
    if (number == null) {
      throw new IllegalArgumentException(joint.label() + " isn't one of the robot's joints");
    }
    return number;
  }

  /**
   * The number of a joint that moves on its own. Most questions can only be asked of such a joint,
   * because a joint that follows another has no position, speed or motors of its own to ask about.
   *
   * @throws IllegalArgumentException for a joint that is not one of the robot's, or one that
   *     follows another, naming the joint to ask about instead
   */
  int numberOfIndependent(Joint joint) {
    int number = numberOf(joint);
    if (!follows(number)) {
      return number;
    }
    String leaderLabel = joints.get(leader[number]).label();
    throw new IllegalArgumentException(
        joint.label() + " follows " + leaderLabel + ": ask about " + leaderLabel);
  }

  /** Whether a joint follows another through a coupling. */
  boolean follows(int number) {
    return leader[number] != NONE;
  }

  /**
   * The joint that a joint follows through a coupling, if it follows one.
   *
   * @throws IllegalArgumentException for a joint that is not one of the robot's
   */
  Optional<Joint> leaderOf(Joint joint) {
    int number = numberOf(joint);
    if (!follows(number)) {
      return Optional.empty();
    }
    return Optional.of(joints.get(leader[number]));
  }

  /**
   * Every joint's position, by number: the positions given, every other joint where it rests, and
   * every joint that follows another where its leader puts it.
   *
   * @param given positions for some of the joints that move on their own
   * @throws IllegalArgumentException for a joint that is not one of the robot's, for one that
   *     follows another (its position is its leader's to give), or for a position that is not a
   *     number
   */
  double[] positions(Map<Joint, Double> given) {
    double[] positions = new double[joints.size()];
    for (int number = 0; number < positions.length; number++) {
      positions[number] = joints.get(number).restPosition();
    }
    for (Map.Entry<Joint, Double> entry : given.entrySet()) {
      int number = numberOfIndependent(entry.getKey());
      if (!Double.isFinite(entry.getValue())) {
        throw new IllegalArgumentException(
            possessive(entry.getKey().label()) + " position must be a number: " + entry.getValue());
      }
      positions[number] = entry.getValue();
    }
    putFollowersWhereTheirLeadersTakeThem(positions);
    return positions;
  }

  /**
   * Every joint's position with one joint put somewhere and every other joint where it rests. The
   * joints that follow the one that was moved go with it.
   */
  double[] positionsAtRestExcept(int number, double position) {
    double[] positions = positions(Map.of());
    positions[number] = position;
    putFollowersWhereTheirLeadersTakeThem(positions);
    return positions;
  }

  private void putFollowersWhereTheirLeadersTakeThem(double[] positions) {
    boolean[] placed = new boolean[positions.length];
    for (int number = 0; number < positions.length; number++) {
      follow(number, positions, placed);
    }
  }

  /**
   * Puts one joint where its leader takes it, and returns where that is. Its leader may itself
   * follow another joint, so the leader is placed first. A joint that moves on its own is left
   * where it is.
   *
   * @param placed which followers have been placed already, so that each is worked out once
   */
  private double follow(int number, double[] positions, boolean[] placed) {
    Coupling followsThrough = coupling[number];
    if (followsThrough == null || placed[number]) {
      return positions[number];
    }
    double leaderPosition = follow(leader[number], positions, placed);
    positions[number] = followsThrough.followerAt(leaderPosition);
    placed[number] = true;
    return positions[number];
  }

  /** Each joint's frame in the robot's frame, by number, with the joints at these positions. */
  Pose3d[] frames(double[] positions) {
    Pose3d[] frames = new Pose3d[joints.size()];
    for (int number = 0; number < frames.length; number++) {
      frame(number, positions, frames);
    }
    return frames;
  }

  /**
   * One joint's frame: its parent's frame, moved to where the joint is mounted on its parent, and
   * then moved by the joint's own position. The parent's frame is worked out first if it has not
   * been yet.
   *
   * @param frames the frames worked out so far, which this adds to
   */
  private Pose3d frame(int number, double[] positions, @Nullable Pose3d[] frames) {
    Pose3d alreadyWorkedOut = frames[number];
    if (alreadyWorkedOut != null) {
      return alreadyWorkedOut;
    }
    Joint joint = joints.get(number);
    Pose3d parentFrame = Pose3d.ZERO;
    if (parent[number] != NONE) {
      parentFrame = frame(parent[number], positions, frames);
    }
    Pose3d frame =
        parentFrame.transformBy(joint.origin()).transformBy(joint.motion(positions[number]));
    frames[number] = frame;
    return frame;
  }

  /**
   * A point given on a part, in the robot's frame. A point on the chassis is in the robot's frame
   * already.
   *
   * @param onPart the point, in the part's own frame
   */
  Translation3d pointOn(Part part, Translation3d onPart, Pose3d[] frames) {
    if (!(part instanceof Joint joint)) {
      return onPart;
    }
    Pose3d frame = frames[numberOf(joint)];
    return frame.getTranslation().plus(onPart.rotateBy(frame.getRotation()));
  }

  /** The direction of a joint's axis in the robot's frame, as a vector one unit long. */
  Translation3d axis(int number, Pose3d[] frames) {
    Vector3 axis = joints.get(number).axis().unit();
    return new Translation3d(axis.x(), axis.y(), axis.z()).rotateBy(frames[number].getRotation());
  }

  /**
   * Which way is up for the part a joint is mounted on, in the robot's frame: the z axis of its
   * parent's frame. For a joint on the chassis that is the robot's own z axis.
   */
  Translation3d upForParentOf(int number, Pose3d[] frames) {
    Translation3d up = new Translation3d(0, 0, 1);
    if (parent[number] == NONE) {
      return up;
    }
    return up.rotateBy(frames[parent[number]].getRotation());
  }

  /**
   * How fast every joint moves when one joint moves at one unit a second, by number: 1 for the
   * joint itself, the slope of its coupling for a joint that follows it, and 0 for the rest.
   *
   * @param moving the number of the joint that moves
   */
  double[] rates(int moving, double[] positions) {
    double[] rates = new double[joints.size()];
    boolean[] known = new boolean[rates.length];
    for (int number = 0; number < rates.length; number++) {
      rate(number, moving, positions, rates, known);
    }
    return rates;
  }

  /**
   * One joint's rate, worked out once and remembered in the array.
   *
   * @param known which joints' rates are in the array already
   */
  private double rate(int number, int moving, double[] positions, double[] rates, boolean[] known) {
    if (!known[number]) {
      rates[number] = rateFromItsLeader(number, moving, positions, rates, known);
      known[number] = true;
    }
    return rates[number];
  }

  /**
   * A follower moves as fast as its leader does, times the slope of its coupling where the leader
   * is. Its leader may itself follow another joint, so the slopes multiply down the chain.
   */
  private double rateFromItsLeader(
      int number, int moving, double[] positions, double[] rates, boolean[] known) {
    if (number == moving) {
      return 1;
    }
    Coupling followsThrough = coupling[number];
    if (followsThrough == null) {
      return 0;
    }
    int leaderNumber = leader[number];
    return followsThrough.slopeAt(positions[leaderNumber])
        * rate(leaderNumber, moving, positions, rates, known);
  }

  /**
   * How fast a point riding on a part moves, in the robot's frame, with the joints moving at these
   * rates. Every moving joint between the part and the chassis adds to it. A joint that turns
   * carries the point round its axis, at the cross product of the axis and the point's offset from
   * it (v = ω × r). A joint that slides carries the point along its axis.
   *
   * @param point where the point is, in the robot's frame
   */
  Translation3d velocityOf(Part part, Translation3d point, Pose3d[] frames, double[] rates) {
    Translation3d velocity = Translation3d.ZERO;
    int nearest = part instanceof Joint joint ? numberOf(joint) : NONE;
    for (int number = nearest; number != NONE; number = parent[number]) {
      if (rates[number] == 0) {
        continue;
      }
      Translation3d perUnitOfRate = carriedBy(number, point, frames);
      velocity = velocity.plus(perUnitOfRate.times(rates[number]));
    }
    return velocity;
  }

  /** How fast one joint carries a point for each unit of the joint's own speed. */
  private Translation3d carriedBy(int number, Translation3d point, Pose3d[] frames) {
    Translation3d axis = axis(number, frames);
    if (joints.get(number) instanceof Joint.Revolute) {
      Translation3d fromAxis = point.minus(frames[number].getTranslation());
      return cross(axis, fromAxis);
    }
    return axis;
  }

  /**
   * How fast a joint's link turns, in the robot's frame, as an angular velocity vector, with the
   * joints moving at these rates. Every turning joint between the link and the chassis adds its
   * axis times its rate. A joint that slides turns nothing.
   */
  Translation3d spinOf(int link, Pose3d[] frames, double[] rates) {
    Translation3d spin = Translation3d.ZERO;
    for (int number = link; number != NONE; number = parent[number]) {
      if (rates[number] != 0 && joints.get(number) instanceof Joint.Revolute) {
        spin = spin.plus(axis(number, frames).times(rates[number]));
      }
    }
    return spin;
  }

  /** The mass, center and inertia of a joint's link, in the robot's frame. */
  MassProperties linkInRobotFrame(int number, Pose3d[] frames) {
    Pose3d frame = frames[number];
    return joints
        .get(number)
        .link()
        .mass()
        .transformedBy(new Transform3d(frame.getTranslation(), frame.getRotation()));
  }

  private static Translation3d cross(Translation3d a, Translation3d b) {
    return new Translation3d(
        a.getY() * b.getZ() - a.getZ() * b.getY(),
        a.getZ() * b.getX() - a.getX() * b.getZ(),
        a.getX() * b.getY() - a.getY() * b.getX());
  }
}
