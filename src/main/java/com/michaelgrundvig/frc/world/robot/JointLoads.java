package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;

/**
 * What a joint's motors have to work against, and what they have to work with: gravity's pull at
 * the joint, how hard the joint is to accelerate, which motors move it, and the ideal feedforward
 * gains that follow from those. {@link RobotMath} asks these on behalf of a joint; here a joint is
 * its number in the {@link JointTree}.
 *
 * <p>Everything is worked out by the principle of virtual work. Move one joint at one unit a second
 * and see how fast every mass and every motor shaft moves with it. A force's effect at the joint is
 * then the force times how fast the thing it acts on moves, in the force's direction. That gives
 * gravity's torque at a joint through any chain of joints and couplings without solving the chain,
 * and the same rates give the inertia the joint feels.
 *
 * <p>A result is a torque, in newton meters, for a joint that turns, and a force, in newtons, for
 * one that slides. Inertia is in kg·m² or in kg in the same way.
 */
final class JointLoads {
  /** Up, as gravity sees it with the robot level: along the robot's z axis. */
  private static final Translation3d ROBOT_UP = new Translation3d(0, 0, 1);

  /** A quarter of a turn, in radians. */
  private static final double QUARTER_TURN = Math.PI / 2;

  /** How many steps across a joint's range gravity is compared with its cosine at. */
  private static final int FIT_STEPS = 360;

  /** Below this, in newton meters, gravity is taken not to turn a joint at all. */
  private static final double NO_GRAVITY = 1e-12;

  private final JointTree tree;
  private final List<Drive> drives;

  JointLoads(JointTree tree, List<Drive> drives) {
    this.tree = tree;
    this.drives = List.copyOf(drives);
  }

  /**
   * What the joint must push with to hold still against gravity, with the robot level and the
   * joints at these positions. Positive pushes the joint forward.
   */
  double gravity(int joint, double[] positions) {
    return gravity(joint, positions, ROBOT_UP);
  }

  /**
   * What the joint must push with to hold still against gravity, with "up" in a direction of the
   * caller's choosing.
   *
   * <p>Each link weighs its mass times g. Holding it takes, at the joint, that weight times how
   * fast the link's center of mass rises for each unit of the joint's speed. The links are added up
   * in the joints' order.
   */
  private double gravity(int joint, double[] positions, Translation3d up) {
    Pose3d[] frames = tree.frames(positions);
    double[] rates = tree.rates(joint, positions);
    double total = 0;
    for (int number = 0; number < tree.size(); number++) {
      MassProperties link = tree.linkInRobotFrame(number, frames);
      Translation3d centerVelocity =
          tree.velocityOf(tree.joints().get(number), link.cg(), frames, rates);
      double rateOfRise = centerVelocity.dot(up);
      total += link.mass().in(Kilograms) * RobotMath.GRAVITY * rateOfRise;
    }
    return total;
  }

  /**
   * Gravity at a joint as its own feedforward sees it: with the joint at a position, every other
   * joint where it rests, and the part the joint is mounted on taken to be level, so that "up" is
   * along that part's z axis. This is what one gain, kG, can stand for, because it leaves out how
   * the joints below this one tilt it.
   */
  double gravityWithParentLevel(int joint, double position) {
    double[] positions = tree.positionsAtRestExcept(joint, position);
    Pose3d[] frames = tree.frames(positions);
    return gravity(joint, positions, tree.upForParentOf(joint, frames));
  }

  /**
   * Gravity's torque at a joint that turns, at the angle where it is greatest, with the joint's
   * parent level.
   *
   * <p>For one rigid load, gravity's torque is a cosine of the joint's angle: peak · cos(angle +
   * offset). Sampling it at 0 and at a quarter turn gives peak·cos(offset) and -peak·sin(offset),
   * and the peak is the hypotenuse of the two.
   */
  double peakGravity(int joint) {
    double atZero = gravityWithParentLevel(joint, 0);
    double atQuarterTurn = gravityWithParentLevel(joint, QUARTER_TURN);
    return Math.hypot(atZero, atQuarterTurn);
  }

  /**
   * What the ideal kG holds against: gravity at the joint with its parent level. A joint that turns
   * is held at the angle where gravity pulls hardest, because an arm's feedforward scales kG by the
   * cosine of the angle. A joint that slides is pulled the same all along, so where it rests does
   * as well as anywhere.
   */
  double gravityHeld(Joint joint, int number) {
    if (joint instanceof Joint.Revolute) {
      return peakGravity(number);
    }
    return gravityWithParentLevel(number, joint.restPosition());
  }

  /**
   * The offset of the cosine that gravity's torque follows at a joint that turns ({@link
   * #peakGravity}): the angle to add to the joint's position so that the cosine is at its peak at
   * zero. It is 0 for a joint gravity does not turn.
   */
  double gravityOffset(int joint) {
    double atZero = gravityWithParentLevel(joint, 0);
    double atQuarterTurn = gravityWithParentLevel(joint, QUARTER_TURN);
    if (Math.hypot(atZero, atQuarterTurn) < NO_GRAVITY) {
      return 0;
    }
    return -Math.atan2(atQuarterTurn, atZero);
  }

  /**
   * How well that one cosine fits gravity's real torque at a joint, from one angle to another. The
   * two are compared at evenly spaced angles, and the widest gap between them is reported as a
   * share of the cosine's peak.
   *
   * @param from the low end of the angles to compare over, in radians
   * @param to the high end
   */
  RobotMath.GravityFit cosineFit(int joint, double from, double to) {
    double offset = gravityOffset(joint);
    double peak = peakGravity(joint);
    double widestGap = 0;
    double worstAt = from;
    for (int step = 0; step <= FIT_STEPS; step++) {
      double angle = from + (to - from) * step / FIT_STEPS;
      double cosine = peak * Math.cos(angle + offset);
      double gap = Math.abs(cosine - gravityWithParentLevel(joint, angle));
      if (gap > widestGap) {
        widestGap = gap;
        worstAt = angle;
      }
    }
    return new RobotMath.GravityFit(offset, peak, misfit(widestGap, peak), worstAt);
  }

  /**
   * The widest gap as a share of the peak. With no gap at all the fit is exact, and that includes a
   * joint with no gravity on it. A gap with no peak to measure it against is infinitely bad.
   */
  static double misfit(double widestGap, double peak) {
    if (widestGap == 0) {
      return 0;
    }
    if (peak > 0) {
      return widestGap / peak;
    }
    return Double.POSITIVE_INFINITY;
  }

  /**
   * How hard a joint is to accelerate, with the joints at these positions: everything that moves
   * with it, as one inertia at the joint.
   *
   * <p>It is the kinetic energy of everything that moves, for one unit of the joint's speed, taken
   * twice (E = ½ I v², so I = 2E). Each link adds its mass times the square of its center's speed,
   * and its own moment of inertia times the square of how fast it turns. Each motor adds its
   * rotor's inertia times the square of how fast its shaft turns, which is the square of its
   * gearing.
   */
  double inertia(int joint, double[] positions) {
    Pose3d[] frames = tree.frames(positions);
    double[] rates = tree.rates(joint, positions);
    double inertia = 0;
    for (int number = 0; number < tree.size(); number++) {
      MassProperties link = tree.linkInRobotFrame(number, frames);
      Translation3d centerVelocity =
          tree.velocityOf(tree.joints().get(number), link.cg(), frames, rates);
      inertia += link.mass().in(Kilograms) * centerVelocity.getSquaredNorm();

      Translation3d spin = tree.spinOf(number, frames, rates);
      double turningRate = spin.getNorm();
      if (turningRate > 0) {
        Vector3 turningAxis = new Vector3(spin.getX(), spin.getY(), spin.getZ());
        inertia += turningRate * turningRate * link.inertia().about(turningAxis);
      }
    }
    for (Drive drive : drives) {
      if (drive.actuator() instanceof Actuator.Motors motors) {
        double shaftRate = shaftRate(drive, rates);
        double rotorInertia = motors.motor().rotorInertia().in(KilogramSquareMeters);
        inertia += motors.count() * rotorInertia * shaftRate * shaftRate;
      }
    }
    return inertia;
  }

  /**
   * The drives of motors that are meant to move a joint, given every joint's rate as that joint
   * moves. {@link RobotMath#drivesOf} says which those are and why.
   */
  List<Drive> motorsThatMove(double[] rates) {
    List<Drive> allThatTurn = new ArrayList<>();
    List<Drive> movingNothingElse = new ArrayList<>();
    for (Drive drive : drives) {
      boolean turnsAsTheJointMoves =
          drive.actuator() instanceof Actuator.Motors && shaftRate(drive, rates) != 0;
      if (!turnsAsTheJointMoves) {
        continue;
      }
      allThatTurn.add(drive);
      if (everyJointMoves(drive, rates)) {
        movingNothingElse.add(drive);
      }
    }
    if (movingNothingElse.isEmpty()) {
      return allThatTurn;
    }
    return movingNothingElse;
  }

  /** Whether every joint a drive turns is moving, at these rates. */
  private boolean everyJointMoves(Drive drive, double[] rates) {
    boolean everyJointMoves = true;
    for (Joint each : drive.transmission().joints()) {
      everyJointMoves &= rates[tree.numberOf(each)] != 0;
    }
    return everyJointMoves;
  }

  /**
   * How fast a drive's shaft turns, in radians a second, with the joints moving at these rates: the
   * sum, over the joints the drive turns, of the shaft's radians for each unit of that joint times
   * the joint's rate. A drive that pushes along a line between two parts has no shaft, and the
   * answer for it is 0.
   */
  double shaftRate(Drive drive, double[] rates) {
    if (drive.transmission() instanceof Transmission.Between) {
      return 0;
    }
    double shaftRate = 0;
    for (Joint each : drive.transmission().joints()) {
      shaftRate += drive.transmission().shaftRadiansPer(each) * rates[tree.numberOf(each)];
    }
    return shaftRate;
  }

  /**
   * What the motors that move a joint push it with for each ampere through each of them, with the
   * joints at rest: for each motor, its torque constant through the gearing, less what the gearing
   * loses. Newton meters, or newtons for a slide, for each ampere. Zero where no motors move the
   * joint.
   */
  double pushPerAmp(int number) {
    double[] atRest = tree.positions(Map.of());
    double[] rates = tree.rates(number, atRest);
    double pushPerAmp = 0;
    for (Drive drive : motorsThatMove(rates)) {
      Actuator.Motors motors = (Actuator.Motors) drive.actuator();
      DCMotor together = motors.model();
      double torqueConstant = together.stallTorque / together.stallCurrent;
      double efficiency = drive.transmission().efficiency();
      pushPerAmp +=
          motors.count() * torqueConstant * Math.abs(shaftRate(drive, rates)) * efficiency;
    }
    return pushPerAmp;
  }

  /**
   * A joint's ideal feedforward gains, from the motors that move it, with every joint where it
   * rests.
   *
   * <p>A DC motor's torque is proportional to the voltage left over once its back-EMF is taken off:
   * torque = (stall torque / nominal voltage) · (volts - speed · nominal voltage / free speed).
   * Through a gearing that turns the shaft g radians for each unit the joint moves, a volt becomes
   * |g| times that torque at the joint, less what the gearing loses. Adding that up over the motors
   * gives what one volt pushes the joint with. Each gain is then the volts needed for one thing:
   *
   * <ul>
   *   <li>kV, to cancel the back-EMF of the motors at one unit a second of the joint's speed;
   *   <li>kA, to accelerate the joint's inertia at one unit a second squared;
   *   <li>kG, to hold the joint against gravity.
   * </ul>
   *
   * @throws IllegalArgumentException if no motors move the joint
   */
  RobotMath.Gains idealGains(Joint joint, int number) {
    double[] atRest = tree.positions(Map.of());
    double[] rates = tree.rates(number, atRest);
    double pushPerVolt = 0;
    double backEmfPush = 0;
    for (Drive drive : motorsThatMove(rates)) {
      DCMotor motor = ((Actuator.Motors) drive.actuator()).model();
      double shaftRate = shaftRate(drive, rates);
      double torquePerVolt = motor.stallTorque / motor.nominalVoltage;
      double efficiency = drive.transmission().efficiency();
      // The datasheet's free speed is used as it is, as the drivetrain's ideal kV uses it, so the
      // motor's own friction (its free current) is already counted in it.
      double backEmfVoltsPerShaftSpeed = motor.nominalVoltage / motor.freeSpeed;
      pushPerVolt += Math.abs(shaftRate) * efficiency * torquePerVolt;
      backEmfPush += shaftRate * shaftRate * efficiency * torquePerVolt * backEmfVoltsPerShaftSpeed;
    }
    if (pushPerVolt == 0) {
      throw new IllegalArgumentException("no motors move " + joint.label());
    }
    double kV = backEmfPush / pushPerVolt;
    double kA = inertia(number, atRest) / pushPerVolt;
    double kG = gravityHeld(joint, number) / pushPerVolt;
    return new RobotMath.Gains(kV, kA, kG);
  }
}
