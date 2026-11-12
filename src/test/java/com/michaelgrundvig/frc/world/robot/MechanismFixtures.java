package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Solid;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

/**
 * Synthetic mechanisms for the tests, one of each kind the audit's table lists, on the test robot's
 * body (a 28 x 24 in frame, 30 in tall, 60 kg). Numbers are chosen to check by hand.
 */
final class MechanismFixtures {
  private MechanismFixtures() {}

  static final Material ALUMINUM = Material.of("aluminum");

  /** Turning about this, a joint whose zero points forward raises it as its position grows. */
  static final Vector3 PITCH_UP = new Vector3(0, -1, 0);

  static final Vector3 UP = Vector3.UP;

  /**
   * A link of a mass centered at a point in its joint's frame, as hard to turn as a box of a size
   * there, and that box its shape.
   */
  static Link link(String label, double kilograms, Translation3d cg, Translation3d size) {
    return Link.of(
            label,
            MassProperties.of(
                Kilograms.of(kilograms), cg, Inertia.box(Kilograms.of(kilograms), size)))
        .shapes(List.of(Solid.Box.of(new Pose3d(cg, Rotation3d.ZERO), size, ALUMINUM)));
  }

  static RevoluteJoint revolute(
      String label,
      Part parent,
      Translation3d at,
      Vector3 axis,
      Optional<Range> range,
      double reference,
      Link link) {
    return RevoluteJoint.builder()
        .label(label)
        .parent(parent)
        .axis(axis)
        .link(link)
        .range(range)
        .rest(reference)
        .place(new Transform3d(at, Rotation3d.ZERO))
        .build();
  }

  /**
   * A joint as a test's fixture has it, its motors taken off for the test to give it others: a
   * block says its motors once, and a joint re-wired keeps its motor.
   */
  static RevoluteJoint unmotored(RevoluteJoint joint) {
    return JointCopies.driven(
        joint, DrivenBuilder.builder(joint.driven()).motors(Optional.empty()).build());
  }

  static PrismaticJoint prismatic(
      String label, Part parent, Translation3d at, Vector3 axis, Range range, Link link) {
    return PrismaticJoint.builder()
        .label(label)
        .parent(parent)
        .axis(axis)
        .link(link)
        .range(range)
        .rest(range.lower())
        .place(new Transform3d(at, Rotation3d.ZERO))
        .build();
  }

  /** The test robot with these mechanisms, and these cameras. */
  static RobotDefinition definition(
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors,
      List<CameraConfig> cameras) {
    return RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of()))
        .cameras(cameras)
        .joints(joints)
        .couplings(couplings)
        .drives(drives)
        .sensors(sensors)
        .build()
        .checked();
  }

  static RobotDefinition definition(
      List<Joint> joints, List<Coupling> couplings, List<Drive> drives, List<Sensor> sensors) {
    return definition(joints, couplings, drives, sensors, List.of());
  }

  static Gearing gearing(double ratio, double efficiency) {
    return Gearing.of(ratio).efficiency(efficiency);
  }

  /** An encoder on one joint: a CANcoder's 4096 counts a turn if absolute, none given if not. */
  static Sensor.Encoder encoder(String label, Joint joint, double ratio, boolean absolute) {
    return encoder(label, joint, ratio, absolute, absolute ? Optional.of(4096) : Optional.empty());
  }

  static Sensor.Encoder encoder(
      String label, Joint joint, double ratio, boolean absolute, Optional<Integer> countsPerTurn) {
    Sensor.Encoder encoder =
        Sensor.Encoder.of(label, List.of(Transmission.Term.of(joint, ratio)), absolute);
    return countsPerTurn.map(encoder::countsPerTurn).orElse(encoder);
  }

  static Coupling.Linear linear(Joint leader, Joint follower, double ratio) {
    return Coupling.Linear.builder()
        .leader(leader)
        .follower(follower)
        .ratio(ratio)
        .offset(0)
        .build();
  }

  static Translation3d point(double x, double y, double z) {
    return new Translation3d(x, y, z);
  }

  static double meters(org.wpilib.units.measure.Distance distance) {
    return distance.in(Meters);
  }

  /** A joint's motors, through a gearing, as a drive of the robot's own. */
  static Drive motors(RevoluteJoint joint, Motor motor, int count, Gearing gearing) {
    return Drive.of(
        joint.label(),
        Actuator.MotorGroup.of(motor, count),
        Transmission.Geared.of(joint, gearing));
  }

  /** A sliding joint's motors, through a gearing onto a drum, as a drive of the robot's own. */
  static Drive motors(
      PrismaticJoint joint, Motor motor, int count, Gearing gearing, Distance pitchRadius) {
    return Drive.of(
        joint.label(),
        Actuator.MotorGroup.of(motor, count),
        Transmission.Drum.of(joint, gearing, pitchRadius));
  }

  /**
   * What a robot's definition says about itself, its mass taken as a program on this season's field
   * takes it: as weighed, or the season's most.
   */
  static List<Finding> findings(RobotDefinition robot) {
    return robot.findings(Rebuilt.LIMITS.robotMass(robot.body()));
  }

  /** What a robot's definition says it doesn't know: its notes, each its message. */
  static List<String> notes(RobotDefinition robot) {
    return findings(robot).stream()
        .filter(finding -> finding.level() == Finding.Level.NOTE)
        .map(Finding::message)
        .toList();
  }

  /** What a robot's definition doubts, each its message. */
  static List<String> doubts(RobotDefinition robot) {
    return findings(robot).stream()
        .filter(finding -> finding.level() == Finding.Level.DOUBT)
        .map(Finding::message)
        .toList();
  }

  /** The scope of each note, in order. */
  static List<Finding.Scope> noteScopes(RobotDefinition robot) {
    return findings(robot).stream()
        .filter(finding -> finding.level() == Finding.Level.NOTE)
        .map(Finding::scope)
        .toList();
  }

  /**
   * A control for a joint held at positions with no absolute encoder: stowed by hand at power on,
   * its feedback {@code kP}.
   */
  static MechanismControl stowed(double kP) {
    return MechanismControl.position(kP).withHoming(MechanismControl.Homing.assumeAtRest());
  }
}
