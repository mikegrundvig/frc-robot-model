package com.michaelgrundvig.frc.world.path;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.HolonomicTrajectory;

/**
 * One point of a path being planned: where the robot is to be at a time, and how it is to be
 * moving. A plan is worked out as a list of these, and then made into a trajectory.
 *
 * @param time seconds from the path's start
 * @param place where the robot is, in field coordinates
 * @param heading which way it faces, in radians, unwrapped
 * @param velocity how it is moving, on the field's axes
 */
record PathPoint(double time, Translation2d place, double heading, ChassisVelocities velocity) {
  /**
   * The points as a WPILib trajectory, field-relative. A trajectory's samples carry accelerations
   * as well. Each sample's is the steady acceleration that takes its velocity to the next sample's
   * in the time between them, which is how the trajectory's own interpolation reads it. The last
   * sample has no next, and its acceleration is zero.
   */
  static HolonomicTrajectory trajectory(List<PathPoint> points) {
    List<HolonomicSample> samples = new ArrayList<>(points.size());
    for (int index = 0; index < points.size(); index++) {
      PathPoint point = points.get(index);
      ChassisAccelerations acceleration = new ChassisAccelerations();
      boolean hasANext = index + 1 < points.size();
      if (hasANext) {
        acceleration = accelerationBetween(point, points.get(index + 1));
      }
      Pose2d pose = new Pose2d(point.place, new Rotation2d(point.heading));
      samples.add(new HolonomicSample(point.time, pose, point.velocity, acceleration));
    }
    return new HolonomicTrajectory(samples);
  }

  /** The steady acceleration that takes one point's velocity to the next point's. */
  private static ChassisAccelerations accelerationBetween(PathPoint point, PathPoint next) {
    double seconds = next.time - point.time;
    return new ChassisAccelerations(
        (next.velocity.vx - point.velocity.vx) / seconds,
        (next.velocity.vy - point.velocity.vy) / seconds,
        (next.velocity.omega - point.velocity.omega) / seconds);
  }
}
