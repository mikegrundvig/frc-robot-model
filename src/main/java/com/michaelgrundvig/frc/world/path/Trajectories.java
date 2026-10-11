package com.michaelgrundvig.frc.world.path;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectoryGenerator;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectoryParameterizer;
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.HolonomicTrajectory;
import org.wpilib.math.trajectory.TrajectoryConfig;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.constraint.CentripetalAccelerationConstraint;

/**
 * The timed paths the drivetrain's moves follow, worked out as a move starts, from where the robot
 * is and how it's moving, under what the move may ask of it: a straight line, or a smooth path
 * through keyframes. Each is a WPILib {@link HolonomicTrajectory}, field-relative, every sample's
 * acceleration the change to the next, as the trajectory's own interpolation takes it.
 */
public final class Trajectories {
  /**
   * What a move may ask of the robot.
   *
   * @param speed top speed, m/s
   * @param rate top turning rate, rad/s
   * @param acceleration how fast it speeds up, slows, and swerves in a curve, m/s²
   * @param angularAcceleration how fast it spins up, rad/s²
   */
  public record Limits(
      double speed, double rate, double acceleration, double angularAcceleration) {}

  /**
   * A timed path, and when it passes each keyframe.
   *
   * @param trajectory the path, field-relative
   * @param passing for each keyframe, the seconds from the path's start it passes it; none for a
   *     line
   */
  public record Plan(HolonomicTrajectory trajectory, double[] passing) {}

  /**
   * How far apart keyframes must be, meters: more than this. Any closer, and at this distance
   * exactly, a spline through them folds back on itself.
   */
  static final double APART_METERS = 0.05;

  /** How near a keyframe a path's sample must come to be where it passes it, meters. */
  private static final double PASSES_METERS = 0.001;

  private Trajectories() {}

  /**
   * Plans one short path, so the first move's plan, at the start of an auto, doesn't load WPILib's
   * spline and trajectory classes inside a robot loop. On a desktop, the first plan takes 65 ms,
   * the next 5 ms, and a plan with the code warm 0.3 ms.
   *
   * @param step the seconds between a trajectory's samples, as {@link #line} takes it
   */
  public static void warmUp(double step) {
    through(
        Pose2d.ZERO,
        new ChassisVelocities(),
        List.of(
            Keyframe.at(new Pose2d(1, 0, Rotation2d.ZERO)),
            Keyframe.at(new Pose2d(2, 1, Rotation2d.CCW_90DEG))),
        new Limits(4, 6, 2, 9),
        step);
  }

  /**
   * In a straight line to a place, turning to a heading on the way, the nearer way round: one
   * trapezoid profile along the line, and one for the heading, each from the robot's speed that way
   * as it starts. To the place it starts at, it turns where it stands.
   *
   * @param step the seconds between the trajectory's samples: a robot loop's
   */
  public static Plan line(
      Pose2d start,
      ChassisVelocities moving,
      Translation2d to,
      Rotation2d facing,
      Limits limits,
      double step) {
    TrapezoidProfile alongLine = profile(limits.speed, limits.acceleration);
    TrapezoidProfile turning = profile(limits.rate, limits.angularAcceleration);

    // Along the line, position is the distance gone from the start. The robot sets off at
    // whatever part of its velocity is already along the line, and arrives still.
    Translation2d toGoal = to.minus(start.getTranslation());
    Rotation2d direction = toGoal.getAngle().orElse(Rotation2d.ZERO);
    double speedAlongLine = moving.vx * direction.getCos() + moving.vy * direction.getSin();
    TrapezoidProfile.State settingOff = new TrapezoidProfile.State(0, speedAlongLine);
    TrapezoidProfile.State arrived = new TrapezoidProfile.State(toGoal.getNorm(), 0);

    // The heading is unwrapped: the goal is the start's heading plus the nearer way round to
    // the heading asked for, so the profile turns that way.
    double startHeading = start.getRotation().getRadians();
    double nearerWayRound = facing.minus(start.getRotation()).getRadians();
    TrapezoidProfile.State turningAtStart = new TrapezoidProfile.State(startHeading, moving.omega);
    TrapezoidProfile.State facingGoal =
        new TrapezoidProfile.State(startHeading + nearerWayRound, 0);

    // The path lasts as long as the slower of the two takes.
    double duration =
        Math.max(
            alongLine.timeLeftUntil(settingOff, arrived),
            turning.timeLeftUntil(turningAtStart, facingGoal));
    int steps = (int) Math.ceil(duration / step);
    List<PathPoint> points = new ArrayList<>();
    for (int index = 0; index <= steps; index++) {
      double time = index * step;
      TrapezoidProfile.State onLine = alongLine.calculate(time, settingOff, arrived);
      TrapezoidProfile.State heading = turning.calculate(time, turningAtStart, facingGoal);
      Translation2d place =
          start.getTranslation().plus(new Translation2d(onLine.position, direction));
      Translation2d velocity = new Translation2d(onLine.velocity, direction);
      points.add(
          new PathPoint(
              time,
              place,
              heading.position,
              new ChassisVelocities(velocity.getX(), velocity.getY(), heading.velocity)));
    }
    return new Plan(PathPoint.trajectory(points), new double[0]);
  }

  /** A trapezoid profile that holds a speed and an acceleration to their limits. */
  private static TrapezoidProfile profile(double topSpeed, double acceleration) {
    return new TrapezoidProfile(new TrapezoidProfile.Constraints(topSpeed, acceleration));
  }

  /**
   * A smooth path through keyframes, stopping at the last: the shape a clamped cubic spline through
   * their places, leaving toward the first and arriving from the one before the last; its timing
   * WPILib's, under the top speed, and the acceleration both along the path and across it in
   * curves. The robot turns to each keyframe's heading from the moment it has passed, and faced,
   * the one before, the nearer way round, on a trapezoid profile; if it hasn't faced the last when
   * it gets there, the path waits there while it does.
   *
   * @param keyframes each at a pose: a keyframe at a place is found for the alliance first
   * @param step the seconds between the trajectory's samples where it waits to face the last: a
   *     robot loop's
   * @throws IllegalArgumentException if no path goes through them: one too near the one before
   */
  public static Plan through(
      Pose2d start,
      ChassisVelocities moving,
      List<Keyframe> keyframes,
      Limits limits,
      double step) {
    List<Translation2d> places = placesKeptApart(start, keyframes);
    List<? extends HolonomicSample> path = splineThrough(start, moving, places, limits);
    double[] passingTimes = passingTimes(path, places);
    double[] headings = unwrappedHeadings(start, keyframes);

    TrapezoidProfile turning = profile(limits.rate, limits.angularAcceleration);
    double startHeading = start.getRotation().getRadians();
    HeadingAlongPath heading =
        new HeadingAlongPath(turning, passingTimes, headings, startHeading, moving.omega);

    // The spline's own samples say where the robot is and how it moves. Its heading and turning
    // rate at each are the heading's own.
    List<PathPoint> points = new ArrayList<>();
    for (HolonomicSample sample : path) {
      TrapezoidProfile.State facing = heading.at(sample.time);
      points.add(
          new PathPoint(
              sample.time,
              sample.pose.getTranslation(),
              facing.position,
              new ChassisVelocities(sample.velocity.vx, sample.velocity.vy, facing.velocity)));
    }

    // If the robot is still turning when the path ends, it waits at the last keyframe while it
    // finishes, a sample every step.
    int last = places.size() - 1;
    double pathEnds = passingTimes[last];
    double facesTheLast = heading.whenItFacesTheLast();
    for (double time = pathEnds + step; time < facesTheLast + step; time += step) {
      TrapezoidProfile.State facing = heading.at(time);
      points.add(
          new PathPoint(
              time,
              places.get(last),
              facing.position,
              new ChassisVelocities(0, 0, facing.velocity)));
    }
    return new Plan(PathPoint.trajectory(points), passingTimes);
  }

  /**
   * Where each keyframe is, having made sure that each is far enough from the one before it, and
   * the first from where the robot starts.
   *
   * @throws IllegalArgumentException naming the first keyframe that is too near
   */
  private static List<Translation2d> placesKeptApart(Pose2d start, List<Keyframe> keyframes) {
    List<Translation2d> places = new ArrayList<>();
    Translation2d before = start.getTranslation();
    for (Keyframe keyframe : keyframes) {
      Translation2d place = poseOf(keyframe).getTranslation();
      if (place.getDistance(before) <= APART_METERS) {
        String whatIsBefore = "the keyframe before";
        if (places.isEmpty()) {
          whatIsBefore = "where the robot starts";
        }
        throw new IllegalArgumentException(
            String.format(
                Locale.ROOT,
                "%s is within %.0f cm of %s",
                keyframe,
                APART_METERS * 100,
                whatIsBefore));
      }
      places.add(place);
      before = place;
    }
    return places;
  }

  /** A keyframe's pose: the path's keyframes are at poses, a place's already found. */
  private static Pose2d poseOf(Keyframe keyframe) {
    return Objects.requireNonNull(keyframe.pose(), "a place's keyframe is found before it is used");
  }

  /**
   * The timed samples of a spline from where the robot starts through each place in turn, stopping
   * at the last.
   *
   * <p>A spline needs to be told which way it sets off and which way it arrives. It sets off
   * straight toward the first place, and arrives straight from the place before the last, or from
   * the start if there is only one. It starts at the part of the robot's speed that is already that
   * way, but never backward and never above the top speed.
   *
   * @throws IllegalArgumentException if WPILib finds no path through them
   */
  private static List<? extends HolonomicSample> splineThrough(
      Pose2d start, ChassisVelocities moving, List<Translation2d> places, Limits limits) {
    int last = places.size() - 1;
    Rotation2d settingOff = places.get(0).minus(start.getTranslation()).getAngle().orElseThrow();
    Translation2d beforeTheLast = start.getTranslation();
    if (last > 0) {
      beforeTheLast = places.get(last - 1);
    }
    Rotation2d arriving = places.get(last).minus(beforeTheLast).getAngle().orElseThrow();
    double speedAlreadyThatWay = moving.vx * settingOff.getCos() + moving.vy * settingOff.getSin();
    TrajectoryConfig config =
        new TrajectoryConfig(limits.speed, limits.acceleration)
            .addConstraint(new CentripetalAccelerationConstraint(limits.acceleration))
            .setStartVelocity(Math.clamp(speedAlreadyThatWay, 0, limits.speed));
    List<? extends HolonomicSample> path;
    try {
      path =
          DrivetrainSplineTrajectoryGenerator.generate(
                  new Pose2d(start.getTranslation(), settingOff),
                  places.subList(0, last),
                  new Pose2d(places.get(last), arriving),
                  config)
              .getSamples();
    } catch (DrivetrainSplineTrajectoryParameterizer.TrajectoryGenerationException e) {
      throw new IllegalArgumentException("no path through its keyframes: " + e.getMessage(), e);
    }
    if (path.size() < 2) {
      throw new IllegalArgumentException("no path through its keyframes");
    }
    return path;
  }

  /**
   * For each place, the seconds from the path's start at which the path passes it. The places are
   * looked for in order, each from the sample where the one before was found. The last place is
   * where the path ends, so its time is the last sample's.
   */
  private static double[] passingTimes(
      List<? extends HolonomicSample> path, List<Translation2d> places) {
    double[] passingTimes = new double[places.size()];
    int sample = 0;
    for (int index = 0; index < places.size(); index++) {
      sample = sampleThatPasses(path, places.get(index), sample);
      passingTimes[index] = path.get(sample).time;
    }
    passingTimes[places.size() - 1] = path.get(path.size() - 1).time;
    return passingTimes;
  }

  /**
   * The index of the sample at which a path passes a place: the first sample, from the one given
   * on, that is within a millimeter of the place, or if none is that near, the nearest of them.
   */
  static int sampleThatPasses(List<? extends HolonomicSample> path, Translation2d place, int from) {
    int nearest = from;
    double nearestDistance = Double.POSITIVE_INFINITY;
    for (int index = from; index < path.size(); index++) {
      double distance = path.get(index).pose.getTranslation().getDistance(place);
      if (distance <= PASSES_METERS) {
        return index;
      }
      if (distance < nearestDistance) {
        nearestDistance = distance;
        nearest = index;
      }
    }
    return nearest;
  }

  /**
   * Each keyframe's heading in radians, unwrapped: each is the one before it plus the nearer way
   * round to the keyframe's heading, starting from the robot's heading as the path starts. So a
   * robot facing 170° that is asked for -170° is given 190°, and turns the 20° that takes.
   */
  private static double[] unwrappedHeadings(Pose2d start, List<Keyframe> keyframes) {
    double[] headings = new double[keyframes.size()];
    double heading = start.getRotation().getRadians();
    for (int index = 0; index < keyframes.size(); index++) {
      Rotation2d asked = poseOf(keyframes.get(index)).getRotation();
      heading += asked.minus(new Rotation2d(heading)).getRadians();
      headings[index] = heading;
    }
    return headings;
  }
}
