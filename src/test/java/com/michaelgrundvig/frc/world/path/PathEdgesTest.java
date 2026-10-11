package com.michaelgrundvig.frc.world.path;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * The pieces a plan is made of, as tables of questions and answers: when the robot turns toward
 * each keyframe's heading, which sample of a path passes a place, what a path starts from when the
 * robot is already moving, and how points become a trajectory. Whole plans are in {@link
 * TrajectoriesTest}.
 */
class PathEdgesTest {
  private static final Trajectories.Limits LIMITS =
      new Trajectories.Limits(4.4, 2 * Math.PI, 2.3, 9.2);

  private static final double STEP = 0.02;

  @ParameterizedTest
  @MethodSource({"headings", "passings", "starts", "waits", "points"})
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments answers(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  /** A number to the nearest millionth, so that a profile's last rounding is not the answer. */
  private static double rounded(double value) {
    return Math.round(value * 1e6) / 1e6;
  }

  /**
   * A robot that starts facing 0, still, and turns at up to 1 rad/s, spinning up at 1 rad/s². Each
   * keyframe's heading is 1 rad past the one before, so each turn takes exactly 2 s: a second
   * speeding up and a second slowing, half a radian each.
   *
   * @param passingTimes when the path passes each keyframe
   */
  private static HeadingAlongPath turning(double... passingTimes) {
    double[] headings = new double[passingTimes.length];
    for (int index = 0; index < headings.length; index++) {
      headings[index] = index + 1;
    }
    TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 1));
    return new HeadingAlongPath(profile, passingTimes, headings, 0, 0);
  }

  // spotless:off

  /** Which keyframe's heading the robot is turning toward, and where that has got it. */
  static Stream<Arguments> headings() {
    return Stream.of(
        answers("halfway through its first turn it has turned half of it",
            () -> rounded(turning(1, 9).at(1).position), 0.5),
        answers("it starts its second turn when it has faced the first, though the path passed it earlier",
            () -> rounded(turning(1, 9).at(3).position), 1.5),
        answers("it holds a heading it has faced until the path passes that keyframe",
            () -> rounded(turning(4, 9).at(3.9).position), 1.0),
        answers("...and starts its next turn as the path does",
            () -> rounded(turning(4, 9).at(5).position), 1.5),
        answers("a third turn starts when the second is done, timed from when the second began",
            () -> rounded(turning(1, 2.5, 9).at(5).position), 2.5),
        answers("it never turns past the last keyframe's heading",
            () -> rounded(turning(1, 2.5, 9).at(60).position), 3.0),
        answers("it faces the last heading before the path ends, when the path is long",
            () -> rounded(turning(1, 2.5, 9).whenItFacesTheLast()), 6.0),
        answers("...and at the same time when the path is short, which then waits for it",
            () -> rounded(turning(1, 2.5, 3).whenItFacesTheLast()), 6.0),
        answers("with one keyframe, it faces it when its only turn is done",
            () -> rounded(turning(0.5).whenItFacesTheLast()), 2.0));
  }

  /** A path that is nothing but places, one sample each. */
  private static List<HolonomicSample> pathThrough(double... xy) {
    List<HolonomicSample> samples = new ArrayList<>();
    for (int index = 0; index < xy.length; index += 2) {
      Pose2d pose = new Pose2d(xy[index], xy[index + 1], Rotation2d.ZERO);
      samples.add(new HolonomicSample(index / 2, pose, new ChassisVelocities(), new ChassisAccelerations()));
    }
    return samples;
  }

  private static int passes(List<HolonomicSample> path, double x, double y, int from) {
    return Trajectories.sampleThatPasses(path, new Translation2d(x, y), from);
  }

  /** Which sample of a path passes a place. */
  static Stream<Arguments> passings() {
    return Stream.of(
        answers("the sample at the place", () -> passes(pathThrough(0, 0, 1, 0, 2, 0), 1, 0, 0), 1),
        answers("a sample exactly a millimeter from the place passes it",
            () -> passes(pathThrough(5, 0, 0.001, 0, 0, 0), 0, 0, 0), 1),
        answers("a sample a hair further does not, when a later one is nearer",
            () -> passes(pathThrough(5, 0, 0.0010001, 0, 0, 0), 0, 0, 0), 2),
        answers("of two samples within a millimeter, the first", () -> passes(pathThrough(1, 0, 1, 0, 1, 0), 1, 0, 0), 0),
        answers("with none that near, the nearest", () -> passes(pathThrough(0, 0, 0.8, 0, 3, 0), 1, 0, 0), 1),
        answers("of two equally near, the first", () -> passes(pathThrough(0, 0, 0.5, 0, 1.5, 0), 1, 0, 0), 1),
        answers("samples before the one to look from are not looked at",
            () -> passes(pathThrough(1, 0, 5, 0, 6, 0), 1, 0, 1), 1),
        answers("with nothing left to look at, the one to look from", () -> passes(pathThrough(0, 0, 1, 0), 9, 9, 1), 1));
  }

  private static Keyframe at(double x, double y) {
    return Keyframe.at(new Pose2d(x, y, Rotation2d.ZERO));
  }

  /** Whether a path to one keyframe is refused for the keyframe being too near the start. */
  private static boolean tooNear(Keyframe keyframe) {
    try {
      Trajectories.through(Pose2d.ZERO, new ChassisVelocities(), List.of(keyframe), LIMITS, STEP);
      return false;
    } catch (IllegalArgumentException e) {
      return String.valueOf(e.getMessage()).contains("is within 5 cm of where the robot starts");
    }
  }

  private static double startSpeed(Trajectories.Plan plan) {
    ChassisVelocities start = plan.trajectory().start().velocity;
    return rounded(Math.hypot(start.vx, start.vy));
  }

  /**
   * What a path starts from: the part of the robot's velocity that is along the way the path sets
   * off. Toward (3, 4), a robot moving at (1, 2) m/s has 1 x 0.6 + 2 x 0.8 = 2.2 m/s that way.
   */
  static Stream<Arguments> starts() {
    ChassisVelocities moving = new ChassisVelocities(1, 2, 0);
    ChassisVelocities backing = new ChassisVelocities(-1, -2, 0);
    return Stream.of(
        answers("a line starts at the robot's speed along it",
            () -> startSpeed(Trajectories.line(Pose2d.ZERO, moving, new Translation2d(3, 4), Rotation2d.ZERO, LIMITS, STEP)),
            2.2),
        answers("a path through keyframes starts at the robot's speed toward the first",
            () -> startSpeed(Trajectories.through(Pose2d.ZERO, moving, List.of(at(3, 4)), LIMITS, STEP)), 2.2),
        answers("...but from rest, if the robot is moving away from it",
            () -> startSpeed(Trajectories.through(Pose2d.ZERO, backing, List.of(at(3, 4)), LIMITS, STEP)), 0.0),
        answers("a keyframe exactly 5 cm from the start is not refused as too near it",
            () -> tooNear(at(0.05, 0)), false),
        answers("a keyframe a hair nearer is", () -> tooNear(at(0.0499999, 0)), true));
  }

  /** A hop of 0.3 m with a half turn: the path is there long before the robot has turned. */
  private static List<? extends HolonomicSample> hopWithAHalfTurn() {
    Keyframe turned = Keyframe.at(new Pose2d(0.3, 0, Rotation2d.PI));
    return Trajectories.through(Pose2d.ZERO, new ChassisVelocities(), List.of(turned), LIMITS, STEP).trajectory().getSamples();
  }

  /** How long a path waits at its end for the robot to face the last keyframe. */
  static Stream<Arguments> waits() {
    return Stream.of(
        answers("it waits until the robot has stopped turning",
            () -> hopWithAHalfTurn().getLast().velocity.omega, 0.0),
        answers("...and no longer: a sample earlier, the robot is still turning",
            () -> hopWithAHalfTurn().get(hopWithAHalfTurn().size() - 2).velocity.omega != 0, true),
        answers("the samples it waits through follow on from the path's own, a step apart",
            () -> {
              List<? extends HolonomicSample> samples = hopWithAHalfTurn();
              int outOfOrder = 0;
              for (int index = 1; index < samples.size(); index++) {
                if (samples.get(index).time <= samples.get(index - 1).time) {
                  outOfOrder++;
                }
              }
              double lastGap = samples.getLast().time - samples.get(samples.size() - 2).time;
              return List.of(outOfOrder, rounded(lastGap));
            },
            List.of(0, STEP)),
        answers("while it waits, it stays at the last keyframe",
            () -> hopWithAHalfTurn().getLast().pose.getTranslation(), new Translation2d(0.3, 0)));
  }

  private static List<HolonomicSample> twoPoints() {
    PathPoint first = new PathPoint(1, new Translation2d(0, 0), 0.5, new ChassisVelocities(1, 2, 3));
    PathPoint second = new PathPoint(1.5, new Translation2d(1, 0), 1, new ChassisVelocities(2, 4, 7));
    return PathPoint.trajectory(List.of(first, second)).getSamples();
  }

  private static List<Double> acceleration(HolonomicSample sample) {
    return List.of(sample.acceleration.ax, sample.acceleration.ay, sample.acceleration.alpha);
  }

  /** How a plan's points become a trajectory's samples. */
  static Stream<Arguments> points() {
    return Stream.of(
        answers("a sample's acceleration is the change to the next, over the time between them",
            () -> acceleration(twoPoints().get(0)), List.of(2.0, 4.0, 8.0)),
        answers("the last sample's acceleration is none", () -> acceleration(twoPoints().get(1)), List.of(0.0, 0.0, 0.0)),
        answers("a sample is at its point's place, facing its heading",
            () -> twoPoints().get(1).pose, new Pose2d(1, 0, new Rotation2d(1))),
        answers("a sample is at its point's time", () -> twoPoints().get(1).time, 1.5));
  }

  // spotless:on
}
