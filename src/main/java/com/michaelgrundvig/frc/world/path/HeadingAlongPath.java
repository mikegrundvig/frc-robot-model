package com.michaelgrundvig.frc.world.path;

import org.wpilib.math.trajectory.TrapezoidProfile;

/**
 * The robot's heading along a path through keyframes, as a function of time.
 *
 * <p>The robot turns toward one keyframe's heading at a time, on a trapezoid profile. It starts on
 * the next keyframe's heading only once two things have happened: the path has passed the keyframe
 * it is turning toward, and the robot has finished turning to that keyframe's heading. So a robot
 * that turns slowly stays on one turn while the path runs ahead, and a robot that turns quickly
 * holds each heading until the path gets there.
 *
 * <p>Headings here are unwrapped: they are angles in radians that may run past half a turn, so that
 * the way round from one to the next is simply from the smaller number to the larger.
 *
 * <p>An instance is asked in order of time. It remembers which keyframe it is turning toward, and
 * never goes back to an earlier one.
 */
final class HeadingAlongPath {
  private final TrapezoidProfile turning;

  /** For each keyframe, the seconds from the path's start at which the path passes it. */
  private final double[] passingTimes;

  /** For each keyframe, its heading, unwrapped, in radians. */
  private final double[] headings;

  /** Which keyframe the robot is turning toward. */
  private int target;

  /** When the turn toward that keyframe began, in seconds from the path's start. */
  private double turnBegan;

  /** The heading and turning rate the robot had as that turn began. */
  private TrapezoidProfile.State turnBeganFrom;

  /**
   * @param turning the profile the heading follows: its top turning rate and how fast it spins up
   * @param passingTimes for each keyframe, when the path passes it, in seconds from its start
   * @param headings for each keyframe, its heading, unwrapped, in radians
   * @param startHeading the robot's heading as the path starts, in radians
   * @param startRate how fast the robot is turning as the path starts, in radians a second
   */
  HeadingAlongPath(
      TrapezoidProfile turning,
      double[] passingTimes,
      double[] headings,
      double startHeading,
      double startRate) {
    this.turning = turning;
    this.passingTimes = passingTimes;
    this.headings = headings;
    turnBeganFrom = new TrapezoidProfile.State(startHeading, startRate);
  }

  /**
   * The heading and turning rate at a time.
   *
   * @param time seconds from the path's start, never earlier than the last time asked
   */
  TrapezoidProfile.State at(double time) {
    int lastKeyframe = headings.length - 1;
    while (target < lastKeyframe) {
      double facedAt = turnBegan + turning.timeLeftUntil(turnBeganFrom, targetHeading());
      boolean pathHasYetToPassIt = time < passingTimes[target];
      boolean robotHasYetToFaceIt = time < facedAt;
      if (pathHasYetToPassIt || robotHasYetToFaceIt) {
        break;
      }
      // The next turn begins at whichever of the two came later, from the heading just reached
      // and with the robot no longer turning.
      turnBegan = Math.max(passingTimes[target], facedAt);
      turnBeganFrom = targetHeading();
      target++;
    }
    return turning.calculate(time - turnBegan, turnBeganFrom, targetHeading());
  }

  /**
   * When the robot faces the last keyframe's heading, in seconds from the path's start. It is asked
   * once the path has ended, and steps through whatever turns are still to come.
   */
  double whenItFacesTheLast() {
    int lastKeyframe = headings.length - 1;
    double time = passingTimes[lastKeyframe];
    while (true) {
      // Asking for the heading at a time moves on to the next turn, if that turn has begun by
      // then. The time at which the turn then under way ends is the next time to ask at.
      at(time);
      time = turnBegan + turning.timeLeftUntil(turnBeganFrom, targetHeading());
      if (target == lastKeyframe) {
        return time;
      }
    }
  }

  /** The heading the robot is turning toward, to be reached with the robot no longer turning. */
  private TrapezoidProfile.State targetHeading() {
    return new TrapezoidProfile.State(headings[target], 0);
  }
}
