package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Bumpers;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;

/**
 * Where a robot starts a match, written as if it were on the blue alliance, and without the robot:
 * a spot on the floor, the way the robot faces there, and which point of the robot's bumpers stands
 * on the spot. A game manual words its starts this way ("bumpers on the starting line", "back
 * against the alliance wall"), so a season writes them as the manual does, once for every robot.
 *
 * <p>{@link #poseFor} stands a robot on the start: it is the one place a robot's pose on a start is
 * worked out. {@link #on} gives that pose for either alliance.
 *
 * @param name what robot code, logs, and the drive team call it
 * @param spot where the anchor point stands, for blue, and the way the robot faces there
 * @param anchor which point of the robot's bumpers stands on the spot
 */
public record Start(String name, Pose2d spot, Anchor anchor) {
  /**
   * Which point of the robot's bumpers, seen from above, stands on a start's spot. Front, back,
   * left and right are the robot's own, as it faces on the start.
   */
  public enum Anchor {
    /** The robot's center: the spot is where the robot's origin is. */
    CENTER(0, 0),
    /** The middle of the front bumper's outside face. */
    FRONT_BUMPER(1, 0),
    /** The middle of the back bumper's outside face. */
    BACK_BUMPER(-1, 0),
    /** The middle of the left bumper's outside face. */
    LEFT_BUMPER(0, 1),
    /** The middle of the right bumper's outside face. */
    RIGHT_BUMPER(0, -1),
    /** The bumpers' front left corner. */
    FRONT_LEFT_CORNER(1, 1),
    /** The bumpers' front right corner. */
    FRONT_RIGHT_CORNER(1, -1),
    /** The bumpers' back left corner. */
    BACK_LEFT_CORNER(-1, 1),
    /** The bumpers' back right corner. */
    BACK_RIGHT_CORNER(-1, -1);

    /** Which way the point is from the robot's center along its length: +1 front, -1 back. */
    private final int forward;

    /** Which way the point is from the robot's center across its width: +1 left, -1 right. */
    private final int left;

    Anchor(int forward, int left) {
      this.forward = forward;
      this.left = left;
    }
  }

  public Start {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a start needs a name");
    }
  }

  /** A start with the robot's center on the spot: an auto's own start, anywhere on the field. */
  public static Start at(String name, Pose2d robot) {
    return new Start(name, robot, Anchor.CENTER);
  }

  /**
   * Where a robot with these bumpers stands on this start, for blue: its center, facing the way the
   * start says, with its anchor point on the spot.
   */
  public Pose2d poseFor(Bumpers bumpers) {
    // The anchor point is this far ahead of the robot's center and to its left, in the robot's own
    // axes, so the center is that far behind the spot and to its right.
    double ahead = anchor.forward * bumpers.halfLengthMeters();
    double toTheLeft = anchor.left * bumpers.halfWidthMeters();
    return spot.transformBy(new Transform2d(-ahead, -toTheLeft, Rotation2d.ZERO));
  }

  /**
   * Where a robot with these bumpers stands on this start, for an alliance, in field coordinates.
   */
  public Pose2d on(FieldModel field, Alliance alliance, Bumpers bumpers) {
    return field.forAlliance(alliance, poseFor(bumpers));
  }
}
