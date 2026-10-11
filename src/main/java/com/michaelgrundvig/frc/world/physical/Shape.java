package com.michaelgrundvig.frc.world.physical;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Rule;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Translation3d;

/**
 * A solid's shape, set where it is in its owner's frame (a link's, for part of a mechanism), and
 * what it's made of: what the robot's extension checks and the views use. Only a {@link Box} so
 * far; other kinds come back when a robot needs one.
 */
public sealed interface Shape {
  /** Where it is, and how it's turned, in its owner's frame: each kind says from which point. */
  Pose3d pose();

  /** What it's made of. */
  Material material();

  /** How far a point in its owner's frame is from it, in meters: zero inside it or on it. */
  double distance(Translation3d point);

  /** Its middle, the center of its volume, in its owner's frame. */
  Translation3d center();

  /** Whether a point in its owner's frame is inside it, or on it. */
  default boolean contains(Translation3d point) {
    return distance(point) == 0;
  }

  /**
   * Points on it, in its owner's frame, that reach as far as it does in every direction: its
   * corners, or around a cylinder's ends closely enough to fall short by a part in a thousand. How
   * far it reaches out of the robot, or up, is how far the farthest of them does.
   */
  List<Translation3d> points();

  /**
   * A box, centered on its pose, its sides along the pose's axes.
   *
   * @param pose its center, and how it's turned
   * @param size its length along the pose's x, width along y, and height along z, in meters
   * @param material what it's made of
   */
  @ConfigRecord
  record Box(Pose3d pose, Translation3d size, Material material) implements Shape {
    public Box {
      Checks.on(Box.class)
          .field("pose", pose, Rule.of(Shape::whyNotAPose))
          .field("size", size, Rule.of(Box::whyNotASize))
          .orThrow();
    }

    /** Why a box cannot be a size, if it cannot: it needs some length along each axis. */
    private static Optional<String> whyNotASize(Translation3d size) {
      boolean positiveAlongEachAxis = size.getX() > 0 && size.getY() > 0 && size.getZ() > 0;
      boolean finite = Double.isFinite(size.getX() + size.getY() + size.getZ());
      if (positiveAlongEachAxis && finite) {
        return Optional.empty();
      }
      return Optional.of(size + " must be positive along each axis");
    }

    /** Starts describing a box, one named value at a time. */
    public static ShapeBoxBuilder.PoseStage builder() {
      return ShapeBoxBuilder.builder();
    }

    /**
     * In the box's own frame, a point is past each pair of faces by some distance, or not at all.
     * Its distance from the box is the length of those three distances taken together, which is the
     * distance to the nearest face, edge or corner as the case may be.
     */
    @Override
    public double distance(Translation3d point) {
      Translation3d fromCenter = inShapesFrame(pose, point);
      double pastX = distancePast(Math.abs(fromCenter.getX()) - size.getX() / 2);
      double pastY = distancePast(Math.abs(fromCenter.getY()) - size.getY() / 2);
      double pastZ = distancePast(Math.abs(fromCenter.getZ()) - size.getZ() / 2);
      return Math.sqrt(pastX * pastX + pastY * pastY + pastZ * pastZ);
    }

    @Override
    public Translation3d center() {
      return pose.getTranslation();
    }

    /** Its eight corners: half its size either way along each of its own axes. */
    @Override
    public List<Translation3d> points() {
      int[] eitherWay = {-1, 1};
      List<Translation3d> corners = new ArrayList<>();
      for (int alongZ : eitherWay) {
        for (int alongY : eitherWay) {
          for (int alongX : eitherWay) {
            Translation3d fromCenter =
                new Translation3d(
                    alongX * size.getX() / 2, alongY * size.getY() / 2, alongZ * size.getZ() / 2);
            corners.add(inOwnersFrame(pose, fromCenter));
          }
        }
      }
      return corners;
    }
  }

  /** A point given in a shape's own frame, in its owner's. */
  private static Translation3d inOwnersFrame(Pose3d pose, Translation3d inShapesFrame) {
    return inShapesFrame.rotateBy(pose.getRotation()).plus(pose.getTranslation());
  }

  /** A point given in a shape's owner's frame, in the shape's own. */
  private static Translation3d inShapesFrame(Pose3d pose, Translation3d inOwnersFrame) {
    return inOwnersFrame.minus(pose.getTranslation()).rotateBy(pose.getRotation().inverse());
  }

  /**
   * How far past a face a coordinate is, given how far it is from the face with outside positive:
   * zero for a coordinate on the inside.
   */
  private static double distancePast(double outsideByThisMuch) {
    return Math.max(outsideByThisMuch, 0);
  }

  /** Why a pose is no pose, if it is not: one of its numbers is missing or infinite. */
  private static Optional<String> whyNotAPose(Pose3d pose) {
    Quaternion turn = pose.getRotation().getQuaternion();
    // A sum with anything that is not a number, or is infinite, is not finite either.
    double sumOfItsNumbers =
        pose.getX()
            + pose.getY()
            + pose.getZ()
            + turn.getW()
            + turn.getX()
            + turn.getY()
            + turn.getZ();
    if (Double.isFinite(sumOfItsNumbers)) {
      return Optional.empty();
    }
    return Optional.of(pose + " must be a pose, numbers all");
  }
}
