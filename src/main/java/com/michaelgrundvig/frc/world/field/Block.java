package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * One solid part of an {@link Element}: an outline seen from above, standing straight up from one
 * height to another. A box, a post, a wall's face, a bar over an opening. Its outline has no corner
 * pointing in, so anything that asks whether it is in the way has a simple shape to ask: something
 * with a corner pointing in is written as several blocks.
 *
 * <p>A block can't slope or lean. A sloped panel or a hanging chain is left out, or drawn as the
 * upright part of it, and the season's field says which.
 *
 * @param outline where it stands, seen from above, in the field frame: no corner points in
 * @param bottomMeters how high its underside is above the floor: zero when it stands on the floor
 * @param topMeters how high its top is above the floor
 * @param material what it's made of
 */
public record Block(Polygon2d outline, double bottomMeters, double topMeters, Material material) {
  public Block {
    if (!outline.convex()) {
      throw new IllegalArgumentException(
          "a block's outline has no corner pointing in: write it as several blocks");
    }
    boolean bottomIsAHeight = bottomMeters >= 0 && Double.isFinite(bottomMeters);
    boolean topIsAboveIt = topMeters > bottomMeters && Double.isFinite(topMeters);
    if (!bottomIsAHeight || !topIsAboveIt) {
      throw new IllegalArgumentException(
          "a block stands from a height of zero or more up to a greater one, not from "
              + bottomMeters
              + " to "
              + topMeters
              + " m");
    }
  }

  /** A block standing on the floor, up to a height. */
  public static Block standing(Polygon2d outline, double heightMeters, Material material) {
    return new Block(outline, 0, heightMeters, material);
  }

  /**
   * Whether something between two heights runs into it: a robot's bumpers, say, between their
   * bottom edge and their top. A bar a robot drives under isn't in its bumpers' way, and neither is
   * a plate lower than they are.
   *
   * @param lowestMeters the underside of what is asking, above the floor
   * @param highestMeters its top
   */
  public boolean inTheWayOf(double lowestMeters, double highestMeters) {
    return bottomMeters < highestMeters && topMeters > lowestMeters;
  }

  /** How far a point in the field frame is from it, in meters: zero inside it or on it. */
  public double distance(Translation3d point) {
    double acrossTheFloor = outline.distance(new Translation2d(point.getX(), point.getY()));
    double belowIt = Math.max(bottomMeters - point.getZ(), 0);
    double aboveIt = Math.max(point.getZ() - topMeters, 0);
    return Math.hypot(acrossTheFloor, Math.max(belowIt, aboveIt));
  }

  /**
   * Whether it stands between two points in the field frame, so that one can't be seen from the
   * other: whether the straight line between them passes through it.
   *
   * <p>Seen from above, the line crosses the outline over one stretch at most, because the outline
   * has no corner pointing in. The stretch is found a side at a time: each side of the outline cuts
   * the line where the line crosses it, and what is left after every side has cut is the part
   * inside. The line's height changes evenly along it, so over that stretch it runs between its
   * heights at the stretch's two ends, and the block is in the way if those reach into its own
   * heights.
   *
   * <p>A line that only touches it, along a face or at a corner, is not blocked. It makes nothing
   * new as it answers.
   */
  public boolean standsBetween(
      double fromX, double fromY, double fromZ, double toX, double toY, double toZ) {
    double alongX = toX - fromX;
    double alongY = toY - fromY;
    // The stretch of the line inside the outline, as shares of the way from one point to the other.
    double enters = 0;
    double leaves = 1;
    int sides = outline.corners().size();
    for (int side = 0; side < sides; side++) {
      Translation2d corner = outline.corner(side);
      Translation2d next = outline.corner(side + 1);
      double sideX = next.getX() - corner.getX();
      double sideY = next.getY() - corner.getY();
      // How far inside this side the line's start is, and how fast the line moves inward of it:
      // each a cross product with the side, positive on the side's left, which is the inside.
      double startInside = sideX * (fromY - corner.getY()) - sideY * (fromX - corner.getX());
      double movesInside = sideX * alongY - sideY * alongX;
      if (movesInside == 0) {
        // The line runs along this side's direction: outside it, or on it, it never gets in.
        if (startInside <= 0) {
          return false;
        }
        continue;
      }
      double crossesAt = -startInside / movesInside;
      if (movesInside > 0) {
        enters = Math.max(enters, crossesAt);
      } else {
        leaves = Math.min(leaves, crossesAt);
      }
      if (enters >= leaves) {
        return false;
      }
    }
    double heightEntering = fromZ + enters * (toZ - fromZ);
    double heightLeaving = fromZ + leaves * (toZ - fromZ);
    double lowest = Math.min(heightEntering, heightLeaving);
    double highest = Math.max(heightEntering, heightLeaving);
    return highest > bottomMeters && lowest < topMeters;
  }
}
