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
}
