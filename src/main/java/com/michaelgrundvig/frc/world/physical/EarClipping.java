package com.michaelgrundvig.frc.world.physical;

import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;

/**
 * Cuts an outline's area into triangles of its own corners, by ear clipping.
 *
 * <p>An ear is a corner whose triangle, with the corners before and after it, lies wholly inside
 * the outline: the outline turns left there, and the triangle holds none of the other corners.
 * Cutting an ear off leaves an outline with one corner fewer, which has ears of its own, so cutting
 * goes on round the outline until only a triangle is left. A corner where the outline goes straight
 * on encloses nothing, and is dropped without a triangle.
 *
 * <p>An instance exists for the length of one cutting. It holds the outline and which of its
 * corners have not yet been cut off.
 */
final class EarClipping {
  /**
   * Two edges that meet at less than this are taken to go straight on. It is the sine of the angle
   * between them, so it does not depend on how long the edges are.
   */
  private static final double STRAIGHT_ON = 1e-12;

  private final Polygon2d outline;
  private final List<Translation2d> corners;

  /** The numbers of the corners not yet cut off, in the outline's order. */
  private final List<Integer> remaining = new ArrayList<>();

  private final List<TriangleMesh.Triangle> triangles = new ArrayList<>();

  private EarClipping(Polygon2d outline) {
    this.outline = outline;
    corners = outline.corners();
    for (int corner = 0; corner < corners.size(); corner++) {
      remaining.add(corner);
    }
  }

  /**
   * The triangles of an outline whose corners run counterclockwise. Each triangle names its corners
   * by their numbers in the outline, counterclockwise. A convex outline is cut into a fan from its
   * first corner.
   *
   * @throws IllegalStateException if no ear can be found, which an outline whose edges do not cross
   *     never causes
   */
  static List<TriangleMesh.Triangle> triangles(Polygon2d outline) {
    EarClipping clipping = new EarClipping(outline);
    clipping.cutEarsUntilThreeCornersRemain();
    clipping.addTheLastTriangle();
    return List.copyOf(clipping.triangles);
  }

  /**
   * Goes round the remaining corners, starting at the second. A corner that can be cut off is, and
   * the corner that takes its place is tried next. Otherwise the next corner is tried.
   */
  private void cutEarsUntilThreeCornersRemain() {
    int position = 1;
    int cornersTriedSinceACut = 0;
    while (remaining.size() > 3) {
      int count = remaining.size();
      position = Math.floorMod(position, count);
      int previous = remaining.get(Math.floorMod(position - 1, count));
      int corner = remaining.get(position);
      int next = remaining.get((position + 1) % count);

      Translation2d arriving = corners.get(corner).minus(corners.get(previous));
      Translation2d leaving = corners.get(next).minus(corners.get(corner));
      // The cross product is the two edges' lengths times the sine of the angle the outline
      // turns by: positive for a left turn.
      double turn = arriving.cross(leaving);
      boolean goesStraightOn =
          Math.abs(turn) <= STRAIGHT_ON * arriving.getNorm() * leaving.getNorm();
      boolean turnsLeft = turn > 0;

      if (goesStraightOn) {
        remaining.remove(position);
        cornersTriedSinceACut = 0;
      } else if (turnsLeft && holdsNoOtherCorner(previous, corner, next)) {
        // An ear.
        triangles.add(new TriangleMesh.Triangle(previous, corner, next));
        remaining.remove(position);
        cornersTriedSinceACut = 0;
      } else {
        position++;
        cornersTriedSinceACut++;
        if (cornersTriedSinceACut > count) {
          throw new IllegalStateException("couldn't cut " + outline + " into triangles");
        }
      }
    }
  }

  /**
   * The three corners left make the last triangle, unless they lie in a line and so enclose
   * nothing.
   */
  private void addTheLastTriangle() {
    int first = remaining.get(0);
    int second = remaining.get(1);
    int third = remaining.get(2);
    Translation2d toSecond = corners.get(second).minus(corners.get(first));
    Translation2d toThird = corners.get(third).minus(corners.get(first));
    boolean enclosesSomething = toSecond.cross(toThird) > 0;
    if (enclosesSomething) {
      triangles.add(new TriangleMesh.Triangle(first, second, third));
    }
  }

  /**
   * Whether the triangle of three corners holds none of the other corners still remaining, its
   * edges included.
   */
  private boolean holdsNoOtherCorner(int a, int b, int c) {
    for (int other : remaining) {
      boolean isOneOfTheThree = other == a || other == b || other == c;
      if (isOneOfTheThree) {
        continue;
      }
      if (isInTriangle(corners.get(other), corners.get(a), corners.get(b), corners.get(c))) {
        return false;
      }
    }
    return true;
  }

  /**
   * Whether a point is in a triangle whose corners run counterclockwise, or on its edge: whether it
   * is on the left of each edge in turn, or on it.
   */
  private static boolean isInTriangle(
      Translation2d point, Translation2d a, Translation2d b, Translation2d c) {
    return b.minus(a).cross(point.minus(a)) >= 0
        && c.minus(b).cross(point.minus(b)) >= 0
        && a.minus(c).cross(point.minus(c)) >= 0;
  }
}
