package com.michaelgrundvig.frc.world.physical;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Comparator;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;

/**
 * The smallest convex outline around a set of points, found by Andrew's monotone chain.
 *
 * <p>The points are sorted from left to right. Walking them in that order while only ever turning
 * left traces the hull's lower side, from its leftmost point to its rightmost. Walking them the
 * other way in the same manner traces its upper side, back to the leftmost. The two sides joined
 * are the hull, counterclockwise.
 */
final class ConvexHull {
  private ConvexHull() {}

  /**
   * The hull's corners, counterclockwise, starting from the leftmost point (the lowest of them, if
   * several are as far left). Points along one of the hull's edges are left out. Points that are
   * all on one line give fewer than three corners, which is no outline.
   *
   * @param points at least one point
   */
  static List<Translation2d> around(Collection<Translation2d> points) {
    List<Translation2d> leftToRight = new ArrayList<>(points);
    leftToRight.sort(
        Comparator.comparingDouble(Translation2d::getX).thenComparingDouble(Translation2d::getY));
    List<Translation2d> lowerSide = leftTurningChain(leftToRight);
    List<Translation2d> upperSide = leftTurningChain(leftToRight.reversed());

    // Each side ends on the point the other starts from, so each gives up its last point.
    lowerSide.remove(lowerSide.size() - 1);
    upperSide.remove(upperSide.size() - 1);
    List<Translation2d> hull = new ArrayList<>(lowerSide);
    hull.addAll(upperSide);
    return hull;
  }

  /**
   * The points of a sorted list that are left when every one at which the path through them does
   * not turn left is dropped. A point is dropped as soon as a later point shows that the path turns
   * right at it, or goes straight on.
   */
  private static List<Translation2d> leftTurningChain(List<Translation2d> sorted) {
    List<Translation2d> chain = new ArrayList<>();
    for (Translation2d point : sorted) {
      while (chain.size() >= 2
          && turnsRightOrGoesStraightOn(chain.get(chain.size() - 2), chain.getLast(), point)) {
        chain.removeLast();
      }
      chain.add(point);
    }
    return chain;
  }

  /**
   * Whether the path from one point through a second to a third turns right at the second, or goes
   * straight on: whether the third point is to the right of the line through the first two, or on
   * it.
   */
  private static boolean turnsRightOrGoesStraightOn(
      Translation2d from, Translation2d through, Translation2d to) {
    return through.minus(from).cross(to.minus(through)) <= 0;
  }
}
