package com.michaelgrundvig.frc.world.physical;

import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;

/**
 * What a list of corners has to get right to be a polygon's outline: real points, each different
 * from the next, listed counterclockwise around some area, with no two edges crossing. {@link
 * Polygon2d} refuses a list that gets any of it wrong.
 */
final class PolygonChecks {
  private PolygonChecks() {}

  /**
   * The first thing that keeps three or more corners from being an outline, if anything does.
   *
   * @return the reason, as the refusal says it
   */
  static Optional<String> whyNotAnOutline(List<Translation2d> corners) {
    for (int index = 0; index < corners.size(); index++) {
      Translation2d corner = corners.get(index);
      if (!Double.isFinite(corner.getX()) || !Double.isFinite(corner.getY())) {
        return Optional.of("corner " + index + " isn't a point: " + corner);
      }
    }
    for (int index = 0; index < corners.size(); index++) {
      int next = (index + 1) % corners.size();
      boolean apart = corners.get(index).getDistance(corners.get(next)) > 0;
      if (!apart) {
        return Optional.of("corners " + index + " and " + next + " are the same point");
      }
    }
    double area = signedArea(corners);
    if (area < 0) {
      return Optional.of("list a polygon's corners counterclockwise, seen from above");
    }
    boolean hasSomeArea = area > 0;
    if (!hasSomeArea) {
      return Optional.of("a polygon needs some area: its corners are in a line");
    }
    return whyEdgesCross(corners);
  }

  /**
   * The area the corners go around, in square meters, by the shoelace formula: half the sum, over
   * the edges, of the cross product of each edge's two corners. It is positive when the corners run
   * counterclockwise and negative when they run clockwise.
   */
  static double signedArea(List<Translation2d> corners) {
    double sum = 0;
    for (int index = 0; index < corners.size(); index++) {
      int next = (index + 1) % corners.size();
      sum += corners.get(index).cross(corners.get(next));
    }
    return sum / 2;
  }

  /**
   * How the outline's edges run into each other, if they do: an edge that doubles back along the
   * one before it, or two edges that cross. Edges that follow one another always share a corner, so
   * only edges that do not follow one another are tried against each other.
   */
  private static Optional<String> whyEdgesCross(List<Translation2d> corners) {
    int count = corners.size();
    for (int edge = 0; edge < count; edge++) {
      Translation2d start = corners.get(edge);
      Translation2d end = corners.get((edge + 1) % count);
      Translation2d afterEnd = corners.get((edge + 2) % count);
      if (doublesBack(start, end, afterEnd)) {
        return Optional.of("the polygon doubles back at corner " + ((edge + 1) % count));
      }
      for (int later = edge + 2; later < count; later++) {
        boolean firstAndLast = edge == 0 && later == count - 1;
        if (firstAndLast) {
          // The last edge follows round to the first: they share corner 0.
          continue;
        }
        Translation2d laterStart = corners.get(later);
        Translation2d laterEnd = corners.get((later + 1) % count);
        if (Segments.meet(start, end, laterStart, laterEnd)) {
          return Optional.of("the polygon's edges " + edge + " and " + later + " cross");
        }
      }
    }
    return Optional.empty();
  }

  /**
   * Whether the outline turns straight back on itself at a corner: the edge leaving it lies along
   * the edge arriving (their cross product is zero) and points the other way (their dot product is
   * negative).
   */
  private static boolean doublesBack(
      Translation2d before, Translation2d corner, Translation2d after) {
    Translation2d arriving = corner.minus(before);
    Translation2d leaving = after.minus(corner);
    return arriving.cross(leaving) == 0 && arriving.dot(leaving) < 0;
  }
}
