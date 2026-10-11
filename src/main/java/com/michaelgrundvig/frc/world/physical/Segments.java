package com.michaelgrundvig.frc.world.physical;

import org.wpilib.math.geometry.Translation2d;

/**
 * Questions about straight segments between two points in a plane: how far a point is from one, and
 * whether two of them meet. {@link Polygon2d} asks them of its edges.
 *
 * <p>Several of them rest on the cross product of two vectors in a plane, {@code a.x * b.y - a.y *
 * b.x}. It is positive when the second points to the left of the first, negative when it points to
 * the right, and zero when the two lie along one line.
 */
final class Segments {
  private Segments() {}

  /** How far a point is from the segment between two others. */
  static double distance(Translation2d point, Translation2d start, Translation2d end) {
    Translation2d segment = end.minus(start);
    // Where the point falls along the segment's line, as a share of the segment: 0 at its start
    // and 1 at its end. Held to that range, it is the share at the segment's nearest point.
    double shareAlong = point.minus(start).dot(segment) / segment.getSquaredNorm();
    double shareOfNearest = Math.max(0, Math.min(1, shareAlong));
    Translation2d nearest = start.plus(segment.times(shareOfNearest));
    return point.getDistance(nearest);
  }

  /**
   * Whether two segments meet, touching included: whether segment a-b and segment c-d share a
   * point.
   *
   * <p>Each end of one segment is on some side of the other segment's line, which the sign of a
   * cross product tells. The segments cross if c and d are on opposite sides of a-b and a and b are
   * on opposite sides of c-d. They touch without crossing if an end of one lies on the other, which
   * is when that end is on the other's line (a cross product of zero) and between its ends.
   */
  static boolean meet(Translation2d a, Translation2d b, Translation2d c, Translation2d d) {
    double sideOfC = b.minus(a).cross(c.minus(a));
    double sideOfD = b.minus(a).cross(d.minus(a));
    double sideOfA = d.minus(c).cross(a.minus(c));
    double sideOfB = d.minus(c).cross(b.minus(c));
    boolean cAndDOnOppositeSides = (sideOfC > 0 && sideOfD < 0) || (sideOfC < 0 && sideOfD > 0);
    boolean aAndBOnOppositeSides = (sideOfA > 0 && sideOfB < 0) || (sideOfA < 0 && sideOfB > 0);
    if (cAndDOnOppositeSides && aAndBOnOppositeSides) {
      return true;
    }
    return (sideOfC == 0 && isBetween(c, a, b))
        || (sideOfD == 0 && isBetween(d, a, b))
        || (sideOfA == 0 && isBetween(a, c, d))
        || (sideOfB == 0 && isBetween(b, c, d));
  }

  /**
   * Whether a point that is known to be on a segment's line is between the segment's ends: whether
   * it is inside the rectangle the two ends span.
   */
  private static boolean isBetween(Translation2d point, Translation2d start, Translation2d end) {
    return point.getX() >= Math.min(start.getX(), end.getX())
        && point.getX() <= Math.max(start.getX(), end.getX())
        && point.getY() >= Math.min(start.getY(), end.getY())
        && point.getY() <= Math.max(start.getY(), end.getY());
  }
}
