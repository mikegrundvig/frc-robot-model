package com.michaelgrundvig.frc.world.physical;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.shape.Rectangle2d;

/**
 * A polygon, in meters: where something stands on the floor, seen from above, or a shape's outline
 * in its own x-y plane. Its corners run counterclockwise seen from above (from +z), so its inside
 * is on the left of each edge, and its edges never cross. Edge {@code i} runs from corner {@code i}
 * to the next, and the last edge back to the first corner.
 *
 * @param corners in order, counterclockwise seen from above
 */
public record Polygon2d(List<Translation2d> corners) {
  /** How close to an edge a point counts as on it: a nanometer, as WPILib compares points. */
  private static final double ON_EDGE_METERS = 1e-9;

  public Polygon2d {
    corners = List.copyOf(corners);
    if (corners.size() < 3) {
      throw new IllegalArgumentException(
          "a polygon needs three corners or more, not " + corners.size());
    }
    Optional<String> whyNot = PolygonChecks.whyNotAnOutline(corners);
    if (whyNot.isPresent()) {
      throw new IllegalArgumentException(whyNot.get());
    }
  }

  /**
   * A rectangle centered on a pose, its sides along and across the pose's heading.
   *
   * @param xLength along the heading
   * @param yWidth across it
   */
  public static Polygon2d rectangle(Pose2d center, double xLength, double yWidth) {
    if (!(xLength > 0) || !(yWidth > 0) || Double.isInfinite(xLength + yWidth)) {
      throw new IllegalArgumentException(
          "a rectangle's sides must be positive: " + xLength + " by " + yWidth);
    }
    // The corners in the pose's own frame, counterclockwise from the one behind it on its right:
    // each is half the length forward or back, and half the width left or right.
    double[][] forwardAndLeftSigns = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    List<Translation2d> corners = new ArrayList<>();
    for (double[] signs : forwardAndLeftSigns) {
      Translation2d fromCenter =
          new Translation2d(signs[0] * xLength / 2, signs[1] * yWidth / 2)
              .rotateBy(center.getRotation());
      corners.add(center.getTranslation().plus(fromCenter));
    }
    return new Polygon2d(corners);
  }

  /**
   * The rectangle between two opposite corners, its sides along the axes: an area of the field
   * between two lines across it and two along it.
   */
  public static Polygon2d rectangle(Translation2d corner, Translation2d opposite) {
    double minX = Math.min(corner.getX(), opposite.getX());
    double maxX = Math.max(corner.getX(), opposite.getX());
    double minY = Math.min(corner.getY(), opposite.getY());
    double maxY = Math.max(corner.getY(), opposite.getY());
    if (!(maxX > minX) || !(maxY > minY)) {
      throw new IllegalArgumentException(
          "a rectangle's sides must be positive: from " + corner + " to " + opposite);
    }
    return new Polygon2d(
        List.of(
            new Translation2d(minX, minY),
            new Translation2d(maxX, minY),
            new Translation2d(maxX, maxY),
            new Translation2d(minX, maxY)));
  }

  /** WPILib's rectangle, as a polygon. */
  public static Polygon2d of(Rectangle2d rectangle) {
    return rectangle(rectangle.getCenter(), rectangle.getXWidth(), rectangle.getYWidth());
  }

  /**
   * The smallest convex polygon around some points: what a shape covers, seen from above. Corners
   * in a line along its edge are left out.
   *
   * @throws IllegalArgumentException if there are no points, or they are all on one line, which
   *     encloses nothing
   */
  public static Polygon2d hull(Collection<Translation2d> points) {
    if (points.isEmpty()) {
      throw new IllegalArgumentException(
          "no points enclose nothing: a hull needs three that are not on one line");
    }
    List<Translation2d> hull = ConvexHull.around(points);
    if (hull.size() < 3) {
      throw new IllegalArgumentException("points all on one line enclose nothing");
    }
    return new Polygon2d(hull);
  }

  /** A corner by its index, counting around: the corner after the last is the first. */
  public Translation2d corner(int index) {
    return corners.get(Math.floorMod(index, corners.size()));
  }

  /** Square meters. */
  public double area() {
    return PolygonChecks.signedArea(corners);
  }

  /**
   * Its center of area.
   *
   * <p>The polygon is taken as a fan of triangles, each from the origin to one edge. A triangle's
   * area is half the cross product of the edge's two corners, signed so that the parts of the fan
   * outside the polygon cancel, and its own center is a third of the sum of its three corners, one
   * of which is the origin. The polygon's center is the triangles' centers, weighted by their
   * areas: the sum of (a + b) · cross(a, b) over the edges, divided by six times the area.
   */
  public Translation2d centroid() {
    double x = 0;
    double y = 0;
    for (int edge = 0; edge < corners.size(); edge++) {
      Translation2d a = corners.get(edge);
      Translation2d b = corner(edge + 1);
      double cross = a.cross(b);
      x += (a.getX() + b.getX()) * cross;
      y += (a.getY() + b.getY()) * cross;
    }
    double area = area();
    return new Translation2d(x / (6 * area), y / (6 * area));
  }

  /** Whether every corner turns left, or goes straight on: no corner points in. */
  public boolean convex() {
    for (int index = 0; index < corners.size(); index++) {
      Translation2d arriving = corner(index + 1).minus(corner(index));
      Translation2d leaving = corner(index + 2).minus(corner(index + 1));
      boolean turnsRight = arriving.cross(leaving) < 0;
      if (turnsRight) {
        return false;
      }
    }
    return true;
  }

  /**
   * Whether a point is inside the polygon or on its edge.
   *
   * <p>A point within a nanometer of an edge is on it. Any other point is judged by ray casting: a
   * ray from the point toward +x crosses the polygon's edges an odd number of times if the point is
   * inside, and an even number if it is outside.
   */
  public boolean contains(Translation2d point) {
    boolean crossingsAreOdd = false;
    for (int edge = 0; edge < corners.size(); edge++) {
      Translation2d a = corners.get(edge);
      Translation2d b = corner(edge + 1);
      if (Segments.distance(point, a, b) <= ON_EDGE_METERS) {
        return true;
      }
      // The ray can only cross an edge whose ends are on opposite sides of the ray's line. An
      // end exactly on the line counts as below it, so a ray through a corner crosses one of
      // the two edges that meet there, not both and not neither.
      boolean endsStraddleTheRay = (a.getY() > point.getY()) != (b.getY() > point.getY());
      if (!endsStraddleTheRay) {
        continue;
      }
      // Where the edge is at the point's y: from a toward b by the share of the edge's rise that
      // the point's y is above a.
      double xWhereEdgeMeetsRaysLine =
          a.getX() + (b.getX() - a.getX()) * (point.getY() - a.getY()) / (b.getY() - a.getY());
      boolean edgeIsAheadOfThePoint = point.getX() < xWhereEdgeMeetsRaysLine;
      if (edgeIsAheadOfThePoint) {
        crossingsAreOdd = !crossingsAreOdd;
      }
    }
    return crossingsAreOdd;
  }

  /**
   * Whether it and another polygon share a point: one inside the other, or their edges crossing or
   * touching.
   */
  public boolean touches(Polygon2d other) {
    for (Translation2d corner : corners) {
      if (other.contains(corner)) {
        return true;
      }
    }
    for (Translation2d corner : other.corners) {
      if (contains(corner)) {
        return true;
      }
    }
    // Neither has a corner in the other, but they may still overlap, as two bars of a cross do:
    // then some edge of one meets some edge of the other.
    for (int edge = 0; edge < corners.size(); edge++) {
      for (int otherEdge = 0; otherEdge < other.corners.size(); otherEdge++) {
        boolean edgesMeet =
            Segments.meet(
                corners.get(edge),
                corner(edge + 1),
                other.corners.get(otherEdge),
                other.corner(otherEdge + 1));
        if (edgesMeet) {
          return true;
        }
      }
    }
    return false;
  }

  /**
   * Whether the straight line from one point to another touches the polygon: an end inside it or on
   * its edge, or the line crossing or touching an edge.
   */
  public boolean touches(Translation2d from, Translation2d to) {
    if (contains(from) || contains(to)) {
      return true;
    }
    for (int edge = 0; edge < corners.size(); edge++) {
      if (Segments.meet(from, to, corners.get(edge), corner(edge + 1))) {
        return true;
      }
    }
    return false;
  }

  /** How far a point is from the polygon, in meters: zero inside it or on its edge. */
  public double distance(Translation2d point) {
    if (contains(point)) {
      return 0;
    }
    double nearest = Double.POSITIVE_INFINITY;
    for (int edge = 0; edge < corners.size(); edge++) {
      double toEdge = Segments.distance(point, corners.get(edge), corner(edge + 1));
      nearest = Math.min(nearest, toEdge);
    }
    return nearest;
  }

  /**
   * How far a point inside the polygon goes, heading one way, before it reaches the edge: zero for
   * a point outside it, or on its edge heading out.
   *
   * <p>The point's path is a ray, {@code from + ahead · heading}, and each edge is a segment,
   * {@code start + share · edge}, where {@code share} runs from 0 to 1. Setting the two equal and
   * taking the cross product of both sides with the edge, and then with the heading, gives the two
   * unknowns:
   *
   * <pre>
   *   ahead = cross(start - from, edge) / cross(heading, edge)
   *   share = cross(start - from, heading) / cross(heading, edge)
   * </pre>
   *
   * <p>The ray reaches the edge if that point is ahead of it and on the edge. The answer is the
   * nearest such point over all the edges.
   */
  public double reach(Translation2d from, Rotation2d way) {
    if (!contains(from)) {
      return 0;
    }
    Translation2d heading = new Translation2d(way.getCos(), way.getSin());
    double nearest = Double.POSITIVE_INFINITY;
    for (int index = 0; index < corners.size(); index++) {
      Translation2d start = corners.get(index);
      Translation2d edge = corner(index + 1).minus(start);
      double headingCrossEdge = heading.cross(edge);
      boolean headingAlongTheEdge = headingCrossEdge == 0;
      if (headingAlongTheEdge) {
        // The ray is parallel to the edge, and never reaches it.
        continue;
      }
      Translation2d toStart = start.minus(from);
      double ahead = toStart.cross(edge) / headingCrossEdge;
      double share = toStart.cross(heading) / headingCrossEdge;
      // A point on the edge it is leaving is not ahead of it. An end of the edge counts as on
      // it, within the same tolerance.
      boolean isAhead = ahead > ON_EDGE_METERS;
      boolean isOnTheEdge = share >= -ON_EDGE_METERS && share <= 1 + ON_EDGE_METERS;
      if (isAhead && isOnTheEdge) {
        nearest = Math.min(nearest, ahead);
      }
    }
    boolean reachesNoEdge = Double.isInfinite(nearest);
    if (reachesNoEdge) {
      return 0;
    }
    return nearest;
  }

  /**
   * Its area cut into triangles of its corners, each by its corners' indices and counterclockwise
   * seen from above. A convex polygon is cut into a fan from its first corner.
   */
  public List<TriangleMesh.Triangle> triangles() {
    return EarClipping.triangles(this);
  }
}
