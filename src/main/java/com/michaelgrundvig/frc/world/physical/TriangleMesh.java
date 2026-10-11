package com.michaelgrundvig.frc.world.physical;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.UnaryOperator;
import org.wpilib.math.geometry.Translation2d;

/**
 * Triangles in 3D sharing corners by index: a shape exactly, as a physics engine, a 3D view, or a
 * glTF file takes it. Each triangle's corners run counterclockwise seen from outside, so its
 * normal, by the right-hand rule, points out of the shape: up, for ground.
 *
 * @param corners every corner, in meters, in the mesh's frame
 * @param triangles each by its three corners' indices
 */
public record TriangleMesh(List<Vector3> corners, List<Triangle> triangles) {
  /**
   * A triangle is taken to stand on edge when the area it covers seen from above is less than this
   * share of the sum of two of its sides' squared lengths, seen from above. Comparing with the
   * sides makes it a matter of the triangle's shape, not of its size.
   */
  private static final double ON_EDGE_SHARE = 1e-12;

  /**
   * How far outside a triangle, as a share of its sides, a point still counts as over it: enough
   * that a point on an edge two triangles share is over at least one of them, whatever the
   * rounding.
   */
  private static final double OVER_EDGE_SHARE = 1e-9;

  /** No triangles at all. */
  public static final TriangleMesh EMPTY = new TriangleMesh(List.of(), List.of());

  /**
   * A triangle, by its corners' indices in the mesh, counterclockwise seen from outside.
   *
   * @param a its first corner
   * @param b the next, counterclockwise
   * @param c the last
   */
  public record Triangle(int a, int b, int c) {}

  /**
   * Where a line straight down meets the mesh.
   *
   * @param height how high, in meters
   * @param normal the face it meets there, a unit vector turned to face up
   */
  public record Hit(double height, Vector3 normal) {}

  public TriangleMesh {
    corners = List.copyOf(corners);
    triangles = List.copyOf(triangles);
    for (int index = 0; index < corners.size(); index++) {
      Vector3 corner = corners.get(index);
      // Each number is asked by itself: three that are each finite can add up to more than a
      // number holds.
      boolean numbersAll =
          Double.isFinite(corner.x()) && Double.isFinite(corner.y()) && Double.isFinite(corner.z());
      if (!numbersAll) {
        throw new IllegalArgumentException("corner " + index + " isn't a point: " + corner);
      }
    }
    for (int index = 0; index < triangles.size(); index++) {
      Triangle triangle = triangles.get(index);
      for (int corner : new int[] {triangle.a(), triangle.b(), triangle.c()}) {
        boolean isOneOfTheMeshs = corner >= 0 && corner < corners.size();
        if (!isOneOfTheMeshs) {
          throw new IllegalArgumentException(
              "triangle " + index + " names corner " + corner + " of " + corners.size());
        }
      }
      // The cross product of two of a triangle's sides is twice its area long.
      Vector3 a = corners.get(triangle.a());
      Vector3 toB = corners.get(triangle.b()).minus(a);
      Vector3 toC = corners.get(triangle.c()).minus(a);
      boolean hasSomeArea = toB.cross(toC).length() > 0;
      if (!hasSomeArea) {
        throw new IllegalArgumentException("triangle " + index + " has no area");
      }
    }
  }

  /** A mesh built a corner and a triangle at a time. */
  public static Builder builder() {
    return new Builder();
  }

  /** A triangle's three corners, in its order. */
  public List<Vector3> corners(Triangle triangle) {
    return List.of(corners.get(triangle.a()), corners.get(triangle.b()), corners.get(triangle.c()));
  }

  /**
   * Which way a triangle faces, a unit vector out of the shape. It is the cross product of the side
   * from its first corner to its second with the side from its first to its third, which by the
   * right-hand rule points toward whoever sees the corners run counterclockwise.
   */
  public Vector3 normal(Triangle triangle) {
    Vector3 a = corners.get(triangle.a());
    Vector3 toB = corners.get(triangle.b()).minus(a);
    Vector3 toC = corners.get(triangle.c()).minus(a);
    return toB.cross(toC).unit();
  }

  /**
   * Whether a triangle stands on edge, so that seen from above it's a line: a wall, which a line
   * straight down only grazes.
   */
  public boolean vertical(Triangle triangle) {
    Vector3 a = corners.get(triangle.a());
    Vector3 b = corners.get(triangle.b());
    Vector3 c = corners.get(triangle.c());
    // The sides from a to b and from a to c, seen from above.
    double toBx = b.x() - a.x();
    double toBy = b.y() - a.y();
    double toCx = c.x() - a.x();
    double toCy = c.y() - a.y();
    double twiceAreaFromAbove = toBx * toCy - toBy * toCx;
    double sidesSquared = toBx * toBx + toBy * toBy + toCx * toCx + toCy * toCy;
    return Math.abs(twiceAreaFromAbove) <= ON_EDGE_SHARE * sidesSquared;
  }

  /**
   * How high a line straight up and down through a point meets a triangle; empty if it passes
   * beside the triangle, or the triangle stands on edge (a wall), which it only grazes. A point on
   * the triangle's edge meets it.
   */
  public OptionalDouble heightOver(Triangle triangle, Translation2d point) {
    if (vertical(triangle)) {
      return OptionalDouble.empty();
    }
    Vector3 a = corners.get(triangle.a());
    Vector3 b = corners.get(triangle.b());
    Vector3 c = corners.get(triangle.c());
    // Everything here is seen from above. The point is written as a, plus a share of the side
    // from a to b, plus a share of the side from a to c: its barycentric coordinates. Each share
    // is a ratio of cross products (Cramer's rule). The point is over the triangle when neither
    // share is negative and the two together are at most 1.
    double toBx = b.x() - a.x();
    double toBy = b.y() - a.y();
    double toCx = c.x() - a.x();
    double toCy = c.y() - a.y();
    double twiceArea = toBx * toCy - toBy * toCx;
    double toPointX = point.getX() - a.x();
    double toPointY = point.getY() - a.y();
    double shareTowardB = (toPointX * toCy - toPointY * toCx) / twiceArea;
    double shareTowardC = (toBx * toPointY - toBy * toPointX) / twiceArea;
    boolean besideTheTriangle =
        shareTowardB < -OVER_EDGE_SHARE
            || shareTowardC < -OVER_EDGE_SHARE
            || shareTowardB + shareTowardC > 1 + OVER_EDGE_SHARE;
    if (besideTheTriangle) {
      return OptionalDouble.empty();
    }
    // The triangle is flat, so its height there is the same shares of its corners' heights.
    double height = a.z() + shareTowardB * (b.z() - a.z()) + shareTowardC * (c.z() - a.z());
    return OptionalDouble.of(height);
  }

  /**
   * What a line cast straight down through a point meets first: the highest triangle over the
   * point, passing walls by. Empty where it meets none.
   */
  public Optional<Hit> castDown(Translation2d point) {
    Optional<Triangle> highest = Optional.empty();
    double highestHeight = Double.NEGATIVE_INFINITY;
    for (Triangle triangle : triangles) {
      OptionalDouble heightOver = heightOver(triangle, point);
      if (heightOver.isPresent() && heightOver.getAsDouble() > highestHeight) {
        highestHeight = heightOver.getAsDouble();
        highest = Optional.of(triangle);
      }
    }
    if (highest.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(hit(highest.get(), highestHeight));
  }

  /**
   * A hit on a triangle at a height. A triangle that faces down, such as the underside of a shape,
   * is hit from above on its back, so its normal is reversed to face up.
   */
  public Hit hit(Triangle triangle, double height) {
    Vector3 normal = normal(triangle);
    boolean facesDown = normal.z() < 0;
    if (facesDown) {
      return new Hit(height, normal.times(-1));
    }
    return new Hit(height, normal);
  }

  /** Both meshes as one. */
  public TriangleMesh plus(TriangleMesh other) {
    List<Vector3> allCorners = new ArrayList<>(corners);
    allCorners.addAll(other.corners);
    // The other mesh's corners follow this one's, so each is numbered that much higher.
    int shift = corners.size();
    List<Triangle> allTriangles = new ArrayList<>(triangles);
    for (Triangle triangle : other.triangles) {
      allTriangles.add(
          new Triangle(triangle.a() + shift, triangle.b() + shift, triangle.c() + shift));
    }
    return new TriangleMesh(allCorners, allTriangles);
  }

  /**
   * The mesh with each corner moved, facing out still.
   *
   * @param move where each corner goes
   * @param mirrors whether the move is a mirror image, which turns counterclockwise into clockwise:
   *     each triangle's corners are then listed the other way round
   */
  public TriangleMesh map(UnaryOperator<Vector3> move, boolean mirrors) {
    List<Vector3> moved = corners.stream().map(move).toList();
    if (!mirrors) {
      return new TriangleMesh(moved, triangles);
    }
    List<Triangle> listedTheOtherWayRound = new ArrayList<>();
    for (Triangle triangle : triangles) {
      listedTheOtherWayRound.add(new Triangle(triangle.a(), triangle.c(), triangle.b()));
    }
    return new TriangleMesh(moved, listedTheOtherWayRound);
  }

  /** Builds a mesh a corner and a triangle at a time. */
  public static final class Builder {
    private final List<Vector3> corners = new ArrayList<>();
    private final List<Triangle> triangles = new ArrayList<>();

    private Builder() {}

    /** A corner, by its index: the first is 0. */
    public int corner(double x, double y, double z) {
      return corner(new Vector3(x, y, z));
    }

    /** A corner, by its index: the first is 0. */
    public int corner(Vector3 at) {
      corners.add(at);
      return corners.size() - 1;
    }

    /** A triangle between three corners, counterclockwise seen from outside. */
    public Builder triangle(int a, int b, int c) {
      triangles.add(new Triangle(a, b, c));
      return this;
    }

    /** A flat face of four corners, counterclockwise seen from outside, as two triangles. */
    public Builder quad(int a, int b, int c, int d) {
      return triangle(a, b, c).triangle(a, c, d);
    }

    public TriangleMesh build() {
      return new TriangleMesh(corners, triangles);
    }
  }
}
