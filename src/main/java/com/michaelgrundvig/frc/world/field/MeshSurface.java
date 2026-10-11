package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.math.geometry.Translation2d;

/**
 * Raised ground of any shape, given as triangles: the escape hatch for what {@link Bump}, {@link
 * Ramp} and {@link Platform} can't say, such as a shape from the field's CAD. Its heights are found
 * by casting a line straight down onto the triangles: the highest one over a point is the ground
 * there. Its footprint is the outline of its triangles seen from above, made convex; its crossings
 * are given, since triangles don't say where a robot crossing them can tell.
 *
 * <p>The triangles over each part of the field are looked up in a grid made once ({@link
 * GroundGrid}), so a CAD mesh of thousands of triangles costs a few of them a lookup.
 */
public final class MeshSurface implements Surface {
  private final String name;
  private final Material material;
  private final TriangleMesh mesh;
  private final List<Crossing> crossings;
  private final Polygon2d footprint;

  /** Which of the mesh's triangles are over each part of the field. */
  private final GroundGrid grid;

  /**
   * A surface covered in the field's carpet.
   *
   * @param name for the log
   * @param mesh its triangles, in the field frame, counterclockwise seen from outside
   * @param crossings the lines on it that place a robot crossing them, if any
   * @throws IllegalArgumentException if it has nothing to stand on (only walls), or a crossing runs
   *     off it
   */
  public MeshSurface(String name, TriangleMesh mesh, List<Crossing> crossings) {
    this(name, mesh, crossings, Material.CARPET);
  }

  /**
   * @param name for the log
   * @param mesh its triangles, in the field frame, counterclockwise seen from outside
   * @param crossings the lines on it that place a robot crossing them, if any
   * @param material what it's covered in
   * @throws IllegalArgumentException if it has nothing to stand on (only walls), or a crossing runs
   *     off it
   */
  public MeshSurface(String name, TriangleMesh mesh, List<Crossing> crossings, Material material) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a surface needs a name");
    }
    this.name = name;
    this.material = material;
    this.mesh = mesh;
    this.crossings = List.copyOf(crossings);
    List<Integer> ground = groundTriangles(mesh);
    if (ground.isEmpty()) {
      throw new IllegalArgumentException(name + ": it has nothing to stand on, only walls");
    }
    footprint = Polygon2d.hull(cornersSeenFromAbove(mesh, ground));
    for (Crossing crossing : this.crossings) {
      if (!footprint.contains(crossing.start()) || !footprint.contains(crossing.end())) {
        throw new IllegalArgumentException(name + ": " + crossing.name() + " runs off it");
      }
    }
    grid = new GroundGrid(mesh, ground, footprint);
  }

  /**
   * The numbers of the triangles a robot can stand on: every triangle that is not a wall. A line
   * straight down only grazes a wall, so walls are never the ground.
   */
  private static List<Integer> groundTriangles(TriangleMesh mesh) {
    List<Integer> ground = new ArrayList<>();
    for (int number = 0; number < mesh.triangles().size(); number++) {
      if (!mesh.vertical(mesh.triangles().get(number))) {
        ground.add(number);
      }
    }
    return ground;
  }

  /** The corners of some of a mesh's triangles, as points on the floor under them. */
  private static List<Translation2d> cornersSeenFromAbove(TriangleMesh mesh, List<Integer> ground) {
    List<Translation2d> seen = new ArrayList<>();
    for (int number : ground) {
      for (Vector3 corner : mesh.corners(mesh.triangles().get(number))) {
        seen.add(new Translation2d(corner.x(), corner.y()));
      }
    }
    return seen;
  }

  @Override
  public String name() {
    return name;
  }

  @Override
  public Material material() {
    return material;
  }

  /** Its triangles' outline seen from above, made convex. */
  @Override
  public Polygon2d footprint() {
    return footprint;
  }

  /** The highest triangle over the point, or the floor where there's none. */
  @Override
  public double heightAt(Translation2d point) {
    return castDown(point).map(TriangleMesh.Hit::height).orElse(0.0);
  }

  /**
   * The way the highest triangle over the point faces, turned to face up if it faces down. Where
   * there is no triangle over the point, straight up.
   */
  @Override
  public Vector3 normalAt(Translation2d point) {
    return castDown(point).map(TriangleMesh.Hit::normal).orElse(Vector3.UP);
  }

  @Override
  public TriangleMesh mesh() {
    return mesh;
  }

  @Override
  public List<Crossing> crossings() {
    return crossings;
  }

  /** Its steepest triangle's slope, walls aside. */
  @Override
  public double steepestSlopeRadians() {
    double steepest = 0;
    for (TriangleMesh.Triangle triangle : mesh.triangles()) {
      if (mesh.vertical(triangle)) {
        continue;
      }
      // A triangle's slope is the angle its normal leans from straight up: the normal's level
      // part against its upright part.
      Vector3 normal = mesh.normal(triangle);
      double level = Math.hypot(normal.x(), normal.y());
      double upright = Math.abs(normal.z());
      steepest = Math.max(steepest, Math.atan2(level, upright));
    }
    return steepest;
  }

  @Override
  public String toString() {
    return "MeshSurface[" + name + ", " + mesh.triangles().size() + " triangles]";
  }

  /**
   * What a line straight down at a point meets: the highest of the triangles over it, or nothing
   * where no triangle is over it.
   */
  private Optional<TriangleMesh.Hit> castDown(Translation2d point) {
    Optional<TriangleMesh.Triangle> highest = Optional.empty();
    double highestHeight = Double.NEGATIVE_INFINITY;
    for (int number : grid.trianglesNear(point)) {
      TriangleMesh.Triangle triangle = mesh.triangles().get(number);
      OptionalDouble heightOver = mesh.heightOver(triangle, point);
      if (heightOver.isPresent() && heightOver.getAsDouble() > highestHeight) {
        highestHeight = heightOver.getAsDouble();
        highest = Optional.of(triangle);
      }
    }
    if (highest.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(mesh.hit(highest.get(), highestHeight));
  }
}
