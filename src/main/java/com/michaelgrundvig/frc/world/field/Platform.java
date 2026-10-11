package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A raised platform: a flat top at a height, each of its sides either a vertical edge or a ramp
 * down to the floor. Each ramp runs straight out from its side, as wide as the side, so where two
 * ramps meet at a corner the floor shows between them. A ramp's top edge is a crossing.
 *
 * @param name for the log
 * @param top the flat top's outline, seen from above, in the field frame: convex, so every ramp
 *     runs out from it clear of the others
 * @param heightMeters how high the top is above the floor
 * @param sides each side of the top in turn: side {@code i} is the top's edge from its corner
 *     {@code i} to the next
 * @param material what its top and ramps are covered in
 */
public record Platform(
    String name, Polygon2d top, double heightMeters, List<Side> sides, Material material)
    implements Surface {
  /**
   * Two edges of the top that meet at less than this are taken to go straight on. It is the sine of
   * the angle between them, so it does not depend on how long the edges are.
   */
  private static final double STRAIGHT_ON = 1e-9;

  /**
   * One side of a platform's top: a vertical edge, or a ramp down to the floor.
   *
   * @param rampMeters how far a ramp runs out from the side to its foot, seen from above; zero for
   *     a vertical edge
   */
  public record Side(double rampMeters) {
    /** A vertical edge: the top drops straight to the floor. */
    public static final Side EDGE = new Side(0);

    public Side {
      if (!(rampMeters >= 0) || Double.isInfinite(rampMeters)) {
        throw new IllegalArgumentException(
            "a ramp runs some distance out from its side, or none for an edge: " + rampMeters);
      }
    }

    /** A ramp from the side down to the floor, its foot this far out from the side. */
    public static Side ramp(double runMeters) {
      if (!(runMeters > 0)) {
        throw new IllegalArgumentException("a ramp runs some distance out: " + runMeters);
      }
      return new Side(runMeters);
    }

    /** Whether a ramp leads down from it, rather than a vertical edge. */
    public boolean ramped() {
      return rampMeters > 0;
    }
  }

  /**
   * One side of the top as geometry: the edge it runs along, and what leads down from it. The top's
   * corners go counterclockwise, so the platform is on the left of the edge and the way out of it
   * is to the right.
   *
   * @param from the corner the edge starts at, in the field frame
   * @param to the corner it ends at, the next one counterclockwise
   * @param side whether a ramp or a vertical edge leads down from it
   */
  private record SideEdge(Translation2d from, Translation2d to, Side side) {
    /** The edge as a vector, from its start to its end. */
    Translation2d along() {
      return to.minus(from);
    }

    /** The way out of the top across the edge: a quarter turn right of along it. */
    Rotation2d outward() {
      return along().getAngle().orElseThrow().minus(Rotation2d.CCW_PI_2);
    }

    /** From any point on the edge to the point of the ramp's foot straight out from it. */
    Translation2d toFoot() {
      return new Translation2d(side.rampMeters(), outward());
    }

    /**
     * How far a point is out from the edge's line, in meters: positive outside the top, negative on
     * the top's side of the line.
     */
    double distanceOut(Translation2d point) {
      return point.minus(from).rotateBy(outward().unaryMinus()).getX();
    }

    /**
     * Whether a point is over the side's ramp, the ramp's own edges included: beside the edge
     * rather than past either end of it, and between the edge and the ramp's foot.
     */
    boolean rampIsUnder(Translation2d point) {
      if (!side.ramped()) {
        return false;
      }
      Translation2d along = along();
      double length = along.getNorm();
      double distanceAlong = point.minus(from).dot(along) / length;
      double distanceOut = distanceOut(point);
      boolean besideTheEdge = distanceAlong >= 0 && distanceAlong <= length;
      boolean betweenEdgeAndFoot = distanceOut >= 0 && distanceOut <= side.rampMeters();
      return besideTheEdge && betweenEdgeAndFoot;
    }
  }

  public Platform {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a platform needs a name");
    }
    if (!top.convex()) {
      throw new IllegalArgumentException(name + ": a platform's top must be convex");
    }
    if (!(heightMeters > 0) || Double.isInfinite(heightMeters)) {
      throw new IllegalArgumentException(name + ": a platform's height must be positive");
    }
    sides = List.copyOf(sides);
    if (sides.size() != top.corners().size()) {
      throw new IllegalArgumentException(
          name
              + ": a platform needs a side for each of its top's "
              + top.corners().size()
              + " edges, not "
              + sides.size());
    }
    for (int corner = 0; corner < sides.size(); corner++) {
      SideEdge arriving = sideEdge(top, sides, corner - 1);
      SideEdge leaving = sideEdge(top, sides, corner);
      boolean betweenTwoRamps = arriving.side().ramped() && leaving.side().ramped();
      if (betweenTwoRamps && goesStraightOn(arriving.along(), leaving.along())) {
        throw new IllegalArgumentException(
            name
                + ": the top's corner "
                + corner
                + " goes straight on between two ramps: make them one side, or one an edge");
      }
    }
  }

  /**
   * Whether the outline does not turn left at a corner, from the edge arriving at it to the edge
   * leaving it. Two ramps on either side of such a corner would lie on top of each other.
   *
   * <p>The cross product of the two edges is the product of their lengths and the sine of the angle
   * the outline turns by, so comparing it with the lengths' product compares the sine.
   */
  private static boolean goesStraightOn(Translation2d arriving, Translation2d leaving) {
    return arriving.cross(leaving) <= STRAIGHT_ON * arriving.getNorm() * leaving.getNorm();
  }

  /** A platform covered in the field's carpet. */
  public Platform(String name, Polygon2d top, double heightMeters, List<Side> sides) {
    this(name, top, heightMeters, sides, Material.CARPET);
  }

  /**
   * The top's height on the top, a ramp's height on a ramp, and the floor's anywhere else. Where
   * two ramps overlap, which they can only do past a corner, the higher one is the ground.
   */
  @Override
  public double heightAt(Translation2d point) {
    if (top.contains(point)) {
      return heightMeters;
    }
    double height = 0;
    for (SideEdge edge : sideEdges()) {
      height = Math.max(height, rampHeight(edge, point));
    }
    return height;
  }

  /** Straight up on the top; leaning out from the side on a ramp; straight up off it. */
  @Override
  public Vector3 normalAt(Translation2d point) {
    if (top.contains(point)) {
      return Vector3.UP;
    }
    Optional<SideEdge> ramp = highestRampUnder(point);
    if (ramp.isEmpty()) {
      return Vector3.UP;
    }
    // A ramp falls by its slope for each meter out from its side, so its surface faces up and
    // out: the normal of a plane that falls at a slope s in a direction is (s in that direction,
    // 1 up), made a unit vector.
    double slope = heightMeters / ramp.get().side().rampMeters();
    Rotation2d outward = ramp.get().outward();
    return new Vector3(slope * outward.getCos(), slope * outward.getSin(), 1).unit();
  }

  /**
   * The ramp that is the ground at a point: of the ramps over it, the one that is highest there, as
   * {@link #heightAt} takes it. Of two equally high, it is the one whose side comes first.
   */
  private Optional<SideEdge> highestRampUnder(Translation2d point) {
    Optional<SideEdge> highest = Optional.empty();
    double highestHeight = Double.NEGATIVE_INFINITY;
    for (SideEdge edge : sideEdges()) {
      if (edge.rampIsUnder(point) && rampHeight(edge, point) > highestHeight) {
        highestHeight = rampHeight(edge, point);
        highest = Optional.of(edge);
      }
    }
    return highest;
  }

  @Override
  public double steepestSlopeRadians() {
    double steepest = 0;
    for (Side side : sides) {
      if (side.ramped()) {
        steepest = Math.max(steepest, Math.atan2(heightMeters, side.rampMeters()));
      }
    }
    return steepest;
  }

  /** The top, and each ramp out from it; the floor between two ramps at a corner left out. */
  @Override
  public Polygon2d footprint() {
    List<Translation2d> outline = new ArrayList<>();
    for (SideEdge edge : sideEdges()) {
      outline.add(edge.from());
      if (edge.side().ramped()) {
        outline.add(edge.from().plus(edge.toFoot()));
        outline.add(edge.to().plus(edge.toFoot()));
      }
    }
    return new Polygon2d(outline);
  }

  /** Its top, and for each side a wall down to the floor, or a ramp with a triangle at each end. */
  @Override
  public TriangleMesh mesh() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    // The top's corners go in first, so corner i of the top is corner i of the mesh, and the
    // top's own triangles can be used as they are.
    int corners = top.corners().size();
    for (int corner = 0; corner < corners; corner++) {
      mesh.corner(top.corner(corner).getX(), top.corner(corner).getY(), heightMeters);
    }
    for (TriangleMesh.Triangle triangle : top.triangles()) {
      mesh.triangle(triangle.a(), triangle.b(), triangle.c());
    }
    List<SideEdge> edges = sideEdges();
    for (int number = 0; number < corners; number++) {
      SideEdge edge = edges.get(number);
      int topFrom = number;
      int topTo = (number + 1) % corners;
      int floorFrom = floorCorner(mesh, edge.from());
      int floorTo = floorCorner(mesh, edge.to());
      if (!edge.side().ramped()) {
        mesh.quad(floorFrom, floorTo, topTo, topFrom);
        continue;
      }
      int footFrom = floorCorner(mesh, edge.from().plus(edge.toFoot()));
      int footTo = floorCorner(mesh, edge.to().plus(edge.toFoot()));
      mesh.quad(topFrom, footFrom, footTo, topTo)
          .triangle(topFrom, floorFrom, footFrom)
          .triangle(topTo, footTo, floorTo);
    }
    return mesh.build();
  }

  /** Each ramp's top edge, named for its side: the way across it is the way up. */
  @Override
  public List<Crossing> crossings() {
    List<Crossing> crossings = new ArrayList<>();
    List<SideEdge> edges = sideEdges();
    for (int number = 0; number < edges.size(); number++) {
      SideEdge edge = edges.get(number);
      if (edge.side().ramped()) {
        // People count sides from one.
        String sideName = name + ", side " + (number + 1);
        crossings.add(new Crossing(sideName, edge.from(), edge.to(), Crossing.Kind.TOP));
      }
    }
    return List.copyOf(crossings);
  }

  /** Every side of the top as geometry, in the order the sides are given. */
  private List<SideEdge> sideEdges() {
    List<SideEdge> edges = new ArrayList<>();
    for (int number = 0; number < sides.size(); number++) {
      edges.add(sideEdge(top, sides, number));
    }
    return edges;
  }

  /**
   * One side of a top as geometry. Sides are numbered as the top's corners are, and wrap round as
   * they do: side -1 is the last side.
   */
  private static SideEdge sideEdge(Polygon2d top, List<Side> sides, int number) {
    Side side = sides.get(Math.floorMod(number, sides.size()));
    return new SideEdge(top.corner(number), top.corner(number + 1), side);
  }

  /**
   * How high a side's ramp is at a point: the top's height at the side, falling in a straight line
   * to zero at the ramp's foot. It is zero off the ramp, and for a side with no ramp.
   */
  private double rampHeight(SideEdge edge, Translation2d point) {
    if (!edge.rampIsUnder(point)) {
      return 0;
    }
    double distanceOut = edge.distanceOut(point);
    return heightMeters * (1 - distanceOut / edge.side().rampMeters());
  }

  /** Adds a corner on the floor to a mesh, and gives its number in the mesh. */
  private static int floorCorner(TriangleMesh.Builder mesh, Translation2d point) {
    return mesh.corner(point.getX(), point.getY(), 0);
  }
}
