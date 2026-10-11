package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * A ramp on its own: one slope up from the floor to a height, then a drop straight back down (a
 * wedge). A ramp leading onto a platform is one of the {@link Platform}'s sides instead. Its top
 * edge is a crossing: a robot that drives up it is there when the slope ends.
 *
 * @param name for the log
 * @param foot the middle of its bottom edge, on the floor, in the field frame
 * @param top the middle of its top edge, seen from above: it climbs from the foot toward here
 * @param widthMeters along its edges, across the way it climbs
 * @param heightMeters how high its top edge stands above the floor
 * @param material what it's covered in
 */
public record Ramp(
    String name,
    Translation2d foot,
    Translation2d top,
    double widthMeters,
    double heightMeters,
    Material material)
    implements Surface {
  public Ramp {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a ramp needs a name");
    }
    if (!(foot.getDistance(top) > 0)) {
      throw new IllegalArgumentException(name + ": a ramp climbs between two different points");
    }
    if (!(widthMeters > 0)
        || !(heightMeters > 0)
        || Double.isInfinite(widthMeters)
        || Double.isInfinite(heightMeters)) {
      throw new IllegalArgumentException(name + ": a ramp's width and height must be positive");
    }
  }

  /** A ramp covered in the field's carpet. */
  public Ramp(
      String name, Translation2d foot, Translation2d top, double widthMeters, double heightMeters) {
    this(name, foot, top, widthMeters, heightMeters, Material.CARPET);
  }

  /** How far it runs, seen from above, from its foot to its top. */
  public double runMeters() {
    return foot.getDistance(top);
  }

  /** The way it climbs, from its foot toward its top. */
  public Rotation2d up() {
    return top.minus(foot).getAngle().orElseThrow();
  }

  /** How steep it is. */
  public double slopeRadians() {
    return Math.atan2(heightMeters, runMeters());
  }

  @Override
  public double steepestSlopeRadians() {
    return slopeRadians();
  }

  /** Rising in a straight line from the floor at the foot to its height at the top edge. */
  @Override
  public double heightAt(Translation2d point) {
    Translation2d fromFoot = inRampFrame(point);
    if (!on(fromFoot)) {
      return 0;
    }
    return heightMeters * fromFoot.getX() / runMeters();
  }

  /**
   * A point on the field as the ramp sees it: from the middle of its foot, with x the way it climbs
   * and y to its left.
   */
  private Translation2d inRampFrame(Translation2d point) {
    return point.minus(foot).rotateBy(up().unaryMinus());
  }

  /** On the ramp the ground faces up and back toward the foot. Off it, it faces straight up. */
  @Override
  public Vector3 normalAt(Translation2d point) {
    if (!on(inRampFrame(point))) {
      return Vector3.UP;
    }
    // The ground rises at the ramp's slope the way it climbs, so it faces up and back toward
    // the foot: the normal of a plane that rises at a slope s in a direction is (-s in that
    // direction, 1 up), made a unit vector.
    double slope = heightMeters / runMeters();
    Rotation2d up = up();
    return new Vector3(-slope * up.getCos(), -slope * up.getSin(), 1).unit();
  }

  /** The rectangle from its foot to its top edge. */
  @Override
  public Polygon2d footprint() {
    Translation2d side = side();
    return new Polygon2d(
        List.of(foot.minus(side), top.minus(side), top.plus(side), foot.plus(side)));
  }

  /**
   * The ramp as four faces: its slope, its back wall straight down from the top edge, and a
   * triangle closing each side. "Left" and "right" are as seen climbing it. The corners named "top"
   * are on the floor under the top edge, and those named "up" are the top edge itself.
   */
  @Override
  public TriangleMesh mesh() {
    Translation2d side = side();
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int footRight = floor(mesh, foot.minus(side));
    int topRight = floor(mesh, top.minus(side));
    int topLeft = floor(mesh, top.plus(side));
    int footLeft = floor(mesh, foot.plus(side));
    int upRight = mesh.corner(top.minus(side).getX(), top.minus(side).getY(), heightMeters);
    int upLeft = mesh.corner(top.plus(side).getX(), top.plus(side).getY(), heightMeters);
    return mesh.quad(footRight, upRight, upLeft, footLeft)
        .quad(topRight, topLeft, upLeft, upRight)
        .triangle(footRight, topRight, upRight)
        .triangle(footLeft, upLeft, topLeft)
        .build();
  }

  /** Its top edge: the way across it is the way up. */
  @Override
  public List<Crossing> crossings() {
    Translation2d side = side();
    return List.of(new Crossing(name, top.plus(side), top.minus(side), Crossing.Kind.TOP));
  }

  /** Half its width, toward its left as it climbs. */
  private Translation2d side() {
    return new Translation2d(widthMeters / 2, up().plus(Rotation2d.CCW_PI_2));
  }

  /** Whether a point, given in the ramp's own frame ({@link #inRampFrame}), is over the ramp. */
  private boolean on(Translation2d fromFoot) {
    boolean betweenFootAndTop = fromFoot.getX() >= 0 && fromFoot.getX() <= runMeters();
    boolean betweenItsSides = Math.abs(fromFoot.getY()) <= widthMeters / 2;
    return betweenFootAndTop && betweenItsSides;
  }

  /** Adds a corner on the floor to a mesh, and gives its number in the mesh. */
  private static int floor(TriangleMesh.Builder mesh, Translation2d point) {
    return mesh.corner(point.getX(), point.getY(), 0);
  }
}
