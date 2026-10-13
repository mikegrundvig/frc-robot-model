package com.michaelgrundvig.frc.world.game;

import com.michaelgrundvig.frc.world.field.Block;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/** The blocks the seasons' elements are drawn with, in the field frame. */
final class FieldShapes {
  private FieldShapes() {}

  /**
   * A box between two corners, its sides along the field's axes.
   *
   * @param x0 one side along the field, and {@code x1} the other
   * @param y0 one side across the field, and {@code y1} the other
   * @param bottom its underside above the floor, and {@code top} its top
   */
  static Block box(
      double x0, double x1, double y0, double y1, double bottom, double top, Material material) {
    Polygon2d outline = Polygon2d.rectangle(new Translation2d(x0, y0), new Translation2d(x1, y1));
    return new Block(outline, bottom, top, material);
  }

  /**
   * A square post, centered on a point and turned to a heading.
   *
   * @param size across each of its sides
   * @param bottom its underside above the floor, and {@code top} its top
   */
  static Block post(
      Translation2d center,
      Rotation2d heading,
      double size,
      double bottom,
      double top,
      Material material) {
    Polygon2d outline = Polygon2d.rectangle(new Pose2d(center, heading), size, size);
    return new Block(outline, bottom, top, material);
  }

  /**
   * The corner of the field a slanted face cuts off, seen from above: between the two walls that
   * meet there and the face's line, which runs through a point on it, square to the way it faces.
   *
   * @param corner where the alliance wall meets the guardrail
   * @param onFace a point on the face, such as its tag
   * @param facing the way the face looks, into the field
   */
  static Polygon2d cutOff(Translation2d corner, Translation2d onFace, Rotation2d facing) {
    Rotation2d alongTheFace = facing.plus(Rotation2d.CCW_PI_2);
    // Going along the face from the point, each meter moves the cosine of that direction along x
    // and its sine along y. So the face meets the alliance wall (the corner's x) and the guardrail
    // (the corner's y) this many meters along it.
    double toWall = (corner.getX() - onFace.getX()) / alongTheFace.getCos();
    double toGuardrail = (corner.getY() - onFace.getY()) / alongTheFace.getSin();
    Translation2d atWall =
        new Translation2d(corner.getX(), onFace.getY() + toWall * alongTheFace.getSin());
    Translation2d atGuardrail =
        new Translation2d(onFace.getX() + toGuardrail * alongTheFace.getCos(), corner.getY());
    // A polygon's corners run counterclockwise: which of the two comes after the corner depends on
    // which corner of the field it is.
    boolean wallThenGuardrailTurnsLeft = atWall.minus(corner).cross(atGuardrail.minus(corner)) > 0;
    if (wallThenGuardrailTurnsLeft) {
      return new Polygon2d(List.of(corner, atWall, atGuardrail));
    }
    return new Polygon2d(List.of(corner, atGuardrail, atWall));
  }
}
