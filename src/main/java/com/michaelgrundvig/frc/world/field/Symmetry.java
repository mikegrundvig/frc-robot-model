package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * How the red alliance's half of the field relates to the blue alliance's. Fields have been one of
 * two kinds, and which one changes by season, so each season's field model declares it. WPILib's
 * tag layouts show 2022, 2025, and 2026 rotated, and 2023 and 2024 mirrored.
 *
 * <p>Each applies to a whole pose, position and heading together, and applying it twice gives back
 * what it started with. It turns nominal geometry, written for blue, into red's: places, zones,
 * surfaces, elements, and the tags' official poses. What was measured, a survey's tags, is never
 * turned: a survey measures each tag where it stands, on both halves.
 */
public enum Symmetry {
  /** Red's half is blue's reflected across the center line, x = length / 2: 2023 and 2024. */
  MIRRORED,

  /** Red's half is blue's turned half a turn about the center of the field: 2022, 2025, 2026. */
  ROTATED;

  /** Where a point on one alliance's half is on the other's. */
  public Translation2d apply(Translation2d point, double length, double width) {
    return switch (this) {
      case MIRRORED -> new Translation2d(length - point.getX(), point.getY());
      case ROTATED -> new Translation2d(length - point.getX(), width - point.getY());
    };
  }

  /** Which way a heading on one alliance's half points on the other's. */
  public Rotation2d apply(Rotation2d heading) {
    return switch (this) {
      case MIRRORED -> Rotation2d.PI.minus(heading);
      case ROTATED -> heading.plus(Rotation2d.PI);
    };
  }

  /** Where a pose on one alliance's half is on the other's. */
  public Pose2d apply(Pose2d pose, double length, double width) {
    return new Pose2d(apply(pose.getTranslation(), length, width), apply(pose.getRotation()));
  }

  /**
   * Where a pose in 3D is on the other half, at the same height. Its x axis is turned as a heading
   * is ({@link #apply(Rotation2d)}), so a tag's image faces where the tag's face would.
   *
   * <p>On a mirrored field there is one thing to know. The true mirror image of a frame is
   * left-handed, which no pose can be. So the image's y axis is reversed to make it right-handed
   * again: its x and z axes are the mirror images of the pose's, and its y axis points opposite to
   * the mirror image of the pose's.
   */
  public Pose3d apply(Pose3d pose, double length, double width) {
    Translation2d image = apply(pose.getTranslation().toTranslation2d(), length, width);
    return new Pose3d(image.getX(), image.getY(), pose.getZ(), apply(pose.getRotation()));
  }

  /**
   * Which way a frame in 3D is turned on the other half. This is the rotation part of {@link
   * #apply(Pose3d, double, double)}, which explains it.
   */
  public Rotation3d apply(Rotation3d rotation) {
    return switch (this) {
      // Reflecting a rotation across the plane x = 0 mirrors its axis and reverses the way it
      // turns, which takes its quaternion (w, x, y, z) to (w, x, -y, -z). That maps the frame's
      // x axis to the opposite of where it belongs and leaves y mirrored, so a half turn about z
      // is applied to the frame first: it reverses both, leaving x mirrored and y reversed.
      case MIRRORED -> {
        Quaternion q = rotation.getQuaternion();
        yield HALF_TURN.rotateBy(
            new Rotation3d(new Quaternion(q.getW(), q.getX(), -q.getY(), -q.getZ())));
      }
      case ROTATED -> rotation.rotateBy(HALF_TURN);
    };
  }

  /**
   * An offset from a frame, as it has to be written from the frame's image on the other half so
   * that it leads to the image of where it led. Something placed at an offset from a tag on blue's
   * half is then placed, at this offset from the tag's partner, on its own image.
   *
   * <p>On a turned field the offset is the same. On a mirrored field it is mirrored across the
   * frame's x-z plane, because the image's y axis runs the other way ({@link #apply(Pose3d, double,
   * double)}).
   */
  public Transform3d inImage(Transform3d offset) {
    if (!mirrors()) {
      return offset;
    }
    Translation3d move = offset.getTranslation();
    // Reversing the frame's y axis mirrors the rotation across the x-z plane: its axis is
    // mirrored and the way it turns is reversed, which takes (w, x, y, z) to (w, -x, y, -z).
    Quaternion q = offset.getRotation().getQuaternion();
    return new Transform3d(
        new Translation3d(move.getX(), -move.getY(), move.getZ()),
        new Rotation3d(new Quaternion(q.getW(), -q.getX(), q.getY(), -q.getZ())));
  }

  /** Where a point in 3D is on the other half: as on the floor, at the same height. */
  public Vector3 apply(Vector3 point, double length, double width) {
    Translation2d image = apply(new Translation2d(point.x(), point.y()), length, width);
    return new Vector3(image.getX(), image.getY(), point.z());
  }

  /**
   * An outline on the other half. A mirror image of a counterclockwise outline runs clockwise, so
   * on a mirrored field the corners are listed backwards to keep them counterclockwise. The first
   * corner is still first, and edge {@code i} of the image is the image of edge {@code n - 1 - i}.
   */
  public Polygon2d apply(Polygon2d polygon, double length, double width) {
    List<Translation2d> corners = new ArrayList<>();
    for (int index = 0; index < polygon.corners().size(); index++) {
      // A polygon's corners wrap round, so corner -1 is its last: going down from zero lists
      // them backwards, starting from the same first corner.
      int original = index;
      if (mirrors()) {
        original = -index;
      }
      corners.add(apply(polygon.corner(original), length, width));
    }
    return new Polygon2d(corners);
  }

  /** A shape on the other half, its triangles still facing out. */
  public TriangleMesh apply(TriangleMesh mesh, double length, double width) {
    return mesh.map(corner -> apply(corner, length, width), mirrors());
  }

  /**
   * A crossing on the other half. The way across a crossing is a quarter turn left of the way along
   * it, and a mirror image turns left into right. So on a mirrored field the crossing's ends are
   * swapped, which keeps the way across the image the image of the way across: the top of a ramp
   * still faces up the ramp.
   */
  public Crossing apply(Crossing crossing, double length, double width) {
    Translation2d start = apply(crossing.start(), length, width);
    Translation2d end = apply(crossing.end(), length, width);
    if (mirrors()) {
      return new Crossing(crossing.name(), end, start, crossing.kind());
    }
    return new Crossing(crossing.name(), start, end, crossing.kind());
  }

  /** A surface on the other half, under the same name. */
  public Surface apply(Surface surface, double length, double width) {
    return switch (surface) {
      case Bump bump -> {
        Crossing crest = apply(bump.crest(), length, width);
        yield new Bump(
            bump.name(),
            crest.start(),
            crest.end(),
            bump.heightMeters(),
            bump.depthMeters(),
            bump.material());
      }
      case Ramp ramp ->
          new Ramp(
              ramp.name(),
              apply(ramp.foot(), length, width),
              apply(ramp.top(), length, width),
              ramp.widthMeters(),
              ramp.heightMeters(),
              ramp.material());
      case Platform platform -> {
        // The top's mirror image has its corners listed backwards (see apply(Polygon2d)), so its
        // edge i is the image of edge n - 1 - i, and the sides are listed backwards to match.
        List<Platform.Side> sides = new ArrayList<>(platform.sides());
        if (mirrors()) {
          Collections.reverse(sides);
        }
        yield new Platform(
            platform.name(),
            apply(platform.top(), length, width),
            platform.heightMeters(),
            sides,
            platform.material());
      }
      case MeshSurface mesh ->
          new MeshSurface(
              mesh.name(),
              apply(mesh.mesh(), length, width),
              mesh.crossings().stream().map(each -> apply(each, length, width)).toList(),
              mesh.material());
    };
  }

  private static final Rotation3d HALF_TURN = new Rotation3d(0, 0, Math.PI);

  /** Whether it's a mirror image, which turns counterclockwise into clockwise. */
  boolean mirrors() {
    return this == MIRRORED;
  }
}
