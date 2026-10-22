package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.physical.Bumpers;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import java.util.ArrayList;
import java.util.List;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.params.provider.Arguments;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * A small made-up field, 16 x 8 m and turned half a turn from blue's half to red's, with a few tags
 * that are easy to check by hand:
 *
 * <ul>
 *   <li>2 and 3: a "hub" on blue's half, faces at x = 4 (facing blue's wall) and x = 5.
 *   <li>12 and 13: red's hub, their images: faces at x = 12 and x = 11.
 *   <li>4 on blue's wall at y = 2, and 14 on red's wall at y = 6, its image.
 * </ul>
 */
final class TestFields {
  static final double LENGTH = 16;
  static final double WIDTH = 8;

  /** The test field with no starts, places or zones. */
  static final FieldModel FIELD = model();

  private TestFields() {}

  static List<FieldTag> tags() {
    List<FieldTag> tags = new ArrayList<>();
    tags.add(tag(2, 4, 4, 180));
    tags.add(tag(3, 5, 4, 0));
    tags.add(tag(12, 12, 4, 0));
    tags.add(tag(13, 11, 4, 180));
    tags.add(tag(4, 0, 2, 0));
    tags.add(tag(14, 16, 6, 180));
    return tags;
  }

  static Field layout(List<FieldTag> tags) {
    return new Field("Test field", "2027", "Test", null, LENGTH, WIDTH, "frc", tags);
  }

  static Field layout() {
    return layout(tags());
  }

  /** A tag standing upright, facing the given way. */
  static FieldTag tag(int id, double x, double y, double degrees) {
    return new FieldTag(
        id, new Pose3d(new Translation3d(x, y, 1), new Rotation3d(0, 0, Math.toRadians(degrees))));
  }

  /** A robot's bumpers, for what stands a robot on a start. */
  static final Bumpers BUMPERS = new Bumpers(Meters.of(0.9), Meters.of(0.8));

  static FieldModel model(List<Start> starts, List<Place> places, List<Zone> zones) {
    return new FieldModel(layout(), Symmetry.ROTATED, starts, places, zones);
  }

  static FieldModel model() {
    return model(List.of(), List.of(), List.of());
  }

  static Translation2d xy(double x, double y) {
    return new Translation2d(x, y);
  }

  /** A polygon from its corners' x and y, counterclockwise. */
  static Polygon2d polygon(double... xy) {
    List<Translation2d> corners = new ArrayList<>();
    for (int i = 0; i < xy.length; i += 2) {
      corners.add(xy(xy[i], xy[i + 1]));
    }
    return new Polygon2d(corners);
  }

  static Pose2d pose(double x, double y, double degrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }

  /**
   * A platform like 2023's charge station: a 2 m square top 25 cm up, from (4, 2) to (6, 4), ramps
   * 0.5 m long down its front and back (toward -x and +x), and edges on its sides.
   */
  static final Platform STATION =
      new Platform(
          "Station",
          Polygon2d.rectangle(xy(4, 2), xy(6, 4)),
          0.25,
          List.of(
              Platform.Side.EDGE,
              Platform.Side.ramp(0.5),
              Platform.Side.EDGE,
              Platform.Side.ramp(0.5)));

  /** A ridge from CAD, closed at its ends and underneath: a crest along y at x = 14.5. */
  static MeshSurface tent() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int frontLeft = mesh.corner(14, 1, 0);
    int frontRight = mesh.corner(15, 1, 0);
    int frontTop = mesh.corner(14.5, 1, 0.2);
    int backLeft = mesh.corner(14, 2.5, 0);
    int backRight = mesh.corner(15, 2.5, 0);
    int backTop = mesh.corner(14.5, 2.5, 0.2);
    TriangleMesh tent =
        mesh.quad(frontLeft, frontTop, backTop, backLeft)
            .quad(frontTop, frontRight, backRight, backTop)
            .triangle(frontLeft, frontRight, frontTop)
            .triangle(backLeft, backTop, backRight)
            .quad(frontLeft, backLeft, backRight, frontRight)
            .build();
    return new MeshSurface(
        "Tent",
        tent,
        List.of(new Crossing("Tent crest", xy(14.5, 1), xy(14.5, 2.5), Crossing.Kind.CREST)));
  }

  /** A row of a table of refusals: what was asked, and the message that refuses it. */
  static Arguments refused(String label, ThrowingCallable building, String message) {
    return Arguments.argumentSet(label, building, message);
  }

  static void assertPose(Pose2d actual, double x, double y, double degrees) {
    assertThat(actual.getX()).as("x of " + actual).isCloseTo(x, within(1e-6));
    assertThat(actual.getY()).as("y of " + actual).isCloseTo(y, within(1e-6));
    assertThat(Math.IEEEremainder(actual.getRotation().getDegrees() - degrees, 360))
        .as("heading of " + actual + ", expected " + degrees + "°")
        .isCloseTo(0.0, within(1e-6));
  }
}
