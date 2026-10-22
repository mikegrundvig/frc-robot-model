package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.STATION;
import static com.michaelgrundvig.frc.world.field.TestFields.polygon;
import static com.michaelgrundvig.frc.world.field.TestFields.pose;
import static com.michaelgrundvig.frc.world.field.TestFields.xy;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The field's decisions at their edges, as tables of questions and answers: which tags are
 * partners, where a platform's ramps begin and end, which triangles a lookup looks at, where a
 * heading stops having a bearing, and which start a robot is on. Each row is exactly on an edge, or
 * a step to one side of it.
 */
class FieldEdgesTest {
  @ParameterizedTest
  @MethodSource({"partners", "platformEdges", "lookups", "ground", "headings", "placements"})
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments answers(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  /** A tag standing upright on the floor's plane, facing the given way. */
  private static Tag tag(int id, double x, double y, double degrees) {
    return Tag.nominal(id, new Pose3d(x, y, 0, new Rotation3d(0, 0, Math.toRadians(degrees))));
  }

  /**
   * The partners on a mirrored field 16 m long: tag 1 is at (1, 1) facing along +x, so its image is
   * at (15, 1) facing along -x. The other tags are candidates for its partner.
   */
  private static TagPartners mirrored(Tag... candidates) {
    List<Tag> tags = new ArrayList<>();
    tags.add(tag(1, 1, 1, 0));
    tags.addAll(List.of(candidates));
    return new TagPartners(tags, Symmetry.MIRRORED, 16, 8);
  }

  // spotless:off

  /** Which tag is a tag's partner, and how far the layout is from its symmetry. */
  static Stream<Arguments> partners() {
    return Stream.of(
        answers("the tag at its image is its partner",
            () -> mirrored(tag(2, 15, 1, 180)).partnerOf(1), Optional.of(2)),
        answers("...and it is that tag's partner in turn",
            () -> mirrored(tag(2, 15, 1, 180)).partnerOf(2), Optional.of(1)),
        answers("a tag exactly 25 cm from the image is still its partner",
            () -> mirrored(tag(2, 15, 1.25, 180)).partnerOf(1), Optional.of(2)),
        answers("a tag a hair further is not",
            () -> mirrored(tag(2, 15, 1.2500001, 180)).partnerOf(1), Optional.empty()),
        answers("a tag at the image facing exactly 20° off is its partner",
            () -> new TagPartners(List.of(tag(1, 1, 1, 180), tag(2, 15, 1, 20)), Symmetry.MIRRORED, 16, 8).partnerOf(1),
            Optional.of(2)),
        answers("a tag at the image facing 19° off is its partner",
            () -> mirrored(tag(2, 15, 1, 161)).partnerOf(1), Optional.of(2)),
        answers("a tag at the image facing 21° off is not",
            () -> mirrored(tag(2, 15, 1, 159)).partnerOf(1), Optional.empty()),
        answers("of two tags equally near the image, the one listed first",
            () -> mirrored(tag(2, 15, 1.25, 180), tag(3, 15, 0.75, 180)).partnerOf(1), Optional.of(2)),
        answers("the nearer tag, though listed second",
            () -> mirrored(tag(2, 15, 1.25, 180), tag(3, 15, 0.875, 180)).partnerOf(1), Optional.of(3)),
        answers("a tag that is not in the layout has none",
            () -> mirrored(tag(2, 15, 1, 180)).partnerOf(9), Optional.empty()),
        answers("a layout with the symmetry is no distance from it",
            () -> mirrored(tag(2, 15, 1, 180)).symmetryError(), 0.0),
        answers("the layout is as far from its symmetry as its worst tag",
            () -> mirrored(tag(2, 15, 1.25, 180)).symmetryError(), 0.25),
        answers("a tag whose image nothing faces makes that infinite",
            () -> mirrored(tag(2, 15, 1, 90)).symmetryError(), Double.POSITIVE_INFINITY),
        answers("the field's edges are on the field",
            () -> List.of(TestFields.FIELD.contains(xy(0, 3)), TestFields.FIELD.contains(xy(3, 0)),
                TestFields.FIELD.contains(xy(16, 8)), TestFields.FIELD.contains(xy(-1e-9, 3)),
                TestFields.FIELD.contains(xy(3, -1e-9)), TestFields.FIELD.contains(xy(16.000001, 3)),
                TestFields.FIELD.contains(xy(3, 8.000001))),
            List.of(true, true, true, false, false, false, false)));
  }

  /** A top 4 by 1 m whose first two sides go straight on at its corner 1, more or less. */
  private static String straightOn(double bend, Platform.Side second) {
    Polygon2d top = polygon(0, 0, 2, 0, 4, bend, 4, 1, 0, 1);
    Platform.Side edge = Platform.Side.EDGE;
    try {
      new Platform("P", top, 1, List.of(Platform.Side.ramp(1), second, edge, edge, edge));
      return "accepted";
    } catch (IllegalArgumentException e) {
      return String.valueOf(e.getMessage());
    }
  }

  /**
   * A platform 0.25 m high whose top is the square from (4, 2) to (6, 4), with one ramp, down its
   * side along y = 2 to a foot at y = 1.5. That side runs along x, which keeps the sums below
   * exact.
   */
  private static Platform rampedToTheSouth() {
    Platform.Side edge = Platform.Side.EDGE;
    Polygon2d top = Polygon2d.rectangle(xy(4, 2), xy(6, 4));
    return new Platform("South", top, 0.25, List.of(Platform.Side.ramp(0.5), edge, edge, edge));
  }

  /** Where a platform's ramp begins and ends, and which corners may have a ramp on both sides. */
  static Stream<Arguments> platformEdges() {
    String refused = "P: the top's corner 1 goes straight on between two ramps: make them one side, or one an edge";
    return Stream.of(
        answers("in line with the start of a ramp's side is on the ramp",
            () -> rampedToTheSouth().heightAt(xy(4, 1.75)), 0.125),
        answers("a hair before it is off the ramp", () -> rampedToTheSouth().heightAt(xy(3.999999, 1.75)), 0.0),
        answers("in line with the side's end is on the ramp",
            () -> rampedToTheSouth().heightAt(xy(6, 1.75)) > 0.12, true),
        answers("a hair past it is off the ramp", () -> rampedToTheSouth().heightAt(xy(6.000001, 1.75)), 0.0),
        answers("a ramp's foot is on the ramp, at the floor's height",
            () -> rampedToTheSouth().heightAt(xy(4, 1.5)), 0.0),
        answers("...and faces as the ramp does", () -> rampedToTheSouth().normalAt(xy(4, 1.5)).y() < -0.4, true),
        answers("a hair past the foot faces straight up",
            () -> rampedToTheSouth().normalAt(xy(4, 1.499999)), Vector3.UP),
        answers("the mesh has the top's corners and four on the floor for each side",
            () -> STATION.mesh().corners().size(), 4 + 2 * 2 + 2 * 4),
        answers("two ramps round a corner that turns by a sine of 1e-9 times a hair are refused",
            () -> straightOn(1e-9, Platform.Side.ramp(1)), refused),
        answers("...and so are two round a corner that turns by exactly 1e-9",
            () -> straightOn(2e-9, Platform.Side.ramp(1)), refused),
        answers("two ramps round a corner that turns by a little more are accepted",
            () -> straightOn(1e-8, Platform.Side.ramp(1)), "accepted"),
        answers("how far a corner turns is judged by its angle, not by its sides' lengths",
            () -> straightOn(2.5e-9, Platform.Side.ramp(1)), "accepted"),
        answers("a ramp and an edge round a straight corner are accepted",
            () -> straightOn(0, Platform.Side.EDGE), "accepted"));
  }

  /**
   * A flat sheet 2 m square at a height, cut into four squares of two triangles each. Its corner
   * nearest the field's origin is at (3, 5).
   */
  private static TriangleMesh sheet() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int[][] corner = new int[3][3];
    for (int column = 0; column <= 2; column++) {
      for (int row = 0; row <= 2; row++) {
        corner[column][row] = mesh.corner(3 + column, 5 + row, 0.1);
      }
    }
    for (int column = 0; column < 2; column++) {
      for (int row = 0; row < 2; row++) {
        mesh.quad(corner[column][row], corner[column + 1][row], corner[column + 1][row + 1], corner[column][row + 1]);
      }
    }
    return mesh.build();
  }

  /** The triangles looked at for a point, given from the sheet's corner. */
  private static List<Integer> near(double x, double y) {
    TriangleMesh sheet = sheet();
    List<Integer> all = List.of(0, 1, 2, 3, 4, 5, 6, 7);
    Polygon2d footprint = Polygon2d.rectangle(xy(3, 5), xy(5, 7));
    List<Integer> near = new ArrayList<>();
    for (int triangle : new GroundGrid(sheet, all, footprint).trianglesNear(xy(3 + x, 5 + y))) {
      near.add(triangle);
    }
    return near;
  }

  /**
   * Which triangles a lookup looks at. Eight triangles want two cells, so each cell is √2 m
   * square, and the sheet takes two columns of two. A triangle is listed in every cell its bounds
   * touch, so the squares that straddle the line √2 m in from the sheet's corner, along x or along
   * y, are listed on both sides of it.
   */
  static Stream<Arguments> lookups() {
    return Stream.of(
        answers("the corner cell lists the squares that reach into it", () -> near(0.5, 0.5), List.of(0, 1, 2, 3, 4, 5, 6, 7)),
        answers("the far corner's cell lists only the far square", () -> near(1.9, 1.9), List.of(6, 7)),
        answers("the cell past √2 along x lists the squares on that side", () -> near(1.9, 0.5), List.of(4, 5, 6, 7)),
        answers("the cell past √2 along y lists the squares on that side", () -> near(0.5, 1.9), List.of(2, 3, 6, 7)),
        answers("a point off the grid looks in the nearest cell", () -> near(50, -50), List.of(4, 5, 6, 7)),
        answers("a mesh surface finds its ground through the grid",
            () -> new MeshSurface("Sheet", sheet(), List.of()).heightAt(xy(4.9, 5.1)), 0.1),
        answers("a mesh surface is carpeted unless it says otherwise",
            () -> new MeshSurface("Sheet", sheet(), List.of()).material(), Material.CARPET));
  }

  private static Ramp ramp(String name) {
    return new Ramp(name, xy(2, 2), xy(3, 2), 1, 0.2);
  }

  /** What the ground is where surfaces meet the floor and each other. */
  static Stream<Arguments> ground() {
    return Stream.of(
        answers("at a ramp's foot the ramp itself leans",
            () -> ramp("Ramp").normalAt(xy(2, 2)).equals(Vector3.UP), false),
        answers("...but the ground there is the floor's, and faces straight up",
            () -> new Terrain(List.of(ramp("Ramp"))).normalAt(xy(2, 2)), Vector3.UP),
        answers("of two surfaces a point is on, the one listed first is under it",
            () -> new Terrain(List.of(ramp("First"), ramp("Second"))).under(xy(2.5, 2), 0).map(Surface::name),
            Optional.of("First")),
        answers("on a ridge's crest, where two faces are as high, the ground faces as the first does",
            () -> TestFields.tent().normalAt(xy(14.5, 2)).x() < 0, true),
        answers("a surface whose crossing ends past a wall runs off the field",
            () -> {
              Ramp acrossTheWall = new Ramp("Ramp", xy(3, 7.9), xy(3.5, 7.9), 1, 0.2);
              try {
                return new FieldModel(TestFields.layout(), Symmetry.ROTATED, List.of(), List.of(), List.of(), List.of(acrossTheWall)).name();
              } catch (IllegalArgumentException e) {
                return String.valueOf(e.getMessage());
              }
            },
            "Ramp (blue): it runs off the field"),
        answers("a field says which symmetry it has", () -> TestFields.FIELD.symmetry(), Symmetry.ROTATED),
        answers("the two halves' surfaces are named for the half each is on",
            () -> Terrain.onBothHalves(List.of(ramp("Ramp")), Symmetry.ROTATED, 16, 8).surfaces().stream().map(Surface::name).toList(),
            List.of("Ramp (blue)", "Ramp (red)")),
        answers("a mesh's crossings are renamed with it",
            () -> Terrain.onBothHalves(List.of(TestFields.tent()), Symmetry.ROTATED, 16, 8).crossings().stream().map(Crossing::name).toList(),
            List.of("Tent crest (blue)", "Tent crest (red)")));
  }

  private static final Place CORNER = Place.at("Corner", Pose2d.ZERO);

  private static Optional<Rotation2d> bearingFrom(double x) {
    return Heading.toward(CORNER).resolve(pose(x, 0, 0), TestFields.FIELD, Optional.of(Alliance.BLUE));
  }

  private static double rateFrom(double x) {
    Translation2d sideways = xy(0, 1);
    return Heading.toward(CORNER).rate(pose(x, 0, 0), TestFields.FIELD, Optional.of(Alliance.BLUE), sideways);
  }

  /** Where a heading stops having a bearing, what it is called, and which are the same. */
  static Stream<Arguments> headings() {
    // The angle whose degrees come out as exactly 0.05, the bound for being called "downfield".
    double exactlyOnTheBound = 8.726646259971648E-4;
    return Stream.of(
        answers("exactly 5 cm from a place, the robot has a bearing on it",
            () -> bearingFrom(0.05), Optional.of(Rotation2d.PI)),
        answers("a hair nearer, it has none", () -> bearingFrom(0.0499999), Optional.empty()),
        answers("exactly 5 cm from a place, the bearing turns as the robot passes",
            () -> rateFrom(0.05), 0.05 / (0.05 * 0.05)),
        answers("a hair nearer, it does not", () -> rateFrom(0.0499999), 0.0),
        answers("the bound's own angle is not close enough to be called downfield",
            () -> List.of(Math.toDegrees(exactlyOnTheBound), Heading.downfield(Radians.of(exactlyOnTheBound)).name()),
            List.of(0.05, "0° left of downfield")),
        answers("a hair less is", () -> Heading.downfield(Degrees.of(0.0499)).name(), "downfield"),
        answers("the same heading, made twice, is the same",
            () -> Heading.toward(CORNER).equals(Heading.toward(Place.at("Corner", Pose2d.ZERO))), true),
        answers("...and has the same hash",
            () -> Heading.toward(CORNER).hashCode() == Heading.toward(Place.at("Corner", Pose2d.ZERO)).hashCode(), true),
        answers("...and a hash of its own", () -> Heading.toward(CORNER).hashCode() != 0, true),
        answers("back first is another heading",
            () -> Heading.toward(CORNER).equals(Heading.toward(CORNER).withBack()), false),
        answers("facing another place is another heading",
            () -> Heading.toward(CORNER).equals(Heading.toward(Place.at("Hub", Pose2d.ZERO))), false),
        answers("a heading on the field is not the same angle from downfield",
            () -> Heading.field(Rotation2d.ZERO).equals(Heading.downfield(Degrees.of(0))), false),
        answers("a heading is not some other thing",
            () -> Heading.field(Rotation2d.ZERO).equals("field 0°"), false));
  }

  private static Placement placed(Pose2d robot, @Nullable Start expected, double toleranceMeters, double toleranceDegrees) {
    Start near = Start.at("Near", pose(1, 3, 0));
    Start far = Start.at("Far", pose(1, 5, 0));
    FieldModel field = TestFields.model(List.of(near, far), List.of(), List.of());
    return Placement.check(
        field, TestFields.BUMPERS, Optional.of(Alliance.BLUE), robot, expected, toleranceMeters, toleranceDegrees);
  }

  /** Starts at (1, 3) and (1, 5), and a robot on, between and beside them. */
  static Stream<Arguments> placements() {
    Start near = Start.at("Near", pose(1, 3, 0));
    return Stream.of(
        answers("exactly on a start, with no tolerance at all, is on it",
            () -> placed(pose(1, 3, 0), near, 0, 0), new Placement(Placement.Status.READY, "Near", 0, 0)),
        answers("exactly at the tolerance from a start is on it",
            () -> placed(pose(1, 3.5, 0), near, 0.5, 0).status(), Placement.Status.READY),
        answers("a hair past the tolerance is not",
            () -> placed(pose(1, 3.5000001, 0), near, 0.5, 0).status(), Placement.Status.NOT_ON_A_START),
        answers("turned exactly by the tolerance is on it",
            () -> placed(pose(1, 3, 90), near, 0, 90).status(), Placement.Status.READY),
        answers("turned a hair more is not",
            () -> placed(pose(1, 3, 90.0001), near, 0, 90).status(), Placement.Status.NOT_ON_A_START),
        answers("halfway between two starts, the one listed first is the nearest",
            () -> placed(pose(1, 4, 0), null, 0.1, 5).start(), "Near"),
        answers("the nearer start, though listed second",
            () -> placed(pose(1, 4.5, 0), null, 0.1, 5).start(), "Far"));
  }

  // spotless:on
}
