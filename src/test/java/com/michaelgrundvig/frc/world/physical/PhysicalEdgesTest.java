package com.michaelgrundvig.frc.world.physical;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.physical.TriangleMesh.Triangle;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The geometry's and the mass model's decisions at their edges, as tables of questions and answers:
 * where segments meet, what an outline may be, which points a polygon holds, how it is cut into
 * triangles, where a line straight down meets a triangle, and which tensors a body can have. Each
 * row is exactly on an edge, or a step to one side of it.
 */
class PhysicalEdgesTest {
  @ParameterizedTest
  @MethodSource({
    "segments",
    "outlines",
    "pointsInPolygons",
    "cuttings",
    "castsDown",
    "tensors",
    "masses",
    "boxes",
    "numbersEachAskedAlone"
  })
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments answers(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  private static Translation2d xy(double x, double y) {
    return new Translation2d(x, y);
  }

  /** A polygon from its corners' x and y, counterclockwise. */
  private static Polygon2d polygon(double... xy) {
    List<Translation2d> corners = new ArrayList<>();
    for (int index = 0; index < xy.length; index += 2) {
      corners.add(xy(xy[index], xy[index + 1]));
    }
    return new Polygon2d(corners);
  }

  /** What building something refuses with, or "accepted". */
  private static String refusal(Supplier<?> building) {
    try {
      Objects.requireNonNull(building.get());
      return "accepted";
    } catch (IllegalArgumentException e) {
      return String.valueOf(e.getMessage());
    }
  }

  /** Whether the segment from (ax, ay) to (bx, by) meets the one from (cx, cy) to (dx, dy). */
  private static boolean meet(
      double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy) {
    return Segments.meet(xy(ax, ay), xy(bx, by), xy(cx, cy), xy(dx, dy));
  }

  // spotless:off

  /** How far a point is from a segment, and whether two segments meet. */
  static Stream<Arguments> segments() {
    return Stream.of(
        answers("a point beside a segment is as far as the line is",
            () -> Segments.distance(xy(1, 3), xy(0, 0), xy(4, 0)), 3.0),
        answers("a point past a segment's end is as far as the end is",
            () -> Segments.distance(xy(7, 4), xy(0, 0), xy(4, 0)), 5.0),
        answers("a point before its start is as far as the start is",
            () -> Segments.distance(xy(-3, 4), xy(0, 0), xy(4, 0)), 5.0),
        answers("two segments that cross meet", () -> meet(0, 0, 2, 2, 0, 2, 2, 0), true),
        answers("two that would cross if one were longer do not", () -> meet(0, 0, 2, 2, 3, 0, 5, -2), false),
        answers("one that ends on the other meets it", () -> meet(0, 0, 0, 2, 0, 1, 1, 1), true),
        answers("...whichever of them is given first", () -> meet(0, 1, 1, 1, 0, 0, 0, 2), true),
        answers("one that starts from the other's side meets it", () -> meet(0, 0, 2, 0, 1, 3, 1, 0), true),
        answers("...whichever of them is given first", () -> meet(1, 3, 1, 0, 0, 0, 2, 0), true),
        answers("two that share an end meet", () -> meet(0, 0, 1, 0, 1, 0, 1, 1), true),
        answers("two in a line that overlap meet", () -> meet(0, 0, 2, 0, 1, 0, 3, 0), true),
        answers("two in a line with a gap between them do not", () -> meet(0, 0, 1, 0, 2, 0, 3, 0), false),
        answers("an end inside the other's bounds, but off it, is no meeting",
            () -> meet(0, 0, 2, 2, 1.5, 0.5, 3, 0.5), false),
        answers("...whichever of them is given first", () -> meet(1.5, 0.5, 3, 0.5, 0, 0, 2, 2), false));
  }

  /** What a list of corners is refused for, and what a few polygons made other ways are. */
  static Stream<Arguments> outlines() {
    Pose2d origin = Pose2d.ZERO;
    return Stream.of(
        answers("an outline that doubles back names the corner it turns at",
            () -> refusal(() -> polygon(0, 0, 2, 0, 1, 0, 1, 1)), "the polygon doubles back at corner 1"),
        answers("two edges that cross are named",
            () -> refusal(() -> polygon(0, 0, 2, 0, 2, 2, 1, -1, 0, 2)), "the polygon's edges 0 and 2 cross"),
        answers("a corner that goes straight on is no fault", () -> polygon(0, 0, 1, 0, 2, 0, 2, 2, 0, 2).convex(), true),
        answers("a corner that points in is not convex", () -> polygon(0, 0, 2, 0, 1, 0.5, 2, 1, 0, 1).convex(), false),
        answers("three points have a hull", () -> Polygon2d.hull(List.of(xy(0, 0), xy(1, 0), xy(0, 1))).corners().size(), 3),
        answers("no points have no hull",
            () -> refusal(() -> Polygon2d.hull(List.of())),
            "no points enclose nothing: a hull needs three that are not on one line"),
        answers("one point has none either",
            () -> refusal(() -> Polygon2d.hull(List.of(xy(1, 1)))), "points all on one line enclose nothing"),
        answers("...nor have three on one line",
            () -> refusal(() -> Polygon2d.hull(List.of(xy(0, 0), xy(1, 1), xy(2, 2)))), "points all on one line enclose nothing"),
        answers("a hull starts from its leftmost point and goes counterclockwise",
            () -> Polygon2d.hull(List.of(xy(1, 1), xy(0, 1), xy(1, 0), xy(0, 0), xy(0.5, 0.5), xy(0.5, 0))).corners(),
            List.of(xy(0, 0), xy(1, 0), xy(1, 1), xy(0, 1))),
        answers("a rectangle of no length is refused as one",
            () -> refusal(() -> Polygon2d.rectangle(origin, 0, 1)), "a rectangle's sides must be positive: 0.0 by 1.0"),
        answers("a rectangle of no width is refused as one",
            () -> refusal(() -> Polygon2d.rectangle(origin, 1, 0)), "a rectangle's sides must be positive: 1.0 by 0.0"),
        answers("a rectangle with no end is refused as one",
            () -> refusal(() -> Polygon2d.rectangle(origin, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)),
            "a rectangle's sides must be positive: Infinity by Infinity"),
        answers("a rectangle between corners in line along y is refused as one",
            () -> refusal(() -> Polygon2d.rectangle(xy(1, 0), xy(1, 2))).startsWith("a rectangle's sides must be positive: from"),
            true),
        answers("a rectangle between corners in line along x is refused as one",
            () -> refusal(() -> Polygon2d.rectangle(xy(0, 2), xy(1, 2))).startsWith("a rectangle's sides must be positive: from"),
            true));
  }

  /** A comb with two teeth pointing up: its back from (0, 0) to (5, 1), teeth at x 0..1 and 4..5. */
  private static Polygon2d comb() {
    return polygon(0, 0, 5, 0, 5, 3, 4, 3, 4, 1, 1, 1, 1, 3, 0, 3);
  }

  private static Polygon2d square() {
    return Polygon2d.rectangle(xy(0, 0), xy(1, 1));
  }

  /** Which points a polygon holds, and how far a point inside it goes before its edge. */
  static Stream<Arguments> pointsInPolygons() {
    return Stream.of(
        answers("a point in a tooth is inside", () -> comb().contains(xy(0.5, 2)), true),
        answers("a point between the teeth is outside", () -> comb().contains(xy(2.5, 2)), false),
        answers("a point in the other tooth is inside", () -> comb().contains(xy(4.5, 2)), true),
        answers("a point past both teeth is outside", () -> comb().contains(xy(6, 2)), false),
        answers("a point before both teeth is outside", () -> comb().contains(xy(-1, 2)), false),
        answers("a point outside, level with the teeth's tips, is outside", () -> comb().contains(xy(-1, 3)), false),
        answers("a point between the teeth, level with their roots, is outside", () -> comb().contains(xy(2.5, 1.5)), false),
        answers("a point in the back, level with nothing, is inside", () -> comb().contains(xy(2.5, 0.5)), true),
        answers("a point outside, level with the back's top, is outside", () -> comb().contains(xy(-1, 1)), false),
        answers("a point exactly a nanometer outside an edge is on it", () -> square().contains(xy(0.5, -1e-9)), true),
        answers("a point a hair further out is outside", () -> square().contains(xy(0.5, -1.1e-9)), false),
        answers("...and is that far from it", () -> square().distance(xy(0.5, -1.1e-9)), 1.1e-9),
        answers("a point a nanometer inside an edge, heading out, does not count the edge as ahead",
            () -> square().reach(xy(0.5, 1e-9), Rotation2d.fromDegrees(-90)), 0.0),
        answers("a point a little further in reaches the edge",
            () -> square().reach(xy(0.5, 0.25), Rotation2d.fromDegrees(-90)), 0.25),
        answers("a point heading along an edge reaches the one ahead of it",
            () -> square().reach(xy(0.25, 0), Rotation2d.ZERO), 0.75));
  }

  /** A polygon's triangles, each by its corners' numbers. */
  private static List<List<Integer>> triangles(Polygon2d polygon) {
    List<List<Integer>> triangles = new ArrayList<>();
    for (Triangle triangle : polygon.triangles()) {
      triangles.add(List.of(triangle.a(), triangle.b(), triangle.c()));
    }
    return triangles;
  }

  /** How an outline is cut into triangles. */
  static Stream<Arguments> cuttings() {
    return Stream.of(
        answers("a convex outline is cut into a fan from its first corner",
            () -> triangles(polygon(0, 0, 2, 0, 3, 1, 2, 2, 0, 2)),
            List.of(List.of(0, 1, 2), List.of(0, 2, 3), List.of(0, 3, 4))),
        answers("a corner that goes straight on is dropped without a triangle",
            () -> triangles(polygon(0, 0, 1, 0, 2, 0, 2, 1, 0, 1)), List.of(List.of(0, 2, 3), List.of(0, 3, 4))),
        answers("a corner that turns by a sine of exactly 1e-12 goes straight on",
            () -> triangles(polygon(0, 0, 1, 0, 2, 1e-12, 2, 1, 0, 1)), List.of(List.of(0, 2, 3), List.of(0, 3, 4))),
        answers("how far a corner turns is judged by its angle, not by its sides' lengths",
            () -> triangles(polygon(0, 0, 10, 0, 20, 1e-12, 20, 5, 0, 5)), List.of(List.of(0, 2, 3), List.of(0, 3, 4))),
        answers("a corner that turns a little more is cut off as any other",
            () -> triangles(polygon(0, 0, 1, 0, 2, 1e-6, 2, 1, 0, 1)).size(), 3),
        answers("an outline that can all be seen from its first corner is cut into a fan too",
            () -> triangles(polygon(0, 0, 2, 0, 2, 1, 1, 1, 1, 2, 0, 2)),
            List.of(List.of(0, 1, 2), List.of(0, 2, 3), List.of(0, 3, 4), List.of(0, 4, 5))),
        answers("a corner whose triangle would hold another corner is no ear, and is passed over",
            () -> triangles(polygon(0, 0, 4, 0, 4, 3, 3, 3, 3, 1, 1, 1, 1, 3, 0, 3)),
            List.of(List.of(1, 2, 3), List.of(1, 3, 4), List.of(1, 4, 5), List.of(5, 6, 7), List.of(5, 7, 0), List.of(0, 1, 5))),
        answers("once an inward corner's two neighbors lie in a line through it, it is dropped",
            () -> triangles(polygon(0, 0, 2, 0, 1, 0.5, 2, 1, 0, 1)),
            List.of(List.of(0, 1, 2), List.of(0, 3, 4))),
        answers("a corner whose triangle has another corner on its edge is no ear",
            () -> triangles(polygon(0, 0, 1, 0, 2, 0, 1, 1, 0, 2, 0, 1)),
            List.of(List.of(0, 2, 3), List.of(3, 4, 5), List.of(0, 3, 5))),
        answers("the same outline listed from its next corner is cut into a fan",
            () -> triangles(polygon(1, 0, 2, 0, 1, 1, 0, 2, 0, 1, 0, 0)),
            List.of(List.of(0, 1, 2), List.of(0, 2, 3), List.of(0, 3, 4), List.of(0, 4, 5))),
        answers("...and listed from the one after, it is cut as the first was",
            () -> triangles(polygon(2, 0, 1, 1, 0, 2, 0, 1, 0, 0, 1, 0)),
            List.of(List.of(0, 2, 3), List.of(3, 4, 5), List.of(0, 3, 5))),
        answers("a triangle is itself", () -> triangles(polygon(0, 0, 1, 0, 0, 1)), List.of(List.of(0, 1, 2))));
  }

  /** The triangle with corners at (0, 0), (1, 0) and (0, 1), rising from 0 at the first to 1 at each other. */
  private static TriangleMesh ramp() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    return mesh.triangle(mesh.corner(0, 0, 0), mesh.corner(1, 0, 1), mesh.corner(0, 1, 1)).build();
  }

  private static OptionalDouble over(double x, double y) {
    TriangleMesh ramp = ramp();
    return ramp.heightOver(ramp.triangles().get(0), xy(x, y));
  }

  /** A ridge along y at x = 1, 1 m high, its two sides down to the floor at x = 0 and x = 2. */
  private static TriangleMesh ridge() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int crestNear = mesh.corner(1, 0, 1);
    int crestFar = mesh.corner(1, 2, 1);
    int east = mesh.corner(2, 0, 0);
    int west = mesh.corner(0, 0, 0);
    return mesh.triangle(crestNear, east, crestFar).triangle(crestNear, crestFar, west).build();
  }

  /** A triangle 10 m long that leans a hair off upright, and one standing upright. */
  private static TriangleMesh walls() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int foot = mesh.corner(0, 0, 0);
    int along = mesh.corner(10, 0, 0);
    int leaning = mesh.corner(0, 1e-12, 1);
    int upright = mesh.corner(0, 0, 1);
    return mesh.triangle(foot, along, leaning).triangle(foot, upright, along).build();
  }

  /** Where a line straight down meets triangles. */
  static Stream<Arguments> castsDown() {
    return Stream.of(
        answers("a point over a triangle meets it at the height there", () -> over(0.25, 0.25), OptionalDouble.of(0.5)),
        answers("a point a billionth of a side outside one edge still meets it", () -> over(0.5, -1e-9).isPresent(), true),
        answers("a hair further out, it passes beside", () -> over(0.5, -1.1e-9), OptionalDouble.empty()),
        answers("a point a billionth of a side outside another edge still meets it", () -> over(-1e-9, 0.5).isPresent(), true),
        answers("a hair further out, it passes beside", () -> over(-1.1e-9, 0.5), OptionalDouble.empty()),
        answers("a point a billionth of a side past the long edge still meets it", () -> over(1, 1e-9).isPresent(), true),
        answers("a hair further out, it passes beside", () -> over(1, 1.1e-9), OptionalDouble.empty()),
        answers("on a crest two faces share, the first listed is met",
            () -> ridge().castDown(xy(1, 1)).map(hit -> hit.normal().x() > 0), Optional.of(true)),
        answers("a triangle that leans a hair off upright, however long, is a wall",
            () -> walls().vertical(walls().triangles().get(0)), true),
        answers("a wall's normal is level, and a hit on it keeps it as it is",
            () -> walls().hit(walls().triangles().get(1), 0.5).normal(), walls().normal(walls().triangles().get(1))));
  }

  private static Inertia inertia(double ixx, double iyy, double izz, double ixy, double ixz, double iyz) {
    return Inertia.builder().ixx(ixx).iyy(iyy).izz(izz).ixy(ixy).ixz(ixz).iyz(iyz).build();
  }

  private static String refusedInertia(double ixx, double iyy, double izz, double ixy, double ixz, double iyz) {
    return refusal(() -> inertia(ixx, iyy, izz, ixy, ixz, iyz));
  }

  private static boolean solid(double ixx, double iyy, double izz, double ixy, double ixz, double iyz) {
    return inertia(ixx, iyy, izz, ixy, ixz, iyz).whyNotSolid() == null;
  }

  /** Which tensors a body can have, and which of those a solid body can. */
  static Stream<Arguments> tensors() {
    String tooLarge = ": the products of inertia are too large for the moments: no body turns that way";
    return Stream.of(
        answers("a moment equal to the other two together is a flat body's",
            () -> refusedInertia(1, 1, 2, 0, 0, 0), "accepted"),
        answers("a moment over the other two together by a rounding is accepted",
            () -> refusedInertia(1, 1, 2.000000000003, 0, 0, 0), "accepted"),
        answers("a moment over the other two together by more is no body's",
            () -> refusedInertia(1, 1, 2.00000000001, 0, 0, 0),
            "Inertia.ixx: (1, 1, 2) can't be a body's: each must be at most the sum of the other two"),
        answers("a product as large as its two moments allow is accepted",
            () -> refusedInertia(4, 1, 4, 0, 0, 2), "accepted"),
        answers("a product over that by a rounding is accepted",
            () -> refusedInertia(1, 1, 2, 1.000000000004, 0, 0), "accepted"),
        answers("the product of x and y too large", () -> refusedInertia(1, 1, 1, 1.1, 0, 0), "Inertia.ixy" + tooLarge),
        answers("the product of x and z too large", () -> refusedInertia(1, 1, 1, 0, 1.1, 0), "Inertia.ixy" + tooLarge),
        answers("the product of y and z too large", () -> refusedInertia(1, 1, 1, 0, 0, 1.1), "Inertia.ixy" + tooLarge),
        answers("products that pass pair by pair, but that no body has together, are refused",
            () -> refusedInertia(1, 1, 1, -0.9, -0.9, -0.9),
            "Inertia.ixy: the products of inertia (-0.9, -0.9, -0.9) are too large together for the moments"
                + " (1, 1, 1): no body turns that way. If they came from CAD, check their signs: some programs"
                + " report all three negated"),
        answers("...and the same products with the other sign are a body's",
            () -> refusedInertia(1, 1, 1, 0.9, 0.9, 0.9), "accepted"),
        answers("products that together leave no effort to turn about one axis are a flat body's",
            () -> refusedInertia(2, 1, 1, 1, 1, 0), "accepted"),
        answers("...and over that by a rounding are accepted",
            () -> refusedInertia(2, 1, 1, 1, 1.00000000000001, 0), "accepted"),
        answers("...and over that by more are refused, the moments and products named",
            () -> refusedInertia(2, 1, 1, 1, 1.000001, 0),
            "Inertia.ixy: the products of inertia (1, 1, 0) are too large together for the moments"
                + " (2, 1, 1): no body turns that way. If they came from CAD, check their signs: some programs"
                + " report all three negated"),
        answers("a tensor with small products is a solid body's", () -> solid(1, 1, 1, 0.4, 0.4, 0.4), true),
        answers("a tensor with every term different is a solid body's", () -> solid(2, 3, 4, 0.5, -0.7, 1.1), true),
        answers("a tensor whose determinant is exactly zero is not", () -> solid(1, 1, 1, 0, 1, 0), false),
        answers("a tensor made flat by its products of x with y and with z is not", () -> solid(2, 1, 1, 1, 1, 0), false),
        answers("...and one a little short of that is", () -> solid(2, 1, 1, 0.9, 0.9, 0), true),
        answers("a tensor made flat by its products of y with z and x with y is not", () -> solid(1, 2, 1, 1, 0, 1), false),
        answers("...and one a little short of that is", () -> solid(1, 2, 1, 0.9, 0, 0.9), true),
        answers("a tensor made flat by its products of x with z and y with z is not", () -> solid(1, 1, 2, 0, 1, 1), false),
        answers("...and one a little short of that is", () -> solid(1, 1, 2, 0, 0.9, 0.9), true),
        answers("a tensor only just a solid body's", () -> solid(1.05, 2.3, 1.49, 1.5, 1.2, 1.8), true),
        answers("...and the same with one product's sign changed is no body's at all",
            () -> refusedInertia(1.05, 2.3, 1.49, 1.5, 1.2, -1.8).contains("are too large together"), true),
        answers("three products of mixed signs that leave it positive", () -> solid(2, 2, 2, 1, -1, -1.4), true),
        answers("...and the same with one sign changed, which no body has",
            () -> refusedInertia(2, 2, 2, 1, -1, 1.4).contains("are too large together"), true),
        answers("a tensor is not zero for having no products", () -> inertia(1, 1, 1, 0, 0, 0).isZero(), false),
        answers("a tensor turned by nothing is itself",
            () -> inertia(2, 3, 4, 0.5, -0.7, 1.1).rotated(Rotation3d.ZERO), inertia(2, 3, 4, 0.5, -0.7, 1.1)));
  }

  private static MassProperties pointMass(double kilograms, double x, double y, double z) {
    return MassProperties.builder().mass(Kilograms.of(kilograms)).cg(new Translation3d(x, y, z)).inertia(Inertia.ZERO).build();
  }

  /** Parts added up and taken away. */
  static Stream<Arguments> masses() {
    return Stream.of(
        answers("two point masses either side of the origin turn about it as the parallel-axis theorem says",
            () -> MassProperties.sum(List.of(pointMass(2, 1, 2, 3), pointMass(2, -1, -2, -3))).inertia(),
            inertia(52, 40, 20, -8, -12, -24)),
        answers("two point masses along a line through neither axis turn about the point between them",
            () -> MassProperties.sum(List.of(pointMass(2, 1, 2, 3), pointMass(2, 3, 2, 3))).inertia(),
            inertia(0, 4, 4, 0, 0, 0)),
        answers("...and are centered on it",
            () -> MassProperties.sum(List.of(pointMass(2, 1, 2, 3), pointMass(2, -1, -2, -3))).cg(), Translation3d.ZERO),
        answers("a part taken from itself leaves nothing",
            () -> refusal(() -> pointMass(2, 1, 2, 3).minus(pointMass(2, 1, 2, 3))),
            "nothing is left: 2 Kg taken from 2 Kg"));
  }

  /** The largest number there is: finite, and any two of them add up to more than a number holds. */
  private static final double HUGE = Double.MAX_VALUE;

  private static final double NONE = Double.POSITIVE_INFINITY;

  /** A box of some size at a pose, or what it is refused for, down to which value. */
  private static String boxRefusal(Pose3d pose, Translation3d size) {
    return refusal(() -> Shape.Box.builder().pose(pose).size(size).material(Material.ALUMINUM).build());
  }

  private static String boxAt(double x, double y, double z) {
    return boxRefusal(new Pose3d(x, y, z, Rotation3d.ZERO), new Translation3d(1, 1, 1));
  }

  /**
   * A box turned by a quaternion with one part infinite: WPILib scales it to length 1, which leaves
   * that one part not a number and the others zero.
   */
  private static boolean boxTurnedIsRefused(double w, double x, double y, double z) {
    Pose3d pose = new Pose3d(Translation3d.ZERO, new Rotation3d(new Quaternion(w, x, y, z)));
    return boxRefusal(pose, new Translation3d(1, 1, 1)).contains("must be a pose, numbers all");
  }

  private static boolean boxSizedIsRefused(double length, double width, double height) {
    return boxRefusal(Pose3d.ZERO, new Translation3d(length, width, height)).contains("must be positive along each axis");
  }

  private static boolean centeredAtIsRefused(double x, double y, double z) {
    return refusal(() -> pointMass(1, x, y, z)).contains("must be a point, numbers all");
  }

  private static String meshWithCorner(double x, double y, double z) {
    return refusal(() -> new TriangleMesh(List.of(Vector3.ZERO, new Vector3(x, y, z)), List.of()));
  }

  /**
   * Whether a thing is made of numbers is asked of each number by itself. Numbers that are each
   * finite are taken though their sum is more than a number holds, and one that is missing or
   * infinite is refused whichever it is.
   */
  static Stream<Arguments> numbersEachAskedAlone() {
    return Stream.of(
        answers("a box as large as numbers go is a box", () -> boxSizedIsRefused(HUGE, HUGE, HUGE), false),
        answers("a box with no end to its length is not", () -> boxSizedIsRefused(NONE, 1, 1), true),
        answers("...nor to its width", () -> boxSizedIsRefused(1, NONE, 1), true),
        answers("...nor to its height", () -> boxSizedIsRefused(1, 1, NONE), true),
        answers("a box as far away as numbers go is somewhere", () -> boxAt(HUGE, HUGE, HUGE), "accepted"),
        answers("a box nowhere along x is not", () -> boxAt(NONE, 0, 0).contains("must be a pose, numbers all"), true),
        answers("...nor along y", () -> boxAt(0, NONE, 0).contains("must be a pose, numbers all"), true),
        answers("...nor along z", () -> boxAt(0, 0, NONE).contains("must be a pose, numbers all"), true),
        answers("a box turned no way by its quaternion's W is refused", () -> boxTurnedIsRefused(NONE, 0, 0, 0), true),
        answers("...by its X", () -> boxTurnedIsRefused(0, NONE, 0, 0), true),
        answers("...by its Y", () -> boxTurnedIsRefused(0, 0, NONE, 0), true),
        answers("...by its Z", () -> boxTurnedIsRefused(0, 0, 0, NONE), true),
        answers("a center of mass as far away as numbers go is a point", () -> centeredAtIsRefused(HUGE, HUGE, HUGE), false),
        answers("a center of mass nowhere along x is not", () -> centeredAtIsRefused(NONE, 0, 0), true),
        answers("...nor along y", () -> centeredAtIsRefused(0, NONE, 0), true),
        answers("...nor along z", () -> centeredAtIsRefused(0, 0, NONE), true),
        answers("a mesh's corner as far away as numbers go is a point", () -> meshWithCorner(HUGE, HUGE, HUGE), "accepted"),
        answers("a corner nowhere along x is not, and is named by its place in the list",
            () -> meshWithCorner(NONE, 0, 0), "corner 1 isn't a point: Vector3[x=Infinity, y=0.0, z=0.0]"),
        answers("...nor along y", () -> meshWithCorner(0, NONE, 0), "corner 1 isn't a point: Vector3[x=0.0, y=Infinity, z=0.0]"),
        answers("...nor along z", () -> meshWithCorner(0, 0, NONE), "corner 1 isn't a point: Vector3[x=0.0, y=0.0, z=Infinity]"));
  }

  private static Shape.Box box(double length, double width, double height) {
    return Shape.Box.builder().pose(Pose3d.ZERO).size(new Translation3d(length, width, height)).material(Material.ALUMINUM).build();
  }

  /** A box's corners, and the sizes it cannot have. */
  static Stream<Arguments> boxes() {
    return Stream.of(
        answers("a box's corners are half its size either way along each axis, x changing fastest",
            () -> box(2, 4, 6).points(),
            List.of(
                new Translation3d(-1, -2, -3), new Translation3d(1, -2, -3),
                new Translation3d(-1, 2, -3), new Translation3d(1, 2, -3),
                new Translation3d(-1, -2, 3), new Translation3d(1, -2, 3),
                new Translation3d(-1, 2, 3), new Translation3d(1, 2, 3))),
        answers("a box of no length is refused", () -> refusal(() -> box(0, 1, 1)).contains("must be positive along each axis"), true),
        answers("a box of no width is refused", () -> refusal(() -> box(1, 0, 1)).contains("must be positive along each axis"), true),
        answers("a box of no height is refused", () -> refusal(() -> box(1, 1, 0)).contains("must be positive along each axis"), true));
  }

  // spotless:on
}
