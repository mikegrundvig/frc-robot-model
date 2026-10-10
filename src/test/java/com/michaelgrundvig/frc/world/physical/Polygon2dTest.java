package com.michaelgrundvig.frc.world.physical;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.shape.Rectangle2d;

/**
 * The tables name a polygon, or give its corners as x y pairs, counterclockwise. The named ones,
 * built as each test runs: a square 2 m on a side; an L, 3 m by 2 m, its notch cut from the top
 * right (concave at corner 3); and a five-pointed star (concave at every other corner).
 */
class Polygon2dTest {
  static Polygon2d polygon(double... xy) {
    List<Translation2d> corners = new ArrayList<>();
    for (int i = 0; i < xy.length; i += 2) {
      corners.add(new Translation2d(xy[i], xy[i + 1]));
    }
    return new Polygon2d(corners);
  }

  private static Polygon2d shape(String spec) {
    if (spec.equals("star")) {
      List<Translation2d> corners = new ArrayList<>();
      for (int i = 0; i < 10; i++) {
        double radius = i % 2 == 0 ? 2 : 0.8;
        double angle = Math.PI / 2 + i * Math.PI / 5;
        corners.add(new Translation2d(radius * Math.cos(angle), radius * Math.sin(angle)));
      }
      return new Polygon2d(corners);
    }
    String corners =
        spec.replace("square", "0 0 2 0 2 2 0 2").replace("L", "0 0 3 0 3 1 1 1 1 2 0 2");
    return polygon(Arrays.stream(corners.split(" ")).mapToDouble(Double::parseDouble).toArray());
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          a square | square | 4 | 1    | 1    | true
          an L     | L      | 4 | 1.25 | 0.75 | false
          a straight corner | 0 0 1 0 2 0 2 1 0 1 | 2 | 1 | 0.5 | true
          """)
  void itKnowsItsAreaCenterAndShape(
      String label, String spec, double area, double x, double y, boolean convex) {
    Polygon2d polygon = shape(spec);

    assertThat(polygon.area()).isCloseTo(area, within(1e-12));
    assertThat(polygon.convex()).isEqualTo(convex);
    assertThat(polygon.centroid().getX()).isCloseTo(x, within(1e-12));
    assertThat(polygon.centroid().getY()).isCloseTo(y, within(1e-12));
  }

  @Test
  void itsCornersCountAround() {
    Polygon2d l = shape("L");

    assertThat(l.corner(6)).isEqualTo(l.corner(0));
    assertThat(l.corner(-1)).isEqualTo(new Translation2d(0, 2));
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          inside                | L | 0.5 | 1.5 | true  | 0
          inside, along the leg | L | 2   | 0.5 | true  | 0
          in the notch          | L | 2   | 1.5 | false | 0.5
          on an edge            | L | 3   | 0.5 | true  | 0
          at the inside corner  | L | 1   | 1   | true  | 0
          far off a corner      | L | 4   | 3   | false | 2.23606797749979
          """)
  void aPointIsInsideOnTheEdgeOrSomeDistanceOff(
      String label, String spec, double x, double y, boolean inside, double distance) {
    Polygon2d polygon = shape(spec);

    assertThat(polygon.contains(new Translation2d(x, y))).isEqualTo(inside);
    assertThat(polygon.distance(new Translation2d(x, y))).isCloseTo(distance, within(1e-12));
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          from the middle, along x           | square | 1   | 1   | 0  | 1
          from the middle, at 45 degrees     | square | 1   | 1   | 45 | 1.4142135623730951
          along the L's leg                  | L      | 0.5 | 0.5 | 0  | 2.5
          up into the notch, which is out    | L      | 2   | 0.5 | 90 | 0.5
          from outside                       | square | 3   | 1   | 180 | 0
          on the edge, heading out           | square | 2   | 1   | 0  | 0
          """)
  void aWayFromInsideReachesTheEdge(
      String label, String spec, double x, double y, double degrees, double reach) {
    assertThat(shape(spec).reach(new Translation2d(x, y), Rotation2d.fromDegrees(degrees)))
        .isCloseTo(reach, within(1e-12));
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          a square, a fan from its first corner | square          | 2
          an L                                  | L               | 4
          a star, concave at every other corner | star            | 8
          a straight corner adds no triangle    | 0 0 1 0 2 0 2 1 0 1 | 2
          """)
  void itCutsIntoTrianglesThatCoverItExactly(String label, String spec, int count) {
    Polygon2d polygon = shape(spec);
    double area = 0;

    for (TriangleMesh.Triangle triangle : polygon.triangles()) {
      Translation2d a = polygon.corners().get(triangle.a());
      double twice =
          polygon
              .corners()
              .get(triangle.b())
              .minus(a)
              .cross(polygon.corners().get(triangle.c()).minus(a));
      assertThat(twice).as("counterclockwise").isPositive();
      area += twice / 2;
    }

    assertThat(polygon.triangles()).hasSize(count);
    assertThat(area).isCloseTo(polygon.area(), within(1e-9));
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          overlapping                | square | 1 1 3 1 3 3 1 3            | true
          one inside the other       | square | 0.5 0.5 1 0.5 1 1          | true
          the other way round        | 0.5 0.5 1 0.5 1 1 | square          | true
          corner to corner           | square | 2 2 3 2 3 3                | true
          along an edge              | square | 2 0.5 3 0.5 3 1            | true
          a bar across, no corner in | square | -1 0.5 3 0.5 3 1.5 -1 1.5  | true
          apart                      | square | 2.1 0 3 0 3 1              | false
          in the L's notch           | L      | 1.5 1.5 2.5 1.5 2.5 2.5 1.5 2.5 | false
          """)
  void twoPolygonsTouchWhereTheyShareAPoint(String label, String one, String other, boolean touch) {
    assertThat(shape(one).touches(shape(other))).isEqualTo(touch);
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          across                 | square | -1 1    | 3 1     | true
          a point in it          | square | 1 1     | 1 1     | true
          to its edge            | square | -1 1    | 0 1     | true
          short of it            | square | -1 1    | -0.1 1  | false
          within the L's notch   | L      | 1.5 1.5 | 2.5 1.5 | false
          into the L             | L      | 2 1.5   | 2 0.5   | true
          """)
  void aLineTouchesAPolygonWhereItCrossesOrReachesIt(
      String label, String spec, String from, String to, boolean touch) {
    double[] a = Arrays.stream(from.split(" ")).mapToDouble(Double::parseDouble).toArray();
    double[] b = Arrays.stream(to.split(" ")).mapToDouble(Double::parseDouble).toArray();

    assertThat(shape(spec).touches(new Translation2d(a[0], a[1]), new Translation2d(b[0], b[1])))
        .isEqualTo(touch);
  }

  @Test
  void aRectangleTurnsWithItsHeadingOrRunsAlongTheAxes() {
    Polygon2d turned = Polygon2d.rectangle(new Pose2d(5, 5, Rotation2d.CCW_PI_2), 4, 2);
    Polygon2d wpilibs =
        Polygon2d.of(new Rectangle2d(new Translation2d(1, 1), new Translation2d(3, 2)));

    assertThat(turned.area()).isCloseTo(8, within(1e-12));
    assertThat(turned.contains(new Translation2d(5, 6.9))).as("its length now along y").isTrue();
    assertThat(turned.contains(new Translation2d(5.9, 5))).isTrue();
    assertThat(turned.contains(new Translation2d(6.1, 5))).isFalse();
    assertThat(wpilibs.area()).isCloseTo(2, within(1e-12));
    assertThat(wpilibs.centroid().getX()).isCloseTo(2, within(1e-12));
    assertThat(Polygon2d.rectangle(new Translation2d(3, 1), new Translation2d(1, 2)).corners())
        .containsExactly(
            new Translation2d(1, 1),
            new Translation2d(3, 1),
            new Translation2d(3, 2),
            new Translation2d(1, 2));
  }

  @Test
  void theHullIsTheConvexOutlineOfPoints() {
    List<Translation2d> points = new ArrayList<>(shape("L").corners());
    points.add(new Translation2d(1, 0.5)); // inside
    points.add(new Translation2d(1.5, 0)); // on an edge

    Polygon2d hull = Polygon2d.hull(points);

    assertThat(hull.convex()).isTrue();
    assertThat(hull.area()).isCloseTo(5, within(1e-12));
    assertThat(hull.corners()).hasSize(5);
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          two corners                 | 0 0 1 0               | three corners
          clockwise                   | 0 0 0 1 1 0           | counterclockwise
          a line, no area             | 0 0 1 0 2 0           | some area
          a corner twice              | 0 0 1 0 1 0 0 1       | same point
          a bow tie with a roof       | 0 0 2 0 0 1 2 1 1 3   | cross
          an outline that doubles back| 0 0 2 0 1 0 1 1       | doubles back
          a corner that is no number  | 0 0 NaN 0 1 1         | isn't a point
          """)
  void aPolygonMustBeOne(String label, String spec, String message) {
    assertThatThrownBy(() -> shape(spec)).hasMessageContaining(message);
  }

  @Test
  void aRectangleNeedsSomeSize() {
    assertThatThrownBy(() -> Polygon2d.rectangle(Pose2d.ZERO, 0, 1))
        .hasMessageContaining("positive");
    assertThatThrownBy(() -> Polygon2d.rectangle(new Translation2d(1, 1), new Translation2d(1, 2)))
        .hasMessageContaining("positive");
    assertThatThrownBy(
            () ->
                Polygon2d.hull(
                    List.of(
                        new Translation2d(0, 0), new Translation2d(1, 1), new Translation2d(2, 2))))
        .hasMessageContaining("one line");
  }
}
