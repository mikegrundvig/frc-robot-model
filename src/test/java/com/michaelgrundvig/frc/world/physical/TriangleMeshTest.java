package com.michaelgrundvig.frc.world.physical;

import static com.michaelgrundvig.frc.world.physical.Cases.assertRefused;
import static com.michaelgrundvig.frc.world.physical.Cases.refused;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Translation2d;

class TriangleMeshTest {
  /**
   * A wedge: a 2 m square slope from the floor at x = 0 up to 1 m at x = 2, with a wall straight
   * down at x = 2.
   */
  private static TriangleMesh wedge() {
    TriangleMesh.Builder mesh = TriangleMesh.builder();
    int footRight = mesh.corner(0, 0, 0);
    int topRight = mesh.corner(2, 0, 1);
    int topLeft = mesh.corner(2, 2, 1);
    int footLeft = mesh.corner(0, 2, 0);
    int backRight = mesh.corner(2, 0, 0);
    int backLeft = mesh.corner(2, 2, 0);
    return mesh.quad(footRight, topRight, topLeft, footLeft)
        .quad(backRight, backLeft, topLeft, topRight)
        .build();
  }

  @Test
  void eachTriangleFacesOutByTheRightHandRule() {
    TriangleMesh wedge = wedge();
    TriangleMesh.Triangle slope = wedge.triangles().get(0);
    TriangleMesh.Triangle back = wedge.triangles().get(2);

    assertThat(wedge.normal(slope).x()).isCloseTo(-1 / Math.sqrt(5), within(1e-12));
    assertThat(wedge.normal(slope).z()).isCloseTo(2 / Math.sqrt(5), within(1e-12));
    assertThat(wedge.normal(back).x()).isCloseTo(1, within(1e-12));
    assertThat(wedge.vertical(back)).isTrue();
    assertThat(wedge.vertical(slope)).isFalse();
    assertThat(wedge.corners(slope))
        .containsExactly(new Vector3(0, 0, 0), new Vector3(2, 0, 1), new Vector3(2, 2, 1));
  }

  /** A line cast down meets the highest face and passes walls by; blank means it meets nothing. */
  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          on the slope                 | 1   | 0.5 | 0.5
          on the top edge, over a wall | 2   | 1   | 1
          on the foot                  | 0   | 1   | 0
          past the wall                | 2.1 | 1   |
          """)
  void aLineCastDownMeetsTheHighestFaceAndPassesWallsBy(
      String label, double x, double y, @Nullable Double height) {
    TriangleMesh wedge = wedge();
    var hit = wedge.castDown(new Translation2d(x, y));

    if (height == null) {
      assertThat(hit).isEmpty();
    } else {
      assertThat(hit.orElseThrow().height()).isCloseTo(height, within(1e-12));
      assertThat(hit.orElseThrow().normal().z()).isPositive();
    }
    assertThat(wedge.heightOver(wedge.triangles().get(2), new Translation2d(2, 1))).isEmpty();
  }

  @Test
  void anUpsideDownFaceIsTurnedUpWhenMet() {
    // A ceiling at 3 m, facing down, over the wedge: the line down meets it first.
    TriangleMesh.Builder ceiling = TriangleMesh.builder();
    int a = ceiling.corner(0, 0, 3);
    int b = ceiling.corner(0, 2, 3);
    int c = ceiling.corner(2, 0, 3);
    TriangleMesh both = wedge().plus(ceiling.triangle(a, b, c).build());

    TriangleMesh.Hit hit = both.castDown(new Translation2d(0.5, 0.5)).orElseThrow();

    assertThat(hit.height()).isEqualTo(3);
    assertThat(hit.normal().z()).isEqualTo(1);
    assertThat(Math.hypot(hit.normal().x(), hit.normal().y())).isZero();
    assertThat(both.triangles()).hasSize(5);
    assertThat(both.triangles().get(4)).isEqualTo(new TriangleMesh.Triangle(6, 7, 8));
  }

  @Test
  void aMirrorImageListsEachTrianglesCornersTheOtherWay() {
    TriangleMesh wedge = wedge();

    TriangleMesh mirrored =
        wedge.map(corner -> new Vector3(-corner.x(), corner.y(), corner.z()), true);
    TriangleMesh moved = wedge.map(corner -> corner.plus(new Vector3(1, 0, 0)), false);

    Vector3 slope = mirrored.normal(mirrored.triangles().get(0));
    assertThat(slope.x())
        .as("still facing out, now toward +x")
        .isCloseTo(1 / Math.sqrt(5), within(1e-12));
    assertThat(slope.z()).isPositive();
    assertThat(moved.normal(moved.triangles().get(0)))
        .isEqualTo(wedge.normal(wedge.triangles().get(0)));
    assertThat(moved.castDown(new Translation2d(2, 0.5)).orElseThrow().height())
        .isCloseTo(0.5, within(1e-12));
  }

  static Stream<Arguments> refusals() {
    List<Vector3> three = List.of(new Vector3(0, 0, 0), new Vector3(1, 0, 0), new Vector3(0, 1, 0));
    return Stream.of(
        refused(
            "a corner that isn't there",
            () -> new TriangleMesh(three, List.of(new TriangleMesh.Triangle(0, 1, 3))),
            "names corner 3 of 3"),
        refused(
            "a triangle with no area",
            () -> new TriangleMesh(three, List.of(new TriangleMesh.Triangle(0, 1, 1))),
            "no area"),
        refused(
            "a corner that is no point",
            () -> new TriangleMesh(List.of(new Vector3(0, 0, Double.NaN)), List.of()),
            "isn't a point"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void aMeshMustHoldTogether(ThrowingCallable building, String[] pieces) {
    assertRefused(building, pieces);
    assertThat(TriangleMesh.EMPTY.castDown(Translation2d.ZERO)).isEmpty();
  }
}
