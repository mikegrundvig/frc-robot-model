package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.refused;
import static com.michaelgrundvig.frc.world.field.TestFields.xy;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import java.util.List;
import java.util.Random;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Translation2d;

class MeshSurfaceTest {
  /**
   * Its grid of triangles finds the same ground as testing every triangle, on a mesh of thousands
   * of them, ragged and overlapping: a CAD export's.
   */
  @Test
  void itsLookupFindsWhatEveryTriangleWould() {
    Random random = new Random(4721);
    TriangleMesh.Builder builder = TriangleMesh.builder();
    for (int i = 0; i < 2000; i++) {
      double x = 3 + 4 * random.nextDouble();
      double y = 2 + 3 * random.nextDouble();
      int a = builder.corner(x, y, 0.5 * random.nextDouble());
      int b = builder.corner(x + 0.3 * random.nextDouble() + 0.01, y, 0.5 * random.nextDouble());
      int c = builder.corner(x, y + 0.3 * random.nextDouble() + 0.01, 0.5 * random.nextDouble());
      builder.triangle(a, b, c);
    }
    TriangleMesh mesh = builder.build();
    MeshSurface surface = new MeshSurface("Rubble", mesh, List.of());

    for (int i = 0; i < 2000; i++) {
      Translation2d point = xy(2.5 + 5 * random.nextDouble(), 1.5 + 4 * random.nextDouble());
      assertThat(surface.heightAt(point))
          .as("at %s", point)
          .isEqualTo(mesh.castDown(point).map(TriangleMesh.Hit::height).orElse(0.0));
    }
    assertThat(surface.toString()).contains("Rubble", "2000 triangles");
  }

  @Test
  void itsCrossingsAreGiven() {
    TriangleMesh slab = TestFields.tent().mesh();

    assertThat(new MeshSurface("Tent", slab, List.of()).crossings()).isEmpty();
    assertThat(TestFields.tent().heightAt(xy(14.5, 1.5))).isCloseTo(0.2, within(1e-12));
  }

  static Stream<Arguments> refusals() {
    TriangleMesh slab = TestFields.tent().mesh();
    Crossing off = new Crossing("Off", xy(14.5, 1), xy(14.5, 4), Crossing.Kind.CREST);
    TriangleMesh.Builder wall = TriangleMesh.builder();
    int a = wall.corner(0, 0, 0);
    int b = wall.corner(1, 0, 0);
    int c = wall.corner(1, 0, 1);
    TriangleMesh walls = wall.triangle(a, b, c).build();
    return Stream.of(
        refused(
            "a crossing off it", () -> new MeshSurface("T", slab, List.of(off)), "Off runs off it"),
        refused("only walls", () -> new MeshSurface("W", walls, List.of()), "nothing to stand on"),
        refused("no name", () -> new MeshSurface(" ", slab, List.of()), "name"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void aMeshSurfaceNeedsGroundAndHonestCrossings(ThrowingCallable building, String message) {
    assertThatThrownBy(building).hasMessageContaining(message);
  }
}
