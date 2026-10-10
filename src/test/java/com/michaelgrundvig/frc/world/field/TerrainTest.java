package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.refused;
import static com.michaelgrundvig.frc.world.field.TestFields.xy;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Translation2d;

/** Bumps: their shape, where they are, and both halves' from one written for blue. */
class TerrainTest {
  /** A bump like REBUILT's: its crest along y at x = 4, 6.5 in tall, 44.4 in deep. */
  private static final Bump BUMP = new Bump("Bump", xy(4, 2), xy(4, 4), 0.1654, 1.128);

  private static FieldModel fieldWith(Symmetry symmetry, Surface... surfaces) {
    return new FieldModel(
        TestFields.layout(), symmetry, List.of(), List.of(), List.of(), List.of(surfaces));
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          on the crest             | 4     | 3   | 0.1654
          half a depth in, near    | 3.718 | 3   | 0.0827
          half a depth in, far     | 4.282 | 3   | 0.0827
          down to the floor        | 4.6   | 3   | 0
          past the crest's end     | 4     | 4.1 | 0
          """)
  void aBumpIsARidgeDownToTheFloorOnBothSides(String where, double x, double y, double height) {
    assertThat(BUMP.heightAt(xy(x, y))).isCloseTo(height, within(1e-12));
  }

  @Test
  void aBumpSaysWhereAPointIsAcrossItsCrestAndAlong() {
    Translation2d point = xy(3.7, 2.5);

    // Along the crest is +y, so across it is -x.
    assertThat(Math.toDegrees(BUMP.slopeRadians())).isCloseTo(16.3, within(0.1));
    assertThat(BUMP.along().getDegrees()).isCloseTo(90, within(1e-9));
    assertThat(BUMP.across().getDegrees()).isCloseTo(180, within(1e-9));
    assertThat(BUMP.acrossOffset(point)).isCloseTo(0.3, within(1e-12));
    assertThat(BUMP.alongOffset(point)).isCloseTo(0.5, within(1e-12));
    assertThat(BUMP.length()).isEqualTo(2);
    assertThat(BUMP.spans(xy(4, 1.9), 0.2)).isTrue();
    assertThat(BUMP.covers(xy(3.3, 3), 0.1)).isFalse();
    assertThat(BUMP.covers(xy(3.3, 3), 0.2)).isTrue();
  }

  @Test
  void theTerrainIsTheHighestBumpAndFindsTheOneUnderAPoint() {
    Terrain terrain = new Terrain(List.of(BUMP, new Bump("Other", xy(8, 2), xy(8, 4), 0.3, 1.0)));

    assertThat(terrain.heightAt(xy(8, 3))).isCloseTo(0.3, within(1e-12));
    assertThat(terrain.heightAt(xy(6, 3))).isZero();
    assertThat(terrain.under(xy(4.2, 3), 0)).contains(BUMP);
    assertThat(terrain.under(xy(6, 3), 0.5)).isEmpty();
    assertThat(Terrain.FLAT.heightAt(xy(4, 3))).isZero();
  }

  @Test
  void theTerrainIsTheHighestSurfaceOfAnyKind() {
    Ramp ramp = new Ramp("Ramp", xy(3, 3), xy(4.5, 3), 1, 0.3);
    Terrain terrain = new Terrain(List.of(BUMP, ramp));
    // Where the ramp climbs onto the bump's near side, the higher of the two is the ground.
    Translation2d both = xy(3.9, 3);

    assertThat(terrain.heightAt(both)).isCloseTo(ramp.heightAt(both), within(1e-12));
    assertThat(ramp.heightAt(both)).isGreaterThan(BUMP.heightAt(both));
    assertThat(terrain.normalAt(both)).isEqualTo(ramp.normalAt(both));
    assertThat(terrain.normalAt(xy(4.2, 3.9))).isEqualTo(BUMP.normalAt(xy(4.2, 3.9)));
    assertThat(terrain.normalAt(xy(8, 7)).z()).isEqualTo(1);
    assertThat(terrain.crossings())
        .extracting(Crossing::kind)
        .containsExactly(Crossing.Kind.CREST, Crossing.Kind.TOP);
    assertThat(terrain.under(xy(2.8, 3), 0.3)).contains(ramp);
    assertThat(terrain.under(xy(4.6, 2.5), 0.1)).as("nearest by footprint").contains(BUMP);
  }

  @Test
  void theFieldPutsEachBlueSurfacesPartnerOnRedsHalfWithItsCrossings() {
    FieldModel field = fieldWith(Symmetry.MIRRORED, TestFields.STATION, TestFields.tent());

    assertThat(field.terrain().surfaces())
        .extracting(Surface::name)
        .containsExactly("Station (blue)", "Station (red)", "Tent (blue)", "Tent (red)");
    assertThat(field.terrain().crossings())
        .extracting(Crossing::name)
        .containsExactly(
            "Station (blue), side 2",
            "Station (blue), side 4",
            "Station (red), side 1",
            "Station (red), side 3",
            "Tent crest (blue)",
            "Tent crest (red)");
    assertThat(field.terrain().heightAt(xy(16 - 14.5, 1.5))).isCloseTo(0.2, within(1e-12));
  }

  @Test
  void theFieldPutsEachBlueBumpsPartnerOnRedsHalf() {
    List<Surface> surfaces = fieldWith(Symmetry.ROTATED, BUMP).terrain().surfaces();

    assertThat(surfaces).extracting(Surface::name).containsExactly("Bump (blue)", "Bump (red)");
    Bump red = (Bump) surfaces.get(1);
    assertThat(red.crestStart().getX()).isCloseTo(12, within(1e-9));
    assertThat(red.crestStart().getY()).isCloseTo(6, within(1e-9));
    assertThat(red.heightAt(xy(12, 5))).isCloseTo(0.1654, within(1e-12));
    assertThat(red.heightAt(xy(4, 5))).isZero();
  }

  static Stream<Arguments> refusals() {
    Translation2d here = xy(1, 1);
    Bump off = new Bump("Off", xy(-1, 2), xy(-1, 4), 0.1, 1);
    return Stream.of(
        refused("a bump with no name", () -> new Bump(" ", here, xy(1, 2), 0.1, 1), "name"),
        refused(
            "a crest of one point",
            () -> new Bump("B", here, here, 0.1, 1),
            "two different points"),
        refused("a bump with no height", () -> new Bump("B", here, xy(1, 2), 0, 1), "positive"),
        refused("a bump off the field", () -> fieldWith(Symmetry.ROTATED, off), "off the field"),
        refused(
            "two surfaces of one name",
            () -> fieldWith(Symmetry.MIRRORED, TestFields.tent(), TestFields.tent()),
            "two surfaces are named Tent (blue)"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void whatCantBeASurfaceOfTheFieldIsRefused(ThrowingCallable building, String message) {
    assertThatThrownBy(building).hasMessageContaining(message);
  }
}
