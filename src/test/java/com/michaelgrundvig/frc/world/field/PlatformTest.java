package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.STATION;
import static com.michaelgrundvig.frc.world.field.TestFields.polygon;
import static com.michaelgrundvig.frc.world.field.TestFields.refused;
import static com.michaelgrundvig.frc.world.field.TestFields.xy;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.List;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/** Platforms, on TestFields.STATION: a 2 m square top 25 cm up, ramps front and back. */
class PlatformTest {
  private static final Platform.Side EDGE = Platform.Side.EDGE;

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          the top                  | 5    | 3   | 0.25
          the top's edge           | 6    | 3   | 0.25
          halfway down a ramp      | 6.25 | 3   | 0.125
          up the front ramp        | 3.9  | 3   | 0.2
          past a ramp's foot       | 6.6  | 3   | 0
          off an edge              | 5    | 4.01| 0
          between ramps, a corner  | 6.2  | 4.2 | 0
          """)
  void itsTopIsFlatAndItsRampsRunDownFromItsSides(String where, double x, double y, double height) {
    assertThat(STATION.heightAt(xy(x, y))).isCloseTo(height, within(1e-12));
  }

  @Test
  void itsNormalsLeanOutOnARampAndStandUpOnTheTop() {
    assertThat(STATION.normalAt(xy(6.25, 3)).x()).as("leaning out").isPositive();
    assertThat(STATION.normalAt(xy(3.75, 3)).x()).isNegative();
    assertThat(STATION.normalAt(xy(5, 3)).z()).isEqualTo(1);
    assertThat(Math.toDegrees(STATION.steepestSlopeRadians()))
        .isCloseTo(Math.toDegrees(Math.atan2(0.25, 0.5)), within(1e-9));
  }

  @Test
  void itsFootprintLeavesTheFloorBetweenRampsOut() {
    assertThat(STATION.footprint().area()).as("the top and two ramps").isCloseTo(6, within(1e-12));
    assertThat(STATION.footprint().contains(xy(6.2, 3))).isTrue();
    assertThat(STATION.footprint().contains(xy(6.2, 4.2))).isFalse();
    assertThat(STATION.covers(xy(6.2, 4.2), 0.25)).isTrue();
  }

  @Test
  void eachRampsTopIsACrossingNamedForItsSide() {
    List<Crossing> crossings = STATION.crossings();

    assertThat(crossings)
        .extracting(Crossing::name)
        .containsExactly("Station, side 2", "Station, side 4");
    assertThat(crossings).allMatch(crossing -> crossing.kind() == Crossing.Kind.TOP);
    assertThat(crossings.get(0).across().getDegrees())
        .as("up the +x ramp is -x")
        .isCloseTo(180, within(1e-9));
    assertThat(crossings.get(1).across().getDegrees()).isCloseTo(0, within(1e-9));
  }

  @Test
  void aPlatformWithoutRampsHasOnlyItsTop() {
    Polygon2d square = Polygon2d.rectangle(new Pose2d(3, 3, Rotation2d.ZERO), 1, 1);
    Platform box = new Platform("Box", square, 0.1, List.of(EDGE, EDGE, EDGE, EDGE));

    assertThat(box.crossings()).isEmpty();
    assertThat(box.steepestSlopeRadians()).isZero();
    assertThat(box.footprint()).isEqualTo(box.top());
    assertThat(box.mesh().triangles()).as("its top, and two for each wall").hasSize(2 + 4 * 2);
  }

  static Stream<Arguments> refusals() {
    Polygon2d square = STATION.top();
    List<Platform.Side> four = STATION.sides();
    Polygon2d notch = polygon(0, 0, 2, 0, 1, 0.5, 2, 1, 0, 1);
    Polygon2d straight = polygon(0, 0, 1, 0, 2, 0, 2, 1, 0, 1);
    Platform.Side ramp = Platform.Side.ramp(1);
    return Stream.of(
        refused("no name", () -> new Platform(" ", square, 1, four), "name"),
        refused("no height", () -> new Platform("P", square, 0, four), "positive"),
        refused(
            "an edge short",
            () -> new Platform("P", square, 1, four.subList(0, 3)),
            "4 edges, not 3"),
        refused(
            "a notched top",
            () -> new Platform("P", notch, 1, List.of(EDGE, EDGE, EDGE, EDGE, EDGE)),
            "convex"),
        refused(
            "two ramps round a straight corner",
            () -> new Platform("P", straight, 1, List.of(ramp, ramp, EDGE, EDGE, EDGE)),
            "corner 1 goes straight on between two ramps"),
        refused("a ramp of no length", () -> Platform.Side.ramp(0), "some distance"),
        refused("a ramp of negative length", () -> new Platform.Side(-1), "some distance"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void aPlatformMustBeOne(ThrowingCallable building, String message) {
    assertThatThrownBy(building).hasMessageContaining(message);
  }
}
