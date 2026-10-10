package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.refused;
import static com.michaelgrundvig.frc.world.field.TestFields.xy;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Translation2d;

/** A ramp on its own: one slope up from its foot to its top edge, then straight down. */
class RampTest {
  /** Climbing toward +x from x = 2 to x = 3.2, 1.5 m wide, to 30 cm. */
  private static final Ramp RAMP = new Ramp("Ramp", xy(2, 4), xy(3.2, 4), 1.5, 0.3);

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          at its foot            | 2    | 4   | 0
          halfway up, to a side  | 2.6  | 4.7 | 0.15
          at its top, to a side  | 3.2  | 3.3 | 0.3
          past its top           | 3.21 | 4   | 0
          beside it              | 2.6  | 4.8 | 0
          """)
  void itRisesEvenlyFromItsFootToItsTop(String where, double x, double y, double height) {
    assertThat(RAMP.heightAt(xy(x, y))).isCloseTo(height, within(1e-12));
  }

  @Test
  void itsShapeIsWhereItClimbsAndHowSteeply() {
    assertThat(RAMP.runMeters()).isCloseTo(1.2, within(1e-12));
    assertThat(RAMP.up().getDegrees()).isCloseTo(0, within(1e-12));
    assertThat(Math.toDegrees(RAMP.slopeRadians())).isCloseTo(14.04, within(0.01));
    assertThat(RAMP.steepestSlopeRadians()).isEqualTo(RAMP.slopeRadians());
    assertThat(RAMP.normalAt(xy(2.6, 4)).x()).as("leaning back down").isNegative();
    assertThat(RAMP.normalAt(xy(4, 4)).z()).as("flat past its top").isEqualTo(1);
    assertThat(RAMP.footprint().area()).isCloseTo(1.8, within(1e-12));
  }

  @Test
  void itsTopIsACrossingWhoseWayAcrossIsUp() {
    assertThat(RAMP.crossings()).hasSize(1);
    Crossing top = RAMP.crossings().get(0);
    assertThat(top.name()).isEqualTo("Ramp");
    assertThat(top.kind()).isEqualTo(Crossing.Kind.TOP);
    assertThat(top.length()).isCloseTo(1.5, within(1e-12));
    assertThat(top.across().getDegrees()).isCloseTo(0, within(1e-9));
    assertThat(top.middle()).isEqualTo(xy(3.2, 4));
  }

  static Stream<Arguments> refusals() {
    Translation2d here = xy(1, 1);
    Translation2d there = xy(2, 1);
    return Stream.of(
        refused("no name", () -> new Ramp(" ", here, there, 1, 1), "a ramp needs a name"),
        refused("one point", () -> new Ramp("R", here, here, 1, 1), "two different points"),
        refused("no width", () -> new Ramp("R", here, there, 0, 1), "must be positive"),
        refused("endless", () -> new Ramp("R", here, there, 1, 1 / 0.0), "must be positive"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void aRampNeedsARealShape(ThrowingCallable building, String message) {
    assertThatThrownBy(building).hasMessageContaining(message);
  }
}
