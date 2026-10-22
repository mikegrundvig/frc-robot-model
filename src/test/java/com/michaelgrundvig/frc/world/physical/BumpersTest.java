package com.michaelgrundvig.frc.world.physical;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Meters;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;

class BumpersTest {
  @Test
  void halfTheBumpersReachFromTheRobotsCenter() {
    Bumpers bumpers = new Bumpers(Meters.of(0.9), Meters.of(0.8));

    assertThat(bumpers.halfLengthMeters()).isEqualTo(0.45);
    assertThat(bumpers.halfWidthMeters()).isEqualTo(0.4);
  }

  /** A length and a width, in meters, that no bumpers have. */
  @ParameterizedTest(name = "{0} m by {1} m")
  @CsvSource({"0, 0.8", "0.9, 0", "-0.9, 0.8", "NaN, 0.8"})
  void bumpersWithoutALengthOrAWidthAreRefused(double length, double width) {
    assertThatThrownBy(() -> new Bumpers(Meters.of(length), Meters.of(width)))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("bumpers need a length and a width above zero");
  }
}
