package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;

import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;

/**
 * A joint's limit switch checked against where it is: doubted when it disagrees by more than a
 * tenth of the joint's range, or 15° for one that turns freely.
 */
class SwitchDoubtTest {
  private static final Joint.Revolute PIVOT = StandardRobot.PIVOT;

  /**
   * The pivot's switch at its lower end, tripping at 30°: its range is 135°, so 13.5° either way.
   */
  private static final Sensor.Switch PIVOT_DOWN =
      limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, Optional.of(Math.toRadians(30)));

  private static Sensor.Switch limit(
      String label, Joint joint, Sensor.Switch.Limit.End end, Optional<Double> position) {
    return Sensor.Switch.builder()
        .label(label)
        .part(joint.parent())
        .kind(new Sensor.Switch.Limit(joint, position, end))
        .build();
  }

  private static @Nullable String doubt(boolean tripped, double degrees) {
    return SwitchDoubt.of(PIVOT, PIVOT_DOWN, Math.toRadians(degrees), tripped);
  }

  /** Tripped with the pivot out at 130°. */
  @Test
  void trippedNowhereNearWhereItTripsIsDoubted() {
    assertThat(doubt(true, 130))
        .isEqualTo(
            "pivot down reads tripped with pivot at 130°, nowhere near where it trips, 30°: a"
                + " cut wire, if it's normally closed, or a switch out of place");
  }

  /** Clear with the pivot at its rest, well past where it trips. */
  @Test
  void clearWellPastWhereItTripsIsDoubted() {
    assertThat(doubt(false, 1.5))
        .isEqualTo(
            "pivot down reads clear with pivot at 1.5°, past where it trips, 30°: a cut wire, if"
                + " it's normally open, or a switch out of place");
  }

  /** Nothing's doubted where it agrees, within the margin. */
  @Test
  void nothingIsDoubtedWhereItAgrees() {
    assertThat(doubt(true, 20)).isNull();
    assertThat(doubt(false, 25)).isNull();
    assertThat(doubt(true, 30 - 13.4)).isNull();
    assertThat(doubt(false, 30 + 13.4)).isNull();
  }

  /** A switch whose position isn't known can't be doubted; a free joint's margin is 15°. */
  @Test
  void aSwitchWithNoPositionIsNeverDoubted() {
    Joint.Revolute turret = StandardRobot.lightPivot("turret", Optional.empty());
    Sensor.Switch somewhere =
        limit("turret left", turret, Sensor.Switch.Limit.End.LOWER, Optional.empty());
    Sensor.Switch placed =
        limit("turret right", turret, Sensor.Switch.Limit.End.UPPER, Optional.of(0.0));

    assertThat(SwitchDoubt.of(turret, somewhere, 3, true)).isNull();
    assertThat(SwitchDoubt.of(turret, placed, Math.toRadians(-16), false)).isNull();
    assertThat(SwitchDoubt.of(turret, placed, Math.toRadians(-14), true)).isNull();
    assertThat(SwitchDoubt.of(turret, placed, Math.toRadians(-16), true)).isNotNull();
  }
}
