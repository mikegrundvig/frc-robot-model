package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.pose;
import static org.assertj.core.api.Assertions.assertThat;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.math.geometry.Pose2d;

class PlacementTest {
  private static final Start LEFT = Start.at("Left", pose(1, 6, 0));
  private static final Start CENTER = Start.at("Center", pose(1, 4, 0));

  /** An auto's start needn't be one of the field's: the check reads the one it's given too. */
  private static final Start WALL = Start.at("By the wall", pose(2, 7, 0));

  private static final FieldModel FIELD =
      TestFields.model(List.of(LEFT, CENTER), List.of(), List.of());
  private static final Map<String, Start> PLACES = Map.of("Center", CENTER, "By the wall", WALL);

  /**
   * A robot off its alliance's center start by (dx, dy, degrees), checked against the start
   * expected, within 10 cm and 5°; red's starts are the images of blue's. A blank alliance is not
   * yet known, a blank expected start is any.
   */
  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          on the expected start        | RED  |  0.02 |  0    |  1 | Center      | READY          | Ready on Center: 2 cm, 1° off
          on another start             | RED  |  0    | -2    |  0 | Center      | WRONG_START    | On Left, not the expected start
          off every start              | RED  | -1    | -1.5  | 12 | Center      | NOT_ON_A_START | Not on a start: Center is 1.80 m and 12° away
          turned too far               | RED  |  0    |  0    | 10 | Center      | NOT_ON_A_START | Not on a start: Center is 0.00 m and 10° away
          nothing expected, on a start | RED  |  0    | -2    |  0 |             | READY          | Ready on Left: 0 cm, 0° off
          nothing expected, off all    | RED  | -1    | -1.5  |  0 |             | NOT_ON_A_START | Not on a start: Left is 1.12 m and 0° away
          blue's starts are blue's     | BLUE |  0    |  0    |  0 | Center      | READY          | Ready on Center: 0 cm, 0° off
          the alliance is not known    |      |  0    |  0    |  0 | Center      | ALLIANCE_UNKNOWN | Alliance unknown
          an own start, on it          | RED  | -1    | -3    |  0 | By the wall | READY          | Ready on By the wall: 0 cm, 0° off
          an own start, on a field one | RED  |  0    |  0    |  0 | By the wall | WRONG_START    | On Center, not the expected start
          an own start, away from it   | RED  | -1    | -1.5  |  0 | By the wall | NOT_ON_A_START | Not on a start: By the wall is 1.50 m and 0° away
          """)
  void aRobotIsReadyOnTheExpectedStartOrSaysWhatItIsOn(
      String label,
      @Nullable Alliance alliance,
      double dx,
      double dy,
      double degrees,
      @Nullable String expected,
      Placement.Status status,
      String described) {
    Pose2d from = alliance == Alliance.BLUE ? pose(1, 4, 0) : pose(15, 4, 180);
    Pose2d robot =
        pose(from.getX() + dx, from.getY() + dy, from.getRotation().getDegrees() + degrees);

    Placement placement =
        Placement.check(
            FIELD,
            TestFields.BUMPERS,
            Optional.ofNullable(alliance),
            robot,
            expected == null ? null : PLACES.get(expected),
            0.10,
            5);

    assertThat(placement.status()).isEqualTo(status);
    assertThat(placement.describe()).isEqualTo(described);
  }

  @Test
  void aFieldWithoutStartsSaysSo() {
    Placement placement =
        Placement.check(
            TestFields.FIELD,
            TestFields.BUMPERS,
            Optional.of(Alliance.RED),
            pose(15, 4, 180),
            null,
            0.1,
            5);

    assertThat(placement.describe()).isEqualTo("No start positions");
  }
}
