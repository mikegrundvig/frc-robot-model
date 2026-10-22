package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.pose;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/** A start stands a robot on it by its bumpers: 0.9 m long and 0.8 m wide here. */
class StartTest {
  /**
   * A start on the spot (2, 3), its robot facing the given way, with each point of the bumpers on
   * the spot: the robot's center is half a length and half a width away, in the robot's own axes.
   * Red's is blue's turned half a turn about the field's center (16 m by 8 m).
   */
  @ParameterizedTest(name = "{0} facing {1}°, {2}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          CENTER             |  0 | BLUE |  2     |  3     |   0
          FRONT_BUMPER       |  0 | BLUE |  1.55  |  3     |   0
          BACK_BUMPER        |  0 | BLUE |  2.45  |  3     |   0
          LEFT_BUMPER        |  0 | BLUE |  2     |  2.6   |   0
          RIGHT_BUMPER       |  0 | BLUE |  2     |  3.4   |   0
          FRONT_LEFT_CORNER  |  0 | BLUE |  1.55  |  2.6   |   0
          FRONT_RIGHT_CORNER |  0 | BLUE |  1.55  |  3.4   |   0
          BACK_LEFT_CORNER   |  0 | BLUE |  2.45  |  2.6   |   0
          BACK_RIGHT_CORNER  |  0 | BLUE |  2.45  |  3.4   |   0
          FRONT_BUMPER       | 90 | BLUE |  2     |  2.55  |  90
          BACK_LEFT_CORNER   | 90 | BLUE |  2.4   |  3.45  |  90
          FRONT_BUMPER       |  0 | RED  | 14.45  |  5     | 180
          """)
  void aRobotStandsWithItsAnchorOnTheSpot(
      Start.Anchor anchor,
      double facingDegrees,
      Alliance alliance,
      double x,
      double y,
      double degrees) {
    Start start = new Start("Start", pose(2, 3, facingDegrees), anchor);

    Pose2d robot = start.on(TestFields.FIELD, alliance, TestFields.BUMPERS);

    assertThat(robot.getX()).isCloseTo(x, within(1e-9));
    assertThat(robot.getY()).isCloseTo(y, within(1e-9));
    assertThat(robot.getRotation().minus(Rotation2d.fromDegrees(degrees)).getRadians())
        .isCloseTo(0, within(1e-9));
  }

  @ParameterizedTest
  @CsvSource({"''", "'  '"})
  void aStartNeedsAName(String name) {
    assertThatThrownBy(() -> Start.at(name, Pose2d.ZERO))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("a start needs a name");
  }
}
