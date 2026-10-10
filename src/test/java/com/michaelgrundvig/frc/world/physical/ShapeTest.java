package com.michaelgrundvig.frc.world.physical;

import static com.michaelgrundvig.frc.world.physical.Cases.assertRefused;
import static com.michaelgrundvig.frc.world.physical.Cases.refused;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/** Shapes: where each kind is from its pose, and how far a point is from it. */
class ShapeTest {
  private static final Translation3d SIZE = new Translation3d(1, 1, 1);

  private static Material aluminum() {
    return Material.builder().name("aluminum").build();
  }

  /** A 2 x 1 x 0.5 m box centered at (1, 1, 1), turned a quarter turn about z: 2 m along y. */
  private static Shape.Box box() {
    return Shape.Box.builder()
        .pose(new Pose3d(1, 1, 1, new Rotation3d(0, 0, Math.PI / 2)))
        .size(new Translation3d(2, 1, 0.5))
        .material(aluminum())
        .build();
  }

  @Test
  void aBoxIsCenteredOnItsPoseAndItsPointsBoundIt() {
    List<Translation3d> corners = box().points();

    assertThat(box().center()).isEqualTo(new Translation3d(1, 1, 1));
    assertThat(box().material()).isEqualTo(aluminum());
    assertThat(corners).hasSize(8);
    assertThat(corners.stream().mapToDouble(Translation3d::getX).max().orElseThrow())
        .isCloseTo(1.5, within(1e-12));
    assertThat(corners.stream().mapToDouble(Translation3d::getY).max().orElseThrow())
        .isCloseTo(2, within(1e-12));
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          along its length, inside   | 1   | 1.9 | 1 | true  | 0
          across its length, outside | 1.9 | 1   | 1 | false | 0.4
          above its top              | 1   | 1   | 2 | false | 0.75
          off a corner               | 1.8 | 2.4 | 1 | false | 0.5
          """)
  void aBoxRunsAlongItsAxesAndSaysHowFarAPointIs(
      String label, double x, double y, double z, boolean inside, double distance) {
    Translation3d point = new Translation3d(x, y, z);

    assertThat(box().contains(point)).isEqualTo(inside);
    assertThat(box().distance(point)).isCloseTo(distance, within(1e-12));
  }

  static Stream<Arguments> refusals() {
    Pose3d at = Pose3d.ZERO;
    return Stream.of(
        refused(
            "a box flat in one axis",
            () ->
                Shape.Box.builder()
                    .pose(at)
                    .size(new Translation3d(1, 0, 1))
                    .material(aluminum())
                    .build(),
            "Box.size",
            "positive along each axis"),
        refused(
            "a box nowhere",
            () ->
                Shape.Box.builder()
                    .pose(new Pose3d(Double.NaN, 0, 0, Rotation3d.ZERO))
                    .size(SIZE)
                    .material(aluminum())
                    .build(),
            "Box.pose"),
        refused(
            "a material with no name",
            () -> Material.builder().name(" ").build(),
            "Material.name"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void aShapeMustBeOne(ThrowingCallable building, String[] pieces) {
    assertRefused(building, pieces);
  }
}
