package com.michaelgrundvig.frc.world.physical;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/** Shapes: where each kind is from its pose, and how far a point is from it. */
class ShapeTest {
  private static final Translation3d SIZE = new Translation3d(1, 1, 1);

  private static Material aluminum() {
    return Material.builder().name("aluminum").build();
  }

  private static Shape.Box box(Pose3d pose, Translation3d size) {
    return Shape.Box.builder().pose(pose).size(size).material(aluminum()).build();
  }

  /** A 2 x 1 x 0.5 m box centered at (1, 1, 1), turned a quarter turn about z: 2 m along y. */
  private static Shape.Box box() {
    return box(
        new Pose3d(1, 1, 1, new Rotation3d(0, 0, Math.PI / 2)), new Translation3d(2, 1, 0.5));
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

  @Test
  void aShapeMustBeOne() {
    assertThatThrownBy(() -> box(Pose3d.ZERO, new Translation3d(1, 0, 1)))
        .hasMessageContaining("Box.size")
        .hasMessageContaining("positive along each axis");
    assertThatThrownBy(() -> box(new Pose3d(Double.NaN, 0, 0, Rotation3d.ZERO), SIZE))
        .hasMessageContaining("Box.pose");
    assertThatThrownBy(() -> Material.builder().name(" ").build())
        .hasMessageContaining("Material.name");
  }
}
