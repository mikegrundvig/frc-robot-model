package com.michaelgrundvig.frc.world.physical;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.world.config.ConfigException;
import java.util.List;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/** Mass properties: how much a part weighs, where, and how hard it is to turn. */
class InertiaTest {
  private static final Rotation3d QUARTER = new Rotation3d(0, 0, Math.PI / 2);
  private static final Translation3d UNIT = new Translation3d(1, 1, 1);

  private static Inertia of(
      double ixx, double iyy, double izz, double ixy, double ixz, double iyz) {
    return Inertia.builder().ixx(ixx).iyy(iyy).izz(izz).ixy(ixy).ixz(ixz).iyz(iyz).build();
  }

  private static MassProperties part(double kilograms, double x, Inertia inertia) {
    return MassProperties.builder()
        .mass(Kilograms.of(kilograms))
        .cg(new Translation3d(x, 0, 0))
        .inertia(inertia)
        .build();
  }

  private static MassProperties cube(double x) {
    return part(1, x, Inertia.box(Kilograms.of(1), UNIT));
  }

  @Test
  void aBoxsInertiaIsItsMassSpreadEvenly() {
    // A 60 kg robot, 0.8 m by 0.7 m by 0.5 m tall.
    Inertia box = Inertia.box(Kilograms.of(60), new Translation3d(0.8, 0.7, 0.5));

    assertThat(box.izz()).isCloseTo(60 * (0.64 + 0.49) / 12, within(1e-12));
    assertThat(box.ixx()).isCloseTo(60 * (0.49 + 0.25) / 12, within(1e-12));
    assertThat(box.iyy()).isCloseTo(60 * (0.64 + 0.25) / 12, within(1e-12));
    assertThat(List.of(box.ixy(), box.ixz(), box.iyz())).containsOnly(0.0);
  }

  /**
   * A 1 kg bar, 1 m by 0.2 m by 0.2 m, turned 45° about z, has ixx = iyy = 0.04667, izz = 0.08667.
   */
  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          a turned bar's own entries         | 0.04666667 | 0.04666667 | 0.08666667 | -0.04 | 0 | 0 |
          a product no body turns by         | 0.04666667 | 0.04666667 | 0.08666667 | 0.1   | 0 | 0 | no body turns that way
          products no body has together      | 1          | 1          | 1          | -0.9  | -0.9 | -0.9 | too large together for the moments (1, 1, 1)
          a negative moment                  | -1         | 1          | 1          | 0     | 0 | 0 | Inertia.ixx: -1 must be zero or more
          a moment over the other two's sum  | 3          | 1          | 1          | 0     | 0 | 0 | each must be at most the sum of the other two
          a product that is no number        | 1          | 1          | 1          | NaN   | 0 | 0 | Inertia.ixy
          """)
  void anInertiaMustBeABodys(
      String label,
      double ixx,
      double iyy,
      double izz,
      double ixy,
      double ixz,
      double iyz,
      @Nullable String refusal) {
    if (refusal == null) {
      assertThatCode(() -> of(ixx, iyy, izz, ixy, ixz, iyz)).doesNotThrowAnyException();
    } else {
      assertThatThrownBy(() -> of(ixx, iyy, izz, ixy, ixz, iyz))
          .isInstanceOf(ConfigException.class)
          .hasMessageContaining(refusal);
    }
  }

  @ParameterizedTest(name = "{0}")
  @CsvSource(
      delimiter = '|',
      textBlock =
          """
          a negative mass          | -1 | 0   | MassProperties.mass
          a mass of nothing known  | -1 | 0   | must be zero or more
          a center that is no place| 1  | NaN | MassProperties.cg
          """)
  void massPropertiesMustBeAPartsOwn(String label, double kilograms, double z, String refusal) {
    assertThatThrownBy(
            () ->
                MassProperties.builder()
                    .mass(Kilograms.of(kilograms))
                    .cg(new Translation3d(0, 0, z))
                    .inertia(Inertia.ZERO)
                    .build())
        .hasMessageContaining(refusal);
  }

  @Test
  void massPropertiesSayWhereTheWeightIs() {
    MassProperties robot =
        MassProperties.builder()
            .mass(Pounds.of(135))
            .cg(new Translation3d(0.02, -0.01, 0.2))
            .inertia(Inertia.box(Pounds.of(135), new Translation3d(0.8, 0.7, 0.5)))
            .build();

    assertThat(robot.mass().in(Kilograms)).isCloseTo(61.235, within(1e-3));
    assertThat(robot.cg().getZ()).isEqualTo(0.2);
  }

  /**
   * What nobody has given is zero, not a guess: no mass, at the frame's origin, turning freely. It
   * moves, turns, adds and comes away as nothing; and a point mass, with no inertia of its own, is
   * a part too. Only a whole body must be solid.
   */
  @Test
  void noneKnownIsZeroAndAddsNothing() {
    MassProperties none = MassProperties.NONE;
    MassProperties seen =
        none.transformedBy(
            new Transform3d(new Translation3d(1, 2, 3), new Rotation3d(0.3, -0.2, 1.1)));

    assertThat(none.isMassless()).isTrue();
    assertThat(none.inertia().isZero()).isTrue();
    assertThat(seen.isMassless()).isTrue();
    assertThat(seen.inertia().isZero()).isTrue();
    assertThat(MassProperties.sum(List.of(none, none))).isEqualTo(MassProperties.NONE);
    assertThat(MassProperties.sum(List.of(none, cube(0.5))).cg().getX()).isEqualTo(0.5);
    assertThat(cube(0.5).minus(none)).isEqualTo(cube(0.5));
    assertThat(part(2, 0.5, Inertia.ZERO).inertia().rotated(new Rotation3d(0, 0, 1)).isZero())
        .as("a point mass")
        .isTrue();
    assertThat(Inertia.ZERO.whyNotSolid()).contains("turns too easily");
    assertThat(Inertia.box(Kilograms.of(1), UNIT).whyNotSolid()).isNull();
    assertThat(of(0, 1, 1, 0, 0, 0).whyNotSolid())
        .as("a thin rod along x: a part, no body")
        .isNotNull();
  }

  /** A bar along x, turned a quarter turn about z, lies along y: x and y trade places. */
  @Test
  void aTurnedInertiaIsTheSameBodyTurned() {
    Inertia bar = Inertia.box(Kilograms.of(1), new Translation3d(1, 0.2, 0.2));
    Inertia turned = bar.rotated(QUARTER);
    // Turned 45 degrees, it's as hard to turn about x as the unturned bar about that diagonal.
    Inertia diagonal = bar.rotated(new Rotation3d(0, 0, Math.PI / 4));

    assertThat(turned.ixx()).isCloseTo(bar.iyy(), within(1e-12));
    assertThat(turned.iyy()).isCloseTo(bar.ixx(), within(1e-12));
    assertThat(turned.izz()).isCloseTo(bar.izz(), within(1e-12));
    assertThat(turned.ixy()).isCloseTo(0, within(1e-12));
    assertThat(bar.about(new Vector3(0, 0, 3))).as("an axis of any length").isEqualTo(bar.izz());
    assertThat(bar.about(new Vector3(1, 1, 0)))
        .isCloseTo((bar.ixx() + bar.iyy()) / 2, within(1e-12));
    assertThat(diagonal.ixx()).isCloseTo(bar.about(new Vector3(1, -1, 0)), within(1e-12));
    assertThat(diagonal.ixy()).as("the tensor's own entry").isNegative();
  }

  /** Two cubes side by side are a box twice as long; take one away and the other is left. */
  @Test
  void partsAddAndComeApart() {
    MassProperties left = cube(-0.5);
    MassProperties right = cube(0.5);
    MassProperties both = MassProperties.sum(List.of(left, right));
    Inertia box = Inertia.box(Kilograms.of(2), new Translation3d(2, 1, 1));
    MassProperties less = both.minus(right);

    assertThat(both.mass().in(Kilograms)).isEqualTo(2);
    assertThat(both.cg().getNorm()).isZero();
    assertThat(both.inertia().ixx()).isCloseTo(box.ixx(), within(1e-12));
    assertThat(both.inertia().iyy()).isCloseTo(box.iyy(), within(1e-12));
    assertThat(both.inertia().izz()).isCloseTo(box.izz(), within(1e-12));
    assertThat(less.cg().getX()).isCloseTo(-0.5, within(1e-12));
    assertThat(less.inertia().izz()).isCloseTo(left.inertia().izz(), within(1e-12));
    assertThatThrownBy(() -> left.minus(both))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageStartingWith("nothing is left: 2 Kg taken from 1 Kg");
    assertThatThrownBy(() -> MassProperties.sum(List.of()))
        .isInstanceOf(IllegalArgumentException.class);
  }

  /** A part given in its joint's frame, seen from the robot's: moved, and turned with it. */
  @Test
  void aPartSeenFromAnotherFrame() {
    MassProperties bar = part(1, 0.5, Inertia.box(Kilograms.of(1), new Translation3d(1, 0.2, 0.2)));
    MassProperties seen = bar.transformedBy(new Transform3d(new Translation3d(0, 0, 1), QUARTER));

    assertThat(seen.cg().getX()).isCloseTo(0, within(1e-12));
    assertThat(seen.cg().getY()).isCloseTo(0.5, within(1e-12));
    assertThat(seen.cg().getZ()).isEqualTo(1);
    assertThat(seen.inertia().iyy()).isCloseTo(bar.inertia().ixx(), within(1e-12));
  }
}
