package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.StandardRobot.PIVOT;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.PIVOT_IN;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.PIVOT_OUT;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.ConfigException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Where a definition stops taking a value and starts refusing it, as a table: each row puts one
 * value exactly on an edge, or one step past it, and says whether the part is still built. What
 * each refusal says is in the tests of the part it belongs to; these rows are the edges.
 */
class DefinitionEdgesTest {
  private static final String TAKEN = "taken";
  private static final String REFUSED = "refused";

  /** Whether something can be built: a part's checks throw when they refuse it. */
  private static String built(Runnable build) {
    try {
      build.run();
      return TAKEN;
    } catch (ConfigException e) {
      return REFUSED;
    }
  }

  private static final Body BODY = ConfigFixtures.body();
  private static final double HALF_LENGTH = BODY.frameLength().in(Meters) / 2;
  private static final double HALF_WIDTH = BODY.frameWidth().in(Meters) / 2;
  private static final double HEIGHT = BODY.height().in(Meters);

  /** The test robot's body with its center of mass here. */
  private static String centerOfMassAt(double x, double y, double z) {
    return built(() -> BodyBuilder.builder(BODY).cg(new Translation3d(x, y, z)).build());
  }

  /** The test robot's drivetrain with one corner's wheel moved. */
  private static String cornerAt(int corner, double x, double y) {
    List<DrivetrainGeometry.Corner> corners =
        new ArrayList<>(ConfigFixtures.drivetrain().corners());
    DrivetrainGeometry.ModuleType module = corners.get(corner).module();
    return built(
        () -> {
          corners.set(corner, new DrivetrainGeometry.Corner(module, new Translation2d(x, y)));
          DrivetrainGeometry.builder().corners(corners).build();
        });
  }

  private static final int FRONT_LEFT = 0;
  private static final int FRONT_RIGHT = 1;
  private static final int BACK_LEFT = 2;

  /** The standard robot's pivot, 0° to 135°, driven with these soft limits, in radians. */
  private static String softLimits(double lower, double upper) {
    Range limits = Range.builder().kind(Position.Kind.ANGLE).lower(lower).upper(upper).build();
    return built(
        () ->
            StandardRobot.robotOf(
                PIVOT
                    .motors(Motor.KRAKEN_X60.through(MechanismFixtures.gearing(25, 1)))
                    .control(
                        MechanismControl.position(30)
                            .withSoftLimits(limits)
                            .withHoming(MechanismControl.Homing.assumeAtRest()))));
  }

  private static final double LOWER_STOP = PIVOT.range().orElseThrow().lower();
  private static final double UPPER_STOP = PIVOT.range().orElseThrow().upper();

  @ParameterizedTest
  @MethodSource({"centersOfMass", "corners", "softLimits"})
  void itIs(Supplier<String> built, String takenOrRefused) {
    assertThat(built.get()).isEqualTo(takenOrRefused);
  }

  private static Arguments row(String label, Supplier<String> built, String takenOrRefused) {
    return argumentSet(label, built, takenOrRefused);
  }

  // spotless:off

  /** A center of mass is inside the frame, above the floor and below the robot's height. */
  static Stream<Arguments> centersOfMass() {
    return Stream.of(
        row("over the front edge of the frame", () -> centerOfMassAt(HALF_LENGTH, 0, 0.2), TAKEN),
        row("the next position forward", () -> centerOfMassAt(Math.nextUp(HALF_LENGTH), 0, 0.2), REFUSED),
        row("over the back edge", () -> centerOfMassAt(-HALF_LENGTH, 0, 0.2), TAKEN),
        row("over the side edge of the frame", () -> centerOfMassAt(0, HALF_WIDTH, 0.2), TAKEN),
        row("the next position out to the side", () -> centerOfMassAt(0, Math.nextUp(HALF_WIDTH), 0.2), REFUSED),
        row("on the floor", () -> centerOfMassAt(0, 0, 0), REFUSED),
        row("the least above the floor", () -> centerOfMassAt(0, 0, Double.MIN_VALUE), TAKEN),
        row("at the robot's height", () -> centerOfMassAt(0, 0, HEIGHT), REFUSED),
        row("the next position below its height", () -> centerOfMassAt(0, 0, Math.nextDown(HEIGHT)), TAKEN));
  }

  /** Each corner's wheel is inside its own quarter of the robot, off the lines through the center. */
  static Stream<Arguments> corners() {
    return Stream.of(
        row("the front left wheel in its quarter", () -> cornerAt(FRONT_LEFT, 0.2, 0.2), TAKEN),
        row("the front left wheel on the line along the center", () -> cornerAt(FRONT_LEFT, 0.2, 0), REFUSED),
        row("the front left wheel on the line across the center", () -> cornerAt(FRONT_LEFT, 0, 0.2), REFUSED),
        row("the front right wheel on the line along the center", () -> cornerAt(FRONT_RIGHT, 0.2, 0), REFUSED),
        row("the back left wheel on the line across the center", () -> cornerAt(BACK_LEFT, 0, 0.2), REFUSED),
        row("the back left wheel the least behind the center", () -> cornerAt(BACK_LEFT, -Double.MIN_VALUE, 0.2), TAKEN));
  }

  /** Soft limits lie inside the hard stops, and every named position inside the soft limits. */
  static Stream<Arguments> softLimits() {
    return Stream.of(
        row("soft limits on the hard stops", () -> softLimits(LOWER_STOP, UPPER_STOP), TAKEN),
        row("an upper soft limit a step past the stop", () -> softLimits(LOWER_STOP, Math.nextUp(UPPER_STOP)), REFUSED),
        row("a lower soft limit a step past the stop", () -> softLimits(Math.nextDown(LOWER_STOP), UPPER_STOP), REFUSED),
        row("soft limits on the lowest and highest named positions", () -> softLimits(PIVOT_IN.value(), PIVOT_OUT.value()), TAKEN),
        row("an upper soft limit a step short of the highest position", () -> softLimits(PIVOT_IN.value(), Math.nextDown(PIVOT_OUT.value())), REFUSED),
        row("a lower soft limit a step past the lowest position", () -> softLimits(Math.nextUp(PIVOT_IN.value()), PIVOT_OUT.value()), REFUSED));
  }

  // spotless:on
}
