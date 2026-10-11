package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.*;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.RadiansPerSecond;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Where a named speed or voltage stops being taken, as a table: an output on the standard robot's
 * roller, and how many problems, doubts or unchecked joints must come of it. The roller's motor
 * turns it at 2000 rpm with a full battery, 12 V. The messages themselves are in {@link
 * PlacesTest}; these rows are the edges.
 */
class JointSpeedChecksTest {
  /** The roller's free speed, in radians a second, as the model works it out. */
  private static final double FREE_SPEED = competition().math().freeSpeed(ROLLER);

  /** The largest speed that still counts as the free speed: a rounding step past it. */
  private static final double JUST_AT_FREE_SPEED = FREE_SPEED * (1 + 1e-9);

  private static final double NOMINAL_VOLTS = 12;
  private static final double JUST_AT_NOMINAL_VOLTS = NOMINAL_VOLTS * (1 + 1e-9);

  /** The standard robot's pivot and a roller with one output, on the roller's usual motor. */
  private static RobotMath rollerWith(Output output) {
    Joint.Revolute roller = named(ROLLER, List.of(), List.of(output));
    return RobotMath.whole(
            ConfigFixtures.body(),
            List.of(PIVOT, roller),
            List.of(),
            List.of(
                DRIVES.get(0),
                MechanismFixtures.motors(roller, Motor.KRAKEN_X60, 1, gearing(3, 1))),
            List.of())
        .orElseThrow();
  }

  /** The same roller with nothing to turn it. */
  private static RobotMath rollerWithNoMotor(Output output) {
    Joint.Revolute roller = named(ROLLER, List.of(), List.of(output));
    return RobotMath.whole(
            ConfigFixtures.body(),
            List.of(PIVOT, roller),
            List.of(),
            List.of(DRIVES.get(0)),
            List.of())
        .orElseThrow();
  }

  private static Output speed(double radiansPerSecond) {
    return Output.of("running", RadiansPerSecond.of(radiansPerSecond));
  }

  private static int problems(RobotMath math) {
    List<Finding> problems = new ArrayList<>();
    JointSpeedChecks.problems(math, problems);
    return problems.size();
  }

  @ParameterizedTest
  @MethodSource({"refused", "doubted", "unchecked"})
  void itFinds(Supplier<Integer> found, int howMany) {
    assertThat(found.get()).isEqualTo(howMany);
  }

  private static Arguments finds(String label, Supplier<Integer> found, int howMany) {
    return argumentSet(label, found, howMany);
  }

  // spotless:off

  /** A speed is refused past the free speed, and a voltage past the motors' nominal voltage. */
  static Stream<Arguments> refused() {
    return Stream.of(
        finds("a speed at the free speed is taken", () -> problems(rollerWith(speed(FREE_SPEED))), 0),
        finds("a speed a rounding step past it is taken", () -> problems(rollerWith(speed(JUST_AT_FREE_SPEED))), 0),
        finds("the next speed up is refused", () -> problems(rollerWith(speed(Math.nextUp(JUST_AT_FREE_SPEED)))), 1),
        finds("backward counts the same", () -> problems(rollerWith(speed(-Math.nextUp(JUST_AT_FREE_SPEED)))), 1),
        finds("a voltage at the nominal voltage is taken", () -> problems(rollerWith(Output.volts("pushing", NOMINAL_VOLTS))), 0),
        finds("a voltage a rounding step past it is taken", () -> problems(rollerWith(Output.volts("pushing", JUST_AT_NOMINAL_VOLTS))), 0),
        finds("the next voltage up is refused", () -> problems(rollerWith(Output.volts("pushing", Math.nextUp(JUST_AT_NOMINAL_VOLTS)))), 1),
        finds("backward counts the same", () -> problems(rollerWith(Output.volts("pushing", -Math.nextUp(JUST_AT_NOMINAL_VOLTS)))), 1),
        finds("a whole battery as a duty cycle is taken", () -> problems(rollerWith(Output.dutyCycle("flat out", 1))), 0),
        finds("a joint with no motors is not checked", () -> problems(rollerWithNoMotor(speed(10 * FREE_SPEED))), 0));
  }

  /** A speed is doubted from just past 90% of the free speed, up to the free speed. */
  static Stream<Arguments> doubted() {
    double noted = JointSpeedChecks.NOTED_SHARE * FREE_SPEED;
    return Stream.of(
        finds("a speed at 90% of the free speed is not doubted", () -> JointSpeedChecks.doubts(rollerWith(speed(noted))).size(), 0),
        finds("the next speed up is doubted", () -> JointSpeedChecks.doubts(rollerWith(speed(Math.nextUp(noted)))).size(), 1),
        finds("backward counts the same", () -> JointSpeedChecks.doubts(rollerWith(speed(-Math.nextUp(noted)))).size(), 1),
        finds("a speed a rounding step past the free speed is doubted", () -> JointSpeedChecks.doubts(rollerWith(speed(JUST_AT_FREE_SPEED))).size(), 1),
        finds("a speed plainly past the free speed is refused instead, and not doubted", () -> JointSpeedChecks.doubts(rollerWith(speed(FREE_SPEED * (1 + 2e-9)))).size(), 0),
        finds("a voltage is never doubted", () -> JointSpeedChecks.doubts(rollerWith(Output.volts("pushing", 11.9))).size(), 0),
        finds("a joint with no motors is not doubted", () -> JointSpeedChecks.doubts(rollerWithNoMotor(speed(0.95 * FREE_SPEED))).size(), 0));
  }

  /** A joint with no motors is listed when it names a speed or a voltage. */
  static Stream<Arguments> unchecked() {
    return Stream.of(
        finds("a speed with no motors is unchecked", () -> JointSpeedChecks.unchecked(rollerWithNoMotor(speed(1))).size(), 1),
        finds("a voltage with no motors is unchecked", () -> JointSpeedChecks.unchecked(rollerWithNoMotor(Output.volts("pushing", 6))).size(), 1),
        finds("a duty cycle checks itself", () -> JointSpeedChecks.unchecked(rollerWithNoMotor(Output.dutyCycle("creep", 0.2))).size(), 0),
        finds("a joint with motors is checked", () -> JointSpeedChecks.unchecked(rollerWith(speed(1))).size(), 0));
  }

  // spotless:on
}
