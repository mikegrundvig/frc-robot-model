package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.Alliance.BLUE;
import static com.michaelgrundvig.frc.world.field.Alliance.RED;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;

import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

class HeadingTest {
  private static final Place HUB = Place.between("Hub", 2, 3);
  private static final FieldModel FIELD = TestFields.model(List.of(), List.of(HUB), List.of());

  /** The headings the tables name, built as each test runs. */
  private static Heading heading(String name) {
    return switch (name) {
      case "hub" -> Heading.toward(HUB);
      case "hub back" -> Heading.toward(HUB).withBack();
      case "field 90" -> Heading.field(Rotation2d.fromDegrees(90));
      case "field 90 back" -> Heading.field(Rotation2d.fromDegrees(90)).withBack();
      case "field 180" -> Heading.field(Rotation2d.PI);
      case "straight" -> Heading.downfield(Degrees.of(0));
      case "left 20" -> Heading.downfield(Degrees.of(20));
      case "right 20" -> Heading.downfield(Degrees.of(-20));
      case "right 20 back" -> Heading.downfield(Degrees.of(-20)).withBack();
      case "behind" -> Heading.downfield(Degrees.of(180));
      default -> throw new AssertionError(name);
    };
  }

  private static Pose2d at(double x, double y) {
    return new Pose2d(x, y, Rotation2d.ZERO);
  }

  /** The heading each kind gives from a spot, for each alliance; null where it gives none. */
  static Stream<Arguments> resolving() {
    return Stream.of(
        argumentSet("a field heading, blue", "field 90", at(1, 1), BLUE, 90.0),
        argumentSet("a field heading, red: the same", "field 90", at(9, 5), RED, 90.0),
        argumentSet("a field heading needs no alliance", "field 90", at(9, 5), null, 90.0),
        argumentSet("downfield 0 is blue's forward", "straight", at(1, 1), BLUE, 0.0),
        argumentSet("downfield 0 is red's forward", "straight", at(1, 1), RED, 180.0),
        argumentSet("downfield left 20, blue", "left 20", at(1, 1), BLUE, 20.0),
        argumentSet("downfield left 20, red", "left 20", at(1, 1), RED, 200.0),
        argumentSet("downfield right 20, red", "right 20", at(1, 1), RED, 160.0),
        argumentSet("downfield needs the alliance", "left 20", at(1, 1), null, null),
        argumentSet("blue's hub ahead", "hub", at(2, 4), BLUE, 0.0),
        argumentSet("blue's hub on the right", "hub", at(4.5, 6), BLUE, -90.0),
        argumentSet("blue's hub on the left", "hub", at(4.5, 2), BLUE, 90.0),
        argumentSet("red's hub ahead", "hub", at(13.5, 4), RED, 180.0),
        argumentSet("red's hub on the right", "hub", at(11.5, 6), RED, -90.0),
        argumentSet("red's hub from the far side", "hub", at(2, 4), RED, 0.0),
        argumentSet("a place needs the alliance", "hub", at(2, 4), null, null),
        argumentSet("on top of a place: no bearing", "hub", at(4.5, 4.03), BLUE, null),
        argumentSet("just off a place: a bearing", "hub", at(4.5, 4.1), BLUE, -90.0),
        argumentSet("the back faces a place", "hub back", at(2, 4), BLUE, 180.0),
        argumentSet("the back faces downfield, blue", "right 20 back", at(2, 4), BLUE, 160.0),
        argumentSet("the back faces downfield, red", "right 20 back", at(2, 4), RED, -20.0),
        argumentSet("the back faces a field heading", "field 90 back", at(2, 4), RED, -90.0),
        argumentSet("the back still needs the alliance", "hub back", at(2, 4), null, null));
  }

  @ParameterizedTest
  @MethodSource("resolving")
  void aHeadingResolvesFromWhereTheRobotIs(
      String which, Pose2d robot, @Nullable Alliance alliance, @Nullable Double degrees) {
    Optional<Rotation2d> resolved =
        heading(which).resolve(robot, FIELD, Optional.ofNullable(alliance));

    if (degrees == null) {
      assertThat(resolved).isEmpty();
    } else {
      assertThat(Math.IEEEremainder(resolved.orElseThrow().getDegrees() - degrees, 360))
          .as("heading %s, expected %s°", resolved, degrees)
          .isCloseTo(0.0, within(1e-6));
    }
  }

  /** The bearing's rate in radians a second, for a robot moving at 1 m/s along +x. */
  static Stream<Arguments> turning() {
    return Stream.of(
        argumentSet("passing below a place: counterclockwise", "hub", at(4.5, 2), BLUE, 0.5),
        argumentSet("the back faces it: the same rate", "hub back", at(4.5, 2), BLUE, 0.5),
        argumentSet("on a place: none", "hub", at(4.5, 4.03), BLUE, 0.0),
        argumentSet("a field heading: none", "field 180", at(1, 1), BLUE, 0.0),
        argumentSet("downfield: none", "left 20", at(1, 1), RED, 0.0),
        argumentSet("a place without an alliance: none", "hub", at(1, 1), null, 0.0));
  }

  @ParameterizedTest
  @MethodSource("turning")
  void aHeadingTurnsWithTheRobotsVelocity(
      String which, Pose2d robot, @Nullable Alliance alliance, double rate) {
    Optional<Alliance> side = Optional.ofNullable(alliance);

    assertThat(heading(which).rate(robot, FIELD, side, new Translation2d(1, 0)))
        .isCloseTo(rate, within(1e-9));
  }

  /** The bearing's rate is the change in the bearing a moment on, for either alliance. */
  @ParameterizedTest
  @EnumSource(Alliance.class)
  void aPlacesBearingRateIsItsChangeAMomentOn(Alliance alliance) {
    double step = 1e-4;
    Pose2d robot = at((alliance == BLUE ? 4.5 : 11.5) + 1, 2);
    Translation2d velocity = new Translation2d(-1.5, 0.7);
    Pose2d later = at(robot.getX() + velocity.getX() * step, robot.getY() + velocity.getY() * step);
    Optional<Alliance> side = Optional.of(alliance);
    Heading hub = heading("hub");

    double before = hub.resolve(robot, FIELD, side).orElseThrow().getDegrees();
    double after = hub.resolve(later, FIELD, side).orElseThrow().getDegrees();
    double measured = Math.toRadians(Math.IEEEremainder(after - before, 360)) / step;

    assertThat(hub.rate(robot, FIELD, side, velocity)).isCloseTo(measured, within(1e-3));
  }

  /** What a heading is called in logs and reasons. */
  static Stream<Arguments> names() {
    return Stream.of(
        argumentSet("a place", "hub", "the Hub"),
        argumentSet("a place, back first", "hub back", "the Hub, back first"),
        argumentSet("downfield", "straight", "downfield"),
        argumentSet("right of downfield", "right 20", "20° right of downfield"),
        argumentSet("left of downfield", "left 20", "20° left of downfield"),
        argumentSet("toward the drivers", "behind", "back toward the drivers"),
        argumentSet("a field heading", "field 90", "field 90°"));
  }

  @ParameterizedTest
  @MethodSource("names")
  void itIsNamedForLogsAndReasons(String which, String name) {
    assertThat(heading(which).name()).isEqualTo(name);
    assertThat(heading(which)).hasToString(name);
  }
}
