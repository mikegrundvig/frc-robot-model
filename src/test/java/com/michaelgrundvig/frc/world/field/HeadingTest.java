package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

class HeadingTest {
  private static final Place HUB = Place.between("Hub", 2, 3);

  private final FieldModel field = TestFields.model(List.of(), List.of(HUB), List.of());

  private static Optional<Alliance> blue() {
    return Optional.of(Alliance.BLUE);
  }

  private static Optional<Alliance> red() {
    return Optional.of(Alliance.RED);
  }

  private static Pose2d at(double x, double y) {
    return new Pose2d(x, y, Rotation2d.ZERO);
  }

  /** Degrees of a resolved heading, the nearer way round from zero. */
  private static double degrees(Optional<Rotation2d> heading) {
    return heading.orElseThrow().getDegrees();
  }

  /** The two headings are one angle, whichever turn they're written in. */
  private static void assertAngle(Optional<Rotation2d> actual, double degrees) {
    assertThat(Math.IEEEremainder(degrees(actual) - degrees, 360))
        .as("heading " + actual + ", expected " + degrees + "°")
        .isCloseTo(0.0, within(1e-6));
  }

  @Test
  void aFieldHeadingIsTheSameForBothAlliancesAndNeedsNeither() {
    Heading ninety = Heading.field(Rotation2d.fromDegrees(90));

    assertAngle(ninety.resolve(at(1, 1), field, blue()), 90);
    assertAngle(ninety.resolve(at(9, 5), field, red()), 90);
    assertAngle(ninety.resolve(at(9, 5), field, Optional.empty()), 90);
  }

  @Test
  void downfieldIsFromEachAlliancesOwnForward() {
    Heading left = Heading.downfield(Degrees.of(20));
    Heading straight = Heading.downfield(Degrees.of(0));

    assertAngle(straight.resolve(at(1, 1), field, blue()), 0);
    assertAngle(straight.resolve(at(1, 1), field, red()), 180);
    assertAngle(left.resolve(at(1, 1), field, blue()), 20);
    assertAngle(left.resolve(at(1, 1), field, red()), 200);
    assertAngle(Heading.downfield(Degrees.of(-20)).resolve(at(1, 1), field, red()), 160);
  }

  @Test
  void downfieldNeedsTheAllianceAndSaysNothingWithout() {
    assertThat(Heading.downfield(Degrees.of(20)).resolve(at(1, 1), field, Optional.empty()))
        .isEmpty();
  }

  @Test
  void aPlaceIsFacedAsItIsForTheRobotsAlliance() {
    Heading hub = Heading.toward(HUB);

    assertAngle(hub.resolve(at(2, 4), field, blue()), 0); // blue's hub at (4.5, 4)
    assertAngle(hub.resolve(at(4.5, 6), field, blue()), -90);
    assertAngle(hub.resolve(at(4.5, 2), field, blue()), 90);
    assertAngle(hub.resolve(at(13.5, 4), field, red()), 180); // red's hub at (11.5, 4)
    assertAngle(hub.resolve(at(11.5, 6), field, red()), -90);
    assertAngle(hub.resolve(at(2, 4), field, red()), 0); // from the far side of the field, too
  }

  @Test
  void aPlaceNeedsTheAlliance() {
    assertThat(Heading.toward(HUB).resolve(at(2, 4), field, Optional.empty())).isEmpty();
  }

  @Test
  void onTheirPlaceTheRobotHasNoBearing() {
    Heading hub = Heading.toward(HUB);

    assertThat(hub.resolve(at(4.5, 4.03), field, blue())).isEmpty();
    assertThat(hub.rate(at(4.5, 4.03), field, blue(), new Translation2d(1, 0))).isZero();
    assertThat(hub.resolve(at(4.5, 4.1), field, blue())).isPresent();
  }

  @Test
  void theBackFacesItInsteadOfTheFront() {
    assertAngle(Heading.toward(HUB).withBack().resolve(at(2, 4), field, blue()), 180);
    assertAngle(
        Heading.downfield(Degrees.of(-20)).withBack().resolve(at(2, 4), field, blue()), 160);
    assertAngle(Heading.downfield(Degrees.of(-20)).withBack().resolve(at(2, 4), field, red()), -20);
    assertAngle(
        Heading.field(Rotation2d.fromDegrees(90)).withBack().resolve(at(2, 4), field, red()), -90);
    assertThat(Heading.toward(HUB).withBack().resolve(at(2, 4), field, Optional.empty())).isEmpty();
  }

  /** The bearing's rate is the change in the bearing a moment on, for either alliance. */
  @Test
  void aPlacesBearingTurnsWithTheRobotsVelocityAcrossIt() {
    Heading hub = Heading.toward(HUB);
    double step = 1e-4;
    for (Alliance alliance : Alliance.values()) {
      double hubX = alliance == Alliance.BLUE ? 4.5 : 11.5;
      Pose2d robot = at(hubX + 1, 2);
      Translation2d velocity = new Translation2d(-1.5, 0.7);
      Pose2d later =
          at(robot.getX() + velocity.getX() * step, robot.getY() + velocity.getY() * step);

      double before = degrees(hub.resolve(robot, field, Optional.of(alliance)));
      double after = degrees(hub.resolve(later, field, Optional.of(alliance)));
      double measured = Math.toRadians(Math.IEEEremainder(after - before, 360)) / step;

      assertThat(hub.rate(robot, field, Optional.of(alliance), velocity))
          .as("rate on " + alliance)
          .isCloseTo(measured, within(1e-3));
    }
  }

  @Test
  void passingBelowThePlaceSweepsTheBearingCounterclockwise() {
    // 2 m below the hub, going right at 1 m/s: the hub swings from ahead-left to behind.
    assertThat(Heading.toward(HUB).rate(at(4.5, 2), field, blue(), new Translation2d(1, 0)))
        .isCloseTo(0.5, within(1e-9));
    assertThat(
            Heading.toward(HUB).withBack().rate(at(4.5, 2), field, blue(), new Translation2d(1, 0)))
        .as("the back faces it, the same rate")
        .isCloseTo(0.5, within(1e-9));
  }

  @Test
  void aFixedHeadingDoesNotTurn() {
    Translation2d fast = new Translation2d(3, 3);

    assertThat(Heading.field(Rotation2d.PI).rate(at(1, 1), field, blue(), fast)).isZero();
    assertThat(Heading.downfield(Degrees.of(20)).rate(at(1, 1), field, red(), fast)).isZero();
    assertThat(Heading.toward(HUB).rate(at(1, 1), field, Optional.empty(), fast)).isZero();
  }

  @Test
  void itIsNamedForLogsAndReasons() {
    assertThat(Heading.toward(HUB).name()).isEqualTo("the Hub");
    assertThat(Heading.toward(HUB).withBack().name()).isEqualTo("the Hub, back first");
    assertThat(Heading.downfield(Degrees.of(0)).name()).isEqualTo("downfield");
    assertThat(Heading.downfield(Degrees.of(-20)).name()).isEqualTo("20° right of downfield");
    assertThat(Heading.downfield(Degrees.of(20)).name()).isEqualTo("20° left of downfield");
    assertThat(Heading.downfield(Degrees.of(180)).name()).isEqualTo("back toward the drivers");
    assertThat(Heading.field(Rotation2d.fromDegrees(90)).name()).isEqualTo("field 90°");
    assertThat(Heading.field(Rotation2d.fromDegrees(90)).toString()).isEqualTo("field 90°");
  }

  @Test
  void headingsAreValues() {
    assertThat(Heading.toward(HUB))
        .isEqualTo(Heading.toward(HUB))
        .hasSameHashCodeAs(Heading.toward(HUB));
    assertThat(Heading.toward(HUB)).isNotEqualTo(Heading.toward(HUB).withBack());
    assertThat(Heading.field(Rotation2d.PI)).isNotEqualTo(Heading.downfield(Degrees.of(180)));
    assertThat(Heading.toward(HUB)).isNotEqualTo("the Hub");
  }
}
