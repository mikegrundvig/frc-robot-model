package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.station;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Millimeters;

import java.util.List;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

class CameraStationTest {
  /** A station said in full, from its name, each verb once. */
  private static final CameraStation SAID =
      CameraStation.named("left")
          .at(Inches.of(-10.5), Inches.of(12), Inches.of(21))
          .facing(Degrees.of(10), Degrees.of(180))
          .roll(Degrees.of(-2))
          .noiseScale(1.5)
          .measuredWith(3);

  @Test
  void aStationIsSaidByItsVerbs() {
    assertThat(SAID.name()).isEqualTo("left");
    assertThat(SAID.position())
        .isEqualTo(new Translation3d(Inches.of(-10.5), Inches.of(12), Inches.of(21)));
    assertThat(SAID.tiltUp().orElseThrow().in(Degrees)).isCloseTo(10, within(1e-9));
    assertThat(SAID.yaw().orElseThrow().in(Degrees)).isCloseTo(180, within(1e-9));
    assertThat(SAID.rollOrDefault().in(Degrees)).isCloseTo(-2, within(1e-9));
    assertThat(SAID.noiseScaleOrDefault()).isEqualTo(1.5);
    assertThat(SAID.measuredWith()).contains(3);
    CameraStation bare = station("front", new Translation3d());
    assertThat(bare.mountTolerance()).isEqualTo(MountTolerance.UNCHECKED);
    assertThat(bare.measuredWith()).isEmpty();
  }

  /**
   * The mount's signs, each pinned by a point worked out by hand, not by composing rotations as the
   * mount does: a point a camera sees along its own axes, and where that point is on the robot.
   * Tilted up, the camera looks up; yawed left, it looks left; rolled, its left tips up; {@code
   * left} moves the lens left; and with tilt and yaw together the tilt stays up, whichever way it's
   * yawed.
   */
  @Test
  void aStationsMountPutsWhatItsCameraSeesWhereItsSignsSay() {
    double cos20 = 0.9396926207859084;
    double sin20 = 0.3420201433256687;
    record Case(String name, CameraStation station, Translation3d seen, Translation3d onTheRobot) {}
    List<Case> cases =
        List.of(
            new Case(
                "tilted up 20°: what's ahead of it is up",
                change().tiltUp(Degrees.of(20)).build(),
                new Translation3d(1, 0, 0),
                new Translation3d(cos20, 0, sin20)),
            new Case(
                "yawed 90°: what's ahead of it is to the robot's left",
                change().yaw(Degrees.of(90)).build(),
                new Translation3d(1, 0, 0),
                new Translation3d(0, 1, 0)),
            new Case(
                "rolled 90°: its own left is up",
                change().roll(Degrees.of(90)).build(),
                new Translation3d(0, 1, 0),
                new Translation3d(0, 0, 1)),
            new Case(
                "0.2 m left: the lens is to the robot's left",
                change().left(Meters.of(0.2)).build(),
                new Translation3d(0, 0, 0),
                new Translation3d(0, 0.2, 0)),
            new Case(
                "tilted up 20° and yawed 90°: up and to the left",
                change().tiltUp(Degrees.of(20)).yaw(Degrees.of(90)).build(),
                new Translation3d(1, 0, 0),
                new Translation3d(0, cos20, sin20)),
            new Case(
                "at (0.3, 0.2, 0.25), yawed 90°: a point 2 m ahead of the lens",
                change()
                    .forward(Meters.of(0.3))
                    .left(Meters.of(0.2))
                    .up(Meters.of(0.25))
                    .yaw(Degrees.of(90))
                    .build(),
                new Translation3d(2, 0, 0),
                new Translation3d(0.3, 2.2, 0.25)));
    for (Case c : cases) {
      Transform3d mount = c.station().robotToCamera();
      Translation3d onTheRobot =
          c.seen().rotateBy(mount.getRotation()).plus(mount.getTranslation());

      assertThat(onTheRobot.getX()).as(c.name()).isCloseTo(c.onTheRobot().getX(), within(1e-12));
      assertThat(onTheRobot.getY()).as(c.name()).isCloseTo(c.onTheRobot().getY(), within(1e-12));
      assertThat(onTheRobot.getZ()).as(c.name()).isCloseTo(c.onTheRobot().getZ(), within(1e-12));
    }
  }

  @Test
  void aStationTiltedUpHasANegativePitchInWpilibTerms() {
    CameraStation tilted =
        change()
            .forward(Inches.of(11.5))
            .left(Inches.of(11.5))
            .up(Inches.of(9))
            .tiltUp(Degrees.of(20))
            .yaw(Degrees.of(45))
            .build();

    assertThat(tilted.position())
        .isEqualTo(new Translation3d(Inches.of(11.5), Inches.of(11.5), Inches.of(9)));
    assertThat(tilted.robotToCamera())
        .isEqualTo(
            new Transform3d(
                tilted.position(), new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(45))));
  }

  @Test
  void valuesThatCantBeAreRefused() {
    var nan = Double.NaN;
    record Case(String says, ThrowingCallable build) {}
    List<Case> cases =
        List.of(
            new Case("CameraStation[ ].name", () -> change().name(" ").build()),
            new Case(
                "CameraStation[front].forward", () -> change().forward(Inches.of(nan)).build()),
            new Case("CameraStation[front].left", () -> change().left(Inches.of(nan)).build()),
            new Case("CameraStation[front].up", () -> change().up(Inches.of(nan)).build()),
            new Case("CameraStation[front].roll", () -> change().roll(Degrees.of(nan)).build()),
            new Case("CameraStation[front].tiltUp", () -> change().tiltUp(Degrees.of(nan)).build()),
            new Case("CameraStation[front].yaw", () -> change().yaw(Degrees.of(nan)).build()),
            new Case("CameraStation[front].noiseScale", () -> change().noiseScale(0.0).build()),
            new Case("CameraStation[front].measuredWith", () -> change().measuredWith(0).build()),
            new Case(
                "MountTolerance.along", () -> MountTolerance.of(Millimeters.of(-1), Degrees.of(1))),
            new Case(
                "MountTolerance.about", () -> MountTolerance.of(Millimeters.of(1), Degrees.of(-1))),
            new Case(
                "MountTolerance.about: 90 ° must be below 90 °",
                () -> MountTolerance.of(Millimeters.of(1), Degrees.of(90))));
    for (Case c : cases) {
      assertThatIllegalArgumentException()
          .as(c.says())
          .isThrownBy(c.build())
          .withMessageStartingWith(c.says());
    }
  }

  @Test
  void aStationsIdIsItsNameAsToolsThatTakeOnlyIdsNameIt() {
    assertThat(change().name("Front Left!").build().id()).isEqualTo("front-left-");
    assertThat(change().name("-x").build().id()).isEqualTo("x-x");
  }

  /**
   * A mount the station doesn't say counts as unchecked, 25 mm and 3°; a measured one, 5 mm and 1°;
   * and a tolerance's spread is its bound over the root of 3, as for an error drawn evenly within
   * it.
   */
  @Test
  void aMountsToleranceSpreadsAsAnEvenError() {
    MountTolerance unchecked = MountTolerance.UNCHECKED;
    MountTolerance measured = MountTolerance.MEASURED;
    MountTolerance calibrated = MountTolerance.of(Millimeters.of(2), Degrees.of(0.3));

    assertThat(unchecked.along().in(Millimeters)).isCloseTo(25, within(1e-9));
    assertThat(unchecked.about().in(Degrees)).isCloseTo(3, within(1e-9));
    assertThat(measured.along().in(Millimeters)).isCloseTo(5, within(1e-9));
    assertThat(measured.about().in(Degrees)).isCloseTo(1, within(1e-9));
    assertThat(measured.offsetStdDevMeters()).isCloseTo(0.005 / Math.sqrt(3), within(1e-12));
    assertThat(measured.turnStdDevRadians())
        .isCloseTo(Math.toRadians(1) / Math.sqrt(3), within(1e-12));
    assertThat(calibrated.turnStdDevRadians())
        .isCloseTo(Math.toRadians(0.3) / Math.sqrt(3), within(1e-12));
  }

  /** Each of a station's verbs says its part once: a second would drop the first. */
  static Stream<Arguments> partsSaidTwice() {
    String mountRefusal =
        "CameraStation[left] has its mount's tolerance already: give it once, in one"
            + " .measuredWith(...) or .mountTolerance(...)";
    return Stream.of(
        argumentSet(
            "its place",
            (ThrowingCallable) () -> SAID.at(Inches.of(0), Inches.of(0), Inches.of(20)),
            "CameraStation[left] has its place already: give it once, in one"
                + " .at(forward, left, up)"),
        argumentSet(
            "its facing",
            (ThrowingCallable) () -> SAID.facing(Degrees.of(0), Degrees.of(0)),
            "CameraStation[left] has its facing already: give it once, in one"
                + " .facing(tiltUp, yaw)"),
        argumentSet(
            "its roll",
            (ThrowingCallable) () -> SAID.roll(Degrees.of(0)),
            "CameraStation[left] has its roll already: give it once, in one .roll(angle)"),
        argumentSet(
            "its noise scale",
            (ThrowingCallable) () -> SAID.noiseScale(2),
            "CameraStation[left] has its noise scale already: give it once, in one"
                + " .noiseScale(scale)"),
        argumentSet(
            "its measurement, again", (ThrowingCallable) () -> SAID.measuredWith(4), mountRefusal),
        argumentSet(
            "a tolerance, after its measurement",
            (ThrowingCallable) () -> SAID.mountTolerance(MountTolerance.MEASURED),
            mountRefusal));
  }

  @ParameterizedTest
  @MethodSource("partsSaidTwice")
  void aPartSaidTwiceIsRefused(ThrowingCallable twice, String refusal) {
    assertThatIllegalArgumentException().isThrownBy(twice).withMessage(refusal);
  }

  /** A station not placed or not pointed can't be used: the robot's definition refuses it. */
  static Stream<Arguments> unmountedStations() {
    return Stream.of(
        argumentSet(
            "not placed",
            CameraStation.named("left").facing(Degrees.of(10), Degrees.of(0)),
            "RobotDefinition[test].cameras: CameraStation[left] has no place: give it one,"
                + " .at(forward, left, up)"),
        argumentSet(
            "not pointed",
            CameraStation.named("left").at(Inches.of(0), Inches.of(0), Inches.of(20)),
            "RobotDefinition[test].cameras: CameraStation[left] has no facing: give it one,"
                + " .facing(tiltUp, yaw)"));
  }

  @ParameterizedTest
  @MethodSource("unmountedStations")
  void aStationNotMountedIsRefusedWithTheCallToWrite(CameraStation station, String refusal) {
    CameraConfig camera = station.holds(Camera.photonVision(1, ConfigFixtures.type()));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ConfigFixtures.definition(List.of(camera)))
        .withMessageStartingWith(refusal);
  }

  /** A copy of a valid station named "front", to change one value. */
  private static CameraStationBuilder change() {
    return CameraStationBuilder.builder(station("front", new Translation3d()));
  }
}
