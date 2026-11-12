package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.camera;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.sensor;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Microseconds;
import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Value;

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
import org.wpilib.units.measure.Angle;

class CameraConfigTest {
  @Test
  void keepsItsDescription() {
    CameraConfig camera = camera("front", new Translation3d(0.3, 0.2, 0.25));

    assertThat(camera.name()).isEqualTo("front");
    assertThat(camera.forward().orElseThrow().in(Meters)).isCloseTo(0.3, within(1e-9));
    assertThat(camera.left().orElseThrow().in(Meters)).isCloseTo(0.2, within(1e-9));
    assertThat(camera.up().orElseThrow().in(Meters)).isCloseTo(0.25, within(1e-9));
    assertThat(camera.rollOrDefault().in(Degrees)).isZero();
    assertThat(camera.tiltUp().orElseThrow().in(Degrees)).isZero();
    assertThat(camera.yaw().orElseThrow().in(Degrees)).isZero();
    assertThat(camera.sensor()).isEqualTo(sensor());

    CameraConfig.ImageSensor sensor = camera.sensor();
    assertThat(sensor.widthPixels()).isEqualTo(1280);
    assertThat(sensor.heightPixels()).isEqualTo(800);
    assertThat(camera.diagonalFieldOfView().in(Degrees)).isCloseTo(80, within(1e-9));
    assertThat(sensor.frameRate().in(Hertz)).isCloseTo(30, within(1e-9));
    assertThat(camera.latencyOrDefault().in(Milliseconds)).isCloseTo(30, within(1e-9));
    assertThat(camera.sightRangeOrDefault().in(Meters)).isCloseTo(7, within(1e-9));
  }

  /**
   * The mount's signs, each pinned by a point worked out by hand, not by composing rotations as the
   * mount does: a point a camera sees along its own axes, and where that point is on the robot.
   * Tilted up, the camera looks up; yawed left, it looks left; rolled, its left tips up; {@code
   * left} moves the lens left; and with tilt and yaw together the tilt stays up, whichever way it's
   * yawed.
   */
  @Test
  void aCamerasMountPutsWhatItSeesWhereItsSignsSay() {
    double cos20 = 0.9396926207859084;
    double sin20 = 0.3420201433256687;
    record Case(String name, CameraConfig camera, Translation3d seen, Translation3d onTheRobot) {}
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
      Transform3d mount = c.camera().robotToCamera();
      Translation3d onTheRobot =
          c.seen().rotateBy(mount.getRotation()).plus(mount.getTranslation());

      assertThat(onTheRobot.getX()).as(c.name()).isCloseTo(c.onTheRobot().getX(), within(1e-12));
      assertThat(onTheRobot.getY()).as(c.name()).isCloseTo(c.onTheRobot().getY(), within(1e-12));
      assertThat(onTheRobot.getZ()).as(c.name()).isCloseTo(c.onTheRobot().getZ(), within(1e-12));
    }
  }

  @Test
  void aCameraTiltedUpHasANegativePitchInWpilibTerms() {
    CameraConfig camera =
        CameraConfigBuilder.builder(camera("front-left", new Translation3d()))
            .forward(Inches.of(11.5))
            .left(Inches.of(11.5))
            .up(Inches.of(9))
            .tiltUp(Degrees.of(20))
            .yaw(Degrees.of(45))
            .build();

    assertThat(camera.position())
        .isEqualTo(new Translation3d(Inches.of(11.5), Inches.of(11.5), Inches.of(9)));
    assertThat(camera.robotToCamera())
        .isEqualTo(
            new Transform3d(
                camera.position(), new Rotation3d(Degrees.of(0), Degrees.of(-20), Degrees.of(45))));
  }

  @Test
  void theMountMustBeNumbers() {
    var nan = Double.NaN;

    assertRejected("CameraConfig[ ].name", () -> change().name(" ").build());
    assertRejected("CameraConfig[front].forward", () -> change().forward(Inches.of(nan)).build());
    assertRejected("CameraConfig[front].left", () -> change().left(Inches.of(nan)).build());
    assertRejected("CameraConfig[front].up", () -> change().up(Inches.of(nan)).build());
    assertRejected("CameraConfig[front].roll", () -> change().roll(Degrees.of(nan)).build());
    assertRejected("CameraConfig[front].tiltUp", () -> change().tiltUp(Degrees.of(nan)).build());
    assertRejected("CameraConfig[front].yaw", () -> change().yaw(Degrees.of(nan)).build());
  }

  @Test
  void aCamerasIdIsItsNameAsToolsThatTakeOnlyIdsNameIt() {
    assertThat(change().name("Front Left!").build().id()).isEqualTo("front-left-");
    assertThat(change().name("-x").build().id()).isEqualTo("x-x");
  }

  @Test
  void aSensorWithNoLatencyIsAllowed() {
    assertThatNoException().isThrownBy(() -> sensor().latency(Milliseconds.of(0)));
  }

  /**
   * A Limelight 4 sees 82° across its 1280 x 800 image, as its datasheet and teams give it: 91.4°
   * across the diagonal, through a lens with square pixels. A diagonal one is as given.
   */
  @Test
  void aHorizontalFieldOfViewIsWiderAcrossTheDiagonal() {
    CameraConfig.ImageSensor horizontal =
        changeSensor()
            .fieldOfView(Degrees.of(82))
            .fieldOfViewAcross(CameraConfig.Across.HORIZONTAL)
            .build();
    double diagonal = Math.hypot(1280, 800);
    double expected = 2 * Math.atan(Math.tan(Math.toRadians(41)) * diagonal / 1280);

    Angle across = horizontal.diagonalFieldOfView().orElseThrow();
    assertThat(across.in(Radians)).isCloseTo(expected, within(1e-12));
    assertThat(across.in(Degrees)).isCloseTo(91.43, within(0.01));
    // Back across its width, through the diagonal, it's what was given.
    double half = Math.atan(Math.tan(across.in(Radians) / 2) * 1280 / diagonal);
    assertThat(Math.toDegrees(2 * half)).isCloseTo(82, within(1e-9));
    assertThat(sensor().diagonalFieldOfView()).isEqualTo(sensor().fieldOfView());
  }

  @Test
  void aCameraRunsPhotonVisionUnlessItSaysVisor() {
    assertThat(camera("front", new Translation3d()).software())
        .isEqualTo(CameraConfig.Software.PHOTONVISION);
    assertThat(change().software(CameraConfig.Software.VISOR).build().software())
        .isEqualTo(CameraConfig.Software.VISOR);
  }

  /** A lens at a frame size, as calibrate.py prints one. */
  private static CameraConfig.Software.Lens lens(int width, int height) {
    return CameraConfig.Software.Lens.builder()
        .width(width)
        .height(height)
        .fx(905.2)
        .fy(905.6)
        .cx(641.3)
        .cy(398.8)
        .distortion(List.of(-0.31, 0.12, 0.0004, -0.0002, -0.02))
        .build();
  }

  @Test
  void aVisorCameraCarriesWhatItsDeployNeeds() {
    CameraConfig.Software.Visor visor =
        CameraConfig.Software.visor()
            .device("/dev/v4l/by-id/usb-cam-video-index0")
            .controls(
                List.of(
                    CameraConfig.Software.Control.of("power_line_frequency", 2),
                    CameraConfig.Software.Control.of("brightness", -10)))
            .timeOffset(Milliseconds.of(-1.5))
            .lens(lens(1280, 800));
    CameraConfig camera = change().software(visor).build();

    assertThat(camera.software()).isEqualTo(visor);
    assertThat(visor.device()).contains("/dev/v4l/by-id/usb-cam-video-index0");
    assertThat(visor.controls())
        .extracting(CameraConfig.Software.Control::name)
        .containsExactly("power_line_frequency", "brightness");
    // Visor alone, for the simulator: nothing for a robot to configure.
    assertThat(CameraConfig.Software.VISOR.device()).isEmpty();
    assertThat(CameraConfig.Software.VISOR.controls()).isEmpty();
    assertThat(CameraConfig.Software.VISOR.lens()).isEmpty();
  }

  /**
   * The exposure and gain's defaults live here, not in Visor: written nothing, a camera runs a
   * fixed 1 ms with its automatic exposure off, at its own gain. Any camera can give its own.
   */
  @Test
  void aVisorCameraRunsAFixedMillisecondAtItsOwnGainUnlessItSays() {
    record Case(
        String name,
        CameraConfig.Software.Visor visor,
        CameraConfig.Software.Exposure exposure,
        CameraConfig.Software.Gain gain) {}
    List<Case> cases =
        List.of(
            new Case(
                "written nothing",
                CameraConfig.Software.visor(),
                new CameraConfig.Software.Exposure.Fixed(Milliseconds.of(1)),
                CameraConfig.Software.Gain.CAMERAS_OWN),
            new Case(
                "for the simulator",
                CameraConfig.Software.VISOR,
                new CameraConfig.Software.Exposure.Fixed(Milliseconds.of(1)),
                CameraConfig.Software.Gain.CAMERAS_OWN),
            new Case(
                "its own exposure and gain",
                CameraConfig.Software.visor().exposure(Milliseconds.of(2.5)).gain(40),
                new CameraConfig.Software.Exposure.Fixed(Milliseconds.of(2.5)),
                new CameraConfig.Software.Gain.Level(40)),
            new Case(
                "the camera's own automatic exposure",
                CameraConfig.Software.visor().camerasOwnExposure(),
                CameraConfig.Software.Exposure.CAMERAS_OWN,
                CameraConfig.Software.Gain.CAMERAS_OWN));
    for (Case c : cases) {
      assertThat(c.visor().exposure()).as(c.name()).isEqualTo(c.exposure());
      assertThat(c.visor().gain()).as(c.name()).isEqualTo(c.gain());
    }
    assertThat(CameraConfig.Software.visor().exposure())
        .isEqualTo(new CameraConfig.Software.Exposure.Fixed(Milliseconds.of(1)));
    assertThat(CameraConfig.Software.visor().gain())
        .isEqualTo(CameraConfig.Software.Gain.CAMERAS_OWN);
  }

  /**
   * A camera's live view is Visor's own unless it says, so the robot writes none; one it gives is
   * kept as given, faster than 30 a second too, which Visor holds to the camera's own rate.
   */
  @Test
  void aVisorCamerasLiveViewIsVisorsOwnUnlessItSays() {
    record Case(
        String name, CameraConfig.Software.Visor visor, double scale, double hertz, int quality) {}
    assertThat(CameraConfig.Software.visor().stream()).as("written nothing").isEmpty();
    assertThat(CameraConfig.Software.VISOR.stream()).as("for the simulator").isEmpty();
    List<Case> cases =
        List.of(
            new Case(
                "its own: smaller, slower, sharper",
                CameraConfig.Software.visor().stream(Percent.of(25), Hertz.of(5), 80),
                0.25,
                5,
                80),
            new Case(
                "its own: whole, faster than 30 a second",
                CameraConfig.Software.visor().stream(Percent.of(100), Hertz.of(60), 60),
                1,
                60,
                60));
    for (Case c : cases) {
      CameraConfig.Software.Stream stream = c.visor().stream().orElseThrow();
      assertThat(stream.scale().in(Value)).as(c.name()).isEqualTo(c.scale());
      assertThat(stream.frameRate().in(Hertz)).as(c.name()).isEqualTo(c.hertz());
      assertThat(stream.quality()).as(c.name()).isEqualTo(c.quality());
    }
  }

  private static CameraConfig.Software.Visor stream(double percent, double hertz, int quality) {
    return CameraConfig.Software.visor().stream(Percent.of(percent), Hertz.of(hertz), quality);
  }

  @Test
  void visorSettingsThatCantBeAreRefused() {
    record Case(String says, ThrowingCallable build) {}
    List<Case> cases =
        List.of(
            new Case(
                "CameraConfig[front].software: has a lens calibrated at 1600x1304, but the camera runs at 1280x800",
                () ->
                    change()
                        .software(CameraConfig.Software.visor().lens(lens(1600, 1304)))
                        .build()),
            new Case(
                "Visor.device: is \"video0\", not a device",
                () -> CameraConfig.Software.visor().device("video0")),
            new Case(
                "Visor.controls: sets brightness twice",
                () ->
                    CameraConfig.Software.visor()
                        .controls(
                            List.of(
                                CameraConfig.Software.Control.of("brightness", 1),
                                CameraConfig.Software.Control.of("brightness", 2)))),
            new Case(
                "Control[Exposure].name", () -> CameraConfig.Software.Control.of("Exposure", 10)),
            new Case(
                "Fixed.time: 0 ms must be from 100 us to 1 s",
                () -> CameraConfig.Software.visor().exposure(Milliseconds.of(0))),
            new Case(
                "Fixed.time: 50 us must be from 100 us to 1 s",
                () -> CameraConfig.Software.visor().exposure(Microseconds.of(50))),
            new Case(
                "Fixed.time: 2 s must be from 100 us to 1 s",
                () -> CameraConfig.Software.visor().exposure(Seconds.of(2))),
            new Case("Stream.scale: 0 % must be from 5 % to 100 %", () -> stream(0, 10, 60)),
            new Case("Stream.scale: 150 % must be from 5 % to 100 %", () -> stream(150, 10, 60)),
            new Case("Stream.frameRate: 0.5 hz must be at least 1 hz", () -> stream(50, 0.5, 60)),
            new Case(
                "Stream.frameRate: NaN hz must be a finite number",
                () -> stream(50, Double.NaN, 60)),
            new Case("Stream.quality: 0 must be from 1 to 100", () -> stream(50, 10, 0)),
            new Case("Stream.quality: 101 must be from 1 to 100", () -> stream(50, 10, 101)),
            new Case(
                "Level.value: is 65536: a gain control takes 0 to 65535",
                () -> CameraConfig.Software.visor().gain(65_536)),
            new Case(
                "Level.value: is -1: a gain control takes 0 to 65535",
                () -> CameraConfig.Software.visor().gain(-1)),
            new Case(
                "Lens.distortion: has 3 coefficients",
                () ->
                    CameraConfigSoftwareLensBuilder.builder(lens(1280, 800))
                        .distortion(List.of(0.1, 0.2, 0.3))
                        .build()),
            new Case(
                "Lens.fx",
                () -> CameraConfigSoftwareLensBuilder.builder(lens(1280, 800)).fx(0).build()));
    for (Case c : cases) {
      assertRejected(c.says(), c.build());
    }
  }

  /**
   * A camera that doesn't say how sure its mount is counts as unchecked, 25 mm and 3°, not as one
   * the mount procedure has checked, 5 mm and 1°: the localizer starts as unsure of it as that.
   */
  @Test
  void aCameraLeftWithoutAToleranceIsUnchecked() {
    CameraConfig.MountTolerance unchecked = camera("front", new Translation3d()).mountTolerance();
    CameraConfig.MountTolerance measured =
        change().mountTolerance(CameraConfig.MountTolerance.MEASURED).build().mountTolerance();
    CameraConfig.MountTolerance calibrated =
        change()
            .mountTolerance(CameraConfig.MountTolerance.of(Millimeters.of(2), Degrees.of(0.3)))
            .build()
            .mountTolerance();

    assertThat(unchecked).isEqualTo(CameraConfig.MountTolerance.UNCHECKED);
    assertThat(unchecked.along().in(Millimeters)).isCloseTo(25, within(1e-9));
    assertThat(unchecked.about().in(Degrees)).isCloseTo(3, within(1e-9));
    assertThat(measured.along().in(Millimeters)).isCloseTo(5, within(1e-9));
    assertThat(measured.about().in(Degrees)).isCloseTo(1, within(1e-9));
    // An error drawn evenly within a bound b has a standard deviation of b over the root of 3.
    assertThat(measured.offsetStdDevMeters()).isCloseTo(0.005 / Math.sqrt(3), within(1e-12));
    assertThat(measured.turnStdDevRadians())
        .isCloseTo(Math.toRadians(1) / Math.sqrt(3), within(1e-12));
    assertThat(calibrated.turnStdDevRadians())
        .isCloseTo(Math.toRadians(0.3) / Math.sqrt(3), within(1e-12));
  }

  /**
   * What a camera's sensor leaves out comes from where it's known: the field of view from its lens
   * where it has one, else 80° across the diagonal; the latency from its software; the sight range
   * 7 m. A field of view given beside a lens is the lens's, and doubted more than 2° from it.
   */
  @Test
  void whatTheSensorLeavesOutComesFromTheLensTheSoftwareOrTheDefaults() {
    record Row(
        String name,
        CameraConfig.ImageSensor sensor,
        CameraConfig.Software software,
        double fieldOfViewDegrees,
        double latencyMillis,
        double sightRangeMeters,
        boolean doubted) {}
    CameraConfig.ImageSensor bare = CameraConfig.ImageSensor.of(1280, 800, Hertz.of(30));
    CameraConfig.Software.Visor withLens = CameraConfig.Software.visor().lens(lens(1280, 800));
    List<Row> rows =
        List.of(
            new Row(
                "nothing said, PhotonVision",
                bare,
                CameraConfig.Software.PHOTONVISION,
                80,
                30,
                7,
                false),
            new Row("nothing said, Visor", bare, CameraConfig.Software.VISOR, 80, 20, 7, false),
            new Row("a lens", bare, withLens, 79.63, 20, 7, false),
            new Row(
                "a lens and a field of view that agrees",
                bare.fieldOfView(Degrees.of(81), CameraConfig.Across.DIAGONAL),
                withLens,
                79.63,
                20,
                7,
                false),
            new Row(
                "a lens and a datasheet's field of view",
                bare.fieldOfView(Degrees.of(90), CameraConfig.Across.DIAGONAL),
                withLens,
                79.63,
                20,
                7,
                true),
            new Row(
                "all given",
                bare.fieldOfView(Degrees.of(70), CameraConfig.Across.DIAGONAL)
                    .latency(Milliseconds.of(25))
                    .sightRange(Meters.of(5)),
                CameraConfig.Software.PHOTONVISION,
                70,
                25,
                5,
                false));
    for (Row row : rows) {
      CameraConfig camera = change().sensor(row.sensor()).software(row.software()).build();

      assertThat(camera.diagonalFieldOfView().in(Degrees))
          .as(row.name())
          .isCloseTo(row.fieldOfViewDegrees(), within(0.01));
      assertThat(camera.latencyOrDefault().in(Milliseconds))
          .as(row.name())
          .isCloseTo(row.latencyMillis(), within(1e-9));
      assertThat(camera.sightRangeOrDefault().in(Meters))
          .as(row.name())
          .isCloseTo(row.sightRangeMeters(), within(1e-9));
      assertThat(camera.whyFieldOfViewDisagreesWithLens().isPresent())
          .as(row.name())
          .isEqualTo(row.doubted());
    }
  }

  @Test
  void rejectsAnImpossibleMountTolerance() {
    assertRejected(
        "MountTolerance.along",
        () -> CameraConfig.MountTolerance.of(Millimeters.of(-1), Degrees.of(1)));
    assertRejected(
        "MountTolerance.about",
        () -> CameraConfig.MountTolerance.of(Millimeters.of(1), Degrees.of(-1)));
    assertRejected(
        "MountTolerance.about: 90 ° must be below 90 °",
        () -> CameraConfig.MountTolerance.of(Millimeters.of(1), Degrees.of(90)));
  }

  @Test
  void rejectsAnImpossibleSensor() {
    assertRejected("ImageSensor.widthPixels", () -> changeSensor().widthPixels(0).build());
    assertRejected("ImageSensor.heightPixels", () -> changeSensor().heightPixels(-800).build());
    assertRejected(
        "ImageSensor.fieldOfView",
        () -> sensor().fieldOfView(Degrees.of(0), CameraConfig.Across.DIAGONAL));
    assertRejected(
        "ImageSensor.fieldOfView: 180 ° must be below 180 °",
        () -> sensor().fieldOfView(Degrees.of(180), CameraConfig.Across.DIAGONAL));
    assertRejected("ImageSensor.frameRate", () -> changeSensor().frameRate(Hertz.of(0)).build());
    assertRejected("ImageSensor.latency", () -> sensor().latency(Milliseconds.of(-1)));
    assertRejected("ImageSensor.sightRange", () -> sensor().sightRange(Meters.of(0)));
  }

  @Test
  void aSettingThatIsOnOrOffIsOneOrZero() {
    assertThat(CameraConfig.Software.Control.on("white_balance_automatic"))
        .isEqualTo(CameraConfig.Software.Control.of("white_balance_automatic", 1));
    assertThat(CameraConfig.Software.Control.off("white_balance_automatic"))
        .isEqualTo(CameraConfig.Software.Control.of("white_balance_automatic", 0));
  }

  /** A camera said in full, from its name and sensor, each verb once. */
  private static final CameraConfig SAID =
      CameraConfig.of("left", sensor())
          .at(Inches.of(-10.5), Inches.of(12), Inches.of(21))
          .facing(Degrees.of(10), Degrees.of(180))
          .roll(Degrees.of(-2))
          .noiseScale(1.5)
          .measured()
          .software(CameraConfig.Software.VISOR);

  @Test
  void aCameraIsSaidByItsVerbs() {
    assertThat(SAID.position())
        .isEqualTo(new Translation3d(Inches.of(-10.5), Inches.of(12), Inches.of(21)));
    assertThat(SAID.tiltUp().orElseThrow().in(Degrees)).isCloseTo(10, within(1e-9));
    assertThat(SAID.yaw().orElseThrow().in(Degrees)).isCloseTo(180, within(1e-9));
    assertThat(SAID.rollOrDefault().in(Degrees)).isCloseTo(-2, within(1e-9));
    assertThat(SAID.noiseScaleOrDefault()).isEqualTo(1.5);
    assertThat(SAID.mountTolerance()).isEqualTo(CameraConfig.MountTolerance.MEASURED);
    assertThat(SAID.software()).isEqualTo(CameraConfig.Software.VISOR);
    assertThat(SAID.withSoftware(CameraConfig.Software.PHOTONVISION).software())
        .isEqualTo(CameraConfig.Software.PHOTONVISION);
  }

  /** Each of a camera's verbs says its part once: a second would drop the first. */
  static Stream<Arguments> partsSaidTwice() {
    return Stream.of(
        argumentSet(
            "its place",
            (ThrowingCallable) () -> SAID.at(Inches.of(0), Inches.of(0), Inches.of(20)),
            "CameraConfig[left] has its place already: give it once, in one"
                + " .at(forward, left, up)"),
        argumentSet(
            "its facing",
            (ThrowingCallable) () -> SAID.facing(Degrees.of(0), Degrees.of(0)),
            "CameraConfig[left] has its facing already: give it once, in one .facing(tiltUp, yaw)"),
        argumentSet(
            "its roll",
            (ThrowingCallable) () -> SAID.roll(Degrees.of(0)),
            "CameraConfig[left] has its roll already: give it once, in one .roll(angle)"),
        argumentSet(
            "its noise scale",
            (ThrowingCallable) () -> SAID.noiseScale(2),
            "CameraConfig[left] has its noise scale already: give it once, in one"
                + " .noiseScale(scale)"),
        argumentSet(
            "its mount's tolerance",
            (ThrowingCallable) SAID::measured,
            "CameraConfig[left] has its mount's tolerance already: give it once, in one"
                + " .measured() or .mountTolerance(...)"),
        argumentSet(
            "its software",
            (ThrowingCallable) () -> SAID.software(CameraConfig.Software.VISOR),
            "CameraConfig[left] has its software already: give it once, in one .software(...),"
                + " or replace it with .withSoftware(...)"));
  }

  @ParameterizedTest
  @MethodSource("partsSaidTwice")
  void aPartSaidTwiceIsRefused(ThrowingCallable twice, String refusal) {
    assertThatIllegalArgumentException().isThrownBy(twice).withMessage(refusal);
  }

  /** A camera not placed or not pointed can't be used: the robot's definition refuses it. */
  static Stream<Arguments> unmountedCameras() {
    return Stream.of(
        argumentSet(
            "not placed",
            CameraConfig.of("left", sensor()).facing(Degrees.of(10), Degrees.of(0)),
            "RobotDefinition[test].cameras: CameraConfig[left] has no place: give it one,"
                + " .at(forward, left, up)"),
        argumentSet(
            "not pointed",
            CameraConfig.of("left", sensor()).at(Inches.of(0), Inches.of(0), Inches.of(20)),
            "RobotDefinition[test].cameras: CameraConfig[left] has no facing: give it one,"
                + " .facing(tiltUp, yaw)"));
  }

  @ParameterizedTest
  @MethodSource("unmountedCameras")
  void aCameraNotMountedIsRefusedWithTheCallToWrite(CameraConfig camera, String refusal) {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ConfigFixtures.definition(List.of(camera)))
        .withMessageStartingWith(refusal);
  }

  /** A copy of a valid camera named "front", to change one value. */
  private static CameraConfigBuilder change() {
    return CameraConfigBuilder.builder(camera("front", new Translation3d()));
  }

  private static CameraConfigImageSensorBuilder changeSensor() {
    return CameraConfigImageSensorBuilder.builder(sensor());
  }

  private static void assertRejected(String message, ThrowingCallable build) {
    assertThatIllegalArgumentException().isThrownBy(build).withMessageStartingWith(message);
  }
}
