package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.station;
import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.type;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Milliseconds;

import java.util.List;
import java.util.Optional;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

class CameraTest {
  /** A lens at a frame size, as calibrate.py prints one. */
  private static Lens lens(int width, int height) {
    return Lens.builder()
        .width(width)
        .height(height)
        .fx(905.2)
        .fy(905.6)
        .cx(641.3)
        .cy(398.8)
        .distortion(List.of(-0.31, 0.12, 0.0004, -0.0002, -0.02))
        .build();
  }

  private static final String DEVICE = "/dev/v4l/by-id/usb-Maker_Cam_ABC123-video-index0";

  /** A camera on Visor carries its device and lens together, said once, with its own number. */
  @Test
  void aVisorCameraCarriesItsDeviceAndLensTogether() {
    Camera camera =
        Camera.visor(2, type())
            .device(DEVICE)
            .lens(lens(1280, 800))
            .timeOffset(Milliseconds.of(-1.5));

    assertThat(camera.number()).isEqualTo(2);
    assertThat(camera.device()).contains(DEVICE);
    assertThat(camera.lens()).contains(lens(1280, 800));
    assertThat(camera.software())
        .isEqualTo(
            new Camera.Software.Visor(
                Optional.of(DEVICE),
                Optional.of(lens(1280, 800)),
                Optional.of(Milliseconds.of(-1.5))));
    // Visor alone, for the simulator: nothing for a robot to configure.
    Camera simulated = Camera.visor(2, type());
    assertThat(simulated.device()).isEmpty();
    assertThat(simulated.lens()).isEmpty();
    assertThat(simulated.software()).isEqualTo(Camera.Software.VISOR);
    // PhotonVision keeps its own device and calibration.
    Camera photonVision = Camera.photonVision(2, type());
    assertThat(photonVision.software()).isEqualTo(Camera.Software.PHOTONVISION);
    assertThat(photonVision.device()).isEmpty();
    assertThat(photonVision.lens()).isEmpty();
  }

  /**
   * A camera's serial is the last part of its {@code by-id/} name, which is what tells two cameras
   * of one kind apart; a {@code by-path/} name says only the port, so no serial.
   */
  @Test
  void aCamerasSerialComesFromItsByIdName() {
    record Case(String name, Camera camera, Optional<String> serial) {}
    List<Case> cases =
        List.of(
            new Case("by id", Camera.visor(1, type()).device(DEVICE), Optional.of("ABC123")),
            new Case(
                "by id, no index",
                Camera.visor(1, type()).device("/dev/v4l/by-id/usb-Maker_Cam_77"),
                Optional.of("77")),
            new Case(
                "by its port",
                Camera.visor(1, type())
                    .device("/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0"),
                Optional.empty()),
            new Case(
                "by id, no serial in it",
                Camera.visor(1, type()).device("/dev/v4l/by-id/usb-camera-video-index0"),
                Optional.empty()),
            new Case("no device", Camera.visor(1, type()), Optional.empty()),
            new Case("PhotonVision", Camera.photonVision(1, type()), Optional.empty()));
    for (Case c : cases) {
      assertThat(c.camera().serial()).as(c.name()).isEqualTo(c.serial());
    }
  }

  /**
   * What a camera's type leaves out comes from where it's known: the field of view from its lens
   * where it has one, else 80° across the diagonal; the latency from its software; the sight range
   * 7 m. A field of view given beside a lens is the lens's, and doubted more than 2° from it.
   */
  @Test
  void whatTheTypeLeavesOutComesFromTheLensTheSoftwareOrTheDefaults() {
    record Row(
        String name,
        ImageSensor sensor,
        boolean onVisor,
        boolean withLens,
        double fieldOfViewDegrees,
        double latencyMillis,
        double sightRangeMeters,
        boolean doubted) {}
    ImageSensor bare = ImageSensor.of(1280, 800, Hertz.of(30));
    List<Row> rows =
        List.of(
            new Row("nothing said, PhotonVision", bare, false, false, 80, 30, 7, false),
            new Row("nothing said, Visor", bare, true, false, 80, 20, 7, false),
            new Row("a lens", bare, true, true, 79.63, 20, 7, false),
            new Row(
                "a lens and a field of view that agrees",
                bare.fieldOfView(Degrees.of(81), ImageSensor.Across.DIAGONAL),
                true,
                true,
                79.63,
                20,
                7,
                false),
            new Row(
                "a lens and a datasheet's field of view",
                bare.fieldOfView(Degrees.of(90), ImageSensor.Across.DIAGONAL),
                true,
                true,
                79.63,
                20,
                7,
                true),
            new Row(
                "all given",
                bare.fieldOfView(Degrees.of(70), ImageSensor.Across.DIAGONAL)
                    .latency(Milliseconds.of(25))
                    .sightRange(Meters.of(5)),
                false,
                false,
                70,
                25,
                5,
                false));
    for (Row row : rows) {
      CameraType type = CameraType.of("row", row.sensor());
      Camera camera = Camera.photonVision(1, type);
      if (row.onVisor()) {
        camera = Camera.visor(1, type);
      }
      if (row.withLens()) {
        camera = camera.lens(lens(1280, 800));
      }

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
  void valuesThatCantBeAreRefused() {
    record Case(String says, ThrowingCallable build) {}
    List<Case> cases =
        List.of(
            new Case("Camera[0].number", () -> Camera.visor(0, type())),
            new Case(
                "Camera[1].software: has a lens calibrated at 1600x1304, but the camera runs at"
                    + " 1280x800",
                () -> Camera.visor(1, type()).lens(lens(1600, 1304))),
            new Case(
                "Visor.device: is \"video0\", not a device",
                () -> Camera.visor(1, type()).device("video0")),
            new Case(
                "camera 1 is on PhotonVision, which keeps its device on its coprocessor",
                () -> Camera.photonVision(1, type()).device(DEVICE)),
            new Case(
                "camera 1 is on PhotonVision, which keeps its lens on its coprocessor",
                () -> Camera.photonVision(1, type()).lens(lens(1280, 800))),
            new Case(
                "Lens.distortion: has 3 coefficients",
                () ->
                    LensBuilder.builder(lens(1280, 800))
                        .distortion(List.of(0.1, 0.2, 0.3))
                        .build()),
            new Case("Lens.fx", () -> LensBuilder.builder(lens(1280, 800)).fx(0).build()));
    for (Case c : cases) {
      assertThatIllegalArgumentException()
          .as(c.says())
          .isThrownBy(c.build())
          .withMessageStartingWith(c.says());
    }
  }

  /**
   * A station measured with a camera counts as measured while it holds that camera, and as
   * unchecked once it holds another, which the robot says at startup; a tolerance given outright
   * holds whichever camera is in it.
   */
  @Test
  void aSwappedCameraLeavesItsStationUnchecked() {
    CameraStation station = station("front", new Translation3d());
    Camera one = Camera.visor(1, type());
    Camera three = Camera.visor(3, type());
    MountTolerance given = MountTolerance.of(Millimeters.of(2), Degrees.of(0.5));
    record Case(
        String name, CameraConfig placed, MountTolerance tolerance, boolean measuredWithAnother) {}
    List<Case> cases =
        List.of(
            new Case("never measured", station.holds(one), MountTolerance.UNCHECKED, false),
            new Case(
                "measured with the camera it holds",
                station.measuredWith(1).holds(one),
                MountTolerance.MEASURED,
                false),
            new Case(
                "measured with a camera since swapped out",
                station.measuredWith(1).holds(three),
                MountTolerance.UNCHECKED,
                true),
            new Case(
                "a tolerance given outright",
                station.mountTolerance(given).holds(three),
                given,
                false));
    for (Case c : cases) {
      assertThat(c.placed().mountTolerance()).as(c.name()).isEqualTo(c.tolerance());
      assertThat(c.placed().measuredWithAnotherCamera())
          .as(c.name())
          .isEqualTo(c.measuredWithAnother());
    }
    RobotDefinition swapped =
        ConfigFixtures.definition(List.of(station.measuredWith(1).holds(three)));
    assertThat(MechanismFixtures.findings(swapped))
        .anySatisfy(finding ->
            assertThat(finding.toString())
                .contains("its station was measured with camera 1, and holds camera 3 now"));
  }

  /**
   * A camera is in one station: the same camera in two is refused, and so are two different cameras
   * numbered alike, since the pit finds a camera by its label.
   */
  @Test
  void aCameraIsInOneStationByItsOwnNumber() {
    Camera one = Camera.visor(1, type()).device(DEVICE);
    Camera otherOne = Camera.visor(1, type());
    CameraConfig left = station("left", new Translation3d(0, 0.1, 0.3)).holds(one);
    record Case(String name, CameraConfig right, String refusal) {}
    List<Case> cases =
        List.of(
            new Case(
                "the same camera twice",
                station("right", new Translation3d(0, -0.1, 0.3)).holds(one),
                "camera 1 is in both left and right: a camera is in one station"),
            new Case(
                "two cameras numbered alike",
                station("right", new Translation3d(0, -0.1, 0.3)).holds(otherOne),
                "two different cameras, in left and right, are both numbered 1"),
            new Case(
                "one device under two numbers",
                station("right", new Translation3d(0, -0.1, 0.3))
                    .holds(Camera.visor(2, type()).device(DEVICE)),
                "cameras 1 (in left) and 2 (in right) have one device, " + DEVICE));
    for (Case c : cases) {
      assertThatIllegalArgumentException()
          .as(c.name())
          .isThrownBy(() -> ConfigFixtures.definition(List.of(left, c.right())))
          .withMessageContaining(c.refusal());
    }
  }

  /**
   * A venue keeps settings by a camera type's name, so two types of one name must be one kind of
   * camera set up one way: they may differ only in their live view, which no venue keeps.
   */
  @Test
  void twoKindsOfCameraCantShareAName() {
    CameraConfig left =
        station("left", new Translation3d(0, 0.1, 0.3)).holds(Camera.visor(1, type()));
    String refused = "two different camera types are both named test camera";
    record Case(String name, CameraType right, String refusal) {}
    List<Case> cases =
        List.of(
            new Case(
                "other images",
                CameraType.of("test camera", ImageSensor.of(1600, 1304, Hertz.of(60))),
                refused),
            new Case("another starting gain", type().gain(30), refused),
            new Case("another exposure", type().exposure(Milliseconds.of(2)), refused),
            new Case(
                "another control",
                type().controls(List.of(CameraType.Control.of("brightness", 1))),
                refused),
            new Case("the same", type(), ""),
            new Case(
                "another live view alone",
                type().stream(org.wpilib.units.Units.Percent.of(25), Hertz.of(5), 60),
                ""));
    for (Case c : cases) {
      List<CameraConfig> cameras =
          List.of(
              left,
              station("right", new Translation3d(0, -0.1, 0.3)).holds(Camera.visor(2, c.right())));
      if (c.refusal().isEmpty()) {
        assertThat(ConfigFixtures.definition(cameras).cameras()).as(c.name()).hasSize(2);
        continue;
      }
      assertThatIllegalArgumentException()
          .as(c.name())
          .isThrownBy(() -> ConfigFixtures.definition(cameras))
          .withMessageContaining(c.refusal());
    }
  }
}
