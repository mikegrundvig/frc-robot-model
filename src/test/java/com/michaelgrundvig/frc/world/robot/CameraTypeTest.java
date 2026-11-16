package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.ConfigFixtures.sensor;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatNoException;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Microseconds;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Value;

import java.util.List;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Angle;

class CameraTypeTest {
  /** A kind of camera that says nothing past its name and images. */
  private static final CameraType BARE = CameraType.of("bare", sensor());

  /**
   * The exposure and gain a camera starts at live here, not in Visor: written nothing, a fixed 1 ms
   * with its automatic exposure off, at its own gain. Any kind can give its own, and one number for
   * the kind's gain is a single verb.
   */
  @Test
  void aTypeRunsAFixedMillisecondAtTheCamerasOwnGainUnlessItSays() {
    assertThat(BARE.gainRange()).as("UVC's whole range unless given").isEmpty();
    CameraType.GainRange range = BARE.gain(160).gainRange(16, 255).gainRange().orElseThrow();
    assertThat(range.holds(16)).isTrue();
    assertThat(range.holds(256)).isFalse();
    assertThat(range.nearest(300)).isEqualTo(255);
    assertThat(range.nearest(4)).isEqualTo(16);
    assertThat(BARE.gainRange(100, 100).gainRange()).as("one value is a range").isPresent();
    record Case(String name, CameraType type, CameraType.Exposure exposure, CameraType.Gain gain) {}
    List<Case> cases =
        List.of(
            new Case(
                "written nothing",
                BARE,
                new CameraType.Exposure.Fixed(Milliseconds.of(1)),
                CameraType.Gain.CAMERAS_OWN),
            new Case(
                "its own exposure and gain",
                BARE.exposure(Milliseconds.of(2.5)).gain(40),
                new CameraType.Exposure.Fixed(Milliseconds.of(2.5)),
                new CameraType.Gain.Level(40)),
            new Case(
                "the camera's own automatic exposure",
                BARE.camerasOwnExposure(),
                CameraType.Exposure.CAMERAS_OWN,
                CameraType.Gain.CAMERAS_OWN));
    for (Case c : cases) {
      assertThat(c.type().exposure()).as(c.name()).isEqualTo(c.exposure());
      assertThat(c.type().gain()).as(c.name()).isEqualTo(c.gain());
    }
    assertThat(CameraType.DEFAULT_EXPOSURE)
        .isEqualTo(new CameraType.Exposure.Fixed(Milliseconds.of(1)));
  }

  /**
   * A camera's live view is Visor's own unless its type says, so the robot writes none; one it
   * gives is kept as given, faster than 30 a second too, which Visor holds to the camera's own
   * rate. Its other settings are kept in their order.
   */
  @Test
  void aTypesLiveViewAndControlsAreKeptAsGiven() {
    assertThat(BARE.stream()).as("written nothing").isEmpty();
    assertThat(BARE.controls()).as("written nothing").isEmpty();
    record Case(String name, CameraType type, double scale, double hertz, int quality) {}
    List<Case> cases =
        List.of(
            new Case(
                "smaller, slower, sharper",
                BARE.stream(Percent.of(25), Hertz.of(5), 80),
                0.25,
                5,
                80),
            new Case(
                "whole, faster than 30 a second",
                BARE.stream(Percent.of(100), Hertz.of(60), 60),
                1,
                60,
                60));
    for (Case c : cases) {
      CameraType.Stream stream = c.type().stream().orElseThrow();
      assertThat(stream.scale().in(Value)).as(c.name()).isEqualTo(c.scale());
      assertThat(stream.frameRate().in(Hertz)).as(c.name()).isEqualTo(c.hertz());
      assertThat(stream.quality()).as(c.name()).isEqualTo(c.quality());
    }
    CameraType controlled =
        BARE.controls(
            List.of(
                CameraType.Control.of("power_line_frequency", 2),
                CameraType.Control.of("brightness", -10)));
    assertThat(controlled.controls())
        .extracting(CameraType.Control::name)
        .containsExactly("power_line_frequency", "brightness");
    assertThat(CameraType.Control.on("white_balance_automatic"))
        .isEqualTo(CameraType.Control.of("white_balance_automatic", 1));
    assertThat(CameraType.Control.off("white_balance_automatic"))
        .isEqualTo(CameraType.Control.of("white_balance_automatic", 0));
  }

  /**
   * A Limelight 4 sees 82° across its 1280 x 800 image, as its datasheet and teams give it: 91.4°
   * across the diagonal, through a lens with square pixels. A diagonal one is as given.
   */
  @Test
  void aHorizontalFieldOfViewIsWiderAcrossTheDiagonal() {
    ImageSensor horizontal =
        ImageSensor.of(1280, 800, Hertz.of(30))
            .fieldOfView(Degrees.of(82), ImageSensor.Across.HORIZONTAL);
    double diagonal = Math.hypot(1280, 800);
    double expected = 2 * Math.atan(Math.tan(Math.toRadians(41)) * diagonal / 1280);

    Angle across = horizontal.diagonalFieldOfView().orElseThrow();
    assertThat(across.in(Radians)).isCloseTo(expected, within(1e-12));
    assertThat(across.in(Degrees)).isCloseTo(91.43, within(0.01));
    // Back across its width, through the diagonal, it's what was given.
    double half = Math.atan(Math.tan(across.in(Radians) / 2) * 1280 / diagonal);
    assertThat(Math.toDegrees(2 * half)).isCloseTo(82, within(1e-9));
    assertThat(sensor().diagonalFieldOfView()).isEqualTo(sensor().fieldOfView());
    assertThatNoException().isThrownBy(() -> sensor().latency(Milliseconds.of(0)));
  }

  private static CameraType stream(double percent, double hertz, int quality) {
    return BARE.stream(Percent.of(percent), Hertz.of(hertz), quality);
  }

  @Test
  void valuesThatCantBeAreRefused() {
    record Case(String says, ThrowingCallable build) {}
    List<Case> cases =
        List.of(
            new Case("CameraType[ ].name", () -> CameraType.of(" ", sensor())),
            new Case(
                "CameraType[bare].controls: sets brightness twice",
                () ->
                    BARE.controls(
                        List.of(
                            CameraType.Control.of("brightness", 1),
                            CameraType.Control.of("brightness", 2)))),
            new Case("Control[Exposure].name", () -> CameraType.Control.of("Exposure", 10)),
            new Case(
                "Fixed.time: 0 ms must be from 100 us to 1 s",
                () -> BARE.exposure(Milliseconds.of(0))),
            new Case(
                "Fixed.time: 50 us must be from 100 us to 1 s",
                () -> BARE.exposure(Microseconds.of(50))),
            new Case(
                "Fixed.time: 2 s must be from 100 us to 1 s", () -> BARE.exposure(Seconds.of(2))),
            new Case("Stream.scale: 0 % must be from 5 % to 100 %", () -> stream(0, 10, 60)),
            new Case("Stream.scale: 150 % must be from 5 % to 100 %", () -> stream(150, 10, 60)),
            new Case("Stream.frameRate: 0.5 hz must be at least 1 hz", () -> stream(50, 0.5, 60)),
            new Case(
                "Stream.frameRate: NaN hz must be a finite number",
                () -> stream(50, Double.NaN, 60)),
            new Case("Stream.quality: 0 must be from 1 to 100", () -> stream(50, 10, 0)),
            new Case("Stream.quality: 101 must be from 1 to 100", () -> stream(50, 10, 101)),
            new Case(
                "Level.value: is 65536: a gain control takes 0 to 65535", () -> BARE.gain(65_536)),
            new Case("Level.value: is -1: a gain control takes 0 to 65535", () -> BARE.gain(-1)),
            new Case(
                "CameraType[bare].gain: is 300, outside the 16 to 255 the kind's gain control"
                    + " takes: give a gain in that range",
                () -> BARE.gain(300).gainRange(16, 255)),
            new Case(
                "CameraType[bare].gain: is 8, outside the 16 to 255",
                () -> BARE.gainRange(16, 255).gain(8)),
            new Case(
                "GainRange.highest: is 16, below lowest (255): give the least first,"
                    + " .gainRange(16, 255)",
                () -> BARE.gainRange(255, 16)),
            new Case(
                "GainRange.lowest: is -1: a gain control takes 0 to 65535",
                () -> BARE.gainRange(-1, 255)),
            new Case(
                "GainRange.highest: is 70000: a gain control takes 0 to 65535",
                () -> BARE.gainRange(16, 70_000)),
            new Case(
                "ImageSensor.widthPixels",
                () -> ImageSensorBuilder.builder(sensor()).widthPixels(0).build()),
            new Case(
                "ImageSensor.heightPixels",
                () -> ImageSensorBuilder.builder(sensor()).heightPixels(-800).build()),
            new Case(
                "ImageSensor.fieldOfView",
                () -> sensor().fieldOfView(Degrees.of(0), ImageSensor.Across.DIAGONAL)),
            new Case(
                "ImageSensor.fieldOfView: 180 ° must be below 180 °",
                () -> sensor().fieldOfView(Degrees.of(180), ImageSensor.Across.DIAGONAL)),
            new Case(
                "ImageSensor.frameRate",
                () -> ImageSensorBuilder.builder(sensor()).frameRate(Hertz.of(0)).build()),
            new Case("ImageSensor.latency", () -> sensor().latency(Milliseconds.of(-1))),
            new Case("ImageSensor.sightRange", () -> sensor().sightRange(Meters.of(0))));
    for (Case c : cases) {
      assertThatIllegalArgumentException()
          .as(c.says())
          .isThrownBy(c.build())
          .withMessageStartingWith(c.says());
    }
  }
}
