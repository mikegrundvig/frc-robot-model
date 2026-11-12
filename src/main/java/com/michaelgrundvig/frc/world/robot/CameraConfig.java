package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.atLeast;
import static com.michaelgrundvig.frc.world.config.Rules.below;
import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Microseconds;
import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Rule;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.Set;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Time;

/**
 * A camera on the robot, for AprilTag pose estimation. Where it sits and points is what turns what
 * the camera sees into where the robot is, so measure it carefully. The sensor describes the camera
 * itself, for simulation.
 *
 * <p>Positions are from the robot's origin, the frame's center on the floor, or, for a camera that
 * rides a mechanism, from its joint's frame, moving with it. Angles apply in the order yaw, tilt,
 * roll, each about the camera's own axes.
 *
 * @param name unique on the robot, even as an id ({@link #id()}); use the name the vision software
 *     knows the camera by
 *     <p>A camera starts from its name and its sensor ({@link #of}), and says the rest with a verb
 *     each: {@code CameraConfig.of("Camera6", SENSOR).at(Inches.of(-10.5), Inches.of(12),
 *     Inches.of(21)) .facing(Degrees.of(10), Degrees.of(180)).roll(Degrees.of(-2)).measured()}. A
 *     robot with a camera that isn't placed ({@link #at}) or pointed ({@link #facing}) is refused
 *     at startup.
 * @param forward how far ahead of the center the lens sits (negative: behind it), given with {@code
 *     left} and {@code up} by {@link #at}
 * @param left how far left of the center (negative: to the right)
 * @param up how far above the floor
 * @param roll rotation about the lens's axis; none given, zero, a camera mounted level, which the
 *     robot says at startup until the mount procedure gives it
 * @param tiltUp how far the camera is tilted up from level (negative: down), given with {@code yaw}
 *     by {@link #facing}
 * @param yaw which way it faces: zero is straight ahead, positive turns it to the left
 * @param sensor the camera itself, often shared by several cameras on a robot
 * @param noiseScale how much noisier this camera's results are than a typical one's: 1 for typical,
 *     2 to trust it half as much. Fit it from the camera's logged residuals. None given, 1 ({@link
 *     #DEFAULT_NOISE_SCALE}), which the robot says at startup
 * @param joint the joint whose link it rides on, which its mount is measured from; empty for a
 *     camera on the chassis
 * @param mountTolerance how far the camera may really sit from where the values above say: {@link
 *     MountTolerance#UNCHECKED} unless given, for a mount nobody has checked, which the robot says
 *     at startup. The mount procedure's report ends with {@code .measured()}, for a mount it has
 *     checked
 * @param software the vision program on the camera's coprocessor: {@link Software#PHOTONVISION}
 *     unless given
 */
@ConfigRecord
public record CameraConfig(
    String name,
    Optional<Distance> forward,
    Optional<Distance> left,
    Optional<Distance> up,
    Optional<Angle> roll,
    Optional<Angle> tiltUp,
    Optional<Angle> yaw,
    ImageSensor sensor,
    Optional<Double> noiseScale,
    Optional<Joint> joint,
    @RecordBuilder.Initializer(value = "UNCHECKED", source = MountTolerance.class)
        MountTolerance mountTolerance,
    @RecordBuilder.Initializer(value = "PHOTONVISION", source = Software.class) Software software) {
  /**
   * The vision program a camera's coprocessor runs, which decides how the robot reads it: {@link
   * #PHOTONVISION}, or {@link #VISOR} with whatever a deploy needs to set it up ({@link #visor()}).
   */
  public sealed interface Software permits Software.PhotonVision, Software.Visor {
    /** PhotonVision: results over NetworkTables, through PhotonLib. */
    Software PHOTONVISION = new PhotonVision();

    /**
     * The time from taking an image to its result reaching the robot that a camera on this software
     * is taken to have until its sensor says ({@link ImageSensor#latency}): a guess, which the
     * robot says at startup.
     */
    Time typicalLatency();

    /**
     * Visor, with nothing said of its device: enough for the simulator. On a robot, the robot
     * configures Visor on whichever coprocessor the camera is plugged into, found by its device
     * ({@link #visor()}).
     *
     * <p>It starts at a fixed 1 ms exposure, the camera's automatic exposure off: blur from the
     * robot turning is the main error in a tag's corners, and a short exposure cuts it. At 1 ms a
     * robot turning at 360°/s moves a tag 4 m away about 2.5 cm across the image; a top team runs
     * the ThriftyCam at 1 ms. Its gain starts as the camera's own ({@link Gain#CAMERAS_OWN}): a
     * {@code gain} control's units are the camera's, so no one number suits every camera, and
     * {@code visor cameras} on its coprocessor lists the range and default.
     */
    Visor VISOR =
        CameraConfigSoftwareVisorBuilder.builder()
            .exposure(Exposure.fixed(Milliseconds.of(1)))
            .gain(Gain.CAMERAS_OWN)
            .build();

    /**
     * A camera on Visor, with nothing said of it yet ({@link #VISOR}), each setting then a verb:
     * {@code Software.visor().device("/dev/v4l/by-id/...").lens(CAM_1)
     * .exposure(Milliseconds.of(1.5)).gain(40).stream(Percent.of(50), Hertz.of(10), 60)}.
     */
    static Visor visor() {
      return VISOR;
    }

    /**
     * PhotonVision, which keeps its settings and calibrations on the coprocessor itself. Its images
     * take about 30 ms to reach the robot: the detection on an Orange Pi 5 and the trip over
     * NetworkTables.
     */
    record PhotonVision() implements Software {
      @Override
      public Time typicalLatency() {
        return Milliseconds.of(30);
      }
    }

    /**
     * Visor: frames over UDP, through Visor's robot client. The camera's place in the robot's list
     * of cameras, from 0, is its index in Visor's configuration. What's here goes into that
     * configuration, which the robot writes for each coprocessor as it starts, from the cameras
     * each one reports plugged in; the simulator needs none of it.
     *
     * <p>Visor has no defaults of its own for the exposure and gain: they're here. Unless given, a
     * camera runs a fixed 1 ms with its automatic exposure off, and its own gain, as {@link #VISOR}
     * starts. Both can be tuned live from Pitwall's Tune view while the robot is disabled, and the
     * robot configures the board again; write the value that works back here, {@code
     * .exposure(Milliseconds.of(1.5))}, {@code .gain(40)}. Its live view, which Pitwall's card for
     * the camera shows, is Visor's own unless given, {@code .stream(Percent.of(25), Hertz.of(5),
     * 60)} to change it.
     *
     * @param device the camera's V4L2 device: its {@code /dev/v4l/by-id/} path, which names the
     *     camera itself (its USB serial), so the robot finds which coprocessor it's plugged into,
     *     and the same camera after a replug; needed on a robot
     * @param controls other camera settings by the names {@code v4l2-ctl --list-ctrls} prints, set
     *     in this order after the exposure and gain; none unless given, so the camera's own. Not
     *     the exposure or gain, which {@code exposure} and {@code gain} set: Visor refuses those,
     *     and the robot says so as it starts.
     * @param timeOffset added to every capture time: the spin test's result; none unless given
     * @param lens the lens calibration, from Visor's {@code tools/calibrate.py}; without one, Visor
     *     sends the tags' corners and no poses
     * @param exposure how long each image is exposed: a fixed time with the camera's automatic
     *     exposure off, or the camera's own automatic exposure
     * @param gain the camera's gain: a value of its {@code gain} control, or the camera's own
     * @param stream the camera's live view: its size, rate and JPEG quality; Visor's own unless
     *     given
     */
    @ConfigRecord
    record Visor(
        Optional<String> device,
        @RecordBuilder.Initializer(value = "of", source = List.class) List<Control> controls,
        Optional<Time> timeOffset,
        Optional<Lens> lens,
        Exposure exposure,
        Gain gain,
        Optional<Stream> stream)
        implements Software {
      public Visor {
        controls = List.copyOf(controls);
        Checks.on(Visor.class)
            .field("device", device, Rule.of(Visor::whyNotADevice))
            .field("controls", controls, Rule.of(Visor::whyControlsRepeat))
            .orThrow();
      }

      /**
       * The same camera, at its V4L2 device: the name {@code visor cameras --names} prints for it.
       * That is its {@code /dev/v4l/by-id/} path for a camera with a USB serial, and its USB port's
       * {@code /dev/v4l/by-path/} path for one without, such as a ThriftyCam: {@code
       * .device("/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0")}.
       */
      public Visor device(String device) {
        return CameraConfigSoftwareVisorBuilder.builder(this).device(device).build();
      }

      /**
       * The same camera, with other settings by V4L2's names, set in this order after the exposure
       * and gain: {@code .controls(List.of(Control.of("power_line_frequency", 2)))}.
       */
      public Visor controls(List<Control> controls) {
        return CameraConfigSoftwareVisorBuilder.builder(this).controls(controls).build();
      }

      /** The same camera, this added to every capture time: the spin test's result. */
      public Visor timeOffset(Time offset) {
        return CameraConfigSoftwareVisorBuilder.builder(this).timeOffset(offset).build();
      }

      /** The same camera, with its lens calibration, from Visor's {@code tools/calibrate.py}. */
      public Visor lens(Lens lens) {
        return CameraConfigSoftwareVisorBuilder.builder(this).lens(lens).build();
      }

      /**
       * The same camera, exposing each image this long, its automatic exposure off: {@code
       * .exposure(Milliseconds.of(1.5))}.
       */
      public Visor exposure(Time time) {
        return exposure(CameraConfigSoftwareExposureFixedBuilder.builder().time(time).build());
      }

      /**
       * The same camera, on its own automatic exposure, which Visor turns back on: it lengthens the
       * exposure in dim light, blurring the tags while the robot turns.
       */
      public Visor camerasOwnExposure() {
        return exposure(Exposure.CAMERAS_OWN);
      }

      private Visor exposure(Exposure exposure) {
        return CameraConfigSoftwareVisorBuilder.builder(this).exposure(exposure).build();
      }

      /**
       * The same camera, at this value of its {@code gain} control, in the camera's own units:
       * {@code .gain(40)}.
       */
      public Visor gain(int value) {
        Gain gain = CameraConfigSoftwareGainLevelBuilder.builder().value(value).build();
        return CameraConfigSoftwareVisorBuilder.builder(this).gain(gain).build();
      }

      /**
       * The same camera, its live view this share of the camera's size, across and down, at this
       * many pictures a second and this JPEG quality, 1 to 100 (higher is sharper, and more bytes a
       * picture): {@code .stream(Percent.of(50), Hertz.of(10), 60)}. Without one, Visor's own.
       */
      public Visor stream(Dimensionless scale, Frequency frameRate, int quality) {
        return CameraConfigSoftwareVisorBuilder.builder(this).stream(
                Stream.of(scale, frameRate, quality))
            .build();
      }

      /**
       * About 20 ms from taking an image to its frame reaching the robot: the detection on its
       * coprocessor and one UDP packet.
       */
      @Override
      public Time typicalLatency() {
        return Milliseconds.of(20);
      }

      private static Optional<String> whyNotADevice(Optional<String> device) {
        if (device.isPresent() && !device.get().startsWith("/dev/")) {
          return Optional.of(
              "is \""
                  + device.get()
                  + "\", not a device: give the name visor cameras --names prints for it, its"
                  + " /dev/v4l/by-id/ path, or its USB port's /dev/v4l/by-path/ path for a camera"
                  + " with no USB serial, such as a ThriftyCam");
        }
        return Optional.empty();
      }

      private static Optional<String> whyControlsRepeat(List<Control> controls) {
        Set<String> names = new HashSet<>();
        for (Control control : controls) {
          if (!names.add(control.name())) {
            return Optional.of("sets " + control.name() + " twice");
          }
        }
        return Optional.empty();
      }
    }

    /**
     * How long a camera on Visor exposes each image: a fixed time with its automatic exposure off
     * ({@code .exposure(Milliseconds.of(1))}), or its own ({@code .camerasOwnExposure()}). Visor
     * sets it every time it opens the camera, after a replug too.
     */
    sealed interface Exposure permits Exposure.CamerasOwn, Exposure.Fixed {
      /**
       * The camera's own automatic exposure, which Visor turns back on: it lengthens the exposure
       * in dim light, which blurs the tags while the robot turns.
       */
      Exposure CAMERAS_OWN = new CamerasOwn();

      /** A fixed exposure, the camera's automatic exposure off. */
      private static Exposure fixed(Time time) {
        return CameraConfigSoftwareExposureFixedBuilder.builder().time(time).build();
      }

      /** The camera's own automatic exposure. */
      record CamerasOwn() implements Exposure {}

      /**
       * A fixed exposure, the camera's automatic exposure off. V4L2 sets it in 100 µs steps, so
       * Visor sets the nearest.
       *
       * @param time how long each image is exposed: 100 µs, one of V4L2's steps, to 1 s; about 1 ms
       *     for AprilTags
       */
      @ConfigRecord
      record Fixed(Time time) implements Exposure {
        /**
         * The shortest exposure V4L2 sets: one of its 100 µs steps. A shorter one rounds to none,
         * which a camera refuses or takes as its automatic exposure.
         */
        static final Time SHORTEST = Microseconds.of(100);

        public Fixed {
          Checks.on(Fixed.class).field("time", time, between(SHORTEST, Seconds.of(1))).orThrow();
        }
      }
    }

    /**
     * A camera on Visor's gain: a value of its {@code gain} control ({@code .gain(40)}), or {@link
     * #CAMERAS_OWN}. Visor sets it every time it opens the camera, after a replug too.
     */
    sealed interface Gain permits Gain.CamerasOwn, Gain.Level {
      /** The camera's own gain: its {@code gain} control's default, which Visor puts back. */
      Gain CAMERAS_OWN = new CamerasOwn();

      /** The camera's own gain. */
      record CamerasOwn() implements Gain {}

      /**
       * A value of the camera's {@code gain} control.
       *
       * @param value in the camera's own units, a whole number from 0 to 65535 (UVC's range; {@code
       *     visor cameras} lists the camera's own, and Visor holds a value to it): higher brightens
       *     the image, and its noise
       */
      @ConfigRecord
      record Level(int value) implements Gain {
        /** The most UVC's gain control holds: it's an unsigned 16-bit number. */
        private static final int MOST = 65_535;

        public Level {
          Checks.on(Level.class).field("value", value, Rule.of(Level::whyNotAGain)).orThrow();
        }

        // Written by hand, not with between(0, MOST): its message shows a number to four
        // figures, and a gain is a whole number shown whole.
        private static Optional<String> whyNotAGain(int value) {
          if (value >= 0 && value <= MOST) {
            return Optional.empty();
          }
          return Optional.of("is " + value + ": a gain control takes 0 to " + MOST);
        }
      }
    }

    /**
     * A camera on Visor's live view: its frames, made smaller, with each tag outlined and its ID,
     * and a line with the frame rate, exposure, gain and brightness. Visor serves it from the
     * camera's coprocessor, and Pitwall's card for the camera shows it, the browser fetching it
     * from the board directly. Visor makes it only while someone watches, and never at detection's
     * expense: a picture costs a few milliseconds of a lower-priority thread.
     *
     * @param scale its size as a share of the camera's, across and down, from 5% to 100%: 50% is
     *     half, 800 x 652 of a 1600 x 1304 camera
     * @param frameRate pictures a second while someone watches, at least 1; one faster than the
     *     camera's own runs at the camera's
     * @param quality JPEG quality, 1 to 100: higher is sharper, and more bytes a picture
     */
    @ConfigRecord
    record Stream(Dimensionless scale, Frequency frameRate, int quality) {
      public Stream {
        Checks.on(Stream.class)
            .field("scale", scale, between(Percent.of(5), Percent.of(100)))
            .field("frameRate", frameRate, atLeast(Hertz.of(1)))
            .field("quality", quality, between(1, 100))
            .orThrow();
      }

      /** A live view this share of the camera's size, this many a second, at this quality. */
      private static Stream of(Dimensionless scale, Frequency frameRate, int quality) {
        return CameraConfigSoftwareStreamBuilder.builder()
            .scale(scale)
            .frameRate(frameRate)
            .quality(quality)
            .build();
      }
    }

    /**
     * One camera setting, as V4L2 names it: {@code Control.of("power_line_frequency", 2)}.
     *
     * @param name as {@code v4l2-ctl --list-ctrls} prints it
     * @param value a number, or for a setting that is on or off, 1 and 0 ({@link #on}, {@link
     *     #off})
     */
    record Control(String name, long value) {
      public Control {
        Checks.on(Control.class, name).field("name", name, Rule.of(Control::whyNotAName)).orThrow();
      }

      /** A setting by its name. */
      public static Control of(String name, long value) {
        return new Control(name, value);
      }

      /** A setting that is on or off, turned on: {@code Control.on("white_balance_automatic")}. */
      public static Control on(String name) {
        return new Control(name, 1);
      }

      /**
       * A setting that is on or off, turned off: {@code Control.off("white_balance_automatic")}.
       */
      public static Control off(String name) {
        return new Control(name, 0);
      }

      private static Optional<String> whyNotAName(String name) {
        if (name.matches("[a-z0-9_]+")) {
          return Optional.empty();
        }
        return Optional.of(
            "is \"" + name + "\": V4L2's names are lowercase letters, digits and underscores");
      }
    }

    /**
     * A camera's lens, as Visor's {@code tools/calibrate.py solve} measures it and prints it to
     * paste here: OpenCV's model, at the frame size it was calibrated at, which must be the
     * camera's.
     *
     * @param width the frame size it was calibrated at
     * @param height the frame size it was calibrated at
     * @param fx focal length across, in pixels
     * @param fy focal length down, in pixels
     * @param cx the optical centre across, in pixels
     * @param cy the optical centre down, in pixels
     * @param distortion OpenCV's coefficients in its order (k1, k2, p1, p2, k3, then k4, k5, k6):
     *     0, 4, 5 or 8 of them
     */
    @ConfigRecord
    record Lens(
        int width,
        int height,
        double fx,
        double fy,
        double cx,
        double cy,
        List<Double> distortion) {
      public Lens {
        distortion = List.copyOf(distortion);
        Checks.on(Lens.class)
            .field("width", width, positive())
            .field("height", height, positive())
            .field("fx", fx, positive())
            .field("fy", fy, positive())
            .field("cx", cx, finite())
            .field("cy", cy, finite())
            .field("distortion", distortion, Rule.of(Lens::whyNotOpenCvs))
            .orThrow();
      }

      /** Starts building a lens, one named value at a time. */
      public static CameraConfigSoftwareLensBuilder.WidthStage builder() {
        return CameraConfigSoftwareLensBuilder.builder();
      }

      /**
       * The angle it sees across its image's diagonal, corner to corner, as its focal lengths and
       * optical centre put the two corners, leaving out its distortion: the angle between the rays
       * through the image's top left and bottom right corners.
       */
      public Angle diagonalFieldOfView() {
        double[] topLeft = {-cx / fx, -cy / fy, 1};
        double[] bottomRight = {(width - cx) / fx, (height - cy) / fy, 1};
        double dot =
            topLeft[0] * bottomRight[0] + topLeft[1] * bottomRight[1] + topLeft[2] * bottomRight[2];
        double lengths = length(topLeft) * length(bottomRight);
        return Radians.of(StrictMath.acos(dot / lengths));
      }

      private static double length(double[] ray) {
        return StrictMath.sqrt(ray[0] * ray[0] + ray[1] * ray[1] + ray[2] * ray[2]);
      }

      private static Optional<String> whyNotOpenCvs(List<Double> distortion) {
        if (!List.of(0, 4, 5, 8).contains(distortion.size())) {
          return Optional.of(
              "has " + distortion.size() + " coefficients; OpenCV's model has 0, 4, 5 or 8");
        }
        if (distortion.stream().anyMatch(c -> !Double.isFinite(c))) {
          return Optional.of("has a coefficient that isn't a number");
        }
        return Optional.empty();
      }
    }
  }

  /** How much noisier a camera's results are than a typical one's when it doesn't say: typical. */
  public static final double DEFAULT_NOISE_SCALE = 1;

  /**
   * The angle a camera is taken to see across its diagonal when neither its sensor nor a lens says:
   * 80°, a common AprilTag camera's (an Arducam OV9281's lens). Only the simulator and the views
   * read it.
   */
  public static final Angle DEFAULT_FIELD_OF_VIEW = Degrees.of(80);

  /**
   * How far a camera is taken to find tags when its sensor doesn't say: 7 m, about where a 6.5 in
   * tag shrinks to 20 pixels across a 1280 x 800 image through an 80° lens. Only the simulator and
   * the views read it.
   */
  public static final Distance DEFAULT_SIGHT_RANGE = Meters.of(7);

  /**
   * How far a field of view given beside a lens may differ from the lens's before it's doubted.
   * Four calibrations of one lens model differ by about a degree, and a datasheet's angle by more.
   */
  private static final Angle FIELD_OF_VIEW_AGREEMENT = Degrees.of(2);

  public CameraConfig {
    Checks.on(CameraConfig.class, name)
        .field("name", name, notBlank())
        .field("forward", forward, given(finite()))
        .field("left", left, given(finite()))
        .field("up", up, given(finite()))
        .field("roll", roll, given(finite()))
        .field("tiltUp", tiltUp, given(finite()))
        .field("yaw", yaw, given(finite()))
        .field("noiseScale", noiseScale, given(positive()))
        .field("software", software, Rule.of(given -> whyLensIsAnotherSize(given, sensor)))
        .orThrow();
  }

  /** A lens calibrated at another frame size than the camera's gives wrong poses everywhere. */
  private static Optional<String> whyLensIsAnotherSize(Software software, ImageSensor sensor) {
    if (software instanceof Software.Visor visor && visor.lens().isPresent()) {
      Software.Lens lens = visor.lens().get();
      if (lens.width() != sensor.widthPixels() || lens.height() != sensor.heightPixels()) {
        return Optional.of(
            "has a lens calibrated at "
                + lens.width()
                + "x"
                + lens.height()
                + ", but the camera runs at "
                + sensor.widthPixels()
                + "x"
                + sensor.heightPixels()
                + ": calibrate at the size it runs");
      }
    }
    return Optional.empty();
  }

  /**
   * A camera by its name, with its sensor, placed and pointed by {@link #at} and {@link #facing}
   * and the rest said by a verb each: {@code CameraConfig.of("Camera6", SENSOR)}.
   *
   * @param name unique on the robot, even as an id ({@link #id()}); use the name the vision
   *     software knows the camera by
   * @param sensor the camera itself, often shared by several cameras on a robot
   */
  public static CameraConfig of(String name, ImageSensor sensor) {
    return builder().name(name).sensor(sensor).build();
  }

  /**
   * The same camera, its lens sitting here: from the robot's origin, the frame's center on the
   * floor, or for a camera on a joint ({@link #on}), from the joint's frame.
   *
   * @param forward how far ahead of the center the lens sits (negative: behind it)
   * @param left how far left of the center (negative: to the right)
   * @param up how far above the floor
   * @throws IllegalArgumentException if it's placed already
   */
  public CameraConfig at(Distance forward, Distance left, Distance up) {
    saidOnce(this.forward.isPresent(), "its place", ".at(forward, left, up)");
    return with().forward(forward).left(left).up(up).build();
  }

  /**
   * The same camera, pointing this way. Angles apply in the order yaw, tilt, roll, each about the
   * camera's own axes.
   *
   * @param tiltUp how far the camera is tilted up from level (negative: down)
   * @param yaw which way it faces: zero is straight ahead, positive turns it to the left
   * @throws IllegalArgumentException if it's pointed already
   */
  public CameraConfig facing(Angle tiltUp, Angle yaw) {
    saidOnce(this.tiltUp.isPresent(), "its facing", ".facing(tiltUp, yaw)");
    return with().tiltUp(tiltUp).yaw(yaw).build();
  }

  /**
   * The same camera, turned this far about its lens's axis.
   *
   * @param roll rotation about the lens's axis; left out, a camera mounted level, which the robot
   *     says at startup until the mount procedure gives it
   * @throws IllegalArgumentException if its roll is given already
   */
  public CameraConfig roll(Angle roll) {
    saidOnce(this.roll.isPresent(), "its roll", ".roll(angle)");
    return with().roll(roll).build();
  }

  /**
   * The same camera, its mount checked by the mount procedure (docs/calibration.md): {@link
   * MountTolerance#MEASURED}, what the procedure's report ends with.
   *
   * @throws IllegalArgumentException if its mount's tolerance is given already
   */
  public CameraConfig measured() {
    return mountTolerance(MountTolerance.MEASURED);
  }

  /**
   * The same camera, sitting within this of where its mount says.
   *
   * @param mountTolerance how far the camera may really sit from where its mount says: {@link
   *     MountTolerance#UNCHECKED} unless given, for a mount nobody has checked
   * @throws IllegalArgumentException if its mount's tolerance is given already
   */
  public CameraConfig mountTolerance(MountTolerance mountTolerance) {
    saidOnce(
        !this.mountTolerance.equals(MountTolerance.UNCHECKED),
        "its mount's tolerance",
        ".measured() or .mountTolerance(...)");
    return with().mountTolerance(mountTolerance).build();
  }

  /**
   * The same camera, riding on a joint's link: its mount is then measured from the joint's frame,
   * moving with it.
   *
   * @param joint the joint whose link it rides on
   * @throws IllegalArgumentException if it rides on a joint already
   */
  public CameraConfig on(Joint joint) {
    saidOnce(this.joint.isPresent(), "its joint", ".on(joint)");
    return with().joint(joint).build();
  }

  /**
   * The same camera, this much noisier than a typical one: 2 to trust it half as much. Fit it from
   * the camera's logged residuals.
   *
   * @param noiseScale how much noisier this camera's results are than a typical one's: 1 for
   *     typical
   * @throws IllegalArgumentException if its noise scale is given already
   */
  public CameraConfig noiseScale(double noiseScale) {
    saidOnce(this.noiseScale.isPresent(), "its noise scale", ".noiseScale(scale)");
    return with().noiseScale(noiseScale).build();
  }

  /**
   * The same camera, on this vision program: {@code .software(Software.visor().device(...))}.
   *
   * @param software the vision program on the camera's coprocessor: {@link Software#PHOTONVISION}
   *     unless given
   * @throws IllegalArgumentException if its software is given already: {@link #withSoftware}
   *     replaces it
   */
  public CameraConfig software(Software software) {
    saidOnce(
        !this.software.equals(Software.PHOTONVISION),
        "its software",
        ".software(...), or replace it with .withSoftware(...)");
    return withSoftware(software);
  }

  /**
   * The same camera on another vision program, or the same program set up otherwise: a camera's
   * Visor settings as Pitwall tuned them.
   */
  public CameraConfig withSoftware(Software software) {
    return with().software(software).build();
  }

  private CameraConfigBuilder with() {
    return CameraConfigBuilder.builder(this);
  }

  /** The rule that a camera says each part of it once. */
  private void saidOnce(boolean alreadySaid, String has, String call) {
    if (alreadySaid) {
      throw new IllegalArgumentException(
          "CameraConfig[" + name + "] has " + has + " already: give it once, in one " + call);
    }
  }

  /**
   * Why a camera can't be used, if it can't: it isn't placed ({@link #at}) or pointed ({@link
   * #facing}). The robot's definition refuses it, with the call to write.
   */
  Optional<String> whyNotMounted() {
    if (forward.isEmpty() || left.isEmpty() || up.isEmpty()) {
      return Optional.of(
          "CameraConfig[" + name + "] has no place: give it one, .at(forward, left, up)");
    }
    if (tiltUp.isEmpty() || yaw.isEmpty()) {
      return Optional.of(
          "CameraConfig[" + name + "] has no facing: give it one, .facing(tiltUp, yaw)");
    }
    return Optional.empty();
  }

  /** Starts building a camera, one named value at a time. */
  static CameraConfigBuilder.NameStage builder() {
    return CameraConfigBuilder.builder();
  }

  /**
   * Its name as an id, as tools that can't take any text name it (Pitwall's World view): lowercase
   * letters, digits, dots, hyphens and underscores, everything else a hyphen ("Front Left" is
   * "front-left"), starting with a letter or digit. Unique on the robot, as its name is.
   */
  public String id() {
    return id(name);
  }

  private static String id(String name) {
    String cleaned = name.toLowerCase(Locale.ROOT).replaceAll("[^a-z0-9._-]+", "-");
    boolean startsWithALetterOrDigit =
        !cleaned.isEmpty() && Character.isLetterOrDigit(cleaned.charAt(0));
    if (startsWithALetterOrDigit) {
      return cleaned;
    }
    return "x" + cleaned;
  }

  /** Its rotation about the lens's axis: as given, or zero, a camera mounted level. */
  public Angle rollOrDefault() {
    return roll.orElse(Degrees.of(0));
  }

  /** How much noisier its results are than a typical camera's: as given, or {@code 1}. */
  public double noiseScaleOrDefault() {
    return noiseScale.orElse(DEFAULT_NOISE_SCALE);
  }

  /**
   * The average time from taking an image to its result reaching the robot: its sensor's, or until
   * that's measured, its software's typical one ({@link Software#typicalLatency}).
   */
  public Time latencyOrDefault() {
    return sensor.latency().orElseGet(software::typicalLatency);
  }

  /** The farthest it finds a tag in most images: its sensor's, or {@link #DEFAULT_SIGHT_RANGE}. */
  public Distance sightRangeOrDefault() {
    return sensor.sightRange().orElse(DEFAULT_SIGHT_RANGE);
  }

  /** Its lens's calibration, for a camera on Visor that has one. */
  public Optional<Software.Lens> lens() {
    if (software instanceof Software.Visor visor) {
      return visor.lens();
    }
    return Optional.empty();
  }

  /**
   * The angle it sees across its image's diagonal, what the simulator's cameras and the views take:
   * its lens's, where it has one ({@link Software.Lens#diagonalFieldOfView}), so the calibration is
   * its one owner; otherwise its sensor's; otherwise {@link #DEFAULT_FIELD_OF_VIEW}.
   */
  public Angle diagonalFieldOfView() {
    if (lens().isPresent()) {
      return lens().get().diagonalFieldOfView();
    }
    return sensor.diagonalFieldOfView().orElse(DEFAULT_FIELD_OF_VIEW);
  }

  /**
   * Why the field of view its sensor gives is doubtful beside its lens's, if it is: more than 2°
   * apart across the diagonal. The lens's is the one used.
   */
  public Optional<String> whyFieldOfViewDisagreesWithLens() {
    Optional<Angle> given = sensor.diagonalFieldOfView();
    if (given.isEmpty() || lens().isEmpty()) {
      return Optional.empty();
    }
    Angle fromLens = lens().get().diagonalFieldOfView();
    double apartDegrees = Math.abs(given.get().in(Degrees) - fromLens.in(Degrees));
    if (apartDegrees <= FIELD_OF_VIEW_AGREEMENT.in(Degrees)) {
      return Optional.empty();
    }
    return Optional.of(
        String.format(
            Locale.ROOT,
            "camera %s's sensor gives a field of view of %.1f° across the diagonal, but its lens"
                + " calibration sees %.1f°: the lens's is used. Leave the sensor's out, or check"
                + " which is right",
            name,
            given.get().in(Degrees),
            fromLens.in(Degrees)));
  }

  /**
   * Where the lens sits: from the robot's origin, or from its joint's frame.
   *
   * @throws IllegalStateException if it isn't placed, which a robot's definition refuses
   */
  public Translation3d position() {
    if (forward.isEmpty() || left.isEmpty() || up.isEmpty()) {
      throw new IllegalStateException(
          "CameraConfig[" + name + "] has no place: give it one, .at(forward, left, up)");
    }
    return new Translation3d(forward.get(), left.get(), up.get());
  }

  /**
   * Where the camera sits and points, in the form WPILib's vision code takes: from the robot's
   * origin, or for a camera on a joint, from the joint's frame ({@code RobotMath.cameraPose} puts
   * it on the robot at the joint's position). WPILib's pitch is positive downward, so the tilt up
   * becomes a negative pitch.
   */
  public Transform3d robotToCamera() {
    mounted();
    Rotation3d turn = new Rotation3d(rollOrDefault(), tiltUp.get().unaryMinus(), yaw.get());
    return new Transform3d(position(), turn);
  }

  /** Throws, saying the call to write, if it isn't placed or pointed. */
  private void mounted() {
    Optional<String> whyNot = whyNotMounted();
    if (whyNot.isPresent()) {
      throw new IllegalStateException(whyNot.get());
    }
  }

  /**
   * The camera itself, as the vision software runs it: {@code ImageSensor.of(1280, 800,
   * Hertz.of(50))}, the size and rate {@code visor cameras} or PhotonVision lists. The rest is
   * optional, each said by its own call: {@code .fieldOfView(Degrees.of(70), Across.HORIZONTAL)},
   * {@code .latency(Milliseconds.of(25))}, {@code .sightRange(Meters.of(6))}. A camera whose lens
   * is calibrated takes its field of view from the lens, and needs none here.
   *
   * @param widthPixels width of the images the vision software processes
   * @param heightPixels height of those images
   * @param frameRate images processed per second
   * @param fieldOfView the angle the camera sees across its image: across the width or the
   *     diagonal, as {@code fieldOfViewAcross} says. A camera's datasheet and most teams' code give
   *     the horizontal one; PhotonVision's lens calibration, and its simulated cameras, the
   *     diagonal. None given, the lens's where the camera has one, otherwise {@link
   *     CameraConfig#DEFAULT_FIELD_OF_VIEW}, which the simulator says at startup
   * @param fieldOfViewAcross which way {@code fieldOfView} is across the image: {@link
   *     Across#DIAGONAL} unless given
   * @param latency the average time from taking an image to its result reaching the robot. Images
   *     that take much longer are thrown out, so measure it: the Vision log's {@code
   *     LatencySeconds} for the camera, less the wait for the robot's loop. The simulator's cameras
   *     take this long too. None given, its software's typical one ({@link
   *     Software#typicalLatency}), which the robot says at startup
   * @param sightRange the farthest a tag is found in most images: farther tags are too small in the
   *     image to read. The simulator's cameras detect tags no farther than this, and AdvantageScope
   *     draws each camera's field of view out to it. None given, {@link
   *     CameraConfig#DEFAULT_SIGHT_RANGE}, which the simulator says at startup
   */
  @ConfigRecord
  public record ImageSensor(
      int widthPixels,
      int heightPixels,
      Frequency frameRate,
      Optional<Angle> fieldOfView,
      Across fieldOfViewAcross,
      Optional<Time> latency,
      Optional<Distance> sightRange) {
    public ImageSensor {
      Checks.on(ImageSensor.class)
          .field("widthPixels", widthPixels, positive())
          .field("heightPixels", heightPixels, positive())
          .field("frameRate", frameRate, positive())
          .field("fieldOfView", fieldOfView, given(positive()))
          .field("fieldOfView", fieldOfView, given(below(Degrees.of(180))))
          .field("latency", latency, given(nonNegative()))
          .field("sightRange", sightRange, given(positive()))
          .orThrow();
    }

    /** A camera's images: their size, and how many a second. */
    public static ImageSensor of(int widthPixels, int heightPixels, Frequency frameRate) {
      return CameraConfigImageSensorBuilder.builder()
          .widthPixels(widthPixels)
          .heightPixels(heightPixels)
          .frameRate(frameRate)
          .fieldOfViewAcross(Across.DIAGONAL)
          .build();
    }

    /** The same, seeing this angle across its width or its diagonal. */
    public ImageSensor fieldOfView(Angle angle, Across across) {
      return with().fieldOfView(angle).fieldOfViewAcross(across).build();
    }

    /** The same, its images taking this long on average to reach the robot. */
    public ImageSensor latency(Time latency) {
      return with().latency(latency).build();
    }

    /** The same, finding tags no farther than this. */
    public ImageSensor sightRange(Distance sightRange) {
      return with().sightRange(sightRange).build();
    }

    private CameraConfigImageSensorBuilder with() {
      return CameraConfigImageSensorBuilder.builder(this);
    }

    /**
     * The angle it sees across its image's diagonal, if its field of view is given: as given, or
     * from the horizontal one through a lens with square pixels, whose tangents of half the angle
     * scale as the image's diagonal over its width.
     */
    public Optional<Angle> diagonalFieldOfView() {
      if (fieldOfView.isEmpty() || fieldOfViewAcross == Across.DIAGONAL) {
        return fieldOfView;
      }
      double diagonal = StrictMath.hypot(widthPixels, heightPixels);
      double halfAcross = fieldOfView.get().in(Radians) / 2;
      return Optional.of(
          Radians.of(2 * StrictMath.atan(StrictMath.tan(halfAcross) * diagonal / widthPixels)));
    }
  }

  /**
   * How far a camera may really sit from its configured mount: the most its lens may be off along
   * each of the robot's axes, and the most it may be turned about each. Whatever uses the mount
   * allows for that much, and no more: the localizer starts each camera's turn on its mount as
   * unsure as this, and learns the rest from the camera's images.
   *
   * <p>An error drawn evenly within a bound has a third of the bound's square as its variance
   * ({@link #turnStdDevRadians}, {@link #offsetStdDevMeters}).
   *
   * @param along the most the lens may sit off its configured place, along each axis
   * @param about the most the camera may be turned off its configured angles, about each axis
   */
  @ConfigRecord
  public record MountTolerance(Distance along, Angle about) {
    /**
     * A mount measured from CAD and checked with the camera calibration (docs/calibration.md): it
     * lands within a few millimeters and a fraction of a degree of the real one, so 5 mm and 1° at
     * the end of that range. A bumped camera, or one never checked, may be off by a degree or more.
     */
    public static final MountTolerance MEASURED =
        CameraConfigMountToleranceBuilder.builder()
            .along(Millimeters.of(5))
            .about(Degrees.of(1))
            .build();

    /**
     * A mount nobody has checked, what a camera has until its definition says: measured with a tape
     * or read off CAD, and never put right by the mount procedure (docs/calibration.md). A camera
     * placed by hand lands within about an inch, and a bracket bent a little turns it a few
     * degrees, so 25 mm and 3°. The localizer starts as unsure of it as that, and learns its turn
     * from its images; the procedure's report ends with {@code .measured()}.
     */
    public static final MountTolerance UNCHECKED =
        CameraConfigMountToleranceBuilder.builder()
            .along(Millimeters.of(25))
            .about(Degrees.of(3))
            .build();

    public MountTolerance {
      Checks.on(MountTolerance.class)
          .field("along", along, nonNegative())
          .field("along", along, finite())
          .field("about", about, nonNegative())
          .field("about", about, below(Degrees.of(90)))
          .orThrow();
    }

    /**
     * A mount within this much of where its camera says it is: {@code
     * MountTolerance.of(Millimeters.of(5), Degrees.of(1))}.
     *
     * @param along the most the lens may sit off its configured place, along each axis
     * @param about the most the camera may be turned off its configured angles, about each axis
     */
    public static MountTolerance of(Distance along, Angle about) {
      return builder().along(along).about(about).build();
    }

    /** Starts building a mount tolerance, one named value at a time. */
    static CameraConfigMountToleranceBuilder.AlongStage builder() {
      return CameraConfigMountToleranceBuilder.builder();
    }

    /**
     * The standard deviation of a turn drawn evenly within the bound about an axis, in radians: the
     * bound over the square root of three.
     */
    public double turnStdDevRadians() {
      return about.in(Radians) / Math.sqrt(3);
    }

    /**
     * The standard deviation of an offset drawn evenly within the bound along an axis, in meters:
     * the bound over the square root of three.
     */
    public double offsetStdDevMeters() {
      return along.in(Meters) / Math.sqrt(3);
    }
  }

  /** Which way a camera's field of view is across its image. */
  public enum Across {
    /** Across its width: a camera's datasheet's, a Limelight's. */
    HORIZONTAL,
    /** Across its diagonal: PhotonVision's lens calibration's. */
    DIAGONAL
  }
}
