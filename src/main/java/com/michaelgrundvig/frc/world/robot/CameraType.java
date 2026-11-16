package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.atLeast;
import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Microseconds;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.Seconds;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Rule;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.wpilib.units.measure.Dimensionless;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Time;

/**
 * A kind of camera, shared by every camera of that kind on the robot: the images it gives, and the
 * settings each camera of the kind starts with on Visor. {@code CameraType.of("ThriftyCam",
 * ImageSensor.of(1600, 1304, Hertz.of(60)))}, the rest each said by a verb: {@code .gain(40)},
 * {@code .exposure(Milliseconds.of(1.5))}, {@code .stream(Percent.of(50), Hertz.of(10), 60)}.
 *
 * <p>The exposure and gain here are where a camera starts. A venue can carry its own for its
 * lighting, chosen in the pit and stored on the robot, and the pit can tune them live; the robot
 * says at startup which each camera uses and where each came from. PhotonVision keeps its own
 * settings, so a camera on PhotonVision reads none of these.
 *
 * @param name what the kind is called, such as "ThriftyCam": the robot's startup report and the
 *     pre-competition checklist name it, and a venue keeps the kind's exposure and gain by it, so
 *     two kinds of camera never share one
 * @param sensor the images it gives: their size and rate, and how far it sees
 * @param exposure how long each image is exposed: a fixed 1 ms with the camera's automatic exposure
 *     off unless given ({@link #DEFAULT_EXPOSURE}), or the camera's own automatic exposure
 * @param gain the camera's gain: the camera's own unless given ({@link Gain#CAMERAS_OWN}), or a
 *     value of its {@code gain} control. This is where a measured gain for the kind goes, {@code
 *     .gain(40)}
 * @param controls other camera settings by the names {@code v4l2-ctl --list-ctrls} prints, set in
 *     this order after the exposure and gain; none unless given, so the camera's own. Not the
 *     exposure or gain, which {@code exposure} and {@code gain} set: Visor refuses those, and the
 *     robot says so as it starts.
 * @param stream the camera's live view: its size, rate and JPEG quality; Visor's own unless given
 * @param gainRange the values the kind's {@code gain} control takes, as its datasheet or firmware
 *     gives them, {@code .gainRange(16, 255)}: what the pit's tuning of its gain goes between, and
 *     what a venue's gain for it is held to. Unless given, UVC's whole range, 0 to 65535, which
 *     Visor then holds to the camera's own
 */
@ConfigRecord
public record CameraType(
    String name,
    ImageSensor sensor,
    Exposure exposure,
    Gain gain,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Control> controls,
    Optional<Stream> stream,
    Optional<GainRange> gainRange) {
  /**
   * The exposure a camera starts at unless its type says: a fixed 1 ms, the camera's automatic
   * exposure off. Blur from the robot turning is the main error in a tag's corners, and a short
   * exposure cuts it. At 1 ms a robot turning at 360°/s moves a tag 4 m away about 2.5 cm across
   * the image; a top team runs the ThriftyCam at 1 ms.
   */
  public static final Exposure DEFAULT_EXPOSURE = Exposure.fixed(Milliseconds.of(1));

  public CameraType {
    controls = List.copyOf(controls);
    Checks.on(CameraType.class, name)
        .field("name", name, notBlank())
        .field("controls", controls, Rule.of(CameraType::whyControlsRepeat))
        .field("gain", gain, Rule.of(given -> whyGainIsOutsideItsRange(given, gainRange)))
        .orThrow();
  }

  /** A kind's gain outside the range its control takes would be refused by the camera. */
  private static Optional<String> whyGainIsOutsideItsRange(Gain gain, Optional<GainRange> range) {
    if (!(gain instanceof Gain.Level level)
        || range.isEmpty()
        || range.get().holds(level.value())) {
      return Optional.empty();
    }
    return Optional.of(
        "is "
            + level.value()
            + ", outside the "
            + range.get().lowest()
            + " to "
            + range.get().highest()
            + " the kind's gain control takes: give a gain in that range, or the range its"
            + " datasheet gives");
  }

  /**
   * A kind of camera by its name and its images, at a fixed 1 ms exposure and the camera's own
   * gain: {@code CameraType.of("ThriftyCam", ImageSensor.of(1600, 1304, Hertz.of(60)))}.
   */
  public static CameraType of(String name, ImageSensor sensor) {
    return CameraTypeBuilder.builder()
        .name(name)
        .sensor(sensor)
        .exposure(DEFAULT_EXPOSURE)
        .gain(Gain.CAMERAS_OWN)
        .build();
  }

  /**
   * The same kind, starting each image at this exposure, its automatic exposure off: {@code
   * .exposure(Milliseconds.of(1.5))}.
   */
  public CameraType exposure(Time time) {
    return with().exposure(Exposure.fixed(time)).build();
  }

  /**
   * The same kind, on its own automatic exposure, which Visor turns back on: it lengthens the
   * exposure in dim light, blurring the tags while the robot turns.
   */
  public CameraType camerasOwnExposure() {
    return with().exposure(Exposure.CAMERAS_OWN).build();
  }

  /**
   * The same kind, starting at this value of its {@code gain} control, in the camera's own units:
   * {@code .gain(40)}.
   */
  public CameraType gain(int value) {
    return with().gain(Gain.level(value)).build();
  }

  /**
   * The same kind, its {@code gain} control taking these values, as its datasheet or firmware gives
   * them: {@code .gainRange(16, 255)}. The pit tunes its gain between them, and a venue's gain for
   * it is held to them.
   *
   * @param lowest the least gain the control takes
   * @param highest the most, at least {@code lowest}
   */
  public CameraType gainRange(int lowest, int highest) {
    GainRange range = CameraTypeGainRangeBuilder.builder().lowest(lowest).highest(highest).build();
    return with().gainRange(range).build();
  }

  /**
   * The same kind, with other settings by V4L2's names, set in this order after the exposure and
   * gain: {@code .controls(List.of(Control.of("power_line_frequency", 2)))}.
   */
  public CameraType controls(List<Control> controls) {
    return with().controls(controls).build();
  }

  /**
   * The same kind, its live view this share of the camera's size, across and down, at this many
   * pictures a second and this JPEG quality, 1 to 100 (higher is sharper, and more bytes a
   * picture): {@code .stream(Percent.of(50), Hertz.of(10), 60)}. Without one, Visor's own.
   */
  public CameraType stream(Dimensionless scale, Frequency frameRate, int quality) {
    return with().stream(Stream.of(scale, frameRate, quality)).build();
  }

  private CameraTypeBuilder with() {
    return CameraTypeBuilder.builder(this);
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

  /**
   * How long a camera on Visor exposes each image: a fixed time with its automatic exposure off
   * ({@code .exposure(Milliseconds.of(1))}), or its own ({@code .camerasOwnExposure()}). Visor sets
   * it every time it opens the camera, after a replug too.
   */
  public sealed interface Exposure permits Exposure.CamerasOwn, Exposure.Fixed {
    /**
     * The camera's own automatic exposure, which Visor turns back on: it lengthens the exposure in
     * dim light, which blurs the tags while the robot turns.
     */
    Exposure CAMERAS_OWN = new CamerasOwn();

    /** A fixed exposure, the camera's automatic exposure off. */
    static Exposure fixed(Time time) {
      return CameraTypeExposureFixedBuilder.builder().time(time).build();
    }

    /** The camera's own automatic exposure. */
    record CamerasOwn() implements Exposure {}

    /**
     * A fixed exposure, the camera's automatic exposure off. V4L2 sets it in 100 µs steps, so Visor
     * sets the nearest.
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
      public static final Time SHORTEST = Microseconds.of(100);

      /** The longest exposure a camera is set to here. */
      public static final Time LONGEST = Seconds.of(1);

      public Fixed {
        Checks.on(Fixed.class).field("time", time, between(SHORTEST, LONGEST)).orThrow();
      }
    }
  }

  /**
   * A camera on Visor's gain: a value of its {@code gain} control ({@code .gain(40)}), or {@link
   * #CAMERAS_OWN}. Visor sets it every time it opens the camera, after a replug too.
   */
  public sealed interface Gain permits Gain.CamerasOwn, Gain.Level {
    /** The camera's own gain: its {@code gain} control's default, which Visor puts back. */
    Gain CAMERAS_OWN = new CamerasOwn();

    /** A value of the camera's {@code gain} control. */
    static Gain level(int value) {
      return CameraTypeGainLevelBuilder.builder().value(value).build();
    }

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
      public static final int MOST = 65_535;

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
   * The values a kind of camera's {@code gain} control takes: {@code .gainRange(16, 255)}.
   *
   * @param lowest the least gain the control takes, from 0
   * @param highest the most, from {@code lowest} to 65535 (UVC's gain control is an unsigned 16-bit
   *     number)
   */
  @ConfigRecord
  public record GainRange(int lowest, int highest) {
    public GainRange {
      Checks.on(GainRange.class)
          .field("lowest", lowest, Rule.of(GainRange::whyNotAGain))
          .field("highest", highest, Rule.of(GainRange::whyNotAGain))
          .field("highest", highest, Rule.of((Integer most) -> whyBackwards(lowest, most)))
          .orThrow();
    }

    /** Whether the control takes this gain. */
    public boolean holds(int gain) {
      return gain >= lowest && gain <= highest;
    }

    /** The gain in the range nearest this one: itself where the control takes it. */
    public int nearest(int gain) {
      return Math.max(lowest, Math.min(highest, gain));
    }

    private static Optional<String> whyNotAGain(int value) {
      if (value >= 0 && value <= Gain.Level.MOST) {
        return Optional.empty();
      }
      return Optional.of("is " + value + ": a gain control takes 0 to " + Gain.Level.MOST);
    }

    private static Optional<String> whyBackwards(int lowest, int highest) {
      if (highest >= lowest) {
        return Optional.empty();
      }
      return Optional.of(
          "is "
              + highest
              + ", below lowest ("
              + lowest
              + "): give the least first, .gainRange("
              + highest
              + ", "
              + lowest
              + ")");
    }
  }

  /**
   * A camera on Visor's live view: its frames, made smaller, with each tag outlined and its ID, and
   * a line with the frame rate, exposure, gain and brightness. Visor serves it from the camera's
   * coprocessor, and Pitwall's card for the camera shows it, the browser fetching it from the board
   * directly. Visor makes it only while someone watches, and never at detection's expense: a
   * picture costs a few milliseconds of a lower-priority thread.
   *
   * @param scale its size as a share of the camera's, across and down, from 5% to 100%: 50% is
   *     half, 800 x 652 of a 1600 x 1304 camera
   * @param frameRate pictures a second while someone watches, at least 1; one faster than the
   *     camera's own runs at the camera's
   * @param quality JPEG quality, 1 to 100: higher is sharper, and more bytes a picture
   */
  @ConfigRecord
  public record Stream(Dimensionless scale, Frequency frameRate, int quality) {
    public Stream {
      Checks.on(Stream.class)
          .field("scale", scale, between(Percent.of(5), Percent.of(100)))
          .field("frameRate", frameRate, atLeast(Hertz.of(1)))
          .field("quality", quality, between(1, 100))
          .orThrow();
    }

    /** A live view this share of the camera's size, this many a second, at this quality. */
    static Stream of(Dimensionless scale, Frequency frameRate, int quality) {
      return CameraTypeStreamBuilder.builder()
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
   * @param value a number, or for a setting that is on or off, 1 and 0 ({@link #on}, {@link #off})
   */
  public record Control(String name, long value) {
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

    /** A setting that is on or off, turned off: {@code Control.off("white_balance_automatic")}. */
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
}
