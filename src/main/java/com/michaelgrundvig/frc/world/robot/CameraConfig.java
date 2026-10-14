package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.below;
import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Rule;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.Locale;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
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
 * @param forward how far ahead of the center the lens sits (negative: behind it)
 * @param left how far left of the center (negative: to the right)
 * @param up how far above the floor
 * @param roll rotation about the lens's axis; zero for a camera mounted level
 * @param tiltUp how far the camera is tilted up from level (negative: down)
 * @param yaw which way it faces: zero is straight ahead, positive turns it to the left
 * @param sensor the camera itself, often shared by several cameras on a robot
 * @param noiseScale how much noisier this camera's results are than a typical one's: 1 for typical,
 *     2 to trust it half as much. Fit it from the camera's logged residuals.
 * @param joint the joint whose link it rides on, which its mount is measured from; empty for a
 *     camera on the chassis
 * @param mountTolerance how far the camera may really sit from where the values above say: {@link
 *     MountTolerance#MEASURED} unless given, for a mount measured from CAD and checked
 */
@ConfigRecord
public record CameraConfig(
    String name,
    Distance forward,
    Distance left,
    Distance up,
    Angle roll,
    Angle tiltUp,
    Angle yaw,
    Sensor sensor,
    double noiseScale,
    Optional<Joint> joint,
    @RecordBuilder.Initializer(value = "MEASURED", source = MountTolerance.class)
        MountTolerance mountTolerance) {
  /**
   * The most characters a camera's {@link #id()} may have: Pitwall's World view names what it draws
   * for a camera by it, in ids of at most 64 characters, the longest twelve more ({@code
   * "camera.<id>.view"}, {@code "ray.<id>.<tag>.bad"}).
   */
  public static final int MAX_ID_LENGTH = 52;

  public CameraConfig {
    Checks.on(CameraConfig.class, name)
        .field("name", name, notBlank())
        .field("name", name, Rule.of(CameraConfig::whyIdIsTooLong))
        .field("forward", forward, finite())
        .field("left", left, finite())
        .field("up", up, finite())
        .field("roll", roll, finite())
        .field("tiltUp", tiltUp, finite())
        .field("yaw", yaw, finite())
        .field("noiseScale", noiseScale, positive())
        .orThrow();
  }

  /** Starts building a camera, one named value at a time. */
  public static CameraConfigBuilder.NameStage builder() {
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

  /**
   * Why a name is too long once it is made into an id, if it is. The reason is read after the name
   * of the field it is in: {@code CameraConfig[...].name: is too long as an id ("...", 53
   * characters; at most 52): shorten it}.
   */
  private static Optional<String> whyIdIsTooLong(String name) {
    String id = id(name);
    if (id.length() <= MAX_ID_LENGTH) {
      return Optional.empty();
    }
    return Optional.of(
        "is too long as an id (\""
            + id
            + "\", "
            + id.length()
            + " characters; at most "
            + MAX_ID_LENGTH
            + "): shorten it");
  }

  /** Where the lens sits: from the robot's origin, or from its joint's frame. */
  public Translation3d position() {
    return new Translation3d(forward, left, up);
  }

  /**
   * Where the camera sits and points, in the form WPILib's vision code takes: from the robot's
   * origin, or for a camera on a joint, from the joint's frame ({@code RobotMath.cameraPose} puts
   * it on the robot at the joint's position). WPILib's pitch is positive downward, so the tilt up
   * becomes a negative pitch.
   */
  public Transform3d robotToCamera() {
    return new Transform3d(position(), new Rotation3d(roll, tiltUp.unaryMinus(), yaw));
  }

  /**
   * The camera itself, as the vision software runs it.
   *
   * @param widthPixels width of the images the vision software processes
   * @param heightPixels height of those images
   * @param fieldOfView the angle the camera sees across its image: across the width or the
   *     diagonal, as {@code fieldOfViewAcross} says. A camera's datasheet and most teams' code give
   *     the horizontal one; PhotonVision's lens calibration, and its simulated cameras, the
   *     diagonal
   * @param fieldOfViewAcross which way {@code fieldOfView} is across the image
   * @param frameRate images processed per second
   * @param latency the average time from taking an image to its result reaching the robot. Images
   *     that take much longer are thrown out, so measure it: the Vision log's {@code
   *     LatencySeconds} for the camera, less the wait for the robot's loop. The simulator's cameras
   *     take this long too.
   * @param sightRange the farthest a tag is found in most images: farther tags are too small in the
   *     image to read. The simulator's cameras detect tags no farther than this, and AdvantageScope
   *     draws each camera's field of view out to it.
   */
  @ConfigRecord
  public record Sensor(
      int widthPixels,
      int heightPixels,
      Angle fieldOfView,
      Across fieldOfViewAcross,
      Frequency frameRate,
      Time latency,
      Distance sightRange) {
    public Sensor {
      Checks.on(Sensor.class)
          .field("widthPixels", widthPixels, positive())
          .field("heightPixels", heightPixels, positive())
          .field("fieldOfView", fieldOfView, positive())
          .field("fieldOfView", fieldOfView, below(Degrees.of(180)))
          .field("frameRate", frameRate, positive())
          .field("latency", latency, nonNegative())
          .field("sightRange", sightRange, positive())
          .orThrow();
    }

    /** Starts building a camera sensor, one named value at a time. */
    public static CameraConfigSensorBuilder.WidthPixelsStage builder() {
      return CameraConfigSensorBuilder.builder();
    }

    /**
     * The angle it sees across its image's diagonal, what the simulator's cameras and the views
     * take: given, or from the horizontal one through a lens with square pixels, whose tangents of
     * half the angle scale as the image's diagonal over its width.
     */
    public Angle diagonalFieldOfView() {
      if (fieldOfViewAcross == Across.DIAGONAL) {
        return fieldOfView;
      }
      double diagonal = StrictMath.hypot(widthPixels, heightPixels);
      return Radians.of(
          2
              * StrictMath.atan(
                  StrictMath.tan(fieldOfView.in(Radians) / 2) * diagonal / widthPixels));
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
        new MountTolerance(Millimeters.of(5), Degrees.of(1));

    public MountTolerance {
      Checks.on(MountTolerance.class)
          .field("along", along, nonNegative())
          .field("along", along, finite())
          .field("about", about, nonNegative())
          .field("about", about, below(Degrees.of(90)))
          .orThrow();
    }

    /** Starts building a mount tolerance, one named value at a time. */
    public static CameraConfigMountToleranceBuilder.AlongStage builder() {
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
