package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static org.wpilib.units.Units.Degrees;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.Locale;
import java.util.Optional;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * A place on the robot a camera can go, named for the place: "front-left". Where the place is and
 * which way a camera there points is what turns what the camera sees into where the robot is, so
 * measure it carefully. The station's name is the camera's name everywhere the robot speaks of it:
 * its logs, Visor's configuration and the frames on the wire, PhotonVision's camera, Pitwall's
 * card. Which camera is in it is the robot's to say ({@link #holds}), so a camera swapped for
 * another keeps its station's place and name.
 *
 * <p>Positions are from the robot's origin, the frame's center on the floor, or, for a station that
 * rides a mechanism, from its joint's frame, moving with it. Angles apply in the order yaw, tilt,
 * roll, each about the camera's own axes.
 *
 * <p>A station starts from its name ({@link #named}), and says the rest with a verb each: {@code
 * CameraStation.named("front-left").at(Inches.of(10.5), Inches.of(12), Inches.of(21))
 * .facing(Degrees.of(10), Degrees.of(0)).roll(Degrees.of(-2)).measuredWith(1)}. A robot with a
 * station that isn't placed ({@link #at}) or pointed ({@link #facing}) is refused at startup.
 *
 * @param name unique on the robot, even as an id ({@link #id()}): a place, such as "front-left"
 * @param forward how far ahead of the center the lens sits (negative: behind it), given with {@code
 *     left} and {@code up} by {@link #at}
 * @param left how far left of the center (negative: to the right)
 * @param up how far above the floor
 * @param roll rotation about the lens's axis; none given, zero, a camera mounted level, which the
 *     robot says at startup until the mount procedure gives it
 * @param tiltUp how far the camera is tilted up from level (negative: down), given with {@code yaw}
 *     by {@link #facing}
 * @param yaw which way it faces: zero is straight ahead, positive turns it to the left
 * @param noiseScale how much noisier the results from this station are than a typical camera's: 1
 *     for typical, 2 to trust it half as much. Fit it from the station's logged residuals, which
 *     its mount's shake and its view of the field decide as much as the camera. None given, 1
 *     ({@link #DEFAULT_NOISE_SCALE}), which the robot says at startup
 * @param joint the joint whose link it rides on, which its mount is measured from; empty for a
 *     station on the chassis
 * @param mountTolerance how far a camera here may really sit from where the values above say, given
 *     outright: {@link MountTolerance#UNCHECKED} unless given or measured ({@code measuredWith})
 * @param measuredWith the number on the label of the camera the mount procedure checked this
 *     station with: while it holds that camera its mount counts as {@link MountTolerance#MEASURED};
 *     holding another, as unchecked, which the robot says at startup ({@link
 *     CameraConfig#mountTolerance})
 */
@ConfigRecord
public record CameraStation(
    String name,
    Optional<Distance> forward,
    Optional<Distance> left,
    Optional<Distance> up,
    Optional<Angle> roll,
    Optional<Angle> tiltUp,
    Optional<Angle> yaw,
    Optional<Double> noiseScale,
    Optional<Joint> joint,
    @RecordBuilder.Initializer(value = "UNCHECKED", source = MountTolerance.class)
        MountTolerance mountTolerance,
    Optional<Integer> measuredWith) {
  /**
   * How much noisier a station's results are than a typical camera's when it doesn't say: typical.
   */
  public static final double DEFAULT_NOISE_SCALE = 1;

  public CameraStation {
    Checks.on(CameraStation.class, name)
        .field("name", name, notBlank())
        .field("forward", forward, given(finite()))
        .field("left", left, given(finite()))
        .field("up", up, given(finite()))
        .field("roll", roll, given(finite()))
        .field("tiltUp", tiltUp, given(finite()))
        .field("yaw", yaw, given(finite()))
        .field("noiseScale", noiseScale, given(positive()))
        .field("measuredWith", measuredWith, given(positive()))
        .orThrow();
  }

  /**
   * A station by its name, placed and pointed by {@link #at} and {@link #facing} and the rest said
   * by a verb each: {@code CameraStation.named("front-left")}.
   *
   * @param name unique on the robot, even as an id ({@link #id()}): a place, such as "front-left"
   */
  public static CameraStation named(String name) {
    return CameraStationBuilder.builder().name(name).build();
  }

  /** This station holding this camera: what a robot's definition lists ({@code withCameras}). */
  public CameraConfig holds(Camera camera) {
    return CameraConfigBuilder.builder().station(this).camera(camera).build();
  }

  /**
   * The same station, its lens sitting here: from the robot's origin, the frame's center on the
   * floor, or for a station on a joint ({@link #on}), from the joint's frame.
   *
   * @param forward how far ahead of the center the lens sits (negative: behind it)
   * @param left how far left of the center (negative: to the right)
   * @param up how far above the floor
   * @throws IllegalArgumentException if it's placed already
   */
  public CameraStation at(Distance forward, Distance left, Distance up) {
    saidOnce(this.forward.isPresent(), "its place", ".at(forward, left, up)");
    return with().forward(forward).left(left).up(up).build();
  }

  /**
   * The same station, pointing this way. Angles apply in the order yaw, tilt, roll, each about the
   * camera's own axes.
   *
   * @param tiltUp how far the camera is tilted up from level (negative: down)
   * @param yaw which way it faces: zero is straight ahead, positive turns it to the left
   * @throws IllegalArgumentException if it's pointed already
   */
  public CameraStation facing(Angle tiltUp, Angle yaw) {
    saidOnce(this.tiltUp.isPresent(), "its facing", ".facing(tiltUp, yaw)");
    return with().tiltUp(tiltUp).yaw(yaw).build();
  }

  /**
   * The same station, a camera in it turned this far about its lens's axis.
   *
   * @param roll rotation about the lens's axis; left out, a camera mounted level, which the robot
   *     says at startup until the mount procedure gives it
   * @throws IllegalArgumentException if its roll is given already
   */
  public CameraStation roll(Angle roll) {
    saidOnce(this.roll.isPresent(), "its roll", ".roll(angle)");
    return with().roll(roll).build();
  }

  /**
   * The same station, its mount checked by the mount procedure (docs/calibration.md) with this
   * camera in it, by the number on its label: what the procedure's report ends with. While the
   * station holds that camera, its mount counts as {@link MountTolerance#MEASURED}; with another
   * camera in it, the mount counts as unchecked until the procedure checks it again.
   *
   * @throws IllegalArgumentException if its mount's tolerance is given already
   */
  public CameraStation measuredWith(int cameraNumber) {
    saidOnce(mountSaid(), "its mount's tolerance", ".measuredWith(...) or .mountTolerance(...)");
    return with().measuredWith(cameraNumber).build();
  }

  /**
   * The same station, a camera in it sitting within this of where the station says, whichever
   * camera it is.
   *
   * @param mountTolerance how far a camera here may really sit from where the station says
   * @throws IllegalArgumentException if its mount's tolerance is given already
   */
  public CameraStation mountTolerance(MountTolerance mountTolerance) {
    saidOnce(mountSaid(), "its mount's tolerance", ".measuredWith(...) or .mountTolerance(...)");
    return with().mountTolerance(mountTolerance).build();
  }

  /** Whether its mount's tolerance is said already, outright or by a measurement. */
  private boolean mountSaid() {
    return measuredWith.isPresent() || !mountTolerance.equals(MountTolerance.UNCHECKED);
  }

  /**
   * The same station, riding on a joint's link: its mount is then measured from the joint's frame,
   * moving with it.
   *
   * @param joint the joint whose link it rides on
   * @throws IllegalArgumentException if it rides on a joint already
   */
  public CameraStation on(Joint joint) {
    saidOnce(this.joint.isPresent(), "its joint", ".on(joint)");
    return with().joint(joint).build();
  }

  /**
   * The same station, its results this much noisier than a typical camera's: 2 to trust it half as
   * much. Fit it from the station's logged residuals.
   *
   * @param noiseScale how much noisier its results are than a typical camera's: 1 for typical
   * @throws IllegalArgumentException if its noise scale is given already
   */
  public CameraStation noiseScale(double noiseScale) {
    saidOnce(this.noiseScale.isPresent(), "its noise scale", ".noiseScale(scale)");
    return with().noiseScale(noiseScale).build();
  }

  private CameraStationBuilder with() {
    return CameraStationBuilder.builder(this);
  }

  /** The rule that a station says each part of it once. */
  private void saidOnce(boolean alreadySaid, String has, String call) {
    if (alreadySaid) {
      throw new IllegalArgumentException(
          "CameraStation[" + name + "] has " + has + " already: give it once, in one " + call);
    }
  }

  /**
   * Why a station can't be used, if it can't: it isn't placed ({@link #at}) or pointed ({@link
   * #facing}). The robot's definition refuses it, with the call to write.
   */
  Optional<String> whyNotMounted() {
    if (forward.isEmpty() || left.isEmpty() || up.isEmpty()) {
      return Optional.of(
          "CameraStation[" + name + "] has no place: give it one, .at(forward, left, up)");
    }
    if (tiltUp.isEmpty() || yaw.isEmpty()) {
      return Optional.of(
          "CameraStation[" + name + "] has no facing: give it one, .facing(tiltUp, yaw)");
    }
    return Optional.empty();
  }

  /**
   * Its name as an id, as tools that can't take any text name it (Pitwall's World view): lowercase
   * letters, digits, dots, hyphens and underscores, everything else a hyphen ("Front Left" is
   * "front-left"), starting with a letter or digit. Unique on the robot, as its name is.
   */
  public String id() {
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
   * Where the lens sits: from the robot's origin, or from its joint's frame.
   *
   * @throws IllegalStateException if it isn't placed, which a robot's definition refuses
   */
  public Translation3d position() {
    if (forward.isEmpty() || left.isEmpty() || up.isEmpty()) {
      throw new IllegalStateException(
          "CameraStation[" + name + "] has no place: give it one, .at(forward, left, up)");
    }
    return new Translation3d(forward.get(), left.get(), up.get());
  }

  /**
   * Where a camera here sits and points, in the form WPILib's vision code takes: from the robot's
   * origin, or for a station on a joint, from the joint's frame ({@code RobotMath.cameraPose} puts
   * it on the robot at the joint's position). WPILib's pitch is positive downward, so the tilt up
   * becomes a negative pitch.
   */
  public Transform3d robotToCamera() {
    Optional<String> whyNot = whyNotMounted();
    if (whyNot.isPresent()) {
      throw new IllegalStateException(whyNot.get());
    }
    Rotation3d turn = new Rotation3d(rollOrDefault(), tiltUp.get().unaryMinus(), yaw.get());
    return new Transform3d(position(), turn);
  }
}
