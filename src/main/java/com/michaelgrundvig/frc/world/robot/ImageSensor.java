package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.below;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Optional;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Frequency;
import org.wpilib.units.measure.Time;

/**
 * The images a kind of camera gives, as the vision software runs it: {@code ImageSensor.of(1280,
 * 800, Hertz.of(50))}, the size and rate {@code visor cameras} or PhotonVision lists. The rest is
 * optional, each said by its own call: {@code .fieldOfView(Degrees.of(70), Across.HORIZONTAL)},
 * {@code .latency(Milliseconds.of(25))}, {@code .sightRange(Meters.of(6))}. A camera whose lens is
 * calibrated takes its field of view from the lens, and needs none here. It is part of a {@link
 * CameraType}.
 *
 * @param widthPixels width of the images the vision software processes
 * @param heightPixels height of those images
 * @param frameRate images processed per second
 * @param fieldOfView the angle the camera sees across its image: across the width or the diagonal,
 *     as {@code fieldOfViewAcross} says. A camera's datasheet and most teams' code give the
 *     horizontal one; PhotonVision's lens calibration, and its simulated cameras, the diagonal.
 *     None given, the lens's where the camera has one, otherwise {@link
 *     Camera#DEFAULT_FIELD_OF_VIEW}, which the simulator says at startup
 * @param fieldOfViewAcross which way {@code fieldOfView} is across the image: {@link
 *     Across#DIAGONAL} unless given
 * @param latency the average time from taking an image to its result reaching the robot. Images
 *     that take much longer are thrown out, so measure it: the Vision log's {@code LatencySeconds}
 *     for the camera, less the wait for the robot's loop. The simulator's cameras take this long
 *     too. None given, its software's typical one ({@link Camera.Software#typicalLatency}), which
 *     the robot says at startup
 * @param sightRange the farthest a tag is found in most images: farther tags are too small in the
 *     image to read. The simulator's cameras detect tags no farther than this, and AdvantageScope
 *     draws each camera's field of view out to it. None given, {@link Camera#DEFAULT_SIGHT_RANGE},
 *     which the simulator says at startup
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

  /** Which way a camera's field of view is across its image. */
  public enum Across {
    /** Across its width: a camera's datasheet's, a Limelight's. */
    HORIZONTAL,
    /** Across its diagonal: PhotonVision's lens calibration's. */
    DIAGONAL
  }

  /** A camera's images: their size, and how many a second. */
  public static ImageSensor of(int widthPixels, int heightPixels, Frequency frameRate) {
    return ImageSensorBuilder.builder()
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

  private ImageSensorBuilder with() {
    return ImageSensorBuilder.builder(this);
  }

  /**
   * The angle it sees across its image's diagonal, if its field of view is given: as given, or from
   * the horizontal one through a lens with square pixels, whose tangents of half the angle scale as
   * the image's diagonal over its width.
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
