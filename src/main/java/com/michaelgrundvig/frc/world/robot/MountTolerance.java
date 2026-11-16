package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.below;
import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Radians;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * How far a camera may really sit from its station's configured mount: the most its lens may be off
 * along each of the robot's axes, and the most it may be turned about each. Whatever uses the mount
 * allows for that much, and no more: the localizer starts each camera's turn on its mount as unsure
 * as this, and learns the rest from the camera's images.
 *
 * <p>An error drawn evenly within a bound has a third of the bound's square as its variance ({@link
 * #turnStdDevRadians}, {@link #offsetStdDevMeters}).
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
      MountToleranceBuilder.builder().along(Millimeters.of(5)).about(Degrees.of(1)).build();

  /**
   * A mount nobody has checked, what a station has until its definition says: measured with a tape
   * or read off CAD, and never put right by the mount procedure (docs/calibration.md). A camera
   * placed by hand lands within about an inch, and a bracket bent a little turns it a few degrees,
   * so 25 mm and 3°. The localizer starts as unsure of it as that, and learns its turn from its
   * images; the procedure's report ends with {@code .measuredWith(...)}.
   */
  public static final MountTolerance UNCHECKED =
      MountToleranceBuilder.builder().along(Millimeters.of(25)).about(Degrees.of(3)).build();

  public MountTolerance {
    Checks.on(MountTolerance.class)
        .field("along", along, nonNegative())
        .field("along", along, finite())
        .field("about", about, nonNegative())
        .field("about", about, below(Degrees.of(90)))
        .orThrow();
  }

  /**
   * A mount within this much of where its station says it is: {@code
   * MountTolerance.of(Millimeters.of(5), Degrees.of(1))}.
   *
   * @param along the most the lens may sit off its configured place, along each axis
   * @param about the most the camera may be turned off its configured angles, about each axis
   */
  public static MountTolerance of(Distance along, Angle about) {
    return MountToleranceBuilder.builder().along(along).about(about).build();
  }

  /**
   * The standard deviation of a turn drawn evenly within the bound about an axis, in radians: the
   * bound over the square root of three.
   */
  public double turnStdDevRadians() {
    return about.in(Radians) / Math.sqrt(3);
  }

  /**
   * The standard deviation of an offset drawn evenly within the bound along an axis, in meters: the
   * bound over the square root of three.
   */
  public double offsetStdDevMeters() {
    return along.in(Meters) / Math.sqrt(3);
  }
}
