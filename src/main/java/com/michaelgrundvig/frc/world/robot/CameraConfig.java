package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Optional;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

/**
 * A camera in its station: what a robot's definition lists of its AprilTag cameras, {@code
 * FRONT_LEFT.holds(CAMERA_1)}. The station gives where the camera sits and points, and its name;
 * the camera gives its kind, its vision program, and on Visor its device and lens. Swapping a
 * camera is changing the camera a station holds: {@code FRONT_LEFT.holds(CAMERA_3)}.
 *
 * <p>Everything that reads the robot's cameras reads them through this, by the station's name: the
 * localizer, the simulator, Visor's configuration, the logs and Pitwall.
 *
 * @param station the place on the robot, and its name
 * @param camera the camera in it
 */
@ConfigRecord
public record CameraConfig(CameraStation station, Camera camera) {
  /** The camera's name everywhere the robot speaks of it: its station's, such as "front-left". */
  public String name() {
    return station.name();
  }

  /** Its station's name as an id ({@link CameraStation#id}). */
  public String id() {
    return station.id();
  }

  /** The kind of camera in the station. */
  public CameraType type() {
    return camera.type();
  }

  /** The images the camera gives: its type's. */
  public ImageSensor sensor() {
    return camera.type().sensor();
  }

  /** The vision program that reads the camera. */
  public Camera.Software software() {
    return camera.software();
  }

  /** The joint the station rides on, if it rides on one. */
  public Optional<Joint> joint() {
    return station.joint();
  }

  /** The station's roll, if it gives one. */
  public Optional<Angle> roll() {
    return station.roll();
  }

  /** The station's noise scale, if it gives one. */
  public Optional<Double> noiseScale() {
    return station.noiseScale();
  }

  /**
   * How far the camera may really sit from where its station says. A tolerance the station gives
   * outright is used whichever camera is in it. A station measured with a camera ({@link
   * CameraStation#measuredWith}) counts as {@link MountTolerance#MEASURED} while it holds that
   * camera, and as {@link MountTolerance#UNCHECKED} once it holds another: a camera put in its
   * place sits as its own housing and the bracket's screws seat it, so the localizer starts as
   * unsure as for a mount nobody checked, and learns the rest from the camera's images.
   */
  public MountTolerance mountTolerance() {
    if (station.measuredWith().isEmpty()) {
      return station.mountTolerance();
    }
    if (measuredWithItsCamera()) {
      return MountTolerance.MEASURED;
    }
    return MountTolerance.UNCHECKED;
  }

  /**
   * Whether the station was measured with another camera than the one it holds now: the mount
   * procedure checked it with a camera since swapped out.
   */
  public boolean measuredWithAnotherCamera() {
    return station.measuredWith().isPresent() && !measuredWithItsCamera();
  }

  private boolean measuredWithItsCamera() {
    return station.measuredWith().orElseThrow() == camera.number();
  }

  /** Its rotation about the lens's axis: as given, or zero, a camera mounted level. */
  public Angle rollOrDefault() {
    return station.rollOrDefault();
  }

  /** How much noisier its results are than a typical camera's: as given, or {@code 1}. */
  public double noiseScaleOrDefault() {
    return station.noiseScaleOrDefault();
  }

  /** The camera's average latency ({@link Camera#latencyOrDefault}). */
  public Time latencyOrDefault() {
    return camera.latencyOrDefault();
  }

  /** The farthest the camera finds a tag in most images ({@link Camera#sightRangeOrDefault}). */
  public Distance sightRangeOrDefault() {
    return camera.sightRangeOrDefault();
  }

  /** The camera's lens calibration, for a camera on Visor that has one. */
  public Optional<Lens> lens() {
    return camera.lens();
  }

  /** The angle the camera sees across its diagonal ({@link Camera#diagonalFieldOfView}). */
  public Angle diagonalFieldOfView() {
    return camera.diagonalFieldOfView();
  }

  /** Where the lens sits ({@link CameraStation#position}). */
  public Translation3d position() {
    return station.position();
  }

  /** Where the camera sits and points ({@link CameraStation#robotToCamera}). */
  public Transform3d robotToCamera() {
    return station.robotToCamera();
  }

  /** Why the station can't be used, if it can't: it isn't placed or pointed. */
  Optional<String> whyNotMounted() {
    return station.whyNotMounted();
  }
}
