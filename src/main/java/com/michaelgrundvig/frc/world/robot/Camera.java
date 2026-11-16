package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.config.Rule;
import java.util.Locale;
import java.util.Optional;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

/**
 * One physical camera: the number on its label, its kind, and the vision program that reads it. For
 * a camera on Visor, it is also the camera's device, which names the camera itself by its USB
 * serial, and its lens's calibration, said together here once, so a camera's lens can't end up with
 * another camera's device. A robot puts it in a station ({@link CameraStation#holds}); moving it to
 * another station, or swapping it for another camera, changes that one reference.
 *
 * <p>{@code Camera.visor(1, THRIFTYCAM).device("/dev/v4l/by-id/usb-...-video-index0")
 * .lens(LENS_1)}, or {@code Camera.photonVision(1, ARDUCAM)} for a camera PhotonVision reads, which
 * keeps its own device and calibration.
 *
 * @param number the number on the camera's label, from 1: what the pit reads to find it, unique
 *     among the robot's cameras
 * @param type the kind of camera it is, which gives its images and its starting exposure and gain
 * @param software the vision program on its coprocessor, with what that program needs of this
 *     camera
 */
@ConfigRecord
public record Camera(int number, CameraType type, Software software) {
  /**
   * The angle a camera is taken to see across its diagonal when neither its type nor a lens says:
   * 80°, a common AprilTag camera's (an Arducam OV9281's lens). Only the simulator and the views
   * read it.
   */
  public static final Angle DEFAULT_FIELD_OF_VIEW = Degrees.of(80);

  /**
   * How far a camera is taken to find tags when its type doesn't say: 7 m, about where a 6.5 in tag
   * shrinks to 20 pixels across a 1280 x 800 image through an 80° lens. Only the simulator and the
   * views read it.
   */
  public static final Distance DEFAULT_SIGHT_RANGE = Meters.of(7);

  /**
   * How far a field of view given beside a lens may differ from the lens's before it's doubted.
   * Four calibrations of one lens model differ by about a degree, and a datasheet's angle by more.
   */
  private static final Angle FIELD_OF_VIEW_AGREEMENT = Degrees.of(2);

  public Camera {
    Checks.on(Camera.class, String.valueOf(number))
        .field("number", number, positive())
        .field("software", software, Rule.of(given -> whyLensIsAnotherSize(given, type)))
        .orThrow();
  }

  /**
   * A camera Visor reads, by the number on its label and its kind, its device and lens each then a
   * verb: {@code Camera.visor(1, THRIFTYCAM).device(...).lens(LENS_1)}. With neither, enough for
   * the simulator.
   */
  public static Camera visor(int number, CameraType type) {
    return CameraBuilder.builder().number(number).type(type).software(Software.VISOR).build();
  }

  /**
   * A camera PhotonVision reads, by the number on its label and its kind. PhotonVision keeps the
   * camera's device and calibration on its coprocessor, and knows the camera by its station's name.
   */
  public static Camera photonVision(int number, CameraType type) {
    return CameraBuilder.builder()
        .number(number)
        .type(type)
        .software(Software.PHOTONVISION)
        .build();
  }

  /**
   * The same camera, at its V4L2 device: the name {@code visor cameras --names} prints for it. That
   * is its {@code /dev/v4l/by-id/} path for a camera with a USB serial, which names this camera
   * wherever it's plugged in, and its USB port's {@code /dev/v4l/by-path/} path for one without,
   * which names the port and not the camera: {@code
   * .device("/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0")}.
   *
   * @throws IllegalArgumentException for a camera on PhotonVision, which keeps its own device
   */
  public Camera device(String device) {
    return withVisor(visorSoftware("its device").device(device));
  }

  /**
   * The same camera, with its lens calibration, from Visor's {@code tools/calibrate.py}.
   *
   * @throws IllegalArgumentException for a camera on PhotonVision, which keeps its own calibration
   */
  public Camera lens(Lens lens) {
    return withVisor(visorSoftware("its lens").lens(lens));
  }

  /**
   * The same camera, this added to every capture time: the spin test's result.
   *
   * @throws IllegalArgumentException for a camera on PhotonVision
   */
  public Camera timeOffset(Time offset) {
    return withVisor(visorSoftware("its time offset").timeOffset(offset));
  }

  private Software.Visor visorSoftware(String what) {
    if (software instanceof Software.Visor visor) {
      return visor;
    }
    throw new IllegalArgumentException(
        "camera "
            + number
            + " is on PhotonVision, which keeps "
            + what
            + " on its coprocessor: give it there, or make the camera Camera.visor(...)");
  }

  private Camera withVisor(Software.Visor visor) {
    return CameraBuilder.builder(this).software(visor).build();
  }

  /** A lens calibrated at another frame size than the camera's gives wrong poses everywhere. */
  private static Optional<String> whyLensIsAnotherSize(Software software, CameraType type) {
    if (!(software instanceof Software.Visor visor) || visor.lens().isEmpty()) {
      return Optional.empty();
    }
    Lens lens = visor.lens().get();
    ImageSensor sensor = type.sensor();
    if (lens.width() == sensor.widthPixels() && lens.height() == sensor.heightPixels()) {
      return Optional.empty();
    }
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

  /** Its lens's calibration, for a camera on Visor that has one. */
  public Optional<Lens> lens() {
    if (software instanceof Software.Visor visor) {
      return visor.lens();
    }
    return Optional.empty();
  }

  /** Its device, for a camera on Visor that names one. */
  public Optional<String> device() {
    if (software instanceof Software.Visor visor) {
      return visor.device();
    }
    return Optional.empty();
  }

  /**
   * Its USB serial, as its {@code /dev/v4l/by-id/} device names it: the last part of the name
   * before {@code -video-index}, {@code ABC123} in {@code usb-Vendor_Camera_ABC123-video-index0}.
   * Empty for a camera that names no device, or one named by its USB port ({@code by-path/}), which
   * says nothing of the camera itself.
   */
  public Optional<String> serial() {
    Optional<String> device = device();
    if (device.isEmpty() || !device.get().startsWith(Software.Visor.BY_ID)) {
      return Optional.empty();
    }
    String name = device.get().substring(Software.Visor.BY_ID.length());
    int index = name.lastIndexOf("-video-index");
    if (index >= 0) {
      name = name.substring(0, index);
    }
    int underscore = name.lastIndexOf('_');
    if (underscore < 0 || underscore == name.length() - 1) {
      return Optional.empty();
    }
    return Optional.of(name.substring(underscore + 1));
  }

  /**
   * The average time from taking an image to its result reaching the robot: its type's, or until
   * that's measured, its software's typical one ({@link Software#typicalLatency}).
   */
  public Time latencyOrDefault() {
    return type.sensor().latency().orElseGet(software::typicalLatency);
  }

  /** The farthest it finds a tag in most images: its type's, or {@link #DEFAULT_SIGHT_RANGE}. */
  public Distance sightRangeOrDefault() {
    return type.sensor().sightRange().orElse(DEFAULT_SIGHT_RANGE);
  }

  /**
   * The angle it sees across its image's diagonal, what the simulator's cameras and the views take:
   * its lens's, where it has one ({@link Lens#diagonalFieldOfView}), so the calibration is its one
   * owner; otherwise its type's; otherwise {@link #DEFAULT_FIELD_OF_VIEW}.
   */
  public Angle diagonalFieldOfView() {
    Optional<Lens> lens = lens();
    if (lens.isPresent()) {
      return lens.get().diagonalFieldOfView();
    }
    return type.sensor().diagonalFieldOfView().orElse(DEFAULT_FIELD_OF_VIEW);
  }

  /**
   * Why the field of view its type gives is doubtful beside its lens's, if it is: more than 2°
   * apart across the diagonal. The lens's is the one used.
   */
  public Optional<String> whyFieldOfViewDisagreesWithLens() {
    Optional<Angle> given = type.sensor().diagonalFieldOfView();
    Optional<Lens> lens = lens();
    if (given.isEmpty() || lens.isEmpty()) {
      return Optional.empty();
    }
    Angle fromLens = lens.get().diagonalFieldOfView();
    double apartDegrees = Math.abs(given.get().in(Degrees) - fromLens.in(Degrees));
    if (apartDegrees <= FIELD_OF_VIEW_AGREEMENT.in(Degrees)) {
      return Optional.empty();
    }
    return Optional.of(
        String.format(
            Locale.ROOT,
            "camera %d's type, %s, gives a field of view of %.1f° across the diagonal, but its"
                + " lens calibration sees %.1f°: the lens's is used. Leave the type's out, or"
                + " check which is right",
            number,
            type.name(),
            given.get().in(Degrees),
            fromLens.in(Degrees)));
  }

  /**
   * The vision program a camera's coprocessor runs, which decides how the robot reads it: {@link
   * #PHOTONVISION}, or {@link #VISOR} with what Visor needs of this one camera.
   */
  public sealed interface Software permits Software.PhotonVision, Software.Visor {
    /** PhotonVision: results over NetworkTables, through PhotonLib. */
    Software PHOTONVISION = new PhotonVision();

    /**
     * Visor, with nothing said of the camera's device or lens: enough for the simulator. On a
     * robot, the robot configures Visor on whichever coprocessor the camera is plugged into, found
     * by its device ({@link Camera#device(String)}).
     */
    Visor VISOR = CameraSoftwareVisorBuilder.builder().build();

    /**
     * The time from taking an image to its result reaching the robot that a camera on this software
     * is taken to have until its type says ({@link ImageSensor#latency}): a guess, which the robot
     * says at startup.
     */
    Time typicalLatency();

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
     * Visor: frames over UDP, through Visor's robot client. What's here, with the camera's type's
     * exposure, gain, controls and live view, goes into Visor's configuration, which the robot
     * writes for each coprocessor as it starts, from the cameras each one reports plugged in; the
     * simulator needs none of it.
     *
     * @param device the camera's V4L2 device: its {@code /dev/v4l/by-id/} path, which names the
     *     camera itself (its USB serial), so the robot finds which coprocessor it's plugged into,
     *     and the same camera after a replug; needed on a robot
     * @param lens the lens calibration, from Visor's {@code tools/calibrate.py}; without one, Visor
     *     sends the tags' corners and no poses
     * @param timeOffset added to every capture time: the spin test's result; none unless given
     */
    @ConfigRecord
    record Visor(Optional<String> device, Optional<Lens> lens, Optional<Time> timeOffset)
        implements Software {
      /** Where V4L2 names a camera by its USB serial. */
      static final String BY_ID = "/dev/v4l/by-id/";

      public Visor {
        Checks.on(Visor.class).field("device", device, Rule.of(Visor::whyNotADevice)).orThrow();
      }

      Visor device(String device) {
        return CameraSoftwareVisorBuilder.builder(this).device(device).build();
      }

      Visor lens(Lens lens) {
        return CameraSoftwareVisorBuilder.builder(this).lens(lens).build();
      }

      Visor timeOffset(Time offset) {
        return CameraSoftwareVisorBuilder.builder(this).timeOffset(offset).build();
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
                  + " with no USB serial");
        }
        return Optional.empty();
      }
    }
  }
}
