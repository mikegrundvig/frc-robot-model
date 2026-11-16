package com.michaelgrundvig.frc.world.field;

import static org.wpilib.units.Units.Microseconds;
import static org.wpilib.units.Units.Seconds;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;
import org.wpilib.units.measure.Time;

/**
 * The camera settings a venue carries for its lighting: an exposure and a gain for each kind of
 * camera, by the camera type's name, and, for a station whose light is its own, that station's. A
 * value the venue leaves out is the camera type's own, from the robot's code. They're chosen in the
 * pit and kept in the venue's file on the robot, so a gain dialed in at an event survives a
 * restart; the file is then downloaded and committed, as a survey is.
 *
 * <p>Each kind of camera has its own because a gain is in a camera's own units: one number across
 * two kinds of camera would mean two different things. A venue may name a type or a station a robot
 * doesn't have: a file shared between robots carries what each of them needs, and each robot uses
 * its own and says what it left unused.
 *
 * @param types what each kind of camera runs at in this venue, by its type's name
 * @param stations what a station's camera runs at here in place of its type's, by the station's
 *     name, each for the type of camera it was kept with: a station holding another kind of camera
 *     since runs at that kind's, since a gain means something else on another kind
 */
public record VenueCameras(Map<String, Settings> types, Map<String, StationSettings> stations) {
  /** A venue with nothing to say of its cameras: each runs at its type's. */
  public static final VenueCameras NONE = new VenueCameras(Map.of(), Map.of());

  /**
   * The shortest exposure a venue may give: one of V4L2's 100 µs steps, the same bound the robot's
   * camera types hold.
   */
  public static final Time SHORTEST_EXPOSURE = Microseconds.of(100);

  /** The longest exposure a venue may give: 1 s, as the robot's camera types hold. */
  public static final Time LONGEST_EXPOSURE = Seconds.of(1);

  /** The most a gain control holds: UVC's is an unsigned 16-bit number. */
  public static final int MOST_GAIN = 65_535;

  public VenueCameras {
    // In their names' order, so a venue's file is written the same way every time.
    types = sorted(types, "a camera type");
    stations = sorted(stations, "a station");
  }

  /**
   * A station's own settings in a venue, for the kind of camera they were kept with.
   *
   * @param type the name of the camera type the station held when they were kept
   * @param settings what a camera of that type runs at in the station
   */
  public record StationSettings(String type, Settings settings) {
    public StationSettings {
      if (type.isBlank()) {
        throw new IllegalArgumentException("a station's camera settings name no camera type");
      }
    }
  }

  /** The settings in their names' order, none of them nameless. */
  private static <V> Map<String, V> sorted(Map<String, V> named, String what) {
    for (String name : named.keySet()) {
      if (name.isBlank()) {
        throw new IllegalArgumentException(
            "a venue's camera settings name " + what + " with no name");
      }
    }
    return Collections.unmodifiableMap(new TreeMap<>(named));
  }

  /**
   * One camera's exposure and gain in a venue, each left out where the venue says nothing.
   *
   * @param exposure a fixed exposure, the camera's automatic exposure off: from 100 µs to 1 s
   * @param gain a value of the camera's {@code gain} control, in its own units: 0 to 65535
   */
  public record Settings(Optional<Time> exposure, Optional<Integer> gain) {
    /** Nothing said: the camera's type's exposure and gain. */
    public static final Settings NONE = new Settings(Optional.empty(), Optional.empty());

    public Settings {
      if (exposure.isPresent()) {
        Time time = exposure.get();
        boolean inRange =
            Double.isFinite(time.in(Seconds))
                && time.gte(SHORTEST_EXPOSURE)
                && time.lte(LONGEST_EXPOSURE);
        if (!inRange) {
          throw new IllegalArgumentException(
              "an exposure of "
                  + time.in(Microseconds)
                  + " us: a venue's exposure is from 100 us to 1 s");
        }
      }
      if (gain.isPresent() && (gain.get() < 0 || gain.get() > MOST_GAIN)) {
        throw new IllegalArgumentException(
            "a gain of " + gain.get() + ": a gain control takes 0 to " + MOST_GAIN);
      }
    }

    /** Whether it says nothing. */
    public boolean empty() {
      return exposure.isEmpty() && gain.isEmpty();
    }

    /** These, with each value the other gives in place of this one's. */
    public Settings overriddenBy(Settings other) {
      return new Settings(other.exposure.or(() -> exposure), other.gain.or(() -> gain));
    }

    /** These, without each value the other gives: what's left once the other is kept elsewhere. */
    public Settings without(Settings other) {
      Optional<Time> keptExposure = exposure;
      if (other.exposure.isPresent()) {
        keptExposure = Optional.empty();
      }
      Optional<Integer> keptGain = gain;
      if (other.gain.isPresent()) {
        keptGain = Optional.empty();
      }
      return new Settings(keptExposure, keptGain);
    }
  }

  /** Whether the venue says nothing of its cameras. */
  public boolean empty() {
    return types.isEmpty() && stations.isEmpty();
  }

  /** The type's settings, where the venue gives that kind of camera any. */
  public Optional<Settings> typesOwn(String type) {
    return Optional.ofNullable(types.get(type));
  }

  /** The station's own settings, where the venue gives it any, with the type they're for. */
  public Optional<StationSettings> stationsOwn(String station) {
    return Optional.ofNullable(stations.get(station));
  }

  /**
   * The station's own settings for a camera of this type: none where it has none, or has them for
   * another kind of camera.
   */
  public Settings stationsOwn(String station, String type) {
    StationSettings own = stations.get(station);
    if (own == null || !own.type().equals(type)) {
      return Settings.NONE;
    }
    return own.settings();
  }

  /**
   * What the venue gives a station's camera of a type: the station's own for that type where it
   * says, else the type's.
   */
  public Settings forCamera(String station, String type) {
    Settings ofType = types.getOrDefault(type, Settings.NONE);
    return ofType.overriddenBy(stationsOwn(station, type));
  }

  /**
   * The same venue, every camera of a type running at these values from now on, kept from one
   * station's tuning: that station gives up its own of the same values, so it runs at them too. The
   * other stations keep their own.
   *
   * @param type the camera type's name
   * @param station the station they were tuned at
   * @param kept the values tuned there: an exposure, a gain, or both
   */
  public VenueCameras keptForType(String type, String station, Settings kept) {
    Map<String, StationSettings> changedStations = new LinkedHashMap<>(stations);
    Settings left = stationsOwn(station, type).without(kept);
    boolean ownForThisType = stationsOwn(station).map(own -> own.type().equals(type)).orElse(false);
    if (ownForThisType && left.empty()) {
      changedStations.remove(station);
    } else if (ownForThisType) {
      changedStations.put(station, new StationSettings(type, left));
    }
    Map<String, Settings> changedTypes = new LinkedHashMap<>(types);
    changedTypes.put(type, types.getOrDefault(type, Settings.NONE).overriddenBy(kept));
    return new VenueCameras(changedTypes, changedStations);
  }

  /**
   * The same venue, one station's camera running at these values from now on, in place of its
   * type's. What the station kept for another kind of camera it held before is dropped.
   *
   * @param station the station
   * @param type the camera type's name, of the camera the station holds
   * @param kept the values tuned there: an exposure, a gain, or both
   */
  public VenueCameras keptForStation(String station, String type, Settings kept) {
    Map<String, StationSettings> changed = new LinkedHashMap<>(stations);
    changed.put(station, new StationSettings(type, stationsOwn(station, type).overriddenBy(kept)));
    return new VenueCameras(types, changed);
  }
}
