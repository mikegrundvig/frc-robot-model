package com.michaelgrundvig.frc.world.field;

import static org.wpilib.units.Units.Milliseconds;

import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.Set;
import org.wpilib.fields.Field;
import org.wpilib.units.measure.Time;

/**
 * Reads a venue file: a small JSON document of ours that embeds WPILib's layout unchanged.
 *
 * <pre>{@code
 * {
 *   "format": 1,
 *   "name": "Practice field, October",
 *   "kind": "practice",                       // practice, shop, event or official
 *   "base": "2026 FRC Rebuilt Welded",        // the official layout it was measured against
 *   "layout": { ... },                        // WPILib's layout, either format, as WPILib writes it
 *   "moved": {"14": "the panel was rehung after the survey"},   // optional
 *   "method": "hand",                                            // optional
 *   "date": "2026-10-12",                                        // optional
 *   "cameras": {                                                 // optional
 *     "types": {"ThriftyCam": {"exposureMs": 1.5, "gain": 40}},  // by camera type, optional
 *     "stations": {"back": {"type": "ThriftyCam", "gain": 52}}  // a station's own, optional
 *   }
 * }
 * }</pre>
 *
 * <p>The cameras' settings are kept by the pit after tuning ({@link #withCameras}); a bare WPILib
 * layout has none, and its cameras run at their types' settings.
 *
 * <p>The embedded layout is either WPILib's field format ({@code field-tags}, {@code
 * field-dimensions}) or its earlier AprilTag layout ({@code tags}, {@code field}), which WPIcal and
 * PhotonVision use; {@link VenueLayoutReader} reads it. A key the format doesn't have is a problem,
 * so a misspelling can't pass silently. Every problem is reported at once. Whether the venue fits
 * the game's field is {@link VenueCheck}'s to say.
 *
 * <p>An instance reads one file's top-level keys, one method to a key, and exists only for the
 * length of that reading.
 */
public final class VenueFile {
  /** The keys a venue file may have at its top level. */
  private static final Set<String> KEYS =
      Set.of("format", "name", "kind", "base", "layout", "moved", "method", "date", "cameras");

  /** The keys a venue's camera settings may have. */
  private static final Set<String> CAMERA_KEYS = Set.of("types", "stations");

  /** The keys a camera type's settings may have. */
  private static final Set<String> SETTING_KEYS = Set.of("exposureMs", "gain");

  /** The keys a station's settings may have: a type's, and the type they were kept for. */
  private static final Set<String> STATION_KEYS = Set.of("type", "exposureMs", "gain");

  /** Any JSON value, written as the file's text: indented, its objects in their own order. */
  private static final JsonType<Object> JSON_VALUE = Jsonb.instance().type(Object.class);

  /**
   * Deeper nesting than this is refused before it's read: a field layout is five levels deep, and
   * the reader recurses for each, so an upload nested thousands deep would overflow its stack.
   */
  static final int MAX_DEPTH = 64;

  /** Text as a JSON string, with its quotes and escapes. */
  private static final JsonType<String> JSON_TEXT = Jsonb.instance().type(String.class);

  /** The file's top-level object. */
  private final Map<?, ?> file;

  private final JsonReading reading = new JsonReading();

  private VenueFile(Map<?, ?> file) {
    this.file = file;
  }

  /**
   * The venue a file describes. Whether it fits a field is {@link VenueCheck}'s to say.
   *
   * @throws IllegalArgumentException listing every problem, if the file isn't a venue file
   */
  public static Venue read(String text) {
    if (!(VenueJson.parse(text) instanceof Map<?, ?> file)) {
      throw new IllegalArgumentException("venue file: not a JSON object");
    }
    if (isBareLayout(file)) {
      throw new IllegalArgumentException(
          "venue file: this is a bare WPILib layout, not a venue file. Wrap it with a name, kind"
              + " and base first (Pitwall's upload does)");
    }
    VenueFile reader = new VenueFile(file);
    Optional<Venue> venue = reader.venue(text);
    if (venue.isEmpty() || reader.reading.foundProblems()) {
      throw new IllegalArgumentException("venue file: " + reader.reading.problemsListed());
    }
    return venue.get();
  }

  /**
   * A venue file around a bare WPILib layout (WPIcal's output, say), with method "unknown". The
   * layout goes in as it is.
   *
   * @param name what to call it; blank for the layout's own name, or "Uploaded layout"
   * @throws IllegalArgumentException if it isn't a WPILib layout, or doesn't make a venue file
   */
  public static Venue wrap(String layout, String name, Venue.Kind kind, String base) {
    if (!(VenueJson.parse(layout) instanceof Map<?, ?> parsed)
        || !VenueLayoutReader.hasTags(parsed)) {
      throw new IllegalArgumentException(
          "not a WPILib layout: it has no \"field-tags\" or \"tags\"");
    }
    String text =
        "{\n  \"format\": "
            + Venue.FORMAT
            + ",\n  \"name\": "
            + JSON_TEXT.toJson(nameToWrapUnder(name, parsed))
            + ",\n  \"kind\": "
            + JSON_TEXT.toJson(kind.name().toLowerCase(Locale.ROOT))
            + ",\n  \"base\": "
            + JSON_TEXT.toJson(base)
            + ",\n  \"method\": \"unknown\",\n  \"layout\": "
            + layout.strip()
            + "\n}\n";
    Venue venue = read(text);
    boolean wrapped = true;
    return new Venue(
        venue.name(),
        venue.kind(),
        venue.base(),
        venue.layout(),
        venue.moved(),
        venue.method(),
        venue.date(),
        venue.cameras(),
        wrapped,
        venue.text());
  }

  /**
   * A venue with no survey: the official layout as it is, named for an event, for the pit to keep
   * its cameras' settings in. It is an official venue, its tags where the official layout puts
   * them, so it fits the field wherever the official layout does.
   *
   * @param name what to call it: the event's name
   * @param official the official layout, as the game gives it
   * @throws IllegalArgumentException if the name is blank
   */
  public static Venue official(String name, Field official) {
    if (name.isBlank()) {
      throw new IllegalArgumentException("a venue needs a name: give the event's");
    }
    Map<String, Object> file = new LinkedHashMap<>();
    file.put("format", Venue.FORMAT);
    file.put("name", name.strip());
    file.put("kind", Venue.Kind.OFFICIAL.name().toLowerCase(Locale.ROOT));
    file.put("base", official.getName());
    file.put("method", "official");
    file.put("layout", VenueJson.parse(layoutText(official)));
    return read(text(file));
  }

  /**
   * The official layout's text: WPILib's own file where it kept it, else WPILib's writing of it.
   */
  private static String layoutText(Field official) {
    String json = official.getJson();
    if (json != null) {
      return json;
    }
    return Jsonb.instance().type(Field.class).toJson(official);
  }

  /**
   * The same venue, its cameras' settings these: its file as it was, the {@code "cameras"} part
   * written anew, or left out when there are none. The rest of the file keeps its values and their
   * order, so its survey is unchanged.
   *
   * @throws IllegalArgumentException if the venue's file can't be read again, which a venue read
   *     from it can always be
   */
  public static Venue withCameras(Venue venue, VenueCameras cameras) {
    if (!(VenueJson.parse(venue.text()) instanceof Map<?, ?> parsed)) {
      throw new IllegalArgumentException("venue file: not a JSON object");
    }
    Map<String, Object> file = new LinkedHashMap<>();
    for (Map.Entry<?, ?> entry : parsed.entrySet()) {
      file.put(String.valueOf(entry.getKey()), entry.getValue());
    }
    file.remove("cameras");
    if (!cameras.empty()) {
      file.put("cameras", written(cameras));
    }
    Venue kept = read(text(file));
    return new Venue(
        kept.name(),
        kept.kind(),
        kept.base(),
        kept.layout(),
        kept.moved(),
        kept.method(),
        kept.date(),
        kept.cameras(),
        venue.wrapped(),
        kept.text());
  }

  /** A venue file's text from its values, indented, ending in a newline. */
  private static String text(Map<String, Object> file) {
    return JSON_VALUE.toJsonPretty(file) + "\n";
  }

  /** A venue's camera settings as its file has them. */
  private static Map<String, Object> written(VenueCameras cameras) {
    Map<String, Object> written = new LinkedHashMap<>();
    if (!cameras.types().isEmpty()) {
      written.put("types", written(cameras.types()));
    }
    if (!cameras.stations().isEmpty()) {
      Map<String, Object> stations = new LinkedHashMap<>();
      cameras
          .stations()
          .forEach((station, own) -> {
            Map<String, Object> entry = new LinkedHashMap<>();
            entry.put("type", own.type());
            entry.putAll(written(own.settings()));
            stations.put(station, entry);
          });
      written.put("stations", stations);
    }
    return written;
  }

  /** Settings by name, as a venue's file has them. */
  private static Map<String, Object> written(Map<String, VenueCameras.Settings> named) {
    Map<String, Object> written = new LinkedHashMap<>();
    named.forEach((name, settings) -> written.put(name, written(settings)));
    return written;
  }

  /** One camera's settings as a venue's file has them: its exposure in milliseconds, its gain. */
  private static Map<String, Object> written(VenueCameras.Settings settings) {
    Map<String, Object> written = new LinkedHashMap<>();
    settings.exposure().ifPresent(time -> written.put("exposureMs", time.in(Milliseconds)));
    settings.gain().ifPresent(gain -> written.put("gain", gain));
    return written;
  }

  /**
   * What a wrapped layout is called: the name given, or if that is blank the layout's own name, or
   * if it has none "Uploaded layout".
   */
  private static String nameToWrapUnder(String given, Map<?, ?> layout) {
    if (!given.isBlank()) {
      return given;
    }
    if (layout.get("name") instanceof String itsOwn && !itsOwn.isBlank()) {
      return itsOwn;
    }
    return "Uploaded layout";
  }

  /**
   * Whether a file is a bare WPILib layout, which {@link #wrap} makes a venue of, rather than a
   * venue file.
   */
  public static boolean bare(String text) {
    try {
      return VenueJson.parse(text) instanceof Map<?, ?> file && isBareLayout(file);
    } catch (IllegalArgumentException e) {
      return false;
    }
  }

  /**
   * Whether a JSON object is a WPILib layout with nothing around it: it has a layout's tags, and
   * not the "format" every venue file starts with.
   */
  private static boolean isBareLayout(Map<?, ?> object) {
    return !object.containsKey("format") && VenueLayoutReader.hasTags(object);
  }

  /**
   * The venue the file describes. Every key is read, whatever was wrong with the ones before it, so
   * that the reading ends up holding all of the file's problems.
   *
   * @param text the file as it was read, which the venue keeps
   * @return empty if something the venue needs could not be read, with the reason in the reading
   */
  private Optional<Venue> venue(String text) {
    noteUnknownKeys();
    checkFormat();
    Optional<String> name = name();
    Optional<Venue.Kind> kind = kind();
    Optional<String> base = reading.text(file, "base", "base");
    Optional<Field> layout = VenueLayoutReader.read(file.get("layout"), name.orElse(""), reading);
    Map<Integer, String> moved = moved();
    Optional<String> method = reading.textOr(file, "method", "method", "unknown");
    Optional<String> date = reading.textOr(file, "date", "date", "");
    VenueCameras cameras = cameras();

    boolean readAllItNeeds =
        name.isPresent()
            && kind.isPresent()
            && base.isPresent()
            && layout.isPresent()
            && method.isPresent();
    if (!readAllItNeeds) {
      return Optional.empty();
    }
    boolean wrapped = false;
    return Optional.of(
        new Venue(
            name.get(),
            kind.get(),
            base.get(),
            layout.get(),
            moved,
            method.get(),
            date.orElse(""),
            cameras,
            wrapped,
            text));
  }

  /**
   * The venue's camera settings: none if the file leaves them out. A value that can't be read is a
   * problem, and is left out.
   */
  private VenueCameras cameras() {
    if (!file.containsKey("cameras")) {
      return VenueCameras.NONE;
    }
    Optional<Map<?, ?>> cameras = reading.object(file.get("cameras"), "cameras");
    if (cameras.isEmpty()) {
      return VenueCameras.NONE;
    }
    noteUnknownKeys(cameras.get(), CAMERA_KEYS, "cameras");
    Map<String, VenueCameras.Settings> types = new LinkedHashMap<>();
    named(cameras.get(), "types", "camera type", SETTING_KEYS)
        .forEach((name, part) -> types.put(name, settings(part, "cameras: camera type " + name)));
    Map<String, VenueCameras.StationSettings> stations = new LinkedHashMap<>();
    named(cameras.get(), "stations", "station", STATION_KEYS)
        .forEach((name, part) -> {
          String where = "cameras: station " + name;
          if (!part.containsKey("type")) {
            // A station's own are for the kind of camera they were kept with, which it names.
            reading.problem(
                where
                    + " doesn't say the camera type its settings were kept for: add it,"
                    + " \"type\": \"ThriftyCam\" for a station that held a ThriftyCam");
            return;
          }
          Optional<String> type = reading.text(part, "type", where + ": type");
          VenueCameras.Settings own = settings(part, where);
          if (type.isPresent() && !type.get().isBlank()) {
            stations.put(name, new VenueCameras.StationSettings(type.get(), own));
          } else if (type.isPresent()) {
            reading.problem(where + ": type is blank");
          }
        });
    return new VenueCameras(types, stations);
  }

  /**
   * The parts of the venue's cameras by name: each camera type's, or each station's. One that isn't
   * an object, or has a key the format doesn't, is a problem.
   *
   * @param key the part: "types" or "stations"
   * @param what what each is, for the problems: "camera type", "station"
   * @param known the keys each may have
   */
  private Map<String, Map<?, ?>> named(
      Map<?, ?> cameras, String key, String what, Set<String> known) {
    Map<String, Map<?, ?>> named = new LinkedHashMap<>();
    if (!cameras.containsKey(key)) {
      return named;
    }
    Optional<Map<?, ?>> listed = reading.object(cameras.get(key), "cameras' " + key);
    for (Map.Entry<?, ?> entry : listed.orElse(Map.of()).entrySet()) {
      String name = String.valueOf(entry.getKey());
      String where = "cameras: " + what + " " + name;
      if (name.isBlank()) {
        reading.problem("cameras: a " + what + " with no name");
        continue;
      }
      Optional<Map<?, ?>> own = reading.object(entry.getValue(), where);
      if (own.isEmpty()) {
        continue;
      }
      noteUnknownKeys(own.get(), known, where);
      named.put(name, own.get());
    }
    return named;
  }

  /**
   * One camera's settings, from its part of the file: an exposure in milliseconds and a gain, each
   * if given. One out of its range is a problem, and is left out.
   *
   * @param where what the problems call this part: a type's or a station's
   */
  private VenueCameras.Settings settings(Map<?, ?> part, String where) {
    Optional<Time> exposure = Optional.empty();
    if (part.containsKey("exposureMs")) {
      OptionalDouble millis = reading.number(part, "exposureMs", where + ": exposureMs");
      if (millis.isPresent()) {
        Time time = Milliseconds.of(millis.getAsDouble());
        if (time.lt(VenueCameras.SHORTEST_EXPOSURE) || time.gt(VenueCameras.LONGEST_EXPOSURE)) {
          reading.problem(
              where + ": exposureMs " + millis.getAsDouble() + " (from 0.1 to 1000 ms)");
        } else {
          exposure = Optional.of(time);
        }
      }
    }
    Optional<Integer> gain = Optional.empty();
    if (part.containsKey("gain")) {
      OptionalDouble written = reading.number(part, "gain", where + ": gain");
      if (written.isPresent()) {
        double value = written.getAsDouble();
        boolean whole = value == Math.rint(value);
        if (!whole || value < 0 || value > VenueCameras.MOST_GAIN) {
          reading.problem(
              where
                  + ": gain "
                  + value
                  + " (a whole number from 0 to "
                  + VenueCameras.MOST_GAIN
                  + ")");
        } else {
          gain = Optional.of((int) value);
        }
      }
    }
    return new VenueCameras.Settings(exposure, gain);
  }

  /** Each key of a part of the file that the format doesn't have is a problem. */
  private void noteUnknownKeys(Map<?, ?> part, Set<String> known, String where) {
    for (Object key : part.keySet()) {
      if (!known.contains(key)) {
        reading.problem(where + ": unknown key \"" + key + "\"");
      }
    }
  }

  private void noteUnknownKeys() {
    for (Object key : file.keySet()) {
      if (!KEYS.contains(key)) {
        reading.problem("unknown key \"" + key + "\"");
      }
    }
  }

  /** The file has to say which format it is in, and it has to be the one this code reads. */
  private void checkFormat() {
    OptionalDouble format = reading.number(file, "format", "format");
    if (format.isPresent() && format.getAsDouble() != Venue.FORMAT) {
      reading.problem(
          "format " + format.getAsDouble() + " (this code reads format " + Venue.FORMAT + ")");
    }
  }

  private Optional<String> name() {
    Optional<String> name = reading.text(file, "name", "name");
    if (name.isPresent() && name.get().isBlank()) {
      reading.problem("name is blank");
    }
    return name;
  }

  /** What kind of field the venue is, written in lower case: "practice". */
  private Optional<Venue.Kind> kind() {
    Optional<String> written = reading.text(file, "kind", "kind");
    if (written.isEmpty()) {
      return Optional.empty();
    }
    try {
      return Optional.of(Venue.Kind.valueOf(written.get().toUpperCase(Locale.ROOT)));
    } catch (IllegalArgumentException e) {
      reading.problem("kind \"" + written.get() + "\" (practice, shop, event or official)");
      return Optional.empty();
    }
  }

  /**
   * Why each tag that the survey found far from its official place is there, by the tag's number.
   * The file may leave "moved" out. An entry is kept only if both its tag number and its reason
   * could be read.
   */
  private Map<Integer, String> moved() {
    Map<Integer, String> moved = new LinkedHashMap<>();
    if (!file.containsKey("moved")) {
      return moved;
    }
    Optional<Map<?, ?>> entries = reading.object(file.get("moved"), "moved");
    if (entries.isEmpty()) {
      return moved;
    }
    for (Map.Entry<?, ?> entry : entries.get().entrySet()) {
      OptionalInt tag = movedTagNumber(entry.getKey());
      Optional<String> reason = movedReason(entry);
      if (tag.isPresent() && reason.isPresent()) {
        moved.put(tag.getAsInt(), reason.get());
      }
    }
    return moved;
  }

  /** The tag a "moved" entry is about. JSON keys are text, so the number is written as "14". */
  private OptionalInt movedTagNumber(Object key) {
    try {
      int tag = Integer.parseInt(String.valueOf(key));
      if (tag >= 0) {
        return OptionalInt.of(tag);
      }
    } catch (NumberFormatException e) {
      // Not a number at all, which is reported the same way as a negative one.
    }
    reading.problem("moved: \"" + key + "\" isn't a tag number");
    return OptionalInt.empty();
  }

  /** Why a "moved" entry's tag is where it is: some words, not left blank. */
  private Optional<String> movedReason(Map.Entry<?, ?> entry) {
    if (entry.getValue() instanceof String reason && !reason.isBlank()) {
      return Optional.of(reason);
    }
    reading.problem("moved: tag " + entry.getKey() + " needs a reason in words");
    return Optional.empty();
  }
}
