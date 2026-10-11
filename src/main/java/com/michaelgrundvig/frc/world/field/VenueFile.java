package com.michaelgrundvig.frc.world.field;

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

/**
 * Reads a venue file: a small JSON document of ours that embeds WPILib's layout unchanged.
 *
 * <pre>{@code
 * {
 *   "format": 1,
 *   "name": "Practice field, October",
 *   "kind": "practice",                       // practice, event or official
 *   "base": "2026 FRC Rebuilt Welded",        // the official layout it was measured against
 *   "layout": { ... },                        // WPILib's layout, either format, as WPILib writes it
 *   "moved": {"14": "the panel was rehung after the survey"},   // optional
 *   "method": "hand",                                            // optional
 *   "date": "2026-10-12"                                         // optional
 * }
 * }</pre>
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
      Set.of("format", "name", "kind", "base", "layout", "moved", "method", "date");

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
        wrapped,
        venue.text());
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
            wrapped,
            text));
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
      reading.problem("kind \"" + written.get() + "\" (practice, event or official)");
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
