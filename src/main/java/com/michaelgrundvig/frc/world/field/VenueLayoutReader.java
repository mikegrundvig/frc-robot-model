package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Reads the WPILib layout a venue file embeds: the field's size and where each tag is.
 *
 * <p>WPILib has written layouts in two formats, and either may be embedded ({@link Format}). They
 * differ only in what they call the list of tags and the field's size. A tag is written the same
 * way in both:
 *
 * <pre>{@code
 * {"ID": 4, "pose": {"translation": {"x": 1, "y": 2, "z": 0.5},
 *                    "rotation": {"quaternion": {"W": 1, "X": 0, "Y": 0, "Z": 0}}}}
 * }</pre>
 *
 * <p>Each method reads one thing and says what is wrong with it when something is. A tag that
 * cannot be read is left out and the reading carries on, so every tag's problems are reported.
 */
final class VenueLayoutReader {
  /**
   * How far a quaternion's length may be from 1 and still be read as a rotation. A rotation's
   * quaternion has length 1 exactly; this leaves room for a survey tool's rounding.
   */
  private static final double UNIT_LENGTH_TOLERANCE = 1e-3;

  /** The two formats WPILib has written a layout in, by the keys each uses. */
  private enum Format {
    /** WPILib's field format. */
    FIELD("field-tags", "field-dimensions"),
    /** WPILib's earlier AprilTag layout, which WPIcal and PhotonVision write. */
    APRIL_TAG_LAYOUT("tags", "field");

    /** The key of the list of tags. */
    private final String tagsKey;

    /** The key of the object that holds the field's length and width. */
    private final String sizeKey;

    Format(String tagsKey, String sizeKey) {
      this.tagsKey = tagsKey;
      this.sizeKey = sizeKey;
    }

    /** The format a layout is in: the field format if it has that format's tags, else the other. */
    static Format of(Map<?, ?> layout) {
      if (layout.containsKey(FIELD.tagsKey)) {
        return FIELD;
      }
      return APRIL_TAG_LAYOUT;
    }
  }

  private final JsonReading reading;

  private VenueLayoutReader(JsonReading reading) {
    this.reading = reading;
  }

  /** Whether a JSON object has a list of tags under either format's key, as a layout does. */
  static boolean hasTags(Map<?, ?> object) {
    return object.containsKey(Format.FIELD.tagsKey)
        || object.containsKey(Format.APRIL_TAG_LAYOUT.tagsKey);
  }

  /**
   * The layout a venue file embeds, as a WPILib field named for the venue.
   *
   * @param value what the file has under "layout", or null if it has nothing
   * @param venueName the venue's name, which the field is given
   * @param reading where the layout's problems are added
   * @return empty if the layout could not be read, with the reasons added to the reading
   */
  static Optional<Field> read(@Nullable Object value, String venueName, JsonReading reading) {
    return new VenueLayoutReader(reading).layout(value, venueName);
  }

  private Optional<Field> layout(@Nullable Object value, String venueName) {
    if (value == null) {
      reading.problem("no layout");
      return Optional.empty();
    }
    Optional<Map<?, ?>> layout = reading.object(value, "layout");
    if (layout.isEmpty()) {
      return Optional.empty();
    }
    Format format = Format.of(layout.get());
    Optional<Map<?, ?>> size = reading.object(layout.get().get(format.sizeKey), "layout's size");
    Optional<List<FieldTag>> tags = tags(layout.get().get(format.tagsKey));
    if (size.isEmpty() || tags.isEmpty()) {
      return Optional.empty();
    }
    OptionalDouble length = reading.number(size.get(), "length", "the layout's length");
    OptionalDouble width = reading.number(size.get(), "width", "the layout's width");
    if (length.isEmpty() || width.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(
        new Field(
            venueName,
            textIfGiven(layout.get(), "season").orElse(""),
            textIfGiven(layout.get(), "game").orElse(""),
            null,
            length.getAsDouble(),
            width.getAsDouble(),
            textIfGiven(layout.get(), "program").orElse("frc"),
            tags.get()));
  }

  /**
   * Text WPILib writes into a layout about itself, which nothing here depends on. It is taken if it
   * is there and is text, and is no problem otherwise.
   */
  private static Optional<String> textIfGiven(Map<?, ?> layout, String key) {
    if (layout.get(key) instanceof String text) {
      return Optional.of(text);
    }
    return Optional.empty();
  }

  /**
   * The layout's tags. A tag that is listed twice is a problem, and is kept both times so that what
   * is returned is what the file says.
   *
   * @return empty if the layout has no list of tags at all
   */
  private Optional<List<FieldTag>> tags(@Nullable Object value) {
    if (!(value instanceof List<?> items)) {
      reading.problem("the layout has no list of tags (\"field-tags\" or \"tags\")");
      return Optional.empty();
    }
    List<FieldTag> tags = new ArrayList<>();
    Set<Integer> idsSeen = new HashSet<>();
    for (Object item : items) {
      Optional<FieldTag> tag = tag(item);
      if (tag.isEmpty()) {
        continue;
      }
      boolean listedBefore = !idsSeen.add(tag.get().getID());
      if (listedBefore) {
        reading.problem("the layout has tag " + tag.get().getID() + " twice");
      }
      tags.add(tag.get());
    }
    return Optional.of(tags);
  }

  /** One tag: its ID, and its pose as a position and a quaternion. */
  private Optional<FieldTag> tag(@Nullable Object item) {
    Optional<Map<?, ?>> tag = reading.object(item, "a tag in the layout");
    if (tag.isEmpty()) {
      return Optional.empty();
    }
    OptionalDouble id = reading.number(tag.get(), "ID", "a tag's ID");
    Optional<Map<?, ?>> pose =
        reading.object(tag.get().get("pose"), "tag " + shown(id) + "'s pose");
    if (id.isEmpty() || pose.isEmpty()) {
      return Optional.empty();
    }
    // From here on the tag is named by its ID in every problem: "tag 4's translation".
    String owner = possessive("tag " + shown(id));
    Optional<Map<?, ?>> translation =
        reading.object(pose.get().get("translation"), owner + " translation");
    Optional<Map<?, ?>> quaternion = quaternionObject(pose.get(), owner);
    if (translation.isEmpty() || quaternion.isEmpty()) {
      return Optional.empty();
    }
    Optional<Translation3d> position = position(translation.get(), owner);
    Optional<Quaternion> turn = quaternion(quaternion.get(), owner);
    if (position.isEmpty() || turn.isEmpty()) {
      return Optional.empty();
    }
    OptionalInt wholeId = wholeId(id.getAsDouble());
    if (wholeId.isEmpty() || !isARotation(turn.get(), owner)) {
      return Optional.empty();
    }
    Pose3d where = new Pose3d(position.get(), new Rotation3d(turn.get()));
    return Optional.of(new FieldTag(wholeId.getAsInt(), where));
  }

  /**
   * A tag's ID as a problem about the tag shows it: cut down to a whole number, or "?" if the tag
   * has none.
   */
  private static String shown(OptionalDouble id) {
    if (id.isEmpty()) {
      return "?";
    }
    return String.valueOf((int) id.getAsDouble());
  }

  /**
   * The object that holds a tag's quaternion, which is inside its pose's "rotation".
   *
   * @param owner the tag as a problem names what is its: "tag 4's"
   */
  private Optional<Map<?, ?>> quaternionObject(Map<?, ?> pose, String owner) {
    Optional<Map<?, ?>> rotation = reading.object(pose.get("rotation"), owner + " rotation");
    if (rotation.isEmpty()) {
      return Optional.empty();
    }
    return reading.object(rotation.get().get("quaternion"), owner + " quaternion");
  }

  /** Where a tag is, in meters, in WPILib's field frame. All three numbers are read. */
  private Optional<Translation3d> position(Map<?, ?> translation, String owner) {
    OptionalDouble x = reading.number(translation, "x", owner + " x");
    OptionalDouble y = reading.number(translation, "y", owner + " y");
    OptionalDouble z = reading.number(translation, "z", owner + " z");
    if (x.isEmpty() || y.isEmpty() || z.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(new Translation3d(x.getAsDouble(), y.getAsDouble(), z.getAsDouble()));
  }

  /**
   * The four numbers of a tag's quaternion, as they are written. All four are read. Whether they
   * make a rotation is {@link #isARotation}'s to say.
   */
  private Optional<Quaternion> quaternion(Map<?, ?> quaternion, String owner) {
    OptionalDouble w = reading.number(quaternion, "W", owner + " quaternion W");
    OptionalDouble x = reading.number(quaternion, "X", owner + " quaternion X");
    OptionalDouble y = reading.number(quaternion, "Y", owner + " quaternion Y");
    OptionalDouble z = reading.number(quaternion, "Z", owner + " quaternion Z");
    if (w.isEmpty() || x.isEmpty() || y.isEmpty() || z.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(
        new Quaternion(w.getAsDouble(), x.getAsDouble(), y.getAsDouble(), z.getAsDouble()));
  }

  /** A tag's ID as the whole number it has to be: 3, never 3.5 or -1. */
  private OptionalInt wholeId(double id) {
    boolean whole = id == Math.rint(id);
    if (whole && id >= 0) {
      return OptionalInt.of((int) id);
    }
    reading.problem("a tag's ID " + id + " isn't a whole number");
    return OptionalInt.empty();
  }

  /** Whether a quaternion is a rotation: whether its length is 1, near enough. */
  private boolean isARotation(Quaternion quaternion, String owner) {
    double w = quaternion.getW();
    double x = quaternion.getX();
    double y = quaternion.getY();
    double z = quaternion.getZ();
    double length = Math.sqrt(w * w + x * x + y * y + z * z);
    if (Math.abs(length - 1) > UNIT_LENGTH_TOLERANCE) {
      reading.problem(owner + " quaternion isn't a rotation (length " + length + ")");
      return false;
    }
    return true;
  }
}
