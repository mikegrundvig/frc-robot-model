package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Function;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;

/**
 * The venue file reader's pieces, and the edges of what it accepts, as tables of questions and
 * answers. What a whole file reads as, and what a survey says against the official layout, are in
 * {@link VenueTest}.
 */
class VenueFileEdgesTest {
  @ParameterizedTest
  @MethodSource({
    "valuesRead",
    "nesting",
    "bareLayouts",
    "wrappedNames",
    "filesRead",
    "filesRefused",
    "surveysAtTheirBounds"
  })
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments answers(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  /** What one read of a value gives, and the problems the reading holds after it. */
  private record Read(Object value, String problems) {}

  private static Read reading(Function<JsonReading, Object> read) {
    JsonReading reading = new JsonReading();
    Object value = read.apply(reading);
    return new Read(value, reading.problemsListed());
  }

  /** A JSON object as the parser gives it, which may hold JSON's null. */
  private static Map<String, @Nullable Object> object(String key, @Nullable Object value) {
    Map<String, @Nullable Object> object = new LinkedHashMap<>();
    object.put(key, value);
    return object;
  }

  // spotless:off

  /** One value read from parsed JSON: what it is, or the problem that says why it is not. */
  static Stream<Arguments> valuesRead() {
    return Stream.of(
        answers("a number",
            () -> reading(r -> r.number(object("x", 2.5), "x", "x")),
            new Read(OptionalDouble.of(2.5), "")),
        answers("a whole number is a number",
            () -> reading(r -> r.number(object("x", 3L), "x", "x")),
            new Read(OptionalDouble.of(3), "")),
        answers("a number that is missing, named by its key",
            () -> reading(r -> r.number(Map.of(), "format", "format")),
            new Read(OptionalDouble.empty(), "format is missing")),
        answers("text where a number belongs",
            () -> reading(r -> r.number(object("format", "1"), "format", "format")),
            new Read(OptionalDouble.empty(), "format isn't a number")),
        answers("null where a number belongs is there, and is no number",
            () -> reading(r -> r.number(object("format", null), "format", "format")),
            new Read(OptionalDouble.empty(), "format isn't a number")),
        answers("a number too large for a double is no number",
            () -> reading(r -> r.number(object("x", Double.POSITIVE_INFINITY), "x", "x")),
            new Read(OptionalDouble.empty(), "x isn't a number")),
        answers("a number named by more than its key is named that way, once",
            () -> reading(r -> r.number(Map.of(), "length", "the layout's length")),
            new Read(OptionalDouble.empty(), "the layout's length is missing")),
        answers("...and so is one that is there and is no number",
            () -> reading(r -> r.number(object("x", "east"), "x", "tag 2's x")),
            new Read(OptionalDouble.empty(), "tag 2's x isn't a number")),
        answers("an object",
            () -> reading(r -> r.object(Map.of("a", 1), "layout")),
            new Read(Optional.of(Map.of("a", 1)), "")),
        answers("an object that is missing",
            () -> reading(r -> r.object(null, "tag 4's pose")),
            new Read(Optional.empty(), "tag 4's pose is missing")),
        answers("a list where an object belongs",
            () -> reading(r -> r.object(List.of(), "layout")),
            new Read(Optional.empty(), "layout isn't an object")),
        answers("text",
            () -> reading(r -> r.text(object("name", "Pit"), "name", "name")),
            new Read(Optional.of("Pit"), "")),
        answers("text that is missing",
            () -> reading(r -> r.text(Map.of(), "name", "name")),
            new Read(Optional.empty(), "name is missing")),
        answers("a number where text belongs",
            () -> reading(r -> r.text(object("name", 4), "name", "name")),
            new Read(Optional.empty(), "name isn't text")),
        answers("text that may be left out, left out",
            () -> reading(r -> r.textOr(Map.of(), "method", "method", "unknown")),
            new Read(Optional.of("unknown"), "")),
        answers("text that may be left out, given",
            () -> reading(r -> r.textOr(object("method", "hand"), "method", "method", "unknown")),
            new Read(Optional.of("hand"), "")),
        answers("text that may be left out, given as something else",
            () -> reading(r -> r.textOr(object("method", 7), "method", "method", "unknown")),
            new Read(Optional.empty(), "method isn't text")),
        answers("problems are listed in the order they were found",
            () -> reading(r -> List.of(r.text(Map.of(), "a", "a"), r.text(Map.of(), "b", "b"), r.foundProblems())),
            new Read(List.of(Optional.empty(), Optional.empty(), true), "a is missing; b is missing")),
        answers("a reading that found nothing wrong",
            () -> reading(JsonReading::foundProblems),
            new Read(false, "")));
  }

  private static Object refusalOf(Supplier<Object> action) {
    try {
      return action.get();
    } catch (IllegalArgumentException e) {
      return String.valueOf(e.getMessage());
    }
  }

  /** How deep text nests, and where the reader stops reading it. */
  static Stream<Arguments> nesting() {
    int most = VenueFile.MAX_DEPTH;
    return Stream.of(
        answers("nothing nested", () -> VenueJson.depth("3"), 0),
        answers("a list in an object in a list", () -> VenueJson.depth("[{\"a\": [1]}]"), 3),
        answers("two lists side by side are one deep", () -> VenueJson.depth("[[], []]"), 2),
        answers("brackets in a string are text", () -> VenueJson.depth("[\"[[{{\"]"), 1),
        answers("an escaped quote does not end its string", () -> VenueJson.depth("[\"\\\"[[\"]"), 1),
        answers("an escaped backslash does not escape the quote after it",
            () -> VenueJson.depth("[\"\\\\\", [[1]]]"), 3),
        answers("nesting after a string is counted", () -> VenueJson.depth("{\"a\": \"]\", \"b\": [[1]]}"), 3),
        answers("as deep as is read",
            () -> refusalOf(() -> VenueJson.parse("[".repeat(most) + "]".repeat(most)) instanceof List<?>),
            true),
        answers("one level deeper than is read",
            () -> refusalOf(() -> VenueJson.parse("[".repeat(most + 1) + "]".repeat(most + 1))),
            "not JSON a venue file can be: nested deeper than 64 levels"),
        answers("JSON's null is read as null", () -> VenueJson.parse("null") == null, true));
  }

  // The files below are written out in full, small enough to read: a field 16 by 8 m with one tag.
  private static final String UPRIGHT = "{\"W\": 1, \"X\": 0, \"Y\": 0, \"Z\": 0}";

  private static String tag(String id, String quaternion) {
    return "{\"ID\": " + id + ", \"pose\": {\"translation\": {\"x\": 1, \"y\": 2, \"z\": 0.5},"
        + " \"rotation\": {\"quaternion\": " + quaternion + "}}}";
  }

  private static String layout(String tags) {
    return "{\"tags\": [" + tags + "], \"field\": {\"length\": 16, \"width\": 8}}";
  }

  private static String file(String name, String layout, String more) {
    return "{\"format\": 1, " + name + "\"kind\": \"event\", \"base\": \"Base\", \"layout\": " + layout + more + "}";
  }

  private static String file(String tags) {
    return file("\"name\": \"Pit\", ", layout(tags), "");
  }

  /** A layout whose field has no length, and whose one tag has no x and no quaternion W. */
  private static String layoutMissingNumbers() {
    String tag =
        "{\"ID\": 2, \"pose\": {\"translation\": {\"y\": 2, \"z\": 0.5},"
            + " \"rotation\": {\"quaternion\": {\"X\": 0, \"Y\": 0, \"Z\": 0}}}}";
    return "{\"tags\": [" + tag + "], \"field\": {\"width\": 8}}";
  }

  /** A parsed layout with one upright tag, its ID whatever number a parser might give. */
  private static Map<String, Object> layoutWithTagNumbered(Number id) {
    Map<String, Object> pose =
        Map.of(
            "translation", Map.of("x", 1, "y", 2, "z", 0.5),
            "rotation", Map.of("quaternion", Map.of("W", 1, "X", 0, "Y", 0, "Z", 0)));
    return Map.of(
        "tags", List.of(Map.of("ID", id, "pose", pose)), "field", Map.of("length", 16, "width", 8));
  }

  /** Which files are bare layouts, to be wrapped before they are read. */
  static Stream<Arguments> bareLayouts() {
    return Stream.of(
        answers("a layout in the earlier format", () -> VenueFile.bare(layout(tag("4", UPRIGHT))), true),
        answers("a layout in the field format", () -> VenueFile.bare("{\"field-tags\": []}"), true),
        answers("a venue file", () -> VenueFile.bare(file(tag("4", UPRIGHT))), false),
        answers("an object with no tags", () -> VenueFile.bare("{\"name\": \"x\"}"), false),
        answers("a list", () -> VenueFile.bare("[]"), false),
        answers("text that is not JSON", () -> VenueFile.bare("tags"), false));
  }

  private static String wrappedName(String layout, String given) {
    return VenueFile.wrap(layout, given, Venue.Kind.EVENT, "Base").name();
  }

  /** What a wrapped layout is called. */
  static Stream<Arguments> wrappedNames() {
    String named = "{\"name\": \"Hall B\", \"tags\": [], \"field\": {\"length\": 16, \"width\": 8}}";
    String blankName = named.replace("Hall B", " ");
    return Stream.of(
        answers("the name given", () -> wrappedName(named, "Ours"), "Ours"),
        answers("the layout's own, given none", () -> wrappedName(named, " "), "Hall B"),
        answers("\"Uploaded layout\", when the layout's own is blank", () -> wrappedName(blankName, ""), "Uploaded layout"),
        answers("\"Uploaded layout\", when the layout has none", () -> wrappedName(layout(""), ""), "Uploaded layout"));
  }

  /** What is read from files at the edge of what is accepted. */
  static Stream<Arguments> filesRead() {
    String described =
        "{\"season\": \"2027\", \"game\": \"Test\", \"program\": \"ftc\", \"field-tags\": [],"
            + " \"field-dimensions\": {\"length\": 16, \"width\": 8}}";
    return Stream.of(
        answers("a tag numbered zero",
            () -> VenueFile.read(file(tag("0", UPRIGHT))).layout().getTags().get(0).getID(), 0),
        answers("a tag numbered as high as a whole number goes",
            () -> VenueFile.read(file(tag("2147483647", UPRIGHT))).layout().getTags().get(0).getID(), 2147483647),
        answers("a reason for tag zero",
            () -> VenueFile.read(file("\"name\": \"Pit\", ", layout(tag("0", UPRIGHT)), ", \"moved\": {\"0\": \"why\"}")).moved(),
            Map.of(0, "why")),
        answers("a quaternion of four halves is a rotation",
            () -> VenueFile.read(file(tag("4", "{\"W\": 0.5, \"X\": 0.5, \"Y\": 0.5, \"Z\": 0.5}"))).layout().getTags().size(),
            1),
        answers("what the layout says of itself is kept",
            () -> {
              var layout = VenueFile.read(file("\"name\": \"Pit\", ", described, "")).layout();
              return List.of(layout.season, layout.game, layout.program);
            },
            List.of("2027", "Test", "ftc")),
        answers("...and has defaults when it says nothing",
            () -> {
              var layout = VenueFile.read(file("")).layout();
              return List.of(layout.season, layout.game, layout.program);
            },
            List.of("", "", "frc")),
        answers("no reasons at all", () -> VenueFile.read(file("")).moved(), Map.of()));
  }

  private static Object refusal(String text) {
    return refusalOf(() -> VenueFile.read(text));
  }

  /** What files past those edges are refused for. */
  static Stream<Arguments> filesRefused() {
    return Stream.of(
        answers("no name", () -> refusal(file("", layout(""), "")), "venue file: name is missing"),
        answers("a blank name", () -> refusal(file("\"name\": \" \", ", layout(""), "")), "venue file: name is blank"),
        answers("a tag numbered below zero",
            () -> refusal(file(tag("-1", UPRIGHT))), "venue file: a tag's ID -1.0 isn't a whole number"),
        answers("a tag with no number and no pose",
            () -> refusal(file("{}")), "venue file: a tag's ID is missing; tag ?'s pose is missing"),
        answers("a layout with no length, and a tag with no x and no quaternion W",
            () -> refusal(file("\"name\": \"Pit\", ", layoutMissingNumbers(), "")),
            "venue file: tag 2's x is missing; tag 2's quaternion W is missing; the layout's length is missing"),
        answers("a tag numbered 3.5 is not called tag 3 in what else is wrong with it",
            () -> refusal(file("{\"ID\": 3.5}")),
            "venue file: a tag's ID 3.5 isn't a whole number; tag 3.5's pose is missing"),
        answers("a tag numbered one past the highest whole number is not read as the highest",
            () -> refusal(file(tag("2147483648", UPRIGHT))),
            "venue file: a tag's ID 2.147483648E9 is too large: a tag's ID is a whole number, at most 2147483647"),
        answers("a tag numbered 1e20, which the JSON library hands over as the largest whole number it keeps",
            () -> refusal(file(tag("1e20", UPRIGHT))),
            "venue file: a tag's ID 9.223372036854776E18 is too large: a tag's ID is a whole number, at most 2147483647"),
        answers("a tag numbered 1e20 as a parser that keeps it would hand it over",
            () -> reading(r -> VenueLayoutReader.read(layoutWithTagNumbered(1e20), "Pit", r).orElseThrow().getTags()),
            new Read(List.of(), "a tag's ID 1.0E20 is too large: a tag's ID is a whole number, at most 2147483647")),
        answers("a quaternion of four ones is twice too long",
            () -> refusal(file(tag("4", "{\"W\": 1, \"X\": 1, \"Y\": 1, \"Z\": 1}"))),
            "venue file: tag 4's quaternion isn't a rotation (length 2.0)"),
        answers("a tag that is no rotation, twice, is not also listed twice",
            () -> refusal(file(tag("4", "{\"W\": 3, \"X\": 0, \"Y\": 0, \"Z\": 0}") + ", " + tag("4", "{\"W\": 3, \"X\": 0, \"Y\": 0, \"Z\": 0}"))),
            "venue file: tag 4's quaternion isn't a rotation (length 3.0);"
                + " tag 4's quaternion isn't a rotation (length 3.0)"),
        answers("reasons that are not an object",
            () -> refusal(file("\"name\": \"Pit\", ", layout(""), ", \"moved\": [1]")), "venue file: moved isn't an object"),
        answers("a reason for a tag numbered below zero",
            () -> refusal(file("\"name\": \"Pit\", ", layout(""), ", \"moved\": {\"-1\": \"why\"}")),
            "venue file: moved: \"-1\" isn't a tag number"),
        answers("a blank reason",
            () -> refusal(file("\"name\": \"Pit\", ", layout(""), ", \"moved\": {\"4\": \" \"}")),
            "venue file: moved: tag 4 needs a reason in words"));
  }

  /**
   * A venue with one tag, on a field of some size, against an official layout with that tag
   * somewhere along x on a field 0.01 m long and as wide. The numbers below are chosen so that the differences
   * come out exactly on the bounds: 0.02 - 0.01 is exactly 0.01, 0.2 - 0.1 exactly 0.1, and 0.002 -
   * 0.001 exactly 0.001. Whether the tag still pairs up is left out: it has no partner here.
   */
  private static List<String> problems(
      String kind, String size, double officialX, String x, String quaternion) {
    String tag =
        "{\"ID\": 4, \"pose\": {\"translation\": {\"x\": " + x + ", \"y\": 0, \"z\": 0},"
            + " \"rotation\": {\"quaternion\": " + quaternion + "}}}";
    String text =
        "{\"format\": 1, \"name\": \"Pit\", \"kind\": \"" + kind + "\", \"base\": \"Base\", \"layout\":"
            + " {\"tags\": [" + tag + "], \"field\": {" + size + "}}}";
    FieldTag official = new FieldTag(4, new Pose3d(officialX, 0, 0, Rotation3d.ZERO));
    Field base = new Field("Base", "", "", null, 0.01, 0.01, "frc", List.of(official));
    return VenueCheck.problems(VenueFile.read(text), base, Symmetry.MIRRORED);
  }

  /** A survey exactly on a bound is within it; a step past it is not. */
  static Stream<Arguments> surveysAtTheirBounds() {
    // A tenth of a turn about z: 36 degrees.
    String turned = "{\"W\": " + Math.cos(Math.PI / 10) + ", \"X\": 0, \"Y\": 0, \"Z\": " + Math.sin(Math.PI / 10) + "}";
    String sameSize = "\"length\": 0.01, \"width\": 0.01";
    return Stream.of(
        answers("a field exactly a centimeter longer is the same size",
            () -> problems("event", "\"length\": 0.02, \"width\": 0.01", 0.1, "0.1", UPRIGHT), List.of()),
        answers("a field exactly a centimeter wider is the same size",
            () -> problems("event", "\"length\": 0.01, \"width\": 0.02", 0.1, "0.1", UPRIGHT), List.of()),
        answers("a field more than a centimeter longer is not",
            () -> problems("event", "\"length\": 0.021, \"width\": 0.01", 0.1, "0.1", UPRIGHT),
            List.of("the field is 0.021 x 0.010 m, the official one 0.010 x 0.010 m")),
        answers("a field more than a centimeter wider is not",
            () -> problems("event", "\"length\": 0.01, \"width\": 0.021", 0.1, "0.1", UPRIGHT),
            List.of("the field is 0.010 x 0.021 m, the official one 0.010 x 0.010 m")),
        answers("a tag exactly 10 cm off needs no reason",
            () -> problems("event", sameSize, 0.1, "0.2", UPRIGHT), List.of()),
        answers("a tag a hair more than 10 cm off needs one",
            () -> problems("event", sameSize, 0.1, "0.2000001", UPRIGHT),
            List.of("tag 4 sits 0.100 m and 0.0° from its official place: if that's right, say why under \"moved\"")),
        answers("a tag turned far from its official way needs a reason",
            () -> problems("event", sameSize, 0.1, "0.1", turned),
            List.of("tag 4 sits 0.000 m and 36.0° from its official place: if that's right, say why under \"moved\"")),
        answers("an official venue's tag exactly a millimeter off matches",
            () -> problems("official", sameSize, 0.001, "0.002", UPRIGHT), List.of()),
        answers("an official venue's tag a hair more than a millimeter off does not",
            () -> problems("official", sameSize, 0.001, "0.0020001", UPRIGHT),
            List.of("an official venue, but tag 4 sits 1.0 mm and 0.00° from WPILib's layout")),
        answers("an official venue's tag turned is reported",
            () -> problems("official", sameSize, 0.1, "0.1", turned),
            List.of("an official venue, but tag 4 sits 0.0 mm and 36.00° from WPILib's layout")));
  }

  // spotless:on
}
