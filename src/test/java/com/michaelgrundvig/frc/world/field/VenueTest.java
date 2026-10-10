package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.tag;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;

class VenueTest {
  private static final String BASE = "Test field";

  /** A venue file over the test field's tags, in WPILib's field format or its earlier one. */
  private static String file(
      String kind, String base, List<FieldTag> tags, boolean fieldFormat, String more) {
    StringBuilder list = new StringBuilder();
    for (FieldTag tag : tags) {
      Pose3d pose = tag.getPose();
      var q = pose.getRotation().getQuaternion();
      list.append(list.isEmpty() ? "" : ",")
          .append(
              String.format(
                  Locale.ROOT,
                  "{\"ID\": %d, \"pose\": {\"translation\": {\"x\": %s, \"y\": %s, \"z\": %s},"
                      + " \"rotation\": {\"quaternion\": {\"W\": %s, \"X\": %s, \"Y\": %s,"
                      + " \"Z\": %s}}}}",
                  tag.getID(),
                  pose.getX(),
                  pose.getY(),
                  pose.getZ(),
                  q.getW(),
                  q.getX(),
                  q.getY(),
                  q.getZ()));
    }
    String layout =
        fieldFormat
            ? "{\"name\": \"Test field\", \"season\": \"2027\", \"game\": \"Test\","
                + " \"field-dimensions\": {\"length\": 16, \"width\": 8}, \"program\": \"frc\","
                + " \"field-tags\": ["
                + list
                + "]}"
            : "{\"tags\": [" + list + "], \"field\": {\"length\": 16, \"width\": 8}}";
    return "{\"format\": 1, \"name\": \"Practice field, October\", \"kind\": \""
        + kind
        + "\", \"base\": \""
        + base
        + "\", \"layout\": "
        + layout
        + more
        + "}";
  }

  private static String file(List<FieldTag> tags, String more) {
    return file("practice", BASE, tags, true, more);
  }

  /** The test field's tags, with one moved by (dx, dy). */
  private static List<FieldTag> moved(int id, double dx, double dy) {
    List<FieldTag> tags = new ArrayList<>();
    for (FieldTag tag : TestFields.tags()) {
      Pose3d pose = tag.getPose();
      tags.add(
          tag.getID() == id
              ? new FieldTag(
                  id,
                  new Pose3d(pose.getX() + dx, pose.getY() + dy, pose.getZ(), pose.getRotation()))
              : tag);
    }
    return tags;
  }

  private static List<String> problems(String text) {
    return VenueCheck.problems(VenueFile.read(text), TestFields.layout(), Symmetry.ROTATED);
  }

  /** A venue's layout on its own, as WPIcal writes one: the earlier format, bare. */
  private static String bareLayout() {
    String bare = file("event", BASE, TestFields.tags(), false, "");
    return bare.substring(bare.indexOf("\"layout\": ") + 10, bare.length() - 1);
  }

  /** The test field's tags with tag 4 given twice. */
  private static List<FieldTag> fourTwice() {
    List<FieldTag> tags = new ArrayList<>(TestFields.tags());
    tags.add(tag(4, 0, 2, 0));
    return tags;
  }

  static Stream<Arguments> formats() {
    return Stream.of(
        argumentSet("WPILib's field format", file(TestFields.tags(), ""), Venue.Kind.PRACTICE),
        argumentSet(
            "the earlier AprilTag format",
            file("event", BASE, TestFields.tags(), false, ""),
            Venue.Kind.EVENT));
  }

  @ParameterizedTest
  @MethodSource("formats")
  void aVenueReadsWhicheverFormatItsLayoutIsIn(String text, Venue.Kind kind) {
    Venue venue = VenueFile.read(text);

    assertThat(venue.name()).isEqualTo("Practice field, October");
    assertThat(venue.kind()).isEqualTo(kind);
    assertThat(venue.base()).isEqualTo(BASE);
    assertThat(venue.layout().getTags()).hasSize(6);
    assertThat(venue.layout().getFieldLength()).isEqualTo(16);
    assertThat(venue.layout().getTagPose(14).orElseThrow().getY()).isCloseTo(6, within(1e-12));
    assertThat(venue.method()).isEqualTo("unknown");
    assertThat(venue.wrapped()).isFalse();
    assertThat(venue.hash()).hasSize(64).matches("[0-9a-f]+");
  }

  @Test
  void readsReasonsAndProvenance() {
    Venue venue =
        VenueFile.read(
            file(
                TestFields.tags(),
                ", \"moved\": {\"4\": \"rehung after the survey\"},"
                    + " \"method\": \"hand\", \"date\": \"2026-10-12\""));
    assertThat(venue.moved()).containsEntry(4, "rehung after the survey");
    assertThat(venue.method()).isEqualTo("hand");
    assertThat(venue.date()).isEqualTo("2026-10-12");
  }

  /** A bare layout is wrapped with stated defaults; a name with JSON's escapes reads back. */
  @ParameterizedTest
  @ValueSource(strings = {"Event A", "Event \"A\" \\ hall\tB\nfield"})
  void aBareLayoutIsWrappedAndItsNameReadsBackAsItWas(String name) {
    Venue venue = VenueFile.wrap(bareLayout(), name, Venue.Kind.EVENT, BASE);

    assertThat(venue.wrapped()).isTrue();
    assertThat(venue.name()).isEqualTo(name);
    assertThat(venue.method()).isEqualTo("unknown");
    assertThat(venue.layout().getTags()).hasSize(6);
    assertThat(VenueFile.read(venue.text()).name()).as("its text reads back").isEqualTo(name);
  }

  private static final String HEAD =
      "{\"format\": 1, \"name\": \"n\", \"kind\": \"practice\", \"base\": \"" + BASE + "\"";

  /** Tag 2, turned half a turn, has W, X and Y 0 and Z 1: with W at 2, it's √5 long. */
  private static final String STRETCHED =
      "tag 2's quaternion isn't a rotation (length " + Math.sqrt(5) + ")";

  /**
   * What makes a file no venue file at all, and every problem its tags and reasons have at once.
   */
  static Stream<Arguments> malformed() {
    String good = file(TestFields.tags(), "");
    String four = ", \"moved\": {\"four\": \"rehung\"}";
    return Stream.of(
        argumentSet("not an object", "[1, 2]", "not a JSON object"),
        argumentSet("no layout", HEAD + "}", "no layout"),
        argumentSet(
            "a layout with no tags",
            HEAD + ", \"layout\": {\"field\": {\"length\": 16, \"width\": 8}}}",
            "the layout has no list of tags (\"field-tags\" or \"tags\")"),
        argumentSet("a tag twice", file(fourTwice(), ""), "the layout has tag 4 twice"),
        argumentSet(
            "an ID that is no whole number",
            good.replace("{\"ID\": 3, ", "{\"ID\": 3.5, "),
            "a tag's ID 3.5 isn't a whole number"),
        argumentSet(
            "a quaternion that is no rotation",
            good.replaceFirst("\"W\": [^,]+", "\"W\": 2"),
            STRETCHED),
        argumentSet(
            "a reason for no tag number",
            file(TestFields.tags(), four),
            "moved: \"four\" isn't a tag number"),
        argumentSet(
            "all of those at once",
            file(fourTwice(), four)
                .replace("{\"ID\": 3, ", "{\"ID\": 3.5, ")
                .replaceFirst("\"W\": [^,]+", "\"W\": 2"),
            STRETCHED
                + "; a tag's ID 3.5 isn't a whole number; the layout has tag 4 twice;"
                + " moved: \"four\" isn't a tag number"));
  }

  @ParameterizedTest
  @MethodSource("malformed")
  void aMalformedVenueFileIsRefusedSayingWhatsWrong(String text, String problem) {
    assertThatThrownBy(() -> VenueFile.read(text))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("venue file: " + problem);
  }

  /** Other refusals, by a fragment of what they say: what the file holds, and what isn't JSON. */
  static Stream<Arguments> refusals() {
    String good = file(TestFields.tags(), "");
    return Stream.of(
        argumentSet("a bare layout, how to make one", "{\"tags\": []}", "Pitwall's upload"),
        argumentSet("a bare layout, called bare", "{\"tags\": []}", "bare WPILib layout"),
        argumentSet(
            "unknown key",
            file("party", BASE, TestFields.tags(), true, ", \"surveyor\": \"Sam\""),
            "unknown key \"surveyor\""),
        argumentSet(
            "unknown kind", file("party", BASE, TestFields.tags(), true, ""), "kind \"party\""),
        argumentSet("no base", good.replace("\"base\": \"Test field\", ", ""), "base is missing"),
        argumentSet(
            "another format",
            good.replace("\"format\": 1", "\"format\": 2"),
            "this code reads format 1"),
        argumentSet("a survey's old sections", file(TestFields.tags(), ", \"sigma\": {}"), "sigma"),
        argumentSet(
            "a key given twice",
            "{\"name\": \"a\", \"name\": \"b\"}",
            "not JSON: Map key 'name' has multiple values"),
        argumentSet("where the JSON broke", "{\"name\": \"a\", \"name\": \"b\"}", "position: 25"),
        argumentSet("a word that is no JSON", "{\"name\": tru}", "not JSON"),
        argumentSet(
            "nested past the reader's stack",
            "[".repeat(5000) + "]".repeat(5000),
            "not JSON a venue file can be: nested deeper than 64 levels"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void aFileThatIsNoVenueIsRefusedSayingWhy(String text, String message) {
    assertThatThrownBy(() -> VenueFile.read(text))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining(message);
  }

  @Test
  void jsonEdgesAreReadAsJsonDoes() {
    assertThat(VenueFile.bare("{\"tags\": [1, 2,]}")).as("a list's trailing comma").isFalse();
    assertThatThrownBy(() -> VenueFile.read("{\"name\": \"[[[[\"}"))
        .as("brackets in a string don't count")
        .hasMessageNotContaining("nested");
    assertThatThrownBy(() -> VenueFile.wrap("{\"a\": 1}", "x", Venue.Kind.EVENT, BASE))
        .hasMessageContaining("not a WPILib layout");
  }

  /** What a survey's tags say against the official layout: the problems, nothing when it fits. */
  static Stream<Arguments> surveys() {
    return Stream.of(
        argumentSet(
            "a survey matching the official layout", file(TestFields.tags(), ""), List.of()),
        argumentSet(
            "a small correction needs no reason", file(moved(4, 0.02, -0.01), ""), List.of()),
        argumentSet(
            "a tag far from its place needs a reason",
            file(moved(4, 0.2, 0), ""),
            List.of(
                "tag 4 sits 0.200 m and 0.0° from its official place: if that's right, say why"
                    + " under \"moved\"")),
        argumentSet(
            "...and has one",
            file(moved(4, 0.2, 0), ", \"moved\": {\"4\": \"rehung\"}"),
            List.of()),
        argumentSet(
            "a survey of another field",
            file("event", "2026 FRC Rebuilt Welded", TestFields.tags(), true, ""),
            List.of(
                "measured against \"2026 FRC Rebuilt Welded\","
                    + " but the field's official layout is \"Test field\"")),
        argumentSet(
            "a survey of other tags",
            file(withoutTagThreeButWithNinetyNine(), ""),
            List.of(
                "the layout is missing tags [3]",
                "the official layout has no tags [99]",
                "tag 13 no longer pairs with tag 3 under the field's symmetry")),
        argumentSet(
            "an official venue is WPILib's layout exactly",
            file("official", BASE, moved(4, 0.005, 0), true, ""),
            List.of("an official venue, but tag 4 sits 5.0 mm and 0.00° from WPILib's layout")),
        argumentSet(
            "a survey that breaks the symmetry",
            file(moved(14, 0, -1), ", \"moved\": {\"14\": \"a typo\"}"),
            List.of(
                "tag 4 no longer pairs with tag 14 under the field's symmetry",
                "tag 14 no longer pairs with tag 4 under the field's symmetry")),
        argumentSet(
            "a reason for a tag the layout lacks",
            file(TestFields.tags(), ", \"moved\": {\"78\": \"why\"}"),
            List.of("moved names tags the layout doesn't have: [78]")));
  }

  private static List<FieldTag> withoutTagThreeButWithNinetyNine() {
    List<FieldTag> tags = new ArrayList<>(TestFields.tags());
    tags.removeIf(tag -> tag.getID() == 3);
    tags.add(tag(99, 8, 4, 0));
    return tags;
  }

  @ParameterizedTest
  @MethodSource("surveys")
  void aSurveyIsCheckedAgainstTheOfficialLayout(String text, List<String> expected) {
    assertThat(problems(text)).containsExactlyInAnyOrderElementsOf(expected);
  }

  @Test
  void aSurveyGivesItsTagsWhereTheOfficialLayoutPutsThemAndWhereItFoundThem() {
    Venue surveyed = VenueFile.read(file(moved(4, 0.02, -0.01), ""));
    Venue official = VenueFile.read(file("official", BASE, TestFields.tags(), true, ""));
    Venue withNinetyNine = VenueFile.read(file(withoutTagThreeButWithNinetyNine(), ""));

    List<Tag> tags = surveyed.tags(TestFields.layout());

    assertThat(tags).extracting(Tag::id).containsExactly(2, 3, 12, 13, 4, 14);
    Tag four = tags.get(4);
    assertThat(four.nominal().getX()).isEqualTo(0);
    assertThat(four.surveyed()).isNotNull();
    assertThat(four.pose().getX()).isCloseTo(0.02, within(1e-12));
    assertThat(four.pose().getY()).isCloseTo(1.99, within(1e-12));
    TagLayout layout = surveyed.tagLayout(TestFields.layout());
    assertThat(layout.name()).isEqualTo("Practice field, October");
    assertThat(layout.lengthMeters()).isEqualTo(16);
    assertThat(layout.tags()).isEqualTo(tags);
    assertThat(official.tags(TestFields.layout()))
        .as("an official venue is nominal")
        .allMatch(t -> t.surveyed() == null);
    assertThatThrownBy(() -> withNinetyNine.tags(TestFields.layout()))
        .as("a tag its base doesn't have")
        .hasMessageContaining("no tag 99");
  }
}
