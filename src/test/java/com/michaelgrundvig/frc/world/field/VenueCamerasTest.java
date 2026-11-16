package com.michaelgrundvig.frc.world.field;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.game.Game;
import com.michaelgrundvig.frc.world.game.Rebuilt;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;

/**
 * A venue's camera settings: read from its file, written back into it without touching its survey,
 * kept from one station's tuning for every camera or for that station, and carried by a venue made
 * from the official layout alone.
 */
class VenueCamerasTest {
  private static final Game GAME = Rebuilt.GAME;

  /** The game's official layout, as a venue with no survey: what the pit makes for an event. */
  private static final Venue EVENT = VenueFile.official("Week 1 event", GAME.official());

  private static VenueCameras.Settings settings(
      @Nullable Double exposureMs, @Nullable Integer gain) {
    return new VenueCameras.Settings(
        Optional.ofNullable(exposureMs).map(Milliseconds::of), Optional.ofNullable(gain));
  }

  /** A venue file's text with its "cameras" part as given, around the event venue's own file. */
  private static String withCamerasPart(String cameras) {
    String text = EVENT.text().strip();
    return text.substring(0, text.length() - 1) + ", \"cameras\": " + cameras + "}";
  }

  @Test
  void aVenueMadeFromTheOfficialLayoutFitsTheFieldAndSaysNothingOfItsCameras() {
    assertThat(EVENT.name()).isEqualTo("Week 1 event");
    assertThat(EVENT.kind()).isEqualTo(Venue.Kind.OFFICIAL);
    assertThat(EVENT.base()).isEqualTo(GAME.official().getName());
    assertThat(EVENT.method()).isEqualTo("official");
    assertThat(EVENT.cameras()).isEqualTo(VenueCameras.NONE);
    assertThat(VenueCheck.problems(EVENT, GAME.layout(GAME.official()), GAME.symmetry())).isEmpty();
    assertThatThrownBy(() -> VenueFile.official(" ", GAME.official()))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("give the event's");
  }

  /** A ThriftyCam's settings, by its type's name. */
  private static Map<String, VenueCameras.Settings> thriftyCam(
      @Nullable Double exposureMs, @Nullable Integer gain) {
    return Map.of("ThriftyCam", settings(exposureMs, gain));
  }

  /** A station's own settings, kept with a ThriftyCam in it. */
  private static VenueCameras.StationSettings own(
      @Nullable Double exposureMs, @Nullable Integer gain) {
    return new VenueCameras.StationSettings("ThriftyCam", settings(exposureMs, gain));
  }

  /** What a venue's file says of its cameras, or every problem with it, one row a case. */
  @Test
  void aVenuesCamerasAreReadFromItsFile() {
    record Row(String what, String cameras, VenueCameras read, String problem) {}
    List<Row> rows =
        List.of(
            new Row(
                "a type's exposure and gain",
                "{\"types\": {\"ThriftyCam\": {\"exposureMs\": 1.5, \"gain\": 40}}}",
                new VenueCameras(thriftyCam(1.5, 40), Map.of()),
                ""),
            new Row(
                "a station's own gain",
                "{\"types\": {\"ThriftyCam\": {\"gain\": 40}},"
                    + " \"stations\": {\"back\": {\"type\": \"ThriftyCam\", \"gain\": 52}}}",
                new VenueCameras(thriftyCam(null, 40), Map.of("back", own(null, 52))),
                ""),
            new Row(
                "two kinds of camera, each its own",
                "{\"types\": {\"ThriftyCam\": {\"gain\": 40}, \"Arducam\": {\"gain\": 8}}}",
                new VenueCameras(
                    Map.of("ThriftyCam", settings(null, 40), "Arducam", settings(null, 8)),
                    Map.of()),
                ""),
            new Row("nothing", "{}", VenueCameras.NONE, ""),
            new Row(
                "a gain for every camera, whatever its kind",
                "{\"gain\": 40}",
                VenueCameras.NONE,
                "cameras: unknown key \"gain\""),
            new Row(
                "a station's misspelling",
                "{\"stations\": {\"back\": {\"type\": \"ThriftyCam\", \"exposure\": 2}}}",
                VenueCameras.NONE,
                "cameras: station back: unknown key \"exposure\""),
            new Row(
                "a gain that isn't whole",
                "{\"types\": {\"ThriftyCam\": {\"gain\": 40.5}}}",
                VenueCameras.NONE,
                "cameras: camera type ThriftyCam: gain 40.5 (a whole number from 0 to 65535)"),
            new Row(
                "a gain too high",
                "{\"stations\": {\"back\": {\"type\": \"ThriftyCam\", \"gain\": 70000}}}",
                VenueCameras.NONE,
                "cameras: station back: gain 70000.0 (a whole number from 0 to 65535)"),
            new Row(
                "an exposure too short",
                "{\"types\": {\"ThriftyCam\": {\"exposureMs\": 0.05}}}",
                VenueCameras.NONE,
                "cameras: camera type ThriftyCam: exposureMs 0.05 (from 0.1 to 1000 ms)"),
            new Row(
                "a station's own without the type it was kept for",
                "{\"stations\": {\"back\": {\"gain\": 52}}}",
                VenueCameras.NONE,
                "cameras: station back doesn't say the camera type its settings were kept for: add"
                    + " it, \"type\": \"ThriftyCam\" for a station that held a ThriftyCam"),
            new Row("not an object", "40", VenueCameras.NONE, "cameras"));
    for (Row row : rows) {
      String text = withCamerasPart(row.cameras());
      if (row.problem().isEmpty()) {
        assertThat(VenueFile.read(text).cameras()).as(row.what()).isEqualTo(row.read());
        continue;
      }
      assertThatThrownBy(() -> VenueFile.read(text))
          .as(row.what())
          .isInstanceOf(IllegalArgumentException.class)
          .hasMessageContaining(row.problem());
    }
  }

  /**
   * Settings written into a venue's file read back as they were, its survey and name untouched; a
   * venue given none has no "cameras" part, and a bare layout wrapped as a venue has none either.
   */
  @Test
  void settingsWrittenIntoAVenueReadBackWithItsSurveyUnchanged() {
    VenueCameras cameras = new VenueCameras(thriftyCam(1.2, 40), Map.of("back", own(2.0, null)));

    Venue kept = VenueFile.withCameras(EVENT, cameras);
    Venue reread = VenueFile.read(kept.text());

    assertThat(reread.cameras()).isEqualTo(cameras);
    assertThat(reread.name()).isEqualTo(EVENT.name());
    assertThat(reread.kind()).isEqualTo(EVENT.kind());
    assertThat(reread.layout().getTags()).isEqualTo(EVENT.layout().getTags());
    assertThat(kept.hash())
        .as("another file, so another venue to commit")
        .isNotEqualTo(EVENT.hash());
    assertThat(kept.text())
        .contains("\"ThriftyCam\"")
        .contains("\"exposureMs\": 1.2")
        .contains("\"gain\": 40");

    Venue cleared = VenueFile.withCameras(kept, VenueCameras.NONE);
    assertThat(cleared.cameras()).isEqualTo(VenueCameras.NONE);
    assertThat(cleared.text()).doesNotContain("\"cameras\"");

    String bare = GAME.official().getJson();
    Venue wrapped = VenueFile.wrap(bare, "", Venue.Kind.EVENT, GAME.official().getName());
    assertThat(wrapped.cameras()).isEqualTo(VenueCameras.NONE);
    Venue wrappedKept = VenueFile.withCameras(wrapped, cameras);
    assertThat(wrappedKept.wrapped()).isTrue();
    assertThat(wrappedKept.cameras()).isEqualTo(cameras);
  }

  /**
   * Kept for a camera's type, a tuned value is every camera of that type's, and the station it was
   * tuned at gives up its own of that value; kept for the station, it's that station's alone. Each
   * camera reads its station's own where it has one for its type, else its type's; another kind of
   * camera never reads a ThriftyCam's, even in a station that kept one with a ThriftyCam in it.
   */
  @Test
  void aTunedValueIsKeptForItsTypeOrForItsStation() {
    VenueCameras start =
        new VenueCameras(
            thriftyCam(1.0, 30),
            Map.of(
                "back", own(null, 52),
                "side", own(2.0, null),
                "top", new VenueCameras.StationSettings("ThriftyCam", settings(null, 70))));
    record Camera(String station, String type) {}
    Camera front = new Camera("front", "ThriftyCam");
    Camera back = new Camera("back", "ThriftyCam");
    Camera side = new Camera("side", "ThriftyCam");
    Camera top = new Camera("top", "Arducam");
    record Row(String what, VenueCameras kept, Map<Camera, VenueCameras.Settings> reads) {}
    List<Row> rows =
        List.of(
            new Row(
                "as it starts",
                start,
                Map.of(
                    front, settings(1.0, 30),
                    back, settings(1.0, 52),
                    side, settings(2.0, 30),
                    top, settings(null, null))),
            new Row(
                "the back's gain, for every ThriftyCam",
                start.keptForType("ThriftyCam", "back", settings(null, 44)),
                Map.of(
                    front, settings(1.0, 44),
                    back, settings(1.0, 44),
                    side, settings(2.0, 44),
                    top, settings(null, null))),
            new Row(
                "the front's exposure and gain, for the front alone",
                start.keptForStation("front", "ThriftyCam", settings(1.5, 36)),
                Map.of(
                    front, settings(1.5, 36),
                    back, settings(1.0, 52),
                    side, settings(2.0, 30),
                    top, settings(null, null))),
            new Row(
                "an Arducam in the top station now, kept for the station: the ThriftyCam's dropped",
                start.keptForStation("top", "Arducam", settings(null, 9)),
                Map.of(
                    front, settings(1.0, 30),
                    back, settings(1.0, 52),
                    side, settings(2.0, 30),
                    top, settings(null, 9))),
            new Row(
                "the top's gain, for every Arducam",
                start.keptForType("Arducam", "top", settings(null, 8)),
                Map.of(
                    front, settings(1.0, 30),
                    back, settings(1.0, 52),
                    side, settings(2.0, 30),
                    top, settings(null, 8))));
    for (Row row : rows) {
      row.reads()
          .forEach((camera, reads) ->
              assertThat(row.kept().forCamera(camera.station(), camera.type()))
                  .as(row.what() + ", " + camera.station())
                  .isEqualTo(reads));
    }
    assertThat(start.keptForType("ThriftyCam", "back", settings(null, 44)).stationsOwn("back"))
        .as("the back gave up its only value of its own")
        .isEmpty();
    assertThatThrownBy(() -> settings(null, -1))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("a gain control takes 0 to 65535");
    assertThatThrownBy(() -> settings(2000.0, null))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining("from 100 us to 1 s");
  }
}
