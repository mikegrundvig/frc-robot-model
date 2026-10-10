package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.Alliance.BLUE;
import static com.michaelgrundvig.frc.world.field.Alliance.RED;
import static com.michaelgrundvig.frc.world.field.TestFields.FIELD;
import static com.michaelgrundvig.frc.world.field.TestFields.assertPose;
import static com.michaelgrundvig.frc.world.field.TestFields.pose;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;

class PlaceTest {
  /** The places the tables name, built as each test runs. */
  private static Place place(String name) {
    return switch (name) {
      case "start" -> Place.at("Start", pose(1, 2, 0));
      case "spot" -> Place.at("Spot", pose(1, 2, 0), pose(14, 1, 180));
      case "hub" -> Place.between("Hub", 2, 3);
      case "loading" -> Place.facing("Loading", 4, 1.5);
      default -> throw new AssertionError(name);
    };
  }

  static Stream<Arguments> places() {
    return Stream.of(
        argumentSet("a start for blue is as written", "start", BLUE, 1, 2, 0),
        argumentSet("a start for red is its image", "start", RED, 15, 6, 180),
        argumentSet("an asymmetric spot for blue", "spot", BLUE, 1, 2, 0),
        argumentSet("an asymmetric spot is written for red too", "spot", RED, 14, 1, 180),
        argumentSet("between tags, blue's own tags", "hub", BLUE, 4.5, 4, 0),
        argumentSet("between tags, red's own tags", "hub", RED, 11.5, 4, 180),
        argumentSet("facing a tag, blue stands before it", "loading", BLUE, 1.5, 2, 180),
        argumentSet("facing a tag, red stands before it", "loading", RED, 14.5, 6, 0));
  }

  @ParameterizedTest
  @MethodSource("places")
  void aPlaceIsWrittenForBlueAndItsImageForRed(
      String place, Alliance alliance, double x, double y, double degrees) {
    assertPose(place(place).on(FIELD, alliance), x, y, degrees);
  }

  /** Each alliance's places come from its own tags, so a measurement corrects both. */
  @Test
  void aMeasuredLayoutMovesEachAlliancesPlacesWithItsOwnTags() {
    List<FieldTag> measured = TestFields.tags();
    measured.set(2, TestFields.tag(12, 12.04, 4, 0)); // red's far face is 4 cm further out
    var model =
        new FieldModel(
            TestFields.layout(measured), Symmetry.ROTATED, List.of(), List.of(), List.of());

    assertPose(place("hub").on(model, RED), 11.52, 4, 180);
    assertPose(place("hub").on(model, BLUE), 4.5, 4, 0);
  }

  /** A task lining up on a place trusts the tags it's found from: each alliance's own. */
  @Test
  void aPlaceNamesTheTagsItIsFoundFrom() {
    assertThat(place("hub").tags(FIELD, BLUE)).containsExactly(2, 3);
    assertThat(place("hub").tags(FIELD, RED)).containsExactly(12, 13);
    assertThat(place("loading").tags(FIELD, RED)).containsExactly(14);
    assertThat(place("start").tags(FIELD, BLUE)).isEmpty();
  }

  @Test
  void aPlaceNeedsANameAndSomethingToStandOn() {
    assertThatThrownBy(() -> Place.at(" ", Pose2d.ZERO)).hasMessage("a place needs a name");
    assertThatThrownBy(() -> Place.between("Hub")).hasMessage("Hub needs at least one tag");
    assertThatThrownBy(() -> Place.facing("Loading", 4, -1))
        .hasMessage("Loading: the distance must be zero or more");
  }
}
