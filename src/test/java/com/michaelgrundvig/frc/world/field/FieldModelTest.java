package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.Alliance.BLUE;
import static com.michaelgrundvig.frc.world.field.Alliance.RED;
import static com.michaelgrundvig.frc.world.field.TestFields.FIELD;
import static com.michaelgrundvig.frc.world.field.TestFields.assertPose;
import static com.michaelgrundvig.frc.world.field.TestFields.pose;
import static com.michaelgrundvig.frc.world.field.TestFields.refused;
import static com.michaelgrundvig.frc.world.field.TestFields.xy;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.List;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;

class FieldModelTest {
  private static Start start() {
    return Start.at("Center", Pose2d.ZERO);
  }

  private static Zone zone() {
    return Zone.ours("Zone", Polygon2d.rectangle(xy(0, 0), xy(1, 1)));
  }

  /** The test field's tags with one more, and a model over them. */
  private static FieldModel withTag(FieldTag extra) {
    List<FieldTag> tags = TestFields.tags();
    tags.add(extra);
    return new FieldModel(
        TestFields.layout(tags), Symmetry.ROTATED, List.of(), List.of(), List.of());
  }

  @Test
  void itLooksTagsUpByNumber() {
    assertThat(FIELD.tags()).containsOnlyKeys(2, 3, 12, 13, 4, 14);
    assertThat(FIELD.tag(4).orElseThrow().getY()).isEqualTo(2);
    assertThat(FIELD.tag(99)).isEmpty();
  }

  @Test
  void anythingOnTheFieldIsAsWrittenForBlueAndItsImageForRed() {
    Pose2d blue = pose(1, 2, 30);
    Polygon2d area = Polygon2d.rectangle(xy(0, 0), xy(2, 3));
    Pose3d blue3d = new Pose3d(1, 2, 1.5, new Rotation3d(0, 0.2, 0.5));

    assertThat(FIELD.forAlliance(BLUE, blue)).isEqualTo(blue);
    assertPose(FIELD.forAlliance(RED, blue), 15, 6, 210);
    assertThat(FIELD.forAlliance(RED, xy(1, 2))).isEqualTo(xy(15, 6));
    assertThat(FIELD.forAlliance(BLUE, area)).isSameAs(area);
    assertThat(FIELD.forAlliance(RED, area).contains(xy(15, 6))).isTrue();
    assertThat(FIELD.forAlliance(RED, area).contains(xy(1, 2))).isFalse();
    assertThat(FIELD.forAlliance(BLUE, blue3d)).isSameAs(blue3d);
    assertThat(FIELD.forAlliance(RED, blue3d))
        .isEqualTo(Symmetry.ROTATED.apply(blue3d, TestFields.LENGTH, TestFields.WIDTH));
  }

  @Test
  void driversFaceEachOtherWhateverTheSymmetry() {
    assertThat(FieldModel.driverForward(BLUE)).isEqualTo(Rotation2d.ZERO);
    assertThat(FieldModel.driverForward(RED)).isEqualTo(Rotation2d.PI);
  }

  @ParameterizedTest(name = "inside: {2} for {0}, {1}")
  @CsvSource({
    "8, 4, true",
    "0, 0, true",
    "-0.1, 4, false",
    "8, 8.1, false",
    "16.1, 4, false",
    "8, -1, false"
  })
  void theFieldIsWithinItsWalls(double x, double y, boolean inside) {
    assertThat(FIELD.contains(xy(x, y))).isEqualTo(inside);
  }

  @ParameterizedTest(name = "{0}''s tag {1} pairs with tag {2}")
  @CsvSource({"BLUE, 2, 2", "RED, 2, 12", "RED, 3, 13", "RED, 12, 2", "RED, 4, 14"})
  void eachTagIsPairedWithTheTagTheSymmetryPutsInItsPlace(Alliance alliance, int tag, int partner) {
    assertThat(FIELD.tagFor(alliance, tag)).isEqualTo(partner);
  }

  /**
   * On 2026's field, trench tags stand back to back, 7.5 cm apart: only one faces the right way.
   */
  @Test
  void aTagFacingAwayIsNotAPartnerHoweverClose() {
    assertThat(withTag(TestFields.tag(15, 12.05, 4, 180)).tagFor(RED, 2)).isEqualTo(12);
  }

  @Test
  void aTagWithoutAPartnerServesBlueButNotRed() {
    FieldModel model = withTag(TestFields.tag(7, 6, 1, 90));

    assertThat(model.tagFor(BLUE, 7)).isEqualTo(7);
    assertThatThrownBy(() -> model.tagFor(RED, 7)).hasMessageContaining("tag 7 has no partner");
  }

  @Test
  void theSymmetryErrorShowsWhetherTheLayoutHasTheDeclaredSymmetry() {
    var mirrored =
        new FieldModel(TestFields.layout(), Symmetry.MIRRORED, List.of(), List.of(), List.of());

    assertThat(FIELD.symmetryError()).isLessThan(1e-9);
    assertThat(mirrored.symmetryError()).as("the wrong symmetry").isGreaterThan(1);
  }

  static Stream<Arguments> refusals() {
    Polygon2d offTheField = Polygon2d.rectangle(xy(-6, 4), xy(-5, 5));
    return Stream.of(
        refused(
            "a place at a tag the layout lacks",
            () -> TestFields.model(List.of(), List.of(Place.between("Goal", 77)), List.of()),
            "Goal: the layout has no tag 77"),
        refused(
            "a start and a place of one name",
            () ->
                TestFields.model(
                    List.of(start()), List.of(Place.at("Center", Pose2d.ZERO)), List.of()),
            "two places are named Center"),
        refused(
            "two zones of one name",
            () -> TestFields.model(List.of(), List.of(), List.of(zone(), zone())),
            "two zones are named Zone"),
        refused(
            "a zone off the field",
            () ->
                TestFields.model(
                    List.of(), List.of(), List.of(Zone.ours("Parking lot", offTheField))),
            "Parking lot: an area is off the field"),
        refused(
            "a partner of a tag the layout lacks",
            () -> FIELD.tagFor(BLUE, 99),
            "the layout has no tag 99"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void aModelThatDoesNotFitItsLayoutIsRefusedSayingWhy(ThrowingCallable building, String message) {
    assertThatThrownBy(building).hasMessageContaining(message);
  }

  @Test
  void theBuilderLeavesOutWhatIsntGiven() {
    FieldModel model =
        FieldModel.builder(TagLayout.of(TestFields.layout()), Symmetry.ROTATED)
            .starts(List.of(start()))
            .build();

    assertThat(model.starts()).containsExactly(start());
    assertThat(model.places()).isEmpty();
    assertThat(model.zones()).isEmpty();
    assertThat(model.terrain().surfaces()).isEmpty();
  }
}
