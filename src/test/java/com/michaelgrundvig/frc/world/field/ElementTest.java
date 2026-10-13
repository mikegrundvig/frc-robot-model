package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.TestFields.polygon;
import static com.michaelgrundvig.frc.world.field.TestFields.refused;
import static com.michaelgrundvig.frc.world.field.TestFields.xy;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.List;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Translation3d;

/**
 * Elements on the small made-up field ({@link TestFields}): a "hub" between tags 2 and 3 with a bar
 * reaching out from it overhead, and what a field refuses.
 */
class ElementTest {
  private static final Material STEEL = Material.builder().name("steel").build();

  /** The hub's body: 1 m square between its tags' faces at x = 4 and x = 5, 2 m tall. */
  private static final Block BODY = Block.standing(square(4, 3.5, 5, 4.5), 2, STEEL);

  /** A bar out of the hub's left side, from 1.5 m up: something to drive under. */
  private static final Block BAR = new Block(square(4.4, 4.5, 4.6, 6), 1.5, 2, STEEL);

  private static final Element HUB = new Element("Hub", List.of(BODY, BAR), List.of(2, 3));

  private static Polygon2d square(double x0, double y0, double x1, double y1) {
    return Polygon2d.rectangle(xy(x0, y0), xy(x1, y1));
  }

  private static FieldModel.Builder field() {
    return FieldModel.builder(TagLayout.of(TestFields.layout()), Symmetry.ROTATED);
  }

  private static Element block(String name, Polygon2d outline, int... tags) {
    return Element.standing(name, outline, 2, STEEL, tags);
  }

  @Test
  void anElementAnswersWhatItCoversHowTallItIsAndWhatIsInTheWay() {
    assertThat(HUB.footprint().area())
        .as("the outline around the body and the bar")
        .isGreaterThan(1.3);
    assertThat(HUB.footprint().contains(xy(4.5, 5.9))).isTrue();
    assertThat(HUB.heightMeters()).isEqualTo(2);
    // Bumpers from 0.05 to 0.2 m up run into the body, and pass under the bar.
    assertThat(HUB.inTheWayOf(0.05, 0.2)).containsExactly(BODY.outline());
    assertThat(HUB.inTheWayOf(1.4, 1.6)).containsExactly(BODY.outline(), BAR.outline());
    assertThat(HUB.inTheWayOf(2, 3)).isEmpty();

    assertThat(HUB.contains(new Translation3d(4.5, 5, 1.7))).as("in the bar").isTrue();
    assertThat(HUB.distance(new Translation3d(4.5, 5, 1.2)))
        .as("under the bar")
        .isCloseTo(0.3, within(1e-9));
    assertThat(HUB.distance(new Translation3d(5.3, 4, 2.4)))
        .as("off its top edge")
        .isCloseTo(0.5, within(1e-9));
  }

  @Test
  void theFieldHasEachElementOnBothHalvesWithItsTagsAndWhatIsSharedOnce() {
    Element pillar = block("Pillar", square(7.5, 3, 8.5, 5));
    FieldModel model = field().elements(List.of(HUB)).shared(List.of(pillar)).build();

    assertThat(model.elements())
        .extracting(Element::name)
        .containsExactly("Hub (blue)", "Hub (red)", "Pillar");
    Element red = model.elements().get(1);
    assertThat(red.tags()).as("the partners of 2 and 3").containsExactly(12, 13);
    assertThat(red.contains(new Translation3d(11.5, 3, 1.7)))
        .as("the bar, turned half a turn")
        .isTrue();
    assertThat(red.inTheWayOf(0.05, 0.2))
        .singleElement()
        .satisfies(body -> assertThat(body.centroid().getDistance(xy(11.5, 4))).isLessThan(1e-9));
    assertThat(model.outline().corners()).containsExactly(xy(0, 0), xy(16, 0), xy(16, 8), xy(0, 8));
    assertThat(TestFields.FIELD.elements()).as("a field with none written").isEmpty();
    FieldModel bare = model.withoutElements();
    assertThat(bare.elements()).as("the same field, with nothing standing on it").isEmpty();
    assertThat(bare.tags()).isEqualTo(model.tags());
    assertThat(bare.outline()).isEqualTo(model.outline());
  }

  static Stream<Arguments> refusals() {
    Polygon2d onTheHub = square(4, 3.5, 5, 4.5);
    return Stream.of(
        refused(
            "an element with no name",
            () -> new Element(" ", List.of(BODY), List.of()),
            "needs a name"),
        refused(
            "an element with no block",
            () -> new Element("Hub", List.of(), List.of()),
            "needs a block"),
        refused("a tag twice", () -> new Element("Hub", List.of(BODY), List.of(2, 2)), "each once"),
        refused(
            "a block upside down", () -> new Block(onTheHub, 2, 1, STEEL), "up to a greater one"),
        refused(
            "an outline with a corner pointing in",
            () -> Block.standing(polygon(0, 0, 2, 0, 2, 2, 1, 0.5, 0, 2), 1, STEEL),
            "several blocks"),
        refused(
            "two of a name",
            () -> field().elements(List.of(HUB, HUB)).build(),
            "two elements are named Hub (blue)"),
        refused(
            "a tag the layout lacks",
            () -> field().elements(List.of(block("Hub", onTheHub, 99))).build(),
            "Hub: the layout has no tag 99"),
        refused(
            "a tag hanging somewhere else",
            () -> field().elements(List.of(block("Post", square(1, 1, 2, 2), 2))).build(),
            "Post (blue): tag 2 is 2.83 m off it"),
        refused(
            "a tag on two elements",
            () ->
                field()
                    .elements(
                        List.of(
                            block("Near", square(3.5, 3.5, 4, 4.5), 2), block("Far", onTheHub, 2)))
                    .build(),
            "tag 2 is on two elements, Near (blue) and Far (blue)"),
        refused(
            "a block off the field",
            () -> field().elements(List.of(block("Lost", square(-3, 1, -2, 2)))).build(),
            "Lost (blue): a block of it is wholly off the field"),
        refused(
            "something shared that is off center",
            () -> field().shared(List.of(block("Pillar", square(7.5, 3, 9, 5)))).build(),
            "Pillar: it isn't its own image under the field's symmetry"),
        refused(
            "something shared carrying half a pair of tags",
            () -> field().shared(List.of(block("Wide", square(4, 3.5, 12, 4.5), 2))).build(),
            "Wide: it carries tag 2 but not its partner, 12"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void whatCannotStandIsRefusedByName(ThrowingCallable building, String message) {
    assertThatThrownBy(building)
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessageContaining(message);
  }
}
