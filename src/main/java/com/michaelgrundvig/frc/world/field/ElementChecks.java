package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * What a field's elements have to get right before anything uses them: each has a name of its own
 * and reaches the field, each tag is in the layout and hangs on the one element that carries it,
 * and what stands across the center line is its own image under the field's symmetry. A {@link
 * FieldModel} runs these as it is built ({@link FieldChecks}).
 */
final class ElementChecks {
  private ElementChecks() {}

  /**
   * How far a tag an element carries may be from the element: tags hang on an element's faces, and
   * blocks drawn from the manual may be a little off its CAD. A tag farther off is a typo.
   */
  static final double TAG_ON_ELEMENT_METERS = 0.1;

  /**
   * How far the image of a corner of a shared element may be from the element, past the layout's
   * own asymmetry: room for a manual's rounding. Half an element left out, or one drawn off center,
   * is much farther off.
   */
  static final double SYMMETRIC_WITHIN_METERS = 0.01;

  /**
   * Refuses elements that get any of it wrong.
   *
   * @param shared the elements that stand across the center line, which are among the field's
   * @throws IllegalArgumentException naming the first thing that is wrong
   */
  static void require(FieldModel field, List<Element> shared) {
    Set<String> names = new HashSet<>();
    Map<Integer, String> carriedBy = new HashMap<>();
    for (Element element : field.elements()) {
      boolean nameIsNew = names.add(element.name());
      if (!nameIsNew) {
        throw new IllegalArgumentException("two elements are named " + element.name());
      }
      requireOnTheField(element, field);
      for (int tag : element.tags()) {
        String other = carriedBy.put(tag, element.name());
        if (other != null) {
          throw new IllegalArgumentException(
              "tag " + tag + " is on two elements, " + other + " and " + element.name());
        }
        requireTagOnIt(element, tag, field);
      }
    }
    for (Element element : shared) {
      requireItsOwnImage(element, field);
    }
  }

  /**
   * Some of each block is on the field, seen from above: within the walls, or on them. What's set
   * into the field's edge (a speaker, a pocket, an outpost's face) reaches it. A block wholly
   * outside the walls is a typo.
   */
  private static void requireOnTheField(Element element, FieldModel field) {
    Polygon2d withinTheWalls = field.outline();
    for (Block block : element.blocks()) {
      if (!block.outline().touches(withinTheWalls)) {
        throw new IllegalArgumentException(
            element.name() + ": a block of it is wholly off the field");
      }
    }
  }

  /** A tag an element carries is in the layout, and hangs on the element, as the layout has it. */
  private static void requireTagOnIt(Element element, int id, FieldModel field) {
    Tag tag = field.tags().get(id);
    if (tag == null) {
      throw new IllegalArgumentException(element.name() + ": the layout has no tag " + id);
    }
    double offItMeters = element.distance(tag.nominal().getTranslation());
    if (offItMeters > TAG_ON_ELEMENT_METERS) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s: tag %d is %.2f m off it, as the official layout has it",
              element.name(),
              id,
              offItMeters));
    }
  }

  /**
   * An element across the center line is its own image, as far as the layout itself is: the image
   * of each corner of each block, at its bottom and its top, lands on the element, and each tag it
   * carries has its partner on it too.
   */
  private static void requireItsOwnImage(Element element, FieldModel field) {
    double layoutsOwnError = Math.min(field.symmetryError(), TagPartners.PARTNER_DISTANCE_METERS);
    double withinMeters = SYMMETRIC_WITHIN_METERS + layoutsOwnError;
    for (Block block : element.blocks()) {
      for (Translation2d corner : block.outline().corners()) {
        Translation2d image = field.symmetry().apply(corner, field.length(), field.width());
        for (double height : new double[] {block.bottomMeters(), block.topMeters()}) {
          double offItMeters =
              element.distance(new Translation3d(image.getX(), image.getY(), height));
          if (offItMeters > withinMeters) {
            throw new IllegalArgumentException(
                String.format(
                    Locale.ROOT,
                    "%s: it isn't its own image under the field's symmetry:"
                        + " a corner at %s lands %.2f m off it",
                    element.name(),
                    corner,
                    offItMeters));
          }
        }
      }
    }
    for (int tag : element.tags()) {
      int partner = Elements.partnerOf(element, tag, field);
      if (!element.tags().contains(partner)) {
        throw new IllegalArgumentException(
            element.name() + ": it carries tag " + tag + " but not its partner, " + partner);
      }
    }
  }
}
