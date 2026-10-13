package com.michaelgrundvig.frc.world.field;

import java.util.ArrayList;
import java.util.List;

/**
 * The elements of a whole field, from what a season's field writes: what stands on the blue
 * alliance's half, and what stands across the center line.
 */
final class Elements {
  private Elements() {}

  /**
   * Every element on the field. Each one written for blue's half is followed by its image on red's
   * half, carrying the partners of its tags, and the two are named for the half each stands on, not
   * for whose it is: "Hub (blue)" and "Hub (red)". What is shared keeps its name and comes last.
   *
   * @param blueElements what stands on the blue alliance's half
   * @param shared what stands across the center line, each written once, whole
   * @param field the field they stand on, its tags already paired
   * @throws IllegalArgumentException if a tag an element carries isn't in the layout, or has no
   *     partner on red's half
   */
  static List<Element> onBothHalves(
      List<Element> blueElements, List<Element> shared, FieldModel field) {
    List<Element> onBothHalves = new ArrayList<>();
    for (Element element : blueElements) {
      onBothHalves.add(new Element(element.name() + " (blue)", element.blocks(), element.tags()));
      onBothHalves.add(onRedsHalf(element, field));
    }
    onBothHalves.addAll(shared);
    return List.copyOf(onBothHalves);
  }

  /** An element's image on red's half: each block turned, and each tag's partner in its place. */
  private static Element onRedsHalf(Element element, FieldModel field) {
    List<Block> blocks = new ArrayList<>();
    for (Block block : element.blocks()) {
      blocks.add(onTheOtherHalf(block, field));
    }
    List<Integer> tags = new ArrayList<>();
    for (int tag : element.tags()) {
      tags.add(partnerOf(element, tag, field));
    }
    return new Element(element.name() + " (red)", blocks, tags);
  }

  /** A block's image under the field's symmetry: its outline turned, at the same heights. */
  static Block onTheOtherHalf(Block block, FieldModel field) {
    return new Block(
        field.symmetry().apply(block.outline(), field.length(), field.width()),
        block.bottomMeters(),
        block.topMeters(),
        block.material());
  }

  /**
   * The tag that stands in a tag's place on the other half.
   *
   * @throws IllegalArgumentException naming the element, if the layout has no such tag or no
   *     partner for it
   */
  static int partnerOf(Element element, int tag, FieldModel field) {
    try {
      return field.tagFor(Alliance.RED, tag);
    } catch (IllegalArgumentException e) {
      throw new IllegalArgumentException(element.name() + ": " + e.getMessage(), e);
    }
  }
}
