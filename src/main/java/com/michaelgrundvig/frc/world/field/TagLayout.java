package com.michaelgrundvig.frc.world.field;

import static org.wpilib.units.Units.Meters;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.units.measure.Distance;

/**
 * The field's size and its AprilTags: what a {@link FieldModel} is built on. The official layout
 * gives only each tag's nominal pose ({@link #of}); a venue adds where its survey found them
 * ({@link Venue#tagLayout}), and keeps the official layout's family and tag size.
 *
 * <p>The tags' family and size are the game's, from its manual: the one place every camera's
 * software, the simulator and the views read them from.
 *
 * @param name what it's called: the official layout's name (such as "2026 FRC Rebuilt Welded"), or
 *     a venue's
 * @param lengthMeters along x, from the blue alliance wall to the red
 * @param widthMeters along y
 * @param family which AprilTag family the tags are printed from
 * @param tagSizeMeters the side of each tag's black square, more than 0 and less than 10 m
 * @param tags every tag, each number once
 */
public record TagLayout(
    String name,
    double lengthMeters,
    double widthMeters,
    TagFamily family,
    double tagSizeMeters,
    List<Tag> tags) {
  /** The largest tag size taken: a tag's side is centimeters, and 10 m is a typing error. */
  static final int LARGEST_TAG_METERS = 10;

  public TagLayout {
    if (!(lengthMeters > 0)
        || !(widthMeters > 0)
        || Double.isInfinite(lengthMeters + widthMeters)) {
      throw new IllegalArgumentException(
          name + ": a field's size is positive, not " + lengthMeters + " by " + widthMeters + " m");
    }
    Objects.requireNonNull(family, "a field's tag family");
    if (!(tagSizeMeters > 0 && tagSizeMeters < LARGEST_TAG_METERS)) {
      throw new IllegalArgumentException(
          name
              + ": a tag's size is its black square, more than 0 and less than "
              + LARGEST_TAG_METERS
              + " m, not "
              + tagSizeMeters);
    }
    tags = List.copyOf(tags);
    Set<Integer> ids = new HashSet<>();
    for (Tag tag : tags) {
      if (!ids.add(tag.id())) {
        throw new IllegalArgumentException(name + " has tag " + tag.id() + " twice");
      }
    }
  }

  /**
   * WPILib's layout, each tag where it says: nominal, with nothing surveyed. Its tags are of this
   * family and size, as the game's manual prints them: {@code TagLayout.of(layout,
   * TagFamily.TAG_36H11, Inches.of(6.5))}.
   */
  public static TagLayout of(Field layout, TagFamily family, Distance tagSize) {
    List<Tag> tags = new ArrayList<>();
    for (FieldTag tag : layout.getTags()) {
      tags.add(Tag.nominal(tag.getID(), tag.getPose()));
    }
    return new TagLayout(
        layout.getName(),
        layout.getFieldLength(),
        layout.getFieldWidth(),
        family,
        tagSize.in(Meters),
        tags);
  }
}
