package com.michaelgrundvig.frc.world.field;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HexFormat;
import java.util.List;
import java.util.Map;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;

/**
 * A field as measured: the official layout corrected where a venue's tags were found, such as
 * WPIcal's output for an event's field. Venues are runtime data, never code: a file uploaded in the
 * pit, or committed under {@code venues/}, read by {@link VenueFile} and checked by {@link
 * VenueCheck} before the robot uses it. Set against the official layout it was measured against, it
 * gives its {@link Tag}s ({@link #tags}), which a {@link FieldModel} is built on.
 *
 * @param name what people call it, such as "Practice field, October"
 * @param kind a practice field, a shop, an event, or the official layout itself
 * @param base the name of the official layout it was measured against (WPILib's, such as "2026 FRC
 *     Rebuilt Welded")
 * @param layout every tag where the survey found it, in WPILib's field frame: the file's layout,
 *     unchanged, as the coprocessors get it
 * @param moved why each tag the survey found far from its official place is there
 * @param method how it was measured: "hand", "wpical", "bundle adjustment", or "unknown"
 * @param date when it was measured (ISO 8601), or empty when unknown
 * @param wrapped true when the file was a bare WPILib layout, given the stated defaults
 * @param text the file as read, which is what's logged, stored and downloaded
 */
public record Venue(
    String name,
    Kind kind,
    String base,
    Field layout,
    Map<Integer, String> moved,
    String method,
    String date,
    boolean wrapped,
    String text) {

  /** The one format this code reads; a file says which it was written in. */
  public static final int FORMAT = 1;

  /** What kind of field a venue is. */
  public enum Kind {
    /** The team's practice field: never valid with the field management system attached. */
    PRACTICE,
    /**
     * A team's shop: some of the game's tags hung wherever they fit, in the field's coordinates.
     * Its tags aren't judged against their official places, and it's never valid with the field
     * management system attached.
     */
    SHOP,
    /** An event's field, surveyed there. */
    EVENT,
    /** The official layout, as WPILib ships it. */
    OFFICIAL;

    /** Whether a venue of this kind is set aside while the field management system is attached. */
    public boolean offTheFieldOnly() {
      return this == PRACTICE || this == SHOP;
    }
  }

  public Venue {
    moved = Map.copyOf(moved);
  }

  /**
   * The venue's tags, set against the official layout it was measured against: each where that
   * layout puts it (its nominal pose) and, unless the venue is the official layout itself, where
   * the survey found it.
   *
   * @param base the official layout named by {@link #base}, as the game gives it
   * @throws IllegalArgumentException if the venue has a tag the base doesn't, which {@link
   *     VenueCheck} reports first
   */
  public List<Tag> tags(TagLayout base) {
    Map<Integer, Pose3d> official = new HashMap<>();
    for (Tag tag : base.tags()) {
      official.put(tag.id(), tag.nominal());
    }
    List<Tag> tags = new ArrayList<>();
    for (FieldTag tag : layout.getTags()) {
      Pose3d nominal = official.get(tag.getID());
      if (nominal == null) {
        throw new IllegalArgumentException(
            name + ": " + base.name() + " has no tag " + tag.getID());
      }
      if (kind == Kind.OFFICIAL) {
        // The official layout is nobody's survey: its tags are where it says, and no more.
        tags.add(Tag.nominal(tag.getID(), nominal));
      } else {
        tags.add(new Tag(tag.getID(), nominal, tag.getPose()));
      }
    }
    return tags;
  }

  /**
   * The venue's tags ({@link #tags}), with the field's size and the venue's name, and the official
   * layout's tag family and size: a survey measures where the tags stand, not what they are.
   */
  public TagLayout tagLayout(TagLayout base) {
    return new TagLayout(
        layout.getName(),
        layout.getFieldLength(),
        layout.getFieldWidth(),
        base.family(),
        base.tagSizeMeters(),
        tags(base));
  }

  /**
   * The file's SHA-256, in hex: what tells an uploaded venue from one in Git with the same name.
   */
  public String hash() {
    return hash(text);
  }

  /** A venue file's SHA-256, in hex, from its text as stored. */
  public static String hash(String text) {
    try {
      return HexFormat.of()
          .formatHex(
              MessageDigest.getInstance("SHA-256").digest(text.getBytes(StandardCharsets.UTF_8)));
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException("every Java has SHA-256", e);
    }
  }
}
