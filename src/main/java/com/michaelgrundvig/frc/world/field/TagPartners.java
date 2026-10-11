package com.michaelgrundvig.frc.world.field;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;

/**
 * Which tag plays each tag's part on the other alliance's half of the field.
 *
 * <p>A tag's partner is found from the official layout alone. The field's symmetry is applied to
 * the tag's official pose, which gives the pose its partner should have: its image. The partner is
 * the tag nearest that image among those facing the way the image faces, provided it is close
 * enough. Where a survey found the tags plays no part, so a venue's corrections never change which
 * tag is whose partner.
 *
 * <p>Everything is judged on the floor: a tag's height and tilt are ignored.
 */
final class TagPartners {
  /**
   * How close a tag's image under the symmetry must be to a tag facing the same way for the two to
   * be partners: room for a layout measured at an event, and well under the spacing of different
   * tags.
   */
  static final double PARTNER_DISTANCE_METERS = 0.25;

  /** How closely a partner must face the way the symmetry turns the tag. */
  static final double PARTNER_ANGLE_RADIANS = Math.toRadians(20);

  private final List<Tag> tags;
  private final Symmetry symmetry;
  private final double fieldLength;
  private final double fieldWidth;

  /** Each tag's partner, by their IDs. A tag with no partner is left out. */
  private final Map<Integer, Integer> partners;

  /**
   * @param tags every tag of the field
   * @param symmetry how the red alliance's half relates to the blue alliance's
   * @param fieldLength the field's length, along x, in meters
   * @param fieldWidth the field's width, along y, in meters
   */
  TagPartners(Collection<Tag> tags, Symmetry symmetry, double fieldLength, double fieldWidth) {
    this.tags = List.copyOf(tags);
    this.symmetry = symmetry;
    this.fieldLength = fieldLength;
    this.fieldWidth = fieldWidth;
    Map<Integer, Integer> found = new LinkedHashMap<>();
    for (Tag tag : this.tags) {
      Pose2d image = imageOf(tag);
      Optional<Tag> nearest = nearestFacing(image);
      boolean closeEnough =
          nearest.isPresent() && distance(nearest.get(), image) <= PARTNER_DISTANCE_METERS;
      if (closeEnough) {
        found.put(tag.id(), nearest.get().id());
      }
    }
    partners = Map.copyOf(found);
  }

  /** The ID of a tag's partner, if it has one. */
  Optional<Integer> partnerOf(int tag) {
    return Optional.ofNullable(partners.get(tag));
  }

  /**
   * How far the official layout is from having the symmetry: over every tag, the largest distance
   * from the tag's image to the nearest tag facing the way the image does, in meters. It is
   * infinite if some tag's image has no tag facing its way at all.
   */
  double symmetryError() {
    double worst = 0;
    for (Tag tag : tags) {
      Pose2d image = imageOf(tag);
      Optional<Tag> nearest = nearestFacing(image);
      double error = Double.POSITIVE_INFINITY;
      if (nearest.isPresent()) {
        error = distance(nearest.get(), image);
      }
      worst = Math.max(worst, error);
    }
    return worst;
  }

  /** Where the symmetry puts a tag's official pose on the other half, on the floor. */
  private Pose2d imageOf(Tag tag) {
    return symmetry.apply(tag.nominal().toPose2d(), fieldLength, fieldWidth);
  }

  /**
   * The tag nearest a pose among those whose official pose faces within {@link
   * #PARTNER_ANGLE_RADIANS} of the way the pose faces. Of two equally near, it is the one listed
   * first.
   */
  private Optional<Tag> nearestFacing(Pose2d pose) {
    Optional<Tag> nearest = Optional.empty();
    double nearestDistance = Double.POSITIVE_INFINITY;
    for (Tag tag : tags) {
      Pose2d official = tag.nominal().toPose2d();
      double turnBetween = Math.abs(official.getRotation().minus(pose.getRotation()).getRadians());
      boolean facesTheSameWay = turnBetween <= PARTNER_ANGLE_RADIANS;
      double distance = official.getTranslation().getDistance(pose.getTranslation());
      if (facesTheSameWay && distance < nearestDistance) {
        nearest = Optional.of(tag);
        nearestDistance = distance;
      }
    }
    return nearest;
  }

  /** How far a tag's official pose is from a pose, on the floor, in meters. */
  private static double distance(Tag tag, Pose2d pose) {
    return tag.nominal().toPose2d().getTranslation().getDistance(pose.getTranslation());
  }
}
