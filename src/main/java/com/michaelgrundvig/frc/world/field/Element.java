package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Something that stands on the field: a hub, a tower, a trench's arm, or something built into the
 * field's edge, such as a speaker or an outpost's face. Robots run into it and cameras can't see
 * through it, and it carries the AprilTags mounted on it. It is made of one or more {@link Block}s
 * in the field frame.
 *
 * <p>An element is what a robot runs into. What a robot drives over is terrain (a {@link Surface}),
 * and a thing is one or the other, never both: a tower's base plate is terrain, and the uprights
 * standing on it are part of the tower.
 *
 * <p>What reads it asks it, and works nothing out again: what it covers seen from above ({@link
 * #footprint}), how tall it is ({@link #heightMeters}), and which of it is in the way of something
 * at a given height ({@link #inTheWayOf}).
 *
 * @param name for the log and the views
 * @param blocks what it's made of, in the field frame
 * @param tags the AprilTags mounted on it, by ID
 */
public record Element(String name, List<Block> blocks, List<Integer> tags) {
  public Element {
    if (name.isBlank()) {
      throw new IllegalArgumentException("an element needs a name");
    }
    blocks = List.copyOf(blocks);
    if (blocks.isEmpty()) {
      throw new IllegalArgumentException(name + ": an element needs a block");
    }
    tags = List.copyOf(tags);
    Set<Integer> seen = new HashSet<>();
    for (int tag : tags) {
      boolean isANumberOfItsOwn = tag >= 0 && seen.add(tag);
      if (!isANumberOfItsOwn) {
        throw new IllegalArgumentException(
            name + ": its tags are numbers of zero or more, each once: " + tags);
      }
    }
  }

  /**
   * An element of one block: an outline on the floor, seen from above in the field frame, standing
   * up to a height.
   *
   * @param tags the AprilTags mounted on it
   */
  public static Element standing(
      String name, Polygon2d outline, double heightMeters, Material material, int... tags) {
    Block block = Block.standing(outline, heightMeters, material);
    return new Element(name, List.of(block), Arrays.stream(tags).boxed().toList());
  }

  /**
   * What it covers, seen from above: the smallest outline with no corner pointing in around all of
   * its blocks, whatever their heights. A view marks it with this, and a path stays clear of it by
   * this when it asks no more closely.
   */
  public Polygon2d footprint() {
    List<Translation2d> corners = new ArrayList<>();
    for (Block block : blocks) {
      corners.addAll(block.outline().corners());
    }
    return Polygon2d.hull(corners);
  }

  /** How tall it is: the top of its tallest block above the floor, in meters. */
  public double heightMeters() {
    double tallest = 0;
    for (Block block : blocks) {
      tallest = Math.max(tallest, block.topMeters());
    }
    return tallest;
  }

  /**
   * The outlines, seen from above, of what something between two heights runs into: each of its
   * blocks that reaches into those heights. A robot asks with its bumpers' bottom edge and top, and
   * gets a trench's column but not the bar it drives under.
   *
   * @param lowestMeters the underside of what is asking, above the floor
   * @param highestMeters its top
   */
  public List<Polygon2d> inTheWayOf(double lowestMeters, double highestMeters) {
    List<Polygon2d> outlines = new ArrayList<>();
    for (Block block : blocks) {
      if (block.inTheWayOf(lowestMeters, highestMeters)) {
        outlines.add(block.outline());
      }
    }
    return outlines;
  }

  /** How far a point in the field frame is from it, in meters: zero inside it or on it. */
  public double distance(Translation3d point) {
    double nearest = Double.POSITIVE_INFINITY;
    for (Block block : blocks) {
      nearest = Math.min(nearest, block.distance(point));
    }
    return nearest;
  }

  /** Whether a point in the field frame is inside it, or on it. */
  public boolean contains(Translation3d point) {
    return distance(point) == 0;
  }

  /**
   * Whether it stands between two points in the field frame, so that one can't be seen from the
   * other: whether any of its blocks does ({@link Block#standsBetween}).
   */
  public boolean standsBetween(
      double fromX, double fromY, double fromZ, double toX, double toY, double toZ) {
    for (int block = 0; block < blocks.size(); block++) {
      if (blocks.get(block).standsBetween(fromX, fromY, fromZ, toX, toY, toZ)) {
        return true;
      }
    }
    return false;
  }
}
