package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * One season's field: its size, AprilTags, and symmetry, the start positions, places, and zones
 * robot code refers to, the terrain robots drive over, and the elements that stand on it. Built
 * once at startup ({@link #builder}) from the field's tags (WPILib's official layout, or a venue's
 * survey of it) and never changed after. Everything is checked against the tags as the model is
 * built, so a wrong tag number fails at startup, and first in the tests.
 *
 * <p>Everything is written for the blue alliance, and {@link #forAlliance} is the one conversion to
 * red (see the package documentation). Tags pair with their partners on the other half by their
 * official poses, which the symmetry turns ({@link TagPartners}); where a survey found them never
 * changes which tag plays which part.
 *
 * <p>The model names the two copies of what's written for blue's half, a surface or an element,
 * "(blue)" and "(red)": for the half each is on, not for whose it is. A 2024 source stands by the
 * opposing alliance's wall, so "Source (blue)" is the red alliance's. Whose something is lives on
 * the places and zones robot code asks about ({@link Zone#ours}), which say it whichever half
 * they're on. What stands across the center line, on both halves at once (2025's barge), is written
 * once, whole ({@link Builder#shared}): it isn't mirrored, and keeps its name.
 *
 * <p>The walls around the field are the field's own {@link #outline}, not elements: no season
 * writes them, they carry no tags of their own, and nothing mirrors them. What's built into a wall
 * (a speaker, a pocket, an outpost's face) is an element like any other.
 */
public final class FieldModel {
  private final String name;
  private final double length;
  private final double width;
  private final Symmetry symmetry;
  private final Map<Integer, Tag> tags;
  private final TagPartners partners;
  private final List<Place> starts;
  private final List<Place> places;
  private final List<Zone> zones;
  private final Terrain terrain;
  private final List<Element> elements;
  private final Polygon2d outline;

  /**
   * A field on a WPILib layout, its tags as it gives them, with no terrain or elements.
   *
   * @param layout the field's size and AprilTags; the model keeps its own copy
   * @param symmetry how red's half relates to blue's
   * @param starts where robots start a match
   * @param places other named places on the floor: spots to line up on, to score or load from
   * @param zones named areas of the field
   * @throws IllegalArgumentException if a place or zone doesn't fit the layout (a tag that isn't in
   *     it, or has no partner on the other half), or two share a name
   */
  public FieldModel(
      Field layout, Symmetry symmetry, List<Place> starts, List<Place> places, List<Zone> zones) {
    this(layout, symmetry, starts, places, zones, List.of());
  }

  /**
   * A field on a WPILib layout, its tags as it gives them, with its terrain.
   *
   * @param surfaces the raised ground on the blue alliance's half, each named as blue's drivers
   *     would ("Left bump"): the model adds each one's partner on red's half, and names them
   *     "(blue)" and "(red)" for the half each is on
   */
  public FieldModel(
      Field layout,
      Symmetry symmetry,
      List<Place> starts,
      List<Place> places,
      List<Zone> zones,
      List<? extends Surface> surfaces) {
    this(
        builder(TagLayout.of(layout), symmetry)
            .starts(starts)
            .places(places)
            .zones(zones)
            .surfaces(surfaces));
  }

  /**
   * Starts describing the field on its tags: the official layout's, or a venue's ({@link
   * Venue#tagLayout}). Everything else is named as it's given, and anything left out is none.
   *
   * @param layout the field's size and tags
   * @param symmetry how red's half relates to blue's
   */
  public static Builder builder(TagLayout layout, Symmetry symmetry) {
    return new Builder(layout, symmetry);
  }

  /** A field being described, each list named as it's given. */
  public static final class Builder {
    private final TagLayout layout;
    private final Symmetry symmetry;
    private List<Place> starts = List.of();
    private List<Place> places = List.of();
    private List<Zone> zones = List.of();
    private List<? extends Surface> surfaces = List.of();
    private List<Element> elements = List.of();
    private List<Element> shared = List.of();

    private Builder(TagLayout layout, Symmetry symmetry) {
      this.layout = layout;
      this.symmetry = symmetry;
    }

    /** Where robots start a match. */
    public Builder starts(List<Place> starts) {
      this.starts = List.copyOf(starts);
      return this;
    }

    /** Other named places on the floor: spots to line up on, to score or load from. */
    public Builder places(List<Place> places) {
      this.places = List.copyOf(places);
      return this;
    }

    /** Named areas of the field. */
    public Builder zones(List<Zone> zones) {
      this.zones = List.copyOf(zones);
      return this;
    }

    /**
     * The raised ground on the blue alliance's half, each named as blue's drivers would ("Left
     * bump"): the model adds each one's partner on red's half, and names them "(blue)" and "(red)"
     * for the half each is on, with their crossings.
     */
    public Builder surfaces(List<? extends Surface> surfaces) {
      this.surfaces = List.copyOf(surfaces);
      return this;
    }

    /**
     * What stands on the blue alliance's half, carrying the tags on it, whichever alliance's it is:
     * the model adds each one's partner on red's half, carrying the partners of its tags, and names
     * them "(blue)" and "(red)" for the half each stands on.
     */
    public Builder elements(List<Element> elements) {
      this.elements = List.copyOf(elements);
      return this;
    }

    /**
     * What stands across the center line, both halves' at once, such as 2025's barge: each written
     * once, whole, and its own image under the field's symmetry, which the model checks. It may
     * carry tags on both halves, each with its partner, and keeps its name as it's given.
     */
    public Builder shared(List<Element> shared) {
      this.shared = List.copyOf(shared);
      return this;
    }

    /**
     * The field, checked.
     *
     * @throws IllegalArgumentException if anything doesn't fit the tags or the field (a tag that
     *     isn't in it, or has no partner on the other half; a surface or zone off the field; an
     *     element's block wholly off it; a shared element that isn't its own image), or two share a
     *     name
     */
    public FieldModel build() {
      return new FieldModel(this);
    }
  }

  private FieldModel(Builder field) {
    TagLayout layout = field.layout;
    name = layout.name();
    length = layout.lengthMeters();
    width = layout.widthMeters();
    symmetry = field.symmetry;
    Map<Integer, Tag> byId = new LinkedHashMap<>();
    for (Tag tag : layout.tags()) {
      byId.put(tag.id(), tag);
    }
    tags = Collections.unmodifiableMap(byId);
    partners = new TagPartners(tags.values(), symmetry, length, width);
    starts = field.starts;
    places = field.places;
    zones = field.zones;
    terrain = Terrain.onBothHalves(field.surfaces, symmetry, length, width);
    outline = Polygon2d.rectangle(Translation2d.ZERO, new Translation2d(length, width));
    elements = Elements.onBothHalves(field.elements, field.shared, this);
    FieldChecks.require(this);
    ElementChecks.require(this, field.shared);
  }

  /** The layout's name, such as "2026 FRC Rebuilt Welded". */
  public String name() {
    return name;
  }

  /** Meters, along x: from the blue alliance wall to the red. */
  public double length() {
    return length;
  }

  /** Meters, along y. */
  public double width() {
    return width;
  }

  public Symmetry symmetry() {
    return symmetry;
  }

  /** Every tag, by ID: where the official layout puts it, and where a survey found it. */
  public Map<Integer, Tag> tags() {
    return tags;
  }

  /**
   * The tags where they are, as WPILib's {@link Field}, for tools that take one, such as a vision
   * simulation: each as surveyed, if it was.
   */
  public Field toField() {
    List<FieldTag> whereTheyAre = new ArrayList<>();
    for (Tag tag : tags.values()) {
      whereTheyAre.add(new FieldTag(tag.id(), tag.pose()));
    }
    return new Field(name, "", "", null, length, width, "frc", whereTheyAre);
  }

  /** Where a tag is, in the field frame, if the field has it: as surveyed, if it was. */
  public Optional<Pose3d> tag(int id) {
    return Optional.ofNullable(tags.get(id)).map(Tag::pose);
  }

  /** Where robots start a match, in the order they were given. */
  public List<Place> starts() {
    return starts;
  }

  /** The named places other than the starts. */
  public List<Place> places() {
    return places;
  }

  public List<Zone> zones() {
    return zones;
  }

  /** The raised ground robots drive over, on both halves of the field. */
  public Terrain terrain() {
    return terrain;
  }

  /**
   * What stands on the field, on both halves, each carrying its tags: named "(blue)" or "(red)" for
   * the half it stands on, and what's shared across the center line, under its own name. Robots run
   * into them; what robots drive over is the {@link #terrain}.
   */
  public List<Element> elements() {
    return elements;
  }

  /**
   * The inside faces of the walls around the field, seen from above: the layout's rectangle,
   * counterclockwise from the corner at the field's origin. A robot stays inside it, as it stays
   * outside each element's {@link Element#footprint}.
   */
  public Polygon2d outline() {
    return outline;
  }

  /**
   * A pose written for the blue alliance, for an alliance: the same pose for blue, and its image
   * under the field's symmetry for red.
   */
  public Pose2d forAlliance(Alliance alliance, Pose2d blue) {
    return switch (alliance) {
      case BLUE -> blue;
      case RED -> symmetry.apply(blue, length, width);
    };
  }

  /**
   * A pose in 3D written for the blue alliance, for an alliance: its image under the field's
   * symmetry for red ({@link Symmetry#apply(Pose3d, double, double)}).
   */
  public Pose3d forAlliance(Alliance alliance, Pose3d blue) {
    return switch (alliance) {
      case BLUE -> blue;
      case RED -> symmetry.apply(blue, length, width);
    };
  }

  /** A point written for the blue alliance, for an alliance. */
  public Translation2d forAlliance(Alliance alliance, Translation2d blue) {
    return switch (alliance) {
      case BLUE -> blue;
      case RED -> symmetry.apply(blue, length, width);
    };
  }

  /** An area written for the blue alliance, for an alliance. */
  public Polygon2d forAlliance(Alliance alliance, Polygon2d blue) {
    return switch (alliance) {
      case BLUE -> blue;
      case RED -> symmetry.apply(blue, length, width);
    };
  }

  /**
   * Which way is straight ahead for an alliance's drivers: +x for blue, -x for red. Always half a
   * turn apart, whatever the field's symmetry, because the driver stations face each other; so
   * field-relative driving turns the sticks by this, never by the symmetry.
   */
  public static Rotation2d driverForward(Alliance alliance) {
    return switch (alliance) {
      case BLUE -> Rotation2d.ZERO;
      case RED -> Rotation2d.PI;
    };
  }

  /**
   * The tag that plays a blue tag's part for an alliance: the tag itself for blue, and for red the
   * tag the symmetry puts in its place, found in the layout (so as measured, when it was).
   *
   * @throws IllegalArgumentException if the layout has no such tag, or no partner for it
   */
  public int tagFor(Alliance alliance, int blueTag) {
    if (!tags.containsKey(blueTag)) {
      throw new IllegalArgumentException("the layout has no tag " + blueTag);
    }
    if (alliance == Alliance.BLUE) {
      return blueTag;
    }
    Optional<Integer> partner = partners.partnerOf(blueTag);
    if (partner.isEmpty()) {
      throw new IllegalArgumentException(
          "tag " + blueTag + " has no partner on the red alliance's half: is the symmetry right?");
    }
    return partner.get();
  }

  /** The tag the symmetry pairs a tag with, on the other alliance's half, if there is one. */
  Optional<Integer> partnerOf(int tag) {
    return partners.partnerOf(tag);
  }

  /** On the floor, where {@link #tagFor} puts a blue tag for an alliance. */
  Pose2d tagPose(Alliance alliance, int blueTag) {
    return tagPose3d(alliance, blueTag).toPose2d();
  }

  /** Where {@link #tagFor} puts a blue tag for an alliance: as surveyed, if it was. */
  Pose3d tagPose3d(Alliance alliance, int blueTag) {
    // The tag it gives is always one of the layout's: the tag itself, or its partner.
    return tag(tagFor(alliance, blueTag)).orElseThrow();
  }

  /** Whether a point is on the field, within the walls. */
  public boolean contains(Translation2d point) {
    return point.getX() >= 0
        && point.getX() <= length
        && point.getY() >= 0
        && point.getY() <= width;
  }

  /**
   * How far the official layout is from the declared symmetry: the largest distance between a tag's
   * image and the nearest tag facing the same way, in meters. Near zero when the field has the
   * symmetry; worth a test, since every place on the red alliance depends on it.
   */
  public double symmetryError() {
    return partners.symmetryError();
  }
}
