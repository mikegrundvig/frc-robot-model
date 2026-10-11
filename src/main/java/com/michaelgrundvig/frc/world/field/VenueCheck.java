package com.michaelgrundvig.frc.world.field;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.TreeSet;
import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;

/**
 * Whether a venue fits the game's field, checked before the robot stores or uses it: every problem
 * at once, in words for the pit. The bounds catch a wrong file or a typo, not a careful survey's
 * small corrections.
 *
 * <p>An instance exists for the length of one check: it holds the venue, the official layout, and
 * the problems found so far.
 */
public final class VenueCheck {
  /**
   * How far a tag may sit from its official place with no reason given: four inches, more than any
   * field's tolerance and less than a tag mistaken for its neighbor.
   */
  public static final double MAX_SHIFT_METERS = 0.10;

  /** How far a tag may turn from its official orientation with no reason given. */
  public static final double MAX_TURN_DEGREES = 5;

  /** How closely an official venue's tags must be where WPILib's layout puts them. */
  static final double OFFICIAL_METERS = 0.001;

  /** How closely an official venue's tags must face the way WPILib's layout turns them. */
  static final double OFFICIAL_RADIANS = 0.001;

  /** How closely the venue's field must match the official one's length and width. */
  private static final double FIELD_SIZE_METERS = 0.01;

  private final Venue venue;
  private final Field base;

  /** Where the official layout puts each tag, by the tag's ID. */
  private final Map<Integer, Pose3d> official;

  /** Where the venue's layout has each tag, by the tag's ID. */
  private final Map<Integer, Pose3d> measured;

  private final List<String> problems = new ArrayList<>();

  private VenueCheck(Venue venue, Field base) {
    this.venue = venue;
    this.base = base;
    official = posesById(base);
    measured = posesById(venue.layout());
  }

  /**
   * What's wrong with a venue against the game's official layout; empty when it fits.
   *
   * @param base the official layout of the field variant the venue says it was measured against
   * @param symmetry the game's, which must still pair every tag with its partner
   */
  public static List<String> problems(Venue venue, Field base, Symmetry symmetry) {
    VenueCheck check = new VenueCheck(venue, base);
    check.itWasMeasuredAgainstThisLayout();
    check.theFieldIsTheSameSize();
    check.itHasTheSameTags();
    for (int id : new TreeSet<>(check.measured.keySet())) {
      check.theTagIsNearItsOfficialPlace(id);
    }
    check.movedNamesOnlyTagsItHas();
    check.tagsStillPairUp(symmetry);
    return check.problems;
  }

  private static Map<Integer, Pose3d> posesById(Field layout) {
    Map<Integer, Pose3d> poses = new HashMap<>();
    for (FieldTag tag : layout.getTags()) {
      poses.put(tag.getID(), tag.getPose());
    }
    return poses;
  }

  private void itWasMeasuredAgainstThisLayout() {
    if (venue.base().equals(base.getName())) {
      return;
    }
    problems.add(
        "measured against \""
            + venue.base()
            + "\", but the field's official layout is \""
            + base.getName()
            + "\"");
  }

  private void theFieldIsTheSameSize() {
    Field layout = venue.layout();
    double lengthOff = Math.abs(layout.getFieldLength() - base.getFieldLength());
    double widthOff = Math.abs(layout.getFieldWidth() - base.getFieldWidth());
    if (lengthOff > FIELD_SIZE_METERS || widthOff > FIELD_SIZE_METERS) {
      problems.add(
          String.format(
              Locale.ROOT,
              "the field is %.3f x %.3f m, the official one %.3f x %.3f m",
              layout.getFieldLength(),
              layout.getFieldWidth(),
              base.getFieldLength(),
              base.getFieldWidth()));
    }
  }

  /** The venue has every tag the official layout has, and no others. */
  private void itHasTheSameTags() {
    Set<Integer> missing = new TreeSet<>(official.keySet());
    missing.removeAll(measured.keySet());
    if (!missing.isEmpty()) {
      problems.add("the layout is missing tags " + missing);
    }
    Set<Integer> extra = new TreeSet<>(measured.keySet());
    extra.removeAll(official.keySet());
    if (!extra.isEmpty()) {
      problems.add("the official layout has no tags " + extra);
    }
  }

  /**
   * A tag is where the official layout puts it, as nearly as its kind of venue asks. An official
   * venue is WPILib's layout itself, so its tags must match almost exactly. A surveyed venue's tags
   * may be a little off, and further off than that only with a reason given under "moved".
   *
   * <p>A tag the official layout doesn't have is not judged here: it is already reported as extra.
   */
  private void theTagIsNearItsOfficialPlace(int id) {
    Pose3d pose = measured.get(id);
    Pose3d nominal = official.get(id);
    if (pose == null || nominal == null) {
      return;
    }
    double shiftMeters = pose.getTranslation().getDistance(nominal.getTranslation());
    double turnDegrees = Math.toDegrees(angleBetween(pose.getRotation().getQuaternion(), nominal));
    if (venue.kind() == Venue.Kind.OFFICIAL) {
      boolean differs =
          shiftMeters > OFFICIAL_METERS || turnDegrees > Math.toDegrees(OFFICIAL_RADIANS);
      if (differs) {
        problems.add(
            String.format(
                Locale.ROOT,
                "an official venue, but tag %d sits %.1f mm and %.2f° from WPILib's layout",
                id,
                shiftMeters * 1000,
                turnDegrees));
      }
      return;
    }
    boolean farOff = shiftMeters > MAX_SHIFT_METERS || turnDegrees > MAX_TURN_DEGREES;
    boolean reasonGiven = venue.moved().containsKey(id);
    if (farOff && !reasonGiven) {
      problems.add(
          String.format(
              Locale.ROOT,
              "tag %d sits %.3f m and %.1f° from its official place: if that's right, say why"
                  + " under \"moved\"",
              id,
              shiftMeters,
              turnDegrees));
    }
  }

  /**
   * The angle a tag as measured is turned from its official orientation, in radians, from 0 to pi.
   *
   * <p>The rotation that takes one orientation to the other is the first's quaternion times the
   * inverse of the second's, and a rotation by an angle has a quaternion whose W is the cosine of
   * half that angle. A quaternion and its negative are the same rotation, so W's sign is dropped;
   * rounding can leave it a hair over 1, which has no arc cosine, so it is held to 1.
   */
  private static double angleBetween(Quaternion measured, Pose3d nominal) {
    Quaternion difference = measured.times(nominal.getRotation().getQuaternion().inverse());
    return 2 * Math.acos(Math.min(1, Math.abs(difference.getW())));
  }

  /** Every tag "moved" gives a reason for is a tag the venue's layout has. */
  private void movedNamesOnlyTagsItHas() {
    Set<Integer> unknown = new TreeSet<>();
    for (int id : venue.moved().keySet()) {
      if (!measured.containsKey(id)) {
        unknown.add(id);
      }
    }
    if (!unknown.isEmpty()) {
      problems.add("moved names tags the layout doesn't have: " + unknown);
    }
  }

  /**
   * Every tag the official layout pairs with a partner under the symmetry is still paired with the
   * same one: places on red's half come from red's tags as measured, through these pairs.
   *
   * <p>A tag the venue doesn't have at all is not reported here: it is already reported as missing.
   */
  private void tagsStillPairUp(Symmetry symmetry) {
    FieldModel officialField = new FieldModel(base, symmetry, List.of(), List.of(), List.of());
    FieldModel measuredField;
    try {
      measuredField = new FieldModel(venue.layout(), symmetry, List.of(), List.of(), List.of());
    } catch (IllegalArgumentException e) {
      problems.add("the layout doesn't make a field: " + e.getMessage());
      return;
    }
    for (int id : new TreeSet<>(officialField.tags().keySet())) {
      Optional<Integer> officialPartner = officialField.partnerOf(id);
      boolean toBeJudged = officialPartner.isPresent() && measured.containsKey(id);
      if (!toBeJudged) {
        continue;
      }
      int partner = officialPartner.get();
      Optional<Integer> partnerNow = measuredField.partnerOf(id);
      if (partnerNow.isEmpty()) {
        problems.add(
            "tag " + id + " no longer pairs with tag " + partner + " under the field's symmetry");
      } else if (partnerNow.get() != partner) {
        problems.add(
            "tag "
                + id
                + " now pairs with tag "
                + partnerNow.get()
                + " under the field's symmetry, not "
                + partner);
      }
    }
  }
}
