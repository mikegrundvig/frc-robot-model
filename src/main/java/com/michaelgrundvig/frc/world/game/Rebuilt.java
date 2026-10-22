package com.michaelgrundvig.frc.world.game;

import static com.michaelgrundvig.frc.world.game.FieldShapes.box;
import static com.michaelgrundvig.frc.world.game.Manuals.IN;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.world.field.Block;
import com.michaelgrundvig.frc.world.field.Bump;
import com.michaelgrundvig.frc.world.field.Element;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Platform;
import com.michaelgrundvig.frc.world.field.Start;
import com.michaelgrundvig.frc.world.field.Surface;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import com.michaelgrundvig.frc.world.field.Zone;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The field the robot plays on in 2026, REBUILT.
 *
 * <p>Everything here is written as if the robot were on the blue alliance, in meters in WPILib's
 * field frame (see {@code com.michaelgrundvig.frc.world.field}); the field model works out the red
 * alliance's half, turned half a turn. Places near AprilTags are derived from them, so a layout
 * measured at an event moves them for both alliances. Sizes come from the game manual's section 5,
 * and where things stand from WPILib's 2026 tag layout. The two exceptions say so where they're
 * used: the tower's uprights and the depot, which no tag marks, are placed from maple-sim's arena.
 *
 * <p>What robots run into is an element ({@link #elements}), and what they drive over is terrain
 * ({@link #surfaces}).
 *
 * <p>Every figure here is the manual's nominal one. The official drawings hold the tolerances, and
 * a venue's survey holds where an event's tags really are.
 *
 * <p>Each season's field is written once, when the game is revealed, and a new version of the model
 * has it: see docs/kickoff.md.
 */
public final class Rebuilt {
  private Rebuilt() {}

  /**
   * What 2026's manual allows a robot: a 110 in frame perimeter and 30 in tall at the start (R104),
   * bumpers within 2.5 to 5.75 in of the floor (R405), and 135 lb with bumpers and without its
   * battery (R408), a battery with its leads weighing about 14.5 lb (R601). Placeholders until the
   * next kickoff. The extension limit is a guess: 2026's wasn't read (2024 allowed 12 in).
   */
  public static final SeasonLimits LIMITS =
      SeasonLimits.builder()
          .framePerimeter(Inches.of(110))
          .startingHeight(Inches.of(30))
          .extension(Inches.of(12))
          .bumperZoneBottom(Inches.of(2.5))
          .bumperZoneTop(Inches.of(5.75))
          .weight(Pounds.of(135))
          .battery(Pounds.of(14.5))
          .build();

  /** The welded field. The AndyMark build is 2 cm smaller, with its own layout. */
  static final Fields LAYOUT = Fields.FRC_2026_REBUILT_WELDED;

  /** Red's half is blue's turned half a turn: every tag lands on a partner within a millimeter. */
  static final Symmetry SYMMETRY = Symmetry.ROTATED;

  private static final double WIDTH = LAYOUT.width;

  // ---- Materials, where the manual names them. Contact numbers come later, per pair. ----

  /** The bumps' ramps: half-inch HDPE with an orange-peel texture (5.5). */
  static final Material HDPE = Material.builder().name("HDPE, orange peel").build();

  /** The tower, and the depot's barriers: powder-coated steel (5.7, 5.8). */
  static final Material STEEL = Material.builder().name("powder-coated steel").build();

  /**
   * What the manual doesn't name a material for: the hub's and the trenches' faces, and the
   * alliance wall's, the tower's and the outpost's.
   */
  static final Material STRUCTURE = Material.builder().name("field structure").build();

  // ---- The hub, the bumps beside it, and the trenches beyond them, across the field ----

  /** The robot starting line, the alliance zone's edge, runs along the hub's near face (tag 26). */
  static final double STARTING_LINE_X = 4.022;

  /** The hub's center line, midway between the tags on its near and far faces (26 and 20). */
  static final double HUB_X = 4.626;

  /** The hub's footprint: 47 in square (5.4). */
  static final double HUB_SIZE = 47.0 * IN;

  /**
   * The front edge of the hub's opening, 72 in up (5.4). The funnel and net above it aren't drawn.
   */
  static final double HUB_HEIGHT = 72.0 * IN;

  // Each bump, beside the hub: 73.0 in wide, 44.4 in deep, and 6.513 in tall, a ridge with 15°
  // ramps down toward both zones, its crest on the hub's center line (5.5).
  static final double BUMP_WIDTH = 1.854;
  static final double BUMP_DEPTH = 1.128;
  static final double BUMP_HEIGHT = 0.1654;

  /** Where a bump ends at the trench, across the field: past the hub's half and a bump's width. */
  private static final double BUMP_EDGE = WIDTH / 2 - HUB_SIZE / 2 - BUMP_WIDTH;

  /** Each trench, beside the hub: 47 in deep, with a 50.34 in opening from the guardrail (5.6). */
  static final double TRENCH_DEPTH = 1.194;

  static final double TRENCH_OPENING = 1.279;

  /** A trench's height: 40.25 in (5.6). */
  static final double TRENCH_HEIGHT = 40.25 * IN;

  /** The clear space under a trench's arm: 22.25 in tall (5.6). */
  static final double TRENCH_CLEARANCE = 22.25 * IN;

  /**
   * The arm's crossbar, front to back: between the faces its two tags hang on (17 and 28, at x
   * 4.588 and 4.663). The rest of the arm's frame isn't in the manual, so it isn't drawn.
   */
  static final double TRENCH_BAR_DEPTH = 0.075;

  /** The bump on the drivers' left of their hub, between it and the trench. */
  public static final Bump LEFT_BUMP =
      new Bump(
          "Left bump",
          new Translation2d(HUB_X, WIDTH / 2 + HUB_SIZE / 2),
          new Translation2d(HUB_X, WIDTH / 2 + HUB_SIZE / 2 + BUMP_WIDTH),
          BUMP_HEIGHT,
          BUMP_DEPTH,
          HDPE);

  /** The bump on the drivers' right of their hub. */
  public static final Bump RIGHT_BUMP =
      new Bump(
          "Right bump",
          new Translation2d(HUB_X, BUMP_EDGE),
          new Translation2d(HUB_X, WIDTH / 2 - HUB_SIZE / 2),
          BUMP_HEIGHT,
          BUMP_DEPTH,
          HDPE);

  // ---- The tower, built into the alliance wall (5.8) ----

  /** The tower's center line across the field: tag 31's, the tower wall's center tag. */
  static final double TOWER_Y = 3.746;

  /** The base: a steel plate 39.0 in wide and 45.18 in out from the wall, edges about 0.25 in. */
  static final double BASE_WIDTH = 39.0 * IN;

  static final double BASE_DEPTH = 45.18 * IN;
  static final double BASE_HEIGHT = 0.25 * IN;

  /**
   * How far the uprights' middles stand out from the alliance wall: 41.8 in, from maple-sim's arena
   * ({@code Arena2026Rebuilt} at 1b1e8ea), which no tag or manual figure gives. Their field faces
   * land 43.5 in out, where the tower spot's test puts the tower's face.
   */
  static final double UPRIGHT_X = 1.062;

  /** Each upright: a sheet metal box 3.5 in deep and 1.5 in thick, 72.1 in tall above the base. */
  static final double UPRIGHT_DEPTH = 3.5 * IN;

  static final double UPRIGHT_THICKNESS = 1.5 * IN;
  static final double UPRIGHT_HEIGHT = 72.1 * IN;

  /** The clear gap between the uprights. */
  static final double UPRIGHT_GAP = 32.25 * IN;

  /** The rungs' centers off the floor: low, mid and high, 18 in apart. */
  static final List<Double> RUNG_HEIGHTS = List.of(27.0 * IN, 45.0 * IN, 63.0 * IN);

  /**
   * A rung: 1-1/4 in Sch 40 pipe, 1.66 in across, reaching 5.875 in past each upright. A block
   * can't be round, so each is drawn as a square bar that far across.
   */
  static final double RUNG_DIAMETER = 1.66 * IN;

  static final double RUNG_OVERHANG = 5.875 * IN;

  /**
   * The braces from each upright back to the tower wall, about 28.40 to 43.38 in off the floor. The
   * manual gives only that span, so each is drawn as a solid block an upright thick.
   */
  static final double BRACE_BOTTOM = 28.40 * IN;

  static final double BRACE_TOP = 43.38 * IN;

  /**
   * The tower as a whole: 49.25 in wide and 78.25 in tall. Its wall, which the manual counts as
   * part of it and its tags (31 and 32) hang on, is drawn that wide and tall: the manual gives no
   * size for the wall alone.
   */
  static final double TOWER_WIDTH = 49.25 * IN;

  static final double TOWER_HEIGHT = 78.25 * IN;

  /**
   * The face of the alliance wall where its tags hang, the tower's and the outpost's: 0.008 m in
   * from the field's edge, as WPILib's layout puts tags 29 to 32. Each part of the wall is drawn
   * from the field's edge to this face; what's behind it no robot reaches.
   */
  static final double WALL_FACE_X = 0.008;

  // ---- The outpost, in the alliance wall by the guardrail (5.3) ----

  /**
   * How far across the field the outpost reaches from the guardrail: the outpost area's 71.0 in,
   * which it bounds on the field's side (5.3). The manual gives no width for the outpost itself.
   */
  static final double OUTPOST_WIDTH = 71.0 * IN;

  /**
   * The top of the outpost's upper opening: its bottom 28.1 in up, and 7.0 in tall. The outpost is
   * drawn up to there: how much higher it stands isn't in the manual's text.
   */
  static final double OUTPOST_HEIGHT = (28.1 + 7.0) * IN;

  // ---- The depot, along the alliance wall (5.7) ----

  /** The depot: 42.0 in wide and 27.0 in deep. */
  static final double DEPOT_WIDTH = 42.0 * IN;

  static final double DEPOT_DEPTH = 27.0 * IN;

  /** Its barriers: steel 3.0 in wide and 1.0 in tall, 1.125 in with the hook fastener under it. */
  static final double BARRIER_WIDTH = 3.0 * IN;

  static final double BARRIER_HEIGHT = 1.125 * IN;

  /**
   * The depot's center across the field: centered on maple-sim's depot fuel ({@code
   * Arena2026Rebuilt} at 1b1e8ea), not checked against the field drawings. Its barriers run along
   * its three open sides, which is an assumption too: the manual's text doesn't say.
   */
  static final double DEPOT_Y = 5.908;

  /** The center of the hub, where shots go: midway between the tags on its near and far faces. */
  public static final Place HUB = Place.between("Hub", 26, 20);

  /** Squared up to the tower's center tag, just clear of its front face: where a climb lines up. */
  public static final Place TOWER = Place.facing("Tower", 31, 1.6);

  /** From the alliance wall to the starting line, wall to wall. */
  public static final Zone ALLIANCE_ZONE =
      Zone.ours(
          "Alliance zone",
          Polygon2d.rectangle(Translation2d.ZERO, new Translation2d(STARTING_LINE_X, WIDTH)));

  /** Under a trench's 22.25 in bar, beside either hub. */
  public static final Zone TRENCH =
      Zone.both(
          "Trench",
          Polygon2d.rectangle(
              new Translation2d(HUB_X - TRENCH_DEPTH / 2, WIDTH - TRENCH_OPENING),
              new Translation2d(HUB_X + TRENCH_DEPTH / 2, WIDTH)),
          Polygon2d.rectangle(
              new Translation2d(HUB_X - TRENCH_DEPTH / 2, 0),
              new Translation2d(HUB_X + TRENCH_DEPTH / 2, TRENCH_OPENING)));

  /**
   * What robots drive over on blue's half: the bumps, the tower's base plate, and the depot's
   * barriers. Each vertical edge is a step a wheel climbs, not a ramp.
   */
  static List<Surface> surfaces() {
    // The depot's outside edges. Its front barrier lies across the whole width, and the two side
    // barriers run from the wall out to the front barrier's inside edge.
    double depotFront = DEPOT_DEPTH - BARRIER_WIDTH;
    double depotLeft = DEPOT_Y + DEPOT_WIDTH / 2;
    double depotRight = DEPOT_Y - DEPOT_WIDTH / 2;
    return List.of(
        LEFT_BUMP,
        RIGHT_BUMP,
        plate(
            "Tower base",
            0,
            BASE_DEPTH,
            TOWER_Y - BASE_WIDTH / 2,
            TOWER_Y + BASE_WIDTH / 2,
            BASE_HEIGHT,
            STEEL),
        plate(
            "Depot barrier, front",
            depotFront,
            DEPOT_DEPTH,
            depotRight,
            depotLeft,
            BARRIER_HEIGHT,
            STEEL),
        plate(
            "Depot barrier, left",
            0,
            depotFront,
            depotLeft - BARRIER_WIDTH,
            depotLeft,
            BARRIER_HEIGHT,
            STEEL),
        plate(
            "Depot barrier, right",
            0,
            depotFront,
            depotRight,
            depotRight + BARRIER_WIDTH,
            BARRIER_HEIGHT,
            STEEL));
  }

  /**
   * What stands on blue's half, and robots run into: the hub, the trenches, the tower, and the
   * outpost. The bumps, the tower's base plate and the depot's barriers are driven over, so they
   * are terrain ({@link #surfaces}), not elements.
   */
  static List<Element> elements() {
    Polygon2d hub =
        Polygon2d.rectangle(new Pose2d(HUB_X, WIDTH / 2, Rotation2d.ZERO), HUB_SIZE, HUB_SIZE);
    return List.of(
        Element.standing("Hub", hub, HUB_HEIGHT, STRUCTURE, 18, 19, 20, 21, 24, 25, 26, 27),
        trench("Right trench", 0, TRENCH_OPENING, BUMP_EDGE, 17, 28),
        trench("Left trench", WIDTH, WIDTH - TRENCH_OPENING, WIDTH - BUMP_EDGE, 22, 23),
        tower(),
        outpost());
  }

  /**
   * A trench: its column, full height, between the opening and the bump; and its arm's crossbar,
   * carrying the tags, over the opening from the clearance up.
   *
   * @param guardrail where the opening starts across the field
   * @param opening where it ends, at the column
   * @param bump where the column meets the bump
   */
  private static Element trench(
      String name, double guardrail, double opening, double bump, Integer... tags) {
    double front = HUB_X - TRENCH_DEPTH / 2;
    double back = HUB_X + TRENCH_DEPTH / 2;
    Block column = box(front, back, opening, bump, 0, TRENCH_HEIGHT, STRUCTURE);
    Block crossbar =
        box(
            HUB_X - TRENCH_BAR_DEPTH / 2,
            HUB_X + TRENCH_BAR_DEPTH / 2,
            guardrail,
            opening,
            TRENCH_CLEARANCE,
            TRENCH_HEIGHT,
            STRUCTURE);
    return new Element(name, List.of(column, crossbar), List.of(tags));
  }

  /**
   * The outpost, from the guardrail along the alliance wall, carrying its tags (29 and 30). Drawn
   * as its face in the wall: its openings, the chute's and the corral's, aren't cut out of it.
   */
  private static Element outpost() {
    Block face = box(0, WALL_FACE_X, 0, OUTPOST_WIDTH, 0, OUTPOST_HEIGHT, STRUCTURE);
    return new Element("Outpost", List.of(face), List.of(29, 30));
  }

  /**
   * The tower: its wall, carrying its tags (31 and 32); two uprights on its base, a brace from each
   * back to the wall; and three rungs across them.
   */
  private static Element tower() {
    List<Block> blocks = new ArrayList<>();
    blocks.add(
        box(
            0,
            WALL_FACE_X,
            TOWER_Y - TOWER_WIDTH / 2,
            TOWER_Y + TOWER_WIDTH / 2,
            0,
            TOWER_HEIGHT,
            STRUCTURE));
    double uprightFront = UPRIGHT_X + UPRIGHT_DEPTH / 2;
    double uprightBack = UPRIGHT_X - UPRIGHT_DEPTH / 2;
    double fromTowersMiddle = UPRIGHT_GAP / 2 + UPRIGHT_THICKNESS / 2;
    for (double y : new double[] {TOWER_Y - fromTowersMiddle, TOWER_Y + fromTowersMiddle}) {
      double rightSide = y - UPRIGHT_THICKNESS / 2;
      double leftSide = y + UPRIGHT_THICKNESS / 2;
      blocks.add(
          box(
              uprightBack,
              uprightFront,
              rightSide,
              leftSide,
              BASE_HEIGHT,
              BASE_HEIGHT + UPRIGHT_HEIGHT,
              STEEL));
      blocks.add(box(0, uprightBack, rightSide, leftSide, BRACE_BOTTOM, BRACE_TOP, STEEL));
    }
    double rungLength = UPRIGHT_GAP + 2 * UPRIGHT_THICKNESS + 2 * RUNG_OVERHANG;
    for (double height : RUNG_HEIGHTS) {
      blocks.add(
          box(
              UPRIGHT_X - RUNG_DIAMETER / 2,
              UPRIGHT_X + RUNG_DIAMETER / 2,
              TOWER_Y - rungLength / 2,
              TOWER_Y + rungLength / 2,
              height - RUNG_DIAMETER / 2,
              height + RUNG_DIAMETER / 2,
              STEEL));
    }
    return new Element("Tower", blocks, List.of(31, 32));
  }

  /**
   * A flat plate lying on the floor, its sides along the field's axes and a vertical edge all
   * round: a platform with no ramps.
   *
   * @param nearX its edge nearer the blue alliance wall, in meters along x
   * @param farX its edge farther from that wall
   * @param rightY its edge to the blue drivers' right, in meters along y
   * @param leftY its edge to their left
   * @param heightMeters how thick it is
   */
  private static Platform plate(
      String name,
      double nearX,
      double farX,
      double rightY,
      double leftY,
      double heightMeters,
      Material material) {
    Pose2d center = new Pose2d((nearX + farX) / 2, (rightY + leftY) / 2, Rotation2d.ZERO);
    Polygon2d top = Polygon2d.rectangle(center, Math.abs(farX - nearX), Math.abs(leftY - rightY));
    List<Platform.Side> edgeAllRound = Collections.nCopies(4, Platform.Side.EDGE);
    return new Platform(name, top, heightMeters, edgeAllRound, material);
  }

  /**
   * Where a robot starts, left to right as the drivers see them: front bumper on the starting line
   * and facing the field, before each trench and before the hub. An auto starts on one ({@code
   * Rebuilt.STARTS.left()}), and the placement check reads whether the robot stands on it.
   */
  public record Starts(Start left, Start center, Start right) {
    /** All three, left to right: the order of Driver Station locations 1, 2, and 3. */
    public List<Start> all() {
      return List.of(left, center, right);
    }
  }

  /** The starts, written for blue. */
  public static final Starts STARTS =
      new Starts(
          onTheStartingLine("Left", WIDTH - TRENCH_OPENING / 2),
          onTheStartingLine("Center", WIDTH / 2),
          onTheStartingLine("Right", TRENCH_OPENING / 2));

  private static Start onTheStartingLine(String name, double y) {
    return new Start(
        name, new Pose2d(STARTING_LINE_X, y, Rotation2d.ZERO), Start.Anchor.FRONT_BUMPER);
  }

  /**
   * The season's field and its venues: the welded field by default, either build a venue may be
   * measured against, and the field model on any layout.
   */
  public static final Game GAME =
      new Game(
          LAYOUT.loadField(),
          List.of(
              Fields.FRC_2026_REBUILT_WELDED.loadField(),
              Fields.FRC_2026_REBUILT_ANDY_MARK.loadField()),
          SYMMETRY,
          Rebuilt::field,
          LIMITS);

  /**
   * The field model on the welded field.
   *
   * <p>Starts, left to right as the drivers see them ({@link #STARTS}). Driver Station locations 1,
   * 2, and 3 choose them when no auto is chosen; a chosen auto names its own.
   */
  public static FieldModel field() {
    return field(LAYOUT);
  }

  /** The field model on another of the season's layouts: the AndyMark build, say. */
  static FieldModel field(Fields layout) {
    return field(layout.loadField());
  }

  /** The field model on a WPILib layout, its tags where it puts them. */
  public static FieldModel field(Field layout) {
    return field(TagLayout.of(layout));
  }

  /**
   * The field model on the field's tags: one of the season's official layouts, or a venue's, its
   * tags as surveyed.
   */
  public static FieldModel field(TagLayout layout) {
    return FieldModel.builder(layout, SYMMETRY)
        .starts(STARTS.all())
        .places(List.of(HUB, TOWER))
        .zones(List.of(ALLIANCE_ZONE, TRENCH))
        .surfaces(surfaces())
        .elements(elements())
        .build();
  }
}
