package com.michaelgrundvig.frc.world.game;

import static com.michaelgrundvig.frc.world.game.FieldShapes.box;
import static com.michaelgrundvig.frc.world.game.FieldShapes.cutOff;
import static com.michaelgrundvig.frc.world.game.FieldShapes.post;
import static com.michaelgrundvig.frc.world.game.Manuals.IN;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.world.field.Block;
import com.michaelgrundvig.frc.world.field.Element;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Start;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import com.michaelgrundvig.frc.world.field.Zone;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * 2025's field, REEFSCAPE. Written as {@link Rebuilt} is: blue's alliance only, in meters in
 * WPILib's field frame, positions from WPILib's 2025 tag layout wherever a tag marks them, and
 * sizes from the 2025 game manual's section 5, cited by section. The few figures neither gives come
 * from maple-sim's arena ({@code Arena2025Reefscape} and its reef, at 1b1e8ea), and say so where
 * they're used.
 *
 * <p>The field is flat: the manual describes nothing robots drive over. What they run into is its
 * elements ({@link #elements}, and the barge across the center line).
 */
public final class Reefscape {
  private Reefscape() {}

  /**
   * What 2025's manual allows a robot: a 120 in perimeter and 3 ft 6 in tall at the start (R104),
   * reaching 18 in beyond it (R105), bumpers filling 2.5 to 5.75 in above the floor (R405), and 135
   * lb with bumpers and without its battery (R408), a battery with its leads 14.5 lb at most
   * (R601).
   */
  public static final SeasonLimits LIMITS =
      SeasonLimits.builder()
          .framePerimeter(Inches.of(120))
          .startingHeight(Inches.of(42))
          .extension(Inches.of(18))
          .bumperZoneBottom(Inches.of(2.5))
          .bumperZoneTop(Inches.of(5.75))
          .weight(Pounds.of(135))
          .battery(Pounds.of(14.5))
          .build();

  /** The welded field. The AndyMark build is 1 cm narrower, with its own layout. */
  static final Fields LAYOUT = Fields.FRC_2025_REEFSCAPE_WELDED;

  /** Red's half is blue's turned half a turn: the processors are on opposite guardrails. */
  static final Symmetry SYMMETRY = Symmetry.ROTATED;

  private static final double LENGTH = LAYOUT.length;
  private static final double WIDTH = LAYOUT.width;

  // ---- Materials, where the manual names them ----

  /** The reef's pipes and the cages: Schedule 40 steel pipe (5.3, 5.4.1). */
  static final Material STEEL = Material.builder().name("steel").build();

  /**
   * What the manual doesn't name a material for: the reef's base, the barge, the stations, the
   * processor.
   */
  static final Material STRUCTURE = Material.builder().name("field structure").build();

  // ---- The reef, 12 ft from the alliance wall and centered between the guardrails (5.3) ----

  /** The reef's near face, 12 ft out: where tag 18 hangs. */
  static final double REEF_NEAR_FACE = 144 * IN;

  /** A hexagon 5 ft 5½ in across: across its flats, as the tags on opposite faces are. */
  static final double REEF_ACROSS = 65.5 * IN;

  static final double REEF_X = REEF_NEAR_FACE + REEF_ACROSS / 2;
  static final double REEF_Y = WIDTH / 2;

  /** The base, up to the trough's front edge, 1 ft 6 in off the carpet. */
  static final double TROUGH_HEIGHT = 18 * IN;

  /**
   * The pipes: 1-¼ in Schedule 40, 1.66 in across, 1 ft 1 in apart on each face. A block can't be
   * round, so each is drawn as a square post that far across.
   */
  static final double PIPE_DIAMETER = 1.66 * IN;

  static final double PIPE_SPACING = 13 * IN;

  /**
   * How far the vertical pipes stand in from the reef's center, from maple-sim's reef ({@code
   * ReefscapeReefBranch} at 1b1e8ea): no tag or manual figure gives it.
   */
  static final double PIPE_FROM_CENTER = 0.525;

  /** L2 and L3 angle up at 35°, L4 is vertical. */
  static final double BRANCH_ANGLE = Math.toRadians(35);

  /** Each level's highest point off the carpet: L2, L3, then L4. */
  static final List<Double> BRANCH_TOPS = List.of(31.875 * IN, 47.625 * IN, 72.0 * IN);

  /** How far in from the base's face L2's and L3's highest points are, and L4's. */
  static final double ANGLED_INSET = 1.625 * IN;

  static final double L4_INSET = 1.125 * IN;

  /** A coral is 11⅞ in long (5.7.1). */
  static final double CORAL_LENGTH = 11.875 * IN;

  // ---- The barge, across the center of the field (5.4) ----

  /** The cages' centers from mid field: 3 ft 5½ in, 7 ft ⅜ in, and 10 ft 7⅜ in. */
  static final List<Double> CAGES = List.of(41.5 * IN, 84.375 * IN, 127.375 * IN);

  /** 29 ft 2 in wide, past both guardrails; 3 ft 8 in deep; 8 ft 5 in tall. */
  static final double BARGE_WIDTH = 350 * IN;

  static final double BARGE_DEPTH = 44 * IN;
  static final double BARGE_TOP = 101 * IN;

  /** The horizontal truss, 5 ft 2 in above the carpet: the clearance under it. */
  static final double TRUSS_BOTTOM = 62 * IN;

  /** A cage: 2 ft tall and 7⅜ in wide outside (5.4.1). */
  static final double CAGE_WIDTH = 7.375 * IN;

  static final double CAGE_HEIGHT = 24 * IN;

  /** A deep cage's bottom, 3⅛ in off the carpet; a shallow one's is 2 ft 6⅛ in (5.4.1). */
  static final double CAGE_BOTTOM = 3.125 * IN;

  /**
   * The vertical truss holding up the barge's middle: a square 0.305 m across, from maple-sim's
   * arena ({@code Arena2025Reefscape} at 1b1e8ea). The manual names it but gives no size.
   */
  static final double PILLAR_SIZE = 0.305;

  // ---- The coral stations, across the corners by the alliance wall, and the processor (5.5) ----

  /** Tags 12 and 13, on the blue stations' faces, and the ways they face. */
  static final Translation2d RIGHT_STATION_TAG = new Translation2d(0.851, 0.655);

  static final Translation2d LEFT_STATION_TAG = new Translation2d(0.851, 7.396);
  static final Rotation2d RIGHT_STATION_FACING = Rotation2d.fromDegrees(54);
  static final Rotation2d LEFT_STATION_FACING = Rotation2d.fromDegrees(-54);

  /**
   * A station's height: drawn to the top of its tag's panel, 4 ft 5¼ in up and 10½ in square (5.8).
   * How much higher the station stands isn't in the manual's text.
   */
  static final double STATION_HEIGHT = (53.25 + 10.5) * IN;

  /** Where the processor is along the field: tag 16's x, centered above its opening (5.8). */
  static final double PROCESSOR_X = 5.988;

  /**
   * The face of the guardrail where its tag hangs: 0.0038 m behind the field's edge, as WPILib's
   * layout puts tag 16. The processor's face is drawn from there to the field's edge.
   */
  static final double PROCESSOR_FACE_Y = -0.0038;

  /** Its opening: 2 ft 4 in wide and 1 ft 8 in tall, 7 in off the carpet. */
  static final double PROCESSOR_OPENING_WIDTH = 28 * IN;

  static final double PROCESSOR_OPENING_HEIGHT = 20 * IN;
  static final double PROCESSOR_OPENING_BOTTOM = 7 * IN;

  /**
   * The top of its tag's panel, 3 ft 9⅞ in up and 10½ in square (5.8): the processor is drawn up to
   * there. How much higher it stands isn't in the manual's text.
   */
  static final double PROCESSOR_TOP = (45.875 + 10.5) * IN;

  // ---- Places and zones ----

  /** The robot starting line, 7 ft 4 in from the reef: taken to the tape's near edge (5.2). */
  static final double STARTING_LINE_X = REEF_X + REEF_ACROSS / 2 + 88 * IN;

  /** The field's tape is 2 in wide (5.2). */
  static final double TAPE_WIDTH = 2 * IN;

  /** The reef's center, among its six tags. */
  static final Place REEF = Place.between("Reef", 17, 18, 19, 20, 21, 22);

  /**
   * The barge zone, 3 ft 10 in deep and 12 ft 2½ in long around the alliance's half of the barge
   * (5.2), on the center line: centered on the middle cage, which the text doesn't say.
   *
   * <p>Blue's half of the barge, with its cages and net, is the half to the blue drivers' left, so
   * the middle cage is that far toward +y from mid field. That is where figure 5-4 draws blue's
   * barge zone, and where maple-sim's arena scores blue's net ({@code ReefscapeBargeSimulation} at
   * 1b1e8ea). The manual's text doesn't say.
   */
  static final Zone BARGE_ZONE =
      Zone.ours(
          "Barge zone",
          Polygon2d.rectangle(
              new Pose2d(LENGTH / 2, WIDTH / 2 + CAGES.get(1), Rotation2d.ZERO),
              46 * IN,
              146.5 * IN));

  /** The reef zone: 7 ft 9½ in across, face to face (5.2). */
  static final double REEF_ZONE_ACROSS = 93.5 * IN;

  /**
   * The reef zone, the six-sided tape around the alliance's reef (5.2): centered on the reef and
   * turned as it is, its faces toward the walls, as figure 5-4 shows it.
   */
  static final Zone REEF_ZONE = Zone.ours("Reef zone", hexagon(REEF_ZONE_ACROSS));

  /**
   * A hexagon around the reef's center, its flats facing the way the reef's tags do: 0°, 60°, and
   * on round.
   *
   * @param across between opposite flats
   */
  private static Polygon2d hexagon(double across) {
    // A regular hexagon's corners are farther from its center than its flats are, by the cosine
    // of 30°. With flats facing 0°, 60° and on round, the corners are midway between: at 30°,
    // 90° and on round.
    double centerToCorner = across / 2 / Math.cos(Math.PI / 6);
    Translation2d center = new Translation2d(REEF_X, REEF_Y);
    List<Translation2d> corners = new ArrayList<>();
    for (int corner = 0; corner < 6; corner++) {
      Rotation2d toCorner = Rotation2d.fromDegrees(30 + 60 * corner);
      corners.add(center.plus(new Translation2d(centerToCorner, toCorner)));
    }
    return new Polygon2d(corners);
  }

  /**
   * What stands on blue's half: the reef, the coral stations, and the processor. The barge, across
   * the center line, is shared ({@link #barge}).
   */
  static List<Element> elements() {
    Polygon2d rightCorner = cutOff(Translation2d.ZERO, RIGHT_STATION_TAG, RIGHT_STATION_FACING);
    Polygon2d leftCorner =
        cutOff(new Translation2d(0, WIDTH), LEFT_STATION_TAG, LEFT_STATION_FACING);
    return List.of(
        reef(),
        processor(),
        Element.standing("Right coral station", rightCorner, STATION_HEIGHT, STRUCTURE, 12),
        Element.standing("Left coral station", leftCorner, STATION_HEIGHT, STRUCTURE, 13));
  }

  /**
   * The processor: its face in the guardrail, below its opening and above it up to its tag's panel,
   * carrying its tag (16). Drawn as wide as the opening, since the manual gives no other width; the
   * opening is left open, and what's behind it, past the field's edge, isn't drawn.
   */
  private static Element processor() {
    double nearSide = PROCESSOR_X - PROCESSOR_OPENING_WIDTH / 2;
    double farSide = PROCESSOR_X + PROCESSOR_OPENING_WIDTH / 2;
    double openingTop = PROCESSOR_OPENING_BOTTOM + PROCESSOR_OPENING_HEIGHT;
    return new Element(
        "Processor",
        List.of(
            box(nearSide, farSide, PROCESSOR_FACE_Y, 0, 0, PROCESSOR_OPENING_BOTTOM, STRUCTURE),
            box(nearSide, farSide, PROCESSOR_FACE_Y, 0, openingTop, PROCESSOR_TOP, STRUCTURE)),
        List.of(16));
  }

  /**
   * The reef: its hexagonal base up to the trough, carrying its six tags, and on each face two
   * pipes. Each pipe is drawn from the base up to where L3 leaves it, and L4 as its vertical part,
   * a coral long: the bend out to L4 isn't in the manual. L2's and L3's branches lean out at 35°,
   * which no upright block can, so they aren't drawn. The trough's channel isn't cut out of the
   * base.
   */
  private static Element reef() {
    List<Block> blocks = new ArrayList<>();
    blocks.add(Block.standing(hexagon(REEF_ACROSS), TROUGH_HEIGHT, STRUCTURE));
    Translation2d center = new Translation2d(REEF_X, REEF_Y);
    double pipeRadius = PIPE_DIAMETER / 2;
    double centerToFace = REEF_ACROSS / 2;
    // Where L3's branch leaves its pipe. The branch's highest point is the top edge of its end, a
    // radius off its axis, and the branch runs down from there at its angle to the pipe.
    double branchEnd = centerToFace - ANGLED_INSET + pipeRadius * Math.sin(BRANCH_ANGLE);
    double branchEndHeight = BRANCH_TOPS.get(1) - pipeRadius * Math.cos(BRANCH_ANGLE);
    double pipeTop = branchEndHeight - (branchEnd - PIPE_FROM_CENTER) * Math.tan(BRANCH_ANGLE);
    // L4's highest point is its outside edge, inset from the base's face.
    double l4FromCenter = centerToFace - L4_INSET - pipeRadius;
    double l4Top = BRANCH_TOPS.get(2);
    for (int face = 0; face < 6; face++) {
      Rotation2d outOfTheFace = Rotation2d.fromDegrees(60 * face);
      for (double alongTheFace : List.of(-PIPE_SPACING / 2, PIPE_SPACING / 2)) {
        Translation2d pipe =
            center.plus(new Translation2d(PIPE_FROM_CENTER, alongTheFace).rotateBy(outOfTheFace));
        Translation2d l4 =
            center.plus(new Translation2d(l4FromCenter, alongTheFace).rotateBy(outOfTheFace));
        blocks.add(post(pipe, outOfTheFace, PIPE_DIAMETER, TROUGH_HEIGHT, pipeTop, STEEL));
        blocks.add(post(l4, outOfTheFace, PIPE_DIAMETER, l4Top - CORAL_LENGTH, l4Top, STEEL));
      }
    }
    return new Element("Reef", blocks, List.of(17, 18, 19, 20, 21, 22));
  }

  /**
   * The barge, whole, across the center line, on the field it's drawn for: both alliances' cages,
   * and the tags above the middle ones, 14 and 4 above blue's, 5 and 15 above red's. It's shared,
   * written once and centered on the field, not mirrored. The pillar under its middle is drawn
   * whole. The truss is drawn as its envelope above the clearance, and the nets it holds aren't
   * drawn. Its legs, outside the guardrails, are off the field, and the cages hang deep, all six:
   * which way each hangs is each team's choice, and the model's field doesn't change after it's
   * built. The chains aren't drawn.
   *
   * @param length the field's, along which its middle is
   * @param width the field's, across which it's centered
   */
  private static Element barge(double length, double width) {
    double x = length / 2;
    double middle = width / 2;
    List<Block> blocks = new ArrayList<>();
    blocks.add(
        box(
            x - BARGE_DEPTH / 2,
            x + BARGE_DEPTH / 2,
            middle - BARGE_WIDTH / 2,
            middle + BARGE_WIDTH / 2,
            TRUSS_BOTTOM,
            BARGE_TOP,
            STRUCTURE));
    Translation2d center = new Translation2d(x, middle);
    blocks.add(post(center, Rotation2d.ZERO, PILLAR_SIZE, 0, TRUSS_BOTTOM, STRUCTURE));
    for (double side : List.of(-1.0, 1.0)) {
      for (double cage : CAGES) {
        Translation2d under = new Translation2d(x, middle + side * cage);
        blocks.add(
            post(
                under, Rotation2d.ZERO, CAGE_WIDTH, CAGE_BOTTOM, CAGE_BOTTOM + CAGE_HEIGHT, STEEL));
      }
    }
    return new Element("Barge", blocks, List.of(14, 4, 5, 15));
  }

  /**
   * Where a robot starts, left to right as the drivers see them, front bumper on the starting
   * line's far edge so the bumpers overlap it: in line with each middle cage, and between them.
   */
  public static final List<Start> STARTS =
      List.of(
          onTheStartingLine("Left", WIDTH / 2 + CAGES.get(1)),
          onTheStartingLine("Center", WIDTH / 2),
          onTheStartingLine("Right", WIDTH / 2 - CAGES.get(1)));

  private static Start onTheStartingLine(String name, double y) {
    return new Start(
        name,
        new Pose2d(STARTING_LINE_X + TAPE_WIDTH, y, Rotation2d.ZERO),
        Start.Anchor.FRONT_BUMPER);
  }

  /** The season's field and its venues: the welded field by default, or the AndyMark build. */
  public static final Game GAME =
      new Game(
          LAYOUT.loadField(),
          List.of(
              Fields.FRC_2025_REEFSCAPE_WELDED.loadField(),
              Fields.FRC_2025_REEFSCAPE_ANDY_MARK.loadField()),
          SYMMETRY,
          Reefscape::field,
          LIMITS);

  /** The field model on the welded field, with its starts ({@link #STARTS}). */
  public static FieldModel field() {
    return field(LAYOUT);
  }

  /** The field model on another of the season's layouts: the AndyMark build, say. */
  static FieldModel field(Fields layout) {
    return field(TagLayout.of(layout.loadField()));
  }

  /**
   * The field model on the field's tags: one of the season's official layouts, or a venue's, its
   * tags as surveyed.
   */
  public static FieldModel field(TagLayout tags) {
    return FieldModel.builder(tags, SYMMETRY)
        .starts(STARTS)
        .places(List.of(REEF))
        .zones(List.of(REEF_ZONE, BARGE_ZONE))
        .elements(elements())
        .shared(List.of(barge(tags.lengthMeters(), tags.widthMeters())))
        .build();
  }
}
