package com.michaelgrundvig.frc.world.game;

import static com.michaelgrundvig.frc.world.game.FieldShapes.box;
import static com.michaelgrundvig.frc.world.game.FieldShapes.cutOff;
import static com.michaelgrundvig.frc.world.game.FieldShapes.post;
import static com.michaelgrundvig.frc.world.game.Manuals.IN;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.field.Block;
import com.michaelgrundvig.frc.world.field.Element;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import com.michaelgrundvig.frc.world.field.Zone;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.robot.Body;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * 2024's field, CRESCENDO, a mirrored one. Written as {@link Rebuilt} is: blue's alliance only, in
 * meters in WPILib's field frame, positions from WPILib's 2024 tag layout wherever a tag marks
 * them, and sizes from the 2024 game manual's section 5, cited by section. The few figures neither
 * gives come from maple-sim's arena ({@code Arena2024Crescendo} at 1b1e8ea), and say so where
 * they're used.
 *
 * <p>The field is flat: the manual describes nothing robots drive over.
 */
public final class Crescendo {
  private Crescendo() {}

  static final Fields LAYOUT = Fields.FRC_2024_CRESCENDO;

  /** Red's half is blue's reflected across the center line: both amps are on one guardrail. */
  static final Symmetry SYMMETRY = Symmetry.MIRRORED;

  private static final double LENGTH = LAYOUT.length;
  private static final double WIDTH = LAYOUT.width;

  // ---- Materials, where the manual names them ----

  /** The subwoofer's vertical faces, and its side inclined ones (5.6.1). */
  static final Material HDPE = Material.builder().name("HDPE").build();

  /**
   * What the manual doesn't name a material for: the stage's truss legs, the source, the amp, and
   * the speaker's part of the alliance wall.
   */
  static final Material STRUCTURE = Material.builder().name("field structure").build();

  /** The stage core's walls (5.5). */
  static final Material POLYCARBONATE = Material.builder().name("polycarbonate").build();

  // ---- The stage, three legs around a six-sided core (5.5) ----

  /** The stage's center: the middle of the tags on its core's three wide faces (14, 15, 16). */
  static final double STAGE_X = 4.868;

  /** Across the field, the tags' middle is the field's, within a millimeter. */
  static final double STAGE_Y = WIDTH / 2;

  /**
   * The truss legs: posts 0.35 m square standing 1.48 m from the stage's center, each facing it,
   * from maple-sim's arena ({@code Arena2024Crescendo} at 1b1e8ea). No tag marks them, and the
   * manual gives only that the stage stands 10 ft 1 in from the alliance wall.
   */
  static final double LEG_FROM_CENTER = 1.48;

  static final double LEG_SIZE = 0.35;

  /** From the center to the core's wide faces, where the tags hang: tag 14 is 0.453 m out. */
  static final double CORE_APOTHEM = 0.453;

  /** The core's aluminum bottom, 2 ft 4¼ in above the carpet: the clearance under it. */
  static final double CORE_BOTTOM = 28.25 * IN;

  /** The core's top, under the microphones: each 1 ft tall, its top 7 ft 4¼ in above the carpet. */
  static final double CORE_TOP = (88.25 - 12) * IN;

  // ---- The speaker's subwoofer, against the alliance wall (5.6.1) ----

  /** The subwoofer's center across the field: tag 7's, centered above it. */
  static final double SPEAKER_Y = 5.548;

  /** 3 ft ⅛ in out from the alliance wall, 3 ft 1 in tall, its vertical panels 8⅜ in tall. */
  static final double SUBWOOFER_DEPTH = 36.125 * IN;

  static final double PANEL_HEIGHT = 8.375 * IN;

  /**
   * How wide its front face is, and how wide it is at the wall, from maple-sim's arena ({@code
   * Arena2024Crescendo} at 1b1e8ea): the manual gives neither.
   */
  static final double SUBWOOFER_FRONT = 1.05;

  static final double SUBWOOFER_BACK = 1.99;

  /**
   * The face of the alliance wall where the speaker's tags hang: 1½ in behind the field's edge, as
   * WPILib's layout puts tags 7 and 8. The speaker's part of the wall is drawn from there to the
   * field's edge.
   */
  static final double SPEAKER_WALL_X = -0.0381;

  /** The speaker opening's lowest edge, 6 ft 6 in up: the top of the alliance wall (5.6.1). */
  static final double OPENING_BOTTOM = 78 * IN;

  /**
   * How far the speaker's part of the wall reaches either side of its center: out to the far edge
   * of tag 8's panel, 1 ft 5 in from the center and 10½ in wide (5.8). The manual gives no width
   * for it.
   */
  static final double SPEAKER_WALL_REACH = (17 + 10.5) * IN;

  // ---- The amp, in the guardrail on the drivers' left (5.6.2) ----

  /** Where it is along the field: tag 6's x, its panel centered above the amp (5.8). */
  static final double AMP_X = 1.8415;

  /**
   * The amp's face, where its tag hangs: 0.0068 m in from the field's edge, as WPILib's layout puts
   * tag 6. The amp is drawn from there to the field's edge.
   */
  static final double AMP_FACE_Y = 8.2042;

  /**
   * How wide the amp is: 4 ft 1½ in from the alliance wall to its near side, and centered under its
   * tag, 6 ft ½ in out, so 3 ft 10 in wide. The manual gives no width for it.
   */
  static final double AMP_WIDTH = 2 * (AMP_X - 49.5 * IN);

  /** Its pocket: 2 ft wide and 1 ft 6 in tall, 2 ft 2 in off the carpet, 3⅞ in deep. */
  static final double POCKET_WIDTH = 24 * IN;

  static final double POCKET_HEIGHT = 18 * IN;
  static final double POCKET_BOTTOM = 26 * IN;

  /**
   * The top of its tag's panel, 4 ft ⅛ in up and 10½ in square (5.8): the amp is drawn up to there.
   * How much higher it stands isn't in the manual's text.
   */
  static final double AMP_TOP = (48.125 + 10.5) * IN;

  // ---- The sources, each in the corner by the opposing alliance's wall (5.4) ----

  /**
   * The source on blue's half, the red alliance's, in the corner by the blue alliance wall: midway
   * between its tags, 9 and 10, on its wall, and the way they face.
   */
  static final Translation2d SOURCE_TAGS = new Translation2d(0.909, 0.565);

  static final Rotation2d SOURCE_FACING = Rotation2d.fromDegrees(60);

  /**
   * The source's height: drawn to the top of its tags' panels, 4 ft ⅛ in up and 10½ in square
   * (5.8). How much higher it stands isn't in the manual's text.
   */
  static final double SOURCE_HEIGHT = (48.125 + 10.5) * IN;

  // ---- Places and zones (5.2) ----

  /** The amp zone: 10 ft 10 in along the guardrail from the alliance wall, 1 ft 5¾ in deep. */
  static final double AMP_ZONE_LENGTH = 130 * IN;

  static final double AMP_ZONE_WIDTH = 17.75 * IN;

  /** The robot starting zone: 6 ft 4⅛ in deep, 23 ft 8⅛ in long beside the amp zone. */
  static final double STARTING_ZONE_DEPTH = 76.125 * IN;

  static final double STARTING_ZONE_LENGTH = 284.125 * IN;

  /** The source zone: 1 ft 6¾ in deep from the source wall, its tape included. */
  static final double SOURCE_ZONE_DEPTH = 18.75 * IN;

  /** The stage's center, among the tags on its core. */
  static final Place STAGE = Place.between("Stage", 14, 15, 16);

  static final Zone AMP_ZONE =
      Zone.ours(
          "Amp zone",
          Polygon2d.rectangle(
              new Translation2d(0, WIDTH - AMP_ZONE_WIDTH),
              new Translation2d(AMP_ZONE_LENGTH, WIDTH)));

  /**
   * The robot starting zone: from the alliance wall to the black tape, 6 ft 4⅛ in out, beside the
   * amp zone and down to the opponent's source zone, whose tape cuts its far corner off along the
   * source wall. The layout's source tags put its far edge 2 cm longer than the manual's 23 ft 8⅛
   * in.
   */
  static final Zone STARTING_ZONE =
      Zone.ours(
          "Robot starting zone",
          new Polygon2d(
              List.of(
                  sourceTape(0),
                  sourceTape(STARTING_ZONE_DEPTH),
                  new Translation2d(STARTING_ZONE_DEPTH, WIDTH - AMP_ZONE_WIDTH),
                  new Translation2d(0, WIDTH - AMP_ZONE_WIDTH))));

  /**
   * The source zone, in front of the alliance's source, by the opposing alliance's wall: a
   * parallelogram along the source wall, 1 ft 6¾ in deep, from the opposing alliance wall (5.2).
   * Its far end lines up with the robot starting zone's far edge, as figure 5-4 shows it; the text
   * doesn't place it. The layout's source tags have the source wall meet the guardrail 5 cm short
   * of there, so its corner by the guardrail reaches 3 cm past the field's edge.
   */
  static final Zone SOURCE_ZONE =
      Zone.ours(
          "Source zone",
          SYMMETRY.apply(
              Polygon2d.hull(
                  List.of(
                      sourceWall(0),
                      sourceWall(STARTING_ZONE_DEPTH),
                      sourceTape(STARTING_ZONE_DEPTH),
                      sourceTape(0))),
              LENGTH,
              WIDTH));

  /**
   * The stage zone, the alliance-colored tape around the stage, six-sided (5.2). The manual gives
   * no size for it, so it's the outline around the stage's legs as they're drawn here (from
   * maple-sim), which is how figure 5-5 shows the tape running.
   */
  static final Zone STAGE_ZONE = Zone.ours("Stage zone", Polygon2d.hull(legCorners()));

  /**
   * Where the source wall on blue's half, red's, crosses a line across the field: its tags' face.
   */
  private static Translation2d sourceWall(double x) {
    return across(SOURCE_TAGS, x);
  }

  /**
   * Where the tape in front of the source on blue's half, red's, crosses a line across the field.
   */
  private static Translation2d sourceTape(double x) {
    return across(SOURCE_TAGS.plus(new Translation2d(SOURCE_ZONE_DEPTH, SOURCE_FACING)), x);
  }

  /**
   * Where a line parallel to the source wall, through a point, is at a distance x from the blue
   * alliance wall.
   *
   * <p>The source wall runs a quarter turn from the way its tags face. Going along it from the
   * point, each meter moves the cosine of that direction along x and its sine along y, so reaching
   * x takes (x - the point's x) / cosine meters, and moves that many times the sine along y.
   */
  private static Translation2d across(Translation2d point, double x) {
    Rotation2d alongTheWall = SOURCE_FACING.plus(Rotation2d.CCW_PI_2);
    double metersAlongTheWall = (x - point.getX()) / alongTheWall.getCos();
    return new Translation2d(x, point.getY() + metersAlongTheWall * alongTheWall.getSin());
  }

  /**
   * The corners of the stage's three legs, seen from above: the stage zone's outline. The legs
   * stand a third of a turn apart around the stage's center, at 60°, 180° and 300°, and each is a
   * square post turned to face the center.
   */
  private static List<Translation2d> legCorners() {
    Translation2d center = new Translation2d(STAGE_X, STAGE_Y);
    List<Translation2d> corners = new ArrayList<>();
    for (int leg = 0; leg < 3; leg++) {
      Rotation2d toLeg = Rotation2d.fromDegrees(120 * leg + 60);
      Translation2d middle = center.plus(new Translation2d(LEG_FROM_CENTER, toLeg));
      Polygon2d post = Polygon2d.rectangle(new Pose2d(middle, toLeg), LEG_SIZE, LEG_SIZE);
      corners.addAll(post.corners());
    }
    return corners;
  }

  /**
   * What stands on blue's half: the stage, the speaker, and the amp, and the source in the corner
   * by the blue alliance wall, which is the red alliance's. The model names each copy for the half
   * it stands on, so this one is "Source (blue)", and blue's own, by the red alliance wall, is
   * "Source (red)".
   */
  static List<Element> elements() {
    Polygon2d sourceCorner = cutOff(Translation2d.ZERO, SOURCE_TAGS, SOURCE_FACING);
    return List.of(
        stage(),
        speaker(),
        amp(),
        Element.standing("Source", sourceCorner, SOURCE_HEIGHT, STRUCTURE, 9, 10));
  }

  /**
   * The stage: its core, hung over the clearance and carrying its three tags, and its three truss
   * legs. Simplified: the core's narrow faces aren't in the manual, so it's drawn as a regular
   * hexagon, every face as far out as the wide ones; the legs are drawn up to the core's top, and
   * the truss over it, the legs' feet, and the gussets above the feet aren't drawn. The chains
   * between the legs droop, which no upright block can, so they aren't drawn either.
   */
  private static Element stage() {
    Translation2d center = new Translation2d(STAGE_X, STAGE_Y);
    // The wide faces look toward the red wall (tag 14) and a third of a turn either way from it,
    // so the hexagon's corners are midway between its six faces: at 30°, 90° and on round.
    double centerToCorner = CORE_APOTHEM / Math.cos(Math.PI / 6);
    List<Translation2d> coreCorners = new ArrayList<>();
    for (int corner = 0; corner < 6; corner++) {
      Rotation2d toCorner = Rotation2d.fromDegrees(30 + 60 * corner);
      coreCorners.add(center.plus(new Translation2d(centerToCorner, toCorner)));
    }
    List<Block> blocks = new ArrayList<>();
    blocks.add(new Block(new Polygon2d(coreCorners), CORE_BOTTOM, CORE_TOP, POLYCARBONATE));
    for (int leg = 0; leg < 3; leg++) {
      Rotation2d toLeg = Rotation2d.fromDegrees(120 * leg + 60);
      Translation2d middle = center.plus(new Translation2d(LEG_FROM_CENTER, toLeg));
      blocks.add(post(middle, toLeg, LEG_SIZE, 0, CORE_TOP, STRUCTURE));
    }
    return new Element("Stage", blocks, List.of(14, 15, 16));
  }

  /**
   * The speaker: its subwoofer, and its part of the alliance wall, carrying its tags (7 and 8), up
   * to the opening's lowest edge. The subwoofer is drawn as its vertical panels all round, which
   * are what a robot runs into. Its inclined panels above them slope up to the wall, which no
   * upright block can, so they aren't drawn, and neither is the hood above the opening.
   */
  private static Element speaker() {
    Polygon2d subwoofer =
        new Polygon2d(
            List.of(
                new Translation2d(0, SPEAKER_Y - SUBWOOFER_BACK / 2),
                new Translation2d(SUBWOOFER_DEPTH, SPEAKER_Y - SUBWOOFER_FRONT / 2),
                new Translation2d(SUBWOOFER_DEPTH, SPEAKER_Y + SUBWOOFER_FRONT / 2),
                new Translation2d(0, SPEAKER_Y + SUBWOOFER_BACK / 2)));
    Block wall =
        box(
            SPEAKER_WALL_X,
            0,
            SPEAKER_Y - SPEAKER_WALL_REACH,
            SPEAKER_Y + SPEAKER_WALL_REACH,
            0,
            OPENING_BOTTOM,
            STRUCTURE);
    return new Element(
        "Speaker", List.of(Block.standing(subwoofer, PANEL_HEIGHT, HDPE), wall), List.of(7, 8));
  }

  /**
   * The amp: its face in the guardrail around its pocket, carrying its tag (6). Its pocket is left
   * open; its recess, past the field's edge, isn't drawn.
   */
  private static Element amp() {
    double nearSide = AMP_X - AMP_WIDTH / 2;
    double farSide = AMP_X + AMP_WIDTH / 2;
    double pocketNearSide = AMP_X - POCKET_WIDTH / 2;
    double pocketFarSide = AMP_X + POCKET_WIDTH / 2;
    double pocketTop = POCKET_BOTTOM + POCKET_HEIGHT;
    return new Element(
        "Amp",
        List.of(
            box(nearSide, farSide, AMP_FACE_Y, WIDTH, 0, POCKET_BOTTOM, STRUCTURE),
            box(nearSide, farSide, AMP_FACE_Y, WIDTH, pocketTop, AMP_TOP, STRUCTURE),
            box(nearSide, pocketNearSide, AMP_FACE_Y, WIDTH, POCKET_BOTTOM, pocketTop, STRUCTURE),
            box(pocketFarSide, farSide, AMP_FACE_Y, WIDTH, POCKET_BOTTOM, pocketTop, STRUCTURE)),
        List.of(6));
  }

  /**
   * The field model for a robot of this size, which sets where it stands in the starting zone.
   *
   * <p>Starts, facing the field: back to the alliance wall beside the amp zone, back to the
   * subwoofer's front face, and centered across the field with the front bumper on the starting
   * zone's line.
   */
  public static FieldModel field(Body body) {
    double halfLength = body.bumperLength().in(Meters) / 2;
    double halfWidth = body.bumperWidth().in(Meters) / 2;
    return FieldModel.builder(TagLayout.of(LAYOUT.loadField()), SYMMETRY)
        .starts(
            List.of(
                Place.at(
                    "Amp side",
                    new Pose2d(halfLength, WIDTH - AMP_ZONE_WIDTH - halfWidth, Rotation2d.ZERO)),
                Place.at(
                    "Subwoofer",
                    new Pose2d(SUBWOOFER_DEPTH + halfLength, SPEAKER_Y, Rotation2d.ZERO)),
                Place.at(
                    "Center",
                    new Pose2d(STARTING_ZONE_DEPTH - halfLength, WIDTH / 2, Rotation2d.ZERO))))
        .places(List.of(STAGE))
        .zones(List.of(STARTING_ZONE, AMP_ZONE, SOURCE_ZONE, STAGE_ZONE))
        .elements(elements())
        .build();
  }
}
