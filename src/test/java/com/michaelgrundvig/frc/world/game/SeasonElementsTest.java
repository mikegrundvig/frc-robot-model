package com.michaelgrundvig.frc.world.game;

import static com.michaelgrundvig.frc.world.game.Manuals.IN;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.field.Alliance;
import com.michaelgrundvig.frc.world.field.Block;
import com.michaelgrundvig.frc.world.field.Element;
import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Place;
import com.michaelgrundvig.frc.world.field.Surface;
import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.ConfigFixtures;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;

/**
 * What stands on each season's field, checked against its tags and a few of its manual's figures:
 * each tag hangs on the one element that carries it, on both halves, and a point just inside a
 * measured face is in the element while a point just outside it is in none.
 */
class SeasonElementsTest {
  private static final Body BODY = ConfigFixtures.body();
  private static final FieldModel REBUILT = Rebuilt.field(BODY);
  private static final FieldModel REEFSCAPE = Reefscape.field(BODY);
  private static final FieldModel CRESCENDO = Crescendo.field(BODY);

  static Stream<Arguments> fields() {
    return Stream.of(
        Arguments.argumentSet(
            "2026",
            REBUILT,
            List.of("Hub", "Right trench", "Left trench", "Tower", "Outpost"),
            List.of()),
        Arguments.argumentSet(
            "2025",
            REEFSCAPE,
            List.of("Reef", "Processor", "Right coral station", "Left coral station"),
            List.of("Barge")),
        Arguments.argumentSet(
            "2024", CRESCENDO, List.of("Stage", "Speaker", "Amp", "Source"), List.of()));
  }

  @ParameterizedTest
  @MethodSource("fields")
  void eachElementStandsOnBothHalvesAndEveryTagHangsOnOne(
      FieldModel field, List<String> onEachHalf, List<String> shared) {
    List<String> names = new ArrayList<>();
    for (String name : onEachHalf) {
      names.add(name + " (blue)");
      names.add(name + " (red)");
    }
    names.addAll(shared);
    assertThat(field.elements()).extracting(Element::name).containsExactlyElementsOf(names);

    // Blue's are drawn from the manual's sizes, within a centimeter of their tags. Red's are blue's
    // images, off red's tags by as much again as the layout itself is off the symmetry.
    double slackMeters = 0.01 + field.symmetryError();
    List<Integer> carried = new ArrayList<>();
    for (Element element : field.elements()) {
      for (int tag : element.tags()) {
        carried.add(tag);
        assertThat(element.distance(field.tag(tag).orElseThrow().getTranslation()))
            .as(element.name() + ", tag " + tag)
            .isLessThan(slackMeters);
      }
    }
    assertThat(carried).containsExactlyInAnyOrderElementsOf(field.tags().keySet());
  }

  /** Nothing is both driven over and run into: no element is named as a surface is. */
  @ParameterizedTest
  @MethodSource("fields")
  void nothingIsBothTerrainAndAnElement(FieldModel field, List<String> onEachHalf) {
    assertThat(field.terrain().surfaces())
        .extracting(Surface::name)
        .doesNotContainAnyElementsOf(field.elements().stream().map(Element::name).toList());
    // A block low enough to drive over would be terrain: every element stands above the bumpers.
    double bumperTop = BODY.bumperTop().in(Meters);
    assertThat(field.elements())
        .allSatisfy(e -> assertThat(e.heightMeters()).isGreaterThan(bumperTop));
  }

  /** A robot on any start, for either alliance, is clear of everything its bumpers could hit. */
  @ParameterizedTest
  @MethodSource("fields")
  void noStartIsInsideAnything(FieldModel field) {
    double lowest = BODY.bumperBottom().in(Meters);
    double highest = BODY.bumperTop().in(Meters);
    // Bumpers a millimeter smaller all round: a start may touch what it backs up to.
    double length = BODY.bumperLength().in(Meters) - 0.002;
    double width = BODY.bumperWidth().in(Meters) - 0.002;
    for (Place start : field.starts()) {
      for (Alliance alliance : Alliance.values()) {
        Pose2d pose = start.on(field, alliance);
        Polygon2d bumpers = Polygon2d.rectangle(pose, length, width);
        for (Element element : field.elements()) {
          assertThat(element.inTheWayOf(lowest, highest))
              .as(start.name() + " for " + alliance + " and " + element.name())
              .noneMatch(bumpers::touches);
        }
      }
    }
  }

  /** A row: a point in the field frame, and the element it is in, or none. */
  private static Arguments at(
      String label, FieldModel field, double x, double y, double z, String... inside) {
    return Arguments.argumentSet(label, field, new Translation3d(x, y, z), List.of(inside));
  }

  static Stream<Arguments> measuredPoints() {
    double hubY = REBUILT.width() / 2;
    double uprightY = Rebuilt.TOWER_Y + Rebuilt.UPRIGHT_GAP / 2 + Rebuilt.UPRIGHT_THICKNESS / 2;
    // Midway between the two tags on the right trench's arm, which hang on its two faces.
    Translation2d underArm =
        REBUILT
            .tag(17)
            .orElseThrow()
            .toPose2d()
            .getTranslation()
            .plus(REBUILT.tag(28).orElseThrow().toPose2d().getTranslation())
            .div(2);
    double lowRung = Rebuilt.RUNG_HEIGHTS.get(0);
    double reefFace = Reefscape.REEF_X + Reefscape.REEF_ACROSS / 2;
    double l4Y = Reefscape.REEF_Y + Reefscape.PIPE_SPACING / 2;
    double l4Top = Reefscape.BRANCH_TOPS.get(2);
    double bargeX = REEFSCAPE.length() / 2;
    double bargeY = REEFSCAPE.width() / 2;
    double cageY = bargeY + Reefscape.CAGES.get(0);
    double ampY = CRESCENDO.width() - 0.003;
    return Stream.of(
        at(
            "2026: the hub's top, 72 in up",
            REBUILT,
            Rebuilt.HUB_X,
            hubY,
            72 * IN - 0.01,
            "Hub (blue)"),
        at("2026: over the hub", REBUILT, Rebuilt.HUB_X, hubY, 72 * IN + 0.01),
        at("2026: red's hub", REBUILT, REBUILT.length() - Rebuilt.HUB_X, hubY, 1, "Hub (red)"),
        at(
            "2026: under a trench's arm, 22.25 in clear",
            REBUILT,
            underArm.getX(),
            underArm.getY(),
            0.56),
        at("2026: the arm", REBUILT, underArm.getX(), underArm.getY(), 0.57, "Right trench (blue)"),
        at(
            "2026: the tower wall's top",
            REBUILT,
            0.004,
            Rebuilt.TOWER_Y,
            Rebuilt.TOWER_HEIGHT - 0.01,
            "Tower (blue)"),
        at("2026: in front of the tower's wall", REBUILT, 0.02, Rebuilt.TOWER_Y, 1),
        at(
            "2026: the low rung, 27 in up",
            REBUILT,
            Rebuilt.UPRIGHT_X,
            Rebuilt.TOWER_Y,
            lowRung,
            "Tower (blue)"),
        at("2026: between the rungs", REBUILT, Rebuilt.UPRIGHT_X, Rebuilt.TOWER_Y, lowRung + 0.2),
        at(
            "2026: an upright's field face, 43.5 in out",
            REBUILT,
            1.104,
            uprightY,
            1,
            "Tower (blue)"),
        at("2026: past the upright", REBUILT, 1.108, uprightY, 1),
        at(
            "2025: the reef's base at the trough's edge",
            REEFSCAPE,
            reefFace - 0.005,
            Reefscape.REEF_Y,
            18 * IN - 0.005,
            "Reef (blue)"),
        at("2025: over the base", REEFSCAPE, reefFace - 0.005, Reefscape.REEF_Y, 18 * IN + 0.005),
        at(
            "2025: L4's top, 72 in up",
            REEFSCAPE,
            reefFace - Reefscape.L4_INSET - 0.001,
            l4Y,
            l4Top - 0.002,
            "Reef (blue)"),
        at("2025: over L4", REEFSCAPE, reefFace - Reefscape.L4_INSET - 0.001, l4Y, l4Top + 0.002),
        at(
            "2025: under the barge's truss, 62 in clear",
            REEFSCAPE,
            bargeX,
            bargeY + 0.5,
            62 * IN - 0.01),
        at("2025: the truss", REEFSCAPE, bargeX, bargeY + 0.5, 62 * IN + 0.01, "Barge"),
        at("2025: under a deep cage", REEFSCAPE, bargeX, cageY, Reefscape.CAGE_BOTTOM - 0.01),
        at("2025: the cage", REEFSCAPE, bargeX, cageY, Reefscape.CAGE_BOTTOM + 0.01, "Barge"),
        at(
            "2025: the corner a station cuts off",
            REEFSCAPE,
            0.01,
            0.01,
            1,
            "Right coral station (blue)"),
        at(
            "2025: red's, turned half a turn",
            REEFSCAPE,
            REEFSCAPE.length() - 0.01,
            REEFSCAPE.width() - 0.01,
            1,
            "Right coral station (red)"),
        at(
            "2024: under the stage's core, 28.25 in clear",
            CRESCENDO,
            Crescendo.STAGE_X,
            Crescendo.STAGE_Y,
            28.25 * IN - 0.01),
        at(
            "2024: the core",
            CRESCENDO,
            Crescendo.STAGE_X,
            Crescendo.STAGE_Y,
            28.25 * IN + 0.01,
            "Stage (blue)"),
        at(
            "2024: the subwoofer's front panel",
            CRESCENDO,
            Crescendo.SUBWOOFER_DEPTH - 0.005,
            Crescendo.SPEAKER_Y,
            0.1,
            "Speaker (blue)"),
        at(
            "2024: in front of the subwoofer",
            CRESCENDO,
            Crescendo.SUBWOOFER_DEPTH + 0.005,
            Crescendo.SPEAKER_Y,
            0.1),
        at(
            "2024: the amp's face under its pocket",
            CRESCENDO,
            Crescendo.AMP_X,
            ampY,
            26 * IN - 0.05,
            "Amp (blue)"),
        at("2024: the amp's pocket, open", CRESCENDO, Crescendo.AMP_X, ampY, 26 * IN + 0.1),
        at(
            "2024: red's amp, on the same guardrail",
            CRESCENDO,
            CRESCENDO.length() - Crescendo.AMP_X,
            ampY,
            0.3,
            "Amp (red)"),
        at("2024: the source by blue's wall", CRESCENDO, 0.01, 0.01, 1, "Source (blue)"),
        at(
            "2024: its mirror image",
            CRESCENDO,
            CRESCENDO.length() - 0.01,
            0.01,
            1,
            "Source (red)"));
  }

  @ParameterizedTest
  @MethodSource("measuredPoints")
  void aPointIsInTheElementTheManualPutsThere(
      FieldModel field, Translation3d point, List<String> inside) {
    List<String> standingThere =
        field.elements().stream()
            .filter(element -> element.contains(point))
            .map(Element::name)
            .toList();

    assertThat(standingThere).isEqualTo(inside);
  }

  /** Every block is whole on both halves: red's copy covers as much floor, at the same heights. */
  @ParameterizedTest
  @MethodSource("fields")
  void redsElementsAreBluesImages(FieldModel field) {
    for (int index = 0; index + 1 < field.elements().size(); index += 2) {
      Element blue = field.elements().get(index);
      if (!blue.name().endsWith("(blue)")) {
        continue;
      }
      Element red = field.elements().get(index + 1);
      assertThat(red.heightMeters()).isEqualTo(blue.heightMeters());
      for (int block = 0; block < blue.blocks().size(); block++) {
        Block ours = blue.blocks().get(block);
        Block theirs = red.blocks().get(block);
        Translation2d image = field.forAlliance(Alliance.RED, ours.outline().centroid());

        assertThat(theirs.outline().area()).isCloseTo(ours.outline().area(), within(1e-9));
        assertThat(theirs.outline().centroid().getDistance(image)).isLessThan(1e-9);
        assertThat(theirs.bottomMeters()).isEqualTo(ours.bottomMeters());
      }
    }
  }
}
