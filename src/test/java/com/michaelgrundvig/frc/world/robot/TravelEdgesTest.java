package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.RAIL_TRAVEL;
import static com.michaelgrundvig.frc.world.robot.InterlockFixtures.WHEEL;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Where a joint's travel changes its answer, as a table of questions and exact answers: which turn
 * of a heading a turret goes to, and where a position stops being reachable. {@link TravelTest}
 * says how a travel is worked out from a joint's block; these rows are the edges.
 */
class TravelEdgesTest {
  private static final double TURN = 2 * Math.PI;

  /** How far past a limit a position may be and still count as on it. */
  private static final double HAIR = 1e-9;

  /** A turret that can go one whole turn either way from its middle, so two turns in all. */
  private static final Travel TURRET =
      new Travel(WHEEL, -TURN, TURN, Travel.Taking.AS_A_HEADING, 4, 2, 8);

  /**
   * A turret whose soft limits leave it only 0 to 1 rad, so that most headings have no turn it can
   * reach.
   */
  private static final Travel NARROWED =
      new Travel(WHEEL, 0, 1, Travel.Taking.AS_A_HEADING, 4, 2, 8);

  /** The same kind of turret, limited to 1 rad either side of its middle. */
  private static final Travel NARROWED_EVENLY =
      new Travel(WHEEL, -1, 1, Travel.Taking.AS_A_HEADING, 4, 2, 8);

  /**
   * A turret whose lower limit is a hair above a quarter of a radian, so that a hair below the
   * limit is a quarter of a radian exactly, and a heading one turn below that comes round to it
   * exactly.
   */
  private static final Travel FROM_A_HAIR_ABOVE_A_QUARTER =
      new Travel(WHEEL, 0.25 + HAIR, 2, Travel.Taking.AS_A_HEADING, 4, 2, 8);

  /** A turret whose upper limit is a hair below one and a half radians, for the same reason. */
  private static final Travel TO_A_HAIR_BELOW_ONE_AND_A_HALF =
      new Travel(WHEEL, 0, 1.5 - HAIR, Travel.Taking.AS_A_HEADING, 4, 2, 8);

  private static final String OUTSIDE_NARROWED =
      ", at every turn of it, is outside wheel's limits, 0° to 57.3°";

  private static Optional<String> whyUnreachable(Travel travel, double position) {
    return Optional.ofNullable(travel.whyUnreachable(position, 0));
  }

  @ParameterizedTest
  @MethodSource({"turnsOfAHeading", "headingsWithNoTurnInside", "reachable"})
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments row(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  // spotless:off

  /** A turret goes to the turn of a heading that is inside its limits and nearest where it is. */
  static Stream<Arguments> turnsOfAHeading() {
    return Stream.of(
        row("a joint that is no turret takes the angle as it is", () -> RAIL_TRAVEL.turnNear(100, 0), 100.0),
        row("from its middle, a turret takes the turn nearest its middle", () -> TURRET.turnNear(0.5, 0), 0.5),
        row("from a turn up, it takes the turn above", () -> TURRET.turnNear(0.5 - TURN, 5), 0.5),
        row("from a turn down, it takes the turn below", () -> TURRET.turnNear(0.5, -5), 0.5 - TURN),
        row("a heading given many turns away comes back inside", () -> TURRET.turnNear(0.5 - 2 * TURN, 0), 0.5 - 2 * TURN + 2 * TURN),
        row("it can take the last turn its limits hold", () -> TURRET.turnNear(0, 6), TURN),
        row("equally near two turns, it takes the lower", () -> TURRET.turnNear(Math.PI, 0), Math.PI - TURN),
        row("a heading on the upper limit is inside", () -> NARROWED.turnNear(1, 0), 1.0),
        row("a hair past the upper limit is put on it", () -> NARROWED.turnNear(1 + HAIR, 0), 1.0),
        row("a hair below the lower limit is put on it", () -> NARROWED.turnNear(-HAIR, 0), 0.0));
  }

  /** With no turn of a heading inside its limits, a turret is taken to the limit nearer it. */
  static Stream<Arguments> headingsWithNoTurnInside() {
    return Stream.of(
        row("past the upper limit, the upper", () -> NARROWED.turnNear(2, 0), 1.0),
        row("below the lower limit, the lower", () -> NARROWED.turnNear(-1, 0), 0.0),
        row("nearer the lower the short way round, the lower", () -> NARROWED.turnNear(TURN - 1, 0), 0.0),
        row("equally far from both, the lower", () -> NARROWED_EVENLY.turnNear(Math.PI, 0), -1.0));
  }

  /** A position is reachable inside the limits, and for a turret, when any turn of it is. */
  static Stream<Arguments> reachable() {
    return Stream.of(
        row("on the upper limit is reachable", () -> whyUnreachable(RAIL_TRAVEL, 16), Optional.empty()),
        row("a rounding step past it is still on it", () -> whyUnreachable(RAIL_TRAVEL, Math.nextUp(16.0)), Optional.empty()),
        row("...and is sent to the limit itself", () -> RAIL_TRAVEL.goalNear(Math.nextUp(16.0), 3), 16.0),
        row("a hair past the upper limit is reachable", () -> whyUnreachable(RAIL_TRAVEL, 16 + HAIR), Optional.empty()),
        row("the next position up is not", () -> whyUnreachable(RAIL_TRAVEL, Math.nextUp(16 + HAIR)), Optional.of("16 m is outside rail's limits, 0 m to 16 m")),
        row("a centimeter past it is not, and shows as past it", () -> whyUnreachable(RAIL_TRAVEL, 16.01), Optional.of("16.01 m is outside rail's limits, 0 m to 16 m")),
        row("on the lower limit is reachable", () -> whyUnreachable(RAIL_TRAVEL, 0), Optional.empty()),
        row("a rounding step below it is still on it", () -> whyUnreachable(RAIL_TRAVEL, -Double.MIN_VALUE), Optional.empty()),
        row("a hair below the lower limit is reachable", () -> whyUnreachable(RAIL_TRAVEL, -HAIR), Optional.empty()),
        row("the next position down is not", () -> whyUnreachable(RAIL_TRAVEL, Math.nextDown(-HAIR)), Optional.of("0 m is outside rail's limits, 0 m to 16 m")),
        row("a turret reaches a heading a turn away", () -> whyUnreachable(TURRET, TURN + 0.5), Optional.empty()),
        row("a hair past a turret's upper limit is reachable", () -> whyUnreachable(NARROWED, 1 + HAIR), Optional.empty()),
        row("the next heading up is not", () -> whyUnreachable(NARROWED, Math.nextUp(1 + HAIR)), Optional.of("57.3°" + OUTSIDE_NARROWED)),
        row("a hair below a turret's lower limit is reachable", () -> whyUnreachable(NARROWED, -HAIR), Optional.empty()),
        row("the next heading down is not", () -> whyUnreachable(NARROWED, Math.nextDown(-HAIR)), Optional.of("0°" + OUTSIDE_NARROWED)),
        row("a heading whose turn comes round to exactly a hair below the lower limit is reachable",
            () -> whyUnreachable(FROM_A_HAIR_ABOVE_A_QUARTER, 0.25 - TURN), Optional.empty()),
        row("...and the next heading down is not",
            () -> whyUnreachable(FROM_A_HAIR_ABOVE_A_QUARTER, Math.nextDown(0.25 - TURN)).isPresent(), true),
        row("a heading whose turn comes round to exactly a hair past the upper limit is reachable",
            () -> whyUnreachable(TO_A_HAIR_BELOW_ONE_AND_A_HALF, 1.5 + TURN), Optional.empty()),
        row("...and the next heading up is not",
            () -> whyUnreachable(TO_A_HAIR_BELOW_ONE_AND_A_HALF, Math.nextUp(1.5 + TURN)).isPresent(), true),
        row("a heading with no turn inside is not", () -> whyUnreachable(NARROWED, 2), Optional.of("114.6°" + OUTSIDE_NARROWED)));
  }

  // spotless:on
}
