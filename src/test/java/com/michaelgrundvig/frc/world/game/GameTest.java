package com.michaelgrundvig.frc.world.game;

import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;

/** What a game says about its field and its robots, as the season's class makes it. */
class GameTest {
  private final Game game = Rebuilt.GAME;

  @Test
  void theSeasonsGameOffersBothBuildsOfTheFieldAndStartsOnTheWelded() {
    assertThat(game.official().getName()).isEqualTo("2026 FRC Rebuilt Welded");
    assertThat(game.variants())
        .extracting(field -> field.getName())
        .containsExactly("2026 FRC Rebuilt Welded", "2026 FRC Rebuilt AndyMark");
    assertThat(game.symmetry()).isEqualTo(Symmetry.ROTATED);
  }

  @Test
  void theOfficialFieldIsTheModelOnTheOfficialLayout() {
    FieldModel official = game.officialField();

    assertThat(official.name()).isEqualTo("2026 FRC Rebuilt Welded");
    assertThat(official.places()).containsExactly(Rebuilt.HUB, Rebuilt.TOWER);
  }

  @Test
  void theModelIsMadeOnAnyLayoutItIsGiven() {
    TagLayout other = TagLayout.of(Fields.FRC_2026_REBUILT_ANDY_MARK.loadField());

    FieldModel andyMark = game.model().apply(other);

    assertThat(andyMark.name()).isEqualTo("2026 FRC Rebuilt AndyMark");
    assertThat(andyMark.zones()).containsExactly(Rebuilt.ALLIANCE_ZONE, Rebuilt.TRENCH);
  }

  /**
   * Each season's game carries its manual's limits, which its robots are checked against, and the
   * weight a robot not yet weighed is taken at: its most with a battery. Without its elements it
   * keeps them.
   */
  @Test
  void eachSeasonsGameCarriesItsLimits() {
    assertThat(game.limits()).isSameAs(Rebuilt.LIMITS);
    assertThat(game.withoutElements().limits()).isSameAs(Rebuilt.LIMITS);
    assertThat(Reefscape.GAME.limits()).isSameAs(Reefscape.LIMITS);
    assertThat(Crescendo.GAME.limits()).isSameAs(Crescendo.LIMITS);
    assertThat(Rebuilt.LIMITS.maximumWithBattery().in(Pounds)).isEqualTo(149.5);
    assertThat(Reefscape.LIMITS.maximumWithBattery().in(Pounds)).isEqualTo(149.5);
    assertThat(Crescendo.LIMITS.maximumWithBattery().in(Pounds)).isEqualTo(155);
  }
}
