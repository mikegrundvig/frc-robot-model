package com.michaelgrundvig.frc.world.game;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import org.junit.jupiter.api.Test;
import org.wpilib.fields.Fields;

/** What a game says about its field, as the season's class makes it. */
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
}
