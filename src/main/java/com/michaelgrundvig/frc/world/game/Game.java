package com.michaelgrundvig.frc.world.game;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import java.util.List;
import java.util.function.Function;
import org.wpilib.fields.Field;

/**
 * What a game says about its field. A season's class, such as {@link Rebuilt}, makes one for a
 * robot of a given size. A program that needs a field the model doesn't have yet can make its own.
 *
 * @param official the official layout the robot uses with no venue: the game's default variant
 * @param variants every official layout a venue may be measured against, the default included
 * @param symmetry the game's, which every venue must keep
 * @param model makes the game's field model (its places, zones and terrain) on the field's tags:
 *     the official layout's, or a venue's
 */
public record Game(
    Field official,
    List<Field> variants,
    Symmetry symmetry,
    Function<TagLayout, FieldModel> model) {
  /** The field model on the official layout: the one the robot starts with. */
  public FieldModel officialField() {
    return model.apply(TagLayout.of(official));
  }

  /**
   * This game with nothing standing on its field, whichever layout it is built on ({@link
   * FieldModel#withoutElements}): for a test that drives a robot across the floor.
   */
  public Game withoutElements() {
    return new Game(official, variants, symmetry, model.andThen(FieldModel::withoutElements));
  }
}
