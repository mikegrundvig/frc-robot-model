package com.michaelgrundvig.frc.world.game;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagLayout;
import java.util.List;
import java.util.function.Function;
import org.wpilib.fields.Field;

/**
 * What a game says about its field and its robots, the same for every robot: a season's class has
 * its own, such as {@link Rebuilt#GAME}. A program that needs a field the model doesn't have yet
 * can make its own.
 *
 * @param official the official layout the robot uses with no venue: the game's default variant
 * @param variants every official layout a venue may be measured against, the default included
 * @param symmetry the game's, which every venue must keep
 * @param model makes the game's field model (its places, zones and terrain) on the field's tags:
 *     the official layout's, or a venue's
 * @param limits what the game's manual allows a robot to be: what each robot's definition is
 *     checked against, and the weight a robot not yet weighed is taken at ({@link
 *     SeasonLimits#robotMass})
 */
public record Game(
    Field official,
    List<Field> variants,
    Symmetry symmetry,
    Function<TagLayout, FieldModel> model,
    SeasonLimits limits) {
  /** The field model on the official layout: the one the robot starts with. */
  public FieldModel officialField() {
    return model.apply(TagLayout.of(official));
  }

  /**
   * This game with nothing standing on its field, whichever layout it is built on ({@link
   * FieldModel#withoutElements}): for a test that drives a robot across the floor.
   */
  public Game withoutElements() {
    return new Game(
        official, variants, symmetry, model.andThen(FieldModel::withoutElements), limits);
  }
}
