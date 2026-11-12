package com.michaelgrundvig.frc.world.game;

import com.michaelgrundvig.frc.world.field.FieldModel;
import com.michaelgrundvig.frc.world.field.Symmetry;
import com.michaelgrundvig.frc.world.field.TagFamily;
import com.michaelgrundvig.frc.world.field.TagLayout;
import java.util.List;
import java.util.function.Function;
import org.wpilib.fields.Field;
import org.wpilib.units.measure.Distance;

/**
 * What a game says about its field and its robots, the same for every robot: a season's class has
 * its own, such as {@link Rebuilt#GAME}. A program that needs a field the model doesn't have yet
 * can make its own.
 *
 * @param official the official layout the robot uses with no venue: the game's default variant
 * @param variants every official layout a venue may be measured against, the default included
 * @param symmetry the game's, which every venue must keep
 * @param tagFamily which AprilTag family the game's tags are printed from, as its manual says
 * @param tagSize the side of each tag's black square, as the game's manual prints them: every
 *     layout of the game's, and every venue's, has it
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
    TagFamily tagFamily,
    Distance tagSize,
    Function<TagLayout, FieldModel> model,
    SeasonLimits limits) {
  /** The field model on the official layout: the one the robot starts with. */
  public FieldModel officialField() {
    return model.apply(layout(official));
  }

  /**
   * One of the game's layouts, the official one or a variant, its tags nominal and of the game's
   * family and size.
   */
  public TagLayout layout(Field variant) {
    return TagLayout.of(variant, tagFamily, tagSize);
  }

  /**
   * This game with nothing standing on its field, whichever layout it is built on ({@link
   * FieldModel#withoutElements}): for a test that drives a robot across the floor.
   */
  public Game withoutElements() {
    return new Game(
        official,
        variants,
        symmetry,
        tagFamily,
        tagSize,
        model.andThen(FieldModel::withoutElements),
        limits);
  }
}
