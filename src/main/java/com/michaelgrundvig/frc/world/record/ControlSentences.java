package com.michaelgrundvig.frc.world.record;

import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.List;

/**
 * The sentences a person reads about the robot's gates and its gamepads' bindings, each made from
 * the facts the robot logs ({@link GateTerm}, {@link BindingStart}). The wording is here and
 * nowhere else.
 */
public final class ControlSentences {
  private ControlSentences() {}

  /**
   * One term that keeps a gate false, in words: "aimed", "pivot doesn't know where it is", "shooter
   * isn't answering", "claw beam lost", "ready to shoot settling", "aimed threw: no target".
   *
   * @param definition the robot, whose joints and sensors a term from the model points into
   * @param gate the gate's name
   * @param term the term
   * @param text the text logged beside the gate's terms, which a term in a team's own words, or one
   *     that says what was thrown, points into
   */
  public static String gateTerm(
      RobotDefinition definition, String gate, GateTerm term, List<String> text) {
    return switch (term.kind()) {
      case TEXT -> text.get(term.index());
      case JOINT_POSITION_UNKNOWN ->
          definition.joints().get(term.index()).label() + " doesn't know where it is";
      case JOINT_NOT_ANSWERING ->
          definition.joints().get(term.index()).label() + " isn't answering";
      case SENSOR_LOST -> definition.everySensor().get(term.index()).label() + " lost";
      case SETTLING -> gate + " settling";
      case THREW -> gate + " threw: " + text.get(term.index());
    };
  }

  /**
   * Why a gate is false, as a reason reads: "ready to shoot is false: aimed, shooter at goal", or
   * just "ready to shoot is false" for a gate without terms.
   *
   * @param gate the gate's name
   * @param blocking the terms that keep it false, each in words ({@link #gateTerm})
   */
  public static String gateWhy(String gate, List<String> blocking) {
    if (blocking.isEmpty()) {
      return gate + " is false";
    }
    return gate + " is false: " + String.join(", ", blocking);
  }

  /**
   * One command bound to a gamepad, as the list of the robot's bindings says it: "operator right
   * trigger: while held: Intake".
   *
   * @param gamepad the gamepad's name: "driver" or "operator"
   * @param button what the command is bound to, as the gamepad names it: "right trigger"
   * @param start when the command starts, and what stops it
   * @param command the command's name
   */
  public static String binding(String gamepad, String button, BindingStart start, String command) {
    return gamepad + " " + button + ": " + start.words() + ": " + command;
  }
}
