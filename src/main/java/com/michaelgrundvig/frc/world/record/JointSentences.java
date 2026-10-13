package com.michaelgrundvig.frc.world.record;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.Sensor;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The sentences a person reads about one joint, each made from the facts the joint logs: where it
 * stands ({@link JointState}), what holds it ({@link Holder}), where it is ({@link JointPlace}) and
 * who asked something of it ({@link JointOwner}). The wording is here and nowhere else.
 */
public final class JointSentences {
  private JointSentences() {}

  /**
   * Where a joint stands: "READY", "UNHOMED", "HOMING: toward its lower stop, for at most 9.5 s",
   * or "FAULTED: " and its fault.
   *
   * @param definition the robot, whose sensors name the switch a joint homes to
   * @param joint the joint the state is of, one of the definition's
   * @param state where it stands
   * @param fault the text of its fault, as logged beside the state; read only while it is faulted
   */
  public static String state(
      RobotDefinition definition, Joint joint, JointState state, String fault) {
    return switch (state.standing()) {
      case UNHOMED, READY -> state.standing().name();
      case HOMING ->
          String.format(
              Locale.ROOT,
              "HOMING: toward %s, for at most %.1f s",
              pushedToward(definition, joint, state.toward()),
              state.secondsAllowed());
      case FAULTED -> "FAULTED: " + fault;
    };
  }

  /** What a homing joint is pushed toward: "its lower stop", or its limit switch's label. */
  private static String pushedToward(
      RobotDefinition definition, Joint joint, JointState.Toward toward) {
    return switch (toward) {
      case NOTHING -> "nothing";
      case LOWER_STOP -> "its lower stop";
      case UPPER_STOP -> "its upper stop";
      case LOWER_SWITCH -> homingSwitch(definition, joint, Sensor.Switch.Limit.End.LOWER);
      case UPPER_SWITCH -> homingSwitch(definition, joint, Sensor.Switch.Limit.End.UPPER);
    };
  }

  /**
   * The label of the limit switch a joint homes to at one end of its travel: its first switch at
   * that end whose position is known, in the order of the definition's sensors.
   */
  private static String homingSwitch(
      RobotDefinition definition, Joint joint, Sensor.Switch.Limit.End end) {
    for (Sensor sensor : definition.everySensor()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.joint().equals(joint)
          && limit.end() == end
          && limit.position().isPresent()) {
        return onSwitch.label();
      }
    }
    return "its " + end.name().toLowerCase(Locale.ROOT) + " limit switch";
  }

  /**
   * What holds a joint, each thing in its own words with "; " between them: "roller runs only when
   * pivot at least 110°; pivot upper limit tripped". Empty when nothing holds it.
   *
   * @param definition the robot, whose interlocks, sensors and joints the holders point into
   * @param holders what holds the joint, in the order it is applied
   */
  public static String heldBy(RobotDefinition definition, List<Holder> holders) {
    List<String> each = new ArrayList<>();
    for (Holder holder : holders) {
      each.add(holder(definition, holder));
    }
    return String.join("; ", each);
  }

  /** One thing that holds a joint, in its own words. */
  private static String holder(RobotDefinition definition, Holder holder) {
    return switch (holder.kind()) {
      case INTERLOCK -> definition.everyInterlock().get(holder.index()).toString();
      case LIMIT_SWITCH -> definition.everySensor().get(holder.index()).label() + " tripped";
      case NO_POSITION ->
          possessive(definition.joints().get(holder.index()).label()) + " position isn't a number";
    };
  }

  /**
   * The joint one holder holds, which is the joint an interlock restricts, the joint a limit switch
   * is at an end of, or the joint whose position is not a number.
   */
  static Joint heldJoint(RobotDefinition definition, Holder holder) {
    return switch (holder.kind()) {
      case INTERLOCK -> definition.everyInterlock().get(holder.index()).joint();
      case LIMIT_SWITCH -> limitOf(definition.everySensor().get(holder.index())).joint();
      case NO_POSITION -> definition.joints().get(holder.index());
    };
  }

  /** What makes a sensor a limit switch: the joint it is on, and the end it is at. */
  private static Sensor.Switch.Limit limitOf(Sensor sensor) {
    if (sensor instanceof Sensor.Switch onSwitch
        && onSwitch.kind() instanceof Sensor.Switch.Limit limit) {
      return limit;
    }
    throw new IllegalArgumentException(sensor.label() + " is not a limit switch");
  }

  /**
   * Where a joint is, in its names for its places: "at out", "between run and out", "below in",
   * "above out", a number for a joint with no places ("45°"), and "unknown" while it does not know.
   * A joint that only runs at outputs is said by what it runs at: the output's name, "stopped", or
   * "idle".
   *
   * @param joint the joint, whose positions and outputs the place points into
   * @param place where it is
   * @param position where it is as a number, in its units: read only for a joint with no places
   */
  public static String place(Joint joint, JointPlace place, double position) {
    return switch (place.where()) {
      case UNKNOWN -> "unknown";
      case UNNAMED -> joint.shown(position);
      case AT -> "at " + placeName(joint, place.place());
      case BETWEEN ->
          "between " + placeName(joint, place.place()) + " and " + placeName(joint, place.other());
      case BELOW -> "below " + placeName(joint, place.place());
      case ABOVE -> "above " + placeName(joint, place.place());
      case RUNNING -> joint.outputs().get(place.place()).name();
      case STOPPED -> "stopped";
      case IDLE -> "idle";
    };
  }

  private static String placeName(Joint joint, int place) {
    return joint.positions().get(place).name();
  }

  /**
   * Who asked something of a joint this loop: the command's name, "outside any command", or "idle"
   * when nothing asked.
   *
   * @param owner who asked
   * @param command the name of the command that asked, as logged beside the owner; read only when a
   *     command asked
   */
  public static String owner(JointOwner owner, String command) {
    return switch (owner) {
      case IDLE -> "idle";
      case COMMAND -> command;
      case OUTSIDE_ANY_COMMAND -> "outside any command";
    };
  }
}
