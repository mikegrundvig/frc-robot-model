package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;

/**
 * The definition's checks of where its switches and port encoders are connected: each input in one
 * pass, from where each is written, its joint's block or the robot's sensors. One sensor on each
 * input, a joint's absolute encoder on a Systemcore port among them; and a switch on a motor
 * controller's limit input only where that controller has one, on the joint whose switch it is. A
 * CANdi's or a CANrange's CAN ID is the devices' to check ({@link CanDevice#sharedIds}). A joint's
 * own limit switch on its controller's input for the other end is refused while the motor isn't
 * reversed, the controller's forward then being the joint's way toward its upper end; with the
 * motor reversed, whether the controller's inputs follow its inversion is unconfirmed (open
 * question M22), so it isn't judged.
 */
final class Inputs {
  private Inputs() {}

  /**
   * What's wrong with where the switches and port encoders are connected, each about the one
   * connected second, or the switch.
   *
   * @param joints the robot's joints, each with its block
   * @param sensors every sensor: each joint's own, then the robot's
   */
  static List<Finding> problems(List<Joint> joints, List<Sensor> sensors) {
    List<Finding> problems = new ArrayList<>();
    Map<SensorWiring.Input, String> taken = new HashMap<>();
    for (Joint joint : joints) {
      Optional<MechanismWiring.PortEncoder> port =
          joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::port);
      if (port.isPresent()) {
        take(
            taken,
            SensorWiring.port(port.get().port()),
            joint.label() + " absolute encoder",
            problems);
      }
    }
    Map<String, Joint> byLabel = new HashMap<>();
    joints.forEach(joint -> byLabel.put(joint.label(), joint));
    for (Sensor sensor : sensors) {
      if (!(sensor instanceof Sensor.Switch onSwitch) || onSwitch.wiring().isEmpty()) {
        continue;
      }
      SensorWiring.Input input = onSwitch.wiring().get().input();
      String label = onSwitch.label();
      take(taken, input, label, problems);
      if (!(input instanceof SensorWiring.ControllerLimit limit)) {
        continue;
      }
      if (onSwitch.kind() instanceof Sensor.Switch.Limit own
          && !own.joint().label().equals(limit.joint())) {
        problems.add(
            Finding.refused(
                label,
                label
                    + " is "
                    + possessive(own.joint().label())
                    + " limit switch, on "
                    + possessive(limit.joint())
                    + " controller, which would stop "
                    + limit.joint()
                    + ": wire it to "
                    + possessive(own.joint().label())));
      }
      Joint on = byLabel.get(limit.joint());
      Optional<Driven.Motors> motors =
          on == null
              ? Optional.empty()
              : on.driven().motors().filter(each -> !each.controllers().isEmpty());
      if (onSwitch.kind() instanceof Sensor.Switch.Limit own && motors.isPresent()) {
        whyOnTheOtherEndsInput(label, own, limit, motors.get()).ifPresent(problems::add);
      }
      if (motors.isEmpty()) {
        problems.add(
            Finding.refused(
                label,
                label
                    + " is on "
                    + possessive(limit.joint())
                    + " controller's limit input, and "
                    + limit.joint()
                    + " isn't wired: wire "
                    + possessive(limit.joint())
                    + " motors to their controllers, or put "
                    + label
                    + " on a Systemcore port or a CANdi"));
      } else if (motors.get().controllers().get(0).controller()
              == MechanismWiring.Controller.TALON_FX
          && !motors.get().motor().equals(Motor.FALCON_500)) {
        problems.add(
            Finding.refused(
                label,
                label
                    + " is on "
                    + possessive(limit.joint())
                    + " TalonFX's limit input, and a Kraken's TalonFX has none: put it on a"
                    + " CANdi"));
      }
    }
    return problems;
  }

  /**
   * Why a joint's own limit switch is refused on its controller's input for the other end, if it
   * is: with its motor not reversed, the controller's forward input stops the joint moving toward
   * its upper end, and its reverse input toward its lower end.
   */
  private static Optional<Finding> whyOnTheOtherEndsInput(
      String label,
      Sensor.Switch.Limit own,
      SensorWiring.ControllerLimit limit,
      Driven.Motors motors) {
    if (!own.joint().label().equals(limit.joint())) {
      return Optional.empty();
    }
    boolean reversed = motors.controllers().get(0).turnsClockwise();
    if (reversed) {
      return Optional.empty();
    }
    SensorWiring.Direction itsEnds = inputFor(own.end());
    if (limit.direction() == itsEnds) {
      return Optional.empty();
    }
    Sensor.Switch.Limit.End stopped =
        own.end() == Sensor.Switch.Limit.End.LOWER
            ? Sensor.Switch.Limit.End.UPPER
            : Sensor.Switch.Limit.End.LOWER;
    return Optional.of(
        Finding.refused(
            label,
            label
                + " is at "
                + possessive(limit.joint())
                + " "
                + lowercase(own.end())
                + " end, on its controller's "
                + lowercase(limit.direction())
                + " limit input, which stops it moving toward its "
                + lowercase(stopped)
                + " end: put it on Direction."
                + itsEnds.name()));
  }

  /** The controller's input for a joint's end, with its motor not reversed. */
  private static SensorWiring.Direction inputFor(Sensor.Switch.Limit.End end) {
    if (end == Sensor.Switch.Limit.End.LOWER) {
      return SensorWiring.Direction.REVERSE;
    }
    return SensorWiring.Direction.FORWARD;
  }

  private static String lowercase(Enum<?> value) {
    return value.name().toLowerCase(Locale.ROOT);
  }

  /** An input taken by what's connected to it, or what's already there named with it. */
  private static void take(
      Map<SensorWiring.Input, String> taken,
      SensorWiring.Input input,
      String label,
      List<Finding> problems) {
    String other = taken.putIfAbsent(input, label);
    if (other != null) {
      problems.add(
          Finding.refused(
              label,
              other
                  + " and "
                  + label
                  + " are on one input, "
                  + SensorWiring.shown(input)
                  + ": move one to an input of its own"));
    }
  }
}
