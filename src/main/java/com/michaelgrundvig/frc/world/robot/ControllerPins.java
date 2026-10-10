package com.michaelgrundvig.frc.world.robot;

import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * What a joint's leading motor controller reads as its forward and reverse limits, from the robot's
 * sensor wiring: a switch on its own limit pin, or, for a TalonFX, a limit switch on a CANdi input
 * it reads over CAN. Worked out for each joint ({@link #of}) from the robot's definition, never
 * written by hand.
 *
 * <p>The controller stops its motor on a limit switch itself, with no code in the loop: the safest
 * stop there is. It never stops on anything else on a pin (a beam break read through it), and a pin
 * with nothing wired to it stops nothing.
 *
 * @param forward its forward limit, if it has one
 * @param reverse its reverse limit, if it has one
 */
public record ControllerPins(Optional<Limit> forward, Optional<Limit> reverse) {
  /** A controller with nothing on its limits. */
  public static final ControllerPins NONE = new ControllerPins(Optional.empty(), Optional.empty());

  /** One of its limits. */
  public sealed interface Limit permits OnPin, OnCandi {
    /** Whether the controller stops its motor on it. */
    boolean stops();
  }

  /**
   * A switch on the controller's own limit pin: read through it, the controller told its polarity.
   *
   * @param polarity which raw signal means tripped
   * @param stops whether it's a limit switch, which the controller stops on
   */
  public record OnPin(SensorWiring.Polarity polarity, boolean stops) implements Limit {}

  /**
   * A limit switch on a CANdi input on the controller's own bus, which a TalonFX reads over CAN as
   * its limit: always stopped on. The CANdi is told which state is tripped, so its "closed" is.
   *
   * @param id the CANdi's CAN ID
   * @param input 1 or 2: its S1 or S2
   */
  public record OnCandi(int id, int input) implements Limit {
    @Override
    public boolean stops() {
      return true;
    }
  }

  /** The same, with one limit set. */
  public ControllerPins with(SensorWiring.Direction direction, Limit limit) {
    return direction == SensorWiring.Direction.FORWARD
        ? new ControllerPins(Optional.of(limit), reverse)
        : new ControllerPins(forward, Optional.of(limit));
  }

  /**
   * What a joint's leading motor controller has on its limits, from the wiring its switches carry
   * ({@link #onLimit}): a switch on one of its pins, its polarity told and stopped on if it's a
   * limit switch; and, for a TalonFX or TalonFXS, its own limit switch on a CANdi input on its bus,
   * which it reads over CAN. A SPARK can't read a CANdi: its limit switch there stops it only
   * through the robot's code. The robot's IO for the joint is given these, and so is the
   * simulator's, which reads the same switches and stops where the controller would.
   */
  public static ControllerPins of(RobotDefinition definition, Joint joint) {
    ControllerPins pins = NONE;
    for (SensorWiring.Direction direction : SensorWiring.Direction.values()) {
      Sensor.Switch on = onLimit(definition, joint, direction);
      if (on == null) {
        continue;
      }
      SensorWiring.Connection wiring = on.wiring().orElseThrow();
      pins =
          pins.with(
              direction,
              wiring.input() instanceof SensorWiring.Candi candi
                  ? new OnCandi(candi.id(), candi.input())
                  : new OnPin(wiring.polarity(), ownLimit(on, joint.label())));
    }
    return pins;
  }

  /**
   * The limit input on a joint's motor controller a switch is on, whose joint reads it: null for a
   * sensor connected anywhere else, or nowhere.
   */
  public static SensorWiring.@Nullable ControllerLimit onPin(Sensor.Placed sensor) {
    return sensor instanceof Sensor.Switch onSwitch
            && onSwitch.wiring().isPresent()
            && onSwitch.wiring().get().input() instanceof SensorWiring.ControllerLimit limit
        ? limit
        : null;
  }

  /**
   * The switch a joint's leading motor controller reads as one of its limits, or null for none: the
   * switch on that limit pin of its own; else, for a TalonFX or TalonFXS, its own limit switch on a
   * CANdi input on its bus, as its reverse limit for the switch at its lower end and its forward
   * for the upper. What's wired to a pin is read through it, so a CANdi's switch takes a limit only
   * where no pin's switch has: a Talon reads one source for each.
   */
  public static Sensor.@Nullable Switch onLimit(
      RobotDefinition definition, Joint joint, SensorWiring.Direction direction) {
    String label = joint.label();
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onPin(onSwitch) instanceof SensorWiring.ControllerLimit limit
          && limit.joint().equals(label)
          && limit.direction() == direction) {
        return onSwitch;
      }
    }
    Sensor.Switch.Limit.End end =
        direction == SensorWiring.Direction.FORWARD
            ? Sensor.Switch.Limit.End.UPPER
            : Sensor.Switch.Limit.End.LOWER;
    for (Sensor sensor : definition.math().sensors()) {
      if (sensor instanceof Sensor.Switch onSwitch
          && ownLimit(onSwitch, label)
          && ((Sensor.Switch.Limit) onSwitch.kind()).end() == end
          && onSwitch.wiring().isPresent()
          && onSwitch.wiring().get().input() instanceof SensorWiring.Candi candi
          && readsCandi(joint, candi)) {
        return onSwitch;
      }
    }
    return null;
  }

  /** Whether a switch is a joint's own limit switch. */
  private static boolean ownLimit(Sensor.Switch sensor, String joint) {
    return sensor.kind() instanceof Sensor.Switch.Limit limit
        && limit.joint().label().equals(joint);
  }

  /** Whether a joint's leading controller can read a CANdi as its limit: a Talon on its bus. */
  private static boolean readsCandi(Joint joint, SensorWiring.Candi candi) {
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    return !controllers.isEmpty()
        && controllers.get(0).bus().equals(candi.bus())
        && (controllers.get(0).controller() == MechanismWiring.Controller.TALON_FX
            || controllers.get(0).controller() == MechanismWiring.Controller.TALON_FXS);
  }
}
