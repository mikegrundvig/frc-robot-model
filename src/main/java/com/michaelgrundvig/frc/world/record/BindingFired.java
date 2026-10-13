package com.michaelgrundvig.frc.world.record;

/**
 * One binding that a press or a release started this loop ({@code Controls/Fired}, empty in a loop
 * that started none). Beside it is the name of the command it started, a team's text ({@code
 * Controls/FiredCommands}, in the same order). The binding's whole sentence is its entry in the
 * list of the robot's bindings ({@code Controls/Bindings}), which {@link ControlSentences#binding}
 * makes.
 *
 * @param binding which of the robot's standing bindings fired: its index in the order they were
 *     made, which is its index in {@code Controls/Bindings}
 * @param port the Driver Station port of the gamepad it is on: 0 for the driver's, 1 for the
 *     operator's
 * @param control what kind of control on the gamepad starts it
 * @param button for {@link Control#BUTTON}, the button's index as WPILib's gamepad numbers its
 *     buttons; {@link Identity#NONE} for every other control
 * @param start when the command starts, and what stops it
 */
public record BindingFired(int binding, int port, Control control, int button, BindingStart start) {
  /** What kind of control on a gamepad a binding is made on. */
  public enum Control {
    /** One button. */
    BUTTON,
    /** The left trigger, pushed past a threshold. */
    LEFT_TRIGGER,
    /** The right trigger, pushed past a threshold. */
    RIGHT_TRIGGER,
    /**
     * Something made from a control: two buttons together, a button with a gate, or a button
     * debounced or negated. The binding's sentence says what.
     */
    COMBINED
  }
}
