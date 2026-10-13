package com.michaelgrundvig.frc.world.record;

/**
 * When a command bound to a gamepad starts, and what stops it. There is one for each of Commands
 * v3's ways to bind.
 */
public enum BindingStart {
  /** {@code onTrue}: starts as the button is pressed, and runs until it ends. */
  ON_PRESS("on press", Edge.PRESS),
  /** {@code onFalse}: starts as the button is let go, and runs until it ends. */
  ON_RELEASE("on release", Edge.RELEASE),
  /** {@code whileTrue}: starts as the button is pressed, and is canceled as it is let go. */
  WHILE_HELD("while held", Edge.PRESS),
  /** {@code whileFalse}: starts as the button is let go, and is canceled as it is pressed. */
  WHILE_RELEASED("while released", Edge.RELEASE),
  /** {@code toggleOnTrue}: each press starts it, or cancels it if it is running. */
  TOGGLE_ON_PRESS("toggled on press", Edge.PRESS),
  /** {@code toggleOnFalse}: each release starts it, or cancels it if it is running. */
  TOGGLE_ON_RELEASE("toggled on release", Edge.RELEASE),
  /** {@code retryWhileTrue}: as while held, and started again if it ends while still held. */
  RETRY_WHILE_HELD("while held, again if it ends", Edge.PRESS),
  /** {@code retryWhileFalse}: as while released, and started again if it ends while released. */
  RETRY_WHILE_RELEASED("while released, again if it ends", Edge.RELEASE);

  /** What a button does that starts a command bound to it. */
  public enum Edge {
    /** The button is pressed. */
    PRESS,
    /** The button is let go. */
    RELEASE
  }

  private final String words;
  private final Edge startsOn;

  BindingStart(String words, Edge startsOn) {
    this.words = words;
    this.startsOn = startsOn;
  }

  /** Which of the button's two edges starts the command. */
  public Edge startsOn() {
    return startsOn;
  }

  /** Whether a press starts the command. If not, a release does. */
  public boolean startsOnPress() {
    return startsOn == Edge.PRESS;
  }

  /** As a binding says it: "while held". */
  public String words() {
    return words;
  }
}
