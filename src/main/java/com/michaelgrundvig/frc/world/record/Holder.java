package com.michaelgrundvig.frc.world.record;

/**
 * One thing that holds a joint short of what it was asked. A joint logs those that hold it this
 * loop, in the order they are applied ({@code Joints/<label>/HeldBy}, empty while nothing holds
 * it), and those that held it as a move on it timed out ({@code Joints/<label>/OutcomeHeldBy}).
 * {@link JointSentences#heldBy} makes the sentence from them.
 *
 * @param kind what holds the joint
 * @param index which one, by the kind: for {@link Kind#INTERLOCK} the interlock's index in the
 *     definition's {@code everyInterlock()}; for {@link Kind#LIMIT_SWITCH} the switch's index in
 *     the definition's {@code everySensor()}; for {@link Kind#NO_POSITION} the joint's index in the
 *     definition's {@code joints()}
 */
public record Holder(Kind kind, int index) {
  /** What holds a joint. */
  public enum Kind {
    /** One of the interlocks on the joint. */
    INTERLOCK,
    /** One of the joint's limit switches, tripped. */
    LIMIT_SWITCH,
    /**
     * The joint's own position, which is not a number: left idle with nowhere to hold, the joint is
     * neutral.
     */
    NO_POSITION
  }

  /** An interlock, by its index in the definition's {@code everyInterlock()}. */
  public static Holder interlock(int index) {
    return new Holder(Kind.INTERLOCK, index);
  }

  /** A tripped limit switch, by its index in the definition's {@code everySensor()}. */
  public static Holder limitSwitch(int index) {
    return new Holder(Kind.LIMIT_SWITCH, index);
  }

  /** A joint's position that is not a number, by the joint's index in {@code joints()}. */
  public static Holder noPosition(int joint) {
    return new Holder(Kind.NO_POSITION, joint);
  }
}
