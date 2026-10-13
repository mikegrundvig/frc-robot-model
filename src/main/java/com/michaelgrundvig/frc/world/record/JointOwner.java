package com.michaelgrundvig.frc.world.record;

/**
 * Who asked something of a joint this loop ({@code Joints/<label>/Owner}). The name of the command
 * that asked is a team's text, logged beside it ({@code Joints/<label>/Command}, empty when no
 * command asked). {@link JointSentences#owner} makes the sentence from both.
 */
public enum JointOwner {
  /** Nothing asked: the joint does what it does idle. */
  IDLE,
  /** A command asked. */
  COMMAND,
  /** Code outside any command asked. */
  OUTSIDE_ANY_COMMAND
}
