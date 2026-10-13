package com.michaelgrundvig.frc.world.record;

/** How a move ended, whether it is a joint's or the drivetrain's. */
public enum MoveResult {
  /** It got there: it stayed inside its band for 40 ms, or its condition held. */
  ARRIVED,
  /** Its deadline passed first. */
  TIMED_OUT,
  /**
   * Another command took what it requires, its binding released, its gate fell, or the robot was
   * disabled.
   */
  CANCELED,
  /** It would not start, or could not go on. */
  REFUSED
}
