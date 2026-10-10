package com.michaelgrundvig.frc.world.robot;

/**
 * What the robot found, in one loop, when it judged an interlock's condition against its sensors.
 * There are three answers and not two because a reading can be missing, and a missing reading must
 * restrict the joint: a keep-out treats {@link #UNKNOWN} as its region being closed, and a
 * run-only-when treats it as its joint not being allowed to move.
 */
public enum ConditionReading {
  /** The condition holds. */
  HOLDS,
  /** The condition was read, and it does not hold. */
  DOES_NOT_HOLD,
  /**
   * The condition could not be judged: its sensor is lost, or the joint it is about does not know
   * where it is or is not answering.
   */
  UNKNOWN;

  /** The reading for a condition that could be judged. */
  public static ConditionReading of(boolean holds) {
    if (holds) {
      return HOLDS;
    }
    return DOES_NOT_HOLD;
  }
}
