package com.michaelgrundvig.frc.world.record;

/** What a move of the drivetrain does. */
public enum DriveMoveTask {
  /** Turns on the spot to a heading. It arrives by its heading alone. */
  TURN_TO,
  /** Drives to a pose on the field. */
  TO_POSE,
  /** Drives to one of the field's named places, found for the robot's alliance as it starts. */
  TO_PLACE,
  /** Drives a path through keyframes. */
  THROUGH_KEYFRAMES,
  /** Drives at a velocity until a condition holds. */
  RUN_UNTIL,
  /** Drives from the driver's sticks until it is interrupted. */
  WITH_THE_STICKS;

  /** Whether the move follows a planned path to its end, and so says where it got to. */
  public boolean followsAPath() {
    return this == TURN_TO || this == TO_POSE || this == TO_PLACE || this == THROUGH_KEYFRAMES;
  }
}
