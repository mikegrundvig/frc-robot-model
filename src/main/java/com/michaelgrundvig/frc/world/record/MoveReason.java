package com.michaelgrundvig.frc.world.record;

/**
 * Why a move ended as it did, where its result alone does not say. Each reason is one of a closed
 * set. Some come with text that only the robot's code can give, which is logged beside the record
 * ({@code OutcomeWhy}), and each of those says so here.
 */
public enum MoveReason {
  /**
   * Nothing more to say: it arrived, its deadline passed, or another command or a released binding
   * canceled it.
   */
  NONE,
  /** The robot was disabled, which cancels every command. */
  ROBOT_DISABLED,
  /**
   * A gate the move starts only under ({@code onlyIf}, {@code onlyWhile}) was false as it would
   * start. The text beside the record names the gate and the terms that kept it false.
   */
  GATE_FALSE,
  /**
   * The gate the move runs only under ({@code onlyWhile}) fell while it ran. The text beside the
   * record names the gate and the terms that made it fall.
   */
  GATE_FELL,
  /**
   * The condition the move ran until never held before its deadline, and the condition is a gate,
   * which says why. The text beside the record names the gate and the terms that kept it false.
   */
  CONDITION_NEVER_HELD,
  /**
   * The move could never run, as it was built: a place that is not the joint's own, an option that
   * cannot be, a joint with no mechanism. The text beside the record says which, and what to do.
   */
  REFUSED_AS_BUILT,
  /** A joint's motor controllers are silent. The record says which joint. */
  JOINT_NOT_ANSWERING,
  /**
   * A joint is faulted. The record says which joint, and the text beside it is the joint's fault,
   * as the joint's own {@code Fault} says it.
   */
  JOINT_FAULTED,
  /**
   * Where a joint was sent is outside its limits. The record says which joint and the position, and
   * the text beside it says the limits.
   */
  TARGET_UNREACHABLE,
  /** A joint's position stopped being a number. The record says which joint. */
  POSITION_LOST,
  /**
   * A follow's target was not a position or a speed the joint takes. The text beside the record
   * says what it was given.
   */
  TARGET_WRONG_KIND,
  /**
   * The drivetrain could plan no path. The text beside the record says why: no alliance yet, or
   * what the planner refused.
   */
  PATH_NOT_PLANNED,
  /** The drivetrain was never handed the driver's sticks. */
  NO_DRIVER
}
