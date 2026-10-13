package com.michaelgrundvig.frc.world.record;

/** What a move of the robot's joints does. Every joint of one move has the same task. */
public enum MoveTask {
  /** Goes to its targets, and ends when every joint is there. */
  GO_TO,
  /** Stays at its targets, each joint at its output or its position, until it is canceled. */
  RUN,
  /** Stays at its targets until a condition holds, which it ends on. */
  RUN_UNTIL,
  /** Chases a target that may change every loop, until it is canceled. */
  FOLLOW,
  /** Holds the joint at the goal it has as the move starts, until it is canceled. */
  HOLD,
  /** Homes the joint again. */
  HOME
}
