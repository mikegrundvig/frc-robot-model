package com.michaelgrundvig.frc.world.record;

/** What kind of thing a move sends one joint to, or runs it at. */
public enum MoveTarget {
  /**
   * Nothing the record can say: a hold, a homing, a follow that ended before it chased anything, or
   * a move that was refused for a place that is not the joint's own.
   */
  NONE,
  /** One of the joint's named positions. */
  PLACE,
  /** A position that was worked out, and has no name. */
  NUMBER,
  /** One of the joint's named outputs. */
  OUTPUT,
  /** Stopped, the output every joint has, which is in no joint's list of outputs. */
  STOPPED,
  /** A speed that was worked out, which a follow chases. */
  SPEED
}
