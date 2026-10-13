package com.michaelgrundvig.frc.world.record;

/** How far a run of a move got before it ended. */
public enum MoveProgress {
  /**
   * It ended before it began: it was refused as it started, or it was canceled while it waited for
   * the robot to be enabled or for its joints to home. It drove nothing.
   */
  NOT_BEGUN,
  /** It began, and so drove its joints, or pushed its joint home. */
  BEGUN
}
