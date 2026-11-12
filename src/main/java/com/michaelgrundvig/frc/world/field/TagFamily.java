package com.michaelgrundvig.frc.world.field;

/**
 * Which AprilTag family a field's tags are printed from: the pattern of black and white squares
 * that a detector looks for. A detector finds only the families it's set for, so the field's family
 * is what a camera's software must detect.
 */
public enum TagFamily {
  /** 36h11: FRC's tags since 2024. */
  TAG_36H11,

  /** 16h5: FRC's tags in 2023, fewer and smaller squares than 36h11's. */
  TAG_16H5
}
