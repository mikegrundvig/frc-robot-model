package com.michaelgrundvig.frc.world.physical;

import static org.wpilib.units.Units.Meters;

import org.wpilib.units.measure.Distance;

/**
 * A robot's bumpers seen from above: a rectangle around the robot's origin, its length along the
 * robot's x axis (front to back) and its width along y (side to side). It is what the field needs
 * of a robot to stand it on a start, so that the field never reads the robot's definition.
 *
 * @param length front to back, across the outside of the bumpers
 * @param width side to side, across the outside of the bumpers
 */
public record Bumpers(Distance length, Distance width) {
  public Bumpers {
    if (!(length.in(Meters) > 0) || !(width.in(Meters) > 0)) {
      throw new IllegalArgumentException("bumpers need a length and a width above zero");
    }
  }

  /** Half the length, in meters: from the robot's origin to its front or back bumper. */
  public double halfLengthMeters() {
    return length.in(Meters) / 2;
  }

  /** Half the width, in meters: from the robot's origin to its left or right bumper. */
  public double halfWidthMeters() {
    return width.in(Meters) / 2;
  }
}
