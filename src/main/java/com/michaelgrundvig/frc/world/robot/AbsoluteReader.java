package com.michaelgrundvig.frc.world.robot;

import java.util.List;

/**
 * Where a joint is for one absolute encoder's reading, for a reading taken every loop ({@link
 * RobotMath#absoluteReader}).
 *
 * <p>Everything that depends only on the robot's definition is worked out once, as this is made:
 * the encoder's ratio to its joint, and the window of the joint's positions that one turn of the
 * encoder tells apart ({@link RobotMath#absoluteWindow}). Working those out makes objects, about
 * 6,000 bytes of them a time. What is left for each reading is arithmetic on four numbers, which
 * makes none, so the robot's loop can ask it every 20 ms.
 */
public final class AbsoluteReader {
  /** How many turns the encoder makes for one turn of the joint, or for one meter of it. */
  private final double encoderTurnsPerJointTurn;

  /**
   * How far the joint moves for one turn of it: 2π radians, or 1 for a joint measured in meters.
   */
  private final double jointUnitsPerTurn;

  /** The lowest position of the joint in the encoder's window, in the joint's units. */
  private final double windowLower;

  /** How far the window reaches above that: what one turn of the encoder tells apart. */
  private final double windowSpan;

  private AbsoluteReader(
      double encoderTurnsPerJointTurn,
      double jointUnitsPerTurn,
      double windowLower,
      double windowSpan) {
    this.encoderTurnsPerJointTurn = encoderTurnsPerJointTurn;
    this.jointUnitsPerTurn = jointUnitsPerTurn;
    this.windowLower = windowLower;
    this.windowSpan = windowSpan;
  }

  /**
   * What reads one absolute encoder's joint from the encoder's readings.
   *
   * @throws IllegalArgumentException if the encoder is not an absolute encoder on one joint alone
   */
  static AbsoluteReader of(Sensor.Encoder encoder) {
    Range window = AbsoluteEncoders.window(List.of(encoder));
    Joint joint = encoder.joint();
    return new AbsoluteReader(
        Math.abs(encoder.ratio(joint)),
        AbsoluteEncoders.jointUnitsPerTurn(joint),
        window.lower(),
        window.upper() - window.lower());
  }

  /**
   * Where the joint is for a reading, in the joint's units, wrapped into the window the encoder
   * tells apart.
   *
   * @param turns the encoder's reading in its turns, its offset applied so it reads zero with the
   *     joint at zero, and growing as the joint moves forward
   */
  public double position(double turns) {
    double position = turns / encoderTurnsPerJointTurn * jointUnitsPerTurn;
    // Java's remainder keeps the sign of what is divided, so a position below the window gives a
    // negative remainder, which one more span brings into the window.
    double pastTheBottom = (position - windowLower) % windowSpan;
    if (pastTheBottom < 0) {
      pastTheBottom = pastTheBottom + windowSpan;
    }
    return windowLower + pastTheBottom;
  }
}
