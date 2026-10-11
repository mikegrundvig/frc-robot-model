package com.michaelgrundvig.frc.world.robot;

/**
 * How a joint is moving in one loop, in the joint's own units (radians or meters).
 *
 * <p>Whatever is handed one reads it during the call and keeps nothing of it. So the robot program
 * need not make a new object every loop: it may hand over the same one each time, holding that
 * loop's numbers, as long as it does not change them while the call runs.
 */
public interface JointMotion {
  /** Where the joint is. */
  double position();

  /** How fast it is moving, a second. */
  double velocity();

  /** How fast its speed changed over the last loop, a second squared. */
  double acceleration();
}
