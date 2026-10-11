package com.michaelgrundvig.frc.world.robot;

import java.util.List;

/**
 * Everything a joint's interlocks are judged against in one loop: how the joint itself is moving,
 * and what each interlock's condition reads. All in the joint's own units (radians or meters).
 *
 * <p>This is the plain form, a value made new for each loop, which suits a test or a replay of a
 * log. The robot's own loop hands {@link InterlockEnforcement#decide} the two parts separately, so
 * that it need make nothing each loop: how the joint is moving ({@link JointMotion}), and the list
 * of condition readings.
 *
 * @param position where the joint is
 * @param velocity how fast it is moving, a second
 * @param acceleration how fast its speed changed over the last loop, a second squared
 * @param conditions what each of the joint's interlocks' conditions reads this loop: one for each
 *     interlock, in the order the interlocks are listed in the {@link InterlockEnforcement}
 */
public record InterlockReadings(
    double position, double velocity, double acceleration, List<ConditionReading> conditions)
    implements JointMotion {
  public InterlockReadings {
    conditions = List.copyOf(conditions);
  }
}
