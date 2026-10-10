package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.robot.Interlock.KeepsOut.Side;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * What one keep-out remembers from one loop to the next.
 *
 * <p>The two holds are never both in force: {@code heldInsideAt} is only present while the joint is
 * inside the region, and {@code outputHeldAtEdge} is only true while it is outside.
 *
 * @param lastOutside the side of the region the joint was on when it was last outside it, or empty
 *     if it has not been outside yet. It is tracked whether the region is open or closed, because a
 *     joint that is inside when the region closes may leave only by the side it came in from.
 * @param heldInsideAt where the joint is being held inside the closed region, or empty while it is
 *     not. It is worked out once, as the hold starts, as the point where the joint could stop.
 *     Worked out afresh each loop, the point would run ahead of a joint that has not slowed yet,
 *     and the joint would follow it through the region.
 * @param outputHeldAtEdge whether the joint, asked to run at an output, is being held at the
 *     region's edge. Once that hold starts it has to be remembered: a joint that has come to rest
 *     at the edge no longer looks as though it would reach the region, but if it were let go while
 *     its output still pushed it that way, it would cross.
 */
public record KeepOutState(
    Optional<Side> lastOutside, OptionalDouble heldInsideAt, boolean outputHeldAtEdge) {
  /**
   * A keep-out as the robot starts: the joint has not been seen outside it, and nothing is held.
   */
  public static final KeepOutState INITIAL = notHolding(Optional.empty());

  /** A keep-out that is holding nothing this loop, the joint last outside on the side given. */
  public static KeepOutState notHolding(Optional<Side> lastOutside) {
    return new KeepOutState(lastOutside, OptionalDouble.empty(), false);
  }

  /** A keep-out holding the joint at a point inside its region. */
  public static KeepOutState heldInside(Optional<Side> lastOutside, double at) {
    return new KeepOutState(lastOutside, OptionalDouble.of(at), false);
  }

  /** A keep-out holding a joint that is run at an output at its region's edge. */
  public static KeepOutState heldAtEdge(Optional<Side> lastOutside) {
    return new KeepOutState(lastOutside, OptionalDouble.empty(), true);
  }
}
