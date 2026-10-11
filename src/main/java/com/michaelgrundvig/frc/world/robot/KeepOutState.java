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

  private static final Optional<Side> LAST_BELOW = Optional.of(Side.BELOW);
  private static final Optional<Side> LAST_ABOVE = Optional.of(Side.ABOVE);

  /**
   * A side, as the side a joint was last outside on. There are only two, so each is made once and a
   * joint that stays on its side makes nothing new each loop.
   */
  static Optional<Side> lastOutsideOn(Side side) {
    return switch (side) {
      case BELOW -> LAST_BELOW;
      case ABOVE -> LAST_ABOVE;
    };
  }

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

  /**
   * Whether this is the state of the loop in which the keep-out began to hold the joint at a point
   * inside its region.
   *
   * @param before what the keep-out remembered in the loop before
   */
  boolean startedHoldingInside(KeepOutState before) {
    return heldInsideAt.isPresent() && before.heldInsideAt().isEmpty();
  }

  /**
   * What a keep-out that remembered this state remembers a loop later. When that is exactly what
   * this state already says, this very object is handed back, so a keep-out with nothing new to
   * remember makes no new state.
   */
  KeepOutState remembering(
      Optional<Side> lastOutside, OptionalDouble heldInsideAt, boolean outputHeldAtEdge) {
    boolean nothingNew =
        this.lastOutside.equals(lastOutside)
            && this.heldInsideAt.equals(heldInsideAt)
            && this.outputHeldAtEdge == outputHeldAtEdge;
    if (nothingNew) {
      return this;
    }
    return new KeepOutState(lastOutside, heldInsideAt, outputHeldAtEdge);
  }
}
