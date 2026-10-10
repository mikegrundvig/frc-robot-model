package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.robot.Interlock.KeepsOut.Side;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * What one keep-out allows its joint to do in one loop. {@link InterlockEnforcement} asks this of
 * each keep-out in turn.
 *
 * <p>The answer depends on three things, and the methods below are split along them: whether the
 * region is closed, whether the joint is inside it, and what kind of thing the joint is asked to
 * do.
 *
 * <ul>
 *   <li>While the region is open, and whenever the joint is asked for neutral, the joint is left
 *       alone.
 *   <li>Asked for a position, a joint outside the region is stopped at the region's edge if its
 *       goal lies beyond that edge. A joint inside the region may go to a goal that takes it out
 *       the way it came in, and is otherwise held where it can stop.
 *   <li>Asked to run at an output, a joint outside the region is held at the edge from the loop in
 *       which it could no longer stop short of it. A joint inside the region is held where it can
 *       stop.
 * </ul>
 *
 * <p>A joint the region closes on while it is moving too fast to stop inside it is not hauled back:
 * it carries on out of the far side, and from then on is kept out from that side.
 *
 * <p>An instance exists only for the length of one decision. It is the decision's working, not
 * something kept: what is kept between loops is the {@link KeepOutState} in the ruling.
 */
final class KeepOutDecision {
  /**
   * What a keep-out decided.
   *
   * @param state what the keep-out remembers now
   * @param allowed what the joint may do: what it was asked, or a position this keep-out holds it
   *     at
   * @param holds whether this keep-out changed what the joint was asked, and so is one of the
   *     interlocks holding it
   * @param startedHoldingInside whether this is the loop in which the keep-out began to hold the
   *     joint at a point inside its region, where the joint's profile must start again from where
   *     the joint is
   */
  record Ruling(
      KeepOutState state, JointAsk allowed, boolean holds, boolean startedHoldingInside) {}

  private final Interlock.KeepsOut keepOut;
  private final Travel travel;
  private final double loopSeconds;
  private final KeepOutState previous;
  private final JointAsk asked;
  private final InterlockReadings readings;

  /** Whether the joint is inside the region now. */
  private final boolean inside;

  /**
   * The side of the region the joint is kept to: the side it is on, or the side it may leave by.
   */
  private final Side side;

  /** The side the joint was last outside on, brought up to date with where it is now. */
  private final Optional<Side> lastOutside;

  private KeepOutDecision(
      Interlock.KeepsOut keepOut,
      Travel travel,
      double loopSeconds,
      KeepOutState previous,
      JointAsk asked,
      InterlockReadings readings) {
    this.keepOut = keepOut;
    this.travel = travel;
    this.loopSeconds = loopSeconds;
    this.previous = previous;
    this.asked = asked;
    this.readings = readings;
    inside = keepOut.isInside(readings.position());
    side = keepOut.sideKeptTo(readings.position(), previous.lastOutside());
    if (inside) {
      lastOutside = previous.lastOutside();
    } else {
      lastOutside = Optional.of(side);
    }
  }

  /**
   * What the keep-out allows this loop.
   *
   * @param travel where the joint can be driven and how quickly it stops
   * @param loopSeconds how long one loop lasts, in seconds
   * @param previous what the keep-out remembered from the last loop
   * @param closed whether the region is closed this loop
   * @param asked what the joint is asked to do, as the interlocks before this one left it
   * @param readings how the joint is moving; its condition readings are not used here
   */
  static Ruling decide(
      Interlock.KeepsOut keepOut,
      Travel travel,
      double loopSeconds,
      KeepOutState previous,
      boolean closed,
      JointAsk asked,
      InterlockReadings readings) {
    KeepOutDecision decision =
        new KeepOutDecision(keepOut, travel, loopSeconds, previous, asked, readings);
    if (!closed) {
      return decision.leftAlone();
    }
    return switch (asked) {
      case JointAsk.Neutral neutral -> decision.leftAlone();
      case JointAsk.ToPosition toPosition -> decision.askedForAPosition(toPosition);
      case JointAsk.AtOutput output -> decision.askedForAnOutput(output);
    };
  }

  /** The joint does what it was asked, and the keep-out holds nothing. */
  private Ruling leftAlone() {
    return new Ruling(KeepOutState.notHolding(lastOutside), asked, false, false);
  }

  /**
   * The joint is asked for a position while the region is closed. All that matters is whether the
   * goal is somewhere the joint may go: on the side it is kept to, at the edge or beyond it. That
   * is the same test for a joint outside the region, which is then simply staying out, and for a
   * joint inside it, which is then leaving the way it came in.
   */
  private Ruling askedForAPosition(JointAsk.ToPosition toPosition) {
    // The goal is judged where the joint's profile will really take it. A joint that turns freely
    // goes the short way round, so asked for -170° from 90° it is headed for 190°.
    double goal = travel.goalNear(toPosition.position(), readings.position());
    boolean goalIsWhereItMayGo = keepOut.distancePastEdge(side, goal) <= 0;
    if (goalIsWhereItMayGo) {
      return leftAlone();
    }
    if (inside) {
      return heldInside();
    }
    return heldAt(keepOut.edge(side), KeepOutState.notHolding(lastOutside), false);
  }

  /** The joint is asked to run at an output while the region is closed. */
  private Ruling askedForAnOutput(JointAsk.AtOutput output) {
    if (inside) {
      return heldInside();
    }
    boolean alreadyHeld = previous.outputHeldAtEdge();
    boolean staysHeld = alreadyHeld && pushesTowardRegion(output);
    if (wouldReachRegion(output) || staysHeld) {
      return heldAt(keepOut.edge(side), KeepOutState.heldAtEdge(lastOutside), false);
    }
    return leftAlone();
  }

  /**
   * Whether a joint run at an output has to be held now to stop short of the region.
   *
   * <p>A hold decided this loop only takes effect in the next one, so the question is where the
   * joint would stop if it ran on for one more loop and then braked. It is asked twice: carrying on
   * as the joint is moving now, and moving at the speed the output asks for, because a light joint
   * reaches that speed within a single loop, even from rest. If either would end at the region's
   * edge or past it, the joint has to be held now. Waiting a loop longer would be too late.
   */
  private boolean wouldReachRegion(JointAsk.AtOutput output) {
    double stopsAsItMoves =
        travel.stoppingPointLoopOn(
            readings.position(), readings.velocity(), readings.acceleration(), loopSeconds);
    double stopsAtSpeedAsked =
        travel.stoppingPointLoopOn(
            readings.position(), output.unloadedSpeed(travel.freeSpeed()), 0, loopSeconds);
    double furthestPastEdge =
        Math.max(
            keepOut.distancePastEdge(side, stopsAsItMoves),
            keepOut.distancePastEdge(side, stopsAtSpeedAsked));
    return furthestPastEdge >= 0;
  }

  /** Whether an output drives the joint toward the region, from the side the joint is on. */
  private boolean pushesTowardRegion(JointAsk.AtOutput output) {
    return switch (side) {
      case BELOW -> output.value() > 0;
      case ABOVE -> output.value() < 0;
    };
  }

  /**
   * Holds the joint inside the region, at the point where it can stop. The point is worked out in
   * the loop the hold starts and kept after that ({@link KeepOutState#heldInsideAt} says why).
   */
  private Ruling heldInside() {
    OptionalDouble alreadyHeldAt = previous.heldInsideAt();
    if (alreadyHeldAt.isPresent()) {
      double point = alreadyHeldAt.getAsDouble();
      return heldAt(point, KeepOutState.heldInside(lastOutside, point), false);
    }
    double point = travel.stoppingPoint(readings.position(), readings.velocity());
    return heldAt(point, KeepOutState.heldInside(lastOutside, point), true);
  }

  /**
   * Holds the joint at a position, moving there at its full cruise.
   *
   * <p>If the joint was already asked for exactly that position, there is nothing to change: it
   * keeps its command's share of the cruise, and the keep-out is not reported as holding it.
   */
  private Ruling heldAt(double position, KeepOutState state, boolean startedHoldingInside) {
    boolean alreadyAskedForIt =
        asked instanceof JointAsk.ToPosition toPosition && toPosition.position() == position;
    if (alreadyAskedForIt) {
      return new Ruling(state, asked, false, startedHoldingInside);
    }
    JointAsk hold = new JointAsk.ToPosition(position, JointAsk.FULL_CRUISE);
    return new Ruling(state, hold, true, startedHoldingInside);
  }
}
