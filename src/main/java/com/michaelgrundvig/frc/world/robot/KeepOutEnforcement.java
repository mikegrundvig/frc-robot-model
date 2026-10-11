package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.robot.Interlock.KeepsOut.Side;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * Decides what one keep-out allows its joint to do in one loop. {@link InterlockEnforcement} makes
 * one of these for each of its keep-outs, once, and asks it every loop.
 *
 * <p>Like the enforcement it is part of, it holds only facts that never change while the robot
 * runs, and it makes nothing new in a loop where the keep-out has nothing new to remember. A
 * decision is asked for in three steps, each a function of plain values:
 *
 * <ol>
 *   <li>{@link #ruling} says what the keep-out does about what the joint is asked: one of four
 *       named things ({@link Ruling}).
 *   <li>{@link #remembers} says what the keep-out remembers after ruling so.
 *   <li>{@link #holds} says whether the ruling changes what the joint was asked, and {@link
 *       #heldAt} says where the keep-out then holds the joint.
 * </ol>
 *
 * <p>The ruling depends on three things, and the methods below are split along them: whether the
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
 */
final class KeepOutEnforcement {
  /** What a keep-out does, in one loop, about what its joint is asked. */
  enum Ruling {
    /** The joint does what it was asked, and the keep-out holds nothing. */
    LEFT_ALONE,
    /** Asked for a position beyond the region's edge, the joint is stopped at that edge. */
    STOPPED_AT_EDGE,
    /**
     * Asked to run at an output, the joint is held at the region's edge. Unlike a joint stopped
     * there on its way to a position, this hold is remembered ({@link
     * KeepOutState#outputHeldAtEdge}).
     */
    OUTPUT_HELD_AT_EDGE,
    /** Inside the region with nowhere it may go, the joint is held at the point where it stops. */
    HELD_INSIDE
  }

  private final Interlock.KeepsOut keepOut;
  private final Travel travel;
  private final double loopSeconds;

  /**
   * @param travel where the joint can be driven and how quickly it stops
   * @param loopSeconds how long one loop lasts, in seconds
   */
  KeepOutEnforcement(Interlock.KeepsOut keepOut, Travel travel, double loopSeconds) {
    this.keepOut = keepOut;
    this.travel = travel;
    this.loopSeconds = loopSeconds;
  }

  /**
   * What the keep-out does this loop about what the joint is asked.
   *
   * @param before what the keep-out remembered from the last loop
   * @param closed whether the region is closed this loop
   * @param asked what the joint is asked to do, as the interlocks before this one left it
   * @param motion how the joint is moving
   */
  Ruling ruling(KeepOutState before, boolean closed, JointAsk asked, JointMotion motion) {
    if (!closed) {
      return Ruling.LEFT_ALONE;
    }
    return switch (asked) {
      case JointAsk.Neutral neutral -> Ruling.LEFT_ALONE;
      case JointAsk.ToPosition toPosition -> askedForAPosition(before, toPosition, motion);
      case JointAsk.AtOutput output -> askedForAnOutput(before, output, motion);
    };
  }

  /**
   * The joint is asked for a position while the region is closed. All that matters is whether the
   * goal is somewhere the joint may go: on the side it is kept to, at the edge or beyond it. That
   * is the same test for a joint outside the region, which is then simply staying out, and for a
   * joint inside it, which is then leaving the way it came in.
   */
  private Ruling askedForAPosition(
      KeepOutState before, JointAsk.ToPosition toPosition, JointMotion motion) {
    // The goal is judged where the joint's profile will really take it. A joint that turns freely
    // goes the short way round, so asked for -170° from 90° it is headed for 190°.
    double goal = travel.goalNear(toPosition.position(), motion.position());
    boolean goalIsWhereItMayGo = keepOut.distancePastEdge(sideKeptTo(before, motion), goal) <= 0;
    if (goalIsWhereItMayGo) {
      return Ruling.LEFT_ALONE;
    }
    if (isInside(motion)) {
      return Ruling.HELD_INSIDE;
    }
    return Ruling.STOPPED_AT_EDGE;
  }

  /** The joint is asked to run at an output while the region is closed. */
  private Ruling askedForAnOutput(
      KeepOutState before, JointAsk.AtOutput output, JointMotion motion) {
    if (isInside(motion)) {
      return Ruling.HELD_INSIDE;
    }
    Side side = sideKeptTo(before, motion);
    boolean alreadyHeld = before.outputHeldAtEdge();
    boolean staysHeld = alreadyHeld && pushesTowardRegion(side, output);
    if (wouldReachRegion(side, output, motion) || staysHeld) {
      return Ruling.OUTPUT_HELD_AT_EDGE;
    }
    return Ruling.LEFT_ALONE;
  }

  /**
   * Whether a joint run at an output has to be held now to stop short of the region.
   *
   * <p>A hold decided this loop only takes effect in the next one, so the question is where the
   * joint would stop if it ran on for one more loop and then braked. It is asked twice: carrying on
   * as the joint is moving now, and moving at the speed the output asks for, because a light joint
   * reaches that speed within a single loop, even from rest. If either would end at the region's
   * edge or past it, the joint has to be held now. Waiting a loop longer would be too late.
   *
   * @param side the side of the region the joint is on
   */
  private boolean wouldReachRegion(Side side, JointAsk.AtOutput output, JointMotion motion) {
    double stopsAsItMoves =
        travel.stoppingPointLoopOn(
            motion.position(), motion.velocity(), motion.acceleration(), loopSeconds);
    double stopsAtSpeedAsked =
        travel.stoppingPointLoopOn(
            motion.position(), output.unloadedSpeed(travel.freeSpeed()), 0, loopSeconds);
    double furthestPastEdge =
        Math.max(
            keepOut.distancePastEdge(side, stopsAsItMoves),
            keepOut.distancePastEdge(side, stopsAtSpeedAsked));
    return furthestPastEdge >= 0;
  }

  /** Whether an output drives the joint toward the region, from the side the joint is on. */
  private static boolean pushesTowardRegion(Side side, JointAsk.AtOutput output) {
    return switch (side) {
      case BELOW -> output.value() > 0;
      case ABOVE -> output.value() < 0;
    };
  }

  /**
   * What the keep-out remembers after ruling as it did this loop.
   *
   * @param before what it remembered from the last loop. That very object is handed back when the
   *     keep-out has nothing new to remember.
   */
  KeepOutState remembers(Ruling ruling, KeepOutState before, JointMotion motion) {
    Optional<Side> lastOutside = lastOutside(before, motion);
    OptionalDouble nowhereInside = OptionalDouble.empty();
    return switch (ruling) {
      case LEFT_ALONE, STOPPED_AT_EDGE -> before.remembering(lastOutside, nowhereInside, false);
      case OUTPUT_HELD_AT_EDGE -> before.remembering(lastOutside, nowhereInside, true);
      case HELD_INSIDE -> before.remembering(lastOutside, heldInsideAt(before, motion), false);
    };
  }

  /**
   * The point inside the region where the joint is held: where it can stop. The point is worked out
   * in the loop the hold starts and kept after that ({@link KeepOutState#heldInsideAt} says why).
   */
  private OptionalDouble heldInsideAt(KeepOutState before, JointMotion motion) {
    if (before.heldInsideAt().isPresent()) {
      return before.heldInsideAt();
    }
    return OptionalDouble.of(travel.stoppingPoint(motion.position(), motion.velocity()));
  }

  /**
   * Whether the ruling changes what the joint was asked, which makes this keep-out one of the
   * interlocks holding it.
   *
   * <p>A joint left alone is not changed. Nor is a joint the ruling holds at exactly the position
   * it was already asked for: it keeps its command's share of the cruise, and the keep-out is not
   * reported as holding it.
   *
   * @param now what the keep-out remembers after this ruling
   * @param asked what the joint is asked to do, as the interlocks before this one left it
   */
  boolean holds(Ruling ruling, KeepOutState now, JointAsk asked) {
    if (ruling == Ruling.LEFT_ALONE) {
      return false;
    }
    boolean alreadyAskedForIt =
        asked instanceof JointAsk.ToPosition toPosition && toPosition.position() == heldAt(now);
    return !alreadyAskedForIt;
  }

  /**
   * Where a keep-out that holds the joint holds it: at the point inside the region that it
   * remembers, or otherwise at the region's edge on the joint's side.
   *
   * @param now what the keep-out remembers after a ruling that holds the joint
   */
  double heldAt(KeepOutState now) {
    if (now.heldInsideAt().isPresent()) {
      return now.heldInsideAt().getAsDouble();
    }
    // A joint that is held, but not at a point inside the region, is outside the region. So the
    // side it was last outside on is the side it is on now.
    return keepOut.edge(now.lastOutside().orElseThrow());
  }

  /** Whether the joint is inside the region now. */
  private boolean isInside(JointMotion motion) {
    return keepOut.isInside(motion.position());
  }

  /**
   * The side of the region the joint is kept to: the side it is on, or the side it may leave by.
   */
  private Side sideKeptTo(KeepOutState before, JointMotion motion) {
    return keepOut.sideKeptTo(motion.position(), before.lastOutside());
  }

  /** The side the joint was last outside on, brought up to date with where it is now. */
  private Optional<Side> lastOutside(KeepOutState before, JointMotion motion) {
    if (isInside(motion)) {
      return before.lastOutside();
    }
    return KeepOutState.lastOutsideOn(sideKeptTo(before, motion));
  }
}
