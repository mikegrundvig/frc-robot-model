package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.robot.Interlock.KeepsOut.Side;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Whether a ring of interlocks deadlocks its joints where the robot is put: every joint in the ring
 * is held by its interlock, and nowhere a held joint may still go frees the joint that waits on it,
 * so none of them can ever move to free another.
 *
 * <p>What a held joint may still do depends on the kind of interlock holding it ({@link Held}). A
 * joint held by a run-only-when stays where it is, and still. A joint held by a keep-out may still
 * go anywhere on its side of the region. A held joint frees the joint that waits on it if anything
 * it may still do opens that joint's interlock.
 *
 * <p>So an arm kept out of the bumper's band while its elevator is low, and the elevator kept up
 * while the arm hangs below the band, are no deadlock: kept below the band, the arm can still rise
 * to the band's edge, which is no longer "below", and that lets the elevator go. The elevator kept
 * low while the arm is above the band is a deadlock: with the arm up and the elevator down, each is
 * held to the side that holds the other.
 *
 * <p>A ring is judged only where the robot is put: as it starts, and in each configuration. Joints
 * can get into a deadlock anywhere else only by moving into it together, each in the very loop the
 * other's interlock closes, so nothing else is refused. Two named positions that no configuration
 * puts together are not refused either, since a pair of interlocks like the one above exists to
 * keep the robot out of exactly that.
 */
final class Deadlock {
  private Deadlock() {}

  /**
   * What a joint that is held by its interlock may still do.
   *
   * <p>A stretch of travel may have an infinite end, for a joint with no stop that way.
   */
  private sealed interface Held {
    /** Held by a run-only-when: the joint stays where it is, and does not run. */
    record Still(Joint joint, double at) implements Held {}

    /**
     * Held by a keep-out: the joint may go anywhere from one position to another, on its side of
     * the region.
     */
    record ToItsSide(Joint joint, double from, double to) implements Held {}
  }

  /**
   * How a ring deadlocks its joints with the robot placed one way, if it does.
   *
   * @return the deadlock as a refusal says it, ready to follow what puts the robot there: "leaves
   *     arm and elevator holding each other by their interlocks, ..."
   */
  static Optional<String> of(InterlockRing ring, Placement placement) {
    List<String> holds = new ArrayList<>();
    for (int index = 0; index < ring.size(); index++) {
      Interlock interlock = ring.interlocks().get(index);
      Interlock reader = ring.readerOf(index);
      Optional<Held> held = heldBy(interlock, placement);
      if (held.isEmpty() || frees(held.get(), reader)) {
        return Optional.empty();
      }
      String stays = howItStays(held.get(), reader);
      holds.add(interlock + ", so " + interlock.joint().label() + " " + stays);
    }
    return Optional.of(
        "leaves "
            + labelsOf(ring)
            + " holding each other by their interlocks, so "
            + (ring.size() == 2 ? "neither can free the other" : "none can free another")
            + ": a deadlock ("
            + String.join("; ", holds)
            + ")");
  }

  /**
   * What an interlock's joint may still do, if the interlock is holding it with the robot placed
   * this way. A condition that cannot be read before the robot runs (a sensor, or the speed of a
   * joint run at a voltage) is not taken to hold the joint: a deadlock is refused only where it can
   * be shown.
   */
  private static Optional<Held> heldBy(Interlock interlock, Placement placement) {
    ConditionReading condition = placement.reading(interlock.condition());
    boolean holdsItsJoint =
        switch (interlock) {
          case Interlock.RunsOnlyWhen runsOnlyWhen -> condition == ConditionReading.DOES_NOT_HOLD;
          case Interlock.KeepsOut keepOut -> condition == ConditionReading.HOLDS;
        };
    if (!holdsItsJoint) {
      return Optional.empty();
    }
    Joint joint = interlock.joint();
    double here = placement.positionOf(joint);
    return Optional.of(
        switch (interlock) {
          case Interlock.RunsOnlyWhen runsOnlyWhen -> new Held.Still(joint, here);
          case Interlock.KeepsOut keepOut -> keptToItsSide(keepOut, here);
        });
  }

  /**
   * Where a joint may still go while its keep-out is closed: from the end of its range to the
   * region's edge, on the side it is kept to. A joint that is inside the region may also be
   * anywhere between where it is and that edge, since it is let out that way, as on the robot.
   */
  private static Held.ToItsSide keptToItsSide(Interlock.KeepsOut keepOut, double here) {
    Joint joint = keepOut.joint();
    Range region = keepOut.region();
    if (keepOut.sideKeptTo(here) == Side.BELOW) {
      double lowest = joint.range().map(Range::lower).orElse(Double.NEGATIVE_INFINITY);
      return new Held.ToItsSide(joint, lowest, Math.max(here, region.lower()));
    }
    double highest = joint.range().map(Range::upper).orElse(Double.POSITIVE_INFINITY);
    return new Held.ToItsSide(joint, Math.min(here, region.upper()), highest);
  }

  /**
   * Whether a held joint frees the joint that waits on it: whether anything the held joint may
   * still do opens the interlock that reads it.
   *
   * @param reader the interlock whose condition is about the held joint
   */
  private static boolean frees(Held held, Interlock reader) {
    if (reader.condition() instanceof Condition.OnJoint onPosition) {
      return freesByPosition(held, reader, onPosition);
    }
    // Every other condition in a ring is on the held joint's speed. A joint that is held still
    // can never reach a speed, so it cannot free a joint that runs only when it is up to speed.
    // In every other case the reader is freed: a joint kept to one side of a region may still
    // run, and a keep-out that is closed while the held joint is up to speed opens as it stops.
    boolean heldStill = held instanceof Held.Still;
    boolean readerWaitsForSpeed = reader instanceof Interlock.RunsOnlyWhen;
    return !(heldStill && readerWaitsForSpeed);
  }

  /** Whether a held joint can be anywhere that opens an interlock on its position. */
  private static boolean freesByPosition(
      Held held, Interlock reader, Condition.OnJoint onPosition) {
    for (double position : positionsItMayBeAt(held, onPosition)) {
      if (letsItsJointMove(reader, onPosition.holdsAt(position))) {
        return true;
      }
    }
    return false;
  }

  /** The positions to try a condition at, among those a held joint may still be at. */
  private static Iterable<Double> positionsItMayBeAt(Held held, Condition.OnJoint condition) {
    return switch (held) {
      case Held.Still still -> List.of(still.at());
      case Held.ToItsSide side -> TrialPositions.within(side.from(), side.to(), condition);
    };
  }

  /**
   * Whether an interlock lets its joint move, given whether its condition holds. A run-only-when
   * lets its joint move while its condition holds; a keep-out, while its condition does not.
   */
  private static boolean letsItsJointMove(Interlock interlock, boolean conditionHolds) {
    return switch (interlock) {
      case Interlock.RunsOnlyWhen runsOnlyWhen -> conditionHolds;
      case Interlock.KeepsOut keepOut -> !conditionHolds;
    };
  }

  /**
   * What a held joint may still do, as the refusal says it: "stays at 30°", "stays still" to a
   * reader that waits on its speed, or "stays at 0.3 m or below".
   */
  private static String howItStays(Held held, Interlock reader) {
    return switch (held) {
      case Held.Still still -> howItStaysStill(still, reader);
      case Held.ToItsSide side -> "stays " + shownStretch(side);
    };
  }

  private static String howItStaysStill(Held.Still still, Interlock reader) {
    boolean readerWaitsOnItsSpeed = reader.condition() instanceof Condition.UpToSpeed;
    if (readerWaitsOnItsSpeed) {
      return "stays still";
    }
    return "stays at " + still.joint().shown(still.at());
  }

  /**
   * A stretch of a joint's travel as people read it: "between -30° and 40°", "at 0.3 m or above".
   */
  private static String shownStretch(Held.ToItsSide stretch) {
    Joint joint = stretch.joint();
    double from = stretch.from();
    double to = stretch.to();
    if (from == to) {
      return "at " + joint.shown(from);
    }
    if (!Double.isFinite(from)) {
      return "at " + joint.shown(to) + " or below";
    }
    if (!Double.isFinite(to)) {
      return "at " + joint.shown(from) + " or above";
    }
    return "between " + joint.shown(from) + " and " + joint.shown(to);
  }

  /**
   * The joints of a ring as a sentence lists them: "arm and elevator", "arm, wrist and elevator".
   */
  private static String labelsOf(InterlockRing ring) {
    List<String> labels =
        ring.interlocks().stream().map(interlock -> interlock.joint().label()).toList();
    String allButTheLast = String.join(", ", labels.subList(0, labels.size() - 1));
    return allButTheLast + " and " + labels.getLast();
  }
}
