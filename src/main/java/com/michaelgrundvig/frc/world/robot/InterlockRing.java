package com.michaelgrundvig.frc.world.robot;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * A ring of interlocks: each restricts its joint by where the next interlock's joint is, or by how
 * fast it runs, and the last restricts its joint by the first's. A ring is the only arrangement in
 * which joints can hold each other so that none can move to free another ({@link Deadlock}).
 *
 * @param interlocks the interlocks, in the order they go round the ring
 */
record InterlockRing(List<Interlock> interlocks) {
  InterlockRing {
    interlocks = List.copyOf(interlocks);
  }

  /**
   * Every ring among some interlocks. Each ring is found once, starting from whichever of its
   * joints is restricted earliest in the list. Two interlocks between the same pair of joints make
   * two rings. An interlock on a sensor is about no other joint, so it is part of no ring.
   */
  static List<InterlockRing> among(List<Interlock> interlocks) {
    Search search = new Search(interlocks);
    for (Joint start : search.restricted) {
      search.follow(start, start);
    }
    return search.found;
  }

  /** How many interlocks, and so how many joints, go round the ring. */
  int size() {
    return interlocks.size();
  }

  /** The joints the ring restricts. */
  Set<Joint> joints() {
    Set<Joint> joints = new HashSet<>();
    for (Interlock interlock : interlocks) {
      joints.add(interlock.joint());
    }
    return joints;
  }

  /**
   * The interlock that reads the joint which the interlock at an index restricts. It is the one
   * before it in the ring, and for the first interlock it is the last.
   */
  Interlock readerOf(int index) {
    int before = (index + size() - 1) % size();
    return interlocks.get(before);
  }

  /**
   * The search for rings: a walk from each restricted joint along the interlocks, from a joint to
   * the joint its interlock is about, until the walk comes back to where it started. An instance
   * lives for one call of {@link #among}.
   */
  private static final class Search {
    /** The interlocks that restrict each joint. */
    private final Map<Joint, List<Interlock>> restricting = new LinkedHashMap<>();

    /** The joints that are restricted, in the order the list first restricts them. */
    private final List<Joint> restricted;

    private final List<InterlockRing> found = new ArrayList<>();

    /** The interlocks walked along so far, and the joints they passed through. */
    private final List<Interlock> path = new ArrayList<>();

    private final Set<Joint> onPath = new HashSet<>();

    Search(List<Interlock> interlocks) {
      for (Interlock interlock : interlocks) {
        if (interlock.other().isPresent()) {
          restricting.computeIfAbsent(interlock.joint(), joint -> new ArrayList<>()).add(interlock);
        }
      }
      restricted = new ArrayList<>(restricting.keySet());
    }

    void follow(Joint start, Joint at) {
      onPath.add(at);
      for (Interlock interlock : restricting.getOrDefault(at, List.of())) {
        Joint next = interlock.other().orElseThrow();
        path.add(interlock);
        if (next.equals(start)) {
          found.add(new InterlockRing(path));
        } else if (!onPath.contains(next) && isRestrictedAfter(start, next)) {
          follow(start, next);
        }
        path.removeLast();
      }
      onPath.remove(at);
    }

    /**
     * Whether a joint is first restricted later in the list than the walk's start. The walk only
     * goes on through such joints, so that a ring is found from its earliest joint and not again
     * from each of the others. A joint that nothing restricts leads nowhere, and is not one.
     */
    private boolean isRestrictedAfter(Joint start, Joint joint) {
      return restricted.indexOf(joint) > restricted.indexOf(start);
    }
  }
}
