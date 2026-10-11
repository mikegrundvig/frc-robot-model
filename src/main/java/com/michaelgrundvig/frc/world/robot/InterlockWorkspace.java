package com.michaelgrundvig.frc.world.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Scratch space for one decision of {@link InterlockEnforcement#decide}: the lists the decision
 * fills in as it goes through the interlocks.
 *
 * <p>The model holds no state, so the workspace belongs to whoever calls {@code decide}. The robot
 * program makes one for each joint and hands the same one in every loop, which is what lets a loop
 * in which nothing changes make no new objects: the decision is worked out in these lists, compared
 * with last loop's, and only a decision that differs is copied out of them.
 *
 * <p>Nothing in a workspace means anything between calls. {@code decide} empties it before using
 * it, and nothing it returns refers to it. The one rule is that two decisions must not use the same
 * workspace at the same time.
 */
public final class InterlockWorkspace {
  /** The interlocks found to hold the joint so far in this decision. */
  final List<Interlock> heldBy = new ArrayList<>();

  /** What each keep-out reached so far in this decision remembers now. */
  final List<KeepOutState> keepOuts = new ArrayList<>();

  /** Empties the workspace, ready for a new decision. */
  void empty() {
    heldBy.clear();
    keepOuts.clear();
  }

  /**
   * Whether two lists hold equal things in the same order. It is asked this way, one index at a
   * time, because a list's own {@code equals} may make an iterator, and this runs every loop.
   */
  static <T> boolean sameThings(List<T> one, List<T> other) {
    if (one.size() != other.size()) {
      return false;
    }
    for (int index = 0; index < one.size(); index++) {
      if (!Objects.equals(one.get(index), other.get(index))) {
        return false;
      }
    }
    return true;
  }
}
