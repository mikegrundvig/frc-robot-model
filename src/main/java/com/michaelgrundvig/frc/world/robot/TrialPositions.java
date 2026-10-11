package com.michaelgrundvig.frc.world.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.SortedSet;
import java.util.TreeSet;

/**
 * The positions of a joint at which the definition tries conditions on it, to learn whether they
 * hold anywhere in a stretch of the joint's travel.
 *
 * <p>A condition on a joint's position holds or does not hold over whole stretches, and changes
 * only at its edges. So trying each edge, a step either side of it, the ends of the stretch and a
 * point between each neighboring pair covers every different answer there is, without solving the
 * conditions against each other.
 */
final class TrialPositions {
  private TrialPositions() {}

  /** How far either side of a condition's edge a position is tried, in the joint's units. */
  static final double STEP = 1e-6;

  /** How far beyond the outermost point a stretch with no end is tried, in the joint's units. */
  private static final double BEYOND = 1;

  /**
   * The positions to try several conditions at, across the whole of a joint's range. A joint that
   * turns freely has no range, and is tried from below the lowest edge to above the highest.
   */
  static List<Double> acrossRange(Joint joint, List<Condition.OnJoint> conditions) {
    SortedSet<Double> edges = new TreeSet<>();
    for (Condition.OnJoint condition : conditions) {
      edges.addAll(edgesOf(condition));
    }
    SortedSet<Double> tried = new TreeSet<>();
    if (joint.range().isPresent()) {
      tried.add(joint.range().get().lower());
      tried.add(joint.range().get().upper());
    } else {
      tried.add(edges.first() - BEYOND);
      tried.add(edges.last() + BEYOND);
    }
    for (double edge : edges) {
      tried.add(edge - STEP);
      tried.add(edge);
      tried.add(edge + STEP);
    }
    tried.addAll(midpoints(edges));
    if (joint.range().isEmpty()) {
      return List.copyOf(tried);
    }
    Range range = joint.range().get();
    return tried.stream().filter(range::contains).toList();
  }

  /**
   * The positions to try one condition at, within a stretch of a joint's travel. One end of the
   * stretch may be infinite, for a joint with no stop that way, and the stretch is then tried a
   * little beyond its outermost point.
   *
   * @param from the low end of the stretch, or negative infinity
   * @param to the high end of the stretch, or positive infinity
   */
  static SortedSet<Double> within(double from, double to, Condition.OnJoint condition) {
    SortedSet<Double> tried = new TreeSet<>();
    for (double edge : edgesOf(condition)) {
      for (double near : new double[] {edge - STEP, edge, edge + STEP}) {
        boolean inTheStretch = near >= from && near <= to;
        if (inTheStretch) {
          tried.add(near);
        }
      }
    }
    if (Double.isFinite(from)) {
      tried.add(from);
    }
    if (Double.isFinite(to)) {
      tried.add(to);
    }
    if (!Double.isFinite(from)) {
      tried.add(tried.first() - BEYOND);
    }
    if (!Double.isFinite(to)) {
      tried.add(tried.last() + BEYOND);
    }
    tried.addAll(midpoints(tried));
    return tried;
  }

  /** The positions where a condition begins or ends holding. */
  static List<Double> edgesOf(Condition.OnJoint condition) {
    return switch (condition) {
      case Condition.At at -> List.of(at.joint().position(at.name()).value());
      case Condition.AtLeast atLeast -> List.of(atLeast.position());
      case Condition.AtMost atMost -> List.of(atMost.position());
      case Condition.Between between -> List.of(between.from(), between.to());
    };
  }

  /** The point halfway between each position and the next. */
  private static List<Double> midpoints(SortedSet<Double> positions) {
    List<Double> inOrder = List.copyOf(positions);
    List<Double> midpoints = new ArrayList<>();
    for (int next = 1; next < inOrder.size(); next++) {
      midpoints.add((inOrder.get(next - 1) + inOrder.get(next)) / 2);
    }
    return midpoints;
  }
}
