package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Whether the joints and places that a condition, a configuration or an interlock names are the
 * robot's own, and how the model refuses one that is not.
 *
 * <p>The definition's other checks of its named places each have a type of their own: {@link
 * JointSpeedChecks} holds each joint's named speeds to what its motors can do, {@link
 * ConfigurationChecks} judges each configuration, and {@link InterlockChecks} judges the
 * interlocks.
 */
final class Places {
  private Places() {}

  /**
   * Refuses something a team wrote, a condition or an interlock, when there is any reason to. The
   * reasons are given in plain words and without the path of a record, because the team wrote a
   * call such as {@code PIVOT.atLeast(...)} and never saw a record.
   *
   * @param reasons each reason it cannot be, where there is one
   * @throws IllegalArgumentException with every reason there is, one to a line
   */
  @SafeVarargs
  static void refuse(Optional<String>... reasons) {
    List<String> found = new ArrayList<>();
    for (Optional<String> reason : reasons) {
      reason.ifPresent(found::add);
    }
    if (!found.isEmpty()) {
      throw new IllegalArgumentException(String.join("\n", found));
    }
  }

  /** A finding that refuses the robot, about one of its joints. */
  static Finding refused(Joint joint, String message) {
    return Finding.refused(joint.label(), message);
  }

  /**
   * Why a place that a condition names is not its joint's own, if one is not: it is another joint's
   * place, or another robot's ({@link Joint#whyNotOwn(Position)}). A condition on a speed or a
   * sensor names no place.
   */
  static Optional<String> whyNotOwn(Condition condition) {
    return switch (condition) {
      case Condition.At at -> whyPlaceIsNotOwn(at);
      case Condition.AtLeast atLeast -> whyPlaceIsNotOwn(atLeast.joint(), atLeast.place());
      case Condition.AtMost atMost -> whyPlaceIsNotOwn(atMost.joint(), atMost.place());
      case Condition.Between between ->
          whyPlaceIsNotOwn(between.joint(), between.fromPlace())
              .or(() -> whyPlaceIsNotOwn(between.joint(), between.toPlace()));
      case Condition.UpToSpeed upToSpeed -> Optional.empty();
      case Condition.OnSensor onSensor -> Optional.empty();
    };
  }

  private static Optional<String> whyPlaceIsNotOwn(Condition.At at) {
    if (at.position().isPresent()) {
      return Optional.ofNullable(at.joint().whyNotOwn(at.position().get()));
    }
    return Optional.ofNullable(at.joint().whyNotOwn(at.output().orElseThrow()));
  }

  /** An edge written as a number names no place, so there is nothing that could be another's. */
  private static Optional<String> whyPlaceIsNotOwn(Joint joint, Optional<Position> place) {
    if (place.isEmpty()) {
      return Optional.empty();
    }
    return Optional.ofNullable(joint.whyNotOwn(place.get()));
  }

  /**
   * Whether a joint can be named by a configuration or an interlock: it is one of the robot's
   * joints, and it moves on its own rather than following another.
   */
  static boolean movesOnItsOwn(RobotMath math, Joint joint) {
    return math.has(joint) && math.leaderOf(joint) == null;
  }

  /**
   * Why a joint cannot be named by a configuration or an interlock, if it cannot ({@link
   * #movesOnItsOwn}).
   *
   * @param what what names it, as the message begins: {@code configuration amp}
   */
  static Optional<String> whyNotMovingOnItsOwn(RobotMath math, Joint joint, String what) {
    if (!math.has(joint)) {
      return Optional.of(
          possessive(what)
              + " joint, "
              + joint.label()
              + ", isn't one of the robot's joints: list it");
    }
    Joint leader = math.leaderOf(joint);
    if (leader == null) {
      return Optional.empty();
    }
    return Optional.of(
        what
            + " names "
            + joint.label()
            + ", which follows "
            + leader.label()
            + ": name "
            + leader.label()
            + " instead");
  }
}
