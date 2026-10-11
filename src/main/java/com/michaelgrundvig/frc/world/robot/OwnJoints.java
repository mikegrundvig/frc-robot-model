package com.michaelgrundvig.frc.world.robot;

import java.util.List;
import java.util.Optional;

/**
 * Re-reads what was written about one robot's joints as being about another robot's own.
 *
 * <p>A practice robot is the competition robot with some joints changed: re-wired, re-tuned, or
 * with their stops and named places somewhere else. A joint changed in those ways is still equal to
 * the joint it was made from, so a configuration or an interlock written with the competition
 * robot's constants ({@code PIVOT.atLeast(PIVOT_OUT)}) can be matched to the practice robot's
 * joint. Matching it matters because the two joints may put the same named place at different
 * positions, and each robot has to be checked, and driven, at its own.
 *
 * <p>A joint the robot does not have is left as it was written. The definition's checks then find
 * it and refuse it, which gives a better message than failing here would.
 */
final class OwnJoints {
  private OwnJoints() {}

  /**
   * The robot's own joint that is equal to a joint, or the joint itself if the robot has no equal
   * of it.
   */
  static Joint own(List<Joint> joints, Joint joint) {
    int at = joints.indexOf(joint);
    if (at < 0) {
      return joint;
    }
    return joints.get(at);
  }

  /** A configuration with each of its targets on the robot's own joint. */
  static Configuration onOwn(List<Joint> joints, Configuration configuration) {
    return ConfigurationBuilder.builder(configuration)
        .targets(configuration.targets().stream().map(target -> onOwn(joints, target)).toList())
        .build();
  }

  /** An interlock restricting the robot's own joint, on a condition about the robot's own. */
  static Interlock onOwn(List<Joint> joints, Interlock interlock) {
    return switch (interlock) {
      case Interlock.RunsOnlyWhen runsOnlyWhen ->
          InterlockRunsOnlyWhenBuilder.builder(runsOnlyWhen)
              .joint(own(joints, runsOnlyWhen.joint()))
              .condition(onOwn(joints, runsOnlyWhen.condition()))
              .build();
      case Interlock.KeepsOut keepsOut ->
          InterlockKeepsOutBuilder.builder(keepsOut)
              .joint(own(joints, keepsOut.joint()))
              .condition(onOwn(joints, keepsOut.condition()))
              .build();
    };
  }

  /**
   * A condition about the robot's own joint, with any place it names put where that joint has it.
   * So {@code ELEVATOR.atMost(ELEVATOR_CLEAR)}, on a practice robot whose elevator clears lower,
   * becomes "at most" the practice robot's own clear. A condition on a sensor is about no joint and
   * is returned as it is.
   */
  static Condition onOwn(List<Joint> joints, Condition condition) {
    return switch (condition) {
      case Condition.At at -> onOwn(joints, at);
      case Condition.AtLeast atLeast -> {
        Joint joint = own(joints, atLeast.joint());
        yield ConditionAtLeastBuilder.builder(atLeast)
            .joint(joint)
            .position(whereItPuts(joint, atLeast.place(), atLeast.position()))
            .build();
      }
      case Condition.AtMost atMost -> {
        Joint joint = own(joints, atMost.joint());
        yield ConditionAtMostBuilder.builder(atMost)
            .joint(joint)
            .position(whereItPuts(joint, atMost.place(), atMost.position()))
            .build();
      }
      case Condition.Between between -> {
        Joint joint = own(joints, between.joint());
        yield ConditionBetweenBuilder.builder(between)
            .joint(joint)
            .from(whereItPuts(joint, between.fromPlace(), between.from()))
            .to(whereItPuts(joint, between.toPlace(), between.to()))
            .build();
      }
      case Condition.UpToSpeed upToSpeed ->
          ConditionUpToSpeedBuilder.builder(upToSpeed)
              .joint(own(joints, upToSpeed.joint()))
              .build();
      case Condition.OnSensor onSensor -> onSensor;
    };
  }

  /**
   * A joint at a named place, on the robot's own joint. The place is kept by name, and the robot's
   * joint says where it is when it is asked.
   */
  private static Condition.At onOwn(List<Joint> joints, Condition.At at) {
    return ConditionAtBuilder.builder(at).joint(own(joints, at.joint())).build();
  }

  /**
   * The position an edge of a condition is at on a joint. An edge written as a named place is where
   * this joint puts that place. An edge written as a number is that number. An edge that names a
   * place the joint does not have keeps the position it was written with, and the definition's
   * checks refuse it.
   *
   * @param place the place the edge was written as, if it was written as one
   * @param position the position the edge was written with
   */
  private static double whereItPuts(Joint joint, Optional<Position> place, double position) {
    if (place.isEmpty()) {
      return position;
    }
    boolean isTheJointsOwn = joint.whyNotOwn(place.get()).isEmpty();
    if (!isTheJointsOwn) {
      return position;
    }
    return joint.own(place.get()).value();
  }
}
