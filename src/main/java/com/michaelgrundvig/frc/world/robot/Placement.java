package com.michaelgrundvig.frc.world.robot;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.Set;

/**
 * Where the robot's joints are put for a check, and which of them run: either as the robot starts,
 * or as one configuration puts them. The definition judges its interlocks against a placement,
 * because that is all it can know before the robot runs.
 *
 * @param positions where each joint is, in its own units. A joint that is missing is where it
 *     rests.
 * @param named the joints whose positions a configuration gives. Every other joint is where it
 *     rests, and a message about it says so.
 * @param running the joints a configuration runs at an output, each with the speed it runs at. The
 *     speed is empty for a voltage or a duty cycle, because how fast those turn the joint is not
 *     known until the robot runs. A joint that is missing is still.
 */
record Placement(
    Map<Joint, Double> positions, Set<Joint> named, Map<Joint, OptionalDouble> running) {
  /** The robot as it starts: every joint where it rests, and none running. */
  static Placement asTheRobotStarts(RobotMath math) {
    return new Placement(math.positions(Map.of()), Set.of(), Map.of());
  }

  /**
   * The robot as a configuration puts it: the joints it names at their named positions or running
   * at their named outputs, and every other joint where it rests.
   */
  static Placement of(RobotMath math, Configuration configuration) {
    Map<Joint, Double> given = new LinkedHashMap<>();
    Map<Joint, OptionalDouble> running = new LinkedHashMap<>();
    for (Condition.At target : configuration.targets()) {
      Joint joint = target.joint();
      if (!target.isOutput()) {
        given.put(joint, joint.position(target.name()).value());
        continue;
      }
      Output output = joint.output(target.name());
      if (output.runs()) {
        running.put(joint, speedOf(output));
      }
    }
    return new Placement(math.positions(given), given.keySet(), running);
  }

  /** The speed an output runs its joint at, when the output is a speed. */
  private static OptionalDouble speedOf(Output output) {
    if (output.kind() == Output.Kind.VELOCITY) {
      return OptionalDouble.of(output.value());
    }
    return OptionalDouble.empty();
  }

  /** Where a joint is, in its own units. */
  double positionOf(Joint joint) {
    return positions.getOrDefault(joint, joint.restPosition());
  }

  /** Whether a configuration runs a joint at an output. */
  boolean runs(Joint joint) {
    return running.containsKey(joint);
  }

  /** Whether a configuration puts a joint anywhere or runs it, so that it is not simply at rest. */
  boolean sets(Joint joint) {
    return named.contains(joint) || runs(joint);
  }

  /**
   * Where a joint is, as a message says it: "pivot is at 130°", with ", where it rests" added for a
   * joint no configuration put there.
   */
  String where(Joint joint) {
    String where = joint.label() + " is at " + joint.shown(positionOf(joint));
    if (named.contains(joint)) {
      return where;
    }
    return where + ", where it rests";
  }

  /**
   * What a condition reads with the robot placed like this. A sensor's reading is unknown before
   * the robot runs, and so is the speed of a joint run at a voltage or a duty cycle.
   */
  ConditionReading reading(Condition condition) {
    return switch (condition) {
      case Condition.OnJoint onJoint ->
          ConditionReading.of(onJoint.holdsAt(positionOf(onJoint.joint())));
      case Condition.UpToSpeed upToSpeed -> reading(upToSpeed);
      case Condition.OnSensor onSensor -> ConditionReading.UNKNOWN;
    };
  }

  private ConditionReading reading(Condition.UpToSpeed upToSpeed) {
    OptionalDouble still = OptionalDouble.of(0);
    OptionalDouble speed = running.getOrDefault(upToSpeed.joint(), still);
    if (speed.isEmpty()) {
      return ConditionReading.UNKNOWN;
    }
    return ConditionReading.of(upToSpeed.holdsAt(speed.getAsDouble()));
  }
}
