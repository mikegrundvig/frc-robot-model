package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * The definition's checks of its configurations, against its joints and its interlocks. Each
 * configuration is judged with the joints it names where it puts them, and every other joint where
 * it rests ({@link Placement}).
 */
final class ConfigurationChecks {
  private ConfigurationChecks() {}

  /**
   * What is wrong with the configurations. Each problem is about its configuration.
   *
   * <p>Two configurations may not share a name. A configuration may name only the robot's own
   * joints, each moving on its own, at their own places; one that names anything else is reported
   * for that and judged no further. Otherwise it may not break an interlock, and it may not leave
   * joints holding each other in a deadlock ({@link Deadlock}).
   */
  static List<Finding> problems(
      RobotMath math, List<Configuration> configurations, List<Interlock> interlocks) {
    List<Finding> problems = new ArrayList<>(namesUsedTwice(configurations));
    List<Interlock> checkable = InterlockChecks.checkable(math, interlocks);
    List<InterlockRing> rings =
        InterlockRing.among(
            interlocks.stream()
                .filter(interlock -> InterlockChecks.namesOnlyTheRobotsOwn(math, interlock))
                .toList());
    for (Configuration configuration : configurations) {
      List<Finding> notTheRobotsOwn = whatIsNotTheRobotsOwn(math, configuration);
      problems.addAll(notTheRobotsOwn);
      if (!notTheRobotsOwn.isEmpty()) {
        continue;
      }
      Placement placement = Placement.of(math, configuration);
      problems.addAll(interlocksBroken(configuration, placement, checkable));
      problems.addAll(deadlocks(configuration, placement, rings));
    }
    return problems;
  }

  /** A finding for each name that two or more configurations share. */
  private static List<Finding> namesUsedTwice(List<Configuration> configurations) {
    Set<String> seen = new HashSet<>();
    Set<String> twice = new LinkedHashSet<>();
    for (Configuration configuration : configurations) {
      boolean seenBefore = !seen.add(configuration.name());
      if (seenBefore) {
        twice.add(configuration.name());
      }
    }
    List<Finding> found = new ArrayList<>();
    for (String name : twice) {
      found.add(Finding.refused(name, "two configurations are named " + name));
    }
    return found;
  }

  /** The findings that refuse a configuration for naming what is not the robot's own. */
  private static List<Finding> whatIsNotTheRobotsOwn(RobotMath math, Configuration configuration) {
    String named = "configuration " + configuration.name();
    List<Finding> found = new ArrayList<>();
    for (Joint joint : configuration.joints()) {
      Places.whyNotMovingOnItsOwn(math, joint, named)
          .ifPresent(why -> found.add(Finding.refused(configuration.name(), why)));
    }
    for (Condition.At target : configuration.targets()) {
      Places.whyNotOwn(target)
          .ifPresent(why -> found.add(Finding.refused(configuration.name(), named + ": " + why)));
    }
    return found;
  }

  /** A finding for each interlock a configuration breaks. */
  private static List<Finding> interlocksBroken(
      Configuration configuration, Placement placement, List<Interlock> checkable) {
    List<Finding> found = new ArrayList<>();
    for (Interlock interlock : checkable) {
      InterlockChecks.howBroken(interlock, placement)
          .ifPresent(
              broken ->
                  found.add(
                      Finding.refused(
                          configuration.name(),
                          "configuration " + configuration.name() + " " + broken)));
    }
    return found;
  }

  /**
   * A finding for each group of joints a configuration leaves in a deadlock. A ring is judged here
   * only if the configuration sets one of its joints. A ring whose joints are all at rest and still
   * is the same in every configuration, and is judged once, as the robot starts ({@link
   * InterlockChecks}).
   */
  private static List<Finding> deadlocks(
      Configuration configuration, Placement placement, List<InterlockRing> rings) {
    List<Finding> found = new ArrayList<>();
    Set<Set<Joint>> reported = new HashSet<>();
    for (InterlockRing ring : rings) {
      boolean setsOneOfItsJoints = ring.joints().stream().anyMatch(placement::sets);
      if (!setsOneOfItsJoints) {
        continue;
      }
      Optional<String> deadlock = Deadlock.of(ring, placement);
      if (deadlock.isPresent() && reported.add(ring.joints())) {
        found.add(
            Finding.refused(
                configuration.name(),
                "configuration " + configuration.name() + " " + deadlock.get()));
      }
    }
    return found;
  }
}
