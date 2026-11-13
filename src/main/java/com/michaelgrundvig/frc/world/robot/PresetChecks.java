package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * The definition's checks of its presets, against its joints and its interlocks. Each preset is
 * judged with the joints it names where it puts them, and every other joint where it rests ({@link
 * TrialPlacement}).
 */
final class PresetChecks {
  private PresetChecks() {}

  /**
   * What is wrong with the presets. Each problem is about its preset.
   *
   * <p>Two presets may not share a name. A preset may name only the robot's own joints, each moving
   * on its own, at their own places; one that names anything else is reported for that and judged
   * no further. Otherwise it may not break an interlock, and it may not leave joints holding each
   * other in a deadlock ({@link Deadlock}).
   */
  static List<Finding> problems(RobotMath math, List<Preset> presets, List<Interlock> interlocks) {
    List<Finding> problems = new ArrayList<>(namesUsedTwice(presets));
    List<Interlock> checkable = InterlockChecks.checkable(math, interlocks);
    List<InterlockRing> rings =
        InterlockRing.among(
            interlocks.stream()
                .filter(interlock -> InterlockChecks.namesOnlyTheRobotsOwn(math, interlock))
                .toList());
    for (Preset preset : presets) {
      List<Finding> notTheRobotsOwn = whatIsNotTheRobotsOwn(math, preset);
      problems.addAll(notTheRobotsOwn);
      if (!notTheRobotsOwn.isEmpty()) {
        continue;
      }
      TrialPlacement placement = TrialPlacement.of(math, preset);
      problems.addAll(interlocksBroken(preset, placement, checkable));
      problems.addAll(deadlocks(preset, placement, rings));
    }
    return problems;
  }

  /** A finding for each name that two or more presets share. */
  private static List<Finding> namesUsedTwice(List<Preset> presets) {
    Set<String> seen = new HashSet<>();
    Set<String> twice = new LinkedHashSet<>();
    for (Preset preset : presets) {
      boolean seenBefore = !seen.add(preset.name());
      if (seenBefore) {
        twice.add(preset.name());
      }
    }
    List<Finding> found = new ArrayList<>();
    for (String name : twice) {
      found.add(Finding.refused(name, "two presets are named " + name));
    }
    return found;
  }

  /** The findings that refuse a preset for naming what is not the robot's own. */
  private static List<Finding> whatIsNotTheRobotsOwn(RobotMath math, Preset preset) {
    String named = "preset " + preset.name();
    List<Finding> found = new ArrayList<>();
    for (Joint joint : preset.joints()) {
      Places.whyNotMovingOnItsOwn(math, joint, named)
          .ifPresent(why -> found.add(Finding.refused(preset.name(), why)));
    }
    for (Condition.At target : preset.targets()) {
      Places.whyNotOwn(target)
          .ifPresent(why -> found.add(Finding.refused(preset.name(), named + ": " + why)));
    }
    return found;
  }

  /** A finding for each interlock a preset breaks. */
  private static List<Finding> interlocksBroken(
      Preset preset, TrialPlacement placement, List<Interlock> checkable) {
    List<Finding> found = new ArrayList<>();
    for (Interlock interlock : checkable) {
      InterlockChecks.howBroken(interlock, placement)
          .ifPresent(broken ->
              found.add(Finding.refused(preset.name(), "preset " + preset.name() + " " + broken)));
    }
    return found;
  }

  /**
   * A finding for each group of joints a preset leaves in a deadlock. A ring is judged here only if
   * the preset sets one of its joints. A ring whose joints are all at rest and still is the same in
   * every preset, and is judged once, as the robot starts ({@link InterlockChecks}).
   */
  private static List<Finding> deadlocks(
      Preset preset, TrialPlacement placement, List<InterlockRing> rings) {
    List<Finding> found = new ArrayList<>();
    Set<Set<Joint>> reported = new HashSet<>();
    for (InterlockRing ring : rings) {
      boolean setsOneOfItsJoints = ring.joints().stream().anyMatch(placement::sets);
      if (!setsOneOfItsJoints) {
        continue;
      }
      Optional<String> deadlock = Deadlock.of(ring, placement);
      if (deadlock.isPresent() && reported.add(ring.joints())) {
        found.add(Finding.refused(preset.name(), "preset " + preset.name() + " " + deadlock.get()));
      }
    }
    return found;
  }
}
