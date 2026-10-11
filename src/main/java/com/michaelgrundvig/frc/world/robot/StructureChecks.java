package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Whether a robot's parts refer to each other in a way that makes one robot: every label used once,
 * every joint that a part names one of the robot's joints, and the couplings sound. Nothing else
 * can be worked out about a robot until this holds, so it is checked first and by itself.
 */
final class StructureChecks {
  private StructureChecks() {}

  /**
   * How far a follower's resting position may be from where its coupling puts it, in the follower's
   * units, before the two are taken to disagree.
   */
  private static final double REST_TOLERANCE = 1e-6;

  /**
   * Everything wrong with how the parts refer to each other, each as a sentence. The list is empty
   * when nothing is wrong.
   *
   * <p>The couplings are judged only once every label is unique and every joint is known, because a
   * coupling between joints that are not the robot's cannot be followed.
   */
  static List<String> problems(
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors,
      List<CameraConfig> cameras) {
    List<String> problems = new ArrayList<>();
    addLabelsUsedTwice(joints, drives, sensors, problems);
    addJointsNotListed(joints, couplings, drives, sensors, cameras, problems);
    if (!problems.isEmpty()) {
      return problems;
    }
    addFollowersOfTwo(couplings, problems);
    addFollowersOfThemselves(couplings, problems);
    addRestsOffTheirCouplings(couplings, problems);
    return problems;
  }

  private static void addLabelsUsedTwice(
      List<Joint> joints, List<Drive> drives, List<Sensor> sensors, List<String> problems) {
    addUsedTwice("joints", joints.stream().map(Joint::label).toList(), problems);
    addUsedTwice("links", joints.stream().map(joint -> joint.link().label()).toList(), problems);
    addUsedTwice("drives", drives.stream().map(Drive::label).toList(), problems);
    addUsedTwice("sensors", sensors.stream().map(Sensor::label).toList(), problems);
  }

  /**
   * @param what what the labels are of, in the plural, as the message says it: "joints"
   */
  private static void addUsedTwice(String what, List<String> labels, List<String> problems) {
    Set<String> seen = new HashSet<>();
    for (String label : labels) {
      boolean alreadyUsed = !seen.add(label);
      if (alreadyUsed) {
        problems.add("two " + what + " are labeled " + label);
      }
    }
  }

  /** Every joint that a part refers to must be one the robot lists. */
  private static void addJointsNotListed(
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors,
      List<CameraConfig> cameras,
      List<String> problems) {
    Set<Joint> listed = new HashSet<>(joints);
    for (Joint joint : joints) {
      String parent = "joint " + possessive(joint.label()) + " parent";
      addIfNotListed(joint.parent(), parent, listed, problems);
    }
    for (Coupling each : couplings) {
      addIfNotListed(each.leader(), "a coupling's leader", listed, problems);
      addIfNotListed(each.follower(), "a coupling's follower", listed, problems);
    }
    for (Drive drive : drives) {
      String drivesJoint = "drive " + possessive(drive.label()) + " joint";
      for (Joint joint : drive.transmission().joints()) {
        addIfNotListed(joint, drivesJoint, listed, problems);
      }
    }
    for (Sensor sensor : sensors) {
      String sensorsJoint = "sensor " + possessive(sensor.label()) + " joint";
      for (Part part : partsNamedBy(sensor)) {
        addIfNotListed(part, sensorsJoint, listed, problems);
      }
    }
    for (CameraConfig camera : cameras) {
      String camerasJoint = "camera " + possessive(camera.name()) + " joint";
      camera.joint().ifPresent(joint -> addIfNotListed(joint, camerasJoint, listed, problems));
    }
  }

  /** The parts a sensor refers to: the joints it reads, and the part it is mounted on. */
  private static List<? extends Part> partsNamedBy(Sensor sensor) {
    return switch (sensor) {
      case Sensor.Encoder encoder -> encoder.joints();
      case Sensor.Switch onSwitch -> partsNamedBy(onSwitch);
      case Sensor.Rangefinder rangefinder -> List.of(rangefinder.part());
    };
  }

  /** A limit switch also names the joint whose travel it is at an end of. */
  private static List<Part> partsNamedBy(Sensor.Switch onSwitch) {
    if (onSwitch.kind() instanceof Sensor.Switch.Limit limit) {
      return List.of(onSwitch.part(), limit.joint());
    }
    return List.of(onSwitch.part());
  }

  /**
   * Adds a problem if a part is a joint the robot does not list. The chassis is always the robot's.
   *
   * @param what what refers to the part, as the message begins: "drive arm motor's joint"
   */
  private static void addIfNotListed(
      Part part, String what, Set<Joint> listed, List<String> problems) {
    if (part instanceof Joint joint && !listed.contains(joint)) {
      problems.add(what + ", " + joint.label() + ", isn't one of the robot's joints: list it");
    }
  }

  /** A joint may follow one other joint at most. */
  private static void addFollowersOfTwo(List<Coupling> couplings, List<String> problems) {
    Set<Joint> followers = new HashSet<>();
    for (Coupling each : couplings) {
      boolean alreadyFollows = !followers.add(each.follower());
      if (alreadyFollows) {
        problems.add(each.follower().label() + " follows two joints: it may follow one");
      }
    }
  }

  /**
   * No joint may follow itself, however many couplings round. Each coupling's chain of leaders is
   * walked up until it ends or comes back to the coupling's own follower.
   */
  private static void addFollowersOfThemselves(List<Coupling> couplings, List<String> problems) {
    // Where a joint follows two others, the chain goes by the coupling listed last.
    Map<Joint, Coupling> followsThrough = new HashMap<>();
    for (Coupling each : couplings) {
      followsThrough.put(each.follower(), each);
    }
    for (Coupling each : couplings) {
      if (leadsBackTo(each.follower(), each.leader(), followsThrough, couplings.size())) {
        problems.add(each.follower().label() + " follows itself, through its couplings");
      }
    }
  }

  /**
   * Whether the chain of leaders above a joint reaches a given follower.
   *
   * @param mostSteps the number of couplings. A chain cannot be longer without going round a loop
   *     that does not include the follower, and stopping there keeps such a loop from running for
   *     ever.
   */
  private static boolean leadsBackTo(
      Joint follower, Joint leader, Map<Joint, Coupling> followsThrough, int mostSteps) {
    Joint above = leader;
    for (int steps = 0; steps <= mostSteps; steps++) {
      if (above.equals(follower)) {
        return true;
      }
      Coupling next = followsThrough.get(above);
      if (next == null) {
        return false;
      }
      above = next.leader();
    }
    return false;
  }

  /**
   * A follower must rest where its coupling puts it with its leader at rest. Otherwise the robot
   * would be described in two positions at once.
   */
  private static void addRestsOffTheirCouplings(List<Coupling> couplings, List<String> problems) {
    for (Coupling each : couplings) {
      double whereCouplingPutsIt = each.followerAt(each.leader().restPosition());
      double off = Math.abs(whereCouplingPutsIt - each.follower().restPosition());
      if (off > REST_TOLERANCE) {
        problems.add(
            String.format(
                Locale.ROOT,
                "%s's rest (%.4g) isn't where its coupling puts it with %s at its own (%.4g)",
                each.follower().label(),
                each.follower().restPosition(),
                each.leader().label(),
                whereCouplingPutsIt));
      }
    }
  }
}
