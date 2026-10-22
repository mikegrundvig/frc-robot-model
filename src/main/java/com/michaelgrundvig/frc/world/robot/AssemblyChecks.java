package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.physical.Shape;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Mass;

/**
 * The definition's checks of what a robot's parts make once they are put together: that the links
 * leave some of the robot's mass for its chassis, that each joint's absolute encoders can say where
 * it is, that each joint's named speeds are ones its motors can reach, and that a camera on a joint
 * sits on that joint's link.
 *
 * <p>Each problem is reported against what it is about: a joint, a camera, or the robot as a whole.
 */
final class AssemblyChecks {
  private AssemblyChecks() {}

  /**
   * Everything wrong with the parts taken together. The list is empty when they make one robot.
   *
   * <p>If the parts do not even refer to each other soundly ({@link StructureChecks}), only that is
   * reported, because nothing further can be worked out.
   *
   * @param robot the robot's name, which a problem about the whole robot is reported against
   */
  static List<Finding> problems(
      String robot,
      Body body,
      List<Joint> joints,
      List<Coupling> couplings,
      List<Drive> drives,
      List<Sensor> sensors,
      List<CameraConfig> cameras) {
    List<String> structure = StructureChecks.problems(joints, couplings, drives, sensors, cameras);
    if (!structure.isEmpty()) {
      return structure.stream().map(problem -> Finding.refused(robot, problem)).toList();
    }
    RobotMath math = RobotMath.ofSoundParts(body, joints, couplings, drives, sensors);
    List<Finding> problems = new ArrayList<>();
    // A weight given is checked here; until the robot is weighed, whoever takes a mass for it
    // checks
    // at that mass ({@code RobotDefinition.massProblems}).
    body.weight()
        .flatMap(weight -> whyMassDoesNotAddUp(math, body, weight))
        .ifPresent(why -> problems.add(Finding.refused(robot, why)));
    for (Joint joint : joints) {
      EncoderChecks.whyNotReadAllAlong(joint, sensors)
          .ifPresent(why -> problems.add(Finding.refused(joint.label(), why)));
    }
    JointSpeedChecks.problems(math, problems);
    for (CameraConfig camera : cameras) {
      if (camera.joint().isEmpty()) {
        continue;
      }
      Joint ridesOn = camera.joint().get();
      whyNotOnItsLink(camera, ridesOn)
          .ifPresent(why -> problems.add(Finding.refused(camera.name(), why)));
      whyOutsideWhenStowed(math, body, camera)
          .ifPresent(why -> problems.add(Finding.refused(camera.name(), why)));
    }
    return problems;
  }

  /**
   * Why the masses given do not make a robot of a mass, if they do not. The links must weigh less
   * than the whole robot, and what is left once they are taken away, the chassis, must be something
   * that could exist. The chassis is only looked at when the links fit, and a robot with no joints
   * has no links to take away.
   *
   * @param robot the whole robot's mass: as weighed, or the one taken in its place until it is
   */
  static Optional<String> whyMassDoesNotAddUp(RobotMath math, Body body, Mass robot) {
    Optional<String> overBudget = whyLinksWeighTooMuch(math, body, robot);
    if (overBudget.isPresent() || math.joints().isEmpty()) {
      return overBudget;
    }
    return whyChassisIsNotABody(math, robot);
  }

  /**
   * Why the links weigh too much for the robot they are on, if they do. The links are part of what
   * the robot weighs, and they must leave some of it for the chassis, which carries the drivetrain,
   * the electronics, the bumpers and the battery.
   */
  private static Optional<String> whyLinksWeighTooMuch(RobotMath math, Body body, Mass mass) {
    double links = math.linksKilograms();
    double robot = mass.in(Kilograms);
    if (links < robot) {
      return Optional.empty();
    }
    String comparedWithTheRobot = links > robot ? "more than" : "as much as";
    String howTheRobotsMassIsKnown =
        body.weight().isPresent() ? " as weighed" : ", the season's most";
    return Optional.of(
        String.format(
            Locale.ROOT,
            "its links weigh %.2f kg together, %s the robot's %.2f kg%s: nothing is left for the"
                + " chassis",
            links,
            comparedWithTheRobot,
            robot,
            howTheRobotsMassIsKnown));
  }

  /**
   * Why the chassis is not a body that could exist, if it is not. The chassis is the robot as
   * weighed, less every link where its joint rests. Taking away links that are too heavy, or too
   * far out, for the body they were measured as part of leaves something with no mass or an inertia
   * no solid has.
   */
  private static Optional<String> whyChassisIsNotABody(RobotMath math, Mass robot) {
    String whyNot;
    try {
      whyNot = math.chassis(robot).inertia().whyNotSolid();
    } catch (IllegalArgumentException e) {
      whyNot = String.valueOf(e.getMessage()).replace('\n', ' ');
    }
    if (whyNot == null) {
      return Optional.empty();
    }
    return Optional.of("the chassis, the body less its links at rest, isn't a body: " + whyNot);
  }

  /**
   * Why a camera is not on the link of the joint it rides on, if it is not. Its mount must be
   * within {@link RobotMath#CAMERA_ON_LINK_METERS} of one of the link's shapes. A link with no
   * shapes cannot be checked.
   */
  private static Optional<String> whyNotOnItsLink(CameraConfig camera, Joint ridesOn) {
    List<Shape> shapes = ridesOn.link().shapes();
    if (shapes.isEmpty()) {
      return Optional.empty();
    }
    Translation3d mount = camera.position();
    double offTheLink = Double.POSITIVE_INFINITY;
    for (Shape shape : shapes) {
      offTheLink = Math.min(offTheLink, shape.distance(mount));
    }
    if (offTheLink > RobotMath.CAMERA_ON_LINK_METERS) {
      return Optional.of(
          String.format(
              Locale.ROOT,
              "camera %s is %.2f m off %s, the link it rides on",
              camera.name(),
              offTheLink,
              ridesOn.link().label()));
    }
    return Optional.empty();
  }

  /** Why a camera on a joint is outside the robot with every joint where it rests, if it is. */
  private static Optional<String> whyOutsideWhenStowed(
      RobotMath math, Body body, CameraConfig camera) {
    Translation3d stowed = math.cameraPose(camera, Map.of()).getTranslation();
    if (body.contains(stowed)) {
      return Optional.empty();
    }
    return Optional.of(
        "camera " + camera.name() + " is outside the robot with its joint stowed, at " + stowed);
  }
}
