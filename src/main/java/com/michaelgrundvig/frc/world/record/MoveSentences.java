package com.michaelgrundvig.frc.world.record;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * The sentence that says how a move ended, made from the facts the robot logs: a move of joints
 * ({@link MoveOutcome}), a move of the drivetrain ({@link DriveOutcome}) and an auto ({@link
 * AutoOutcome}). Each sentence holds, in order, the move's name and how it ended, how long it ran
 * unless it was refused, where things were, and why. The wording is here and nowhere else.
 */
public final class MoveSentences {
  /** What a move that the robot's disabling canceled says of why. */
  public static final String ROBOT_WAS_DISABLED = "the robot was disabled";

  /** What a refusal says when the drivetrain has not been handed the driver's sticks. */
  public static final String NO_DRIVER =
      "the drivetrain has no driver's sticks: the robot hands them over as it builds the drive,"
          + " drive.takeDriver(...)";

  private MoveSentences() {}

  /**
   * How a move of the robot's joints ended: "pivot → out: ARRIVED in 0.41 s", "pivot → out:
   * TIMED_OUT after 1.62 s, pivot at 1.5°, held: pivot keeps out of 40° to 80° while elevator at
   * most clear", "pivot and elevator → stowed: ARRIVED in 0.72 s (pivot 0.41 s, elevator 0.72 s)".
   *
   * @param definition the robot, whose joints, interlocks and sensors the facts point into
   * @param name the move's name, as logged beside its outcome
   * @param parts every joint's part of the move, in the move's order: at least one
   * @param heldAsItEnded what held each of the move's joints as it ended, as logged beside its
   *     outcome; read only for a move that timed out
   * @param why the text of the move's reason, as logged beside its outcome; read only for a reason
   *     that has text ({@link MoveReason})
   */
  public static String outcome(
      RobotDefinition definition,
      String name,
      List<MoveOutcome> parts,
      List<Holder> heldAsItEnded,
      String why) {
    MoveOutcome move = parts.getFirst();
    StringBuilder text = new StringBuilder(name).append(": ").append(move.result());
    text.append(howLong(move.result(), move.seconds()));
    switch (move.result()) {
      case ARRIVED -> text.append(whenEachArrived(definition, parts));
      case TIMED_OUT -> {
        text.append(whereEachWas(definition, parts));
        text.append(whatHeldEach(definition, parts, heldAsItEnded));
      }
      case CANCELED -> text.append(whereEachWas(definition, parts));
      case REFUSED -> {}
    }
    return withReason(text, reasonSaid(definition, move, why));
  }

  /**
   * How a move of the drivetrain ended: "drive → 90.0°: ARRIVED in 0.84 s", "drive → (2.00, 1.00)
   * 90.0°: TIMED_OUT after 0.20 s, at (0.31, 0.12) facing 14.0°", "drive → Mark: REFUSED: alliance
   * unknown".
   *
   * @param name the move's name, as logged beside its outcome
   * @param outcome how it ended
   * @param why the text of its reason, as logged beside its outcome; read only for a reason that
   *     has text ({@link MoveReason})
   */
  public static String driveOutcome(String name, DriveOutcome outcome, String why) {
    StringBuilder text = new StringBuilder(name).append(": ").append(outcome.result());
    text.append(howLong(outcome.result(), outcome.seconds()));
    boolean saysWhereItGotTo =
        outcome.result() == MoveResult.TIMED_OUT && outcome.task().followsAPath();
    if (saysWhereItGotTo) {
      text.append(", ").append(whereTheRobotGotTo(outcome));
    }
    String said =
        switch (outcome.reason()) {
          case NONE -> "";
          case ROBOT_DISABLED -> ROBOT_WAS_DISABLED;
          case NO_DRIVER -> NO_DRIVER;
          default -> why;
        };
    return withReason(text, said);
  }

  /**
   * How an auto's latest run went: "Left Double: running", "Left Double: COMPLETED in 9.80 s",
   * "Left Double: CUT OFF after 15.00 s, as autonomous ended", "Left Double: FAILED after 0.00 s:
   * it couldn't be built: IllegalStateException: no path".
   *
   * @param name the auto's name, as logged beside its outcome
   * @param outcome how the run went
   * @param why what its command threw, as logged beside its outcome; empty for none
   */
  public static String autoOutcome(String name, AutoOutcome outcome, String why) {
    double seconds = outcome.seconds();
    String how =
        switch (outcome.result()) {
          case RUNNING -> "running";
          case COMPLETED -> String.format(Locale.ROOT, "COMPLETED in %.2f s", seconds);
          case CUT_OFF ->
              String.format(Locale.ROOT, "CUT OFF after %.2f s, as autonomous ended", seconds);
          case CANCELED -> String.format(Locale.ROOT, "CANCELED after %.2f s", seconds);
          case FAILED -> String.format(Locale.ROOT, "FAILED after %.2f s", seconds);
        };
    return withReason(new StringBuilder(name).append(": ").append(how), why);
  }

  /** How long a move ran, as its sentence says it after how it ended. A refusal says no time. */
  private static String howLong(MoveResult result, double seconds) {
    return switch (result) {
      case ARRIVED -> String.format(Locale.ROOT, " in %.2f s", seconds);
      case TIMED_OUT, CANCELED -> String.format(Locale.ROOT, " after %.2f s", seconds);
      case REFUSED -> "";
    };
  }

  /** A sentence with its reason after a colon, or as it is when there is no reason to give. */
  private static String withReason(StringBuilder text, String reason) {
    if (reason.isEmpty()) {
      return text.toString();
    }
    return text.append(": ").append(reason).toString();
  }

  /**
   * When each joint of a move of several got there: " (pivot 0.41 s, elevator 0.72 s)". Empty for a
   * move of one joint, whose own time says it.
   */
  private static String whenEachArrived(RobotDefinition definition, List<MoveOutcome> parts) {
    if (parts.size() <= 1) {
      return "";
    }
    List<String> each = new ArrayList<>();
    for (MoveOutcome part : parts) {
      if (part.cameInsideItsBand()) {
        String label = jointOf(definition, part).label();
        each.add(String.format(Locale.ROOT, "%s %.2f s", label, part.arrivedSeconds()));
      }
    }
    return " (" + String.join(", ", each) + ")";
  }

  /** Where each joint the move positioned was as it ended: ", pivot at 62°". */
  private static String whereEachWas(RobotDefinition definition, List<MoveOutcome> parts) {
    StringBuilder text = new StringBuilder();
    for (MoveOutcome part : parts) {
      boolean positionedAJoint =
          Identity.isSome(part.joint()) && !part.ranAtAnOutput() && part.task() != MoveTask.HOME;
      if (positionedAJoint) {
        text.append(", ").append(whereItWas(jointOf(definition, part), part));
      }
    }
    return text.toString();
  }

  /**
   * Where one joint was as the move ended: "pivot at 62°". A follow says it in the quantity its
   * joint was chasing, and what the target was: "shooter at 2990 rpm, its target 3000 rpm". A
   * follow that ended before it chased anything says it in the quantity its joint is driven by, a
   * position for a joint held at positions and a speed for any other.
   */
  private static String whereItWas(Joint joint, MoveOutcome part) {
    if (part.task() != MoveTask.FOLLOW) {
      return atItsPosition(joint, part);
    }
    return switch (part.target()) {
      case NUMBER ->
          atItsPosition(joint, part) + ", its target " + joint.shown(part.targetNumber());
      case SPEED ->
          atItsSpeed(joint, part) + ", its target " + joint.shownSpeed(part.targetNumber());
      default -> joint.holdsPositions() ? atItsPosition(joint, part) : atItsSpeed(joint, part);
    };
  }

  private static String atItsPosition(Joint joint, MoveOutcome part) {
    return joint.label() + " at " + joint.shown(part.position());
  }

  private static String atItsSpeed(Joint joint, MoveOutcome part) {
    return joint.label() + " at " + joint.shownSpeed(part.velocity());
  }

  /**
   * What held each joint as the move timed out, joint by joint in the move's order: ", held: pivot
   * keeps out of 40° to 80° while elevator at most clear".
   */
  private static String whatHeldEach(
      RobotDefinition definition, List<MoveOutcome> parts, List<Holder> heldAsItEnded) {
    StringBuilder text = new StringBuilder();
    for (MoveOutcome part : parts) {
      if (!Identity.isSome(part.joint())) {
        continue;
      }
      Joint joint = jointOf(definition, part);
      List<Holder> holdingIt = new ArrayList<>();
      for (Holder holder : heldAsItEnded) {
        if (JointSentences.heldJoint(definition, holder).equals(joint)) {
          holdingIt.add(holder);
        }
      }
      if (!holdingIt.isEmpty()) {
        text.append(", held: ").append(JointSentences.heldBy(definition, holdingIt));
      }
    }
    return text.toString();
  }

  /** Why a move ended as it did, in words, or empty when its result says it all. */
  private static String reasonSaid(RobotDefinition definition, MoveOutcome move, String why) {
    return switch (move.reason()) {
      case NONE -> "";
      case ROBOT_DISABLED -> ROBOT_WAS_DISABLED;
      case NO_DRIVER -> NO_DRIVER;
      case POSITION_LOST ->
          "can't go on: "
              + possessive(reasonJoint(definition, move).label())
              + " position stopped being a number: check its encoder and its motor controller's"
              + " settings";
      case JOINT_NOT_ANSWERING -> cannotGoOn(move) + notAnswering(reasonJoint(definition, move));
      case JOINT_FAULTED -> cannotGoOn(move) + why;
      case GATE_FALSE,
          GATE_FELL,
          CONDITION_NEVER_HELD,
          REFUSED_AS_BUILT,
          TARGET_UNREACHABLE,
          TARGET_WRONG_KIND,
          PATH_NOT_PLANNED ->
          why;
    };
  }

  /**
   * What is said of a joint whose motor controllers are silent, which refuses every move on it:
   * "pivot isn't answering: its motor controllers are silent".
   */
  public static String notAnswering(Joint joint) {
    return joint.label() + " isn't answering: its motor controllers are silent";
  }

  /**
   * What leads the reason of a move that a joint stopped part way: "can't go on: ". A move that was
   * refused as it started says only why, and so does a homing, which a faulted joint ends.
   */
  private static String cannotGoOn(MoveOutcome move) {
    boolean stoppedPartWay = move.progress() == MoveProgress.BEGUN && move.task() != MoveTask.HOME;
    return stoppedPartWay ? "can't go on: " : "";
  }

  private static Joint jointOf(RobotDefinition definition, MoveOutcome part) {
    return definition.joints().get(part.joint());
  }

  private static Joint reasonJoint(RobotDefinition definition, MoveOutcome move) {
    return definition.joints().get(move.reasonJoint());
  }

  /** Where the robot got to, as a move along a path that timed out says it. */
  private static String whereTheRobotGotTo(DriveOutcome outcome) {
    double degrees = Math.toDegrees(outcome.headingRadians());
    if (outcome.task() == DriveMoveTask.TURN_TO) {
      return String.format(Locale.ROOT, "heading at %.1f°", degrees);
    }
    return String.format(
        Locale.ROOT, "at (%.2f, %.2f) facing %.1f°", outcome.x(), outcome.y(), degrees);
  }
}
