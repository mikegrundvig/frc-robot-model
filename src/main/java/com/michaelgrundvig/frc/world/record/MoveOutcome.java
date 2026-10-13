package com.michaelgrundvig.frc.world.record;

/**
 * How the latest run of a move ended, for one of the joints it moved. A move of several joints, a
 * configuration's, has one of these for each joint, and they agree on everything that is the
 * move's: {@code result}, {@code task}, {@code progress}, {@code seconds}, {@code configuration},
 * {@code reason} and {@code reasonJoint}.
 *
 * <p>Under each joint the robot logs every joint's part of the last move that ended on it, in the
 * move's order ({@code Joints/<label>/Outcome}), so one key holds the whole move. Beside it are the
 * move's name ({@code OutcomeName}), which is its command's name and so a team's text, the text of
 * a reason that has some ({@code OutcomeWhy}, see {@link MoveReason}), and what held each joint as
 * a move timed out ({@code OutcomeHeldBy}, see {@link Holder}). {@link MoveSentences#outcome} makes
 * the sentence from all of it.
 *
 * @param result how the move ended
 * @param task what the move does
 * @param progress whether the run began before it ended
 * @param seconds how long the run took, in seconds: from when it began, having waited for the robot
 *     to be enabled and for its joints to home, or, for a run that never began, from when it
 *     started to wait
 * @param joint which joint this is the part of: its index in the definition's {@code joints()}. A
 *     move that moves no joint the robot drives, which is always refused, has one part that is no
 *     joint's: {@link Identity#NONE}.
 * @param configuration the configuration the move goes to or stays at: its index in the
 *     definition's {@code configurations()}, or {@link Identity#NONE} for a move of one joint
 * @param target what kind of thing the move sent this joint to or ran it at
 * @param targetIndex which one: for {@link MoveTarget#PLACE} its index in the joint's {@code
 *     positions()}, for {@link MoveTarget#OUTPUT} its index in the joint's {@code outputs()}, and
 *     {@link Identity#NONE} for every other target
 * @param targetNumber the target as a number, or NaN where it has none: a position in the joint's
 *     units (radians, or meters) for {@link MoveTarget#PLACE} and {@link MoveTarget#NUMBER}; a
 *     speed in the joint's units a second for {@link MoveTarget#SPEED}; the output's own value for
 *     {@link MoveTarget#OUTPUT} (radians or meters a second, volts, or a share of the battery, by
 *     the output's kind); zero for {@link MoveTarget#STOPPED}. For a follow it is the last target
 *     the joint chased.
 * @param position where the joint was as the run ended, in its units: radians, or meters
 * @param velocity how fast the joint moved as the run ended, in its units a second
 * @param arrivedSeconds when this joint came inside its band for good, in seconds from when the run
 *     began, for a move that goes to its targets; NaN for a joint that was outside its band as the
 *     run ended, and for every other task
 * @param reason why it ended so, where the result alone does not say
 * @param reasonJoint the joint the reason is about, for a reason that is about one: its index in
 *     the definition's {@code joints()}, or {@link Identity#NONE}
 */
public record MoveOutcome(
    MoveResult result,
    MoveTask task,
    MoveProgress progress,
    double seconds,
    int joint,
    int configuration,
    MoveTarget target,
    int targetIndex,
    double targetNumber,
    double position,
    double velocity,
    double arrivedSeconds,
    MoveReason reason,
    int reasonJoint) {

  /** Whether the move was of a configuration's joints, and not one joint's own. */
  public boolean isOfAConfiguration() {
    return Identity.isSome(configuration);
  }

  /** Whether this joint was run at an output, where every other target positions the joint. */
  public boolean ranAtAnOutput() {
    return target == MoveTarget.OUTPUT || target == MoveTarget.STOPPED;
  }

  /** Whether this joint was inside its band as a move that goes to its targets ended. */
  public boolean cameInsideItsBand() {
    return !Double.isNaN(arrivedSeconds);
  }
}
