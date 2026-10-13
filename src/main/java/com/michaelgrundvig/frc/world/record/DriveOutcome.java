package com.michaelgrundvig.frc.world.record;

/**
 * How the latest run of a move of the drivetrain ended ({@code Drive/Outcome}). Beside it are the
 * move's name ({@code Drive/OutcomeName}), which is its command's name and may hold a team's own
 * names for a place, a keyframe or a gate, and the text of a reason that has some ({@code
 * Drive/OutcomeWhy}, see {@link MoveReason}). {@link MoveSentences#driveOutcome} makes the sentence
 * from all of it.
 *
 * @param result how the move ended
 * @param task what the move does
 * @param seconds how long the run took, in seconds, from when the robot was enabled
 * @param x where the robot was as the run ended, by its own estimate: meters along the field
 * @param y where the robot was as the run ended: meters across the field
 * @param headingRadians which way the robot faced as the run ended, in radians, counterclockwise
 *     from the field's x axis
 * @param reason why it ended so, where the result alone does not say
 */
public record DriveOutcome(
    MoveResult result,
    DriveMoveTask task,
    double seconds,
    double x,
    double y,
    double headingRadians,
    MoveReason reason) {}
