package com.michaelgrundvig.frc.world.record;

/**
 * How the latest run of an auto went ({@code Auto/Outcome}). Beside it are the auto's name ({@code
 * Auto/OutcomeName}), which is a team's text, and what its command threw if it failed ({@code
 * Auto/OutcomeWhy}). {@link MoveSentences#autoOutcome} makes the sentence from all of it.
 *
 * @param result how the run went
 * @param seconds how long it ran, in seconds; zero while it still runs
 */
public record AutoOutcome(Result result, double seconds) {
  /** How a run of an auto went. */
  public enum Result {
    /** It is running now. */
    RUNNING,
    /** Its command finished. */
    COMPLETED,
    /** Autonomous ended while it was still running. */
    CUT_OFF,
    /** Something else canceled it. */
    CANCELED,
    /** Its command threw, as it was built or as it ran. */
    FAILED
  }
}
