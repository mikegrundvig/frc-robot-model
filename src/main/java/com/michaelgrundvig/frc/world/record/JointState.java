package com.michaelgrundvig.frc.world.record;

/**
 * Where a joint stands in knowing where it is ({@code Joints/<label>/State}). Beside it is the text
 * of its fault, which says what is wrong with numbers and what to do about it ({@code
 * Joints/<label>/Fault}, empty while it is not faulted). {@link JointSentences#state} makes the
 * sentence from both.
 *
 * @param standing where it stands
 * @param toward what its homing pushes it toward, while it homes; {@link Toward#NOTHING} otherwise
 * @param secondsAllowed how long its homing may take at most, in seconds, while it homes; zero
 *     otherwise
 * @param fault why it is faulted, while it is; {@link Fault#NONE} otherwise
 */
public record JointState(Standing standing, Toward toward, double secondsAllowed, Fault fault) {
  /** Ready, with nothing more to say: every joint's state once it knows where it is. */
  public static final JointState READY = standing(Standing.READY);

  /** Unhomed, with nothing more to say. */
  public static final JointState UNHOMED = standing(Standing.UNHOMED);

  /** Where a joint stands. */
  public enum Standing {
    /** Held at positions, and it does not yet know where it is. */
    UNHOMED,
    /** Finding where it is, now. */
    HOMING,
    /** It knows where it is, or only runs at outputs. */
    READY,
    /** Its moves are refused until what is wrong is put right. */
    FAULTED
  }

  /** What a homing joint is pushed toward. */
  public enum Toward {
    /** It is not homing. */
    NOTHING,
    /** The hard stop at the lower end of its range. */
    LOWER_STOP,
    /** The hard stop at the upper end of its range. */
    UPPER_STOP,
    /** Its limit switch at the lower end of its travel. */
    LOWER_SWITCH,
    /** Its limit switch at the upper end of its travel. */
    UPPER_SWITCH
  }

  /** Why a joint is faulted. */
  public enum Fault {
    /** It is not faulted. */
    NONE,
    /** It stands in for a joint that has no mechanism, and nothing drives it. */
    NO_MECHANISM,
    /** One of its motor controllers did not take its settings at startup. */
    SETTINGS_NOT_TAKEN,
    /** Its absolute encoder gave no reading in the time it waited for one. */
    ENCODER_GAVE_NO_READING,
    /** The absolute encoder its loop closes on stopped answering while it ran. */
    ENCODER_LOST,
    /** Its absolute encoder says its magnet is too close or too far. */
    ENCODER_MAGNET_BAD,
    /** Its absolute encoder and its motor's count came apart: one of them slipped. */
    ENCODER_SLIPPED,
    /** Its homing pushed toward a hard stop and found none in time. */
    HOMING_FOUND_NO_STOP,
    /** Its homing did not trip its limit switch, or did not back off it, in time. */
    HOMING_MISSED_ITS_SWITCH,
    /** Its recipe homes it to a limit switch it does not have. */
    HOMING_HAS_NO_SWITCH,
    /** Its motor controller restarted, and it is only ever taken to be at rest at startup. */
    CONTROLLER_RESTARTED
  }

  /** A state that is its standing alone: not homing, and not faulted. */
  private static JointState standing(Standing standing) {
    return new JointState(standing, Toward.NOTHING, 0, Fault.NONE);
  }

  /** A joint homing toward something, for at most so many seconds. */
  public static JointState homing(Toward toward, double secondsAllowed) {
    return new JointState(Standing.HOMING, toward, secondsAllowed, Fault.NONE);
  }

  /**
   * A joint faulted for a reason. Each reason has one state, made once, so asking for it in a loop
   * makes nothing.
   */
  public static JointState faulted(Fault fault) {
    return FAULTED[fault.ordinal()];
  }

  /** The state of a joint faulted for each reason, in the reasons' order. */
  private static final JointState[] FAULTED = faultedForEachReason();

  private static JointState[] faultedForEachReason() {
    Fault[] reasons = Fault.values();
    JointState[] faulted = new JointState[reasons.length];
    for (Fault reason : reasons) {
      faulted[reason.ordinal()] = new JointState(Standing.FAULTED, Toward.NOTHING, 0, reason);
    }
    return faulted;
  }
}
