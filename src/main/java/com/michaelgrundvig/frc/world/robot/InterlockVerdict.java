package com.michaelgrundvig.frc.world.robot;

import java.util.List;

/**
 * What a joint's interlocks decided for one loop ({@link InterlockEnforcement#decide}).
 *
 * @param state what the interlocks remember now, to hand back in the next loop
 * @param allowed what the joint may do this loop: what it was asked, when no interlock holds it,
 *     and otherwise a position to hold or neutral
 * @param heldBy the interlocks that hold the joint short of what it was asked, in the order they
 *     are listed; empty when none does
 * @param restartProfile whether the joint's motion profile must start again from where the joint
 *     is, not carry on from where it had got to. That is so in the loop an interlock starts holding
 *     the joint where it can stop (a run-only-when, or a keep-out whose region the joint is
 *     inside), so that the profile brakes the joint from where it really is. It is also so in the
 *     loop a run-only-when lets go, so that the profile sets off for the command's goal from where
 *     the joint was held.
 */
public record InterlockVerdict(
    InterlockState state, JointAsk allowed, List<Interlock> heldBy, boolean restartProfile) {
  public InterlockVerdict {
    heldBy = List.copyOf(heldBy);
  }

  /**
   * The verdict to hand to the first {@link InterlockEnforcement#decide} of a run, which has no
   * last loop's verdict to hand it: one that carries what the interlocks remember and says that
   * nothing holds the joint.
   */
  public static InterlockVerdict startingFrom(InterlockState state) {
    return new InterlockVerdict(state, JointAsk.NEUTRAL, List.of(), false);
  }

  /**
   * What the interlocks decided in the loop after this verdict. When that is exactly what this
   * verdict already says, this very object is handed back, so a loop in which nothing changed makes
   * no new verdict. Otherwise a new verdict is made, which copies the list.
   *
   * @param state what the interlocks remember now. It is the last loop's own object whenever they
   *     remember nothing new ({@link InterlockState#remembering}), so it is compared as an object.
   * @param heldBy the interlocks that hold the joint now. It may be a list the caller goes on to
   *     change: it is only read here, or copied.
   */
  InterlockVerdict deciding(
      InterlockState state, JointAsk allowed, List<Interlock> heldBy, boolean restartProfile) {
    boolean nothingChanged =
        this.state == state
            && this.allowed.equals(allowed)
            && this.restartProfile == restartProfile
            && InterlockWorkspace.sameThings(this.heldBy, heldBy);
    if (nothingChanged) {
      return this;
    }
    return new InterlockVerdict(state, allowed, heldBy, restartProfile);
  }
}
