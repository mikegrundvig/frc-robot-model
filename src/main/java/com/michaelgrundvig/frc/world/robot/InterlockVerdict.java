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
}
