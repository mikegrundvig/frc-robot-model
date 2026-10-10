package com.michaelgrundvig.frc.world.robot;

import java.util.List;
import java.util.OptionalDouble;

/**
 * What a joint's interlocks remember from one loop to the next. The model holds no state, so the
 * robot program keeps this value: it starts from {@link InterlockEnforcement#initialState}, hands
 * the state back in each loop, and takes the next one from the {@link InterlockVerdict}.
 *
 * @param runsOnlyWhenHeldAt where the joint is being held because a run-only-when does not let it
 *     move, or empty while none holds it. It is worked out once, as the hold starts, and then kept:
 *     the joint is held at the point where it could stop, not at a point that moves with it.
 * @param keepOuts what each keep-out remembers: one for each of the joint's keep-outs, in the order
 *     they are listed in the {@link InterlockEnforcement}. A run-only-when has no entry here.
 */
public record InterlockState(OptionalDouble runsOnlyWhenHeldAt, List<KeepOutState> keepOuts) {
  public InterlockState {
    keepOuts = List.copyOf(keepOuts);
  }
}
