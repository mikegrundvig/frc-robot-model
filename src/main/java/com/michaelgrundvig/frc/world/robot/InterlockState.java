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

  /**
   * What the interlocks remember now, as the loop after this state worked it out. When that is
   * exactly what this state already says, this very object is handed back, so a loop in which the
   * interlocks have nothing new to remember makes no new state. Otherwise a new state is made,
   * which copies the list.
   *
   * @param runsOnlyWhenHeldAt where a run-only-when holds the joint now, or empty
   * @param keepOuts what each keep-out remembers now. It may be a list the caller goes on to
   *     change: it is only read here, or copied.
   */
  InterlockState remembering(OptionalDouble runsOnlyWhenHeldAt, List<KeepOutState> keepOuts) {
    boolean nothingNew =
        this.runsOnlyWhenHeldAt.equals(runsOnlyWhenHeldAt)
            && InterlockWorkspace.sameThings(this.keepOuts, keepOuts);
    if (nothingNew) {
      return this;
    }
    return new InterlockState(runsOnlyWhenHeldAt, keepOuts);
  }
}
