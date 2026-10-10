package com.michaelgrundvig.frc.world.physical;

import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.params.provider.Arguments;

/** Rows for the tables of this package's tests. */
final class Cases {
  private Cases() {}

  /** A row of a table of refusals: what was asked, and pieces of the message that refuses it. */
  static Arguments refused(String label, ThrowingCallable building, String... pieces) {
    return argumentSet(label, building, pieces);
  }

  /** What the refusal says, whole or in pieces. */
  static void assertRefused(ThrowingCallable building, String[] pieces) {
    assertThatThrownBy(building).hasMessageContainingAll(pieces);
  }
}
