package com.michaelgrundvig.frc.world.record;

/**
 * What a record's index says when there is nothing for it to point at. An index in a record is
 * where a thing is in one of the robot definition's lists, counted from zero, and a record cannot
 * leave a field out, so a field with nothing to point at holds {@link #NONE}.
 */
public final class Identity {
  /** No index: the field has nothing to point at. */
  public static final int NONE = -1;

  private Identity() {}

  /** Whether an index points at something. */
  public static boolean isSome(int index) {
    return index >= 0;
  }
}
