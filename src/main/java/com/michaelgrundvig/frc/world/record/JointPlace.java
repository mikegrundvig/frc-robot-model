package com.michaelgrundvig.frc.world.record;

/**
 * Where a joint is, in terms of its named places, or what a joint that only runs at outputs is
 * doing ({@code Joints/<label>/At}). {@link JointPlaces} works it out, and {@link
 * JointSentences#place} makes the sentence from it.
 *
 * @param where how the joint lies among its places, or what it is doing
 * @param place which place or output, or {@link Identity#NONE}: for {@link Where#AT}, {@link
 *     Where#BELOW} and {@link Where#ABOVE} the place's index in the joint's {@code positions()};
 *     for {@link Where#BETWEEN} the index of the lower of the two places; for {@link Where#RUNNING}
 *     the output's index in the joint's {@code outputs()}
 * @param other for {@link Where#BETWEEN}, the index of the upper of the two places in the joint's
 *     {@code positions()}; {@link Identity#NONE} otherwise
 */
public record JointPlace(Where where, int place, int other) {
  /** A joint that does not know where it is. */
  public static final JointPlace UNKNOWN = of(Where.UNKNOWN);

  /** A joint with no named places, which is said by its position as a number. */
  public static final JointPlace UNNAMED = of(Where.UNNAMED);

  /** A joint that only runs at outputs, stopped. */
  public static final JointPlace STOPPED = of(Where.STOPPED);

  /** A joint that only runs at outputs, driven at something that has no name. */
  public static final JointPlace IDLE = of(Where.IDLE);

  /** How a joint lies among its named places, or what one that only runs at outputs is doing. */
  public enum Where {
    /** It holds positions, and does not know where it is. */
    UNKNOWN,
    /** It has no named places: its position, as a number, is all there is to say. */
    UNNAMED,
    /** Within its band of one of its places. */
    AT,
    /** Between two of its places, and within its band of neither. */
    BETWEEN,
    /** Below its lowest place. */
    BELOW,
    /** Above its highest place. */
    ABOVE,
    /** It only runs at outputs, and a command runs it at one of its named outputs. */
    RUNNING,
    /** It only runs at outputs, and its motors are neutral. */
    STOPPED,
    /** It only runs at outputs, and it is driven at something that has no name. */
    IDLE
  }

  private static JointPlace of(Where where) {
    return new JointPlace(where, Identity.NONE, Identity.NONE);
  }

  /** Where a joint is by one of its places: at it, below it or above it. */
  public static JointPlace by(Where where, int place) {
    return new JointPlace(where, place, Identity.NONE);
  }

  /** Between two of its places, by their indices: the lower one, then the upper. */
  public static JointPlace between(int lower, int upper) {
    return new JointPlace(Where.BETWEEN, lower, upper);
  }

  /** Running at one of its named outputs, by the output's index. */
  public static JointPlace running(int output) {
    return new JointPlace(Where.RUNNING, output, Identity.NONE);
  }
}
