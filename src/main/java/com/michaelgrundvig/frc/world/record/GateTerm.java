package com.michaelgrundvig.frc.world.record;

/**
 * One term that keeps a gate false ({@code Gates/<name>/Blocking}, empty while the gate holds or
 * has no terms). A term that comes from the robot's definition is identified by what it is about. A
 * term that is a team's own is text, logged beside the terms ({@code Gates/<name>/BlockingText}).
 * {@link ControlSentences#gateTerm} makes each term's words.
 *
 * @param kind what the term says
 * @param index which thing it is about, by the kind: for {@link Kind#JOINT_POSITION_UNKNOWN} and
 *     {@link Kind#JOINT_NOT_ANSWERING} the joint's index in the definition's {@code joints()}; for
 *     {@link Kind#SENSOR_LOST} the sensor's index in the definition's {@code everySensor()}; for
 *     {@link Kind#TEXT} and {@link Kind#THREW} the index of its text in the text logged beside the
 *     terms; {@link Identity#NONE} for {@link Kind#SETTLING}
 */
public record GateTerm(Kind kind, int index) {
  /** What a term that keeps a gate false says. */
  public enum Kind {
    /**
     * A team's own words: the name of a gate that is one of this gate's terms, or why a gate that
     * cannot be was refused.
     */
    TEXT,
    /** The gate reads where a joint is, and the joint does not know. */
    JOINT_POSITION_UNKNOWN,
    /** The gate reads how fast a joint runs, and its motor controllers are silent. */
    JOINT_NOT_ANSWERING,
    /** The gate reads a sensor, and the sensor is lost. */
    SENSOR_LOST,
    /** The gate is debounced, and what it reads has not yet held for its time. */
    SETTLING,
    /** The gate's logic threw. Its text is what was thrown. */
    THREW
  }

  /** A term in a team's own words, by where its text is in the text logged beside the terms. */
  public static GateTerm text(int index) {
    return new GateTerm(Kind.TEXT, index);
  }

  /** A debounced gate that is still settling. */
  public static GateTerm settling() {
    return new GateTerm(Kind.SETTLING, Identity.NONE);
  }
}
