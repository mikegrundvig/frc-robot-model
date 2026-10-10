package com.michaelgrundvig.frc.world.robot;

import java.util.List;

/**
 * How far apart a joint's absolute encoder and its motor's count may be, still, and whether they
 * are: apart by more than the margin, and more than a loop's travel at the joint's speed, one of
 * them has slipped (a magnet loose on its shaft, a belt or a chain that skipped). An analog encoder
 * can't say it's unplugged, and a magnet turned on its shaft still reads: this is how either shows.
 * How long they must stay apart before the robot says so is the robot program's, which reads a
 * clock; this answers each loop alone.
 *
 * @param joint the joint, which says a position in its units
 * @param margin how far apart they may be, still, in the joint's units ({@link #margin})
 * @param turn the encoder's turn at the joint, where the joint travels past it ({@link #turn}):
 *     they're apart by the nearest way round it; infinite where it doesn't
 */
public record EncoderGap(Joint joint, double margin, double turn) {
  /**
   * The least margin, without one in the joint's control: 5° on a joint that turns, 2 cm on one
   * that slides. The backlash and give between a motor and its joint's encoder (a gearbox's lash, a
   * chain's or a belt's stretch, an elevator's rigging) is a degree or two, or a centimeter, and no
   * resolution shows it; a magnet that slipped, or a belt that skipped a tooth, is usually well
   * past it.
   */
  static final double ANGLE_MARGIN = Math.toRadians(5);

  static final double LENGTH_MARGIN = 0.02;

  /**
   * ...and never under this many of the encoder's counts at the joint: a reading flickers by a
   * count, and an encoder geared down from its joint has coarser ones there.
   */
  static final int COUNTS = 8;

  /**
   * The counts in an encoder's signal: a CANcoder's in its turn, and an analog reading's over its
   * range (12 bits); a through-bore's absolute output's (10 bits), the coarsest of the rest the
   * robot program reads, on a SPARK's data port or a Systemcore port.
   */
  static final int CANCODER_COUNTS = 4096;

  static final int ANALOG_COUNTS = 4096;
  static final int OTHER_COUNTS = 1024;

  /**
   * A joint's, from its block: the margin its control gives or the least of the encoder's counts,
   * and the turn its encoder travels past, if it does.
   */
  public static EncoderGap of(RobotMath math, Joint joint, MechanismControl control) {
    return new EncoderGap(joint, margin(math, joint, control), turn(math, joint));
  }

  /**
   * The encoder's turn at a joint that travels past it, in the joint's units: one that turns
   * freely, or whose range is wider than the window its encoder tells apart ({@link
   * RobotMath#absoluteWindow}). Its encoder reads within its turn while its motor's count goes on,
   * so the two are a whole turn apart where they agree. Infinite for a joint whose range the
   * encoder tells apart, where a reading a turn off is one that slipped.
   */
  static double turn(RobotMath math, Joint joint) {
    Sensor.Encoder encoder = math.absoluteEncoder(joint);
    if (encoder == null) {
      return Double.POSITIVE_INFINITY;
    }
    double window = math.absoluteWindow(List.of(encoder)).width();
    boolean past = joint.range().map(range -> range.width() > window).orElse(true);
    return past ? window : Double.POSITIVE_INFINITY;
  }

  /**
   * How far apart a joint's encoder and its motor's count may be, still: its control's ({@code
   * withEncoderGap}), else the least margin or eight of the encoder's counts at the joint,
   * whichever is more: its counts its definition's, else its device's.
   */
  static double margin(RobotMath math, Joint joint, MechanismControl control) {
    if (control.encoderGap().isPresent()) {
      return control.encoderGap().get().margin();
    }
    boolean turns = joint instanceof Joint.Revolute;
    double least = turns ? ANGLE_MARGIN : LENGTH_MARGIN;
    Sensor.Encoder encoder = math.absoluteEncoder(joint);
    if (encoder == null) {
      return least;
    }
    // Its signal's turns for each of the joint's units: radians, or meters.
    double perUnit = Math.abs(encoder.ratio(joint)) / (turns ? 2 * Math.PI : 1);
    int counts = encoder.countsPerTurn().orElseGet(() -> counts(joint));
    return perUnit > 0 ? Math.max(least, COUNTS / (counts * perUnit)) : least;
  }

  /** The counts in a joint's encoder's signal, by what it's wired to. */
  private static int counts(Joint joint) {
    Driven.AbsoluteEncoder wired = joint.driven().absoluteEncoder().orElse(null);
    if (wired != null && wired.cancoder().isPresent()) {
      return CANCODER_COUNTS;
    }
    boolean analog =
        wired != null
            && wired
                .port()
                .filter(port -> port.signal() == MechanismWiring.PortEncoder.Signal.ANALOG)
                .isPresent();
    return analog ? ANALOG_COUNTS : OTHER_COUNTS;
  }

  /**
   * Whether they're apart this loop: by more than the margin and a loop's travel at the joint's
   * speed. Either reading missing (NaN) says nothing.
   *
   * @param encoder where the encoder says the joint is
   * @param motor where its motor's count says it is
   * @param velocity how fast it moves: a loop's travel at it is allowed beside the margin
   * @param loopSeconds how long a loop is, in seconds
   */
  public boolean apart(double encoder, double motor, double velocity, double loopSeconds) {
    return gap(encoder, motor) > margin + Math.abs(velocity) * loopSeconds;
  }

  /** Whether they agree now, inside the margin: an encoder distrusted for its gap is trusted. */
  public boolean agree(double encoder, double motor) {
    return gap(encoder, motor) <= margin;
  }

  /** How far apart they are, in the joint's words: "6° apart, more than 5°". */
  public String shown(double encoder, double motor) {
    return joint.shown(gap(encoder, motor)) + " apart, more than " + joint.shown(margin);
  }

  /** How far apart they are: the nearest way round the encoder's turn, where it wraps. */
  private double gap(double encoder, double motor) {
    double apart = encoder - motor;
    return Math.abs(Double.isInfinite(turn) ? apart : apart - turn * Math.rint(apart / turn));
  }
}
