package com.michaelgrundvig.frc.world.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import java.util.OptionalDouble;

/**
 * What a joint's absolute encoders can tell about where the joint is: where one encoder's reading
 * wraps, how much of the joint's travel one or two encoders tell apart, and where the joint is for
 * a reading.
 *
 * <p>An absolute encoder reads where it is within one of its own turns, and its reading wraps round
 * at the end of the turn. Geared to the joint, it turns some number of times for each turn of the
 * joint (or each meter, for a joint that slides), which is its ratio. So it tells positions apart
 * across 1 / |ratio| of the joint's turns and then repeats. Two encoders with different ratios can
 * tell apart more than either does alone ({@link #decoding}).
 *
 * <p>Throughout, a reading is in the encoder's turns, with the encoder's offset already applied so
 * that it reads 0 with the joint at 0.
 */
final class AbsoluteEncoders {
  private AbsoluteEncoders() {}

  /**
   * The lowest and highest readings an encoder gives as its joint goes from one end of its range to
   * the other, in the encoder's turns. A joint that turns freely has no range, and both are then
   * the reading where the joint rests.
   */
  record Readings(double low, double high) {
    /**
     * The readings over the range of the one joint the encoder reads.
     *
     * @throws IllegalStateException if the encoder reads several joints
     */
    static Readings overTravelOf(Sensor.Encoder encoder) {
      Joint joint = encoder.joint();
      if (joint.range().isEmpty()) {
        double atRest = turnsAt(encoder, joint.restPosition());
        return new Readings(atRest, atRest);
      }
      return over(encoder, joint.range().get());
    }

    /** The readings from one end of a range to the other. */
    static Readings over(Sensor.Encoder encoder, Range range) {
      // An encoder geared backward reads highest at the low end of the range.
      double atLowerStop = turnsAt(encoder, range.lower());
      double atUpperStop = turnsAt(encoder, range.upper());
      return new Readings(Math.min(atLowerStop, atUpperStop), Math.max(atLowerStop, atUpperStop));
    }

    /** How many of the encoder's turns the readings cover. */
    double turns() {
      return high - low;
    }

    /**
     * Where the encoder's reading should wrap, in its turns, to be as far from these readings as it
     * can be: half a turn past their middle, which puts the middle of the encoder's turn on the
     * middle of the readings. Phoenix only takes a point from 0 to 1, so it is held to that.
     */
    double wrapPoint() {
      double middle = (low + high) / 2;
      return Math.max(0, Math.min(1, middle + 0.5));
    }
  }

  /**
   * How a decoder reads a joint's absolute encoders, and how much of the joint's travel it tells
   * apart that way.
   */
  sealed interface Decoding permits Decoding.ByOne, Decoding.ByDifference {
    /** How far the joint can move with every position still told apart, in the joint's units. */
    double span();

    /**
     * One encoder says where the joint is. Any others are finer, and add precision but no reach.
     *
     * @param encoder the encoder that tells the most apart, the one geared slowest
     */
    record ByOne(Sensor.Encoder encoder, double span) implements Decoding {}

    /**
     * Two encoders geared nearly alike say where the joint is by the difference between their
     * readings, which is a vernier.
     */
    record ByDifference(double span) implements Decoding {}
  }

  /**
   * The absolute encoders, among a robot's sensors, that read one joint and nothing else, in the
   * sensors' order. These are the ones that can say where that joint is at power on.
   */
  static List<Sensor.Encoder> readingOnly(Joint joint, List<Sensor> sensors) {
    List<Sensor.Encoder> found = new ArrayList<>();
    for (Sensor sensor : sensors) {
      if (sensor instanceof Sensor.Encoder encoder
          && encoder.absolute()
          && encoder.joints().equals(List.of(joint))) {
        found.add(encoder);
      }
    }
    return found;
  }

  /** How far a joint moves for one turn of it: 2π radians, or 1 for a joint measured in meters. */
  private static double jointUnitsPerTurn(Joint joint) {
    if (joint instanceof Joint.Revolute) {
      return 2 * Math.PI;
    }
    return 1;
  }

  /**
   * What an encoder that reads one joint reads with the joint at a position, in the encoder's
   * turns.
   */
  static double turnsAt(Sensor.Encoder encoder, double position) {
    Joint joint = encoder.joint();
    double jointTurns = joint instanceof Joint.Revolute ? position / (2 * Math.PI) : position;
    return encoder.ratio(joint) * jointTurns;
  }

  /**
   * Where an absolute encoder's reading should wrap, in its turns: Phoenix's {@code
   * AbsoluteSensorDiscontinuityPoint} ({@link RobotMath#discontinuityPoint}).
   *
   * @throws IllegalArgumentException if the encoder is not absolute, or reads more over its joint's
   *     range than one turn that wraps there holds
   */
  static double discontinuityPoint(Sensor.Encoder encoder) {
    if (!encoder.absolute()) {
      throw new IllegalArgumentException(encoder.label() + " isn't absolute: it doesn't wrap");
    }
    Readings readings = Readings.overTravelOf(encoder);
    double wrapPoint = readings.wrapPoint();
    // The encoder's turn runs from one turn below the wrap point up to the wrap point itself.
    boolean readingsSpillOutOfTheTurn =
        readings.low() < wrapPoint - 1 || readings.high() >= wrapPoint;
    if (readingsSpillOutOfTheTurn) {
      throw new IllegalArgumentException(
          String.format(
              Locale.ROOT,
              "%s reads %.3f to %.3f turns over %s's range, which no one turn from -1 to 1 holds",
              encoder.label(),
              readings.low(),
              readings.high(),
              encoder.joint().label()));
    }
    return wrapPoint;
  }

  /**
   * How far inside an encoder's turn its joint's range sits when the encoder wraps at a point, in
   * the encoder's turns, at whichever end is nearer the wrap ({@link RobotMath#wrapMargin}).
   */
  static double wrapMargin(Sensor.Encoder encoder, double discontinuityPoint) {
    Joint joint = encoder.joint();
    if (joint.range().isEmpty()) {
      return Double.POSITIVE_INFINITY;
    }
    Readings readings = Readings.over(encoder, joint.range().get());
    double bottomOfTurn = discontinuityPoint - 1;
    double clearAtTheBottom = readings.low() - bottomOfTurn;
    double clearAtTheTop = discontinuityPoint - readings.high();
    return Math.min(clearAtTheBottom, clearAtTheTop);
  }

  /**
   * Where a joint is for one absolute encoder's reading, in the joint's units, inside the window
   * the encoder tells apart ({@link #window}).
   *
   * @param turns the encoder's reading, in its turns, growing as the joint moves forward
   */
  static double positionFor(Sensor.Encoder encoder, double turns) {
    Range window = window(List.of(encoder));
    Joint joint = encoder.joint();
    double position = turns / Math.abs(encoder.ratio(joint)) * jointUnitsPerTurn(joint);
    double span = window.upper() - window.lower();
    // Java's remainder keeps the sign of what is divided, so a position below the window gives a
    // negative remainder, which one more span brings into the window.
    double pastTheBottom = (position - window.lower()) % span;
    if (pastTheBottom < 0) {
      pastTheBottom = pastTheBottom + span;
    }
    return window.lower() + pastTheBottom;
  }

  /**
   * The positions of a joint that a decoder reading these encoders tells apart, from the lowest to
   * the highest, in the joint's units ({@link RobotMath#absoluteWindow}).
   *
   * @throws IllegalArgumentException as {@link #decoding} does
   */
  static Range window(List<Sensor.Encoder> encoders) {
    Decoding decoding = decoding(encoders);
    Joint joint = encoders.get(0).joint();
    return switch (decoding) {
      case Decoding.ByOne byOne -> windowOfOne(byOne.encoder(), joint);
      case Decoding.ByDifference byDifference -> windowAboutTheRange(joint, byDifference.span());
    };
  }

  /**
   * One encoder's window is its own turn, which ends where its reading wraps: from one turn below
   * the wrap point to the wrap point, turned into the joint's units by its ratio.
   */
  private static Range windowOfOne(Sensor.Encoder encoder, Joint joint) {
    double wrapPoint = Readings.overTravelOf(encoder).wrapPoint();
    double ratio = encoder.ratio(joint);
    double perTurn = jointUnitsPerTurn(joint);
    double atBottomOfTurn = (wrapPoint - 1) / ratio * perTurn;
    double atWrap = wrapPoint / ratio * perTurn;
    // An encoder geared backward has the bottom of its turn at the joint's higher position.
    return Range.in(joint, Math.min(atBottomOfTurn, atWrap), Math.max(atBottomOfTurn, atWrap));
  }

  /**
   * A pair read by their difference can have its window anywhere, since where the difference wraps
   * is the decoder's own choice. It is centered on the joint's range, or on where a joint that
   * turns freely rests.
   */
  private static Range windowAboutTheRange(Joint joint, double span) {
    double middle =
        joint
            .range()
            .map(range -> (range.lower() + range.upper()) / 2)
            .orElse(joint.restPosition());
    return Range.in(joint, middle - span / 2, middle + span / 2);
  }

  /**
   * How a decoder reads these encoders: by the one that tells the most apart, or by the difference
   * between a pair if some pair tells more apart than any one of them.
   *
   * @throws IllegalArgumentException if there are no encoders, if they are not all absolute and on
   *     the same one joint, or if a pair geared nearly alike does not give each encoder's counts
   *     per turn
   */
  static Decoding decoding(List<Sensor.Encoder> encoders) {
    if (encoders.isEmpty()) {
      throw new IllegalArgumentException("no encoders tell anything apart");
    }
    Joint joint = encoders.get(0).joint();
    for (Sensor.Encoder encoder : encoders) {
      if (!encoder.absolute() || !encoder.joints().equals(List.of(joint))) {
        throw new IllegalArgumentException(
            encoder.label() + ": a span is of absolute encoders on one joint");
      }
    }

    // Spans are compared in the joint's turns (or meters) and converted at the end.
    double widest = 0;
    Optional<Sensor.Encoder> widestAlone = Optional.empty();
    for (Sensor.Encoder encoder : encoders) {
      double alone = 1 / Math.abs(encoder.ratio(joint));
      if (alone > widest) {
        widest = alone;
        widestAlone = Optional.of(encoder);
      }
    }
    for (int first = 0; first < encoders.size(); first++) {
      for (int second = first + 1; second < encoders.size(); second++) {
        OptionalDouble byDifference =
            spanByDifference(encoders.get(first), encoders.get(second), joint);
        if (byDifference.isPresent() && byDifference.getAsDouble() > widest) {
          widest = byDifference.getAsDouble();
          widestAlone = Optional.empty();
        }
      }
    }

    double span = widest * jointUnitsPerTurn(joint);
    if (widestAlone.isPresent()) {
      return new Decoding.ByOne(widestAlone.get(), span);
    }
    return new Decoding.ByDifference(span);
  }

  /**
   * How much of the joint's travel a pair of encoders tells apart when read by the difference
   * between their readings, in the joint's turns, if the pair can be read that way.
   *
   * <p>The difference between two encoders' readings itself turns once for every 1 / |difference of
   * their ratios| turns of the joint, which is far more than either encoder alone when the ratios
   * are close. Two encoders on 17 and 18 teeth against a turret's ring tell more than three of the
   * turret's turns apart.
   *
   * <p>A pair is read this way only when two things hold. Their ratios must differ by less than the
   * smaller of them: otherwise one turns at least twice as fast as the other, and a decoder reads
   * them coarse then fine instead, which adds no reach. And their resolution must be good enough
   * for the difference to pick the right turn of the slower encoder: with each reading one count
   * off, the difference must still be off by less than half of that turn.
   *
   * @throws IllegalArgumentException for a pair geared nearly alike that does not give each
   *     encoder's counts per turn, since nothing else says whether the difference can be trusted
   */
  private static OptionalDouble spanByDifference(
      Sensor.Encoder first, Sensor.Encoder second, Joint joint) {
    double firstRatio = Math.abs(first.ratio(joint));
    double secondRatio = Math.abs(second.ratio(joint));
    double slowerRatio = Math.min(firstRatio, secondRatio);
    double ratiosApart = Math.abs(firstRatio - secondRatio);
    boolean gearedNearlyAlike = ratiosApart < slowerRatio;
    if (!gearedNearlyAlike) {
      return OptionalDouble.empty();
    }
    if (first.countsPerTurn().isEmpty() || second.countsPerTurn().isEmpty()) {
      throw new IllegalArgumentException(
          "decoding "
              + first.label()
              + " and "
              + second.label()
              + " by their difference needs each encoder's counts per turn: give them");
    }
    // One count of each encoder, as a share of its turn, is the most the difference can be off by.
    double differenceOffBy = 1.0 / first.countsPerTurn().get() + 1.0 / second.countsPerTurn().get();
    boolean picksTheRightTurn = 2 * slowerRatio * differenceOffBy < ratiosApart;
    if (!picksTheRightTurn) {
      return OptionalDouble.empty();
    }
    return OptionalDouble.of(1 / ratiosApart);
  }
}
