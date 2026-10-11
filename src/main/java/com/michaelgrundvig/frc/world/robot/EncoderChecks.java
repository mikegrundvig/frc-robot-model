package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static com.michaelgrundvig.frc.world.robot.RobotMath.WRAP_MARGIN;

import com.michaelgrundvig.frc.world.robot.AbsoluteEncoders.Readings;
import java.math.BigDecimal;
import java.math.MathContext;
import java.math.RoundingMode;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

/**
 * The definition's check that a joint's absolute encoders can say where it is anywhere in its
 * range: between them they must tell the whole range apart, and their reading must not wrap inside
 * the range or within a fortieth of a turn of either end of it ({@link RobotMath#WRAP_MARGIN}).
 *
 * <p>A joint with no absolute encoder, or with no range, has nothing to check.
 *
 * <p>An absolute encoder written in the joint's own block is judged more strictly than one listed
 * among the robot's sensors, and its refusal says what to change in the block ({@link OwnEncoder}).
 */
final class EncoderChecks {
  private EncoderChecks() {}

  /**
   * Why a joint's absolute encoders cannot say where it is all along its range, if they cannot.
   *
   * @param sensors every sensor of the robot, the joint's own included
   */
  static Optional<String> whyNotReadAllAlong(Joint joint, List<Sensor> sensors) {
    List<Sensor.Encoder> encoders = AbsoluteEncoders.readingOnly(joint, sensors);
    if (encoders.isEmpty() || joint.range().isEmpty()) {
      return Optional.empty();
    }
    Range range = joint.range().get();
    double span;
    try {
      span = AbsoluteEncoders.decoding(encoders).span();
    } catch (IllegalArgumentException e) {
      return Optional.of(String.valueOf(e.getMessage()));
    }
    boolean isTheJointsOwnAlone =
        encoders.size() == 1 && joint.ownSensors().contains(encoders.get(0));
    if (isTheJointsOwnAlone) {
      return new OwnEncoder(joint, range, encoders.get(0)).whyNotReadAllAlong();
    }

    boolean tellsTheRangeApart = span > range.width();
    if (!tellsTheRangeApart) {
      return Optional.of(
          String.format(
              Locale.ROOT,
              "%s's absolute encoders tell %s of its %s apart: gear one differently, or add one",
              joint.label(),
              joint.shown(span),
              joint.shown(range.width())));
    }

    Range window = AbsoluteEncoders.window(encoders);
    boolean crossesTheWrap = range.lower() < window.lower() || range.upper() >= window.upper();
    if (crossesTheWrap) {
      return Optional.of(
          String.format(
              Locale.ROOT,
              "%s's absolute encoders read it from %s to %s, and its range, %s to %s, crosses where"
                  + " they wrap",
              joint.label(),
              joint.shown(window.lower()),
              joint.shown(window.upper()),
              joint.shown(range.lower()),
              joint.shown(range.upper())));
    }

    double margin = WRAP_MARGIN * span;
    boolean comesNearTheWrap =
        range.lower() < window.lower() + margin || range.upper() > window.upper() - margin;
    if (comesNearTheWrap) {
      double clearBelow = range.lower() - window.lower();
      double clearAbove = window.upper() - range.upper();
      return Optional.of(
          String.format(
              Locale.ROOT,
              "%s's absolute encoders read it from %s to %s, and its range, %s to %s, comes within"
                  + " %s of where they wrap, a fortieth of their turn: a reading a hair past the"
                  + " wrap reads a turn away",
              joint.label(),
              joint.shown(window.lower()),
              joint.shown(window.upper()),
              joint.shown(range.lower()),
              joint.shown(range.upper()),
              joint.shown(Math.min(clearBelow, clearAbove))));
    }
    return Optional.empty();
  }

  /**
   * The check of an absolute encoder written in a joint's own block, when it is the joint's only
   * one. The motor controller reads this encoder itself, so it has to fit what Phoenix can be told:
   * the joint's travel must lie inside one turn of the encoder's signal, a fortieth of a turn clear
   * of the wrap at each end, and the wrap can only be put within a turn of where the encoder reads
   * 0, which is the joint's zero.
   *
   * <p>When it does not fit, the refusal says the slowest gearing that would: the block's own ratio
   * scaled down until both conditions hold.
   *
   * <p>Readings here are in turns of the encoder's signal. That is the encoder's own turn, except
   * for a potentiometer, whose signal covers several turns ({@code turnsPerSignal}); the messages
   * give turns of the encoder itself.
   *
   * <p>An instance exists for the length of one check.
   */
  private static final class OwnEncoder {
    /** The most of the signal's range the travel may cover: all but the margin at each end. */
    private static final double MOST_OF_A_TURN = 1 - 2 * WRAP_MARGIN;

    /** The farthest from 0 the encoder may read anywhere in the travel. */
    private static final double FARTHEST_FROM_ZERO = 1 - WRAP_MARGIN;

    private final Joint joint;
    private final Range range;
    private final Readings readings;

    /** How many turns of its signal the encoder makes over the joint's travel. */
    private final double turnsOverTravel;

    /** The farthest from 0 the encoder reads anywhere in the travel, in turns of its signal. */
    private final double farthestFromZero;

    /** What to say is wrong, and what the team could do other than re-gear the encoder. */
    private record Complaint(String whatIsWrong, String orElse) {}

    OwnEncoder(Joint joint, Range range, Sensor.Encoder encoder) {
      this.joint = joint;
      this.range = range;
      readings = Readings.over(encoder, range);
      turnsOverTravel = readings.turns();
      farthestFromZero = Math.max(Math.abs(readings.low()), Math.abs(readings.high()));
    }

    Optional<String> whyNotReadAllAlong() {
      double wrapPoint = readings.wrapPoint();
      boolean clearOfTheWrap =
          readings.low() >= wrapPoint - 1 + WRAP_MARGIN
              && readings.high() <= wrapPoint - WRAP_MARGIN;
      if (clearOfTheWrap) {
        return Optional.empty();
      }
      Complaint complaint =
          turnsOverTravel >= MOST_OF_A_TURN ? coversTooMuchOfItsTurn() : readsTooFarFromZero();
      return Optional.of(
          String.format(
              Locale.ROOT,
              "%s. Gear it to at most %s of its turns for each %s, or %s",
              complaint.whatIsWrong(),
              roundedDown(block().ratio() * scaleThatFits()),
              perWhatOfTheJoint(),
              complaint.orElse()));
    }

    /**
     * How much slower the encoder would have to be geared to fit. Gearing it slower shrinks every
     * reading toward 0 by the same factor, so there is one factor at which the travel just fits in
     * the turn, and another at which the farthest reading is just near enough to 0. The smaller of
     * the two satisfies both.
     */
    private double scaleThatFits() {
      return Math.min(scaleThatFitsTheTravel(), scaleThatReadsNearZero());
    }

    private double scaleThatFitsTheTravel() {
      return MOST_OF_A_TURN / turnsOverTravel;
    }

    private double scaleThatReadsNearZero() {
      return FARTHEST_FROM_ZERO / farthestFromZero;
    }

    /**
     * The encoder turns too far over the travel for one turn of its signal to hold it.
     *
     * <p>When the joint's zero is a long way from its travel, reading near 0 limits the gearing
     * more than the length of the travel does. The message then says so as well, because the bound
     * it gives could not be worked out from the travel alone.
     */
    private Complaint coversTooMuchOfItsTurn() {
      boolean zeroLimitsItMore = scaleThatReadsNearZero() < scaleThatFitsTheTravel();
      String mustAlsoReadNearZero = "";
      String orMoveItsZero = "";
      if (zeroLimitsItMore) {
        mustAlsoReadNearZero =
            String.format(
                Locale.ROOT,
                " and it must read within %s turns of 0 all along it, counting from 0 at %s's"
                    + " zero,",
                shown(FARTHEST_FROM_ZERO * block().turnsPerSignal()),
                joint.label());
        orMoveItsZero =
            "move " + possessive(joint.label()) + " zero, its origin, to a hard stop, or ";
      }
      String whatIsWrong =
          String.format(
              Locale.ROOT,
              "%s's absolute encoder turns %s times over %s: %s must cover the travel, with a"
                  + " fortieth of it to spare at each end,%s to tell where %s is",
              joint.label(),
              shown(turnsOverTravel * block().turnsPerSignal()),
              shownTravel(),
              whatMustCoverTheTravel(),
              mustAlsoReadNearZero,
              joint.label());
      String orElse =
          orMoveItsZero
              + "leave it out and give "
              + possessive(joint.label())
              + " control a homing recipe";
      return new Complaint(whatIsWrong, orElse);
    }

    /** The travel fits in a turn, but lies too far from where the encoder reads 0. */
    private Complaint readsTooFarFromZero() {
      String whatIsWrong =
          String.format(
              Locale.ROOT,
              "%s's absolute encoder reads %s to %s turns over %s, counting from 0 at %s's zero,"
                  + " and must read within %s turns of 0 all along it, where its wrap can be put",
              joint.label(),
              shown(readings.low() * block().turnsPerSignal()),
              shown(readings.high() * block().turnsPerSignal()),
              shownTravel(),
              joint.label(),
              shown(FARTHEST_FROM_ZERO * block().turnsPerSignal()));
      String orElse = "move " + possessive(joint.label()) + " zero, its origin, to a hard stop";
      return new Complaint(whatIsWrong, orElse);
    }

    /** The encoder as the joint's block wrote it, which has the ratio a team would change. */
    private Driven.AbsoluteEncoder block() {
      return joint.driven().absoluteEncoder().orElseThrow();
    }

    private String shownTravel() {
      return String.format(
          Locale.ROOT,
          "its travel, %s to %s",
          joint.shown(range.lower()),
          joint.shown(range.upper()));
    }

    /** What has to cover the travel: the encoder's turn, or a potentiometer's signal range. */
    private String whatMustCoverTheTravel() {
      double turnsPerSignal = block().turnsPerSignal();
      if (turnsPerSignal == 1) {
        return "one of its turns";
      }
      return "its signal's range, " + shown(turnsPerSignal) + " of its turns,";
    }

    /** What the encoder's ratio is per: a turn of the joint, or of a sliding joint's drum. */
    private String perWhatOfTheJoint() {
      if (joint instanceof Joint.Revolute) {
        return "of " + possessive(joint.label()) + " turns";
      }
      return "turn of " + possessive(joint.label()) + " drum";
    }

    /**
     * A positive number rounded down to three significant figures. A bound that is rounded down is
     * still safe to meet exactly.
     */
    private static String roundedDown(double value) {
      return new BigDecimal(value)
          .round(new MathContext(3, RoundingMode.FLOOR))
          .stripTrailingZeros()
          .toPlainString();
    }
  }
}
