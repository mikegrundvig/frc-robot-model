package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.encoder;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.lightSlide;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.AbsoluteEncoders.Decoding;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Where what a joint's absolute encoders tell changes, as a table of questions and exact answers.
 * Every joint here is a slide whose encoder turns once for each meter, so that a reading in turns
 * is the position in meters and a row can sit exactly on an edge. How the same answers come out for
 * real arms and elevators is in {@link RobotMathTest} and {@link AbsoluteReadingsTest}.
 */
class AbsoluteEncodersTest {
  /** A fortieth of the one meter these encoders tell apart. */
  private static final double MARGIN = RobotMath.WRAP_MARGIN * 1;

  /** A slide with its stops at these positions, in meters. */
  private static Joint.Prismatic slide(double lower, double upper) {
    return lightSlide("slide", new Vector3(1, 0, 0), Range.meters(lower, upper));
  }

  private static final Joint.Prismatic SLIDE = slide(0.25, 0.75);

  /** An absolute encoder on a slide that turns this many times for each meter. */
  private static Sensor.Encoder turning(String label, double turnsPerMeter, int countsPerTurn) {
    return encoder(label, SLIDE, turnsPerMeter, true, Optional.of(countsPerTurn));
  }

  private static Sensor.Encoder onceAMeter(Joint joint) {
    return encoder("encoder", joint, 1, true);
  }

  /** Where the encoder on a slide wraps, or the word "refused" if no turn holds its readings. */
  private static Object wrapsAt(double lower, double upper) {
    try {
      return AbsoluteEncoders.discontinuityPoint(onceAMeter(slide(lower, upper)));
    } catch (IllegalArgumentException e) {
      return "refused";
    }
  }

  /** How a slide's one encoder, listed among the robot's sensors, is judged. */
  private static String judged(double lower, double upper) {
    Joint slide = slide(lower, upper);
    Optional<String> why = EncoderChecks.whyNotReadAllAlong(slide, List.of(onceAMeter(slide)));
    if (why.isEmpty()) {
      return "taken";
    }
    if (why.get().contains("crosses where they wrap")) {
      return "crosses the wrap";
    }
    if (why.get().contains("of where they wrap")) {
      return "too near the wrap";
    }
    if (why.get().contains("apart: gear one differently")) {
      return "tells too little apart";
    }
    return why.get();
  }

  /**
   * How a point for a slide's encoder to wrap at is judged, as a motor controller's helper judges a
   * team's own: clear of the slide's range, too near it, or inside it.
   */
  private static String wrapping(double lower, double upper, double wrapPoint) {
    String problem = ControllerPlan.wrapProblem(onceAMeter(slide(lower, upper)), wrapPoint);
    if (problem == null) {
      return "clear";
    }
    if (problem.contains(" inside slide's range")) {
      return "inside the range";
    }
    return "too near the range";
  }

  /** Which of these encoders a decoder reads alone, or "their difference" for a pair. */
  private static String readBy(Sensor.Encoder... encoders) {
    return switch (AbsoluteEncoders.decoding(List.of(encoders))) {
      case Decoding.ByOne byOne -> byOne.encoder().label();
      case Decoding.ByDifference byDifference -> "their difference";
    };
  }

  @ParameterizedTest
  @MethodSource({"wrapPoints", "positions", "decodings", "judgements", "ownWrapPoints", "misfits"})
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments row(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  // spotless:off

  /** An encoder's turn runs from one turn below its wrap point up to, but not onto, the point. */
  static Stream<Arguments> wrapPoints() {
    return Stream.of(
        row("readings in the middle of a turn wrap at its end", () -> wrapsAt(0.25, 0.75), 1.0),
        row("a reading just short of the wrap is held", () -> wrapsAt(0.5, Math.nextDown(1.0)), 1.0),
        row("a reading on the wrap is not", () -> wrapsAt(0.5, 1.0), "refused"),
        row("a reading on the bottom of the turn is held", () -> wrapsAt(-1.0, -0.5), 0.0),
        row("the next reading below it is not", () -> wrapsAt(Math.nextDown(-1.0), -0.5), "refused"));
  }

  /** A reading is put in the window from 0 m to 1 m that the slide's encoder tells apart. */
  static Stream<Arguments> positions() {
    Sensor.Encoder encoder = onceAMeter(SLIDE);
    return Stream.of(
        row("a reading inside the window is itself", () -> AbsoluteEncoders.positionFor(encoder, 0.5), 0.5),
        row("a reading on the bottom of the window stays there", () -> AbsoluteEncoders.positionFor(encoder, 0), 0.0),
        row("a reading on the wrap is the bottom", () -> AbsoluteEncoders.positionFor(encoder, 1), 0.0),
        row("a reading below the window comes up a turn", () -> AbsoluteEncoders.positionFor(encoder, -0.25), 0.75),
        row("a reading above the window comes down a turn", () -> AbsoluteEncoders.positionFor(encoder, 1.25), 0.25));
  }

  /** A decoder reads the encoder that tells the most apart, or a pair that tells more. */
  static Stream<Arguments> decodings() {
    return Stream.of(
        row("of two geared alike, the first", () -> readBy(turning("first", 1, 4096), turning("second", 1, 4096)), "first"),
        row("the slower of two", () -> readBy(turning("fast", 4, 4096), turning("slow", 1, 4096)), "slow"),
        row("a pair geared nearly alike, by their difference", () -> readBy(turning("a", 2, 4096), turning("b", 1.5, 4096)), "their difference"),
        row("a pair that tells no more than a third encoder does alone, the third", () -> readBy(turning("a", 2, 4096), turning("b", 1.5, 4096), turning("slow", 0.5, 4096)), "slow"),
        row("one turning exactly twice as fast is coarse then fine, and needs no counts", () -> readBy(encoder("fast", SLIDE, 2, true, Optional.empty()), encoder("slow", SLIDE, 1, true, Optional.empty())), "slow"),
        row("a pair fine enough to pick the right turn, by their difference", () -> readBy(turning("a", 1.5, 16), turning("b", 1, 16)), "their difference"),
        row("a pair whose error is exactly half a turn is not fine enough", () -> readBy(turning("a", 1.5, 8), turning("b", 1, 8)), "b"));
  }

  /** The definition takes an encoder that tells the range apart and wraps clear of it. */
  static Stream<Arguments> judgements() {
    return Stream.of(
        row("a range well inside the turn is taken", () -> judged(0, 0.5), "taken"),
        row("a range as wide as the turn is not told apart", () -> judged(0, 1), "tells too little apart"),
        row("a range that reaches the wrap crosses it", () -> judged(0.5, 1.0), "crosses the wrap"),
        row("a range just short of the wrap is too near it", () -> judged(0.5, Math.nextDown(1.0)), "too near the wrap"),
        row("a range a fortieth of a turn short of the wrap is taken", () -> judged(0.5, 1 - MARGIN), "taken"),
        row("the next range longer is too near", () -> judged(0.5, Math.nextUp(1 - MARGIN)), "too near the wrap"),
        row("a range that starts on the bottom of the turn is too near, not across", () -> judged(-1.0, -0.5), "too near the wrap"),
        row("the next range lower crosses the wrap", () -> judged(Math.nextDown(-1.0), -0.5), "crosses the wrap"),
        row("a range a fortieth of a turn above the bottom is taken", () -> judged(-1 + MARGIN, -0.5), "taken"),
        row("the next range lower is too near", () -> judged(Math.nextDown(-1 + MARGIN), -0.5), "too near the wrap"));
  }

  /** A wrap point is clear of a range that ends a fortieth of a turn or more short of it. */
  static Stream<Arguments> ownWrapPoints() {
    return Stream.of(
        row("a range that ends a fortieth of a turn short of the wrap is clear", () -> wrapping(-0.5, 0.025, 0.05), "clear"),
        row("the next range longer is too near", () -> wrapping(-0.5, Math.nextUp(0.025), 0.05), "too near the range"),
        row("a range that ends on the wrap is too near, not across", () -> wrapping(-0.5, 0.05, 0.05), "too near the range"),
        row("the next range longer has the wrap inside it", () -> wrapping(-0.5, Math.nextUp(0.05), 0.05), "inside the range"));
  }

  /** How badly one cosine fits gravity, as a share of the cosine's peak. */
  static Stream<Arguments> misfits() {
    return Stream.of(
        row("no gap is a perfect fit", () -> JointLoads.misfit(0, 2), 0.0),
        row("no gap and no gravity is a perfect fit too", () -> JointLoads.misfit(0, 0), 0.0),
        row("a gap is a share of the peak", () -> JointLoads.misfit(0.5, 2), 0.25),
        row("a gap with no peak to measure it by is as bad as can be", () -> JointLoads.misfit(0.5, 0), Double.POSITIVE_INFINITY));
  }

  // spotless:on
}
