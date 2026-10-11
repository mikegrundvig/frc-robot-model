package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.encoder;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.linear;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.link;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.point;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.prismatic;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Translation3d;

/**
 * How a robot's parts may refer to each other, and where the joints end up when they refer to each
 * other soundly, as a table of questions and exact answers. Every joint is a slide a meter long, so
 * that positions add up exactly: {@code A} slides along x, {@code B} along y and {@code C} along z.
 */
class StructureChecksTest {
  private static Joint.Prismatic slide(String label, Part parent, Vector3 axis, double rest) {
    Joint.Prismatic slide =
        prismatic(
            label,
            parent,
            point(0, 0, 0),
            axis,
            Range.meters(-1, 1),
            link(label + " link", 1, point(0, 0, 0), point(0.1, 0.1, 0.1)));
    return JointPrismaticBuilder.builder(slide).rest(rest).build();
  }

  private static final Vector3 X = new Vector3(1, 0, 0);
  private static final Vector3 Y = new Vector3(0, 1, 0);

  // Three slides on the chassis, for couplings.
  private static final Joint.Prismatic A = slide("a", Part.CHASSIS, X, 0);
  private static final Joint.Prismatic B = slide("b", Part.CHASSIS, Y, 0);
  private static final Joint.Prismatic C = slide("c", Part.CHASSIS, UP, 0);

  // Three slides stacked: the top rides on the middle, which rides on the bottom.
  private static final Joint.Prismatic BOTTOM = slide("bottom", Part.CHASSIS, X, 0);
  private static final Joint.Prismatic MIDDLE = slide("middle", BOTTOM, Y, 0);
  private static final Joint.Prismatic TOP = slide("top", MIDDLE, UP, 0);

  private static final Joint.Prismatic NOT_LISTED = slide("stray", Part.CHASSIS, X, 0);

  /** A slide on the chassis that rests this far from where a coupling to {@code A} puts it. */
  private static Joint.Prismatic restingOff(double off) {
    return slide("off", Part.CHASSIS, Y, off);
  }

  private static List<String> problems(List<Joint> joints, List<Coupling> couplings) {
    return StructureChecks.problems(joints, couplings, List.of(), List.of(), List.of());
  }

  private static List<String> problemsWith(List<Joint> joints, Sensor... sensors) {
    return StructureChecks.problems(joints, List.of(), List.of(), List.of(sensors), List.of());
  }

  private static RobotMath math(List<Joint> joints, Coupling... couplings) {
    return RobotMath.of(ConfigFixtures.body(), joints, List.of(couplings), List.of(), List.of());
  }

  /** Where the top slide's frame is with the stack at these positions. */
  private static Translation3d topAt(List<Joint> listed, double bottom, double middle, double top) {
    return math(listed)
        .pose(TOP, Map.of(BOTTOM, bottom, MIDDLE, middle, TOP, top))
        .getTranslation();
  }

  @ParameterizedTest
  @MethodSource({"references", "couplings", "positions"})
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments row(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  // spotless:off

  /** Every label is used once, and every joint a part names is one the robot lists. */
  static Stream<Arguments> references() {
    Joint.Prismatic sharesALink = prismatic("d", Part.CHASSIS, point(0, 0, 0), X, Range.meters(-1, 1), A.link());
    return Stream.of(
        row("joints that name only each other are sound", () -> problems(List.of(BOTTOM, MIDDLE, TOP), List.of()), List.of()),
        row("two links with one label", () -> problems(List.of(A, sharesALink), List.of()), List.of("two links are labeled a link")),
        row("two sensors with one label", () -> problemsWith(List.of(A), encoder("reader", A, 1, false), encoder("reader", A, 2, false)), List.of("two sensors are labeled reader")),
        row("a coupling whose leader is not listed", () -> problems(List.of(A), List.of(linear(NOT_LISTED, A, 1))), List.of("a coupling's leader, stray, isn't one of the robot's joints: list it")),
        row("a coupling whose follower is not listed", () -> problems(List.of(A), List.of(linear(A, NOT_LISTED, 1))), List.of("a coupling's follower, stray, isn't one of the robot's joints: list it")),
        row("a limit switch on a joint that is not listed", () -> problemsWith(List.of(A), Sensor.Switch.limit("stop", NOT_LISTED, Sensor.Switch.Limit.End.LOWER)), List.of("sensor stop's joint, stray, isn't one of the robot's joints: list it")));
  }

  /** A joint follows one other at most, never itself, and rests where its coupling puts it. */
  static Stream<Arguments> couplings() {
    return Stream.of(
        row("a chain of followers is sound", () -> problems(List.of(A, B, C), List.of(linear(A, B, 1), linear(B, C, 1))), List.of()),
        row("a joint that follows two", () -> problems(List.of(A, B, C), List.of(linear(A, C, 1), linear(B, C, 1))), List.of("c follows two joints: it may follow one")),
        row("two joints that follow each other", () -> problems(List.of(A, B), List.of(linear(A, B, 1), linear(B, A, 1))), List.of("b follows itself, through its couplings", "a follows itself, through its couplings")),
        row("a joint that follows a ring it is not part of does not follow itself", () -> problems(List.of(A, B, C), List.of(linear(B, A, 1), linear(C, B, 1), linear(B, C, 1))), List.of("b follows itself, through its couplings", "c follows itself, through its couplings")),
        row("a follower resting a millionth off its coupling is sound", () -> problems(List.of(A, restingOff(1e-6)), List.of(linear(A, restingOff(1e-6), 1))), List.of()),
        row("the next rest further off is not", () -> problems(List.of(A, restingOff(Math.nextUp(1e-6))), List.of(linear(A, restingOff(Math.nextUp(1e-6)), 1))), List.of("off's rest (1.000e-06) isn't where its coupling puts it with a at its own (0.000)")));
  }

  /** Each joint is carried by its parent, and each follower goes where its leader takes it. */
  static Stream<Arguments> positions() {
    return Stream.of(
        row("a joint is carried by its parent and its parent's parent", () -> topAt(List.of(BOTTOM, MIDDLE, TOP), 0.25, 0.5, 0.75), new Translation3d(0.25, 0.5, 0.75)),
        row("the same with the joints listed child first", () -> topAt(List.of(TOP, MIDDLE, BOTTOM), 0.25, 0.5, 0.75), new Translation3d(0.25, 0.5, 0.75)),
        row("a follower's follower goes where the chain takes it", () -> math(List.of(A, B, C), linear(A, B, 0.5), linear(B, C, 0.5)).positions(Map.of(A, 1.0)), Map.of(A, 1.0, B, 0.5, C, 0.25)),
        row("the same with the followers listed first", () -> math(List.of(C, B, A), linear(A, B, 0.5), linear(B, C, 0.5)).positions(Map.of(A, 1.0)), Map.of(A, 1.0, B, 0.5, C, 0.25)));
  }

  // spotless:on
}
