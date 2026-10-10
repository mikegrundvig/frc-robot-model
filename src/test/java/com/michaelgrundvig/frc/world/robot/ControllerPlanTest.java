package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.PITCH_UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.encoder;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.linear;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.link;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.motors;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.point;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.prismatic;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.revolute;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * How the program drives a joint, worked out once: its gains tuned where given and seeded where
 * not, gravity's kind and where it pulls hardest, the one feedforward law, its followers'
 * directions, and what no motor controller could do or what's doubtful. An arm of 4 kg 0.3 m out on
 * two Krakens through 60:1, a wrist on it, an elevator, a turret. Each table row holds a plan as a
 * lambda, so the plan is worked out inside its test.
 */
class ControllerPlanTest {
  // spotless:off
  private static final Joint.Revolute ARM = revolute("arm", Part.CHASSIS, point(0.1, 0, 0.5), PITCH_UP,
      Optional.of(Range.degrees(-30, 120)), Math.toRadians(-30), link("arm link", 4, point(0.3, 0, 0), point(0.6, 0.05, 0.05)));
  private static final Joint.Revolute WRIST = revolute("wrist", ARM, point(0.6, 0, 0), PITCH_UP,
      Optional.of(Range.degrees(-90, 90)), 0, link("claw", 1, point(0.1, 0, 0), point(0.2, 0.05, 0.05)));
  private static final Joint.Prismatic ELEVATOR = prismatic("elevator", Part.CHASSIS, point(-0.2, 0, 0.1), UP,
      Range.meters(0, 1.2), link("carriage", 5, point(0, 0, 0), point(0.3, 0.3, 0.1)));
  private static final Joint.Revolute TURRET = turning("turret", Optional.of(Range.degrees(-160, 180)));

  // A turning joint with no stops, reached the short way round.
  private static final Joint.Revolute SPINNER = turning("spinner", Optional.empty());

  // An arm whose mass isn't known, so it has no ideal kG to compare a tuned one with.
  private static final Joint.Revolute MASSLESS = Joint.pivot("massless arm", Part.CHASSIS, point(0.1, 0, 0.5), PITCH_UP)
      .range(Range.degrees(-30, 120)).rest(Degrees.of(-30)).carries(Link.unseen("massless link"));

  // An intake that swings a heavy floor at half its angle: gravity one cosine fits badly.
  private static final Joint.Revolute INTAKE = revolute("intake", Part.CHASSIS, point(0.2, 0, 0.3), PITCH_UP,
      Optional.of(Range.degrees(0, 135)), 0, link("intake arm", 1, point(0.2, 0, 0), point(0.4, 0.05, 0.05)));
  private static final Joint.Revolute FLOOR = revolute("floor", Part.CHASSIS, point(0.2, 0, 0.3), PITCH_UP,
      Optional.of(Range.degrees(0, 67.5)), 0, link("floor", 6, point(0.3, 0, 0), point(0.6, 0.05, 0.05)));

  // An arm as its block says, with its motor and control, wired to one controller.
  private static final MechanismWiring.Device BLOCK_CONTROLLER = MechanismWiring.Controller.TALON_FX.can(11);
  private static final Joint.Revolute BLOCK_ARM = Joint.pivot("block arm", Part.CHASSIS, point(0.1, 0, 0.5), PITCH_UP)
      .range(Range.degrees(-30, 120)).rest(Degrees.of(-30))
      .carries(link("block arm link", 4, point(0.3, 0, 0), point(0.6, 0.05, 0.05)))
      .motor(Motor.KRAKEN_X60, Gearing.of(60), BLOCK_CONTROLLER).control(MechanismFixtures.stowed(30));

  private static final Drive ARM_MOTORS = motors(ARM, Motor.KRAKEN_X60, 2, gearing(60, 1));
  private static final Drive ARM_ON_NEOS = motors(ARM, Motor.NEO, 2, gearing(60, 1));
  private static final Drive WRIST_MOTOR = motors(WRIST, Motor.KRAKEN_X44, 1, gearing(30, 1));
  private static final Drive ELEVATOR_MOTORS = motors(ELEVATOR, Motor.KRAKEN_X60, 2, gearing(10, 1), Meters.of(0.02));
  private static final Drive TURRET_MOTOR = motors(TURRET, Motor.KRAKEN_X60, 1, gearing(50, 1));
  private static final Drive SPINNER_MOTOR = motors(SPINNER, Motor.KRAKEN_X60, 1, gearing(50, 1));
  private static final Drive MASSLESS_MOTOR = motors(MASSLESS, Motor.KRAKEN_X60, 1, gearing(60, 1));
  private static final Drive INTAKE_MOTOR = motors(INTAKE, Motor.KRAKEN_X60, 1, gearing(30, 1));

  // A wrist chained through the shoulder's axle: one motor turning both joints.
  private static final Drive WRIST_THROUGH_THE_SHOULDER = Drive.builder().label("wrist")
      .actuator(Actuator.Motors.builder().motor(Motor.KRAKEN_X44).count(1).build())
      .transmission(Transmission.Combined.builder().terms(List.of(term(WRIST, 25), term(ARM, 25))).efficiency(1).build())
      .build();

  private static final Sensor.Encoder ARM_ENCODER = encoder("arm encoder", ARM, 1, true);
  private static final Sensor.Encoder ARM_BACKWARDS = encoder("arm backwards", ARM, -1, true);
  private static final Sensor.Encoder ARM_DOUBLED = encoder("arm doubled", ARM, 2, true);
  private static final Sensor.Encoder ELEVATOR_ENCODER = encoder("elevator encoder", ELEVATOR, 0.5, true);

  private static final RobotMath MATH = definition(List.of(ARM, WRIST, ELEVATOR, TURRET, SPINNER, MASSLESS), List.of(),
      List.of(ARM_MOTORS, WRIST_MOTOR, ELEVATOR_MOTORS, TURRET_MOTOR, SPINNER_MOTOR, MASSLESS_MOTOR), List.of(ARM_ENCODER)).math();

  // Their own maths: the intake and its heavy floor, a wrist on one motor, and the block's arm; the rest would be too heavy for one robot.
  private static final RobotMath INTAKE_MATH =
      definition(List.of(INTAKE, FLOOR), List.of(linear(INTAKE, FLOOR, 0.5)), List.of(INTAKE_MOTOR), List.of()).math();
  private static final RobotMath WRIST_THROUGH_THE_SHOULDER_MATH =
      definition(List.of(ARM, WRIST), List.of(), List.of(ARM_MOTORS, WRIST_THROUGH_THE_SHOULDER), List.of()).math();
  private static final RobotMath BLOCK_MATH = definition(List.of(BLOCK_ARM), List.of(), List.of(), List.of()).math();
  // spotless:on

  /** Two TalonFXs, the second turned the other way. */
  private static final List<MechanismWiring.Device> PAIR =
      List.of(
          MechanismWiring.Controller.TALON_FX.can(11),
          MechanismWiring.Controller.TALON_FX.can(12).clockwise());

  /** A joint turning about this axis, three kilograms on a plate. */
  private static Joint.Revolute turning(String label, Optional<Range> stops) {
    return revolute(
        label,
        Part.CHASSIS,
        point(0, 0, 0.3),
        UP,
        stops,
        0,
        link(label + " plate", 3, point(0, 0, 0), point(0.3, 0.3, 0.02)));
  }

  private static Transmission.Term term(Joint joint, double ratio) {
    return Transmission.Term.builder().joint(joint).ratio(ratio).build();
  }

  private static ControllerPlan arm(MechanismControl control) {
    return ControllerPlan.of(MATH, ARM, ARM_MOTORS, PAIR, control);
  }

  private static ControllerPlan elevator(MechanismControl control) {
    return ControllerPlan.of(MATH, ELEVATOR, ELEVATOR_MOTORS, List.of(), control);
  }

  /** The plan for a joint on its motors, wired to nothing. */
  private static ControllerPlan unwired(Joint joint, Drive drive, MechanismControl control) {
    return ControllerPlan.of(MATH, joint, drive, List.of(), control);
  }

  private static MechanismControl held() {
    return MechanismControl.position(30);
  }

  private static double idealKv() {
    return MATH.idealGains(ARM).kV();
  }

  // What no motor controller could do is refused, saying what.

  @ParameterizedTest
  @MethodSource("refusals")
  void itRefusesSayingExactly(ThrowingCallable attempt, String message) {
    assertThatIllegalArgumentException().isThrownBy(attempt).withMessage(message);
  }

  // spotless:off

  static Stream<Arguments> refusals() {
    return Stream.of(
        refuses("motors the wiring doesn't name",
            () -> ControllerPlan.of(MATH, ARM, ARM_MOTORS, PAIR.subList(0, 1), held()),
            "arm is 2 motors, and its wiring names 1"),
        refuses("motors that don't move the joint",
            () -> ControllerPlan.of(MATH, WRIST, ARM_MOTORS, PAIR, held()),
            "arm doesn't move wrist"),
        refuses("a tuned kG on a joint gravity can't move",
            () -> unwired(TURRET, TURRET_MOTOR, held().withKg(0.3)),
            "turret has a tuned kG of 0.3 V, but gravity can't move it: it turns about the"
                + " vertical. Leave its kG out, or write 0"),
        refuses("a joint with no control", () -> ControllerPlan.of(MATH, ARM),
            "arm has no control"));
  }

  private static Arguments refuses(String label, ThrowingCallable attempt, String message) {
    return argumentSet(label, attempt, message);
  }

  // spotless:on

  // The one feedforward law.

  /**
   * Feedforward is friction in the direction of travel, kV at the speed, kA at the acceleration,
   * and gravity.
   */
  @ParameterizedTest
  @MethodSource("feedforwards")
  void itFeedsForward(
      Supplier<ControllerPlan> plan,
      double position,
      double velocity,
      double acceleration,
      double volts) {
    assertThat(plan.get().feedforward(position, velocity, acceleration))
        .isCloseTo(volts, within(1e-12));
  }

  // spotless:off

  static Stream<Arguments> feedforwards() {
    Supplier<ControllerPlan> everything = () -> arm(held().withKs(0.1).withKv(2).withKa(0.05).withKg(0.5, Degrees.of(0)));
    Supplier<ControllerPlan> kgAtMinus30 = () -> arm(held().withKg(0.4, Degrees.of(-30)));
    Supplier<ControllerPlan> slide = () -> elevator(MechanismControl.position(60).withKg(0.9));
    Supplier<ControllerPlan> turret = () -> unwired(TURRET, TURRET_MOTOR, MechanismControl.position(20));
    return Stream.of(
        feeds("level: all of kG",                   everything,  0,                    0,  0, 0.5),
        feeds("straight up: none",                  everything,  Math.PI / 2,          0,  0, 0),
        feeds("forward at 1 with 2 of acceleration", everything, 0,                    1,  2, 0.1 + 2 + 0.1 + 0.5),
        feeds("backward: friction turns with it",   everything,  0,                   -1,  0, -0.1 - 2 + 0.5),
        feeds("kG given at -30°: all of it there",  kgAtMinus30, Math.toRadians(-30),  0,  0, 0.4),
        feeds("kG given at -30°: none 90° on",      kgAtMinus30, Math.toRadians(60),   0,  0, 0),
        feeds("a slide near the bottom: steady",    slide,       0.1,                  0,  0, 0.9),
        feeds("a slide near the top: steady",       slide,       1.1,                  0,  0, 0.9),
        feeds("a turret: none",                     turret,      1,                    0,  0, 0));
  }

  private static Arguments feeds(String label, Supplier<ControllerPlan> plan, double position,
      double velocity, double acceleration, double volts) {
    return argumentSet(label, plan, position, velocity, acceleration, volts);
  }

  // spotless:on

  // Gains, gravity, wiring and wraps.

  @ParameterizedTest
  @MethodSource("answers")
  void itAnswers(Supplier<?> question, @Nullable Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  @ParameterizedTest
  @MethodSource("measures")
  void itMeasures(DoubleSupplier question, double answer) {
    assertThat(question.getAsDouble()).isCloseTo(answer, within(1e-9));
  }

  // spotless:off

  static Stream<Arguments> answers() {
    return Stream.of(
        // What isn't tuned is the definition's seed, from its motors, gearing and mass.
        asks("an untuned kV is the definition's", () -> arm(held()).kV(), MATH.idealGains(ARM).kV()),
        asks("an untuned kA is the definition's", () -> arm(held()).kA(), MATH.idealGains(ARM).kA()),
        asks("an untuned kG is the definition's", () -> arm(held()).gravity().kG(), MATH.idealGains(ARM).kG()),
        asks("an untuned kG is more than none", () -> arm(held()).gravity().kG() > 0, true),
        asks("a tuned kV is kept", () -> arm(held().withKv(idealKv() * 1.1).withKa(0.05)).kV(), idealKv() * 1.1),
        asks("a tuned kA is kept", () -> arm(held().withKv(idealKv() * 1.1).withKa(0.05)).kA(), 0.05),
        asks("a tuned kG with no angle is kept", () -> unwired(MASSLESS, MASSLESS_MOTOR, held().withKg(0.4)).gravity().kG(), 0.4),

        // Gravity: an arm's is one cosine, a slide's the same all along, a turret's none.
        asks("an arm's gravity is a cosine", () -> arm(held()).gravity().kind(), ControllerPlan.Gravity.Kind.COSINE),
        asks("a slide's gravity is steady", () -> elevator(held().withKg(0.9)).gravity().kind(), ControllerPlan.Gravity.Kind.STEADY),
        asks("a turret has none", () -> unwired(TURRET, TURRET_MOTOR, held()).gravity().kind(), ControllerPlan.Gravity.Kind.NONE),
        asks("a turret with a range turns on stops", () -> unwired(TURRET, TURRET_MOTOR, held()).turnsFreely(), false),
        asks("a joint with no stops turns freely", () -> unwired(SPINNER, SPINNER_MOTOR, held()).turnsFreely(), true),
        asks("an arm has stops", () -> arm(held()).turnsFreely(), false),

        // A joint is planned on its block's control and controllers and the definition's drive,
        // or on another control as the pit tunes it. Motors brake unless told to coast.
        asks("a joint is planned on its block's control", () -> ControllerPlan.of(BLOCK_MATH, BLOCK_ARM).kP(), 30.0),
        asks("a joint is planned on its block's controllers",
            () -> ControllerPlan.of(BLOCK_MATH, BLOCK_ARM).controllers(), List.of(BLOCK_CONTROLLER)),
        asks("a joint is planned on the definition's drive",
            () -> ControllerPlan.of(BLOCK_MATH, BLOCK_ARM).drive(), BLOCK_MATH.drivesOf(BLOCK_ARM).get(0)),
        asks("a joint tuned in the pit is planned on that control",
            () -> ControllerPlan.of(BLOCK_MATH, BLOCK_ARM, MechanismControl.position(45)).kP(), 45.0),
        asks("a joint tuned in the pit keeps its block's controllers",
            () -> ControllerPlan.of(BLOCK_MATH, BLOCK_ARM, MechanismControl.position(45)).controllers(), List.of(BLOCK_CONTROLLER)),
        asks("a joint tuned in the pit keeps the definition's drive",
            () -> ControllerPlan.of(BLOCK_MATH, BLOCK_ARM, MechanismControl.position(45)).drive(), BLOCK_MATH.drivesOf(BLOCK_ARM).get(0)),
        asks("motors brake unless told to coast", () -> arm(held()).brakes(), true),
        asks("motors told to coast do", () -> arm(held().coasting()).brakes(), false),

        // Motors and the controllers each runs on.
        asks("a wired pair's controllers", () -> arm(held()).controllers(), PAIR),
        asks("a wired pair is two motors", () -> arm(held()).motors(), 2),
        asks("a follower turned the other way opposes", () -> arm(held()).opposes(PAIR.get(1)), true),
        asks("a follower turned alike doesn't", () -> arm(held()).opposes(MechanismWiring.Controller.TALON_FX.can(13)), false),
        asks("Krakens wired to nothing are on TalonFXs", () -> unwired(ARM, ARM_MOTORS, held()).controllers(),
            List.of(MechanismWiring.Controller.TALON_FX.can(0), MechanismWiring.Controller.TALON_FX.can(1))),
        asks("NEOs wired to nothing are on SPARK MAXes",
            () -> unwired(ARM, ARM_ON_NEOS, held()).controllers().stream().map(MechanismWiring.Device::controller).toList(),
            List.of(MechanismWiring.Controller.SPARK_MAX, MechanismWiring.Controller.SPARK_MAX)),

        // A motor that turns other joints too is noted beside the wrist's own gravity note.
        asks("a motor that turns another joint too is noted",
            () -> ControllerPlan.of(WRIST_THROUGH_THE_SHOULDER_MATH, WRIST, WRIST_THROUGH_THE_SHOULDER, List.of(), MechanismControl.position(10))
                .notes().contains("wrist's rotor turns 25 times for each turn of arm too, which the controller reads as wrist moving: take it out in code, as a swerve module's coupling is"),
            true),

        // Where an absolute encoder wraps, and when that is a doubt.
        asks("a wrap clear of the range is no problem", () -> ControllerPlan.wrapProblem(ARM_ENCODER, 45.0 / 360 + 0.5), null),
        asks("a wrap inside the range", () -> ControllerPlan.wrapProblem(ARM_ENCODER, 1.0),
            "arm encoder wraps at 1 of its turn, inside arm's range (-30° to 120°), and -30° from where"
                + " it stows (-30°): a reading a hair past the wrap reads a turn away"),
        asks("a wrap near the range, in degrees", () -> ControllerPlan.wrapProblem(ARM_ENCODER, 0.35),
            "arm encoder wraps at 0.35 of its turn, 6° from arm's range (-30° to 120°), and 156°"
                + " from where it stows (-30°): a reading a hair past the wrap reads a turn away"),
        asks("a wrap near the range, through a ratio", () -> ControllerPlan.wrapProblem(ARM_DOUBLED, 0.68),
            "arm doubled wraps at 0.68 of its turn, 2.4° from arm's range (-30° to 120°), and"
                + " 27.6° from where it stows (-30°): a reading a hair past the wrap reads a turn"
                + " away"),
        asks("a wrap near the range, on a slide, in meters", () -> ControllerPlan.wrapProblem(ELEVATOR_ENCODER, 0.61),
            "elevator encoder wraps at 0.61 of its turn, 0.02 m from elevator's range (0 m to 1.2"
                + " m), and 0.78 m from where it stows (0 m): a reading a hair past the wrap reads a"
                + " turn away"));
  }

  static Stream<Arguments> measures() {
    return Stream.of(
        measures("an arm's mass sits on its zero", () -> arm(held()).gravity().offset(), 0),
        measures("a given kG angle, as an offset from the cosine's peak",
            () -> arm(held().withKg(0.4, Degrees.of(-30))).gravity().offset(), Math.toRadians(30)),
        measures("a drum turn, in meters", () -> elevator(held()).rotations().perRotation(), 2 * Math.PI * 0.02),
        measures("a rotor turn per meter", () -> elevator(held()).rotorRatio(), 10 / (2 * Math.PI * 0.02)),
        measures("a wrap half a turn from the range's middle", () -> ControllerPlan.wrapPoint(MATH, ARM_ENCODER), 45.0 / 360 + 0.5),
        measures("a wrap, the encoder geared to turn back", () -> ControllerPlan.wrapPoint(MATH, ARM_BACKWARDS), 45.0 / 360 + 0.5),
        measures("room each way at the wrap, less the range's half", () -> ControllerPlan.wrapMargin(ARM_ENCODER, 45.0 / 360 + 0.5), 105.0 / 360),
        measures("the same, the encoder geared to turn back", () -> ControllerPlan.wrapMargin(ARM_BACKWARDS, 45.0 / 360 + 0.5), 105.0 / 360));
  }

  private static Arguments asks(String label, Supplier<?> question, @Nullable Object answer) {
    return argumentSet(label, question, answer);
  }

  private static Arguments measures(String label, DoubleSupplier question, double answer) {
    return argumentSet(label, question, answer);
  }

  // spotless:on

  // What is doubtful is noted beside it.

  /** A note says what's doubtful, with the part of it that tells this doubt from another. */
  @ParameterizedTest
  @MethodSource("notes")
  void itNotesSaying(Supplier<ControllerPlan> plan, String part) {
    assertThat(plan.get().notes()).singleElement().asString().contains(part);
  }

  /** A tuned gain from half to twice the ideal is within it, exactly half or twice as well. */
  @ParameterizedTest
  @MethodSource("notNoted")
  void itNotesNothing(Supplier<ControllerPlan> plan) {
    assertThat(plan.get().notes()).isEmpty();
  }

  // spotless:off

  static Stream<Arguments> notes() {
    Supplier<ControllerPlan> perArmTurn = () -> arm(held().withKv(idealKv() * 60));
    Supplier<ControllerPlan> perInch = () -> elevator(MechanismControl.position(60).withKv(MATH.idealGains(ELEVATOR).kV() * 0.0254));
    return Stream.of(
        says("a gain in the rotor's frame, by the look of it", perArmTurn,
            ": the ideal per turn of arm, within 1%, entered as if per turn of its rotor, by the look of it"),
        says("a gain 4.3% off a frame is within 5%", () -> arm(held().withKv(idealKv() * 60 * 1.043)),
            ": the ideal per turn of arm, within 5%, entered as if per turn of its rotor, by the look of it"),
        says("a kG a third of the ideal", () -> arm(held().withKg(MATH.idealGains(ARM).kG() / 3)),
            "is 0.3333 times the ideal"),
        says("a kV just over twice the ideal", () -> arm(held().withKv(Math.nextUp(2 * idealKv()))),
            "arm's tuned kV, "),
        says("a kV just under half the ideal", () -> arm(held().withKv(Math.nextDown(0.5 * idealKv()))),
            "arm's tuned kV, "),
        says("a tuned kG on a slide, against its ideal", () -> elevator(MechanismControl.position(60).withKg(0.9)),
            "elevator's tuned kG, 0.9 V, is "),
        says("a slide's kV is shown per meter", perInch,
            " V/(m/s), is 0.0254 times the ideal, "),
        says("a slide's kV worked out per inch", perInch,
            ": the ideal per inch, within 1%, entered as if per meter, by the look of it"),
        says("a slide's kV worked out per turn of its drum",
            () -> elevator(MechanismControl.position(60).withKv(MATH.idealGains(ELEVATOR).kV() / (2 * Math.PI * 0.02))),
            ": the ideal per meter, within 1%, entered as if per turn of its drum, by the look of it"),
        says("a wrist on an arm that turns, whose gravity tilts",
            () -> unwired(WRIST, WRIST_MOTOR, MechanismControl.position(10)),
            "wrist rides arm, whose turning tilts its gravity: a controller's gravity term holds it"
                + " only with arm at rest, so feed RobotMath.gravityTorque at the joints'"
                + " positions as the request's feedforward"),
        says("gravity one cosine fits badly, by how much of its peak",
            () -> ControllerPlan.of(INTAKE_MATH, INTAKE, INTAKE_MOTOR, List.of(), MechanismControl.position(10)),
            "one cosine misses intake's gravity by " + Math.round(INTAKE_MATH.gravityFit(INTAKE).misfit() * 100) + "% of its peak, at "));
  }

  static Stream<Arguments> notNoted() {
    return Stream.of(
        argumentSet("untuned gains", (Supplier<ControllerPlan>) () -> arm(held())),
        argumentSet("a kV and kA near their ideal", (Supplier<ControllerPlan>) () -> arm(held().withKv(idealKv() * 1.1).withKa(0.05))),
        argumentSet("a kV exactly twice the ideal", (Supplier<ControllerPlan>) () -> arm(held().withKv(2 * idealKv()))),
        argumentSet("a kV exactly half the ideal", (Supplier<ControllerPlan>) () -> arm(held().withKv(0.5 * idealKv()))),
        argumentSet("a kG on a joint whose mass isn't known, with no ideal to compare",
            (Supplier<ControllerPlan>) () -> unwired(MASSLESS, MASSLESS_MOTOR, held().withKg(0.4))));
  }

  private static Arguments says(String label, Supplier<ControllerPlan> plan, String part) {
    return argumentSet(label, plan, part);
  }

  // spotless:on
}
