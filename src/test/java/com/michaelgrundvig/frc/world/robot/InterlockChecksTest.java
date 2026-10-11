package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.linear;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.*;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Which interlocks the definition can judge, and what it says of one that names something that is
 * not the robot's own, as tables of questions and answers. The refusals of a whole definition are
 * in {@link PlacesTest}; these ask the checks directly, so that an interlock naming a joint the
 * robot does not have can be asked about at all.
 */
class InterlockChecksTest {
  // spotless:off
  private static final Joint.Revolute GHOST = lightPivot("ghost", Optional.of(Range.degrees(-90, 90)));
  private static final Joint.Prismatic STAGE = lightSlide("stage", Vector3.UP, Range.meters(0, 0.6));
  private static final Sensor.Switch BEAM = Sensor.Switch.beam("claw beam", Part.CHASSIS);
  private static final Position ARM_OUT = Position.degrees("out", 60);

  private static final Interlock ON_ITS_OWN = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN));
  private static final Interlock RESTRICTS_A_GHOST = Interlock.that(GHOST).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN));
  private static final Interlock ABOUT_A_GHOST = Interlock.that(ROLLER).runsOnlyWhen(GHOST.atLeast(Degrees.of(10)));
  private static final Interlock RESTRICTS_A_FOLLOWER = Interlock.that(STAGE).keepsOut(Range.meters(0.1, 0.2)).when(ELEVATOR.atLeast(ELEVATOR_TOP));
  private static final Interlock ABOUT_A_FOLLOWER = Interlock.that(ELEVATOR).keepsOut(Range.meters(0.1, 0.2)).when(STAGE.atLeast(Meters.of(0.3)));
  private static final Interlock ON_ANOTHER_JOINTS_PLACE = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(ARM_OUT));
  // The arm and the pivot, each run only when the other has moved: a deadlock where they rest.
  private static final Interlock ARM_BY_PIVOT = Interlock.that(ARM).runsOnlyWhen(PIVOT.atLeast(Degrees.of(60)));
  private static final Interlock PIVOT_BY_ARM = Interlock.that(PIVOT).runsOnlyWhen(ARM.atLeast(Degrees.of(10)));
  private static final Interlock ARM_BY_ANOTHERS_PLACE = Interlock.that(ARM).runsOnlyWhen(PIVOT.atLeast(ARM_OUT));
  private static final Interlock ON_A_SENSOR = Interlock.that(ROLLER).runsOnlyWhen(BEAM.tripped());
  private static final Interlock ON_A_SPEED = Interlock.that(PIVOT).runsOnlyWhen(ROLLER.upToSpeed(Degrees.of(90).per(org.wpilib.units.Units.Second)));
  // spotless:on

  private static RobotMath standard() {
    return competition().math();
  }

  /** The elevator with a stage that follows it. */
  private static RobotMath lifted() {
    return MechanismFixtures.definition(
            List.of(ELEVATOR, STAGE), List.of(linear(ELEVATOR, STAGE, 0.5)), List.of(), List.of())
        .math();
  }

  private static RobotMath withTheBeam() {
    return MechanismFixtures.definition(JOINTS, List.of(), DRIVES, List.of(BEAM)).math();
  }

  private static List<String> messages(List<Finding> findings) {
    return findings.stream().map(Finding::message).toList();
  }

  @ParameterizedTest
  @MethodSource({"judged", "notTheRobotsOwn"})
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments answers(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  // spotless:off

  /** Only an interlock on a position, naming the robot's own joints and places, is judged. */
  static Stream<Arguments> judged() {
    return Stream.of(
        answers("one on the robot's own joints is judged",
            () -> InterlockChecks.checkable(standard(), List.of(ON_ITS_OWN)), List.of(ON_ITS_OWN)),
        answers("one that restricts a joint the robot has not is not",
            () -> InterlockChecks.checkable(standard(), List.of(RESTRICTS_A_GHOST, ON_ITS_OWN)), List.of(ON_ITS_OWN)),
        answers("one about a joint the robot has not is not",
            () -> InterlockChecks.checkable(standard(), List.of(ABOUT_A_GHOST, ON_ITS_OWN)), List.of(ON_ITS_OWN)),
        answers("one that restricts a joint that follows another is not",
            () -> InterlockChecks.checkable(lifted(), List.of(RESTRICTS_A_FOLLOWER)), List.of()),
        answers("one about a joint that follows another is not",
            () -> InterlockChecks.checkable(lifted(), List.of(ABOUT_A_FOLLOWER)), List.of()),
        answers("one on another joint's place is not",
            () -> InterlockChecks.checkable(standard(), List.of(ON_ANOTHER_JOINTS_PLACE, ON_ITS_OWN)), List.of(ON_ITS_OWN)),
        answers("one on a sensor is not, since a sensor is not read until the robot runs",
            () -> InterlockChecks.checkable(withTheBeam(), List.of(ON_A_SENSOR, ON_ITS_OWN)), List.of(ON_ITS_OWN)),
        answers("one on a speed is not, though all it names is the robot's own",
            () -> InterlockChecks.checkable(standard(), List.of(ON_A_SPEED)), List.of()),
        answers("one on a speed names only the robot's own",
            () -> InterlockChecks.namesOnlyTheRobotsOwn(standard(), ON_A_SPEED), true),
        answers("one on a sensor names only the robot's own joints",
            () -> InterlockChecks.namesOnlyTheRobotsOwn(standard(), ON_A_SENSOR), true),
        answers("one about a joint the robot has not does not",
            () -> InterlockChecks.namesOnlyTheRobotsOwn(standard(), ABOUT_A_GHOST), false),
        answers("a configuration that leaves a ring of the robot's own in a deadlock is refused",
            () -> ConfigurationChecks.problems(standard(), List.of(STOWED), List.of(ARM_BY_PIVOT, PIVOT_BY_ARM)).size(), 1),
        answers("a ring through another joint's place is not judged: its interlock is refused instead",
            () -> ConfigurationChecks.problems(standard(), List.of(STOWED), List.of(ARM_BY_ANOTHERS_PLACE, PIVOT_BY_ARM)), List.of()),
        answers("a joint resting inside a keep-out about a ghost is not doubted",
            () -> InterlockChecks.startDoubts(standard(), List.of(
                Interlock.that(ARM).keepsOut(Range.degrees(-30, 0)).when(GHOST.atMost(Degrees.of(10))))),
            List.of()));
  }

  /** What is said of an interlock that names what is not the robot's own. */
  static Stream<Arguments> notTheRobotsOwn() {
    return Stream.of(
        answers("an interlock about a joint the robot has not",
            () -> messages(InterlockChecks.problems(standard(), List.of(ABOUT_A_GHOST))),
            List.of("interlock \"roller runs only when ghost at least 10°\"'s joint, ghost, isn't one"
                + " of the robot's joints: list it")),
        answers("an interlock that restricts a joint the robot has not",
            () -> messages(InterlockChecks.problems(standard(), List.of(RESTRICTS_A_GHOST))),
            List.of("interlock \"ghost runs only when pivot at least run\"'s joint, ghost, isn't one"
                + " of the robot's joints: list it")),
        answers("an interlock about a joint that follows another",
            () -> messages(InterlockChecks.problems(lifted(), List.of(ABOUT_A_FOLLOWER))),
            List.of("interlock \"elevator keeps out of 0.1 m to 0.2 m while stage at least 0.3 m\""
                + " names stage, which follows elevator: name elevator instead")),
        answers("an interlock on a sensor the robot has not",
            () -> messages(InterlockChecks.problems(standard(), List.of(ON_A_SENSOR))),
            List.of("interlock \"roller runs only when claw beam tripped\"'s sensor, claw beam, isn't"
                + " one of the robot's sensors: list it")),
        answers("an interlock on a sensor the robot has",
            () -> messages(InterlockChecks.problems(withTheBeam(), List.of(ON_A_SENSOR))),
            List.of()),
        answers("every interlock that names another's is reported, and nothing else is judged",
            () -> InterlockChecks.problems(standard(), List.of(ABOUT_A_GHOST, ON_A_SENSOR)).size(),
            2),
        answers("an interlock is about the joint it restricts",
            () -> InterlockChecks.problems(standard(), List.of(ABOUT_A_GHOST)).getFirst().subject(),
            "roller"),
        answers("a keep-out's edge named by another joint's place",
            () -> messages(InterlockChecks.problems(standard(), List.of(
                Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Position.of("clear", Inches.of(3))))))),
            List.of("interlock \"arm keeps out of -10° to 30° while elevator at most clear\":"
                + " elevator's clear is at 0.4 m, and this clear, at 0.0762 m, isn't one it has had:"
                + " another joint's or another robot's; give elevator's own")));
  }

  // spotless:on
}
