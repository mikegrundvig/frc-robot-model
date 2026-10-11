package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.StandardRobot.*;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.RPM;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Rings of interlocks, how the robot is placed, and whether a ring deadlocks there, as tables of
 * questions and answers. The refusals a whole definition gives for a deadlock are in {@link
 * PlacesTest}; these are the pieces those are made of, and the cases a robot with stops never
 * reaches: two wheels, left and right, that turn freely and rest at 0°.
 */
class DeadlockTest {
  // spotless:off
  private static final Joint.Revolute LEFT = lightPivot("left", Optional.empty());
  private static final Joint.Revolute RIGHT = lightPivot("right", Optional.empty());
  private static final Sensor.Switch BEAM = Sensor.Switch.beam("claw beam", Part.CHASSIS);

  // Each wheel kept out of a band above where it rests while the other is at or below that band.
  private static final Interlock LEFT_BELOW_WHILE_RIGHT_BELOW = Interlock.that(LEFT).keepsOut(Range.degrees(60, 120)).when(RIGHT.atMost(Degrees.of(60)));
  private static final Interlock RIGHT_BELOW_WHILE_LEFT_BELOW = Interlock.that(RIGHT).keepsOut(Range.degrees(60, 120)).when(LEFT.atMost(Degrees.of(60)));
  // The same with the bands below where they rest.
  private static final Interlock LEFT_ABOVE_WHILE_RIGHT_ABOVE = Interlock.that(LEFT).keepsOut(Range.degrees(-120, -60)).when(RIGHT.atLeast(Degrees.of(-60)));
  private static final Interlock RIGHT_ABOVE_WHILE_LEFT_ABOVE = Interlock.that(RIGHT).keepsOut(Range.degrees(-120, -60)).when(LEFT.atLeast(Degrees.of(-60)));
  // The right wheel let go once the left has come up to the band's edge, which it may still do.
  private static final Interlock RIGHT_BELOW_WHILE_LEFT_WELL_BELOW = Interlock.that(RIGHT).keepsOut(Range.degrees(60, 120)).when(LEFT.atMost(Degrees.of(59)));

  // A ring of three on the standard robot, and interlocks that are part of no ring.
  private static final Interlock PIVOT_BY_ELEVATOR = Interlock.that(PIVOT).runsOnlyWhen(ELEVATOR.atLeast(Inches.of(10)));
  private static final Interlock ELEVATOR_BY_ARM = Interlock.that(ELEVATOR).runsOnlyWhen(ARM.atLeast(Degrees.of(10)));
  private static final Interlock ARM_BY_PIVOT = Interlock.that(ARM).runsOnlyWhen(PIVOT.atLeast(Degrees.of(10)));
  private static final Interlock PIVOT_BY_ARM = Interlock.that(PIVOT).runsOnlyWhen(ARM.atLeast(Degrees.of(10)));
  private static final Interlock ARM_BY_PIVOT_AGAIN = Interlock.that(ARM).runsOnlyWhen(PIVOT.atMost(Degrees.of(100)));
  private static final Interlock ARM_BY_ELEVATOR = Interlock.that(ARM).runsOnlyWhen(ELEVATOR.atLeast(Inches.of(10)));
  private static final Interlock ROLLER_BY_BEAM = Interlock.that(ROLLER).runsOnlyWhen(BEAM.tripped());

  private static final Condition.UpToSpeed ROLLER_SPUN_UP = ROLLER.upToSpeed(RPM.of(1500));
  private static final Joint.Revolute PUSHING_ROLLER = named(ROLLER, List.of(), List.of(Output.volts("pushing", 6), Output.rpm("slow", 1000)));
  // spotless:on

  private static final Placement AT_REST = new Placement(Map.of(), Set.of(), Map.of());

  private static List<InterlockRing> rings(Interlock... interlocks) {
    return InterlockRing.among(List.of(interlocks));
  }

  private static InterlockRing ring(Interlock... interlocks) {
    return new InterlockRing(List.of(interlocks));
  }

  /** The standard robot as a configuration puts it. */
  private static Placement placed(Condition.At... targets) {
    return Placement.of(competition().math(), Configuration.of("placed", targets));
  }

  /** The standard robot with a roller that also runs at a voltage, as a configuration puts it. */
  private static Placement placedWithAPushingRoller(Condition.At... targets) {
    RobotDefinition robot = robotOf(PIVOT, PUSHING_ROLLER);
    return Placement.of(robot.math(), Configuration.of("placed", targets));
  }

  @ParameterizedTest
  @MethodSource({"ringsFound", "readers", "placements", "deadlocks"})
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  private static Arguments answers(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  // spotless:off

  /** Every ring is found once, from the joint of it that the list restricts first. */
  static Stream<Arguments> ringsFound() {
    return Stream.of(
        answers("two joints restricted by each other are a ring",
            () -> rings(PIVOT_BY_ARM, ARM_BY_PIVOT),
            List.of(ring(PIVOT_BY_ARM, ARM_BY_PIVOT))),
        answers("a ring of three is found once, not once from each of its joints",
            () -> rings(PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM, ARM_BY_PIVOT),
            List.of(ring(PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM, ARM_BY_PIVOT))),
        answers("a ring starts from the joint restricted first in the list",
            () -> rings(ARM_BY_PIVOT, PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM),
            List.of(ring(ARM_BY_PIVOT, PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM))),
        answers("two interlocks between the same joints make two rings",
            () -> rings(PIVOT_BY_ARM, ARM_BY_PIVOT, ARM_BY_PIVOT_AGAIN),
            List.of(ring(PIVOT_BY_ARM, ARM_BY_PIVOT), ring(PIVOT_BY_ARM, ARM_BY_PIVOT_AGAIN))),
        answers("a ring of two inside a ring of three: both are found",
            () -> rings(PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM, ARM_BY_PIVOT, ARM_BY_ELEVATOR),
            List.of(ring(PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM, ARM_BY_PIVOT), ring(ELEVATOR_BY_ARM, ARM_BY_ELEVATOR))),
        answers("a chain that never comes back is no ring",
            () -> rings(PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM),
            List.of()),
        answers("an interlock on a sensor is part of no ring",
            () -> rings(ROLLER_BY_BEAM, PIVOT_BY_ARM),
            List.of()),
        answers("the joints of a ring",
            () -> ring(PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM, ARM_BY_PIVOT).joints(),
            Set.of(PIVOT, ELEVATOR, ARM)));
  }

  /** The interlock that reads a joint is the one before its own in the ring. */
  static Stream<Arguments> readers() {
    InterlockRing ring = ring(PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM, ARM_BY_PIVOT);
    return Stream.of(
        answers("the first joint is read by the last interlock", () -> ring.readerOf(0), ARM_BY_PIVOT),
        answers("the second joint is read by the first interlock", () -> ring.readerOf(1), PIVOT_BY_ELEVATOR),
        answers("the third joint is read by the second interlock", () -> ring.readerOf(2), ELEVATOR_BY_ARM));
  }

  /** Where a placement puts the joints, which it runs, and what a condition reads there. */
  static Stream<Arguments> placements() {
    return Stream.of(
        answers("a joint a configuration names is where it puts it",
            () -> placed(PIVOT.at(PIVOT_OUT)).where(PIVOT), "pivot is at 130°"),
        answers("a joint it does not name is where it rests, and said to be",
            () -> placed(PIVOT.at(PIVOT_OUT)).where(ARM), "arm is at -20°, where it rests"),
        answers("a joint it puts somewhere is set", () -> placed(PIVOT.at(PIVOT_OUT)).sets(PIVOT), true),
        answers("a joint it runs is set", () -> placed(ROLLER.at(ROLLER_INTAKING)).sets(ROLLER), true),
        answers("a joint it leaves alone is not set", () -> placed(PIVOT.at(PIVOT_OUT)).sets(ARM), false),
        answers("a joint it stops is not run", () -> placed(ROLLER.at(Output.STOPPED)).runs(ROLLER), false),
        answers("a position condition is read where the joint is put",
            () -> placed(PIVOT.at(PIVOT_OUT)).reading(PIVOT.atLeast(PIVOT_RUN)), ConditionReading.HOLDS),
        answers("a position condition is read where the joint rests",
            () -> placed(ARM.at(ARM_UP)).reading(PIVOT.atLeast(PIVOT_RUN)), ConditionReading.DOES_NOT_HOLD),
        answers("a joint that is not run has not reached a speed",
            () -> placed(PIVOT.at(PIVOT_OUT)).reading(ROLLER_SPUN_UP), ConditionReading.DOES_NOT_HOLD),
        answers("a joint run at a speed past the one to reach has reached it",
            () -> placed(ROLLER.at(ROLLER_INTAKING)).reading(ROLLER_SPUN_UP), ConditionReading.HOLDS),
        answers("a joint run at a speed short of the one to reach has not",
            () -> placedWithAPushingRoller(PUSHING_ROLLER.at(PUSHING_ROLLER.output("slow")))
                .reading(PUSHING_ROLLER.upToSpeed(RPM.of(1500))), ConditionReading.DOES_NOT_HOLD),
        answers("how fast a voltage runs a joint is not known",
            () -> placedWithAPushingRoller(PUSHING_ROLLER.at(PUSHING_ROLLER.output("pushing")))
                .reading(PUSHING_ROLLER.upToSpeed(RPM.of(1500))), ConditionReading.UNKNOWN),
        answers("what a sensor reads is not known",
            () -> placed(PIVOT.at(PIVOT_OUT)).reading(BEAM.tripped()), ConditionReading.UNKNOWN));
  }

  /** Whether a ring deadlocks where the robot is put, and how the refusal says it. */
  static Stream<Arguments> deadlocks() {
    return Stream.of(
        answers("two wheels with no stops, each kept below a band by the other being below it",
            () -> Deadlock.of(ring(LEFT_BELOW_WHILE_RIGHT_BELOW, RIGHT_BELOW_WHILE_LEFT_BELOW), AT_REST),
            Optional.of("leaves left and right holding each other by their interlocks, so neither can"
                + " free the other: a deadlock (left keeps out of 60° to 120° while right at most 60°,"
                + " so left stays at 60° or below; right keeps out of 60° to 120° while left at most"
                + " 60°, so right stays at 60° or below)")),
        answers("two wheels with no stops, each kept above a band by the other being above it",
            () -> Deadlock.of(ring(LEFT_ABOVE_WHILE_RIGHT_ABOVE, RIGHT_ABOVE_WHILE_LEFT_ABOVE), AT_REST),
            Optional.of("leaves left and right holding each other by their interlocks, so neither can"
                + " free the other: a deadlock (left keeps out of -120° to -60° while right at least"
                + " -60°, so left stays at -60° or above; right keeps out of -120° to -60° while left"
                + " at least -60°, so right stays at -60° or above)")),
        answers("a wheel that may still rise to where it frees the other is no deadlock",
            () -> Deadlock.of(ring(LEFT_BELOW_WHILE_RIGHT_BELOW, RIGHT_BELOW_WHILE_LEFT_WELL_BELOW), AT_REST),
            Optional.empty()),
        answers("a ring with a joint its interlock is not holding is no deadlock",
            () -> Deadlock.of(ring(PIVOT_BY_ARM, ARM_BY_PIVOT), placed(ARM.at(ARM_UP))),
            Optional.empty()),
        answers("three joints, each run only when the next has moved",
            () -> Deadlock.of(ring(PIVOT_BY_ELEVATOR, ELEVATOR_BY_ARM, ARM_BY_PIVOT), AT_REST),
            Optional.of("leaves pivot, elevator and arm holding each other by their interlocks, so"
                + " none can free another: a deadlock (pivot runs only when elevator at least 0.254"
                + " m, so pivot stays at 1.5°; elevator runs only when arm at least 10°, so elevator"
                + " stays at 0 m; arm runs only when pivot at least 10°, so arm stays at -20°)")));
  }

  // spotless:on
}
