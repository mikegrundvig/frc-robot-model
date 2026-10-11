package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.doubts;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.linear;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.notes;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.stowed;
import static com.michaelgrundvig.frc.world.robot.StandardRobot.*;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.assertThatIllegalStateException;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Percent;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * Named positions and outputs, configurations and interlocks, and the definition's checks of them,
 * on the intake and stacked arm of {@link StandardRobot}. What is refused or taken is a table of
 * attempts and the exact messages; what the model answers, a table of questions and answers. A row
 * holds its question as a lambda, so the model runs inside the test and not as the table is built.
 */
class PlacesTest {
  // spotless:off
  private static final Condition ELEVATOR_LOW = ELEVATOR.atMost(ELEVATOR_CLEAR);
  private static final Condition ARM_DOWN = ARM.atMost(Degrees.of(0));
  private static final Condition.UpToSpeed SPUN = ROLLER.upToSpeed(RPM.of(1500));

  // Interlocks, named for what they keep and while what.
  private static final Interlock.KeepsOut ELEVATOR_OFF_THE_BOTTOM = Interlock.that(ELEVATOR).keepsOut(Range.meters(0, 0.3)).when(ARM_DOWN);
  private static final Interlock ELEVATOR_LOW_WHILE_ARM_DOWN = Interlock.that(ELEVATOR).keepsOut(Range.meters(0.1, 0.6)).when(ARM_DOWN);
  private static final Interlock ELEVATOR_LOW_WHILE_ARM_UP = Interlock.that(ELEVATOR).keepsOut(Range.meters(0.1, 0.6)).when(ARM.atLeast(Degrees.of(20)));
  private static final Interlock ARM_CLEARS_BUMPER_UNDER_32_IN = Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Inches.of(32)));
  private static final Interlock ROLLER_NEEDS_PIVOT_AT_125 = Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(125)));
  private static final Interlock ARM_KEEP_OUT_IN_A_LIST = Interlock.that(ARM).keepsOut(Range.degrees(40, 80)).when(ELEVATOR.atMost(Inches.of(10)));

  private static final Configuration ROLLING = Configuration.of("rolling", ROLLER.at(ROLLER_INTAKING));
  private static final Configuration LOW = Configuration.of("low", ARM.at(ARM_LEVEL));
  private static final Configuration REACHING = Configuration.of("reaching", ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_UP));

  // The arm's "out", at 60°: not the pivot's, whose out is at 130°.
  private static final Position ARM_OUT = Position.degrees("out", 60);
  private static final String ARM_OUT_ISNT_PIVOTS =
      "pivot's out is at 130°, and this out, at 60°, isn't one it has had: another joint's or"
          + " another robot's; give pivot's own";

  // A shuttle along the chassis, in, half out and out, its outer part closed while the elevator is low.
  private static final Position SHUTTLE_HALF = Position.of("half", Meters.of(0.1));
  private static final Position SHUTTLE_OUT = Position.of("out", Meters.of(0.3));
  private static final Joint.Prismatic SHUTTLE = named(lightSlide("shuttle", new Vector3(1, 0, 0), Range.meters(0, 0.3)),
      List.of(Position.of("in", Meters.of(0)), SHUTTLE_HALF, SHUTTLE_OUT), List.of());
  private static final Interlock SHUTTLE_IN_WHILE_ELEVATOR_LOW = Interlock.that(SHUTTLE).keepsOut(Range.meters(0.1, 0.3)).when(ELEVATOR.atMost(Meters.of(0.35)));

  // A stage on the elevator's lift, with a place of its own it may not have.
  private static final Joint.Prismatic STAGE = lightSlide("stage", Vector3.UP, Range.meters(0, 0.6));
  private static final Joint.Prismatic STAGE_WITH_A_PLACE = named(STAGE, List.of(Position.of("half", Inches.of(10))), List.of());

  // Rollers named at 2.5 times what their motor turns them at, and at 13 V, past the battery's 12.
  private static final Joint.Revolute TOO_FAST = named(ROLLER, List.of(), List.of(Output.rpm("intaking", 5000), Output.rpm("feeding", 1900)));
  private static final Joint.Revolute PUSHING = named(ROLLER, List.of(), List.of(Output.volts("pushing", 13), Output.dutyCycle("eject", -0.8)));
  private static final Joint.Revolute GEARED_PIVOT = PIVOT.motors(Motor.KRAKEN_X60.through(Gearing.of(25)));
  private static final Joint.Revolute COASTING = named(lightPivot("coasting", Optional.empty()), List.of(), List.of(Output.rpm("spinning", 100)));

  // A joint that rests above the low place it is at most, and a pivot that rests out, past its run.
  private static final Position HANGING_LOW = Position.degrees("low", -60);
  private static final Joint.Revolute HANGING = named(lightPivot("hanging", Optional.of(Range.degrees(-90, 0))),
      List.of(HANGING_LOW, Position.degrees("top", 0)), List.of());
  private static final Joint.Revolute PIVOT_RESTING_OUT = JointRevoluteBuilder.builder(PIVOT).rest(inRadians(120)).build();

  private static final String ROLLER_AT_FREE_SPEED =
      "roller's intaking, 2000 rpm, is 100% of its free speed, 2000 rpm: a velocity loop has"
          + " little or nothing left to hold it under load";

  // Conditions on the pivot, the arm and the elevator, asked at positions below.
  private static final Supplier<Condition.OnJoint> AT_LEAST_RUN = () -> PIVOT.atLeast(PIVOT_RUN);
  private static final Supplier<Condition.OnJoint> AT_MOST_RUN = () -> PIVOT.atMost(PIVOT_RUN);
  private static final Supplier<Condition.OnJoint> AT_LEAST_110 = () -> PIVOT.atLeast(Degrees.of(110));
  private static final Supplier<Condition.OnJoint> BETWEEN_OUT_AND_RUN = () -> PIVOT.between(PIVOT_OUT, PIVOT_RUN);
  private static final Supplier<Condition.OnJoint> BETWEEN_RUN_AND_OUT = () -> PIVOT.between(PIVOT_RUN, PIVOT_OUT);
  private static final Supplier<Condition.OnJoint> AT_OUT = () -> PIVOT.at(PIVOT_OUT);
  private static final Supplier<Condition.OnJoint> BETWEEN_45_AND_0 = () -> ARM.between(Degrees.of(45), Degrees.of(0));
  private static final Supplier<Condition.OnJoint> BETWEEN_0_AND_45 = () -> ARM.between(Degrees.of(0), Degrees.of(45));
  private static final Supplier<Condition.OnJoint> AT_MOST_10_IN = () -> ELEVATOR.atMost(Inches.of(10));
  // spotless:on

  private static double inRadians(double degrees) {
    return Math.toRadians(degrees);
  }

  private static double rpm(double rpm) {
    return RPM.of(rpm).in(RadiansPerSecond);
  }

  private static double inMeters(double inches) {
    return Inches.of(inches).in(Meters);
  }

  // The robots a row asks about are built by these, so they are built inside the row's test.

  /** A practice robot's pivot runs from 120°, its program's code unchanged. */
  private static RobotDefinition practice() {
    return competition().with(PIVOT.replaced(PIVOT_RUN, Degrees.of(120)));
  }

  /** The practice robot's own "pivot at least run", as its interlock holds it. */
  private static Condition.AtLeast practiceRun() {
    return (Condition.AtLeast) practice().interlocks().getFirst().condition();
  }

  /** A practice robot's roller that unjams at -1 V and intakes at 1800 rpm. */
  private static Joint.Revolute reValuedRoller() {
    return ROLLER.replaced(ROLLER_OUTTAKING, Volts.of(-1)).replaced(ROLLER_INTAKING, RPM.of(1800));
  }

  /** The arm with an interlock in its own block rather than the robot's list. */
  private static Joint.Revolute armWithAKeepOut() {
    return ARM.keepsOut(Range.degrees(40, 80)).when(ELEVATOR.atMost(Inches.of(10)));
  }

  /** The arm with a keep-out in its block, and the roller's interlock in the list. */
  private static RobotDefinition keepOutInABlock() {
    return RobotDefinitionBuilder.builder(robotWith(ROLLER_ONLY_WHEN_OUT))
        .joints(List.of(PIVOT, ROLLER, ELEVATOR, armWithAKeepOut()))
        .build();
  }

  /** The elevator and a follower on its lift. */
  private static RobotDefinition lifted() {
    return MechanismFixtures.definition(
        List.of(ELEVATOR, STAGE), List.of(linear(ELEVATOR, STAGE, 0.5)), List.of(), List.of());
  }

  private static RobotDefinition onShuttle(Configuration... configurations) {
    return RobotDefinitionBuilder.builder(robotOf(ELEVATOR, SHUTTLE))
        .configurations(List.of(configurations))
        .interlocks(List.of(SHUTTLE_IN_WHILE_ELEVATOR_LOW))
        .build();
  }

  /** A configuration and an interlock that both name the arm's "out" for the pivot. */
  private static RobotDefinition usingAnotherJointsOut() {
    return robot(
        List.of(Configuration.of("out", PIVOT.at(ARM_OUT), ROLLER.at(ROLLER_INTAKING))),
        List.of(Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(ARM_OUT))));
  }

  // What is refused, and what is taken.

  @ParameterizedTest
  @MethodSource({
    "nameRefusals",
    "kindRefusals",
    "edgeRefusals",
    "regionRefusals",
    "deadlockRefusals",
    "configurationRefusals",
    "practiceRefusals",
    "limitRefusals"
  })
  void itRefusesSayingExactly(ThrowingCallable attempt, String message) {
    assertThatIllegalArgumentException().isThrownBy(attempt).withMessage(message);
  }

  @ParameterizedTest
  @MethodSource({"interlockRefusals", "practiceRobotRefusals", "definitionRefusals"})
  void itRefusesSayingThis(ThrowingCallable attempt, String partOfTheMessage) {
    assertThatIllegalArgumentException()
        .isThrownBy(attempt)
        .withMessageContaining(partOfTheMessage);
  }

  @ParameterizedTest
  @MethodSource("accepted")
  void itTakes(ThrowingCallable attempt) {
    assertThatCode(attempt).doesNotThrowAnyException();
  }

  private static Arguments refuses(String label, ThrowingCallable attempt, String message) {
    return argumentSet(label, attempt, message);
  }

  private static Arguments accepts(String label, ThrowingCallable attempt) {
    return argumentSet(label, attempt);
  }

  // spotless:off

  /** A place or output is the joint's, by a name it has once, inside its stops. */
  static Stream<Arguments> nameRefusals() {
    return Stream.of(
        refuses("a position it hasn't", () -> PIVOT.position("deployed"),
            "pivot has no position named \"deployed\": it has in, run, out"),
        refuses("a position it hasn't, on a slide", () -> ELEVATOR.position("x"),
            "elevator has no position named \"x\": it has down, clear, top"),
        refuses("a position of a joint that names none", () -> ROLLER.position("in"),
            "roller has no position named \"in\": it has none"),
        refuses("an output it hasn't", () -> ROLLER.output("feeding"),
            "roller has no output named \"feeding\": it has intaking, outtaking, and stopped"),
        refuses("a position past the hard stops",
            () -> robotOf(named(PIVOT, List.of(PIVOT_IN, Position.degrees("up", 140)), List.of())),
            "RobotDefinition[test].joints: pivot's up, 140°, is outside its hard stops, 0° to"
                + " 135°: it can't get there"),
        refuses("a name that is a position and an output",
            () -> named(ELEVATOR, List.of(Position.of("top", Inches.of(1))),
                List.of(Output.of("top", MetersPerSecond.of(1)))),
            "Prismatic[elevator].outputs: it names top twice: each name is one position or output"
                + " of the joint"),
        refuses("an output named twice",
            () -> named(ROLLER, List.of(), List.of(Output.rpm("a", 1), Output.rpm("a", 2))),
            "Revolute[roller].outputs: it names a twice: each name is one position or output of"
                + " the joint"),
        refuses("an output named stopped, which every joint has",
            () -> named(ROLLER, List.of(), List.of(Output.volts("stopped", 0))),
            "Revolute[roller].outputs: it names stopped, which every joint has built in (its"
                + " motors at neutral): name it something else"));
  }

  /** A place, a speed and a range are of turning or of sliding, as the joint they're for is. */
  static Stream<Arguments> kindRefusals() {
    return Stream.of(
        refuses("a length for a joint that turns",
            () -> named(PIVOT, List.of(Position.of("deep", Inches.of(1))), List.of()),
            "Revolute[pivot].positions: deep is a length, and the joint turns: give it an angle,"
                + " Position.degrees(...)"),
        refuses("an angle for a joint that slides",
            () -> named(ELEVATOR, List.of(Position.degrees("level", 0)), List.of()),
            "Prismatic[elevator].positions: level is an angle, and the joint slides: give it a"
                + " length, Position.of(name, Inches.of(...))"),
        refuses("a speed of turning for a slide",
            () -> named(ELEVATOR, List.of(), List.of(Output.rpm("creeping", 10))),
            "Prismatic[elevator].outputs: creeping, 10 rpm, is a speed of turning, and elevator"
                + " slides: give it a speed along it, Output.of(\"creeping\","
                + " InchesPerSecond.of(...))"),
        refuses("a speed along a slide for a roller",
            () -> named(ROLLER, List.of(), List.of(Output.of("feeding", MetersPerSecond.of(3)))),
            "Revolute[roller].outputs: feeding, 3 m/s, is a speed along a slide, and roller"
                + " turns: give it a speed of turning, Output.rpm(\"feeding\", ...)"),
        refuses("a speed with no unit",
            () -> Output.builder().name("fast").kind(Output.Kind.VELOCITY).value(10).build(),
            "Output[fast].motion: a speed is of turning or of sliding: give it with its unit,"
                + " Output.rpm(name, rpm) for a joint that turns, Output.of(name,"
                + " InchesPerSecond.of(...)) for one that slides"),
        refuses("a voltage that says it turns",
            () -> Output.builder().name("push").kind(Output.Kind.VOLTAGE).value(6)
                .motion(Position.Kind.ANGLE).build(),
            "Output[push].motion: only a speed is of turning or of sliding, and this is a voltage"),
        refuses("stops in angles for a slide",
            () -> Joint.elevator("elevator", Part.CHASSIS, MechanismFixtures.point(0, 0, 0))
                .range(Range.degrees(0, 135)).rest(Inches.of(0)),
            "Prismatic[elevator].range: its range is angles, and the joint slides: give it in"
                + " lengths, Range.meters(...) or Range.of(Inches.of(...), Inches.of(...))"),
        refuses("a region in lengths for a joint that turns",
            () -> Interlock.that(PIVOT).keepsOut(Range.meters(0, 1)).when(ELEVATOR.atMost(Inches.of(10))),
            "the region is lengths, and the joint turns: give it in angles, Range.degrees(...)"));
  }

  /** An edge isn't an end of the range, a band isn't of one place, a condition isn't off its place. */
  static Stream<Arguments> edgeRefusals() {
    return Stream.of(
        refuses("at least the lowest place", () -> ELEVATOR.atLeast(ELEVATOR_DOWN),
            "down is elevator's lowest: it's at least that everywhere"),
        refuses("at least the lowest angle", () -> PIVOT.atLeast(Degrees.of(0)),
            "0° is pivot's lowest: it's at least that everywhere"),
        refuses("at most the highest angle", () -> PIVOT.atMost(Degrees.of(135)),
            "135° is pivot's highest: it's at most that everywhere"),
        refuses("at least past the range", () -> ELEVATOR.atLeast(Meters.of(2)),
            "2 m is outside elevator's range, 0 m to 1.2 m"),
        refuses("at least a hair above the lowest, which counts as the lowest",
            () -> Condition.AtLeast.builder().joint(ELEVATOR).position(Range.EDGE).build(),
            "0 m is elevator's lowest: it's at least that everywhere"),
        refuses("at most a hair below the highest, which counts as the highest",
            () -> Condition.AtMost.builder().joint(ELEVATOR).position(1.2 - Range.EDGE / 2).build(),
            "1.2 m is elevator's highest: it's at most that everywhere"),
        refuses("between two positions a hair apart, which count as one",
            () -> Condition.Between.builder().joint(ELEVATOR).from(0).to(Range.EDGE).build(),
            "a joint between 0 m and itself is at it"),
        refuses("between a place and itself", () -> PIVOT.between(PIVOT_RUN, PIVOT_RUN),
            "a joint between run and itself is at it"),
        refuses("a condition described by hand with no position",
            () -> Condition.AtLeast.builder().joint(PIVOT).position(Double.NaN).build(),
            "NaN isn't a position"),
        refuses("a condition described by hand at other than its place",
            () -> Condition.AtMost.builder().joint(PIVOT).position(inRadians(100)).place(PIVOT_RUN).build(),
            "run is 110°, not 100°"),
        refuses("a condition described by hand two hairs off its place, which shows as the place and is not it",
            () -> Condition.AtMost.builder().joint(PIVOT).position(inRadians(110) + 2 * Range.EDGE).place(PIVOT_RUN).build(),
            "run is 110°, not 110°"));
  }

  /** A keep-out region is part of the joint's range, and held by something other than itself. */
  static Stream<Arguments> regionRefusals() {
    return Stream.of(
        refuses("a region past the range",
            () -> Interlock.that(ARM).keepsOut(Range.degrees(130, 140)).when(ELEVATOR_LOW),
            "130° to 140° is outside arm's range, -30° to 120°"),
        refuses("a region that only touches the range's lower end from outside",
            () -> Interlock.that(ARM).keepsOut(Range.degrees(-40, -30)).when(ELEVATOR_LOW),
            "-40° to -30° is outside arm's range, -30° to 120°"),
        refuses("a region that only touches the range's upper end from outside",
            () -> Interlock.that(ARM).keepsOut(Range.degrees(120, 130)).when(ELEVATOR_LOW),
            "120° to 130° is outside arm's range, -30° to 120°"),
        refuses("a region of more than the range",
            () -> Interlock.that(ARM).keepsOut(Range.degrees(-40, 130)).when(ELEVATOR_LOW),
            "-40° to 130° is all of arm's range, -30° to 120°: it would have nowhere to be"),
        refuses("a region that is the range",
            () -> Interlock.that(ARM).keepsOut(Range.degrees(-30, 120)).when(ELEVATOR_LOW),
            "-30° to 120° is all of arm's range, -30° to 120°: it would have nowhere to be"),
        refuses("a joint held by where another is exactly at a place",
            () -> Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(PIVOT.at(PIVOT_OUT)),
            "pivot at out holds only with pivot exactly at out, 130°, which a measured position"
                + " never is, so the interlock would never hold: say the edge where the parts"
                + " clear, pivot.atLeast(Degrees.of(...)) or pivot.atMost(Degrees.of(...)), or a"
                + " band around out, pivot.between(Degrees.of(...), Degrees.of(...))"));
  }

  /** Joints holding each other, as the robot starts or in a configuration, can't be freed. */
  static Stream<Arguments> deadlockRefusals() {
    return Stream.of(
        refuses("two joints that each run only when the other has moved",
            () -> robotWith(Interlock.that(PIVOT).runsOnlyWhen(ARM.atLeast(ARM_UP)),
                Interlock.that(ARM).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN))),
            "RobotDefinition[test].interlocks: as the robot starts, it leaves pivot and arm"
                + " holding each other by their interlocks, so neither can free the other: a"
                + " deadlock (pivot runs only when arm at least up, so pivot stays at 1.5°; arm"
                + " runs only when pivot at least run, so arm stays at -20°)"),
        refuses("two keep-outs that hold each other as the robot starts",
            () -> robotWith(ELEVATOR_LOW_WHILE_ARM_DOWN, ARM_CLEARS_BUMPER_UNDER_32_IN),
            "RobotDefinition[test].interlocks: as the robot starts, it leaves elevator and arm"
                + " holding each other by their interlocks, so neither can free the other: a"
                + " deadlock (elevator keeps out of 0.1 m to 0.6 m while arm at most 0°, so"
                + " elevator stays between 0 m and 0.1 m; arm keeps out of -10° to 30° while"
                + " elevator at most 0.8128 m, so arm stays between -30° and -10°)"),
        refuses("a configuration that puts two joints where they hold each other",
            () -> robot(List.of(STOWED, REACHING), List.of(ARM_CLEARS_BUMPER, ELEVATOR_LOW_WHILE_ARM_UP)),
            "RobotDefinition[test].configurations: configuration reaching leaves arm and"
                + " elevator holding each other by their interlocks, so neither can free the"
                + " other: a deadlock (arm keeps out of -10° to 30° while elevator at most 0.35 m,"
                + " so arm stays between 30° and 120°; elevator keeps out of 0.1 m to 0.6 m while"
                + " arm at least 20°, so elevator stays between 0 m and 0.1 m)"),
        refuses("a configuration that puts a joint in a region it started in",
            () -> robot(List.of(Configuration.of("down", ELEVATOR.at(ELEVATOR_DOWN))),
                List.of(ELEVATOR_OFF_THE_BOTTOM, ARM_CLEARS_BUMPER_UNDER_32_IN)),
            "RobotDefinition[test].configurations: configuration down puts elevator at 0 m, but"
                + " elevator keeps out of 0 m to 0.3 m while arm at most 0°, and arm is at -20°,"
                + " where it rests"),
        refuses("a named position that every place of the other joint keeps it out of",
            () -> robotWith(Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR_LOW),
                Interlock.that(ARM).keepsOut(Range.degrees(-5, 10)).when(ELEVATOR.atLeast(ELEVATOR_CLEAR))),
            "RobotDefinition[test].interlocks: arm can never reach level, 0°: no position of"
                + " elevator opens it (arm keeps out of -10° to 30° while elevator at most"
                + " clear; arm keeps out of -5° to 10° while elevator at least clear)"),
        refuses("a shuttle put at the end a closed region reaches",
            () -> onShuttle(Configuration.of("reaching", ELEVATOR.at(ELEVATOR_DOWN), SHUTTLE.at(SHUTTLE_OUT))),
            "RobotDefinition[test].configurations: configuration reaching puts shuttle at 0.3 m,"
                + " but shuttle keeps out of 0.1 m to 0.3 m while elevator at most 0.35 m, and"
                + " elevator is at 0 m"));
  }

  /** A configuration sets each joint once, a joint of the robot's; no two are named alike. */
  static Stream<Arguments> configurationRefusals() {
    return Stream.of(
        refuses("a joint set twice", () -> Configuration.of("twice", ARM.at(ARM_LEVEL), ARM.at(ARM_UP)),
            "Configuration[twice].targets: set each joint once, not arm"),
        refuses("no joint set", () -> Configuration.of("none"),
            "Configuration[none].targets: sets no joint: name one at least"),
        refuses("two configurations of a name", () -> robot(List.of(AMP, AMP), List.of()),
            "RobotDefinition[test].configurations: two configurations are named amp"));
  }

  /** A practice robot's constants are its own; re-placing or re-valuing one it hasn't is refused. */
  static Stream<Arguments> practiceRefusals() {
    return Stream.of(
        refuses("another joint's out, re-placed", () -> PIVOT.replaced(ARM_OUT, Degrees.of(128)),
            ARM_OUT_ISNT_PIVOTS),
        refuses("a position the joint hasn't, re-placed", () -> PIVOT.replaced(ARM_UP, Degrees.of(100)),
            "pivot has no position up to re-place: it has in, run, out. Its positions are what"
                + " its code names, the same on every robot of the design"),
        refuses("an output the joint hasn't, re-valued",
            () -> ROLLER.replaced(Output.volts("feeding", 4), Volts.of(5)),
            "roller has no output feeding to re-value: it has intaking, outtaking. Its outputs"
                + " are what its code names, the same on every robot of the design"));
  }

  /** What the motors can't do, or the battery can't give, or a block can't mean. */
  static Stream<Arguments> limitRefusals() {
    return Stream.of(
        refuses("a named speed past what the motors turn it at",
            () -> MechanismFixtures.definition(List.of(PIVOT, TOO_FAST), List.of(),
                List.of(MechanismFixtures.motors(TOO_FAST, Motor.KRAKEN_X60, 1, gearing(3, 1))), List.of()),
            "RobotDefinition[test].joints: roller's intaking, 5000 rpm, is 2.5 times its free"
                + " speed, 2000 rpm: its motors can't turn it that fast; give it a speed under that"),
        refuses("a named voltage past the battery's",
            () -> MechanismFixtures.definition(List.of(PIVOT, PUSHING), List.of(),
                List.of(MechanismFixtures.motors(PUSHING, Motor.KRAKEN_X60, 1, gearing(3, 1))), List.of()),
            "RobotDefinition[test].joints: roller's pushing, 13 V, is past its motors' nominal 12"
                + " V: the battery can't give it"),
        refuses("a duty cycle past a whole battery", () -> Output.dutyCycle("flat out", 1.2),
            "Output[flat out].value: 1.2 must be a share of the battery's voltage, from -1 to 1"),
        refuses("a profile that cruises past what the motors turn the pivot at",
            () -> robotOf(GEARED_PIVOT.control(stowed(30)
                .withProfile(DegreesPerSecond.of(1800), DegreesPerSecondPerSecond.of(3600)))),
            "RobotDefinition[test].joints: pivot's profile cruises at 1800°/s, 1.25 times its free"
                + " speed, 1440°/s: its motors can't turn it that fast, so its moves would lag"
                + " their profile; give it a cruise under its free speed"),
        refuses("bands in angles for a roller",
            () -> robotOf(PIVOT, ROLLER.motors(Motor.KRAKEN_X60.through(Gearing.of(3)))
                .control(MechanismControl.velocity(0.02).withBands(Degrees.of(2), Degrees.of(4)))),
            "RobotDefinition[test].joints: roller only runs at outputs, so its bands are shares of"
                + " the speed it's asked for, not angles: withBands(Percent.of(3), Percent.of(6))"
                + " is 3% to arrive and 6% to stay"),
        refuses("bands in shares for a joint held at positions",
            () -> robotOf(GEARED_PIVOT.control(stowed(30).withBands(Percent.of(3), Percent.of(6)))),
            "RobotDefinition[test].joints: pivot is held at positions, so its bands are angles,"
                + " not shares of a speed: withBands(Degrees.of(2), Degrees.of(4)) is 2° to arrive"
                + " and 4° to stay"),
        refuses("an interlock on a joint that follows another",
            () -> RobotDefinitionBuilder.builder(lifted()).interlocks(List.of(Interlock.that(STAGE)
                .keepsOut(Range.meters(0.1, 0.2)).when(ELEVATOR.atLeast(ELEVATOR_TOP)))).build(),
            "RobotDefinition[test].interlocks: interlock \"stage keeps out of 0.1 m to 0.2 m while"
                + " elevator at least top\" names stage, which follows elevator: name elevator"
                + " instead"));
  }

  /** The same, where the message is one of several or has a clause that varies: a part pins it. */
  static Stream<Arguments> interlockRefusals() {
    return Stream.of(
        refuses("a joint held by where it is itself",
            () -> Interlock.that(PIVOT).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN)),
            "pivot can't be held by where it is itself"),
        refuses("a joint held by how fast it runs itself",
            () -> Interlock.that(ROLLER).runsOnlyWhen(ROLLER.upToSpeed(RPM.of(100))),
            "roller can't be held by how fast it runs itself"),
        refuses("a joint held by another's output",
            () -> Interlock.that(PIVOT).runsOnlyWhen(ROLLER.at(ROLLER_INTAKING)),
            "an interlock speaks of where a joint is, or how fast it runs, and intaking is"
                + " roller's output: say the speed it must reach, roller.upToSpeed(...)"),
        refuses("a joint held by where another is exactly at a place",
            () -> Interlock.that(ARM).runsOnlyWhen(ELEVATOR.at(ELEVATOR_CLEAR)),
            "elevator at clear holds only with elevator exactly at clear, 0.4 m, which a measured"
                + " position never is, so the interlock would never hold: say the edge where the"
                + " parts clear, elevator.atLeast(Inches.of(...)) or elevator.atMost(Inches.of(...)),"
                + " or a band around clear, elevator.between(Inches.of(...), Inches.of(...))"),
        refuses("two joints holding each other through a speed",
            () -> robotWith(Interlock.that(PIVOT).runsOnlyWhen(ROLLER.upToSpeed(RPM.of(100))),
                Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))),
            "as the robot starts, it leaves pivot and roller holding each other by their"
                + " interlocks, so neither can free the other: a deadlock (pivot runs only when"
                + " roller at least 100 rpm, so pivot stays at 1.5°; roller runs only when pivot"
                + " at least 110°, so roller stays still)"),
        refuses("conditions to run on places that can't all hold",
            () -> robotWith(Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(PIVOT_OUT)),
                Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atMost(PIVOT_RUN))),
            "roller can never run: no position of pivot has pivot at least out and pivot at most run"),
        refuses("conditions to run on numbers that can't all hold",
            () -> robotWith(Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(Degrees.of(130))),
                Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atMost(Degrees.of(110)))),
            "roller can never run: no position of pivot has pivot at least 130° and pivot at most 110°"),
        refuses("a block's keep-out written in the list too",
            () -> RobotDefinitionBuilder.builder(keepOutInABlock())
                .interlocks(List.of(ROLLER_ONLY_WHEN_OUT, ARM_KEEP_OUT_IN_A_LIST)).build(),
            "is written twice: once is enough, in arm's block or the robot's interlocks"),
        refuses("a block's keep-out that deadlocks with the list's",
            () -> RobotDefinitionBuilder.builder(keepOutInABlock())
                .interlocks(List.of(Interlock.that(ELEVATOR).runsOnlyWhen(ARM.atLeast(ARM_UP)))).build(),
            "as the robot starts, it leaves arm and elevator holding each other by their interlocks"));
  }

  /** A practice robot is checked as a robot is, and another joint's constant is told apart. */
  static Stream<Arguments> practiceRobotRefusals() {
    return Stream.of(
        refuses("a configuration on another joint's out", () -> usingAnotherJointsOut(),
            "configurations: configuration out: " + ARM_OUT_ISNT_PIVOTS),
        refuses("an interlock on another joint's out", () -> usingAnotherJointsOut(),
            "interlocks: interlock \"roller runs only when pivot at least out\": " + ARM_OUT_ISNT_PIVOTS),
        refuses("an output re-valued by another joint's value",
            () -> ROLLER.replaced(Output.rpm("intaking", 1000), RPM.of(1800)),
            "roller's intaking runs at 2000 rpm, and this intaking, at 1000"),
        refuses("a run re-placed past an interlock's edge",
            () -> competition().with(PIVOT.replaced(PIVOT_RUN, Degrees.of(132))),
            "configuration intaking runs roller, but roller runs only when pivot at least run"),
        refuses("an out re-placed past the hard stops",
            () -> competition().with(PIVOT.replaced(PIVOT_OUT, Degrees.of(140))),
            "pivot's out, 140°, is outside its hard stops, 0° to 135°: it can't get there"),
        refuses("stops drawn in past a place",
            () -> competition().with(ELEVATOR.range(Range.meters(0, 1))),
            "elevator's top, 1.1 m, is outside its hard stops, 0 m to 1 m: it can't get there"),
        refuses("stops drawn in past an interlock's edge",
            () -> robot(List.of(STOWED, AMP), List.of(ROLLER_NEEDS_PIVOT_AT_125, ARM_CLEARS_BUMPER))
                .with(PIVOT.replaced(PIVOT_OUT, Degrees.of(118)).range(Range.degrees(0, 120))),
            "125° is outside pivot's range, 0° to 120°"),
        refuses("a rest past the hard stops",
            () -> competition().with(PIVOT.rest(Degrees.of(140))),
            "pivot rests at 140°, outside its hard stops, 0° to 135°: it can't start there"),
        refuses("a joint the robot hasn't",
            () -> practice().own(lightPivot("wrist", Optional.of(Range.degrees(0, 90)))),
            "has no joint wrist: add it to the definition's joints"));
  }

  /** A configuration obeys the interlocks; a definition names only joints it lists; and the rest. */
  static Stream<Arguments> definitionRefusals() {
    return Stream.of(
        refuses("a configuration that runs the roller with the pivot in",
            () -> robot(List.of(ROLLING, LOW), List.of(ROLLER_ONLY_WHEN_OUT, ARM_CLEARS_BUMPER)),
            "configuration rolling runs roller, but roller runs only when pivot at least run, and"
                + " pivot is at 1.5°, where it rests"),
        refuses("a configuration that puts the arm in the bumper's band",
            () -> robot(List.of(ROLLING, LOW), List.of(ROLLER_ONLY_WHEN_OUT, ARM_CLEARS_BUMPER)),
            "configuration low puts arm at 0°, but arm keeps out of -10° to 30° while elevator at"
                + " most 0.35 m, and elevator is at 0 m, where it rests"),
        refuses("a configuration of a joint the robot hasn't",
            () -> robot(List.of(Configuration.of("elsewhere",
                named(ROLLER, List.of(), List.of(Output.volts("pushing", 3))).at(Output.volts("pushing", 3)))),
                List.of(ROLLER_ONLY_WHEN_OUT, ARM_CLEARS_BUMPER)),
            "configuration elsewhere's joint, roller, isn't one of the robot's joints: list it"),
        refuses("a follower with places of its own",
            () -> MechanismFixtures.definition(List.of(ELEVATOR, STAGE_WITH_A_PLACE),
                List.of(linear(ELEVATOR, STAGE_WITH_A_PLACE, 0.5)), List.of(), List.of()),
            "stage follows elevator, whose moves carry it: give its places to elevator, and none to"
                + " stage"),
        refuses("a blank position name", () -> Position.degrees(" ", 1),
            "Position[ ].name: must not be blank"),
        refuses("an output of no value", () -> Output.rpm("fast", Double.POSITIVE_INFINITY),
            "Output[fast].value"),
        refuses("a speed to reach of none", () -> ROLLER.upToSpeed(RPM.of(0)),
            "a speed to reach must be forward, and more than none"),
        refuses("a speed to reach backward", () -> ROLLER.upToSpeed(RPM.of(-100)),
            "must be forward"),
        refuses("an edge past the range", () -> PIVOT.atLeast(Degrees.of(150)),
            "150° is outside pivot's range, 0° to 135°"),
        refuses("a band of one place", () -> ARM.between(Degrees.of(10), Degrees.of(10)),
            "is at it"),
        refuses("a profile that cruises past what the motors turn the elevator at",
            () -> robotOf(ELEVATOR.motors(Motor.KRAKEN_X60.through(Gearing.of(10)).drum(Inches.of(1))).control(
                stowed(60).withProfile(MetersPerSecond.of(40), MetersPerSecondPerSecond.of(80)))),
            "elevator's profile cruises at 40 m/s"),
        refuses("a profile that cruises past what the motors turn the roller at, said in rpm",
            () -> robotOf(PIVOT, ROLLER.motors(Motor.KRAKEN_X60.through(Gearing.of(1))).control(
                MechanismControl.velocity(0.02).withProfile(RPM.of(7500), RotationsPerSecondPerSecond.of(200)))),
            "roller's profile cruises at 7500 rpm, 1.25 times its free speed, 6000 rpm:"));
  }

  /** Each of these is taken, though its neighbor in a table above is refused. */
  static Stream<Arguments> accepted() {
    return Stream.of(
        accepts("at least two hairs above the lowest: an edge of its own",
            () -> Condition.AtLeast.builder().joint(ELEVATOR).position(2 * Range.EDGE).build()),
        accepts("a condition described by hand a rounding step off its place: at it",
            () -> Condition.AtMost.builder().joint(PIVOT).position(Math.nextUp(inRadians(110))).place(PIVOT_RUN).build()),
        accepts("...and a hair above it",
            () -> Condition.AtLeast.builder().joint(PIVOT).position(inRadians(110) + Range.EDGE / 2).place(PIVOT_RUN).build()),
        accepts("...and a hair below it",
            () -> Condition.AtLeast.builder().joint(PIVOT).position(inRadians(110) - Range.EDGE / 2).place(PIVOT_RUN).build()),
        accepts("...and exactly a hair off it",
            () -> Condition.AtMost.builder().joint(ELEVATOR).position(Range.EDGE).place(ELEVATOR_DOWN).build()),
        accepts("between two positions two hairs apart",
            () -> Condition.Between.builder().joint(ELEVATOR).from(0).to(2 * Range.EDGE).build()),
        accepts("a speed along a slide for a slide",
            () -> named(ELEVATOR, List.of(), List.of(Output.of("creeping", MetersPerSecond.of(0.1))))),
        accepts("a configuration that stops the roller, the pivot staying in",
            () -> robot(List.of(Configuration.of("stopped", ROLLER.at(Output.STOPPED), PIVOT.at(PIVOT_IN))),
                List.of(ROLLER_ONLY_WHEN_OUT, ARM_CLEARS_BUMPER))),
        accepts("a keep-out and one kept up while the arm hangs low: each can free the other",
            () -> robot(List.of(STOWED, AMP), List.of(ARM_CLEARS_BUMPER,
                Interlock.that(ELEVATOR).keepsOut(Range.meters(0, 0.3)).when(ARM.atMost(Degrees.of(-25)))))),
        accepts("the robot starting with the arm below 20°, so the elevator may rise",
            () -> robot(List.of(STOWED), List.of(ARM_CLEARS_BUMPER, ELEVATOR_LOW_WHILE_ARM_UP))),
        accepts("a joint starting at the end a region reaches, free to leave by the other edge",
            () -> robotWith(ELEVATOR_OFF_THE_BOTTOM, ARM_CLEARS_BUMPER_UNDER_32_IN)),
        accepts("a joint starting inside a region it may only leave",
            () -> robotWith(Interlock.that(ARM).keepsOut(Range.degrees(-25, 30)).when(ELEVATOR_LOW))),
        accepts("conditions to run that can all hold",
            () -> robotWith(Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(PIVOT_OUT)),
                Interlock.that(ROLLER).runsOnlyWhen(PIVOT.between(PIVOT_RUN, PIVOT_OUT)))),
        accepts("a pivot held by how fast the roller runs", () -> robotWith(Interlock.that(PIVOT).runsOnlyWhen(SPUN))),
        accepts("a shuttle at a region's inner edge, or out with the region open",
            () -> onShuttle(Configuration.of("near", ELEVATOR.at(ELEVATOR_DOWN), SHUTTLE.at(SHUTTLE_HALF)),
                Configuration.of("out", ELEVATOR.at(ELEVATOR_CLEAR), SHUTTLE.at(SHUTTLE_OUT)))),
        accepts("a profile that cruises within what the motors turn the pivot at",
            () -> robotOf(GEARED_PIVOT.control(stowed(30)
                .withProfile(DegreesPerSecond.of(1000), DegreesPerSecondPerSecond.of(3600))))),
        accepts("an elevator held by an arm that, kept below 40°, can still pass 0°",
            () -> RobotDefinitionBuilder.builder(keepOutInABlock())
                .interlocks(List.of(Interlock.that(ELEVATOR).runsOnlyWhen(ARM.atLeast(Degrees.of(0))))).build()),
        accepts("a roller held by a beam, an arm kept out of its band while a switch is clear",
            () -> {
              Sensor.Switch beam = Sensor.Switch.beam("claw beam", Part.CHASSIS);
              Interlock.that(ROLLER).runsOnlyWhen(beam.tripped());
              Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(beam.clear());
            }),
        accepts("a joint with no range kept out of any stretch",
            () -> Interlock.that(ROLLER).keepsOut(Range.degrees(10, 20))
                .when(Sensor.Switch.beam("claw beam", Part.CHASSIS).tripped())));
  }

  // spotless:on

  // What the model answers: a table of questions and answers.

  @ParameterizedTest
  @MethodSource({"placesAndOutputs", "speeds", "ownership", "practiceRobots", "interlocksInBlocks"})
  void itAnswers(Supplier<?> question, @Nullable Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  @ParameterizedTest
  @MethodSource("measures")
  void itMeasures(DoubleSupplier question, double answer) {
    assertThat(question.getAsDouble()).isCloseTo(answer, within(1e-9));
  }

  @ParameterizedTest
  @MethodSource("descriptions")
  void itIsDescribedAs(Supplier<?> described, String description) {
    assertThat(described.get()).hasToString(description);
  }

  /** A condition on a joint holds at a position in the joint's own units: radians, or meters. */
  @ParameterizedTest
  @MethodSource({"heldAtAngles", "heldAtLengths"})
  void aConditionHoldsAt(Supplier<Condition.OnJoint> condition, double position, boolean holds) {
    assertThat(condition.get().holdsAt(position)).isEqualTo(holds);
  }

  @ParameterizedTest
  @MethodSource("doubted")
  void itDoubts(Supplier<RobotDefinition> robot, List<String> doubts) {
    assertThat(doubts(robot.get())).containsExactlyInAnyOrderElementsOf(doubts);
  }

  private static Arguments asks(String label, Supplier<?> question, @Nullable Object answer) {
    return argumentSet(label, question, answer);
  }

  private static Arguments measures(String label, DoubleSupplier question, double answer) {
    return argumentSet(label, question, answer);
  }

  private static Arguments reads(String label, Supplier<?> described, String description) {
    return argumentSet(label, described, description);
  }

  private static Arguments held(
      String label, Supplier<Condition.OnJoint> condition, double position, boolean holds) {
    return argumentSet(label, condition, position, holds);
  }

  // spotless:off

  /** What kind a place is, what an output runs, and the stopped every joint has built in. */
  static Stream<Arguments> placesAndOutputs() {
    return Stream.of(
        asks("a pivot's place is an angle", () -> PIVOT.position("run").kind(), Position.Kind.ANGLE),
        asks("an elevator's place is a length", () -> ELEVATOR.position("clear").kind(), Position.Kind.LENGTH),
        asks("a range in degrees is the range of degrees", () -> Range.of(Degrees.of(0), Degrees.of(135)), Range.degrees(0, 135)),
        asks("a range in inches is of lengths", () -> Range.of(Inches.of(0), Inches.of(10)).kind(), Position.Kind.LENGTH),
        asks("an rpm is a speed of turning", () -> ROLLER_INTAKING.motion(), Optional.of(Position.Kind.ANGLE)),
        asks("a speed along a slide is of sliding", () -> Output.of("creeping", MetersPerSecond.of(0.1)).motion(), Optional.of(Position.Kind.LENGTH)),
        asks("a voltage is of neither", () -> Output.volts("pushing", 6).motion(), Optional.empty()),
        asks("a voltage in volts is a voltage",
            () -> Output.of("hold", Volts.of(-6)), Output.builder().name("hold").kind(Output.Kind.VOLTAGE).value(-6).build()),
        asks("a share of the battery is a duty cycle",
            () -> ROLLER.replaced(ROLLER_INTAKING, Percent.of(60)).output("intaking").kind(), Output.Kind.DUTY_CYCLE),
        asks("a duty cycle runs the joint", () -> Output.dutyCycle("eject", -0.8).runs(), true),
        asks("stopped doesn't", () -> Output.STOPPED.runs(), false),
        asks("every joint has stopped built in", () -> ROLLER.output("stopped"), Output.STOPPED),
        asks("stopped is an output of every joint", () -> ROLLER.at(Output.STOPPED).isOutput(), true),
        asks("a joint at an output is an output", () -> ROLLER.at(ROLLER_INTAKING).isOutput(), true),
        asks("a joint at a position isn't", () -> PIVOT.at(PIVOT_OUT).isOutput(), false),
        asks("a configuration's joints", () -> AMP.joints(), List.of(ELEVATOR, ARM)),
        asks("a pivot held by how fast the roller runs is about the roller",
            () -> Interlock.that(PIVOT).runsOnlyWhen(SPUN).other(), Optional.of(ROLLER)),
        asks("a joint held by a switch is about no other joint",
            () -> Interlock.that(ROLLER).runsOnlyWhen(Sensor.Switch.beam("claw beam", Part.CHASSIS).tripped()).other(), Optional.empty()));
  }

  /** A speed to reach holds at that speed and above it, and is the joint's own when described by hand. */
  static Stream<Arguments> speeds() {
    return Stream.of(
        asks("a speed to reach, above it", () -> SPUN.holdsAt(rpm(1600)), true),
        asks("a speed to reach, exactly", () -> SPUN.holdsAt(SPUN.speed()), true),
        asks("a speed to reach, just under", () -> SPUN.holdsAt(Math.nextDown(SPUN.speed())), false),
        asks("a speed to reach, well under", () -> SPUN.holdsAt(rpm(1400)), false),
        asks("a speed to reach, described by hand", () -> Condition.UpToSpeed.builder().joint(ROLLER).speed(SPUN.speed()).build(), SPUN));
  }

  /** Whether a place a condition names is the joint's own, and if not why not. */
  static Stream<Arguments> ownership() {
    return Stream.of(
        asks("a band of the joint's own places is its own", () -> competition().whyNotOwn(PIVOT.between(PIVOT_RUN, PIVOT_OUT)), null),
        asks("a place of the right name at another value", () -> competition().whyNotOwn(PIVOT.atMost(Position.degrees("deployed", 120))),
            "pivot has no position deployed at 120°: it has in, run, out"),
        asks("a place of the right name of another kind", () -> competition().whyNotOwn(PIVOT.atLeast(Position.of("out", Inches.of(5)))),
            "pivot's out is an angle, and this out a length: another joint's"),
        asks("another joint's place", () -> competition().whyNotOwn(PIVOT.between(PIVOT_RUN, ARM_UP)),
            "pivot has no position up at 90°: it has in, run, out"),
        asks("another joint's out of the same name", () -> competition().whyNotOwn(PIVOT.between(PIVOT_RUN, ARM_OUT)),
            ARM_OUT_ISNT_PIVOTS),
        asks("an output the joint hasn't", () -> competition().whyNotOwn(ROLLER.at(Output.rpm("feeding", 1000))),
            "roller has no output feeding: it has intaking, outtaking, and stopped"),
        asks("an output of the right name at another speed", () -> competition().whyNotOwn(ROLLER.at(Output.rpm("intaking", 1000))),
            "roller's intaking runs at 2000 rpm, and this intaking, at 1000 rpm, isn't one it has"
                + " had: another joint's or another robot's; give roller's own"),
        asks("an output re-valued, by its old value", () -> reValuedRoller().whyNotOwn(Output.volts("outtaking", -2)),
            Optional.of("roller's outtaking runs at -1 V, and this outtaking, at -2 V, isn't one it has had:"
                + " another joint's or another robot's; give roller's own")),
        asks("an output re-valued to a share, by another share",
            () -> ROLLER.replaced(ROLLER_INTAKING, Percent.of(60)).whyNotOwn(Output.dutyCycle("intaking", 0.5)),
            Optional.of("roller's intaking runs at 60% of the battery, and this intaking, at 50% of the"
                + " battery, isn't one it has had: another joint's or another robot's; give"
                + " roller's own")));
  }

  /** A practice robot keeps the design's names at its own values. */
  static Stream<Arguments> practiceRobots() {
    return Stream.of(
        asks("the practice robot's run", () -> practice().own(PIVOT).position("run"), Position.degrees("run", 120)),
        asks("the competition robot's run", () -> competition().own(PIVOT).position("run"), PIVOT_RUN),
        asks("the practice robot's interlock is on its own run", () -> practiceRun().joint().position("run"), Position.degrees("run", 120)),
        asks("a condition the program names is the practice robot's own", () -> practice().own(PIVOT.atLeast(PIVOT_RUN)), practiceRun()),
        asks("an output the program names is the same", () -> practice().own(ROLLER.at(ROLLER_INTAKING)), ROLLER.at(ROLLER_INTAKING)),
        asks("a configuration is on the practice robot's pivot",
            () -> practice().configurations().get(2).targets().getFirst().joint().position("run"), Position.degrees("run", 120)),
        asks("the configurations are the design's", () -> practice().configurations(), List.of(STOWED, AMP, INTAKING)),
        asks("a practice pivot is the pivot, re-placed", () -> PIVOT.replaced(PIVOT_OUT, Degrees.of(125)).at(PIVOT_OUT), PIVOT.at(PIVOT_OUT)),
        asks("re-placed again, by the competition's constant",
            () -> PIVOT.replaced(PIVOT_OUT, Degrees.of(125)).replaced(PIVOT_OUT, Degrees.of(120)).position("out"), Position.of("out", Degrees.of(120))),
        asks("stops of its own",
            () -> competition().with(PIVOT.range(Range.degrees(0, 132))).joints().getFirst().range(), Optional.of(Range.degrees(0, 132))),
        asks("a re-valued roller is the roller", () -> reValuedRoller(), ROLLER),
        asks("a re-valued output keeps its name", () -> reValuedRoller().output("outtaking"), Output.volts("outtaking", -1)),
        asks("a re-valued output is owned by the old one", () -> reValuedRoller().own(ROLLER_OUTTAKING), Output.volts("outtaking", -1)),
        asks("what a re-valued roller ran at", () -> reValuedRoller().formerOutputs(), List.of(ROLLER_OUTTAKING, ROLLER_INTAKING)),
        asks("an output added and re-valued", () -> ELEVATOR.replaced(ELEVATOR_CLEAR, Meters.of(0.38))
                .outputs(Output.of("climbing", MetersPerSecond.of(0.5)))
                .replaced(Output.of("climbing", MetersPerSecond.of(0.5)), MetersPerSecond.of(0.4)).output("climbing"),
            Output.of("climbing", MetersPerSecond.of(0.4))));
  }

  /** An interlock written in a joint's block is the robot's as one in its list is. */
  static Stream<Arguments> interlocksInBlocks() {
    return Stream.of(
        asks("the arm is the same arm, with the block", () -> armWithAKeepOut(), ARM),
        asks("a block's interlock is the joint's own", () -> armWithAKeepOut().ownInterlocks(), List.of(ARM_KEEP_OUT_IN_A_LIST)),
        asks("the list has only what the list says", () -> keepOutInABlock().interlocks(), List.of(ROLLER_ONLY_WHEN_OUT)),
        asks("every interlock has the block's too", () -> keepOutInABlock().everyInterlock(), List.of(ARM_KEEP_OUT_IN_A_LIST, ROLLER_ONLY_WHEN_OUT)),
        asks("a re-tuned joint is as written, without its block's interlocks",
            () -> keepOutInABlock().with(ARM.control(stowed(20))).everyInterlock(), List.of(ROLLER_ONLY_WHEN_OUT)));
  }

  /** A place or output's value, in the units the joint runs in. */
  static Stream<Arguments> measures() {
    return Stream.of(
        measures("a pivot's run, in radians", () -> PIVOT.position("run").value(), inRadians(110)),
        measures("an elevator's clear, in meters", () -> ELEVATOR.position("clear").value(), 0.4),
        measures("a roller's intaking, in radians a second", () -> ROLLER.output("intaking").value(), rpm(2000)),
        measures("a turn a second, in radians a second", () -> Output.of("spin", RotationsPerSecond.of(1)).value(), 2 * Math.PI),
        measures("half a meter a second, as it is", () -> Output.of("slide", MetersPerSecond.of(0.5)).value(), 0.5),
        measures("60 percent of the battery, as 0.6", () -> ROLLER.replaced(ROLLER_INTAKING, Percent.of(60)).output("intaking").value(), 0.6),
        measures("an edge at a place written another way", () -> PIVOT.atLeast(Position.of("out", Degrees.of(130))).position(), inRadians(130)),
        measures("the practice robot's run", () -> practiceRun().position(), inRadians(120)),
        measures("the competition's constant, on a practice robot",
            () -> PIVOT.replaced(PIVOT_OUT, Degrees.of(125)).atLeast(PIVOT_OUT).position(), inRadians(125)),
        measures("another joint's out, kept as the code says",
            () -> PIVOT.replaced(PIVOT_OUT, Degrees.of(125)).atLeast(ARM_OUT).position(), inRadians(60)),
        measures("an output re-valued in rpm", () -> reValuedRoller().own(ROLLER_INTAKING).value(), rpm(1800)),
        measures("a place re-placed, owned by the old one",
            () -> ELEVATOR.replaced(ELEVATOR_CLEAR, Meters.of(0.38)).own(ELEVATOR_CLEAR).value(), 0.38),
        measures("a rest set on a practice robot",
            () -> competition().with(PIVOT.rest(Degrees.of(5))).joints().getFirst().restPosition(), inRadians(5)),
        measures("a rest past the out, its run still from 110°",
            () -> competition().with(PIVOT.rest(Degrees.of(132))).joints().getFirst().restPosition(), inRadians(132)));
  }

  /** What a condition, an interlock or a configuration says of itself: the words a refusal uses. */
  static Stream<Arguments> descriptions() {
    return Stream.of(
        reads("at least a place", () -> PIVOT.atLeast(PIVOT_RUN), "pivot at least run"),
        reads("at least a number", () -> PIVOT.atLeast(Degrees.of(110)), "pivot at least 110°"),
        reads("at most a place", () -> ELEVATOR.atMost(ELEVATOR_CLEAR), "elevator at most clear"),
        reads("at most a number, in meters", () -> ELEVATOR.atMost(Inches.of(10)), "elevator at most 0.254 m"),
        reads("between two places", () -> PIVOT.between(PIVOT_RUN, PIVOT_OUT), "pivot between run and out"),
        reads("between a place and another joint's", () -> PIVOT.between(PIVOT_RUN, ARM_UP), "pivot between run and up"),
        reads("at a place", () -> ARM.at(ARM_LEVEL), "arm at level"),
        reads("up to a speed", () -> SPUN, "roller at least 1500 rpm"),
        reads("the practice robot's run", () -> practiceRun(), "pivot at least run"),
        reads("a configuration", () -> AMP, "amp (elevator at clear, arm at level)"),
        reads("a roller run only when the pivot is out", () -> ROLLER_ONLY_WHEN_OUT, "roller runs only when pivot at least run"),
        reads("an arm kept out of a band", () -> ARM_CLEARS_BUMPER, "arm keeps out of -10° to 30° while elevator at most 0.35 m"),
        reads("a region short of the range's top",
            () -> Interlock.that(ARM).keepsOut(Range.degrees(-30, 119)).when(ELEVATOR_LOW), "arm keeps out of -30° to 119° while elevator at most clear"),
        reads("a region short of the range's bottom",
            () -> Interlock.that(ARM).keepsOut(Range.degrees(-29, 120)).when(ELEVATOR_LOW), "arm keeps out of -29° to 120° while elevator at most clear"));
  }

  /** Where a condition on a turning joint holds, at the edges and either side of them. */
  static Stream<Arguments> heldAtAngles() {
    return Stream.of(
        held("at least run, above it",        AT_LEAST_RUN, inRadians(120), true),
        held("at least run, on it",           AT_LEAST_RUN, inRadians(110), true),
        held("at least run, just below it",   AT_LEAST_RUN, inRadians(109), false),
        held("at most run, well below it",    AT_MOST_RUN,  inRadians(100), true),
        held("at most run, on it",            AT_MOST_RUN,  inRadians(110), true),
        held("at most run, just above it",    AT_MOST_RUN,  inRadians(111), false),
        held("at least 110°, just above",     AT_LEAST_110, inRadians(111), true),
        held("at least 110°, just below",     AT_LEAST_110, inRadians(109), false),
        held("at least 110°, of a pivot resting out, just above", () -> PIVOT_RESTING_OUT.atLeast(Degrees.of(110)), inRadians(111), true),
        held("at least 110°, of a pivot resting out, just below", () -> PIVOT_RESTING_OUT.atLeast(Degrees.of(110)), inRadians(109), false),
        held("at most a low place of a joint resting above it, below", () -> HANGING.atMost(HANGING_LOW), inRadians(-70), true),
        held("at most a low place of a joint resting above it, above", () -> HANGING.atMost(HANGING_LOW), inRadians(-50), false),
        held("at least a low place of a joint resting above it, above", () -> HANGING.atLeast(HANGING_LOW), inRadians(-50), true),
        held("between out and run, inside",   BETWEEN_OUT_AND_RUN, inRadians(120), true),
        held("between run and out, past out", BETWEEN_RUN_AND_OUT, inRadians(131), false),
        held("between 45° and 0°, at 0°",     BETWEEN_45_AND_0, inRadians(0),  true),
        held("between 45° and 0°, at 45°",    BETWEEN_45_AND_0, inRadians(45), true),
        held("between 45° and 0°, inside",    BETWEEN_45_AND_0, inRadians(30), true),
        held("between 45° and 0°, just below", BETWEEN_45_AND_0, inRadians(-1), false),
        held("between 45° and 0°, just above", BETWEEN_45_AND_0, inRadians(46), false),
        held("between 0° and 45°, at 0°",     BETWEEN_0_AND_45, inRadians(0),  true),
        held("between 0° and 45°, at 45°",    BETWEEN_0_AND_45, inRadians(45), true),
        held("between 0° and 45°, just below", BETWEEN_0_AND_45, inRadians(-1), false),
        held("between 0° and 45°, just above", BETWEEN_0_AND_45, inRadians(46), false),
        held("at out, on it",                 AT_OUT, inRadians(130), true),
        held("at out, just below",            AT_OUT, inRadians(129), false),
        held("at out, just above",            AT_OUT, inRadians(131), false));
  }

  /** The same on a slide, at lengths in meters. */
  static Stream<Arguments> heldAtLengths() {
    return Stream.of(
        held("at most 10 in, under it", AT_MOST_10_IN, inMeters(9), true),
        held("at most 10 in, over it",  AT_MOST_10_IN, inMeters(11), false));
  }

  /** What a robot doubts, and so says, though it takes it. */
  static Stream<Arguments> doubted() {
    String leavesByTheNearerEdge =
        "as the robot starts, it puts arm at -20°, but arm keeps out of -25° to 30° while"
            + " elevator at most clear, and elevator is at 0 m, where it rests: arm may only"
            + " leave it, by its edge at -25°, until that changes";
    String leavesByTheOtherEdge =
        "as the robot starts, it puts elevator at 0 m, but elevator keeps out of 0 m to 0.3 m"
            + " while arm at most 0°, and arm is at -20°, where it rests: elevator may only"
            + " leave it, by its edge at 0.3 m, until that changes";
    return Stream.of(
        doubting("a roller named at all its motors can turn it at",
            () -> robot(List.of(), List.of()), List.of(ROLLER_AT_FREE_SPEED)),
        doubting("a joint that starts inside a region it may only leave by the nearer edge",
            () -> robotWith(Interlock.that(ARM).keepsOut(Range.degrees(-25, 30)).when(ELEVATOR_LOW)),
            List.of(ROLLER_AT_FREE_SPEED, leavesByTheNearerEdge)),
        doubting("a joint that starts inside a region at its range's end, leaving by the other edge",
            () -> robotWith(ELEVATOR_OFF_THE_BOTTOM, ARM_CLEARS_BUMPER_UNDER_32_IN),
            List.of(ROLLER_AT_FREE_SPEED, leavesByTheOtherEdge)),
        doubting("a joint no motors turn, whose outputs can't be checked",
            () -> robotOf(COASTING), List.of()));
  }

  private static Arguments doubting(String label, Supplier<RobotDefinition> robot, List<String> doubts) {
    return argumentSet(label, robot, doubts);
  }

  // spotless:on

  // Two cases that are not a row of a table.

  @Test
  void aConditionOnAnOutputIsNotAPosition() {
    assertThatIllegalStateException()
        .isThrownBy(() -> ROLLER.at(ROLLER_INTAKING).holdsAt(0))
        .withMessage("roller at intaking is an output, not a position");
  }

  @Test
  void aJointNoMotorsTurnIsNotedUnchecked() {
    assertThat(notes(robotOf(COASTING)))
        .contains(
            "coasting names outputs, and no motors turn it: its speeds and voltages aren't"
                + " checked against what its motors can do");
  }
}
