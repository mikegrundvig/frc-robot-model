package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.PITCH_UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.UP;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.link;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.point;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.prismatic;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.revolute;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * The design's own examples, built once for the tests of places, conditions and interlocks: an
 * intake (a pivot with a roller on its end) and a stacked arm (an arm on an elevator), on the test
 * robot of {@link MechanismFixtures}. Everything here is immutable, so a test shares it freely and
 * changes a copy: {@code named(PIVOT, ...)}, {@code PIVOT.rest(...)}.
 */
final class StandardRobot {
  private StandardRobot() {}

  static final Position PIVOT_IN = Position.degrees("in", 1.5);
  static final Position PIVOT_RUN = Position.degrees("run", 110);
  static final Position PIVOT_OUT = Position.degrees("out", 130);
  static final Output ROLLER_INTAKING = Output.rpm("intaking", 2000);
  static final Output ROLLER_OUTTAKING = Output.rpm("outtaking", -500);
  static final Position ELEVATOR_DOWN = Position.of("down", Inches.of(0));
  static final Position ELEVATOR_CLEAR = Position.of("clear", Inches.of(0.4 / 0.0254));
  static final Position ELEVATOR_TOP = Position.of("top", Inches.of(1.1 / 0.0254));
  static final Position ARM_STOWED = Position.of("stowed", Degrees.of(-20));
  static final Position ARM_LEVEL = Position.degrees("level", 0);
  static final Position ARM_UP = Position.degrees("up", 90);

  // spotless:off

  /** An intake's pivot, stowed in at 1.5°, run from 110°, out at 130°, its stops at 0° and 135°. */
  static final Joint.Revolute PIVOT = named(
      revolute("pivot", Part.CHASSIS, point(0.25, 0, 0.3), PITCH_UP, Optional.of(Range.degrees(0, 135)),
          Math.toRadians(1.5), link("intake arm", 2, point(0.15, 0, 0), point(0.3, 0.4, 0.05))),
      List.of(PIVOT_IN, PIVOT_RUN, PIVOT_OUT), List.of());

  /** Its roller, through 3:1: a Kraken X60's 6000 rpm is 2000 of the roller's, all "intaking". */
  static final Joint.Revolute ROLLER = named(
      revolute("roller", PIVOT, point(0.3, 0, 0), PITCH_UP, Optional.empty(), 0,
          link("roller", 0.5, point(0, 0, 0), point(0.05, 0.4, 0.05))),
      List.of(), List.of(ROLLER_INTAKING, ROLLER_OUTTAKING));

  /** An elevator, 0 to 1.2 m: down, clear of the bumper at 0.4 m, and at the top. */
  static final Joint.Prismatic ELEVATOR = named(
      prismatic("elevator", Part.CHASSIS, point(-0.1, 0, 0.1), UP, Range.meters(0, 1.2),
          link("carriage", 3, point(0, 0, 0.1), point(0.1, 0.4, 0.2))),
      List.of(ELEVATOR_DOWN, ELEVATOR_CLEAR, ELEVATOR_TOP), List.of());

  /** An arm on it, stowed pointing down and back at -20°, level at 0°, up at 90°. */
  static final Joint.Revolute ARM = named(
      revolute("arm", ELEVATOR, point(0, 0, 0.2), PITCH_UP, Optional.of(Range.degrees(-30, 120)),
          Math.toRadians(-20), link("arm", 2, point(0.2, 0, 0), point(0.4, 0.05, 0.05))),
      List.of(ARM_STOWED, ARM_LEVEL, ARM_UP), List.of());

  // spotless:on

  static final List<Joint> JOINTS = List.of(PIVOT, ROLLER, ELEVATOR, ARM);

  static final List<Drive> DRIVES =
      List.of(
          MechanismFixtures.motors(PIVOT, Motor.KRAKEN_X60, 1, gearing(25, 1)),
          MechanismFixtures.motors(ROLLER, Motor.KRAKEN_X60, 1, gearing(3, 1)),
          MechanismFixtures.motors(ELEVATOR, Motor.KRAKEN_X60, 2, gearing(10, 1), Inches.of(1)),
          MechanismFixtures.motors(ARM, Motor.KRAKEN_X60, 1, gearing(60, 1)));

  static final Interlock ROLLER_ONLY_WHEN_OUT =
      Interlock.that(ROLLER).runsOnlyWhen(PIVOT.atLeast(PIVOT_RUN));

  /** The arm kept out of the bumper's band while the elevator is below 0.35 m, where it clears. */
  static final Interlock.KeepsOut ARM_CLEARS_BUMPER =
      Interlock.that(ARM).keepsOut(Range.degrees(-10, 30)).when(ELEVATOR.atMost(Meters.of(0.35)));

  static final Configuration STOWED =
      Configuration.of("stowed", ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_STOWED));

  static final Configuration AMP =
      Configuration.of("amp", ELEVATOR.at(ELEVATOR_CLEAR), ARM.at(ARM_LEVEL));

  static final Configuration INTAKING =
      Configuration.of("intaking", PIVOT.at(PIVOT_OUT), ROLLER.at(ROLLER_INTAKING));

  /** The robot with all of the above, as the design has it. */
  static RobotDefinition competition() {
    return robot(List.of(STOWED, AMP, INTAKING), List.of(ROLLER_ONLY_WHEN_OUT, ARM_CLEARS_BUMPER));
  }

  /** The intake and stacked arm, with these configurations and interlocks. */
  static RobotDefinition robot(List<Configuration> configurations, List<Interlock> interlocks) {
    return RobotDefinitionBuilder.builder(definition(JOINTS, List.of(), DRIVES, List.of()))
        .configurations(configurations)
        .interlocks(interlocks)
        .build();
  }

  /** The intake and stacked arm, with these interlocks and no configurations. */
  static RobotDefinition robotWith(Interlock... interlocks) {
    return robot(List.of(), Arrays.asList(interlocks));
  }

  /** The test robot with only these joints, and no drives. */
  static RobotDefinition robotOf(Joint... joints) {
    return definition(Arrays.asList(joints), List.of(), List.of(), List.of());
  }

  /** A light pivot on the chassis resting at 0°, stopped at these, or turning freely if none. */
  static Joint.Revolute lightPivot(String label, Optional<Range> stops) {
    return revolute(
        label,
        Part.CHASSIS,
        point(0, 0, 0.5),
        PITCH_UP,
        stops,
        0,
        link(label, 1, point(0.1, 0, 0), point(0.1, 0.1, 0.1)));
  }

  /** A light slide on the chassis along this axis, resting at its range's lower end. */
  static Joint.Prismatic lightSlide(String label, Vector3 axis, Range travel) {
    return prismatic(
        label,
        Part.CHASSIS,
        point(0.1, 0, 0.3),
        axis,
        travel,
        link(label, 1, point(0, 0, 0.1), point(0.1, 0.3, 0.05)));
  }

  /** A joint with these positions and outputs in place of its own. */
  static Joint.Revolute named(
      Joint.Revolute joint, List<Position> positions, List<Output> outputs) {
    return JointRevoluteBuilder.builder(joint).positions(positions).outputs(outputs).build();
  }

  static Joint.Prismatic named(
      Joint.Prismatic joint, List<Position> positions, List<Output> outputs) {
    return JointPrismaticBuilder.builder(joint).positions(positions).outputs(outputs).build();
  }
}
