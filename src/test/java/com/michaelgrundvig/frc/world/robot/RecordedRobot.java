package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;

import java.util.List;
import java.util.Optional;

/**
 * The design's example robot ({@link StandardRobot}) with two sensors, for the tests of the record
 * of what a robot did, which live in another package: its joints, interlocks and configurations,
 * and where each is in the definition's lists.
 */
public final class RecordedRobot {
  private RecordedRobot() {}

  /** The pivot's limit switch at the lower end of its travel, which trips at 0°. */
  public static final Sensor.Switch PIVOT_DOWN =
      Sensor.Switch.limit("pivot down", StandardRobot.PIVOT, Sensor.Switch.Limit.End.LOWER, 0);

  /** A beam break on the chassis. */
  public static final Sensor.Switch CLAW_BEAM = Sensor.Switch.beam("claw beam", Part.CHASSIS);

  /**
   * The robot. Its joints are the pivot, the roller, the elevator and the arm, in that order. Its
   * interlocks are the roller's run-only-when and then the arm's keep-out. Its configurations are
   * "stowed", "amp" and "intaking". Its sensors are the pivot's lower limit switch and then the
   * claw's beam.
   */
  public static final RobotDefinition ROBOT =
      RobotDefinitionBuilder.builder(
              definition(
                  StandardRobot.JOINTS,
                  List.of(),
                  StandardRobot.DRIVES,
                  List.of(PIVOT_DOWN, CLAW_BEAM)))
          .configurations(List.of(StandardRobot.STOWED, StandardRobot.AMP, StandardRobot.INTAKING))
          .interlocks(List.of(StandardRobot.ROLLER_ONLY_WHEN_OUT, StandardRobot.ARM_CLEARS_BUMPER))
          .build();

  public static final int PIVOT = 0;
  public static final int ROLLER = 1;
  public static final int ELEVATOR = 2;
  public static final int ARM = 3;

  /** A joint of the robot, by its index in the definition's joints. */
  public static Joint joint(int index) {
    return ROBOT.joints().get(index);
  }

  /** A pivot with no named places, turning freely. */
  public static Joint unnamedTurret() {
    return JointRevoluteBuilder.builder(StandardRobot.lightPivot("turret", Optional.empty()))
        .build();
  }
}
