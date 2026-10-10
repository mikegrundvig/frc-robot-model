package com.michaelgrundvig.frc.world.examples;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import org.wpilib.math.geometry.Translation3d;

/**
 * A few joints as a robot's definition writes them: an intake that swings out with a roller on its
 * end, and an elevator with an arm on it. The model's Javadoc shows this code, so what it shows
 * compiles ({@code ExampleJointsTest} builds each).
 */
final class ExampleJoints {
  private ExampleJoints() {}

  static final Position PIVOT_IN = Position.degrees("in", 1.5);
  static final Position PIVOT_OUT = Position.degrees("out", 130);

  static final Joint.Revolute PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
          .range(Range.degrees(0, 135))
          .rest(PIVOT_IN)
          .positions(PIVOT_IN, PIVOT_OUT)
          .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(32))
          .control(MechanismControl.position(30));

  static final Output INTAKING = Output.rpm("intaking", 2000);

  static final Joint.Revolute ROLLER =
      Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35))
          .outputs(INTAKING)
          .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(31))
          // @start region=runs-only-when
          // The roller runs only once the pivot is out of the frame.
          .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
          // @end region=runs-only-when
          .control(MechanismControl.velocity(0.02));

  static final Position ELEVATOR_DOWN = Position.of("down", Inches.of(0));

  static final Joint.Prismatic ELEVATOR =
      Joint.elevator("elevator", Part.CHASSIS, new Translation3d(0, 0, 0.1))
          .range(Range.meters(0, 1.2))
          .rest(ELEVATOR_DOWN)
          .positions(ELEVATOR_DOWN)
          .motor(Motor.KRAKEN_X60, Gearing.of(5), Inches.of(0.75), TALON_FX.can(20))
          .control(MechanismControl.position(30));

  static final Position ARM_STOWED = Position.degrees("stowed", -45);

  static final Joint.Revolute ARM =
      Joint.pivot("arm", ELEVATOR, new Translation3d(0.1, 0, 0.3), Joint.PITCH_UP)
          .range(Range.degrees(-60, 150))
          .rest(ARM_STOWED)
          .positions(ARM_STOWED)
          .motor(Motor.KRAKEN_X60, Gearing.of(60), TALON_FX.can(22))
          // @start region=keeps-out
          // The arm keeps out of the bumper's band while the elevator is too low to clear it.
          .keepsOut(Range.degrees(-10, 30))
          .when(ELEVATOR.atMost(Inches.of(32)))
          // @end
          .control(MechanismControl.position(30));

  // @start region=configurations
  static final Configuration INTAKE =
      Configuration.of("intake", PIVOT.at(PIVOT_OUT), ROLLER.at(INTAKING));

  static final Configuration STOWED =
      Configuration.of(
          "stowed", PIVOT.at(PIVOT_IN), ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_STOWED));

  // @end
}
