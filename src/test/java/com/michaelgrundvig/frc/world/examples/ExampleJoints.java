package com.michaelgrundvig.frc.world.examples;

import static com.michaelgrundvig.frc.world.robot.MechanismWiring.Controller.TALON_FX;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.robot.Gearing;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.MechanismControl;
import com.michaelgrundvig.frc.world.robot.Motor;
import com.michaelgrundvig.frc.world.robot.Output;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Preset;
import com.michaelgrundvig.frc.world.robot.PrismaticJoint;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RevoluteJoint;

/**
 * A few joints as a robot's definition writes them: an intake that swings out with a roller on its
 * end, and an elevator with an arm on it. The model's Javadoc shows this code, so what it shows
 * compiles ({@code ExampleJointsTest} builds each).
 */
final class ExampleJoints {
  private ExampleJoints() {}

  static final Position PIVOT_IN = Position.degrees("in", 1.5);
  static final Position PIVOT_OUT = Position.degrees("out", 130);

  static final RevoluteJoint PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, Joint.PITCH_DOWN)
          .at(Meters.of(0.25), Meters.of(0), Meters.of(0.32))
          .range(Range.degrees(0, 135))
          .rest(PIVOT_IN)
          .positions(PIVOT_IN, PIVOT_OUT)
          .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(32)))
          .control(MechanismControl.position(30));

  static final Output INTAKING = Output.rpm("intaking", 2000);

  static final RevoluteJoint ROLLER =
      Joint.roller("roller", PIVOT)
          .at(Meters.of(0), Meters.of(0), Meters.of(0.35))
          .outputs(INTAKING)
          .motors(Motor.KRAKEN_X60.through(Gearing.of(3)).on(TALON_FX.can(31)))
          // @start region=runs-only-when
          // The roller runs only once the pivot is out of the frame.
          .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
          // @end region=runs-only-when
          .control(MechanismControl.velocity(0.02));

  static final Position ELEVATOR_DOWN = Position.of("down", Inches.of(0));

  static final PrismaticJoint ELEVATOR =
      Joint.elevator("elevator", Part.CHASSIS)
          .at(Meters.of(0), Meters.of(0), Meters.of(0.1))
          .range(Range.meters(0, 1.2))
          .rest(ELEVATOR_DOWN)
          .positions(ELEVATOR_DOWN)
          .motors(
              Motor.KRAKEN_X60.through(Gearing.of(5)).drum(Inches.of(0.75)).on(TALON_FX.can(20)))
          .control(MechanismControl.position(30));

  static final Position ARM_STOWED = Position.degrees("stowed", -45);

  static final RevoluteJoint ARM =
      Joint.pivot("arm", ELEVATOR, Joint.PITCH_UP)
          .at(Meters.of(0.1), Meters.of(0), Meters.of(0.3))
          .range(Range.degrees(-60, 150))
          .rest(ARM_STOWED)
          .positions(ARM_STOWED)
          .motors(Motor.KRAKEN_X60.through(Gearing.of(60)).on(TALON_FX.can(22)))
          // @start region=keeps-out
          // The arm keeps out of the bumper's band while the elevator is too low to clear it.
          .keepsOut(Range.degrees(-10, 30))
          .when(ELEVATOR.atMost(Inches.of(32)))
          // @end
          .control(MechanismControl.position(30));

  // @start region=presets
  static final Preset INTAKE = Preset.of("intake", PIVOT.at(PIVOT_OUT), ROLLER.at(INTAKING));

  static final Preset STOWED =
      Preset.of("stowed", PIVOT.at(PIVOT_IN), ELEVATOR.at(ELEVATOR_DOWN), ARM.at(ARM_STOWED));

  // @end
}
