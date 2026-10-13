package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.config.Finding;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.List;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * How long a joint's motors take to bring it to a speed at their stator current limit, and the note
 * about a speed joint with no mass that says so: its rotors alone spin it up in about that long,
 * which hides a shot that feeds before its shooter is at speed.
 */
class SpinUpNoteTest {
  private static final Translation3d AXLE = new Translation3d(-0.2, 0, 0.45);
  private static final double RPM_3000 = 3000 * 2 * Math.PI / 60;
  private static final Output SHOOTING = Output.rpm("shooting", 3000);

  /** What 1.8 kg of wheels and disks on a shaft weigh, and how hard they are to turn about it. */
  private static final Link FLYWHEEL =
      Link.builder()
          .label("flywheel")
          .mass(
              MassProperties.builder()
                  .mass(Kilograms.of(1.8))
                  .cg(Translation3d.ZERO)
                  .inertia(
                      Inertia.builder()
                          .ixx(0.00232)
                          .iyy(0.00232)
                          .izz(0.00232)
                          .ixy(0)
                          .ixz(0)
                          .iyz(0)
                          .build())
                  .build())
          .build();

  /** A link with a turning inertia and no mass. */
  private static final Link INERTIA_ONLY =
      Link.builder()
          .label("flywheel")
          .mass(
              MassProperties.builder()
                  .mass(Kilograms.of(0))
                  .cg(Translation3d.ZERO)
                  .inertia(
                      Inertia.builder()
                          .ixx(0.00232)
                          .iyy(0.00232)
                          .izz(0.00232)
                          .ixy(0)
                          .ixz(0)
                          .iyz(0)
                          .build())
                  .build())
          .build();

  /** A shooter of two Krakens, straight onto a shaft, run at its outputs. */
  private static Joint.Revolute shooter(Link link, Output... outputs) {
    return Joint.roller("shooter", Part.CHASSIS, AXLE)
        .carries(link)
        .outputs(outputs)
        .motors(Motor.KRAKEN_X60.through(Gearing.of(1)).count(2))
        .control(MechanismControl.velocity(0.02));
  }

  private static RobotDefinition robot(Joint joint) {
    return MechanismFixtures.definition(List.of(joint), List.of(), List.of(), List.of());
  }

  /** The note a robot says about a joint's link having no mass, if it says one. */
  private static List<String> massNotes(Joint joint) {
    return robot(joint).findings().stream()
        .filter(finding -> finding.level() == Finding.Level.NOTE)
        .map(Finding::message)
        .filter(message -> message.contains(" has no mass"))
        .toList();
  }

  /**
   * Two Krakens at 40 A push with 2 x 40 A x 7.09 N m / 366 A = 1.55 N m through no gearing. With
   * only their rotors, 2 x 1.3e-5 kg m^2, 3000 rpm (314 rad/s) takes 2.6e-5 x 314 / 1.55 = 5 ms;
   * with a 1.8 kg flywheel of 0.00232 kg m^2 on the shaft, 0.00235 x 314 / 1.55 = 0.48 s.
   */
  @Test
  void theTimeIsTheInertiaTimesTheSpeedOverTheTorqueAtTheLimit() {
    record Row(String joint, Joint.Revolute shooter, double speed, double seconds) {}
    List<Row> rows =
        List.of(
            new Row("no flywheel", shooter(Link.unseen("flywheel"), SHOOTING), RPM_3000, 0.00527),
            new Row("flywheel", shooter(FLYWHEEL, SHOOTING), RPM_3000, 0.4756),
            new Row("half the speed", shooter(FLYWHEEL, SHOOTING), RPM_3000 / 2, 0.2378),
            new Row("backwards", shooter(FLYWHEEL, SHOOTING), -RPM_3000, 0.4756));
    for (Row row : rows) {
      OptionalDouble seconds =
          robot(row.shooter()).math().secondsFromRestAtStatorLimit(row.shooter(), row.speed());
      assertThat(seconds).as(row.joint()).isPresent();
      assertThat(seconds.getAsDouble()).as(row.joint()).isCloseTo(row.seconds(), within(0.0005));
    }
  }

  /** A joint with no control has no stator limit to work from. */
  @Test
  void aJointWithNoControlHasNoTime() {
    Joint.Revolute uncontrolled =
        Joint.roller("shooter", Part.CHASSIS, AXLE)
            .outputs(SHOOTING)
            .motors(Motor.KRAKEN_X60.through(Gearing.of(1)).count(2));

    assertThat(robot(uncontrolled).math().secondsFromRestAtStatorLimit(uncontrolled, RPM_3000))
        .isEmpty();
  }

  /** A joint no motors turn has no torque to work from. */
  @Test
  void aJointNoMotorsTurnHasNoTime() {
    Joint.Revolute unmotored = Joint.roller("shooter", Part.CHASSIS, AXLE).outputs(SHOOTING);

    assertThat(robot(unmotored).math().secondsFromRestAtStatorLimit(unmotored, RPM_3000)).isEmpty();
  }

  /**
   * The note about a link with no mass says, for a joint that is only run at speeds, how long its
   * rotors take to bring it to its fastest speed, and is as it was for any other.
   */
  @Test
  void aSpeedJointWithNoMassSaysHowFastItsRotorsSpinItUp() {
    record Row(String joint, Joint shooter, List<String> notes) {}
    List<Row> rows =
        List.of(
            new Row(
                "a speed joint with no mass and a speed",
                shooter(Link.unseen("shooter"), SHOOTING),
                List.of(
                    "shooter has no mass: shooter adds nothing to shooter's inertia, so its rotors"
                        + " alone bring it to shooting, 3000 rpm, in about 5 ms")),
            new Row(
                "its fastest speed, by its name",
                shooter(Link.unseen("shooter"), Output.rpm("idling", 500), SHOOTING),
                List.of(
                    "shooter has no mass: shooter adds nothing to shooter's inertia, so its rotors"
                        + " alone bring it to shooting, 3000 rpm, in about 5 ms")),
            new Row(
                "a speed beside a voltage",
                shooter(Link.unseen("shooter"), Output.volts("pushing", 6), SHOOTING),
                List.of(
                    "shooter has no mass: shooter adds nothing to shooter's inertia, so its rotors"
                        + " alone bring it to shooting, 3000 rpm, in about 5 ms")),
            new Row(
                "tenths of a second when it takes a tenth or more",
                Joint.roller("shooter", Part.CHASSIS, AXLE)
                    .outputs(Output.rpm("shooting", 3000))
                    .motors(Motor.KRAKEN_X60.through(Gearing.of(1)))
                    .control(MechanismControl.velocity(0.02).withStatorLimit(Amps.of(1))),
                List.of(
                    "shooter has no mass: shooter adds nothing to shooter's inertia, so its rotors"
                        + " alone bring it to shooting, 3000 rpm, in about 0.2 s")),
            new Row(
                "under a millisecond",
                shooter(Link.unseen("shooter"), Output.rpm("creeping", 50)),
                List.of(
                    "shooter has no mass: shooter adds nothing to shooter's inertia, so its rotors"
                        + " alone bring it to creeping, 50 rpm, in under a millisecond")),
            new Row("a speed joint with a mass", shooter(FLYWHEEL, SHOOTING), List.of()),
            new Row(
                "only voltages and duty cycles",
                shooter(
                    Link.unseen("shooter"),
                    Output.volts("pushing", 6),
                    Output.dutyCycle("easing", 0.2)),
                List.of("shooter has no mass: shooter adds nothing to shooter's inertia")),
            new Row(
                "its inertia and no mass",
                shooter(INERTIA_ONLY, SHOOTING),
                List.of(
                    "flywheel has no mass, only its inertia: flywheel adds only its own turning to"
                        + " shooter's inertia")),
            new Row(
                "a position joint with no mass",
                Joint.pivot("turret", Part.CHASSIS, AXLE, Vector3.UP)
                    .positions(Position.degrees("front", 0))
                    .motors(Motor.KRAKEN_X60.through(Gearing.of(50)))
                    .control(
                        MechanismControl.position(30)
                            .withHoming(MechanismControl.Homing.assumeAtRest())),
                List.of("turret has no mass: turret adds nothing to turret's inertia")));
    for (Row row : rows) {
      assertThat(massNotes(row.shooter())).as(row.joint()).isEqualTo(row.notes());
    }
  }
}
