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
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

/**
 * How the program drives a joint, worked out once: its gains tuned where given and seeded where
 * not, gravity's kind and where it pulls hardest, the one feedforward law, its followers'
 * directions, and what no motor controller could do or what's doubtful. An arm of 4 kg 0.3 m out on
 * two Krakens through 60:1, a wrist on it, an elevator, a turret.
 */
class ControllerPlanTest {
  private static final Joint.Revolute ARM =
      revolute(
          "arm",
          Part.CHASSIS,
          point(0.1, 0, 0.5),
          PITCH_UP,
          Optional.of(Range.degrees(-30, 120)),
          Math.toRadians(-30),
          link("arm link", 4, point(0.3, 0, 0), point(0.6, 0.05, 0.05)));

  private static final Joint.Revolute WRIST =
      revolute(
          "wrist",
          ARM,
          point(0.6, 0, 0),
          PITCH_UP,
          Optional.of(Range.degrees(-90, 90)),
          0,
          link("claw", 1, point(0.1, 0, 0), point(0.2, 0.05, 0.05)));

  private static final Joint.Prismatic ELEVATOR =
      prismatic(
          "elevator",
          Part.CHASSIS,
          point(-0.2, 0, 0.1),
          UP,
          Range.meters(0, 1.2),
          link("carriage", 5, point(0, 0, 0), point(0.3, 0.3, 0.1)));

  private static final Joint.Revolute TURRET =
      revolute(
          "turret",
          Part.CHASSIS,
          point(0, 0, 0.3),
          UP,
          Optional.of(Range.degrees(-160, 180)),
          0,
          link("turret plate", 3, point(0, 0, 0), point(0.3, 0.3, 0.02)));

  private static final Drive ARM_MOTORS = motors(ARM, Motor.KRAKEN_X60, 2, gearing(60, 1));
  private static final Drive WRIST_MOTOR = motors(WRIST, Motor.KRAKEN_X44, 1, gearing(30, 1));
  private static final Drive ELEVATOR_MOTORS =
      motors(ELEVATOR, Motor.KRAKEN_X60, 2, gearing(10, 1), Meters.of(0.02));
  private static final Drive TURRET_MOTOR = motors(TURRET, Motor.KRAKEN_X60, 1, gearing(50, 1));

  private static final Sensor.Encoder ARM_ENCODER = encoder("arm encoder", ARM, 1, true);

  private static final RobotMath MATH =
      definition(
              List.of(ARM, WRIST, ELEVATOR, TURRET),
              List.of(),
              List.of(ARM_MOTORS, WRIST_MOTOR, ELEVATOR_MOTORS, TURRET_MOTOR),
              List.of(ARM_ENCODER))
          .math();

  /** Two TalonFXs, the second turned the other way. */
  private static final List<MechanismWiring.Device> PAIR =
      List.of(
          MechanismWiring.Controller.TALON_FX.can(11),
          MechanismWiring.Controller.TALON_FX.can(12).clockwise());

  private static ControllerPlan arm(MechanismControl control) {
    return ControllerPlan.of(MATH, ARM, ARM_MOTORS, PAIR, control);
  }

  /** Friction in the direction of travel, kV at the speed, kA at the acceleration, and gravity. */
  @Test
  void itsFeedforwardIsFrictionSpeedAccelerationAndGravity() {
    ControllerPlan plan =
        arm(
            MechanismControl.position(30)
                .withKs(0.1)
                .withKv(2)
                .withKa(0.05)
                .withKg(0.5, Degrees.of(0)));

    assertThat(plan.feedforward(0, 0, 0)).as("level: all of kG").isCloseTo(0.5, within(1e-12));
    assertThat(plan.feedforward(Math.PI / 2, 0, 0)).as("straight up").isCloseTo(0, within(1e-12));
    assertThat(plan.feedforward(0, 1, 2)).isCloseTo(0.1 + 2 + 0.1 + 0.5, within(1e-12));
    assertThat(plan.feedforward(0, -1, 0)).isCloseTo(-0.1 - 2 + 0.5, within(1e-12));
    assertThat(plan.kP()).isEqualTo(30);
  }

  /** What isn't tuned is the definition's seed, from its motors, gearing and mass. */
  @Test
  void untunedGainsAreTheDefinitionsSeeds() {
    RobotMath.Gains seed = MATH.idealGains(ARM);
    ControllerPlan seeded = arm(MechanismControl.position(30));
    ControllerPlan tuned = arm(MechanismControl.position(30).withKv(seed.kV() * 1.1).withKa(0.05));

    assertThat(seeded.kV()).isEqualTo(seed.kV());
    assertThat(seeded.kA()).isEqualTo(seed.kA());
    assertThat(seeded.gravity().kG()).isEqualTo(seed.kG()).isPositive();
    assertThat(seeded.gravity().kind()).isEqualTo(ControllerPlan.Gravity.Kind.COSINE);
    assertThat(seeded.gravity().offset()).as("its mass on its zero").isCloseTo(0, within(1e-12));
    assertThat(seeded.notes()).isEmpty();
    assertThat(tuned.kV()).isEqualTo(seed.kV() * 1.1);
    assertThat(tuned.kA()).isEqualTo(0.05);
    assertThat(tuned.notes()).as("near its ideal").isEmpty();
  }

  /** An arm's tuned kG pulls hardest at the angle given with it, along one cosine. */
  @Test
  void aGivenGravityAnglePullsHardest() {
    ControllerPlan plan = arm(MechanismControl.position(30).withKg(0.4, Degrees.of(-30)));

    assertThat(plan.gravity().offset()).isCloseTo(Math.toRadians(30), within(1e-12));
    assertThat(plan.feedforward(Math.toRadians(-30), 0, 0)).isCloseTo(0.4, within(1e-12));
    assertThat(plan.feedforward(Math.toRadians(60), 0, 0)).isCloseTo(0, within(1e-12));
  }

  /** A slide's gravity is the same all along; a turret's is none. */
  @Test
  void aSlidesGravityIsSteadyAndATurretsNone() {
    ControllerPlan elevator =
        ControllerPlan.of(
            MATH, ELEVATOR, ELEVATOR_MOTORS, List.of(), MechanismControl.position(60).withKg(0.9));
    ControllerPlan turret =
        ControllerPlan.of(MATH, TURRET, TURRET_MOTOR, List.of(), MechanismControl.position(20));

    assertThat(elevator.gravity().kind()).isEqualTo(ControllerPlan.Gravity.Kind.STEADY);
    assertThat(elevator.feedforward(0.1, 0, 0)).isEqualTo(0.9);
    assertThat(elevator.feedforward(1.1, 0, 0)).isEqualTo(0.9);
    assertThat(elevator.rotations().perRotation()).isCloseTo(2 * Math.PI * 0.02, within(1e-15));
    assertThat(elevator.rotorRatio()).isCloseTo(10 / (2 * Math.PI * 0.02), within(1e-9));
    assertThat(elevator.notes())
        .singleElement()
        .asString()
        .startsWith("elevator's tuned kG, 0.9 V, is ");
    assertThat(turret.gravity().kind()).isEqualTo(ControllerPlan.Gravity.Kind.NONE);
    assertThat(turret.feedforward(1, 0, 0)).isZero();
    assertThat(turret.turnsFreely()).as("it has a range").isFalse();
  }

  /**
   * A gain entered in another frame names the frames: per turn of the arm, as if per rotor turn.
   */
  @Test
  void aTunedGainInAnotherFrameIsNamed() {
    double perArmTurn = MATH.idealGains(ARM).kV() * 2 * Math.PI;

    assertThat(arm(MechanismControl.position(30).withKv(perArmTurn * 60 / (2 * Math.PI))).notes())
        .singleElement()
        .asString()
        .endsWith(
            ": the ideal per turn of arm, within 1%, entered as if per turn of its rotor, by the"
                + " look of it");
    assertThat(arm(MechanismControl.position(30).withKg(MATH.idealGains(ARM).kG() / 3)).notes())
        .singleElement()
        .asString()
        .contains("is 0.3333 times the ideal");
  }

  /** A wrist on an arm: its gravity tilts as the arm turns, which one cosine can't follow. */
  @Test
  void aWristOnATurningArmIsNoted() {
    ControllerPlan wrist =
        ControllerPlan.of(MATH, WRIST, WRIST_MOTOR, List.of(), MechanismControl.position(10));

    assertThat(wrist.notes())
        .containsExactly(
            "wrist rides arm, whose turning tilts its gravity: a controller's gravity term holds it"
                + " only with arm at rest, so feed RobotMath.gravityTorque at the joints'"
                + " positions as the request's feedforward");
  }

  /** A follower turned the other way from its leader opposes it; one turned alike doesn't. */
  @Test
  void itsFollowersDirectionsAreTheWirings() {
    ControllerPlan plan = arm(MechanismControl.position(30));

    assertThat(plan.controllers()).isEqualTo(PAIR);
    assertThat(plan.motors()).isEqualTo(2);
    assertThat(plan.opposes(PAIR.get(1))).isTrue();
    assertThat(plan.opposes(MechanismWiring.Controller.TALON_FX.can(13))).isFalse();
    assertThat(plan.brakes()).isTrue();
    assertThat(plan.softLimits()).isEmpty();
    assertThat(plan.currentLimits()).isEqualTo(MechanismControl.CurrentLimits.DEFAULTS);
    assertThat(plan.joint()).isEqualTo(ARM);
    assertThat(plan.drive()).isEqualTo(ARM_MOTORS);
  }

  /**
   * Motors its block wires to nothing are planned on the controller each runs on, which the
   * simulator runs them as and their settings are checked on: a Kraken's a TalonFX, a NEO's a SPARK
   * MAX, as many as its motors.
   */
  @Test
  void unwiredMotorsArePlannedOnTheirOwnControllers() {
    MechanismControl control = MechanismControl.position(30);
    Drive onNeos = motors(ARM, Motor.NEO, 2, gearing(60, 1));

    assertThat(ControllerPlan.of(MATH, ARM, ARM_MOTORS, List.of(), control).controllers())
        .containsExactly(
            MechanismWiring.Controller.TALON_FX.can(0), MechanismWiring.Controller.TALON_FX.can(1));
    assertThat(ControllerPlan.of(MATH, ARM, onNeos, List.of(), control).controllers())
        .extracting(MechanismWiring.Device::controller)
        .containsExactly(
            MechanismWiring.Controller.SPARK_MAX, MechanismWiring.Controller.SPARK_MAX);
  }

  /** What no motor controller could do is refused, saying what. */
  @Test
  void itRefusesWhatNoMotorControllerCouldDo() {
    MechanismControl control = MechanismControl.position(30);

    assertThatIllegalArgumentException()
        .isThrownBy(() -> ControllerPlan.of(MATH, ARM, ARM_MOTORS, PAIR.subList(0, 1), control))
        .withMessage("arm is 2 motors, and its wiring names 1");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ControllerPlan.of(MATH, WRIST, ARM_MOTORS, PAIR, control))
        .withMessage("arm doesn't move wrist");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> ControllerPlan.of(MATH, TURRET, TURRET_MOTOR, List.of(), control.withKg(0.3)))
        .withMessage(
            "turret has a tuned kG of 0.3 V, but gravity can't move it: it turns about the"
                + " vertical. Leave its kG out, or write 0");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> ControllerPlan.of(MATH, ARM))
        .withMessage("arm has no control");
  }

  /**
   * An encoder on the arm's shaft wraps half a turn from the middle of its range, as its motor
   * controller reads it: clear of it, and doubted when it wraps inside it.
   */
  @Test
  void anAbsoluteEncoderWrapsClearOfItsJointsRange() {
    double point = ControllerPlan.wrapPoint(MATH, ARM_ENCODER);

    assertThat(point).isCloseTo(45.0 / 360 + 0.5, within(1e-12));
    assertThat(ControllerPlan.wrapMargin(ARM_ENCODER, point))
        .as("75° each way, less the range's half")
        .isCloseTo(105.0 / 360, within(1e-12));
    assertThat(ControllerPlan.wrapProblem(ARM_ENCODER, point)).isNull();
    assertThat(ControllerPlan.wrapProblem(ARM_ENCODER, 1.0))
        .startsWith("arm encoder wraps at 1 of its turn, inside arm's range (-30° to 120°)");
  }

  /**
   * Its loop is the control as given: its integral and derivative gains, the soft limits its motor
   * controllers stop it at, and coasting when told nothing.
   */
  @Test
  void itsLoopAndLimitsAreTheControlAsGiven() {
    MechanismControl control =
        MechanismControl.position(30)
            .withKi(0.2)
            .withKd(1.5)
            .withSoftLimits(Range.degrees(-20, 110))
            .coasting();
    ControllerPlan plan = arm(control);

    assertThat(plan.control()).isSameAs(control);
    assertThat(plan.kI()).isEqualTo(0.2);
    assertThat(plan.kD()).isEqualTo(1.5);
    assertThat(plan.brakes()).isFalse();
    assertThat(plan.softLimits())
        .hasValueSatisfying(
            limits -> {
              assertThat(limits.lower()).isCloseTo(Math.toRadians(-20), within(1e-12));
              assertThat(limits.upper()).isCloseTo(Math.toRadians(110), within(1e-12));
            });
  }

  /**
   * A joint is planned on its block's control, or on another one as the pit tunes it, each on its
   * block's controllers and the definition's drive.
   */
  @Test
  void aJointIsPlannedOnItsBlocksControlOrOneAsThePitTunesIt() {
    MechanismWiring.Device controller = MechanismWiring.Controller.TALON_FX.can(11);
    Joint.Revolute blockArm =
        Joint.pivot("block arm", Part.CHASSIS, point(0.1, 0, 0.5), PITCH_UP)
            .range(Range.degrees(-30, 120))
            .rest(Degrees.of(-30))
            .carries(link("block arm link", 4, point(0.3, 0, 0), point(0.6, 0.05, 0.05)))
            .motor(Motor.KRAKEN_X60, Gearing.of(60), controller)
            .control(MechanismFixtures.stowed(30));
    RobotMath math = definition(List.of(blockArm), List.of(), List.of(), List.of()).math();
    Drive drive = math.drivesOf(blockArm).get(0);

    ControllerPlan its = ControllerPlan.of(math, blockArm);
    ControllerPlan tuned = ControllerPlan.of(math, blockArm, MechanismControl.position(45));

    assertThat(its.kP()).isEqualTo(30);
    assertThat(its.controllers()).containsExactly(controller);
    assertThat(its.drive()).isEqualTo(drive);
    assertThat(tuned.kP()).isEqualTo(45);
    assertThat(tuned.controllers()).containsExactly(controller);
    assertThat(tuned.drive()).isEqualTo(drive);
  }

  /** A joint with no range to its turn, which no stops hold, is reached the short way round. */
  @Test
  void aTurningJointWithNoStopsTurnsFreely() {
    Joint.Revolute spinner =
        revolute(
            "spinner",
            Part.CHASSIS,
            point(0, 0, 0.3),
            UP,
            Optional.empty(),
            0,
            link("spinner plate", 3, point(0, 0, 0), point(0.3, 0.3, 0.02)));
    Drive spinnerMotor = motors(spinner, Motor.KRAKEN_X60, 1, gearing(50, 1));
    RobotMath math =
        definition(List.of(spinner), List.of(), List.of(spinnerMotor), List.of()).math();

    assertThat(
            ControllerPlan.of(math, spinner, spinnerMotor, List.of(), MechanismControl.position(20))
                .turnsFreely())
        .isTrue();
    assertThat(arm(MechanismControl.position(30)).turnsFreely()).as("an arm has stops").isFalse();
  }

  /**
   * A drive whose motor turns other joints too, as a wrist chained through the shoulder's axle
   * does, is noted: the controller reads that turning as the joint moving.
   */
  @Test
  void aMotorThatTurnsOtherJointsToTooIsNoted() {
    Drive chain =
        Drive.builder()
            .label("wrist")
            .actuator(Actuator.Motors.builder().motor(Motor.KRAKEN_X44).count(1).build())
            .transmission(
                Transmission.Combined.builder()
                    .terms(List.of(term(WRIST, 25), term(ARM, 25)))
                    .efficiency(1)
                    .build())
            .build();
    RobotMath math =
        definition(List.of(ARM, WRIST), List.of(), List.of(ARM_MOTORS, chain), List.of()).math();

    assertThat(
            ControllerPlan.of(math, WRIST, chain, List.of(), MechanismControl.position(10)).notes())
        .contains(
            "wrist's rotor turns 25 times for each turn of arm too, which the controller reads as"
                + " wrist moving: take it out in code, as a swerve module's coupling is");
  }

  /**
   * The wrap an encoder is told is the one its motor controller sees, growing with the joint: an
   * encoder geared to turn back wraps where the forward one does, with the same room either side.
   */
  @Test
  void anEncoderGearedToTurnBackWrapsWhereAForwardOneDoes() {
    Sensor.Encoder backwards = encoder("arm backwards", ARM, -1, true);

    double point = ControllerPlan.wrapPoint(MATH, backwards);

    assertThat(point).isCloseTo(ControllerPlan.wrapPoint(MATH, ARM_ENCODER), within(1e-12));
    assertThat(ControllerPlan.wrapMargin(backwards, point))
        .isCloseTo(ControllerPlan.wrapMargin(ARM_ENCODER, point), within(1e-12))
        .isCloseTo(105.0 / 360, within(1e-12));
  }

  /**
   * A wrap too near the joint's range is doubted, saying how near it is, in the joint's units, and
   * how far from where the joint stows: for an encoder through a ratio, and on a slide.
   */
  @Test
  void aWrapNearTheRangeIsDoubtedSayingHowNearAndHowFarFromWhereItStows() {
    Sensor.Encoder doubled = encoder("arm doubled", ARM, 2, true);
    Sensor.Encoder onSlide = encoder("elevator encoder", ELEVATOR, 0.5, true);

    assertThat(ControllerPlan.wrapProblem(ARM_ENCODER, 0.35))
        .isEqualTo(
            "arm encoder wraps at 0.35 of its turn, 6° from arm's range (-30° to 120°), and 156°"
                + " from where it stows (-30°): a reading a hair past the wrap reads a turn away");
    assertThat(ControllerPlan.wrapProblem(doubled, 0.68))
        .isEqualTo(
            "arm doubled wraps at 0.68 of its turn, 2.4° from arm's range (-30° to 120°), and"
                + " 27.6° from where it stows (-30°): a reading a hair past the wrap reads a turn"
                + " away");
    assertThat(ControllerPlan.wrapProblem(onSlide, 0.61))
        .isEqualTo(
            "elevator encoder wraps at 0.61 of its turn, 0.02 m from elevator's range (0 m to 1.2"
                + " m), and 0.78 m from where it stows (0 m): a reading a hair past the wrap reads a"
                + " turn away");
  }

  /** A tuned gain from half to twice the ideal is within it, exactly half or twice as well. */
  @Test
  void aTunedGainFromHalfToTwiceTheIdealIsNotNoted() {
    double ideal = MATH.idealGains(ARM).kV();

    assertThat(arm(MechanismControl.position(30).withKv(2 * ideal)).notes()).isEmpty();
    assertThat(arm(MechanismControl.position(30).withKv(0.5 * ideal)).notes()).isEmpty();
    assertThat(arm(MechanismControl.position(30).withKv(Math.nextUp(2 * ideal))).notes())
        .singleElement()
        .asString()
        .startsWith("arm's tuned kV, ");
    assertThat(arm(MechanismControl.position(30).withKv(Math.nextDown(0.5 * ideal))).notes())
        .singleElement()
        .asString()
        .startsWith("arm's tuned kV, ");
  }

  /** A joint whose mass isn't known has no ideal kG to compare a tuned one with: none is noted. */
  @Test
  void aTunedGainWithNoIdealToCompareWithIsNotNoted() {
    Joint.Revolute massless =
        Joint.pivot("massless arm", Part.CHASSIS, point(0.1, 0, 0.5), PITCH_UP)
            .range(Range.degrees(-30, 120))
            .rest(Degrees.of(-30))
            .carries(Link.unseen("massless link"));
    Drive drive = motors(massless, Motor.KRAKEN_X60, 1, gearing(60, 1));
    RobotMath math = definition(List.of(massless), List.of(), List.of(drive), List.of()).math();

    ControllerPlan plan =
        ControllerPlan.of(
            math, massless, drive, List.of(), MechanismControl.position(30).withKg(0.4));

    assertThat(plan.gravity().kG()).isEqualTo(0.4);
    assertThat(plan.notes()).isEmpty();
  }

  /**
   * The frames a far gain may have been worked out in are named by how near the gain comes to
   * another frame's ratio, in whole percents rounded up: "within 5%" for a gain 4.3% off.
   */
  @Test
  void aFrameIsNamedWithinTheWholePercentItIsOff() {
    double perRotorTurn = MATH.idealGains(ARM).kV() * 60;

    assertThat(arm(MechanismControl.position(30).withKv(perRotorTurn * 1.043)).notes())
        .singleElement()
        .asString()
        .endsWith(
            ": the ideal per turn of arm, within 5%, entered as if per turn of its rotor, by the"
                + " look of it");
  }

  /**
   * A slide's gain is shown per meter, and its frames are the meter, the inch, a turn of its rotor
   * and a turn of its drum.
   */
  @Test
  void aSlidesFarGainNamesItsFramesAndShowsMeters() {
    double ideal = MATH.idealGains(ELEVATOR).kV();
    double perDrumTurn = 2 * Math.PI * 0.02;

    assertThat(elevator(MechanismControl.position(60).withKv(ideal * 0.0254)).notes())
        .singleElement()
        .asString()
        .startsWith("elevator's tuned kV, ")
        .contains(" V/(m/s), is 0.0254 times the ideal, ")
        .endsWith(": the ideal per inch, within 1%, entered as if per meter, by the look of it");
    assertThat(elevator(MechanismControl.position(60).withKv(ideal / perDrumTurn)).notes())
        .singleElement()
        .asString()
        .endsWith(
            ": the ideal per meter, within 1%, entered as if per turn of its drum, by the look of"
                + " it");
  }

  /**
   * An arm with a linkage that swings a heavy floor at half its angle is gravity one cosine can't
   * follow: noted, with how far it misses, in whole percents of its peak.
   */
  @Test
  void aGravityOneCosineFitsBadlyIsNotedWithHowBadly() {
    Joint.Revolute intake =
        revolute(
            "intake",
            Part.CHASSIS,
            point(0.2, 0, 0.3),
            PITCH_UP,
            Optional.of(Range.degrees(0, 135)),
            0,
            link("intake arm", 1, point(0.2, 0, 0), point(0.4, 0.05, 0.05)));
    Joint.Revolute floor =
        revolute(
            "floor",
            Part.CHASSIS,
            point(0.2, 0, 0.3),
            PITCH_UP,
            Optional.of(Range.degrees(0, 67.5)),
            0,
            link("floor", 6, point(0.3, 0, 0), point(0.6, 0.05, 0.05)));
    Drive intakeMotor = motors(intake, Motor.KRAKEN_X60, 1, gearing(30, 1));
    RobotMath math =
        definition(
                List.of(intake, floor),
                List.of(linear(intake, floor, 0.5)),
                List.of(intakeMotor),
                List.of())
            .math();

    RobotMath.GravityFit fit = math.gravityFit(intake);
    ControllerPlan plan =
        ControllerPlan.of(math, intake, intakeMotor, List.of(), MechanismControl.position(10));

    assertThat(plan.notes()).hasSize(1);
    assertThat(plan.notes().get(0))
        .startsWith(
            "one cosine misses intake's gravity by "
                + Math.round(fit.misfit() * 100)
                + "% of its peak, at ");
    assertThat(fit.misfit()).isGreaterThan(ControllerPlan.MISFIT);
  }

  private static ControllerPlan elevator(MechanismControl control) {
    return ControllerPlan.of(MATH, ELEVATOR, ELEVATOR_MOTORS, List.of(), control);
  }

  private static Transmission.Term term(Joint joint, double ratio) {
    return Transmission.Term.builder().joint(joint).ratio(ratio).build();
  }
}
