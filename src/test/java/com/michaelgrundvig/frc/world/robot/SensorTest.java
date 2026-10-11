package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.definition;
import static com.michaelgrundvig.frc.world.robot.MechanismFixtures.gearing;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.offset;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;

/**
 * Switches and rangefinders as the physical devices they are, and the conditions over them that say
 * what they mean.
 */
class SensorTest {
  private static final Joint.Revolute PIVOT =
      Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.2, 0, 0.3), Joint.PITCH_UP)
          .range(Range.degrees(0, 120))
          .rest(Degrees.of(0))
          .carries(Link.unseen("arm"));

  private static final Joint.Revolute ROLLER =
      Joint.roller("roller", PIVOT, new Translation3d(0.3, 0, 0));

  /**
   * A limit switch at the bottom of the pivot's travel trips at or below its position; the top one
   * at or above; one whose position isn't known never says it's tripped, since nothing says where.
   */
  @Test
  void aLimitSwitchTripsAtItsEndOfTheTravel() {
    Sensor.Switch bottom =
        Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER, 0);
    Sensor.Switch top =
        Sensor.Switch.limit("pivot up", PIVOT, Sensor.Switch.Limit.End.UPPER, Math.toRadians(120));
    Sensor.Switch.Limit unplaced =
        (Sensor.Switch.Limit)
            Sensor.Switch.limit("pivot somewhere", PIVOT, Sensor.Switch.Limit.End.LOWER).kind();

    Sensor.Switch.Limit lower = (Sensor.Switch.Limit) bottom.kind();
    Sensor.Switch.Limit upper = (Sensor.Switch.Limit) top.kind();
    assertThat(lower.trippedAt(0)).isTrue();
    assertThat(lower.trippedAt(-0.01)).isTrue();
    assertThat(lower.trippedAt(0.01)).isFalse();
    assertThat(upper.trippedAt(Math.toRadians(120))).as("at its position exactly").isTrue();
    assertThat(upper.trippedAt(Math.toRadians(121))).isTrue();
    assertThat(upper.trippedAt(Math.toRadians(119))).isFalse();
    assertThat(unplaced.trippedAt(0)).isFalse();
    assertThat(bottom.part())
        .as("mounted where it's pressed: the pivot's parent")
        .isEqualTo(Part.CHASSIS);
  }

  /**
   * What a switch or a rangefinder means is a condition over its reading, either way: a beam
   * tripped or clear, a range nearer or farther than a distance it can read.
   */
  @Test
  void conditionsReadASwitchEitherWayAndARangeAgainstADistance() {
    Sensor.Switch claw = Sensor.Switch.beam("claw beam", PIVOT);
    Sensor.Rangefinder hopper =
        Sensor.Rangefinder.builder()
            .label("hopper range")
            .part(Part.CHASSIS)
            .range(Meters.of(1.3))
            .fieldOfView(Degrees.of(27))
            .build();

    assertThat(claw.tripped()).hasToString("claw beam tripped");
    assertThat(claw.clear()).hasToString("claw beam clear");
    assertThat(claw.tripped().sensor()).isEqualTo(claw);
    assertThat(hopper.nearerThan(Inches.of(6))).hasToString("hopper range nearer than 0.152 m");
    assertThat(hopper.fartherThan(Meters.of(0.5))).hasToString("hopper range farther than 0.500 m");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> hopper.nearerThan(Meters.of(2)))
        .withMessageContaining("2.000 m is at or beyond hopper range's range, 1.300 m");
  }

  private static final Sensor.Rangefinder HOPPER_RANGE =
      Sensor.Rangefinder.of("hopper range", Part.CHASSIS, Meters.of(1.3), Degrees.of(27));

  private static final String NEVER_CHANGES =
      ", so the condition could never change: either the distance or the sensor's stated range is"
          + " wrong";

  /**
   * A rangefinder is asked about a distance it can tell: more than none, and short of the farthest
   * it reads, nearer or farther alike. At its range or beyond, nothing it reads could ever make the
   * condition change.
   */
  @ParameterizedTest
  @MethodSource("distancesARangefinderCannotTell")
  void aRangefinderConditionNeedsADistanceItCanTell(Supplier<Condition> built, String refusal) {
    assertThatIllegalArgumentException().isThrownBy(built::get).withMessage(refusal);
  }

  // spotless:off
  static Stream<Arguments> distancesARangefinderCannotTell() {
    return Stream.of(
        argumentSet("nearer than no distance",
            (Supplier<Condition>) () -> HOPPER_RANGE.nearerThan(Meters.of(0)),
            "0.000 m isn't inside hopper range's range, 1.300 m"),
        argumentSet("farther than no distance",
            (Supplier<Condition>) () -> HOPPER_RANGE.fartherThan(Meters.of(0)),
            "0.000 m isn't inside hopper range's range, 1.300 m"),
        argumentSet("nearer than a distance behind it",
            (Supplier<Condition>) () -> HOPPER_RANGE.nearerThan(Meters.of(-0.5)),
            "-0.500 m isn't inside hopper range's range, 1.300 m"),
        argumentSet("nearer than its range, which it reads anything as",
            (Supplier<Condition>) () -> HOPPER_RANGE.nearerThan(Meters.of(1.3)),
            "1.300 m is at or beyond hopper range's range, 1.300 m" + NEVER_CHANGES),
        argumentSet("farther than its range, which it never reads",
            (Supplier<Condition>) () -> HOPPER_RANGE.fartherThan(Meters.of(1.3)),
            "1.300 m is at or beyond hopper range's range, 1.300 m" + NEVER_CHANGES),
        argumentSet("nearer than a hair short of its range, which counts as its range",
            (Supplier<Condition>) () -> HOPPER_RANGE.nearerThan(Meters.of(1.3 - Range.EDGE / 2)),
            "1.300 m is at or beyond hopper range's range, 1.300 m" + NEVER_CHANGES),
        argumentSet("farther than exactly a hair short of its range, which still counts as its range",
            (Supplier<Condition>) () -> HOPPER_RANGE.fartherThan(Meters.of(1.3 - Range.EDGE)),
            "1.300 m is at or beyond hopper range's range, 1.300 m" + NEVER_CHANGES),
        argumentSet("nearer than beyond its range",
            (Supplier<Condition>) () -> HOPPER_RANGE.nearerThan(Meters.of(2)),
            "2.000 m is at or beyond hopper range's range, 1.300 m" + NEVER_CHANGES),
        argumentSet("farther than beyond its range",
            (Supplier<Condition>) () -> HOPPER_RANGE.fartherThan(Meters.of(2)),
            "2.000 m is at or beyond hopper range's range, 1.300 m" + NEVER_CHANGES));
  }
  // spotless:on

  /** Just short of its range is a distance it can tell, nearer or farther alike. */
  @Test
  void aRangefinderConditionMayBeJustShortOfItsRange() {
    Distance justShort = Meters.of(1.3 - 2 * Range.EDGE);

    assertThat(HOPPER_RANGE.nearerThan(justShort).distance()).isEqualTo(justShort);
    assertThat(HOPPER_RANGE.fartherThan(justShort).distance()).isEqualTo(justShort);
  }

  /**
   * A switch has a name, and a rangefinder a name, a range it reads and a cone it sees across, that
   * make sense: a blank name, no range, or a cone wider than all around are refused.
   */
  @Test
  void aSensorIsRefusedWithoutWhatMakesItOne() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Sensor.Switch.beam(" ", Part.CHASSIS))
        .withMessageContaining("Switch[ ].label");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Sensor.Rangefinder.of(" ", Part.CHASSIS, Meters.of(1.3), Degrees.of(27)))
        .withMessageContaining("Rangefinder[ ].label");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(0), Degrees.of(27)))
        .withMessageContaining("Rangefinder[hopper].range");
    assertThatIllegalArgumentException()
        .isThrownBy(
            () -> Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(200)))
        .withMessageContaining("Rangefinder[hopper].fieldOfView");
  }

  /** A switch is the same one wired or not, and a different one under another name. */
  @Test
  void aSwitchIsEqualToItselfWhereverItsWired() {
    Sensor.Switch beam = Sensor.Switch.beam("claw beam", Part.CHASSIS);

    assertThat(beam.on(SensorWiring.port(1).normallyClosed()))
        .isEqualTo(beam)
        .hasSameHashCodeAs(beam);
    assertThat(Sensor.Switch.beam("hopper beam", Part.CHASSIS))
        .as("another name: another switch, with another hash")
        .isNotEqualTo(beam)
        .doesNotHaveSameHashCodeAs(beam);
  }

  /**
   * A rangefinder is the same one wired or not, and a different one if its name, its part, its
   * place, its range or its cone differs.
   */
  @Test
  void aRangefinderIsEqualToItselfWhereverItsWired() {
    Sensor.Rangefinder hopper =
        Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27));

    assertThat(hopper.on(SensorWiring.CANRANGE.can(31)))
        .isEqualTo(hopper)
        .hasSameHashCodeAs(hopper);
    assertThat(hopper)
        .isNotEqualTo(
            Sensor.Rangefinder.of("feeder", Part.CHASSIS, Meters.of(1.3), Degrees.of(27)));
    assertThat(Sensor.Rangefinder.of("feeder", Part.CHASSIS, Meters.of(1.3), Degrees.of(27)))
        .as("another name: another hash, so a map keeps the two apart cheaply")
        .doesNotHaveSameHashCodeAs(hopper);
    assertThat(hopper)
        .isNotEqualTo(Sensor.Rangefinder.of("hopper", PIVOT, Meters.of(1.3), Degrees.of(27)));
    assertThat(hopper)
        .isNotEqualTo(hopper.at(new Pose3d(new Translation3d(0.1, 0, 0), Rotation3d.ZERO)));
    assertThat(hopper)
        .isNotEqualTo(Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(2), Degrees.of(27)));
    assertThat(hopper)
        .isNotEqualTo(
            Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(40)));
    assertThat(hopper).isNotEqualTo("hopper");
  }

  /**
   * A switch on a part the robot doesn't have is refused; one not placed on its part, and a limit
   * switch whose position isn't known, are in the missing-data report, as a link's mass is.
   */
  @Test
  void theDefinitionKnowsWhereItsSwitchesAreAndSaysWhatItDoesnt() {
    Drive pivotMotor = MechanismFixtures.motors(PIVOT, Motor.KRAKEN_X60, 1, gearing(30, 1));
    Drive rollerMotor = MechanismFixtures.motors(ROLLER, Motor.KRAKEN_X60, 1, gearing(3, 1));
    Sensor.Switch placed =
        Sensor.Switch.beam("claw beam", PIVOT)
            .at(new Pose3d(new Translation3d(0.35, 0, 0), Rotation3d.ZERO));
    Sensor.Switch unplaced = Sensor.Switch.contact("cage", Part.CHASSIS);
    Sensor.Switch unknown = Sensor.Switch.limit("pivot down", PIVOT, Sensor.Switch.Limit.End.LOWER);

    RobotDefinition robot =
        definition(
            List.of(PIVOT, ROLLER),
            List.of(),
            List.of(pivotMotor, rollerMotor),
            List.of(placed, unplaced, unknown));
    Joint.Revolute stranger = Joint.roller("stranger", Part.CHASSIS, new Translation3d());

    assertThat(robot.math().sensorPose(placed, Map.of(PIVOT, Math.PI / 2)).getTranslation())
        .as("on the arm, near its end, raised")
        .satisfies(
            where -> {
              assertThat(where.getX()).isCloseTo(0.2, offset(1e-9));
              assertThat(where.getZ()).isCloseTo(0.65, offset(1e-9));
            });
    assertThat(robot.math().sensorPose(unplaced, Map.of()))
        .as("at its part's origin")
        .isEqualTo(Pose3d.ZERO);
    assertThat(MechanismFixtures.notes(robot))
        .contains(
            "cage isn't placed on its part: it's drawn, and simulated, at its part's origin",
            "pivot down isn't placed on its part: it's drawn, and simulated, at its part's origin",
            "pivot down's position on pivot isn't known: it can stop pivot but not home it")
        .noneMatch(note -> note.startsWith("claw beam isn't placed"));
    assertThatIllegalArgumentException()
        .isThrownBy(
            () ->
                definition(
                    List.of(PIVOT, ROLLER),
                    List.of(),
                    List.of(pivotMotor, rollerMotor),
                    List.of(Sensor.Switch.beam("stray beam", stranger))))
        .withMessageContaining(
            "sensor stray beam's joint, stranger, isn't one of the robot's joints");
  }
}
