package com.michaelgrundvig.frc.world.game;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Kilograms;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;

import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Shape;
import com.michaelgrundvig.frc.world.physical.Vector3;
import com.michaelgrundvig.frc.world.robot.Body;
import com.michaelgrundvig.frc.world.robot.BodyBuilder;
import com.michaelgrundvig.frc.world.robot.ConfigFixtures;
import com.michaelgrundvig.frc.world.robot.Configuration;
import com.michaelgrundvig.frc.world.robot.Coupling;
import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.JointRevoluteBuilder;
import com.michaelgrundvig.frc.world.robot.Link;
import com.michaelgrundvig.frc.world.robot.LinkBuilder;
import com.michaelgrundvig.frc.world.robot.Part;
import com.michaelgrundvig.frc.world.robot.Position;
import com.michaelgrundvig.frc.world.robot.Range;
import com.michaelgrundvig.frc.world.robot.RobotDefinition;
import com.michaelgrundvig.frc.world.robot.RobotDefinitionBuilder;
import java.util.List;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * The season's limits on what a robot is: every robot's definition fits them, and one that doesn't
 * is told each way it doesn't. An arm on the base robot, 0.7 m long and pivoting 5 cm up at its
 * center, stands in for a mechanism: it starts straight up, inside the frame and under the starting
 * height, and reaches 0.35 m out laid flat, more than the 12 in allowed.
 */
class SeasonLimitsTest {
  private static final double UPRIGHT = Math.PI / 2;

  /**
   * A season's limits: a 110 in frame perimeter, 30 in tall at the start, bumpers within 2.5 to
   * 5.75 in of the floor, 135 lb without the battery, which weighs 14.5 lb.
   */
  private static final SeasonLimits LIMITS =
      SeasonLimits.builder()
          .framePerimeter(Inches.of(110))
          .startingHeight(Inches.of(30))
          .extension(Inches.of(12))
          .bumperZoneBottom(Inches.of(2.5))
          .bumperZoneTop(Inches.of(5.75))
          .weight(Pounds.of(135))
          .battery(Pounds.of(14.5))
          .build();

  /**
   * A robot that fits exactly: the largest square frame within the perimeter, 27.5 in, bumpers 4 in
   * beyond it in the bumper zone, 30 in tall, not weighed.
   */
  private static final RobotDefinition BASE =
      RobotDefinitionBuilder.builder(ConfigFixtures.definition(List.of()))
          .body(
              BodyBuilder.builder(ConfigFixtures.body())
                  .frameLength(Inches.of(27.5))
                  .frameWidth(Inches.of(27.5))
                  .bumperEnds(Inches.of(4))
                  .bumperSides(Inches.of(4))
                  .weight(LIMITS.unweighed())
                  .inertia(Body.Turning.yaw(KilogramSquareMeters.of(6.7)))
                  .build())
          .build();

  @Test
  void aRobotWithAnArmStartingUprightFits() {
    RobotDefinition robot = withArm(arm(0.7, UPRIGHT));

    assertThat(LIMITS.problems(robot)).isEmpty();
  }

  /**
   * A robot with a joint that follows another is checked with each where its references put it: the
   * follower's position is its leader's to give, not the check's.
   */
  @Test
  void aRobotWithACoupledJointIsCheckedStowed() {
    Joint.Revolute arm = arm(0.7, UPRIGHT);
    Joint.Revolute forearm =
        JointRevoluteBuilder.builder(arm)
            .label("forearm")
            .link(LinkBuilder.builder(arm.link()).label("forearm").build())
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(BASE)
            .joints(List.of(arm, forearm))
            .couplings(
                List.of(
                    Coupling.Linear.builder()
                        .leader(arm)
                        .follower(forearm)
                        .ratio(1)
                        .offset(0)
                        .build()))
            .build();

    assertThat(LIMITS.problems(robot)).isEmpty();
  }

  /**
   * A robot not yet weighed is the most the season allows, with its battery: the base robot's own,
   * which fits exactly.
   */
  @Test
  void anUnweighedRobotIsTheSeasonsMostWithABattery() {
    Body.Weight unweighed = LIMITS.unweighed();

    assertThat(unweighed.weighed()).isFalse();
    assertThat(unweighed.mass().in(Pounds)).isCloseTo(149.5, within(1e-9));
    assertThat(BASE.body().weight()).isEqualTo(unweighed);
    assertThat(LIMITS.problems(BASE)).isEmpty();
  }

  /** Everything wrong is told at once. */
  @Test
  void aRobotThatDoesntFitIsToldEveryWay() {
    Body weighed = BASE.body();
    Body big =
        BodyBuilder.builder(weighed)
            .frameLength(Inches.of(30))
            .height(Inches.of(31))
            .bumperBottom(Inches.of(2))
            .weight(Body.Weight.weighed(Pounds.of(155)))
            .build();
    RobotDefinition robot = RobotDefinitionBuilder.builder(withArm(arm(0.7, 0))).body(big).build();

    assertThat(LIMITS.problems(robot))
        .containsExactly(
            "its frame is 115 in around, more than the 110 in allowed",
            "it starts 0.787 m tall, more than the 0.762 m allowed",
            "it starts reaching 0.319 m beyond its frame perimeter: it must start inside it",
            "its bumpers, 2 in to 5.75 in up, aren't within the bumper zone, 2.5 in to 5.75 in",
            "it weighs 140.5 lb. without its battery, more than the 135 lb. allowed: lighten it, or"
                + " weigh it again");
  }

  /** A link above the body at the start is too tall, as a body that tall would be. */
  @Test
  void aLinkStartingAboveTheStartingHeightIsTooTall() {
    RobotDefinition robot = withArm(arm(0.8, UPRIGHT));

    assertThat(LIMITS.problems(robot))
        .containsExactly("it starts 0.850 m tall, more than the 0.762 m allowed");
  }

  /**
   * Each configuration is checked against the extension limit, with the joints it doesn't name at
   * their references: the arm laid flat reaches 0.351 m out, more than the 12 in allowed, and
   * raised halfway, it's inside.
   */
  @Test
  void eachConfigurationIsCheckedAgainstTheExtensionLimit() {
    Joint.Revolute arm =
        JointRevoluteBuilder.builder(arm(0.7, UPRIGHT))
            .positions(
                List.of(
                    Position.degrees("flat", 0),
                    Position.degrees("halfway", 45),
                    Position.degrees("upright", 90)))
            .build();
    RobotDefinition robot =
        RobotDefinitionBuilder.builder(withArm(arm))
            .configurations(
                List.of(
                    Configuration.of("flat", arm.at(arm.position("flat"))),
                    Configuration.of("halfway", arm.at(arm.position("halfway")))))
            .build();

    assertThat(LIMITS.problems(robot))
        .containsExactly(
            "configuration flat reaches 0.351 m beyond its frame perimeter, more than the 0.305 m"
                + " allowed");
  }

  @Test
  void theBumperZoneHasATopAboveItsBottom() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> SeasonLimitsBuilder.builder(LIMITS).bumperZoneTop(Inches.of(2.5)).build())
        .withMessageStartingWith("SeasonLimits.bumperZoneTop: ");
  }

  /** The base robot with its body changed. */
  private static RobotDefinition withBody(UnaryOperator<BodyBuilder> change) {
    Body body = change.apply(BodyBuilder.builder(BASE.body())).build();
    return RobotDefinitionBuilder.builder(BASE).body(body).build();
  }

  /** The base robot with an arm of a length that it lays flat, forward, in a configuration. */
  private static RobotDefinition layingFlat(double armLength) {
    Joint.Revolute arm =
        JointRevoluteBuilder.builder(arm(armLength, UPRIGHT))
            .positions(List.of(Position.degrees("flat", 0)))
            .build();
    return RobotDefinitionBuilder.builder(withArm(arm))
        .configurations(List.of(Configuration.of("flat", arm.at(arm.position("flat")))))
        .build();
  }

  /**
   * Limits in meters that the base robot fits with room to spare, except where a row changes it: 2
   * m around, 0.75 m tall, and bumpers from 0.0625 m to 0.125 m up. Given in meters, a robot's size
   * can be put exactly a rounding past one of them.
   */
  private static SeasonLimits metric() {
    return SeasonLimitsBuilder.builder(LIMITS)
        .framePerimeter(Meters.of(4))
        .startingHeight(Meters.of(1))
        .bumperZoneBottom(Meters.of(0.0625))
        .bumperZoneTop(Meters.of(0.15))
        .build();
  }

  private static Arguments answers(String label, Supplier<?> question, Object answer) {
    return argumentSet(label, question, answer);
  }

  // spotless:off

  /**
   * A robot at a limit fits it, and so does one past it by less than a rounding of its conversion
   * from inches or pounds. One past it by more is told so.
   */
  static Stream<Arguments> robotsAtTheLimits() {
    // The frame is 27.5 in long, so an arm pivoting at its center reaches 13.75 in to its edge,
    // and may reach 12 in beyond.
    double reachesTheLimit = Inches.of(13.75 + 12).in(Meters);
    return Stream.of(
        answers("a frame a rounding over the perimeter fits",
            () -> LIMITS.problems(withBody(body -> body.frameLength(Inches.of(27.5000000001)).frameWidth(Inches.of(27.5)))), List.of()),
        answers("a frame a little more over does not",
            () -> LIMITS.problems(withBody(body -> body.frameLength(Inches.of(27.51)).frameWidth(Inches.of(27.5)))),
            List.of("its frame is 110 in around, more than the 110 in allowed")),
        answers("a robot a rounding over the starting height fits",
            () -> LIMITS.problems(withBody(body -> body.height(Inches.of(30.0000000001)))), List.of()),
        answers("a robot a little taller does not",
            () -> LIMITS.problems(withBody(body -> body.height(Inches.of(30.01)))),
            List.of("it starts 0.762 m tall, more than the 0.762 m allowed")),
        answers("bumpers a rounding below the bumper zone fit",
            () -> LIMITS.problems(withBody(body -> body.bumperBottom(Inches.of(2.4999999999)))), List.of()),
        answers("bumpers a little lower do not",
            () -> LIMITS.problems(withBody(body -> body.bumperBottom(Inches.of(2.49)))),
            List.of("its bumpers, 2.49 in to 5.75 in up, aren't within the bumper zone, 2.5 in to 5.75 in")),
        answers("bumpers a rounding above the bumper zone fit",
            () -> LIMITS.problems(withBody(body -> body.bumperTop(Inches.of(5.7500000001)))), List.of()),
        answers("bumpers a little higher do not",
            () -> LIMITS.problems(withBody(body -> body.bumperTop(Inches.of(5.76)))),
            List.of("its bumpers, 2.5 in to 5.76 in up, aren't within the bumper zone, 2.5 in to 5.75 in")),
        answers("a robot a rounding over the weight fits",
            () -> LIMITS.problems(withBody(body -> body.weight(Body.Weight.weighed(Pounds.of(149.5000000001))))), List.of()),
        answers("a robot a little heavier does not",
            () -> LIMITS.problems(withBody(body -> body.weight(Body.Weight.weighed(Pounds.of(149.6))))),
            List.of("it weighs 135.1 lb. without its battery, more than the 135 lb. allowed: lighten it, or weigh it again")),
        answers("a frame over the perimeter by exactly the rounding allowed fits",
            () -> metric().problems(withBody(body -> body.frameLength(Meters.of((4 + 1e-9) / 4)).frameWidth(Meters.of((4 + 1e-9) / 4)))),
            List.of()),
        answers("a robot over the starting height by exactly the rounding allowed fits",
            () -> metric().problems(withBody(body -> body.height(Meters.of(1 + 1e-9)))), List.of()),
        answers("bumpers below the bumper zone by exactly the rounding allowed fit",
            () -> metric().problems(withBody(body -> body.bumperBottom(Meters.of(0.0625 - 1e-9)))), List.of()),
        answers("bumpers above the bumper zone by exactly the rounding allowed fit",
            () -> metric().problems(withBody(body -> body.bumperTop(Meters.of(0.15 + 1e-9)))), List.of()),
        answers("an arm that reaches exactly as far as is allowed fits",
            () -> LIMITS.problems(layingFlat(reachesTheLimit)), List.of()),
        answers("an arm a little longer does not",
            () -> LIMITS.problems(layingFlat(reachesTheLimit + 0.001)),
            List.of("configuration flat reaches 0.306 m beyond its frame perimeter, more than the 0.305 m allowed")));
  }

  // spotless:on

  @ParameterizedTest
  @MethodSource("robotsAtTheLimits")
  void itAnswers(Supplier<?> question, Object answer) {
    assertThat(question.get()).isEqualTo(answer);
  }

  /** The base robot with one arm, the arm's own reference its start. */
  private static RobotDefinition withArm(Joint arm) {
    return RobotDefinitionBuilder.builder(BASE).joints(List.of(arm)).build();
  }

  /**
   * A 3 kg arm of a length, 5 cm wide and deep, pivoting 5 cm up at the robot's center, raised as
   * its position grows, from flat forward at 0 to straight up at a quarter turn.
   */
  private static Joint.Revolute arm(double length, double reference) {
    Translation3d center = new Translation3d(length / 2, 0, 0);
    Translation3d size = new Translation3d(length, 0.05, 0.05);
    return Joint.Revolute.builder()
        .label("arm")
        .parent(Part.CHASSIS)
        .origin(new Transform3d(0, 0, 0.05, Rotation3d.ZERO))
        .axis(new Vector3(0, -1, 0))
        .link(
            Link.builder()
                .label("arm")
                .mass(
                    MassProperties.builder()
                        .mass(Kilograms.of(3))
                        .cg(center)
                        .inertia(Inertia.box(Kilograms.of(3), size))
                        .build())
                .shapes(
                    List.of(
                        Shape.Box.builder()
                            .pose(new Pose3d(center, Rotation3d.ZERO))
                            .size(size)
                            .material(Material.builder().name("aluminum").build())
                            .build()))
                .build())
        .range(Range.degrees(-10, 100))
        .rest(reference)
        .build();
  }
}
