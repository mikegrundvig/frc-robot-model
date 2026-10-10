package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.field.Alliance.BLUE;
import static com.michaelgrundvig.frc.world.field.Alliance.RED;
import static com.michaelgrundvig.frc.world.field.TestFields.FIELD;
import static com.michaelgrundvig.frc.world.field.TestFields.polygon;
import static com.michaelgrundvig.frc.world.field.TestFields.refused;
import static com.michaelgrundvig.frc.world.field.TestFields.xy;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.assertj.core.api.Assertions.within;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.List;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

class ZoneTest {
  /** The blue alliance's end of the field, 3 m deep. */
  private static final Polygon2d BLUE_END = Polygon2d.rectangle(xy(0, 0), xy(3, 8));

  /** The zones the tables name, built as each test runs. */
  private static Zone zone(String name) {
    return switch (name) {
      case "ours" -> Zone.ours("Alliance zone", BLUE_END);
      case "theirs" -> Zone.theirs("Their zone", BLUE_END);
      case "trench" -> Zone.both("Trench", Polygon2d.rectangle(xy(5, 8), xy(4, 7)));
      // An L: a corner cut out of a rectangle, as 2024's starting zone is.
      case "L" -> Zone.ours("Corner", polygon(0, 0, 3, 0, 3, 1, 1, 1, 1, 3, 0, 3));
      // A 1 m square turned 45°: a diamond reaching 0.71 m from its center along each axis.
      case "diamond" ->
          Zone.ours(
              "Diamond", Polygon2d.rectangle(new Pose2d(8, 4, Rotation2d.fromDegrees(45)), 1, 1));
      default -> throw new AssertionError(name);
    };
  }

  static Stream<Arguments> containment() {
    return Stream.of(
        argumentSet("ours: blue's end is blue's", "ours", BLUE, xy(1, 1), true),
        argumentSet("ours: red's end is red's", "ours", RED, xy(15, 7), true),
        argumentSet("ours: blue's end is not red's", "ours", RED, xy(1, 1), false),
        argumentSet("theirs: red's end is blue's opponent", "theirs", BLUE, xy(15, 7), true),
        argumentSet("theirs: blue's end is red's opponent", "theirs", RED, xy(1, 1), true),
        argumentSet("both: blue's half, for blue", "trench", BLUE, xy(4.5, 7.5), true),
        argumentSet("both: red's half, for blue", "trench", BLUE, xy(11.5, 0.5), true),
        argumentSet("both: between them, for blue", "trench", BLUE, xy(8, 4), false),
        argumentSet("both: blue's half, for red", "trench", RED, xy(4.5, 7.5), true),
        argumentSet("both: red's half, for red", "trench", RED, xy(11.5, 0.5), true),
        argumentSet("both: between them, for red", "trench", RED, xy(8, 4), false),
        argumentSet("an L holds what's in the L", "L", BLUE, xy(0.5, 2.5), true),
        argumentSet("an L does not hold its cut", "L", BLUE, xy(2, 2), false),
        argumentSet("an L for red, in the L", "L", RED, xy(15.5, 5.5), true),
        argumentSet("an L for red, in the cut", "L", RED, xy(14, 6), false));
  }

  @ParameterizedTest
  @MethodSource("containment")
  void aZoneIsWhereItsSideAndAllianceSayItIs(
      String zone, Alliance alliance, Translation2d point, boolean inside) {
    assertThat(zone(zone).contains(FIELD, alliance, point)).isEqualTo(inside);
  }

  @Test
  void aZoneOnBothHalvesIsEachAreaAndItsImage() {
    assertThat(zone("trench").areas(FIELD, BLUE)).hasSize(2);
    assertThat(zone("ours").areas(FIELD, BLUE)).hasSize(1);
  }

  static Stream<Arguments> distances() {
    return Stream.of(
        argumentSet("inside is zero", "ours", xy(1, 4), 0.0),
        argumentSet("outside is meters to the edge", "ours", xy(5, 4), 2.0),
        argumentSet("from an L's cut", "L", xy(2, 2), 1.0));
  }

  @ParameterizedTest
  @MethodSource("distances")
  void distanceIsZeroInsideAndMetersToTheEdgeOutside(
      String zone, Translation2d point, double meters) {
    assertThat(zone(zone).distance(FIELD, BLUE, point)).isCloseTo(meters, within(1e-9));
  }

  /** Whether blue's robot, from a point at a velocity, reaches the zone within some seconds. */
  static Stream<Arguments> reaching() {
    return Stream.of(
        argumentSet("4 m out, 0.5 s", "ours", xy(5, 4), xy(-2, 0), 0.5, false),
        argumentSet("2 m out, 1 s: at the edge", "ours", xy(5, 4), xy(-2, 0), 1.0, true),
        argumentSet("heading away", "ours", xy(5, 4), xy(2, 0), 5.0, false),
        argumentSet("already inside, standing still", "ours", xy(1, 4), xy(0, 0), 0.0, true),
        argumentSet("through a turned area", "diamond", xy(7, 4), xy(2, 0), 1.0, true),
        argumentSet("above its top corner", "diamond", xy(7, 4.8), xy(2, 0), 1.0, false),
        argumentSet("stopping short", "diamond", xy(8, 2), xy(0, 1.2), 1.0, false),
        argumentSet("beside it, straight up", "diamond", xy(7, 4.8), xy(0, 1), 1.0, false),
        argumentSet("just reaching its bottom corner", "diamond", xy(8, 2), xy(0, 1.3), 1.0, true),
        argumentSet("down into an L, short", "L", xy(2, 2), xy(0, -2), 0.4, false),
        argumentSet("down into an L", "L", xy(2, 2), xy(0, -2), 0.5, true));
  }

  @ParameterizedTest
  @MethodSource("reaching")
  void itLooksAheadAlongTheRobotsVelocity(
      String zone, Translation2d from, Translation2d velocity, double seconds, boolean reached) {
    assertThat(zone(zone).reachedWithin(FIELD, BLUE, from, velocity, seconds)).isEqualTo(reached);
  }

  /** A trigger with a 0.4 s lookahead and a 0.1 m margin, for a robot heading at -x. */
  @Test
  void aTriggerTurnsOnBeforeArrivingAndOffOnlyPastTheMargin() {
    ZoneTrigger trigger = new ZoneTrigger(zone("ours"), 0.4, 0.1);
    Translation2d toward = xy(-2, 0);

    assertThat(trigger.update(FIELD, BLUE, xy(4, 4), toward)).as("1 m out, 0.5 s away").isFalse();
    assertThat(trigger.update(FIELD, BLUE, xy(3.7, 4), toward)).as("0.35 s away").isTrue();
    assertThat(trigger.update(FIELD, BLUE, xy(3.05, 4), xy(0, 0)))
        .as("stopped just outside, within the margin")
        .isTrue();
    assertThat(trigger.update(FIELD, BLUE, xy(3.2, 4), xy(0, 0))).as("past the margin").isFalse();
    assertThat(trigger.isActive()).isFalse();
    assertThat(trigger.update(FIELD, BLUE, xy(3.05, 4), xy(0, 0)))
        .as("the margin only holds it on")
        .isFalse();
    assertThat(trigger.zone()).isEqualTo(zone("ours"));
  }

  static Stream<Arguments> refusals() {
    return Stream.of(
        refused("a zone with no name", () -> Zone.ours(" ", BLUE_END), "a zone needs a name"),
        refused(
            "a zone with no area",
            () -> new Zone("Nowhere", Zone.Side.OURS, List.of()),
            "Nowhere needs at least one area"),
        refused("a negative lookahead", () -> new ZoneTrigger(zone("ours"), -1, 0), TRIGGER),
        refused(
            "a margin that is no number",
            () -> new ZoneTrigger(zone("ours"), 0, Double.NaN),
            TRIGGER));
  }

  private static final String TRIGGER = "the lookahead and the margin must be zero or more";

  @ParameterizedTest
  @MethodSource("refusals")
  void whatCantWorkIsRefusedSayingWhy(ThrowingCallable building, String message) {
    assertThatThrownBy(building).isInstanceOf(IllegalArgumentException.class).hasMessage(message);
  }
}
