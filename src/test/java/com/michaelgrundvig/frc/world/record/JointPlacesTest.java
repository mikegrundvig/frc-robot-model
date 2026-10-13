package com.michaelgrundvig.frc.world.record;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.RecordedRobot;
import org.junit.jupiter.api.Test;

/**
 * Where a joint is among its places: the pivot's are in at 1.5°, run at 110° and out at 130°, which
 * are places 0, 1 and 2 of its positions.
 */
class JointPlacesTest {
  private static final Joint PIVOT = RecordedRobot.joint(RecordedRobot.PIVOT);
  private static final JointPlaces PLACES = new JointPlaces(PIVOT, Math.toRadians(2));

  private static JointPlace at(double degrees) {
    return PLACES.at(Math.toRadians(degrees));
  }

  @Test
  void itIsAtAPlaceWithinItsBand() {
    assertThat(at(1.5)).isEqualTo(JointPlace.by(JointPlace.Where.AT, 0));
    assertThat(at(108.5)).isEqualTo(JointPlace.by(JointPlace.Where.AT, 1));
    assertThat(at(131.9)).isEqualTo(JointPlace.by(JointPlace.Where.AT, 2));
  }

  @Test
  void itIsBetweenTwoOrBeyondThem() {
    assertThat(at(60)).isEqualTo(JointPlace.between(0, 1));
    assertThat(at(120)).isEqualTo(JointPlace.between(1, 2));
    assertThat(at(-3)).isEqualTo(JointPlace.by(JointPlace.Where.BELOW, 0));
    assertThat(at(134)).isEqualTo(JointPlace.by(JointPlace.Where.ABOVE, 2));
  }

  /** The same place gives the very same record, so a loop that asks makes no new object. */
  @Test
  void theSamePlaceIsTheSameRecord() {
    assertThat(at(60)).isSameAs(at(61));
    assertThat(at(130)).isSameAs(at(131));
  }

  @Test
  void aJointWithNoPlacesIsUnnamed() {
    JointPlaces none = new JointPlaces(RecordedRobot.unnamedTurret(), Math.toRadians(2));

    assertThat(none.at(Math.toRadians(45))).isSameAs(JointPlace.UNNAMED);
  }
}
