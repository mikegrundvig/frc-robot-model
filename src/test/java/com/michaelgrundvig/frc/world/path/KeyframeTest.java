package com.michaelgrundvig.frc.world.path;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.world.field.Place;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/** A keyframe is at a pose or at a place, and a place's is shown by its name. */
class KeyframeTest {
  private static final Pose2d MIDDLE = new Pose2d(8.27, 4.03, Rotation2d.ZERO);

  private static final Place MARK = Place.at("Mark", new Pose2d(6.8, 3.2, Rotation2d.PI));

  @Test
  void aKeyframeIsAtAPoseOrAPlace() {
    assertThat(Keyframe.at(MARK).toString()).isEqualTo("Mark");
    assertThat(Keyframe.at(MARK).named("grab").toString()).isEqualTo("grab");
    assertThatThrownBy(() -> new Keyframe(MIDDLE, MARK, null)).hasMessageContaining("not both");
    assertThatThrownBy(() -> new Keyframe(null, null, null)).hasMessageContaining("not both");
  }
}
