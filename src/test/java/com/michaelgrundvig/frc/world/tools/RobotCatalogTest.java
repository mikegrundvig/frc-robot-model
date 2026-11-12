package com.michaelgrundvig.frc.world.tools;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.michaelgrundvig.frc.world.robot.CameraConfig;
import com.michaelgrundvig.frc.world.robot.ConfigFixtures;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/** A program's robots as the tool writers find them: by name, or refused naming those there are. */
class RobotCatalogTest {
  private static RobotDescription robot(String name, List<CameraConfig> cameras) {
    return new RobotDescription(
        name, ConfigFixtures.body(), ConfigFixtures.drivetrain(), cameras, Optional.empty());
  }

  private static final RobotCatalog TWO =
      () -> List.of(robot("competition", List.of()), robot("practice", List.of()));

  @Test
  void aRobotIsFoundByItsNameAndAnUnknownOneIsRefused() {
    record Row(String name, String found) {}
    for (Row row :
        List.of(new Row("competition", "competition"), new Row("practice", "practice"))) {
      assertThat(TWO.named(row.name()).name()).as(row.name()).isEqualTo(row.found());
    }
    assertThatThrownBy(() -> TWO.named("beta"))
        .isInstanceOf(IllegalArgumentException.class)
        .hasMessage("no robot named beta; the robots are competition, practice");
  }

  @Test
  void aDescriptionNeedsANameAndKeepsItsOwnCameras() {
    List<CameraConfig> cameras = new ArrayList<>();
    cameras.add(ConfigFixtures.camera("front", new Translation3d(0.2, 0, 0.5)));
    RobotDescription described = robot("competition", cameras);
    cameras.clear();

    assertThat(described.cameras()).hasSize(1);
    assertThatThrownBy(() -> robot(" ", List.of())).hasMessage("a robot needs a name");
  }
}
