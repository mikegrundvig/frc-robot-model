package com.michaelgrundvig.frc.world;

import static com.tngtech.archunit.library.Architectures.layeredArchitecture;

import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.core.importer.ImportOption;
import org.junit.jupiter.api.Test;

/**
 * The world model's packages, kept apart, checked on every build. Gradle keeps the robot program,
 * the simulator, localization and Pitwall out of the model's reach; the robot program's
 * ArchitectureTest keeps the model free of clocks, unseeded draws, threads and shared state.
 */
class ArchitectureTest {
  /**
   * The validation kit ({@code config}) is what every description is built with, and the physical
   * values ({@code physical}) what the field and the robot are both made of; the field and the
   * robot's definition know nothing of each other; the paths ({@code path}) read the field, and the
   * games ({@code game}: a season's limits, which read the robot's definition, and its field, which
   * reads the field, the physical values and the robot's body) read what they describe, and neither
   * the paths nor the games are read by the field, the robot or the other; the writers for other
   * tools ({@code bridges}) read all of it, and nothing reads them.
   */
  @Test
  void theModelsPackagesStayApart() {
    layeredArchitecture()
        .consideringOnlyDependenciesInLayers()
        .layer("kit")
        .definedBy("com.michaelgrundvig.frc.world.config..")
        .layer("physical")
        .definedBy("com.michaelgrundvig.frc.world.physical..")
        .layer("field")
        .definedBy("com.michaelgrundvig.frc.world.field..")
        .layer("robot")
        .definedBy("com.michaelgrundvig.frc.world.robot..")
        .layer("path")
        .definedBy("com.michaelgrundvig.frc.world.path..")
        .layer("game")
        .definedBy("com.michaelgrundvig.frc.world.game..")
        .layer("bridges")
        .definedBy("com.michaelgrundvig.frc.world.bridges..")
        .whereLayer("kit")
        .mayOnlyBeAccessedByLayers("physical", "field", "robot", "game", "bridges")
        .whereLayer("physical")
        .mayOnlyBeAccessedByLayers("field", "robot", "game", "bridges")
        .whereLayer("field")
        .mayOnlyBeAccessedByLayers("path", "game", "bridges")
        .whereLayer("robot")
        .mayOnlyBeAccessedByLayers("game", "bridges")
        .whereLayer("path")
        .mayOnlyBeAccessedByLayers("bridges")
        .whereLayer("game")
        .mayOnlyBeAccessedByLayers("bridges")
        .whereLayer("bridges")
        .mayNotBeAccessedByAnyLayer()
        .because(
            "a field that knew the robot, or a robot the field, would change whenever the other did;"
                + " so would the kit and the physical values if they knew what's built on them")
        .check(
            new ClassFileImporter()
                .withImportOption(ImportOption.Predefined.DO_NOT_INCLUDE_TESTS)
                .importPackages("com.michaelgrundvig.frc.world"));
  }
}
