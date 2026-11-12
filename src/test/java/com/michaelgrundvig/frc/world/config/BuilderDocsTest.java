package com.michaelgrundvig.frc.world.config;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * The generated builders' setters, as an editor shows them: each with its component's own text from
 * its record's {@code @param} (gradle/builder-docs.gradle), not RecordBuilder's "Set a new value
 * for the record component".
 */
class BuilderDocsTest {
  /** Where the build writes the generated builders' sources, as the test task says. */
  private static final Path GENERATED = Path.of(System.getProperty("builderSources", "missing"));

  static Stream<Arguments> setters() {
    return Stream.of(
        argumentSet(
            "a staged setter",
            "robot/SwerveWiringModuleWiringBuilder.java",
            "the drive motor's controller: <code>.clockwise()</code> if it turns clockwise"),
        argumentSet(
            "an optional component's setter",
            "robot/MechanismControlGainsBuilder.java",
            "volts to hold the joint against gravity, where tuned"),
        argumentSet(
            "an optional measure's setter",
            "robot/DrivetrainControlLimitsBuilder.java",
            "the fastest the driver can ask the robot to spin"));
  }

  @ParameterizedTest
  @MethodSource("setters")
  void aBuilderSetterCarriesItsComponentsOwnText(String builder, String text) throws IOException {
    String source =
        Files.readString(
            GENERATED.resolve("com/michaelgrundvig/frc/world").resolve(builder),
            StandardCharsets.UTF_8);

    assertThat(source).contains(text).doesNotContain("Set a new value for the");
  }
}
