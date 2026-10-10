package com.michaelgrundvig.frc.world.examples;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

/** The joints the model's Javadoc shows are ones a definition accepts. */
class ExampleJointsTest {
  @Test
  void theJointsTheJavadocShowsAreBuilt() {
    assertThat(ExampleJoints.ROLLER.label()).isEqualTo("roller");
    assertThat(ExampleJoints.ARM.label()).isEqualTo("arm");
    assertThat(ExampleJoints.INTAKE.name()).isEqualTo("intake");
    assertThat(ExampleJoints.STOWED.name()).isEqualTo("stowed");
  }
}
