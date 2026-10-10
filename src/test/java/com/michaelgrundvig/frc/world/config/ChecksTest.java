package com.michaelgrundvig.frc.world.config;

import static com.michaelgrundvig.frc.world.config.Rules.atLeast;
import static com.michaelgrundvig.frc.world.config.Rules.below;
import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.assertj.core.api.Assertions.catchThrowableOfType;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Seconds;

import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.units.measure.Time;

class ChecksTest {
  /** A record that checks itself the way configuration records do. */
  record Gains(double steerKp, double steerKd, int samples, Time lookahead) {
    Gains {
      Checks.on(Gains.class)
          .field("steerKp", steerKp, nonNegative())
          .field("steerKd", steerKd, nonNegative())
          .field("samples", samples, atLeast(1))
          .field("lookahead", lookahead, between(Seconds.of(0), Seconds.of(0.2)))
          .check(
              "steerKd",
              () ->
                  steerKd <= steerKp
                      ? null
                      : shown(steerKd) + " must be at most steerKp (" + shown(steerKp) + ")")
          .orThrow();
    }
  }

  private static Arguments refusal(String label, ThrowingCallable building, String... lines) {
    return argumentSet(label, building, String.join("\n", lines));
  }

  static Stream<Arguments> refusals() {
    List<Finding> found =
        List.of(
            Finding.refused("pivot", "pivot's soft limits reach past its stops"),
            Finding.doubt("pivot", "pivot's kG is far from its weight's"),
            Finding.note(Finding.Scope.SIMULATOR, "pivot", "pivot's arm has no mass"),
            Finding.refused("roller", "roller has motors and no control"));
    return Stream.of(
        refusal(
            "every violation at once",
            () -> new Gains(-1, Double.NaN, 0, Seconds.of(0.5)),
            "Gains.steerKp: -1 must be zero or more",
            "Gains.steerKd: NaN must be a finite number",
            "Gains.samples: 0 must be at least 1",
            "Gains.lookahead: 0.5 s must be from 0 s to 0.2 s"),
        refusal(
            "one violation to a line",
            () -> new Gains(16, 0, 0, Seconds.of(-1)),
            "Gains.samples: 0 must be at least 1",
            "Gains.lookahead: -1 s must be from 0 s to 0.2 s"),
        refusal(
            "a check across values, under its name",
            () -> new Gains(16, 40, 3, Seconds.of(0)),
            "Gains.steerKd: 40 must be at most steerKp (16)"),
        refusal(
            "checks across values wait for each value's own rule",
            () -> new Gains(-1, 0.5, 3, Seconds.of(0)),
            "Gains.steerKp: -1 must be zero or more"),
        refusal(
            "a record with a name of its own",
            () -> Checks.on(Gains.class, "front").field("noise", 0.0, positive()).orThrow(),
            "Gains[front].noise: 0 must be positive"),
        refusal(
            "a value stops at the first rule it breaks",
            () ->
                Checks.on(Gains.class)
                    .field("fieldOfView", Double.NaN, positive())
                    .field("fieldOfView", Double.NaN, below(180))
                    .field("fieldOfView", 200.0, below(180))
                    .orThrow(),
            "Gains.fieldOfView: NaN must be a finite number"),
        refusal(
            "each refusal a finding makes, under the check's name",
            () -> Checks.on(Gains.class, "robot").findings("joints", () -> found).orThrow(),
            "Gains[robot].joints: pivot's soft limits reach past its stops",
            "Gains[robot].joints: roller has motors and no control"));
  }

  @ParameterizedTest
  @MethodSource("refusals")
  void whatBreaksARuleIsRefusedOneViolationToALine(ThrowingCallable building, String message) {
    ConfigException refused = catchThrowableOfType(ConfigException.class, building);

    assertThat(refused).hasMessage(message);
    assertThat(
            refused.violations().stream()
                .map(Violation::toString)
                .collect(Collectors.joining("\n")))
        .isEqualTo(message);
  }

  @Test
  void aValidRecordBuildsAndADoubtOrNoteIsNeverARefusal() {
    List<Finding> found =
        List.of(
            Finding.doubt("pivot", "pivot's kG is far from its weight's"),
            Finding.note(Finding.Scope.SIMULATOR, "pivot", "pivot's arm has no mass"));

    assertThatCode(() -> new Gains(16, 0.1, 3, Seconds.of(0.05))).doesNotThrowAnyException();
    assertThatCode(() -> Checks.on(Gains.class).findings("joints", () -> found).orThrow())
        .doesNotThrowAnyException();
  }

  @Test
  void aRuleThatThrowsIsReportedWithTheRest() {
    Rule<List<Integer>> firstPositive = list -> list.get(0) > 0 ? null : "must start positive";

    List<Violation> violations =
        catchThrowableOfType(
                ConfigException.class,
                () ->
                    Checks.on(Gains.class)
                        .field("first", List.<Integer>of(), firstPositive)
                        .field("second", -1.0, positive())
                        .orThrow())
            .violations();

    assertThat(violations).hasSize(2);
    assertThat(violations.get(0).where()).isEqualTo("Gains.first");
    assertThat(violations.get(0).what()).startsWith("couldn't be checked: ");
    assertThat(violations.get(1)).isEqualTo(new Violation("Gains.second", "-1 must be positive"));
  }

  @Test
  void anExceptionOrAFindingNeedsSomethingToSay() {
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new ConfigException(List.of()))
        .withMessageContaining("at least one violation");
    assertThatIllegalArgumentException()
        .isThrownBy(() -> Finding.doubt(" ", "nothing"))
        .withMessage("Finding[ ].subject: must not be blank");
  }
}
