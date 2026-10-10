package com.michaelgrundvig.frc.world.config;

import static com.michaelgrundvig.frc.world.config.Rules.atLeast;
import static com.michaelgrundvig.frc.world.config.Rules.atMost;
import static com.michaelgrundvig.frc.world.config.Rules.below;
import static com.michaelgrundvig.frc.world.config.Rules.between;
import static com.michaelgrundvig.frc.world.config.Rules.each;
import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.oneOf;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static com.michaelgrundvig.frc.world.config.Rules.size;
import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class RulesTest {
  /** A row: a rule (built as the test runs), a value, and why not, or null when it's fine. */
  private static Arguments row(
      String label, Supplier<? extends Rule<?>> rule, Object value, @Nullable String why) {
    return argumentSet(label, rule, value, why);
  }

  static Stream<Arguments> rules() {
    return Stream.of(
        row("positive: above zero", () -> positive(), 0.5, null),
        row("positive: zero", () -> positive(), 0.0, "0 must be positive"),
        row("positive: a measure above zero", () -> positive(), Meters.of(0.1), null),
        row(
            "positive: a measure below zero",
            () -> positive(),
            Meters.of(-2),
            "-2 m must be positive"),
        row(
            "positive: not a number at all",
            () -> positive(),
            "seven",
            "can't be checked as a number: it's a String"),
        row("non-negative: zero", () -> nonNegative(), 0, null),
        row("non-negative: below zero", () -> nonNegative(), -1, "-1 must be zero or more"),
        row("at least: at it", () -> atLeast(2), 2.0, null),
        row("at least: under it", () -> atLeast(2), 1.0, "1 must be at least 2"),
        row("at most: at it", () -> atMost(90), 90.0, null),
        row("at most: over it", () -> atMost(90), 90.5, "90.5 must be at most 90"),
        row("below: just under", () -> below(180), 179.9, null),
        row("below: at it", () -> below(180), 180.0, "180 must be below 180"),
        row("between: at the low end", () -> between(-1, 1), -1.0, null),
        row("between: at the high end", () -> between(-1, 1), 1.0, null),
        row("between: past the high end", () -> between(-1, 1), 1.25, "1.25 must be from -1 to 1"),
        row(
            "measures compare in real units",
            () -> between(Degrees.of(-90), Degrees.of(90)),
            Radians.of(1.5),
            null),
        row(
            "measures show their own unit",
            () -> between(Degrees.of(-90), Degrees.of(90)),
            Radians.of(1.6),
            "1.6 rad must be from -90 ° to 90 °"),
        row(
            "amps over a limit",
            () -> atMost(Amps.of(80)),
            Amps.of(120),
            "120 A must be at most 80 A"),
        row(
            "an angle at a limit",
            () -> below(Degrees.of(180)),
            Radians.of(3.2),
            "3.2 rad must be below 180 °"),
        row(
            "a distance under a minimum",
            () -> atLeast(Meters.of(1)),
            Meters.of(0.5),
            "0.5 m must be at least 1 m"),
        row("text: not blank", () -> notBlank(), "front", null),
        row("text: blank", () -> notBlank(), "  ", "must not be blank"),
        row("one of: listed", () -> oneOf("left", "right"), "right", null),
        row(
            "one of: not listed",
            () -> oneOf("left", "right"),
            "up",
            "up must be one of left, right"),
        row("size: right", () -> size(4), List.of(1, 2, 3, 4), null),
        row("size: wrong", () -> size(4), List.of(1, 2, 3), "must have 4, has 3"),
        row("each: all fine", () -> each(positive()), List.of(1.0, 1.01), null),
        row(
            "each: names every item that fails",
            () -> each(positive()),
            List.of(1.0, -1.0, 1.0, Double.NaN),
            "[1] -1 must be positive; [3] NaN must be a finite number"),
        row("given: nothing given", () -> given(positive()), Optional.empty(), null),
        row("given: fine", () -> given(positive()), Optional.of(4096), null),
        row("given: broken", () -> given(positive()), Optional.of(0), "0 must be positive"));
  }

  @ParameterizedTest
  @MethodSource("rules")
  void aRuleSaysWhyNotInThePeoplesWords(
      Supplier<Rule<Object>> rule, Object value, @Nullable String why) {
    assertThat(rule.get().whyNot(value)).isEqualTo(why);
  }

  static Stream<Arguments> numberRules() {
    return Stream.of(
        of("positive", () -> positive()),
        of("non-negative", () -> nonNegative()),
        of("finite", () -> finite()),
        of("at least", () -> atLeast(0)),
        of("at most", () -> atMost(10)),
        of("below", () -> below(10)),
        of("between", () -> between(-1, 1)));
  }

  static Stream<Arguments> angleRules() {
    return Stream.of(
        of("positive", () -> positive()),
        of("non-negative", () -> nonNegative()),
        of("finite", () -> finite()),
        of("at least", () -> atLeast(Degrees.of(0))),
        of("at most", () -> atMost(Degrees.of(90))),
        of("below", () -> below(Degrees.of(90))),
        of("between", () -> between(Degrees.of(-90), Degrees.of(90))));
  }

  private static Arguments of(String label, Supplier<? extends Rule<?>> rule) {
    return argumentSet(label, rule);
  }

  @ParameterizedTest
  @MethodSource("numberRules")
  void nanAndInfinityFailEveryRuleThatTakesANumber(Supplier<Rule<Object>> rule) {
    assertThat(rule.get().whyNot(Double.NaN)).isEqualTo("NaN must be a finite number");
    assertThat(rule.get().whyNot(Double.POSITIVE_INFINITY))
        .isEqualTo("Infinity must be a finite number");
    assertThat(rule.get().whyNot(Double.NEGATIVE_INFINITY))
        .isEqualTo("-Infinity must be a finite number");
  }

  @ParameterizedTest
  @MethodSource("angleRules")
  void nanFailsEveryRuleThatTakesAMeasure(Supplier<Rule<Object>> rule) {
    assertThat(rule.get().whyNot(Degrees.of(Double.NaN)))
        .isEqualTo("NaN ° must be a finite number");
  }

  @ParameterizedTest
  @MethodSource("shownValues")
  void numbersAreShownAsPeopleWriteThem(Supplier<String> shown, String text) {
    assertThat(shown.get()).isEqualTo(text);
  }

  static Stream<Arguments> shownValues() {
    return Stream.of(
        shows("a measure to three places", () -> shown(MetersPerSecond.of(4.728761)), "4.729 m/s"),
        shows("a whole measure", () -> shown(Amps.of(10)), "10 A"),
        shows("a small number", () -> shown(0.08), "0.08"),
        shows("zero", () -> shown(0), "0"),
        shows("a big number to four figures", () -> shown(1234567), "1235000"),
        shows("not a number", () -> shown(Double.NaN), "NaN"));
  }

  private static Arguments shows(String label, Supplier<String> shown, String text) {
    return argumentSet(label, shown, text);
  }
}
