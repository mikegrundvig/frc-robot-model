package com.michaelgrundvig.frc.world.config;

import java.math.BigDecimal;
import java.math.MathContext;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoublePredicate;
import java.util.stream.Collectors;
import org.wpilib.units.Measure;

/**
 * The common {@link Rule}s. Every one that takes a number fails NaN and infinity, and says what it
 * found the way people write it: {@code "-3 A must be positive"}.
 *
 * <p>{@link #positive}, {@link #nonNegative}, and {@link #finite} take a plain number or a measure.
 * A range takes plain numbers ({@code between(-1, 1)}) or measures ({@code between(Degrees.of(-90),
 * Degrees.of(90))}): a measure's range is in real units, so a value written in radians is still
 * checked against degrees correctly.
 *
 * <p>Each rule here is written the same way: a number or a measure is refused if it is not finite,
 * and otherwise if what the rule asks of it does not hold ({@link #refusedUnless}). Measures are
 * compared in their base units, and shown in the unit they were written in.
 */
public final class Rules {
  /** How many significant digits a number is shown to. */
  private static final int DIGITS_SHOWN = 4;

  private Rules() {}

  /** Greater than zero. */
  public static <V> Rule<V> positive() {
    return comparedWithZero(magnitude -> magnitude > 0, "must be positive");
  }

  /** Zero or more. */
  public static <V> Rule<V> nonNegative() {
    return comparedWithZero(magnitude -> magnitude >= 0, "must be zero or more");
  }

  /** Any number but NaN and infinity. */
  public static <V> Rule<V> finite() {
    // Nothing more is asked of a finite number, so there is nothing more to say of one.
    return comparedWithZero(magnitude -> true, "");
  }

  /** At least {@code min}. */
  public static Rule<Double> atLeast(double min) {
    return Rule.of(value -> refusedUnless(value, value >= min, "must be at least " + shown(min)));
  }

  /** At most {@code max}. */
  public static Rule<Double> atMost(double max) {
    return Rule.of(value -> refusedUnless(value, value <= max, "must be at most " + shown(max)));
  }

  /** Less than {@code limit}. */
  public static Rule<Double> below(double limit) {
    return Rule.of(value -> refusedUnless(value, value < limit, "must be below " + shown(limit)));
  }

  /** From {@code min} to {@code max}, both included. */
  public static Rule<Double> between(double min, double max) {
    return Rule.of(
        value -> {
          boolean inRange = value >= min && value <= max;
          return refusedUnless(value, inRange, "must be from " + shown(min) + " to " + shown(max));
        });
  }

  /** At least {@code min}, in real units. */
  public static <M extends Measure<?>> Rule<M> atLeast(M min) {
    return Rule.of(
        value -> {
          boolean atLeast = value.baseUnitMagnitude() >= min.baseUnitMagnitude();
          return refusedUnless(value, atLeast, "must be at least " + shown(min));
        });
  }

  /** At most {@code max}, in real units. */
  public static <M extends Measure<?>> Rule<M> atMost(M max) {
    return Rule.of(
        value -> {
          boolean atMost = value.baseUnitMagnitude() <= max.baseUnitMagnitude();
          return refusedUnless(value, atMost, "must be at most " + shown(max));
        });
  }

  /** Less than {@code limit}, in real units. */
  public static <M extends Measure<?>> Rule<M> below(M limit) {
    return Rule.of(
        value -> {
          boolean below = value.baseUnitMagnitude() < limit.baseUnitMagnitude();
          return refusedUnless(value, below, "must be below " + shown(limit));
        });
  }

  /** From {@code min} to {@code max}, both included, in real units. */
  public static <M extends Measure<?>> Rule<M> between(M min, M max) {
    return Rule.of(
        value -> {
          double magnitude = value.baseUnitMagnitude();
          boolean inRange =
              magnitude >= min.baseUnitMagnitude() && magnitude <= max.baseUnitMagnitude();
          return refusedUnless(value, inRange, "must be from " + shown(min) + " to " + shown(max));
        });
  }

  /** Text with something in it besides spaces. */
  public static Rule<String> notBlank() {
    return Rule.of(
        value -> {
          if (value.isBlank()) {
            return Optional.of("must not be blank");
          }
          return Optional.empty();
        });
  }

  /** One of the values listed. */
  @SafeVarargs
  public static <V> Rule<V> oneOf(V... allowed) {
    List<V> choices = List.of(allowed);
    return Rule.of(
        value -> {
          if (choices.contains(value)) {
            return Optional.empty();
          }
          String listed = choices.stream().map(String::valueOf).collect(Collectors.joining(", "));
          return Optional.of(value + " must be one of " + listed);
        });
  }

  /** Exactly {@code count} items. */
  public static <V> Rule<List<V>> size(int count) {
    return Rule.of(
        list -> {
          if (list.size() == count) {
            return Optional.empty();
          }
          return Optional.of("must have " + count + ", has " + list.size());
        });
  }

  /** Every item in a list passes the rule. Each one that doesn't is named by its position. */
  public static <V> Rule<List<V>> each(Rule<? super V> rule) {
    return Rule.of(
        list -> {
          List<String> problems = new ArrayList<>();
          for (int position = 0; position < list.size(); position++) {
            Optional<String> whyNot = rule.reason(list.get(position));
            if (whyNot.isPresent()) {
              problems.add("[" + position + "] " + whyNot.get());
            }
          }
          if (problems.isEmpty()) {
            return Optional.empty();
          }
          return Optional.of(String.join("; ", problems));
        });
  }

  /** An optional value: none passes, and one given passes the rule. */
  public static <V> Rule<Optional<V>> given(Rule<? super V> rule) {
    return Rule.of(
        value -> {
          if (value.isPresent()) {
            return rule.reason(value.get());
          }
          return Optional.empty();
        });
  }

  /** A measure as people write it: "4.729 m/s" rather than "4.728761e+00 m/s". */
  public static String shown(Measure<?> value) {
    return shown(value.magnitude()) + " " + value.unit().symbol();
  }

  /**
   * A name's possessive, as a message says it: "pivot's", and "rollers'" for a name that ends in s,
   * never "rollers's".
   */
  public static String possessive(String name) {
    if (name.endsWith("s")) {
      return name + "'";
    }
    return name + "'s";
  }

  /** A number to four significant digits, without an exponent: "0.08", "1235000". */
  public static String shown(double value) {
    if (!Double.isFinite(value)) {
      return Double.toString(value);
    }
    BigDecimal rounded = new BigDecimal(value).round(new MathContext(DIGITS_SHOWN));
    return rounded.stripTrailingZeros().toPlainString();
  }

  /**
   * A rule that asks something of a number, or of a measure in its base unit, where what it asks
   * does not depend on the unit: how it compares with zero.
   *
   * @param holds what is asked of the number
   * @param whatIsAsked what is asked, as a refusal says it after the value: "must be positive"
   */
  private static <V> Rule<V> comparedWithZero(DoublePredicate holds, String whatIsAsked) {
    return Rule.of(
        value -> {
          if (value instanceof Number number) {
            double plain = number.doubleValue();
            return refusedUnless(plain, holds.test(plain), whatIsAsked);
          }
          if (value instanceof Measure<?> measure) {
            return refusedUnless(measure, holds.test(measure.baseUnitMagnitude()), whatIsAsked);
          }
          return Optional.of(
              "can't be checked as a number: it's a " + value.getClass().getSimpleName());
        });
  }

  /**
   * The reason a number is refused: it is not finite, or what is asked of it does not hold.
   *
   * @param holds whether what is asked of the number holds
   * @param whatIsAsked what is asked, as a refusal says it after the value: "must be at least 1"
   */
  private static Optional<String> refusedUnless(double value, boolean holds, String whatIsAsked) {
    if (!Double.isFinite(value)) {
      return Optional.of(shown(value) + " must be a finite number");
    }
    if (holds) {
      return Optional.empty();
    }
    return Optional.of(shown(value) + " " + whatIsAsked);
  }

  /** The same for a measure, which is finite when its magnitude in its base unit is. */
  private static Optional<String> refusedUnless(
      Measure<?> value, boolean holds, String whatIsAsked) {
    if (!Double.isFinite(value.baseUnitMagnitude())) {
      return Optional.of(shown(value) + " must be a finite number");
    }
    if (holds) {
      return Optional.empty();
    }
    return Optional.of(shown(value) + " " + whatIsAsked);
  }
}
