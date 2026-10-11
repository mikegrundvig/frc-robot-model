package com.michaelgrundvig.frc.world.config;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;

/**
 * Checks a record's values when it is built, and reports everything wrong with them at once. A
 * record lists each value with its {@link Rule}, then any checks across its values, in its compact
 * constructor, as a port encoder's wiring does:
 *
 * {@snippet class=com.michaelgrundvig.frc.world.robot.MechanismWiring region=checks}
 *
 * <p>Every value is checked, and {@link #orThrow()} throws one {@link ConfigException} naming every
 * value that broke a rule: a configuration with three bad values reports three. A value may be
 * listed more than once, with a rule each time; once it breaks one, its later rules are skipped.
 * The checks across values run last, and only once every value has passed its own rule, so they
 * never compare a value that's already reported. Values are named from the record that checks them
 * ({@code PortEncoder.port}): a nested record is built, and checked, before the record that holds
 * it.
 *
 * <p>The two kinds of check run at different times. A value's rule runs as it is listed, since the
 * value is in hand. A check across values is only kept as it is listed, and runs in {@link
 * #orThrow()}, once it is known whether every value passed.
 */
public final class Checks {
  /**
   * A check across values, kept until {@link Checks#orThrow()} runs it.
   *
   * @param name what its refusals are reported under
   * @param found runs the check: everything it finds, each a refusal or not
   */
  private record Across(String name, Supplier<List<Finding>> found) {}

  /** What a rule or a check that threw is reported as, followed by what it threw. */
  private static final String COULD_NOT_BE_CHECKED = "couldn't be checked: ";

  /** The record the values are named from, with its own name if it has one: {@code Gains}. */
  private final String record;

  private final List<Violation> violations = new ArrayList<>();
  private final List<Across> checksAcrossValues = new ArrayList<>();

  /** The names of the values that have broken a rule, whose later rules are skipped. */
  private final Set<String> brokeARule = new HashSet<>();

  private Checks(String record) {
    this.record = record;
  }

  /**
   * Checks for a record, whose violations are named after it: {@code Gains.steerKp}, or for a check
   * across its values, by the path from it, {@code Arm.limits.top} (an arm's record holding its
   * limits). Keep that form: Pitwall reads each line's name back to the value it names, to show the
   * reason beside it on its page (see its {@code LiveConfig}).
   */
  public static Checks on(Class<? extends Record> record) {
    return new Checks(record.getSimpleName());
  }

  /**
   * Checks for a record with a name of its own, which names its violations too: {@code
   * CameraConfig[front].tiltUp}.
   */
  public static Checks on(Class<? extends Record> record, String name) {
    return new Checks(record.getSimpleName() + "[" + name + "]");
  }

  /** Checks a number, or a whole number, against a rule. */
  public Checks field(String name, double value, Rule<Double> rule) {
    return checkValue(name, () -> rule.reason(value));
  }

  /** Checks a value against a rule. */
  public <V> Checks field(String name, V value, Rule<? super V> rule) {
    return checkValue(name, () -> rule.reason(value));
  }

  /**
   * Runs one value's rule, unless the value has already broken an earlier one. A rule that throws
   * is a bug in the rule. It is reported as a violation with the rest, so that the others are not
   * lost.
   */
  private Checks checkValue(String name, Supplier<Optional<String>> reason) {
    if (brokeARule.contains(name)) {
      return this;
    }
    Optional<String> whyNot;
    try {
      whyNot = reason.get();
    } catch (RuntimeException e) {
      whyNot = Optional.of(COULD_NOT_BE_CHECKED + e);
    }
    if (whyNot.isPresent()) {
      violations.add(new Violation(record + "." + name, whyNot.get()));
      brokeARule.add(name);
    }
    return this;
  }

  /**
   * A check across values, reported under {@code name}. It runs in {@link #orThrow()}, only if
   * every value passed its own rule.
   *
   * @param why what's wrong, written to follow the name, or null when it's fine
   */
  public Checks check(String name, Supplier<@Nullable String> why) {
    return whyNot(name, () -> Optional.ofNullable(why.get()));
  }

  /**
   * A check across values, reported under {@code name}, that gives the reason the values are wrong
   * if there is one. It is {@link #check} for a check written to return an {@link Optional} rather
   * than null, and runs at the same point.
   *
   * @param whyNot the reason, written to follow the name, or empty when the values are fine
   */
  public Checks whyNot(String name, Supplier<Optional<String>> whyNot) {
    return findings(name, () -> asFindings(whyNot.get()));
  }

  /** A reason the values are wrong as the one refusal it is, or no reason as nothing found. */
  private List<Finding> asFindings(Optional<String> whyNot) {
    if (whyNot.isEmpty()) {
      return List.of();
    }
    return List.of(Finding.refused(record, whyNot.get()));
  }

  /**
   * A check across values that says what it finds ({@link Finding}), reported under {@code name}:
   * each refusal a violation of its own. Doubts and notes aren't refusals: the program says them at
   * startup. It runs in {@link #orThrow()}, only if every value passed its own rule.
   */
  public Checks findings(String name, Supplier<List<Finding>> found) {
    checksAcrossValues.add(new Across(name, found));
    return this;
  }

  /**
   * Runs the checks across values, if every value passed its own rule, then throws if anything was
   * found wrong.
   *
   * @throws ConfigException listing every violation
   */
  public void orThrow() {
    boolean everyValuePassed = violations.isEmpty();
    if (everyValuePassed) {
      for (Across check : checksAcrossValues) {
        run(check);
      }
    }
    if (!violations.isEmpty()) {
      throw new ConfigException(violations);
    }
  }

  /**
   * Runs one check across values, and adds a violation for each refusal it finds. A check that
   * throws is a bug in the check. It is reported as a violation with the rest, so that the others
   * are not lost.
   */
  private void run(Across check) {
    List<Finding> found;
    try {
      found = check.found().get();
    } catch (RuntimeException e) {
      found = List.of(Finding.refused(record, COULD_NOT_BE_CHECKED + e));
    }
    for (Finding finding : found) {
      if (finding.refused()) {
        violations.add(new Violation(record + "." + check.name(), finding.message()));
      }
    }
  }
}
