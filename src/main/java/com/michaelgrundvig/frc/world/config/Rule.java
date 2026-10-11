package com.michaelgrundvig.frc.world.config;

import java.util.Optional;
import java.util.function.Function;
import org.jspecify.annotations.Nullable;

/**
 * A check on one value: why it's wrong, or null when it's fine. {@link Rules} has the common ones,
 * and any lambda is one too: {@code v -> v % 2 == 0 ? null : "must be even"}. A record runs its
 * rules with {@link Checks}.
 *
 * <p>A rule must let nothing strange through: write it so that NaN fails ({@code !(value >= min)},
 * never {@code value < min}).
 *
 * @param <V> the value it checks
 */
@FunctionalInterface
public interface Rule<V> {
  /**
   * Why the value breaks this rule, or null when it's fine. Never throws.
   *
   * @return what's wrong, written to follow the value's name: {@code "-3 A must be positive"}
   */
  @Nullable String whyNot(V value);

  /**
   * A rule from a check that gives the reason a value is wrong, if there is one, as an {@link
   * Optional} rather than as a string that may be null: {@code
   * Rule.of(Configuration::whyNotOnceEach)}.
   *
   * @param whyNot the reason the value is wrong, or empty when it is fine
   */
  static <V> Rule<V> of(Function<? super V, Optional<String>> whyNot) {
    return value -> whyNot.apply(value).orElse(null);
  }
}
