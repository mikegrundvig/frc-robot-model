package com.michaelgrundvig.frc.world.field;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalDouble;
import org.jspecify.annotations.Nullable;

/**
 * Takes values out of a parsed JSON file one at a time, and keeps a list of everything that was
 * wrong with them.
 *
 * <p>Each method reads one value. If the value is missing or is the wrong kind of thing, the method
 * adds a problem that says so and returns empty, and the caller carries on with the next value.
 * That is how one reading of a file reports every problem in it at once, in the order a person
 * reading the file from the top would meet them.
 */
final class JsonReading {
  private final List<String> problems = new ArrayList<>();

  /** Adds a problem that is not about one value's kind: an unknown key, a tag listed twice. */
  void problem(String problem) {
    problems.add(problem);
  }

  boolean foundProblems() {
    return !problems.isEmpty();
  }

  /** Every problem found so far, in the order they were found, as one line. */
  String problemsListed() {
    return String.join("; ", problems);
  }

  /**
   * A value that has to be a JSON object.
   *
   * @param what the value as a problem names it: "layout", "tag 4's pose"
   */
  Optional<Map<?, ?>> object(@Nullable Object value, String what) {
    if (value instanceof Map<?, ?> object) {
      return Optional.of(object);
    }
    if (value == null) {
      problems.add(what + " is missing");
    } else {
      problems.add(what + " isn't an object");
    }
    return Optional.empty();
  }

  /**
   * A number an object has to have under a key. JSON has no infinity and no not-a-number, but a
   * number too large for a double reads as infinite, and that is refused as not a number.
   *
   * @param what the number as a problem names it. When it is not simply the key, the problem names
   *     it as this followed by the key again, so a caller that passes "tag 2's x" for the key "x"
   *     gets "tag 2's x's x is missing". The reader's messages are its contract with the tools that
   *     show them, so the wording stays as it is until those can change with it.
   */
  OptionalDouble number(Map<?, ?> object, String key, String what) {
    if (object.get(key) instanceof Number number && Double.isFinite(number.doubleValue())) {
      return OptionalDouble.of(number.doubleValue());
    }
    String named = what;
    if (!key.equals(what)) {
      named = possessive(what) + " " + key;
    }
    if (object.containsKey(key)) {
      problems.add(named + " isn't a number");
    } else {
      problems.add(named + " is missing");
    }
    return OptionalDouble.empty();
  }

  /**
   * Text an object has to have under a key.
   *
   * @param what the text as a problem names it
   */
  Optional<String> text(Map<?, ?> object, String key, String what) {
    if (object.get(key) instanceof String text) {
      return Optional.of(text);
    }
    if (object.containsKey(key)) {
      problems.add(what + " isn't text");
    } else {
      problems.add(what + " is missing");
    }
    return Optional.empty();
  }

  /**
   * Text an object may leave out. Leaving it out is fine, and gives the stated default. Giving
   * something that is not text is a problem all the same.
   */
  Optional<String> textOr(Map<?, ?> object, String key, String what, String ifLeftOut) {
    if (!object.containsKey(key)) {
      return Optional.of(ifLeftOut);
    }
    return text(object, key, what);
  }
}
