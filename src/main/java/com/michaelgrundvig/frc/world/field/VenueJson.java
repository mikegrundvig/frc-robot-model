package com.michaelgrundvig.frc.world.field;

import io.avaje.json.JsonException;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import org.jspecify.annotations.Nullable;

/**
 * Turns the text of a venue file, or of a bare WPILib layout, into plain Java values. What those
 * values mean is {@link VenueFile}'s business.
 */
final class VenueJson {
  /** Any JSON value: maps in the file's order, lists, text, numbers, true or false. */
  private static final JsonType<Object> ANY_VALUE = Jsonb.instance().type(Object.class);

  private VenueJson() {}

  /**
   * The value JSON text holds: a map for an object, a list for an array, and null for JSON's null.
   *
   * @throws IllegalArgumentException if the text is not JSON, saying where, or if it nests deeper
   *     than {@link VenueFile#MAX_DEPTH}
   */
  static @Nullable Object parse(String text) {
    // The depth is counted before the text is parsed, because it is the parser that would
    // overflow its stack on text nested thousands deep.
    if (depth(text) > VenueFile.MAX_DEPTH) {
      throw new IllegalArgumentException(
          "not JSON a venue file can be: nested deeper than " + VenueFile.MAX_DEPTH + " levels");
    }
    try {
      return ANY_VALUE.fromJson(text);
    } catch (JsonException e) {
      throw new IllegalArgumentException("not JSON: " + e.getMessage(), e);
    }
  }

  /**
   * How deep the text's lists and objects nest at its deepest. Brackets inside a string are text,
   * not nesting, so they are not counted.
   */
  static int depth(String text) {
    int depth = 0;
    int deepest = 0;
    boolean inString = false;
    for (int index = 0; index < text.length(); index++) {
      char character = text.charAt(index);
      if (inString) {
        if (character == '\\') {
          // A backslash escapes the character after it, which is skipped: an escaped quote does
          // not end the string.
          index++;
        } else if (character == '"') {
          inString = false;
        }
        continue;
      }
      switch (character) {
        case '"' -> inString = true;
        case '[', '{' -> {
          depth++;
          deepest = Math.max(deepest, depth);
        }
        case ']', '}' -> depth--;
        default -> {}
      }
    }
    return deepest;
  }
}
