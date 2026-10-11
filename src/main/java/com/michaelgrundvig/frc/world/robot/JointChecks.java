package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.Rule;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Transform3d;

/**
 * The checks a joint makes of its own values as it is built. Most are about kind: a joint that
 * turns is measured in angles and one that slides in lengths, and its range, its named positions,
 * its named speeds and its control all have to be of its kind.
 *
 * <p>A joint does not check that where it rests and its named positions are inside its stops. The
 * robot's definition checks that on the finished joint ({@code Blocks}), so that a practice robot
 * may move its stops and its places in either order.
 *
 * <p>These checks report through {@link Checks}, which takes a reason as text, or null where there
 * is none. Each reason here is worked out as an {@code Optional}, and becomes null only where it is
 * handed over.
 */
final class JointChecks {
  private JointChecks() {}

  /**
   * Checks the values that both kinds of joint have.
   *
   * @param record the kind of joint, which names its violations: {@code Revolute[arm].range}
   * @param kind what the joint is measured in
   * @throws com.michaelgrundvig.frc.world.config.ConfigException listing everything wrong
   */
  static void shared(
      Class<? extends Record> record,
      Position.Kind kind,
      String label,
      Transform3d origin,
      Vector3 axis,
      Optional<Range> range,
      Optional<Double> rest,
      List<Position> positions,
      List<Output> outputs) {
    Checks.on(record, label)
        .field("label", label, notBlank())
        .field("origin", origin, numbersAll())
        .field("axis", axis, pointingSomeWay())
        .field("rest", rest, given(finite()))
        .check("range", () -> whyRangeIsOfTheOtherKind(kind, range).orElse(null))
        .check("positions", () -> whyPositionsAreOfTheOtherKind(kind, positions).orElse(null))
        .check("outputs", () -> whyNamesAreNotOneEach(positions, outputs).orElse(null))
        .check("outputs", () -> whySpeedsAreOfTheOtherKind(label, kind, outputs).orElse(null))
        .orThrow();
  }

  /**
   * Checks how a joint that turns is driven: its motors have no drum, and its control is in angles.
   */
  static void turning(String label, Driven driven) {
    Checks.on(Joint.Revolute.class, label)
        .check("driven.motors", () -> whyTurningHasADrum(driven).orElse(null))
        .check(
            "driven.control",
            () -> whyControlIsOfTheOtherKind(Position.Kind.ANGLE, driven).orElse(null))
        .orThrow();
  }

  /**
   * Checks how a joint that slides is driven: its motors have a drum, its gravity feedforward has
   * no angle, and its control is in lengths.
   */
  static void sliding(String label, Driven driven) {
    Checks.on(Joint.Prismatic.class, label)
        .check("driven.motors", () -> whySlidingHasNoDrum(driven).orElse(null))
        .check("driven.control", () -> whySlidingHasAGravityAngle(driven).orElse(null))
        .check(
            "driven.control",
            () -> whyControlIsOfTheOtherKind(Position.Kind.LENGTH, driven).orElse(null))
        .orThrow();
  }

  /**
   * How a stretch of a joint's travel is refused when it is given in the other kind's units,
   * lengths for a joint that turns or angles for one that slides: "its range is lengths, and the
   * joint turns: give it in angles, Range.degrees(...)".
   *
   * @param whatItIs what the stretch is, and the verb: "its range is", "the region is"
   * @param jointsKind what the joint is measured in
   */
  static String wrongKind(String whatItIs, Position.Kind jointsKind) {
    return switch (jointsKind) {
      case ANGLE ->
          whatItIs + " lengths, and the joint turns: give it in angles, Range.degrees(...)";
      case LENGTH ->
          whatItIs
              + " angles, and the joint slides: give it in lengths, Range.meters(...) or"
              + " Range.of(Inches.of(...), Inches.of(...))";
    };
  }

  /**
   * The rule that a transform is made of numbers: none of its translation or its rotation is NaN or
   * infinite. Like every {@link Rule}, it answers null for a value that passes.
   */
  private static Rule<Transform3d> numbersAll() {
    return transform -> {
      Quaternion turn = transform.getRotation().getQuaternion();
      // A sum is finite only if every part of it is.
      double sum =
          transform.getX()
              + transform.getY()
              + transform.getZ()
              + turn.getW()
              + turn.getX()
              + turn.getY()
              + turn.getZ();
      if (Double.isFinite(sum)) {
        return null;
      }
      return transform + " must be a transform, numbers all";
    };
  }

  /** The rule that an axis has a direction: its length is a number, and more than zero. */
  private static Rule<Vector3> pointingSomeWay() {
    return axis -> {
      boolean pointsSomeWay = Double.isFinite(axis.length()) && axis.length() > 0;
      if (pointsSomeWay) {
        return null;
      }
      return axis + " must point some way: an axis can't be zero";
    };
  }

  private static Optional<String> whyRangeIsOfTheOtherKind(
      Position.Kind kind, Optional<Range> range) {
    if (range.isEmpty() || range.get().kind() == kind) {
      return Optional.empty();
    }
    return Optional.of(wrongKind("its range is", kind));
  }

  /**
   * Why a joint's named positions are not all of its kind, if they are not: each length named on a
   * joint that turns, and each angle named on one that slides.
   */
  private static Optional<String> whyPositionsAreOfTheOtherKind(
      Position.Kind kind, List<Position> positions) {
    List<String> wrong = new ArrayList<>();
    for (Position position : positions) {
      if (position.kind() == kind) {
        continue;
      }
      wrong.add(
          position.name()
              + " is "
              + switch (kind) {
                case ANGLE ->
                    "a length, and the joint turns: give it an angle, Position.degrees(...)";
                case LENGTH ->
                    "an angle, and the joint slides: give it a length, Position.of(name,"
                        + " Inches.of(...))";
              });
    }
    return joined(wrong);
  }

  /**
   * Why a joint's named speeds are not all of its kind, if they are not: each speed of sliding
   * named on a joint that turns, and each speed of turning named on one that slides. An rpm on an
   * elevator would otherwise be run as that many meters a second. A voltage or a duty cycle is of
   * neither kind, and suits either joint.
   */
  private static Optional<String> whySpeedsAreOfTheOtherKind(
      String label, Position.Kind kind, List<Output> outputs) {
    List<String> wrong = new ArrayList<>();
    for (Output output : outputs) {
      Optional<Position.Kind> speedOf = output.motion();
      if (speedOf.isEmpty() || speedOf.get() == kind) {
        continue;
      }
      wrong.add(
          output.name()
              + ", "
              + speedOf.get().shownSpeed(output.value())
              + ", is "
              + switch (kind) {
                case ANGLE ->
                    "a speed along a slide, and "
                        + label
                        + " turns: give it a speed of turning, Output.rpm(\""
                        + output.name()
                        + "\", ...)";
                case LENGTH ->
                    "a speed of turning, and "
                        + label
                        + " slides: give it a speed along it, Output.of(\""
                        + output.name()
                        + "\", InchesPerSecond.of(...))";
              });
    }
    return joined(wrong);
  }

  /**
   * Why a joint's names are not one position or output each, if they are not. No name may be used
   * twice among its positions and outputs together, and none may be the name of the output that
   * every joint has built in ({@link Output#STOPPED_NAME}).
   */
  private static Optional<String> whyNamesAreNotOneEach(
      List<Position> positions, List<Output> outputs) {
    List<String> names = new ArrayList<>(JointNames.ofPositions(positions));
    names.addAll(JointNames.ofOutputs(outputs));
    Set<String> seen = new HashSet<>();
    Set<String> twice = new LinkedHashSet<>();
    for (String name : names) {
      boolean seenBefore = !seen.add(name);
      if (seenBefore) {
        twice.add(name);
      }
    }
    if (seen.contains(Output.STOPPED_NAME)) {
      return Optional.of(
          "it names "
              + Output.STOPPED_NAME
              + ", which every joint has built in (its motors at neutral): name it something else");
    }
    if (twice.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(
        "it names "
            + String.join(", ", twice)
            + " twice: each name is one position or output of the joint");
  }

  private static Optional<String> whyTurningHasADrum(Driven driven) {
    boolean hasADrum = driven.motors().flatMap(Driven.Motors::pitchRadius).isPresent();
    if (!hasADrum) {
      return Optional.empty();
    }
    return Optional.of("a joint that turns has no drum: give its motors a gearing alone");
  }

  private static Optional<String> whySlidingHasNoDrum(Driven driven) {
    Optional<Driven.Motors> motors = driven.motors();
    if (motors.isEmpty() || motors.get().pitchRadius().isPresent()) {
      return Optional.empty();
    }
    return Optional.of("a joint that slides needs its drum: give its motors the pitch radius");
  }

  private static Optional<String> whySlidingHasAGravityAngle(Driven driven) {
    boolean hasAGravityAngle =
        driven.control().flatMap(control -> control.gains().gravityAngle()).isPresent();
    if (!hasAGravityAngle) {
      return Optional.empty();
    }
    return Optional.of(
        "a joint that slides is held against gravity the same all along it, so its kG has no"
            + " angle: give kG alone, withKg(kG)");
  }

  /** Why a joint's control is given in the other kind's units, if it is. The control says. */
  private static Optional<String> whyControlIsOfTheOtherKind(Position.Kind kind, Driven driven) {
    return driven.control().map(control -> control.ofTheOtherKind(kind));
  }

  /** Several reasons as one, or none when there are none. */
  private static Optional<String> joined(List<String> reasons) {
    if (reasons.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(String.join("; ", reasons));
  }
}
