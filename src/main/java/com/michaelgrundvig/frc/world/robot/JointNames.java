package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.possessive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * A joint's named positions and outputs, taken as names: finding one by its name, saying whether a
 * position or an output that code hands the joint is the joint's own, and putting one somewhere
 * else on a practice robot.
 *
 * <p>A position or an output is a name on a joint, and its value belongs to the robot. A team's
 * constant, {@code PIVOT_OUT}, holds the competition robot's value. A practice robot may put the
 * same name at another value ({@code replaced}), and then remembers the value it had, so the team's
 * constant still counts as the practice robot's own. A constant at any value the joint has never
 * had is some other joint's, or some other robot's, and is refused wherever it is given.
 */
final class JointNames {
  private JointNames() {}

  /** The names of some positions, in their order. */
  static List<String> ofPositions(List<Position> positions) {
    return positions.stream().map(Position::name).toList();
  }

  /** The names of some outputs, in their order. */
  static List<String> ofOutputs(List<Output> outputs) {
    return outputs.stream().map(Output::name).toList();
  }

  /** What a joint has by name, as a message ends: "it has in, run, out", or "it has none". */
  static String has(List<String> names) {
    if (names.isEmpty()) {
      return "it has none";
    }
    return "it has " + String.join(", ", names);
  }

  /**
   * A joint's position by its name.
   *
   * @throws IllegalArgumentException if the joint has none by that name, naming those it has
   */
  static Position position(Joint joint, String name) {
    for (Position position : joint.positions()) {
      if (position.name().equals(name)) {
        return position;
      }
    }
    throw new IllegalArgumentException(
        joint.label()
            + " has no position named \""
            + name
            + "\": "
            + has(ofPositions(joint.positions())));
  }

  /**
   * A joint's output by its name: one of its own, or {@link Output#STOPPED}, which every joint has.
   *
   * @throws IllegalArgumentException if the joint has none by that name, naming those it has
   */
  static Output output(Joint joint, String name) {
    for (Output output : joint.outputs()) {
      if (output.name().equals(name)) {
        return output;
      }
    }
    if (name.equals(Output.STOPPED_NAME)) {
      return Output.STOPPED;
    }
    throw new IllegalArgumentException(
        joint.label()
            + " has no output named \""
            + name
            + "\": "
            + has(ofOutputs(joint.outputs()))
            + ", and "
            + Output.STOPPED_NAME);
  }

  /**
   * Why a position is not a joint's own, if it is not.
   *
   * <p>A position is the joint's own when the joint has a position of that name and kind, and the
   * one given is where the joint's is now or where it was before it was re-placed ({@link
   * Joint#formerPositions}). Otherwise it is refused: a position of a name the joint has, but
   * somewhere it has never been, is another joint's or another robot's, and a position of a name
   * the joint does not have is refused with the names it does have.
   *
   * <p>Two joints may each have a position of the same name and value. Nothing tells those apart,
   * so each is taken as either joint's own, and it drives each joint to its own.
   */
  static Optional<String> whyNotOwn(Joint joint, Position position) {
    Optional<Position> sameName =
        joint.positions().stream().filter(own -> own.name().equals(position.name())).findFirst();
    if (sameName.isEmpty()) {
      return Optional.of(
          joint.label()
              + " has no position "
              + position.name()
              + " at "
              + position.kind().shown(position.value())
              + ": "
              + has(ofPositions(joint.positions())));
    }
    Position own = sameName.get();
    if (own.kind() != position.kind()) {
      return Optional.of(
          possessive(joint.label())
              + " "
              + own.name()
              + " is "
              + aOrAn(own.kind())
              + ", and this "
              + own.name()
              + " "
              + aOrAn(position.kind())
              + ": another joint's");
    }
    boolean isWhereItIs = isSamePlace(own, position);
    boolean isWhereItWas =
        joint.formerPositions().stream().anyMatch(former -> isSamePlace(former, position));
    if (isWhereItIs || isWhereItWas) {
      return Optional.empty();
    }
    return Optional.of(
        notOneItHasHad(
            joint, own.name(), "is at " + joint.shown(own.value()), joint.shown(position.value())));
  }

  /**
   * Why an output is not a joint's own, if it is not. {@link Output#STOPPED} is every joint's own.
   * Any other output is judged as a position is ({@link #whyNotOwn(Joint, Position)}): it is the
   * joint's own when the joint has an output of that name, and the one given runs at what the
   * joint's runs at now, or ran at before it was re-valued ({@link Joint#formerOutputs}).
   */
  static Optional<String> whyNotOwn(Joint joint, Output output) {
    if (output.name().equals(Output.STOPPED_NAME)) {
      return Optional.empty();
    }
    Optional<Output> sameName =
        joint.outputs().stream().filter(own -> own.name().equals(output.name())).findFirst();
    if (sameName.isEmpty()) {
      return Optional.of(
          joint.label()
              + " has no output "
              + output.name()
              + ": "
              + has(ofOutputs(joint.outputs()))
              + ", and "
              + Output.STOPPED_NAME);
    }
    Output own = sameName.get();
    boolean isWhatItRunsAt = isSameOutput(own, output);
    boolean isWhatItRanAt =
        joint.formerOutputs().stream().anyMatch(former -> isSameOutput(former, output));
    if (isWhatItRunsAt || isWhatItRanAt) {
      return Optional.empty();
    }
    return Optional.of(
        notOneItHasHad(
            joint, own.name(), "runs at " + shownOutput(joint, own), shownOutput(joint, output)));
  }

  /**
   * Where a joint rests when its block says it rests at a named position.
   *
   * <p>If the joint has a position of that name, it rests where it puts its own. A block may also
   * say where the joint rests before it names its positions, and then the joint has no such name
   * yet: it rests where the position given is, provided that is of the joint's kind. Whether either
   * is inside the joint's stops is for the robot's definition to judge.
   *
   * @throws IllegalArgumentException if the position is another joint's, or of the other kind
   */
  static double whereToRestAt(Joint joint, Position rest) {
    boolean hasThatName =
        joint.positions().stream().anyMatch(each -> each.name().equals(rest.name()));
    if (hasThatName) {
      return joint.own(rest).value();
    }
    Position.Kind kind = Position.Kind.of(joint);
    if (rest.kind() == kind) {
      return rest.value();
    }
    throw new IllegalArgumentException(
        joint.label()
            + " can't rest at "
            + rest.name()
            + ", "
            + switch (kind) {
              case ANGLE -> "a length, since it turns: give it an angle, rest(Degrees.of(...))";
              case LENGTH -> "an angle, since it slides: give it a length, rest(Inches.of(...))";
            });
  }

  /**
   * A joint's positions with one of them put somewhere else: the position of the same name is
   * swapped for the one given.
   *
   * @throws IllegalArgumentException if the joint has no position of that name
   */
  static List<Position> withReplaced(Joint joint, Position replacing) {
    List<Position> positions = new ArrayList<>();
    for (Position each : joint.positions()) {
      if (each.name().equals(replacing.name())) {
        positions.add(replacing);
      } else {
        positions.add(each);
      }
    }
    if (!positions.contains(replacing)) {
      throw new IllegalArgumentException(
          joint.label()
              + " has no position "
              + replacing.name()
              + " to re-place: "
              + has(ofPositions(joint.positions()))
              + ". Its positions are what its code names, the same on every robot of the design");
    }
    return positions;
  }

  /**
   * A joint's outputs with one of them at another value: the output of the same name is swapped for
   * the one given.
   *
   * @throws IllegalArgumentException if the joint has no output of that name
   */
  static List<Output> withReplaced(Joint joint, Output replacing) {
    List<Output> outputs = new ArrayList<>();
    for (Output each : joint.outputs()) {
      if (each.name().equals(replacing.name())) {
        outputs.add(replacing);
      } else {
        outputs.add(each);
      }
    }
    if (!outputs.contains(replacing)) {
      throw new IllegalArgumentException(
          joint.label()
              + " has no output "
              + replacing.name()
              + " to re-value: "
              + has(ofOutputs(joint.outputs()))
              + ". Its outputs are what its code names, the same on every robot of the design");
    }
    return outputs;
  }

  /**
   * The values a named place has had, once it is replaced: the values it had before, and the one it
   * has now unless that is already among them or is the same as the one replacing it. The joint
   * keeps these so that a constant at any of them is still the joint's own.
   *
   * @param <P> a position or an output
   */
  static <P> List<P> formerWith(List<P> former, P now, P replacing) {
    List<P> had = new ArrayList<>(former);
    boolean isNewToTheList = !now.equals(replacing) && !had.contains(now);
    if (isNewToTheList) {
      had.add(now);
    }
    return had;
  }

  /**
   * Whether two positions of one name are the same place. They are when they are within a nanometer
   * or a nanoradian of each other, so that a place written in degrees and the same place written in
   * radians, which convert a rounding step apart, are one.
   */
  private static boolean isSamePlace(Position one, Position other) {
    return one.name().equals(other.name())
        && one.kind() == other.kind()
        && Math.abs(one.value() - other.value()) <= Range.EDGE;
  }

  /** Whether two outputs of one name run a joint the same: see {@link #isSamePlace}. */
  private static boolean isSameOutput(Output one, Output other) {
    return one.name().equals(other.name())
        && one.kind() == other.kind()
        && one.motion().equals(other.motion())
        && Math.abs(one.value() - other.value()) <= Range.EDGE;
  }

  /**
   * The refusal of a place whose name the joint has, at a value the joint has never had:
   * "elevator's L1 is at 0.5 m, and this L1, at 0.25 m, isn't one it has had: another joint's or
   * another robot's; give elevator's own".
   *
   * @param howItsOwnIs how the joint's own one is: "is at 0.5 m"
   * @param given the value of the one given: "0.25 m"
   */
  private static String notOneItHasHad(Joint joint, String name, String howItsOwnIs, String given) {
    return possessive(joint.label())
        + " "
        + name
        + " "
        + howItsOwnIs
        + ", and this "
        + name
        + ", at "
        + given
        + ", isn't one it has had: another joint's or another robot's; give "
        + possessive(joint.label())
        + " own";
  }

  /**
   * What an output runs a joint at, as people read it: "1000 rpm", "-4 V", "60% of the battery".
   */
  private static String shownOutput(Joint joint, Output output) {
    return switch (output.kind()) {
      case VELOCITY -> joint.shownSpeed(output.value());
      case VOLTAGE -> shown(output.value()) + " V";
      case DUTY_CYCLE -> shown(output.value() * 100) + "% of the battery";
    };
  }

  private static String aOrAn(Position.Kind kind) {
    return switch (kind) {
      case ANGLE -> "an angle";
      case LENGTH -> "a length";
    };
  }
}
