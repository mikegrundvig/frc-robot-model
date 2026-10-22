package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Bumpers;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Pose2d;

/**
 * The pre-match placement check: is the robot on a start position, and on the one expected? The
 * drive team reads it while setting up, instead of an auto spending its first seconds correcting a
 * placement error. It is only as good as the pose it is given: on a robot, that takes cameras that
 * see tags.
 *
 * @param status the verdict
 * @param start the start the robot is on, or the one it should move to; null when the alliance is
 *     unknown or the field has no starts
 * @param distanceMeters how far the robot is from that start
 * @param headingErrorDegrees how far the robot is turned from that start's heading
 */
public record Placement(
    Status status, @Nullable String start, double distanceMeters, double headingErrorDegrees) {
  public enum Status {
    /** The alliance isn't known yet, so neither are the start positions. */
    ALLIANCE_UNKNOWN,
    /** On the expected start (or on any start, when none is expected), within tolerance. */
    READY,
    /** On a start, but not the expected one. */
    WRONG_START,
    /** Not on any start. */
    NOT_ON_A_START
  }

  /**
   * Checks the robot's pose against the field's starts for its alliance, and against the expected
   * start, which needn't be one of the field's: an auto may start anywhere it says.
   *
   * @param bumpers the robot's, which stand it on each start ({@link Start#poseFor})
   * @param expected the start the robot should be on (the selected auto's), or null for any
   * @param toleranceMeters how far from a start still counts as on it
   * @param toleranceDegrees how far turned from a start's heading still counts as on it
   */
  public static Placement check(
      FieldModel field,
      Bumpers bumpers,
      Optional<Alliance> alliance,
      Pose2d pose,
      @Nullable Start expected,
      double toleranceMeters,
      double toleranceDegrees) {
    if (alliance.isEmpty()) {
      return new Placement(Status.ALLIANCE_UNKNOWN, null, 0, 0);
    }
    // How far the robot is from each start it could be on. Until the end, none of these says the
    // robot is on its start: that is decided below, against the tolerances.
    Optional<Placement> fromNearest = Optional.empty();
    Optional<Placement> fromExpected = Optional.empty();
    for (Start start : startsToJudge(field, expected)) {
      Placement fromStart = offsetFrom(start, start.on(field, alliance.get(), bumpers), pose);
      if (start.equals(expected)) {
        fromExpected = Optional.of(fromStart);
      }
      boolean nearerThanAnyBefore =
          fromNearest.isEmpty() || fromStart.distanceMeters < fromNearest.get().distanceMeters;
      if (nearerThanAnyBefore) {
        fromNearest = Optional.of(fromStart);
      }
    }

    if (fromNearest.isEmpty()) {
      return new Placement(Status.NOT_ON_A_START, null, 0, 0);
    }
    boolean onTheExpectedStart =
        fromExpected.isPresent() && fromExpected.get().within(toleranceMeters, toleranceDegrees);
    if (onTheExpectedStart) {
      return fromExpected.get().withStatus(Status.READY);
    }
    boolean onAnotherStart = fromNearest.get().within(toleranceMeters, toleranceDegrees);
    if (onAnotherStart && expected == null) {
      return fromNearest.get().withStatus(Status.READY);
    }
    if (onAnotherStart) {
      return fromNearest.get().withStatus(Status.WRONG_START);
    }
    // Off every start: point the drive team to the one the auto expects, if it expects one.
    return fromExpected.orElse(fromNearest.get());
  }

  /**
   * The starts the robot might be on: the field's, and the expected one too when it is not one of
   * the field's.
   */
  private static List<Start> startsToJudge(FieldModel field, @Nullable Start expected) {
    List<Start> starts = new ArrayList<>(field.starts());
    if (expected != null && !starts.contains(expected)) {
      starts.add(expected);
    }
    return starts;
  }

  /**
   * How far a pose is from a start, and how far it is turned from the start's heading, with the
   * robot not yet said to be on it.
   *
   * @param spot where the start is for the robot's alliance
   */
  private static Placement offsetFrom(Start start, Pose2d spot, Pose2d pose) {
    double distanceMeters = pose.getTranslation().getDistance(spot.getTranslation());
    double turnedDegrees = Math.abs(pose.getRotation().minus(spot.getRotation()).getDegrees());
    return new Placement(Status.NOT_ON_A_START, start.name(), distanceMeters, turnedDegrees);
  }

  /** One line for the drive team, such as "Ready on Center: 2 cm, 1° off". */
  public String describe() {
    return switch (status) {
      case ALLIANCE_UNKNOWN -> "Alliance unknown";
      case READY -> "Ready on " + start + ": " + offBy();
      case WRONG_START -> "On " + start + ", not the expected start";
      case NOT_ON_A_START -> describeNotOnAStart();
    };
  }

  private String describeNotOnAStart() {
    if (start == null) {
      return "No start positions";
    }
    return "Not on a start: " + start + " is " + away();
  }

  private boolean within(double toleranceMeters, double toleranceDegrees) {
    return distanceMeters <= toleranceMeters && headingErrorDegrees <= toleranceDegrees;
  }

  private Placement withStatus(Status newStatus) {
    return new Placement(newStatus, start, distanceMeters, headingErrorDegrees);
  }

  private String offBy() {
    return String.format(
        Locale.ROOT, "%.0f cm, %.0f° off", distanceMeters * 100, headingErrorDegrees);
  }

  private String away() {
    return String.format(Locale.ROOT, "%.2f m and %.0f° away", distanceMeters, headingErrorDegrees);
  }
}
