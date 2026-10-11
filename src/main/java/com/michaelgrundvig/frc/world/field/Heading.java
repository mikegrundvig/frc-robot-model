package com.michaelgrundvig.frc.world.field;

import static org.wpilib.units.Units.Radians;

import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Angle;

/**
 * Where the robot should face: a fixed heading on the field ({@link #field}), an angle from the
 * drivers' forward ({@link #downfield}), or a place on the field ({@link #toward}). It's worked out
 * again every loop ({@link #resolve}) from where the robot is, the field and the alliance, so a
 * button that snaps to an angle, or aims at the hub, is written once for both alliances.
 *
 * <p>Its front faces it unless {@link #withBack} says the back does, for a robot that shoots from
 * its back.
 */
public final class Heading {
  /** Closer than this to a place, the robot has no bearing on it: it's on top of it. */
  public static final double ON_THE_PLACE_METERS = 0.05;

  /**
   * An angle from the drivers' forward within this of zero, or of half a turn, is named for what it
   * is ("downfield") rather than by its degrees.
   */
  private static final double NAMED_WITHIN_DEGREES = 0.05;

  /** What the robot is to face. */
  private sealed interface Aim {
    /** A heading on the field, counterclockwise from +x, whatever the alliance. */
    record OnField(Rotation2d heading) implements Aim {}

    /** An angle counterclockwise from the way the alliance's drivers look down the field. */
    record FromDownfield(Rotation2d offset) implements Aim {}

    /** A place, as it is for the robot's alliance, from wherever the robot is. */
    record AtPlace(Place place) implements Aim {}
  }

  /** Which end of the robot faces what it is aimed at. */
  private enum End {
    FRONT,
    BACK
  }

  private final Aim aim;
  private final End end;

  private Heading(Aim aim, End end) {
    this.aim = aim;
    this.end = end;
  }

  /** A fixed heading on the field, whatever the alliance: counterclockwise from +x. */
  public static Heading field(Rotation2d heading) {
    return new Heading(new Aim.OnField(heading), End.FRONT);
  }

  /**
   * An angle counterclockwise from the drivers' forward ({@link FieldModel#driverForward}): zero
   * faces away from the drivers, whichever alliance they're on.
   */
  public static Heading downfield(Angle offset) {
    Rotation2d fromDownfield = new Rotation2d(offset.in(Radians));
    return new Heading(new Aim.FromDownfield(fromDownfield), End.FRONT);
  }

  /**
   * Facing a place as it is for the robot's alliance ({@link Place#on}), from wherever the robot
   * is. On top of it, there is no bearing.
   */
  public static Heading toward(Place place) {
    return new Heading(new Aim.AtPlace(place), End.FRONT);
  }

  /** The same, the robot's back facing it instead of its front. */
  public Heading withBack() {
    return new Heading(aim, End.BACK);
  }

  /**
   * The heading the robot should have, as of where it is.
   *
   * @return empty if it needs the alliance and it isn't known, or the robot is on the place
   */
  public Optional<Rotation2d> resolve(Pose2d robot, FieldModel field, Optional<Alliance> alliance) {
    Optional<Rotation2d> wayToFace =
        switch (aim) {
          case Aim.OnField onField -> Optional.of(onField.heading());
          case Aim.FromDownfield fromDownfield ->
              alliance.map(ours -> FieldModel.driverForward(ours).rotateBy(fromDownfield.offset()));
          case Aim.AtPlace atPlace -> bearing(atPlace.place(), robot, field, alliance);
        };
    return switch (end) {
      case FRONT -> wayToFace;
      // The robot's heading is where its front points, so its back faces a way when its front
      // faces the opposite way.
      case BACK -> wayToFace.map(way -> way.rotateBy(Rotation2d.PI));
    };
  }

  /**
   * The direction from the robot to a place, on the field. It is empty when the alliance is not
   * known, since the place is then not known either, and when the robot is on the place.
   */
  private static Optional<Rotation2d> bearing(
      Place place, Pose2d robot, FieldModel field, Optional<Alliance> alliance) {
    if (alliance.isEmpty()) {
      return Optional.empty();
    }
    Translation2d placeOnField = place.on(field, alliance.get()).getTranslation();
    Translation2d robotToPlace = placeOnField.minus(robot.getTranslation());
    if (robotToPlace.getNorm() < ON_THE_PLACE_METERS) {
      return Optional.empty();
    }
    return Optional.of(new Rotation2d(robotToPlace.getX(), robotToPlace.getY()));
  }

  /**
   * How fast the heading itself turns as the robot moves, in radians a second, counterclockwise:
   * the feedforward for turning with it. A fixed heading doesn't turn. The bearing to a place turns
   * with the robot's velocity across the line to it: the cross product of where the robot is from
   * the place and its velocity, over the squared distance.
   *
   * @param fieldVelocity how fast the robot moves, on the field's axes, in meters a second
   * @return zero if it doesn't resolve ({@link #resolve})
   */
  public double rate(
      Pose2d robot, FieldModel field, Optional<Alliance> alliance, Translation2d fieldVelocity) {
    if (!(aim instanceof Aim.AtPlace atPlace) || alliance.isEmpty()) {
      return 0;
    }
    Translation2d placeOnField = atPlace.place().on(field, alliance.get()).getTranslation();
    Translation2d placeToRobot = robot.getTranslation().minus(placeOnField);
    double squaredDistance = placeToRobot.getSquaredNorm();
    if (squaredDistance < ON_THE_PLACE_METERS * ON_THE_PLACE_METERS) {
      return 0;
    }
    double crossProduct =
        placeToRobot.getX() * fieldVelocity.getY() - placeToRobot.getY() * fieldVelocity.getX();
    return crossProduct / squaredDistance;
  }

  /**
   * What logs and reasons call it: "the Hub", "20° right of downfield", "field 90°". Its back
   * facing, it says so: "the Hub, back first".
   */
  public String name() {
    String whatItFaces =
        switch (aim) {
          case Aim.OnField onField ->
              String.format(Locale.ROOT, "field %.0f°", onField.heading().getDegrees());
          case Aim.FromDownfield fromDownfield -> downfieldName(fromDownfield.offset());
          case Aim.AtPlace atPlace -> "the " + atPlace.place().name();
        };
    return switch (end) {
      case FRONT -> whatItFaces;
      case BACK -> whatItFaces + ", back first";
    };
  }

  /** An angle from the drivers' forward as people say it. Counterclockwise is to their left. */
  private static String downfieldName(Rotation2d offset) {
    double degrees = offset.getDegrees();
    double size = Math.abs(degrees);
    if (size < NAMED_WITHIN_DEGREES) {
      return "downfield";
    }
    if (Math.abs(size - 180) < NAMED_WITHIN_DEGREES) {
      return "back toward the drivers";
    }
    String side = "right";
    if (degrees > 0) {
      side = "left";
    }
    return String.format(Locale.ROOT, "%.0f° %s of downfield", size, side);
  }

  @Override
  public boolean equals(Object other) {
    return other instanceof Heading heading && aim.equals(heading.aim) && end == heading.end;
  }

  @Override
  public int hashCode() {
    return Objects.hash(aim, end);
  }

  @Override
  public String toString() {
    return name();
  }
}
