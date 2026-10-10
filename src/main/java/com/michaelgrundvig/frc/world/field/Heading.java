package com.michaelgrundvig.frc.world.field;

import static org.wpilib.units.Units.Radians;

import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
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

  private enum Kind {
    FIELD,
    DOWNFIELD,
    TOWARD
  }

  private final Kind kind;

  /** The field heading for {@link Kind#FIELD}, or the angle from the drivers' forward. */
  private final Rotation2d angle;

  private final @Nullable Place place;
  private final boolean back;

  private Heading(Kind kind, Rotation2d angle, @Nullable Place place, boolean back) {
    this.kind = kind;
    this.angle = angle;
    this.place = place;
    this.back = back;
  }

  /** A fixed heading on the field, whatever the alliance: counterclockwise from +x. */
  public static Heading field(Rotation2d heading) {
    return new Heading(Kind.FIELD, heading, null, false);
  }

  /**
   * An angle counterclockwise from the drivers' forward ({@link FieldModel#driverForward}): zero
   * faces away from the drivers, whichever alliance they're on.
   */
  public static Heading downfield(Angle offset) {
    return new Heading(Kind.DOWNFIELD, new Rotation2d(offset.in(Radians)), null, false);
  }

  /**
   * Facing a place as it is for the robot's alliance ({@link Place#on}), from wherever the robot
   * is. On top of it, there is no bearing.
   */
  public static Heading toward(Place place) {
    return new Heading(Kind.TOWARD, Rotation2d.ZERO, place, false);
  }

  /** The same, the robot's back facing it instead of its front. */
  public Heading withBack() {
    return new Heading(kind, angle, place, true);
  }

  private Translation2d target(FieldModel field, Alliance alliance) {
    return Objects.requireNonNull(place).on(field, alliance).getTranslation();
  }

  /**
   * The heading the robot should have, as of where it is.
   *
   * @return empty if it needs the alliance and it isn't known, or the robot is on the place
   */
  public Optional<Rotation2d> resolve(Pose2d robot, FieldModel field, Optional<Alliance> alliance) {
    Optional<Rotation2d> facing =
        switch (kind) {
          case FIELD -> Optional.of(angle);
          case DOWNFIELD -> alliance.map(ours -> FieldModel.driverForward(ours).rotateBy(angle));
          case TOWARD -> bearing(robot, field, alliance);
        };
    return back ? facing.map(heading -> heading.rotateBy(Rotation2d.PI)) : facing;
  }

  private Optional<Rotation2d> bearing(
      Pose2d robot, FieldModel field, Optional<Alliance> alliance) {
    if (alliance.isEmpty()) {
      return Optional.empty();
    }
    Translation2d to = target(field, alliance.get()).minus(robot.getTranslation());
    return to.getNorm() < ON_THE_PLACE_METERS
        ? Optional.empty()
        : Optional.of(new Rotation2d(to.getX(), to.getY()));
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
    if (kind != Kind.TOWARD || alliance.isEmpty()) {
      return 0;
    }
    Translation2d from = robot.getTranslation().minus(target(field, alliance.get()));
    double squared = from.getSquaredNorm();
    if (squared < ON_THE_PLACE_METERS * ON_THE_PLACE_METERS) {
      return 0;
    }
    return (from.getX() * fieldVelocity.getY() - from.getY() * fieldVelocity.getX()) / squared;
  }

  /**
   * What logs and reasons call it: "the Hub", "20° right of downfield", "field 90°". Its back
   * facing, it says so: "the Hub, back first".
   */
  public String name() {
    String shown =
        switch (kind) {
          case FIELD -> String.format(Locale.ROOT, "field %.0f°", angle.getDegrees());
          case DOWNFIELD -> downfieldName();
          case TOWARD -> "the " + Objects.requireNonNull(place).name();
        };
    return back ? shown + ", back first" : shown;
  }

  private String downfieldName() {
    double degrees = angle.getDegrees();
    if (Math.abs(degrees) < 0.05) {
      return "downfield";
    }
    if (Math.abs(Math.abs(degrees) - 180) < 0.05) {
      return "back toward the drivers";
    }
    return String.format(
        Locale.ROOT, "%.0f° %s of downfield", Math.abs(degrees), degrees > 0 ? "left" : "right");
  }

  @Override
  public boolean equals(Object other) {
    return other instanceof Heading heading
        && kind == heading.kind
        && angle.equals(heading.angle)
        && Objects.equals(place, heading.place)
        && back == heading.back;
  }

  @Override
  public int hashCode() {
    return Objects.hash(kind, angle, place, back);
  }

  @Override
  public String toString() {
    return name();
  }
}
