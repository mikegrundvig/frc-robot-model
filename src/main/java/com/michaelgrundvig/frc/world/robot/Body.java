package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.positive;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Bumpers;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import java.util.Optional;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;

/**
 * The robot as a whole: its frame, bumpers, and mass. These live here rather than under the
 * drivetrain because simulation, path planning, and rule checks all read them.
 *
 * <p>Lengths run along the robot's x axis (front to back), widths along y (side to side). The
 * robot's origin is the center of the frame, on the floor.
 *
 * <p>Its weight is the robot as weighed, bumpers and battery on, with every mechanism at its
 * joint's rest position (stowed): its links are part of it. What needs the chassis alone, the
 * simulator, takes them off ({@code RobotMath.chassis}). It's left out until the robot is weighed:
 * whatever needs a mass then takes the season's maximum in its place, which the program that runs
 * the robot on a season's field picks ({@code SeasonLimits.robotMass}), and the robot's
 * missing-data report says so. So everything here that depends on the mass takes it as given
 * ({@link #massProperties}). Its inertia is CAD's whole tensor, or its yaw alone, as PathPlanner,
 * Choreo or a rotation test gives it, or left out; what isn't given is a solid box's of its mass,
 * bumpers and overall height: never zero, or the drivetrain couldn't turn it. Its center of mass,
 * which only the simulator reads, is the frame's center, a third of its overall height up, until
 * it's given ({@link #cgOrDefault}).
 *
 * @param frameLength front to back, across the outside of the frame
 * @param frameWidth side to side, across the outside of the frame
 * @param overallHeight the robot's tallest point above the floor in its starting configuration
 * @param bumperEnds how far the bumpers stick out beyond the frame at the front and the back
 * @param bumperSides how far they stick out at the left and the right: the same as at the ends,
 *     often, but bumpers square around a frame that isn't are thicker on its shorter sides
 * @param bumperBottom how high the bumpers' bottom edge is above the floor
 * @param bumperTop how high their top edge is
 * @param weight everything that moves, robot, bumpers, and battery, with every mechanism stowed, as
 *     weighed; none until it is
 * @param cg its center of mass from the robot's origin, every mechanism stowed; none given, the
 *     frame's center, a third of its overall height up
 * @param inertia how hard it is to turn about its center of mass, along the robot's axes: CAD's
 *     whole tensor ({@link Turning#tensor}), or its yaw alone ({@link Turning#yaw}), the box giving
 *     its pitch and roll; none given, the box's all
 */
@ConfigRecord
public record Body(
    Distance frameLength,
    Distance frameWidth,
    Distance overallHeight,
    Distance bumperEnds,
    Distance bumperSides,
    Distance bumperBottom,
    Distance bumperTop,
    Optional<Mass> weight,
    Optional<Translation3d> cg,
    Optional<Turning> inertia) {
  public Body {
    Distance length = frameLength.plus(bumperEnds.times(2));
    Distance width = frameWidth.plus(bumperSides.times(2));
    Distance height = overallHeight;
    Checks.on(Body.class)
        .field("frameLength", frameLength, positive())
        .field("frameWidth", frameWidth, positive())
        .field("overallHeight", overallHeight, positive())
        .field("bumperEnds", bumperEnds, positive())
        .field("bumperSides", bumperSides, positive())
        .field("bumperBottom", bumperBottom, nonNegative())
        .field("bumperTop", bumperTop, positive())
        .field("weight", weight, given(positive()))
        .whyNot("bumperTop", () -> whyBumperTopIsMisplaced(bumperTop, bumperBottom, height))
        // Against a box of the weight given. Until the robot is weighed, whoever takes a mass for
        // it checks its inertia at that mass ({@code RobotDefinition.massProblems}).
        .whyNot(
            "inertia",
            () -> weight.flatMap(given -> whyNotABody(inertia, box(given, length, width, height))))
        .whyNot("cg", () -> cg.flatMap(given -> whyOutside(given, frameLength, frameWidth, height)))
        .orThrow();
  }

  /**
   * Why the top of the bumpers is not where it can be, if it is not: above their bottom, and no
   * higher than the robot.
   */
  private static Optional<String> whyBumperTopIsMisplaced(
      Distance bumperTop, Distance bumperBottom, Distance overallHeight) {
    if (bumperTop.gt(bumperBottom) && bumperTop.lte(overallHeight)) {
      return Optional.empty();
    }
    return Optional.of(
        shown(bumperTop)
            + " must be above the bumpers' bottom ("
            + shown(bumperBottom)
            + ") and no higher than the robot ("
            + shown(overallHeight)
            + ")");
  }

  /**
   * How hard the robot is to turn, as far as anyone knows: CAD's whole tensor, or its yaw alone.
   * What isn't given, a solid box of its mass, bumpers and overall height supplies, and the robot's
   * missing-data report says which.
   */
  public sealed interface Turning permits Turning.Tensor, Turning.Yaw {
    /** CAD's whole tensor, about the center of mass, along the robot's axes. */
    static Turning tensor(Inertia inertia) {
      return Tensor.builder().inertia(inertia).build();
    }

    /**
     * The yaw alone, as PathPlanner's or Choreo's robot MOI, CAD or a rotation test gives it: a
     * solid box gives the pitch and roll, and no products of inertia.
     */
    static Turning yaw(MomentOfInertia yaw) {
      return Yaw.builder().yaw(yaw).build();
    }

    /**
     * CAD's whole tensor.
     *
     * @param inertia about the center of mass, along the robot's axes
     */
    @ConfigRecord
    record Tensor(Inertia inertia) implements Turning {
      /** Starts describing a whole tensor, one named value at a time. */
      public static BodyTurningTensorBuilder.InertiaStage builder() {
        return BodyTurningTensorBuilder.builder();
      }
    }

    /**
     * The yaw alone.
     *
     * @param yaw how hard the robot is to spin about the vertical through its center of mass
     */
    @ConfigRecord
    record Yaw(MomentOfInertia yaw) implements Turning {
      public Yaw {
        Checks.on(Yaw.class).field("yaw", yaw, positive()).orThrow();
      }

      /** Starts describing a yaw inertia, one named value at a time. */
      public static BodyTurningYawBuilder.YawStage builder() {
        return BodyTurningYawBuilder.builder();
      }
    }
  }

  /**
   * Why the inertia given can't be the body's at a mass, if it can't ({@link #whyNotABody}). The
   * definition asks this at the weight given; until the robot is weighed, whoever takes a mass for
   * it asks at that mass.
   */
  public Optional<String> whyInertiaDoesNotFit(Mass mass) {
    return whyNotABody(inertia, box(mass));
  }

  /**
   * Why a robot's inertia cannot be a body's, if it cannot. A tensor that was given must be a
   * solid's. A yaw that was given stands beside the box's roll and pitch, and the three must be
   * able to belong to one body ({@link #whyYawDoesNotFitTheBox}). Nothing is ever adjusted to fit:
   * the answer to a yaw that does not fit is to give the whole tensor.
   */
  private static Optional<String> whyNotABody(Optional<Turning> inertia, Inertia box) {
    if (inertia.isEmpty()) {
      return Optional.empty();
    }
    return switch (inertia.get()) {
      case Turning.Tensor tensor -> Optional.ofNullable(tensor.inertia().whyNotSolid());
      case Turning.Yaw yaw -> whyYawDoesNotFitTheBox(yaw.yaw().in(KilogramSquareMeters), box);
    };
  }

  /**
   * Why a yaw inertia cannot stand beside a box's roll and pitch, if it cannot. A body's three
   * principal moments of inertia obey the triangle inequality: each is at most the sum of the other
   * two.
   *
   * @param yaw the inertia about the vertical axis, in kg·m²
   */
  private static Optional<String> whyYawDoesNotFitTheBox(double yaw, Inertia box) {
    double roll = box.ixx();
    double pitch = box.iyy();
    boolean eachIsAtMostTheOtherTwo =
        yaw <= roll + pitch && roll <= pitch + yaw && pitch <= roll + yaw;
    if (eachIsAtMostTheOtherTwo) {
      return Optional.empty();
    }
    return Optional.of(
        "a yaw of "
            + shown(yaw)
            + " kg m² can't stand beside the box's roll and pitch, "
            + shown(roll)
            + " and "
            + shown(pitch)
            + " kg m²: each must be at most the sum of the other two. Give the whole tensor");
  }

  /** A solid box of a mass, as long and wide as the bumpers and as tall as the robot. */
  private Inertia box(Mass mass) {
    return box(mass, bumperLength(), bumperWidth(), overallHeight);
  }

  /** A solid box of a mass and size. */
  private static Inertia box(Mass mass, Distance length, Distance width, Distance height) {
    return Inertia.box(mass, new Translation3d(length, width, height));
  }

  /**
   * Why a center of mass is not somewhere a robot's can be, if it is not: inside the frame, above
   * the floor and below the robot's overall height.
   */
  private static Optional<String> whyOutside(
      Translation3d cg, Distance frameLength, Distance frameWidth, Distance overallHeight) {
    boolean insideTheFrame =
        Math.abs(cg.getX()) <= frameLength.in(Meters) / 2
            && Math.abs(cg.getY()) <= frameWidth.in(Meters) / 2;
    boolean betweenFloorAndTop = cg.getZ() > 0 && cg.getZ() < overallHeight.in(Meters);
    if (insideTheFrame && betweenFloorAndTop) {
      return Optional.empty();
    }
    return Optional.of(
        cg
            + " must be inside the frame, above the floor and below the robot's overall height ("
            + shown(overallHeight)
            + ")");
  }

  /**
   * Its mass, center of mass and inertia together, at a mass: the weight given, or until the robot
   * is weighed, the one taken in its place ({@code SeasonLimits.robotMass}). What's given of its
   * inertia stands, and a solid box of the mass, as long and wide as its bumpers and as tall as the
   * robot, supplies the rest: all of it, or the pitch and roll beside a given yaw, with no products
   * of inertia.
   *
   * @param mass the robot's, bumpers and battery on
   */
  public MassProperties massProperties(Mass mass) {
    return MassProperties.builder().mass(mass).cg(cgOrDefault()).inertia(inertia(mass)).build();
  }

  /** Its inertia at a mass: what's given, and a box's of that mass for the rest. */
  private Inertia inertia(Mass mass) {
    Inertia box = box(mass);
    if (inertia.isEmpty()) {
      return box;
    }
    return switch (inertia.get()) {
      case Turning.Tensor tensor -> tensor.inertia();
      case Turning.Yaw yaw ->
          Inertia.builder()
              .ixx(box.ixx())
              .iyy(box.iyy())
              .izz(yaw.yaw().in(KilogramSquareMeters))
              .ixy(0)
              .ixz(0)
              .iyz(0)
              .build();
    };
  }

  /**
   * How hard the robot is to spin about the vertical axis through its center of mass, at a mass
   * ({@link #massProperties}).
   */
  public MomentOfInertia yawMomentOfInertia(Mass mass) {
    return KilogramSquareMeters.of(inertia(mass).izz());
  }

  /**
   * Its center of mass from the robot's origin, every mechanism stowed: as given, or until it is,
   * the frame's center, a third of the robot's overall height up.
   */
  public Translation3d cgOrDefault() {
    return cg.orElseGet(() -> new Translation3d(0, 0, overallHeight.in(Meters) / 3));
  }

  /**
   * The center of mass's height above the floor, every mechanism stowed, as {@link #cgOrDefault}.
   */
  public Distance cgHeight() {
    return Meters.of(cgOrDefault().getZ());
  }

  /** Starts building a body, one named value at a time. */
  public static BodyBuilder.FrameLengthStage builder() {
    return BodyBuilder.builder();
  }

  /** Front to back, across the outside of the bumpers. */
  public Distance bumperLength() {
    return frameLength.plus(bumperEnds.times(2));
  }

  /** Side to side, across the outside of the bumpers. */
  public Distance bumperWidth() {
    return frameWidth.plus(bumperSides.times(2));
  }

  /** The bumpers seen from above: what the field stands the robot on a start with. */
  public Bumpers bumpers() {
    return new Bumpers(bumperLength(), bumperWidth());
  }

  /** Around the outside of the frame. The game rules limit it. */
  public Distance framePerimeter() {
    return frameLength.plus(frameWidth).times(2);
  }

  /**
   * Whether a point, measured from the robot's origin, lies within the robot: inside the bumpers,
   * and between the floor and the robot's overall height.
   */
  public boolean contains(Translation3d point) {
    return Math.abs(point.getX()) <= bumperLength().in(Meters) / 2
        && Math.abs(point.getY()) <= bumperWidth().in(Meters) / 2
        && point.getZ() >= 0
        && point.getZ() <= overallHeight.in(Meters);
  }
}
