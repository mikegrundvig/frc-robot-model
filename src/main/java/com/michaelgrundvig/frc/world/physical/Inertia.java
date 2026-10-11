package com.michaelgrundvig.frc.world.physical;

import static com.michaelgrundvig.frc.world.config.Rules.finite;
import static com.michaelgrundvig.frc.world.config.Rules.nonNegative;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Kilograms;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Mass;

/**
 * How hard a part is to turn, about its center of mass, in its own frame: the six terms of its
 * inertia tensor, in kg·m². They're the tensor's own entries ({@code [ixx ixy ixz; ixy iyy iyz; ixz
 * iyz izz]}), as URDF, ODE and MuJoCo take them, so a product of inertia is minus the integral of,
 * say, x times y over the mass. A CAD program's mass-properties report gives them about the center
 * of mass, but check its sign: some report the integral itself.
 *
 * <p>All zero is a part whose inertia nobody has given ({@link #ZERO}): a point mass, or no mass at
 * all. What must turn as a body, the robot itself, asks for a solid one ({@link #whyNotSolid}).
 *
 * @param ixx about the part's x axis, kg·m²
 * @param iyy about its y axis, kg·m²
 * @param izz about its z axis, kg·m²
 * @param ixy the tensor's entry for x and y, kg·m²: zero when the part is symmetric across either
 *     axis
 * @param ixz its entry for x and z, kg·m²
 * @param iyz its entry for y and z, kg·m²
 */
@ConfigRecord
public record Inertia(double ixx, double iyy, double izz, double ixy, double ixz, double iyz) {
  /** None: a part whose inertia isn't known, or a point mass. Turning it takes nothing. */
  public static final Inertia ZERO =
      InertiaBuilder.builder().ixx(0).iyy(0).izz(0).ixy(0).ixz(0).iyz(0).build();

  /**
   * How far a tensor built from others' sums may miss a bound, as a share of its size: a few
   * roundings, which a zero or a point mass turned on its frame comes out with.
   */
  private static final double ROUNDING = 1e-12;

  public Inertia {
    Checks.on(Inertia.class)
        .field("ixx", ixx, nonNegative())
        .field("iyy", iyy, nonNegative())
        .field("izz", izz, nonNegative())
        .field("ixy", ixy, finite())
        .field("ixz", ixz, finite())
        .field("iyz", iyz, finite())
        .whyNot("ixx", () -> whyNotABodysMoments(ixx, iyy, izz))
        .whyNot("ixy", () -> whyProductsAreTooLarge(ixx, iyy, izz, ixy, ixz, iyz))
        .orThrow();
  }

  /**
   * Why three moments of inertia cannot be one body's, if they cannot.
   *
   * <p>A moment about an axis adds up each bit of mass times the square of its distance from that
   * axis: about x, that is y² + z². So the moments about y and z together add up x² + y² + 2z²,
   * which is never less than the moment about x, and likewise for the other two. This is the
   * triangle inequality of moments of inertia.
   */
  private static Optional<String> whyNotABodysMoments(double ixx, double iyy, double izz) {
    double slack = ROUNDING * (ixx + iyy + izz);
    boolean eachAtMostTheOtherTwo =
        ixx <= iyy + izz + slack && iyy <= ixx + izz + slack && izz <= ixx + iyy + slack;
    if (eachAtMostTheOtherTwo) {
      return Optional.empty();
    }
    return Optional.of(
        "("
            + shown(ixx)
            + ", "
            + shown(iyy)
            + ", "
            + shown(izz)
            + ") can't be a body's: each must be at most the sum of the other two");
  }

  /**
   * Why the products of inertia are too large for the moments, if they are.
   *
   * <p>No body takes less than no effort to turn about any axis, which is to say that its tensor is
   * positive semidefinite. That is judged here by the tensor's three second-order principal minors,
   * one for each pair of axes: each product of inertia, squared, is at most the product of its two
   * axes' moments. All zero passes, for a part with no inertia given, and so does a point mass,
   * which has none about its center.
   *
   * <p>The full test would also ask that the tensor's determinant is not negative. That is not
   * asked, and never has been: a tensor that passes the three minors and has a negative determinant
   * is accepted. Asking it would refuse tensors that are accepted today.
   */
  private static Optional<String> whyProductsAreTooLarge(
      double ixx, double iyy, double izz, double ixy, double ixz, double iyz) {
    double size = ixx + iyy + izz;
    double slack = ROUNDING * size * size;
    double minorXy = ixx * iyy - ixy * ixy;
    double minorXz = ixx * izz - ixz * ixz;
    double minorYz = iyy * izz - iyz * iyz;
    boolean noMinorIsNegative = minorXy >= -slack && minorXz >= -slack && minorYz >= -slack;
    if (noMinorIsNegative) {
      return Optional.empty();
    }
    return Optional.of(
        "the products of inertia are too large for the moments: no body turns that way");
  }

  /**
   * Why it isn't a solid body's, or null when it is: one that takes effort to turn about every
   * axis, as a whole robot does. A part may have none ({@link #ZERO}); the robot can't, or its
   * drivetrain couldn't turn it.
   */
  public @Nullable String whyNotSolid() {
    return whyItTurnsTooEasily().orElse(null);
  }

  /**
   * The reason it is not a solid body's, if it is not. A solid body's tensor is positive definite,
   * which by Sylvester's criterion is when its leading principal minors are all positive: ixx, the
   * minor of x and y, and the determinant. The moments about y and z are asked to be positive too,
   * which that already implies.
   */
  private Optional<String> whyItTurnsTooEasily() {
    boolean momentsArePositive = ixx > 0 && iyy > 0 && izz > 0;
    boolean minorXyIsPositive = ixx * iyy - ixy * ixy > 0;
    if (momentsArePositive && minorXyIsPositive && determinant() > 0) {
      return Optional.empty();
    }
    return Optional.of(
        "("
            + shown(ixx)
            + ", "
            + shown(iyy)
            + ", "
            + shown(izz)
            + ") turns too easily: a whole body takes effort to turn about every axis");
  }

  /** Whether it's all zero: none given. */
  public boolean isZero() {
    return ixx == 0 && iyy == 0 && izz == 0 && ixy == 0 && ixz == 0 && iyz == 0;
  }

  /** The tensor's determinant, expanded along its first row. */
  private double determinant() {
    return ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz);
  }

  /** Starts describing an inertia, one named value at a time. */
  public static InertiaBuilder.IxxStage builder() {
    return InertiaBuilder.builder();
  }

  /**
   * The same body's inertia along turned axes: its own axes turned by a rotation, as a part turned
   * on its mount is seen from what it's mounted on ({@code R I R}<sup>T</sup>).
   */
  public Inertia rotated(Rotation3d rotation) {
    double[] turn = rotation(rotation);
    double[] tensor = entries();
    double[] turned = new double[9];
    for (int row = 0; row < 3; row++) {
      for (int column = 0; column < 3; column++) {
        // The entry of R I Rᵀ in this row and column: the sum, over every entry of I, of that
        // entry times R's entry in this row and times R's entry in this column.
        double sum = 0;
        for (int k = 0; k < 3; k++) {
          for (int l = 0; l < 3; l++) {
            sum += turn[at(row, k)] * tensor[at(k, l)] * turn[at(column, l)];
          }
        }
        turned[at(row, column)] = sum;
      }
    }
    return of(turned);
  }

  /**
   * How hard it is to turn about an axis through its center of mass, in kg·m²: with the axis as a
   * unit vector n, the tensor's quadratic form, n<sup>T</sup> I n.
   */
  public double about(Vector3 axis) {
    Vector3 unit = axis.unit();
    double[] tensor = entries();
    double[] n = {unit.x(), unit.y(), unit.z()};
    double sum = 0;
    for (int row = 0; row < 3; row++) {
      for (int column = 0; column < 3; column++) {
        sum += n[row] * tensor[at(row, column)] * n[column];
      }
    }
    return sum;
  }

  /** Where a matrix's entry in a row and a column is, in its nine entries listed row by row. */
  static int at(int row, int column) {
    return 3 * row + column;
  }

  /** The tensor's nine entries, row by row. */
  double[] entries() {
    return new double[] {ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz};
  }

  /**
   * An inertia from a tensor's nine entries, row by row, made symmetric.
   *
   * @throws com.michaelgrundvig.frc.world.config.ConfigException if no body has it
   */
  static Inertia of(double[] tensor) {
    // A tensor is symmetric, but one that was summed or turned may differ across its diagonal by
    // a rounding, so each pair of entries across it is averaged.
    return builder()
        .ixx(tensor[at(0, 0)])
        .iyy(tensor[at(1, 1)])
        .izz(tensor[at(2, 2)])
        .ixy((tensor[at(0, 1)] + tensor[at(1, 0)]) / 2)
        .ixz((tensor[at(0, 2)] + tensor[at(2, 0)]) / 2)
        .iyz((tensor[at(1, 2)] + tensor[at(2, 1)]) / 2)
        .build();
  }

  /** A rotation's matrix, row by row. */
  static double[] rotation(Rotation3d rotation) {
    var matrix = rotation.toMatrix();
    double[] entries = new double[9];
    for (int row = 0; row < 3; row++) {
      for (int column = 0; column < 3; column++) {
        entries[at(row, column)] = matrix.get(row, column);
      }
    }
    return entries;
  }

  /**
   * A solid box's, of a mass spread evenly through it, about its center, its axes along its sides:
   * the provisional stand-in until CAD or a measurement gives the real one. A robot or a link isn't
   * solid and even, so this is a guess, and its register row says so.
   *
   * <p>About each axis it is a twelfth of the mass times the sum of the squares of the two sides
   * across that axis.
   *
   * @param size its length along x, width along y, and height along z, in meters
   */
  public static Inertia box(Mass mass, Translation3d size) {
    double twelfthOfMass = mass.in(Kilograms) / 12;
    double lengthSquared = size.getX() * size.getX();
    double widthSquared = size.getY() * size.getY();
    double heightSquared = size.getZ() * size.getZ();
    return builder()
        .ixx(twelfthOfMass * (widthSquared + heightSquared))
        .iyy(twelfthOfMass * (lengthSquared + heightSquared))
        .izz(twelfthOfMass * (lengthSquared + widthSquared))
        .ixy(0)
        .ixz(0)
        .iyz(0)
        .build();
  }
}
