package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Inertia;
import com.michaelgrundvig.frc.world.physical.MassProperties;
import com.michaelgrundvig.frc.world.physical.Material;
import com.michaelgrundvig.frc.world.physical.Solid;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.List;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;

/**
 * What a joint moves: an arm, a carriage, a wrist with what it holds. Everything here is in its
 * joint's frame, which moves with the joint.
 *
 * @param label what logs and writers call it: unique among the robot's links
 * @param mass its mass, center of mass and inertia, in its joint's frame: from CAD's
 *     mass-properties report, or weighed on a scale. None known is {@code MassProperties.NONE},
 *     zero, never a guess: its joint then has no gravity to hold (no kG seed), it adds nothing to
 *     the robot's center of mass, and its joint's kA seeds from the motors' rotors alone. The
 *     robot's missing-data report lists it ({@code RobotDefinition.missing}).
 * @param shapes its solid shape, simplified by hand from CAD: what interlocks and the extension
 *     limit are checked against, and what the views draw. None: it isn't drawn or checked.
 */
@ConfigRecord
public record Link(
    String label,
    MassProperties mass,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Solid> shapes) {
  public Link {
    Checks.on(Link.class).field("label", label, notBlank()).orThrow();
    shapes = List.copyOf(shapes);
  }

  /**
   * A link of known mass properties, from CAD's mass-properties report: {@code Link.of("arm",
   * MassProperties.of(Pounds.of(6), cg, inertia))}. It has no shape until {@link #shapes} gives
   * one.
   *
   * @param label what logs and writers call it: unique among the robot's links
   * @param mass its mass, center of mass and inertia, in its joint's frame
   */
  public static Link of(String label, MassProperties mass) {
    return builder().label(label).mass(mass).build();
  }

  /**
   * The same link, drawn and checked as these solids: {@code .shapes(List.of(box))}.
   *
   * @param shapes its solid shape, simplified by hand from CAD: what interlocks and the extension
   *     limit are checked against, and what the views draw
   * @throws IllegalArgumentException if it has its shapes already
   */
  public Link shapes(List<Solid> shapes) {
    if (!this.shapes.isEmpty()) {
      throw new IllegalArgumentException(label + " has its shapes already: give them once");
    }
    return LinkBuilder.builder(this).shapes(shapes).build();
  }

  /** Starts describing a link, one named value at a time. */
  private static LinkBuilder.LabelStage builder() {
    return LinkBuilder.builder();
  }

  /**
   * A link nobody has weighed, drawn and checked as a box centered at a point in its joint's frame,
   * as long (along x), wide (along y) and tall (along z) as given: {@code Link.box("shooter
   * wheels", Translation3d.ZERO, Inches.of(3), Inches.of(19.7), Inches.of(3))}. Its mass is none,
   * never a guess.
   */
  public static Link box(
      String label, Translation3d center, Distance length, Distance width, Distance height) {
    return builder()
        .label(label)
        .mass(MassProperties.NONE)
        .shapes(List.of(box(center, new Translation3d(length, width, height))))
        .build();
  }

  /** A link nobody has weighed or drawn: where its shape would be a guess on a guess. */
  public static Link unseen(String label) {
    return builder().label(label).mass(MassProperties.NONE).build();
  }

  /**
   * A link of a known mass, its center at a point in its joint's frame, as hard to turn as a solid
   * box as long (along x), wide (along y) and tall (along z) as given there, and drawn so: CAD's or
   * a scale's mass, without CAD's inertia. {@code Link.weighed("arm", Pounds.of(6), new
   * Translation3d(0.3, 0, 0), Inches.of(24), Inches.of(2), Inches.of(2))}.
   */
  public static Link weighed(
      String label, Mass mass, Translation3d cg, Distance length, Distance width, Distance height) {
    Translation3d size = new Translation3d(length, width, height);
    return builder()
        .label(label)
        .mass(MassProperties.of(mass, cg, Inertia.box(mass, size)))
        .shapes(List.of(box(cg, size)))
        .build();
  }

  private static Solid box(Translation3d center, Translation3d size) {
    return Solid.Box.of(new Pose3d(center, Rotation3d.ZERO), size, Material.ALUMINUM);
  }
}
