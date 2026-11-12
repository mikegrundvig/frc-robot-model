package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.michaelgrundvig.frc.world.physical.Vector3;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

/**
 * A joint that slides along its axis, measured in meters: an elevator's carriage or stage, a
 * telescoping arm, a linear deploy.
 *
 * @param label what logs and writers call it, unique among the robot's joints
 * @param parent what it hangs from: the chassis, or another joint's link
 * @param place where its frame is in its parent's frame at position 0, where the slide starts, if
 *     its block says ({@code .at(...)}); none, at its parent's origin
 * @param axis what it slides along, in its own frame
 * @param range its hard stops, in meters
 * @param rest where it rests as the robot starts, stowed, in meters, if its block says. Without one
 *     it starts at 0.
 * @param link what it moves
 * @param positions its named positions, in meters: {@code Position.of("clear", Inches.of(20))}
 * @param outputs its named outputs: a speed in meters a second, a voltage, or a duty cycle
 * @param formerPositions the values its positions had before they were re-placed ({@code
 *     replaced}), which still count as its own: on a practice robot, the competition robot's
 * @param formerOutputs the values its outputs had before they were re-valued, which still count as
 *     its own
 * @param driven how it is driven and wired, its own sensors, and its control, as its block says
 *     them. This is not part of what makes it the same joint.
 */
@ConfigRecord
public record PrismaticJoint(
    String label,
    Part parent,
    Optional<Transform3d> place,
    Vector3 axis,
    Optional<Range> range,
    Optional<Double> rest,
    Link link,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> positions,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> outputs,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Position> formerPositions,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Output> formerOutputs,
    @RecordBuilder.Initializer(value = "NONE", source = Driven.class) Driven driven)
    implements Joint.Block<PrismaticJoint, Distance, LinearVelocity> {
  public PrismaticJoint {
    positions = List.copyOf(positions);
    outputs = List.copyOf(outputs);
    formerPositions = List.copyOf(formerPositions);
    formerOutputs = List.copyOf(formerOutputs);
    JointChecks.shared(
        PrismaticJoint.class,
        Position.Kind.LENGTH,
        label,
        place,
        axis,
        range,
        rest,
        positions,
        outputs);
    JointChecks.sliding(label, driven);
    JointWiringChecks.check(label, driven);
  }

  /** Starts describing a prismatic joint, one named value at a time. */
  static PrismaticJointBuilder.LabelStage builder() {
    return PrismaticJointBuilder.builder();
  }

  /**
   * How far its frame has moved from its origin when it is at a position, which the program asks
   * through {@link RobotMath#motion}.
   */
  Transform3d motion(double position) {
    Vector3 along = axis.unit().times(position);
    return new Transform3d(new Translation3d(along.x(), along.y(), along.z()), Rotation3d.ZERO);
  }

  @Override
  public boolean equals(@Nullable Object other) {
    return JointCopies.sameJoint(this, other);
  }

  @Override
  public int hashCode() {
    return JointCopies.identityHash(this);
  }
}
