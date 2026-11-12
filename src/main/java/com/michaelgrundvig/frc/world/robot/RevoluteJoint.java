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
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/**
 * A joint that turns about its axis, measured in radians: an arm, a wrist, a turret, a roller.
 *
 * @param label what logs and writers call it, unique among the robot's joints
 * @param parent what it hangs from: the chassis, or another joint's link
 * @param place where its frame is in its parent's frame at position 0, where it pivots, if its
 *     block says ({@code .at(...)}); none, at its parent's origin
 * @param axis what it turns about, in its own frame. A positive position turns it by the right-hand
 *     rule.
 * @param range its hard stops, in radians and not wrapped to one turn. A joint that turns freely
 *     has none.
 * @param rest where it rests as the robot starts, stowed, in radians, if its block says. Without
 *     one it starts at 0.
 * @param link what it moves
 * @param positions its named positions, in radians: {@code Position.degrees("out", 130)}
 * @param outputs its named outputs: {@code Output.rpm("intaking", 2000)}, {@code
 *     Output.volts("push", 6)}
 * @param formerPositions the values its positions had before they were re-placed ({@code
 *     replaced}), which still count as its own: on a practice robot, the competition robot's
 * @param formerOutputs the values its outputs had before they were re-valued, which still count as
 *     its own
 * @param driven how it is driven and wired, its own sensors, and its control, as its block says
 *     them. This is not part of what makes it the same joint.
 */
@ConfigRecord
public record RevoluteJoint(
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
    implements Joint.Block<RevoluteJoint, Angle, AngularVelocity> {
  public RevoluteJoint {
    positions = List.copyOf(positions);
    outputs = List.copyOf(outputs);
    formerPositions = List.copyOf(formerPositions);
    formerOutputs = List.copyOf(formerOutputs);
    JointChecks.shared(
        RevoluteJoint.class,
        Position.Kind.ANGLE,
        label,
        place,
        axis,
        range,
        rest,
        positions,
        outputs);
    JointChecks.turning(label, driven);
    JointWiringChecks.check(label, driven);
  }

  /** Starts describing a revolute joint, one named value at a time. */
  static RevoluteJointBuilder.LabelStage builder() {
    return RevoluteJointBuilder.builder();
  }

  /**
   * How far its frame has moved from its origin when it is at a position, which the program asks
   * through {@link RobotMath#motion}.
   */
  Transform3d motion(double position) {
    return new Transform3d(
        Translation3d.ZERO,
        new Rotation3d(VecBuilder.fill(axis.x(), axis.y(), axis.z()), position));
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
