package com.michaelgrundvig.frc.world.robot;

import java.util.List;
import java.util.function.UnaryOperator;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Transform3d;

/**
 * The copies a joint's block makes of a joint, one part changed, and what makes two joints the same
 * joint. A joint's block is the only public way to change one, so that every change goes through
 * its said-once checks. Each copy switches over the two kinds and uses the kind's generated
 * builder.
 */
final class JointCopies {
  private JointCopies() {}

  /** The same joint, its stops elsewhere. */
  static <J extends Joint.Block<J, ?, ?>> J withRange(Joint.Block<J, ?, ?> joint, Range range) {
    return copied(joint, turning -> turning.range(range), sliding -> sliding.range(range));
  }

  /** The same joint, driven otherwise. */
  static <J extends Joint.Block<J, ?, ?>> J driven(Joint.Block<J, ?, ?> joint, Driven driven) {
    return copied(joint, turning -> turning.driven(driven), sliding -> sliding.driven(driven));
  }

  /** The same joint, its frame placed elsewhere on its parent. */
  static <J extends Joint.Block<J, ?, ?>> J withPlace(
      Joint.Block<J, ?, ?> joint, Transform3d place) {
    return copied(joint, turning -> turning.place(place), sliding -> sliding.place(place));
  }

  /** The same joint, resting elsewhere, in its own units. */
  static <J extends Joint.Block<J, ?, ?>> J withRest(Joint.Block<J, ?, ?> joint, double rest) {
    return copied(joint, turning -> turning.rest(rest), sliding -> sliding.rest(rest));
  }

  /** The same joint, moving another link. */
  static <J extends Joint.Block<J, ?, ?>> J withLink(Joint.Block<J, ?, ?> joint, Link link) {
    return copied(joint, turning -> turning.link(link), sliding -> sliding.link(link));
  }

  /** The same joint with other named positions, and those it had before they were re-placed. */
  static <J extends Joint.Block<J, ?, ?>> J withPositions(
      Joint.Block<J, ?, ?> joint, List<Position> positions, List<Position> former) {
    return copied(
        joint,
        turning -> turning.positions(positions).formerPositions(former),
        sliding -> sliding.positions(positions).formerPositions(former));
  }

  /** The same joint with other named outputs, and those it had before they were re-valued. */
  static <J extends Joint.Block<J, ?, ?>> J withOutputs(
      Joint.Block<J, ?, ?> joint, List<Output> outputs, List<Output> former) {
    return copied(
        joint,
        turning -> turning.outputs(outputs).formerOutputs(former),
        sliding -> sliding.outputs(outputs).formerOutputs(former));
  }

  /** A copy of the joint with one change made through its kind's builder. */
  private static <J extends Joint.Block<J, ?, ?>> J copied(
      Joint.Block<J, ?, ?> joint,
      UnaryOperator<RevoluteJointBuilder> turning,
      UnaryOperator<PrismaticJointBuilder> sliding) {
    Joint copy =
        switch (joint) {
          case RevoluteJoint revolute ->
              turning.apply(RevoluteJointBuilder.builder(revolute)).build();
          case PrismaticJoint prismatic ->
              sliding.apply(PrismaticJointBuilder.builder(prismatic)).build();
        };
    return ofItsKind(copy);
  }

  /**
   * A copy as the kind of joint it was made from. Each kind is its own block's {@code J}: a {@code
   * RevoluteJoint} is a {@code Block<RevoluteJoint, ...>}, so a copy of one is a {@code J}.
   */
  @SuppressWarnings("unchecked") // each kind of joint is its own block's J
  private static <J extends Joint.Block<J, ?, ?>> J ofItsKind(Joint copy) {
    return (J) copy;
  }

  /**
   * Whether a joint and another object are the same joint. Two joints are the same when they are of
   * one kind and agree on everything but how they are driven, where their stops are, where they
   * rest, and where their named places are. A practice robot's joint is therefore equal to the
   * competition robot's, and stands in its place.
   */
  static boolean sameJoint(Joint joint, @Nullable Object other) {
    return other instanceof Joint that
        && joint.getClass() == that.getClass()
        && joint.label().equals(that.label())
        && joint.parent().equals(that.parent())
        && RobotMath.origin(joint).equals(RobotMath.origin(that))
        && joint.axis().equals(that.axis())
        && joint.range().isPresent() == that.range().isPresent()
        && joint.link().equals(that.link())
        && JointNames.ofPositions(joint.positions())
            .equals(JointNames.ofPositions(that.positions()))
        && JointNames.ofOutputs(joint.outputs()).equals(JointNames.ofOutputs(that.outputs()));
  }

  /**
   * A hash that two joints that are the same ({@link #sameJoint}) share: the hash of the label,
   * which each of a robot's joints has its own. The robot's loop looks joints up in maps, so
   * hashing one makes no objects: a text keeps its hash once it is worked out, where hashing the
   * whole of a joint's definition made about a kilobyte of them each time.
   */
  static int identityHash(Joint joint) {
    return joint.label().hashCode();
  }
}
