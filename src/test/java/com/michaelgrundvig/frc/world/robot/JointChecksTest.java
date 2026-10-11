package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThatCode;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.stream.Stream;
import org.assertj.core.api.ThrowableAssert.ThrowingCallable;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * What a joint refuses of its own shape and of how it is driven, as a table of attempts and the
 * exact messages. The refusals of a place, a speed or a range of the wrong kind are in {@link
 * PlacesTest}, beside the other refusals of named places.
 */
class JointChecksTest {
  private static final Translation3d AXLE = new Translation3d(0.25, 0, 0.32);
  private static final Transform3d NOWHERE =
      new Transform3d(new Translation3d(Double.NaN, 0, 0.3), Rotation3d.ZERO);
  private static final Transform3d HERE = new Transform3d(Translation3d.ZERO, Rotation3d.ZERO);
  private static final Vector3 NO_WAY = new Vector3(0, 0, 0);
  private static final Vector3 NOT_A_WAY = new Vector3(Double.NaN, 0, 1);

  private static final double NONE = Double.POSITIVE_INFINITY;

  private static Transform3d at(double x, double y, double z) {
    return new Transform3d(new Translation3d(x, y, z), Rotation3d.ZERO);
  }

  /**
   * A transform turned by a quaternion with one part infinite: WPILib scales it to length 1, which
   * leaves that one part not a number and the others zero.
   */
  private static Transform3d turned(double w, double x, double y, double z) {
    return new Transform3d(Translation3d.ZERO, new Rotation3d(new Quaternion(w, x, y, z)));
  }

  private static Joint.Revolute pivotAt(Transform3d origin, Vector3 axis) {
    return Joint.Revolute.builder()
        .label("pivot")
        .parent(Part.CHASSIS)
        .origin(origin)
        .axis(axis)
        .link(Link.unseen("pivot"))
        .build();
  }

  private static Joint.Prismatic elevator() {
    return Joint.elevator("elevator", Part.CHASSIS, AXLE);
  }

  @ParameterizedTest
  @MethodSource({"shapes", "drives"})
  void itRefusesSayingExactly(ThrowingCallable attempt, String message) {
    assertThatIllegalArgumentException().isThrownBy(attempt).withMessage(message);
  }

  @ParameterizedTest
  @MethodSource("taken")
  void itTakes(ThrowingCallable attempt) {
    assertThatCode(attempt).doesNotThrowAnyException();
  }

  private static Arguments refuses(String label, ThrowingCallable attempt, String message) {
    return argumentSet(label, attempt, message);
  }

  // spotless:off

  /** A joint is somewhere, and its axis points some way. */
  static Stream<Arguments> shapes() {
    return Stream.of(
        refuses("an origin that is not a number",
            () -> pivotAt(NOWHERE, Joint.PITCH_UP),
            "Revolute[pivot].origin: " + NOWHERE + " must be a transform, numbers all"),
        refuses("an origin nowhere along y",
            () -> pivotAt(at(0, Double.NaN, 0), Joint.PITCH_UP),
            "Revolute[pivot].origin: " + at(0, Double.NaN, 0) + " must be a transform, numbers all"),
        refuses("an origin nowhere along z",
            () -> pivotAt(at(0, 0, Double.NEGATIVE_INFINITY), Joint.PITCH_UP),
            "Revolute[pivot].origin: " + at(0, 0, Double.NEGATIVE_INFINITY) + " must be a transform, numbers all"),
        refuses("an origin turned no way by its quaternion's W",
            () -> pivotAt(turned(NONE, 0, 0, 0), Joint.PITCH_UP),
            "Revolute[pivot].origin: " + turned(NONE, 0, 0, 0) + " must be a transform, numbers all"),
        refuses("...by its X",
            () -> pivotAt(turned(0, NONE, 0, 0), Joint.PITCH_UP),
            "Revolute[pivot].origin: " + turned(0, NONE, 0, 0) + " must be a transform, numbers all"),
        refuses("...by its Y",
            () -> pivotAt(turned(0, 0, NONE, 0), Joint.PITCH_UP),
            "Revolute[pivot].origin: " + turned(0, 0, NONE, 0) + " must be a transform, numbers all"),
        refuses("...by its Z",
            () -> pivotAt(turned(0, 0, 0, NONE), Joint.PITCH_UP),
            "Revolute[pivot].origin: " + turned(0, 0, 0, NONE) + " must be a transform, numbers all"),
        refuses("an axis of no length",
            () -> pivotAt(HERE, NO_WAY),
            "Revolute[pivot].axis: " + NO_WAY + " must point some way: an axis can't be zero"),
        refuses("an axis that is not a number",
            () -> pivotAt(HERE, NOT_A_WAY),
            "Revolute[pivot].axis: " + NOT_A_WAY + " must point some way: an axis can't be zero"));
  }

  /** A joint that turns is geared alone; one that slides winds onto a drum, and has no gravity angle. */
  static Stream<Arguments> drives() {
    return Stream.of(
        refuses("a drum on a joint that turns",
            () -> Joint.pivot("pivot", Part.CHASSIS, AXLE, Joint.PITCH_UP)
                .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).drum(Inches.of(1))),
            "Revolute[pivot].driven.motors: a joint that turns has no drum: give its motors a"
                + " gearing alone"),
        refuses("no drum on a joint that slides",
            () -> elevator().motors(Motor.KRAKEN_X60.through(Gearing.of(10))),
            "Prismatic[elevator].driven.motors: a joint that slides needs its drum: give its"
                + " motors the pitch radius"),
        refuses("a gravity angle on a joint that slides",
            () -> elevator().control(MechanismControl.position(30).withKg(0.3, Degrees.of(0))),
            "Prismatic[elevator].driven.control: a joint that slides is held against gravity the"
                + " same all along it, so its kG has no angle: give kG alone, withKg(kG)"));
  }

  static Stream<Arguments> taken() {
    return Stream.of(
        argumentSet("an origin as far away as numbers go, whose numbers add up to more than a number holds",
            (ThrowingCallable) () -> pivotAt(at(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE), Joint.PITCH_UP)),
        argumentSet("an axis of any length but zero",
            (ThrowingCallable) () -> pivotAt(HERE, new Vector3(0, 0, 1e-9))),
        argumentSet("a slide with a drum",
            (ThrowingCallable) () -> elevator().motors(Motor.KRAKEN_X60.through(Gearing.of(10)).drum(Inches.of(1)))),
        argumentSet("a slide with no motors has no drum to need",
            (ThrowingCallable) JointChecksTest::elevator));
  }

  // spotless:on
}
