package com.michaelgrundvig.frc.world.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Optional;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Distance;

/**
 * What a joint's block makes for the robot: the drives, sensors, CAN devices and interlocks that
 * the block declares in the joint's own words, each as the robot's definition lists them.
 *
 * <p>A block says "these motors, through this gearing", and the definition needs a {@link Drive}
 * with a {@link Transmission}. A block says "a limit switch at the lower end", and the definition
 * needs a {@link Sensor} with a label. This type does that translation, so that a joint's block and
 * the robot's own lists of drives and sensors end up as the same things.
 */
final class JointParts {
  private JointParts() {}

  /**
   * A joint's own drives: its motors, labeled as the joint is, and each of its servos under the
   * servo's own label.
   */
  static List<Drive> drives(Joint joint) {
    List<Drive> drives = new ArrayList<>();
    joint.driven().motors().ifPresent(motors -> drives.add(motorDrive(joint, motors)));
    for (Driven.Servo servo : joint.driven().servos()) {
      drives.add(servoDrive(joint, servo));
    }
    return drives;
  }

  private static Drive motorDrive(Joint joint, Driven.Motors motors) {
    return Drive.builder()
        .label(joint.label())
        .actuator(Actuator.Motors.builder().motor(motors.motor()).count(motors.count()).build())
        .transmission(transmission(joint, motors))
        .build();
  }

  /**
   * What carries the motors' turning to the joint: a gearing alone for a joint that turns, and a
   * gearing onto a drum for one that slides.
   */
  private static Transmission transmission(Joint joint, Driven.Motors motors) {
    return switch (joint) {
      case Joint.Revolute turning ->
          Transmission.Geared.builder().joint(turning).gearing(motors.gearing()).build();
      case Joint.Prismatic sliding ->
          Transmission.Drum.builder()
              .joint(sliding)
              .gearing(motors.gearing())
              .pitchRadius(motors.pitchRadius().orElseThrow())
              .build();
    };
  }

  /**
   * A servo's drive: it pushes between the joint's parent and the joint's link. The block gives
   * where the servo's body is fixed from the joint's origin, and the transmission wants it in the
   * parent's frame, so it is moved there.
   */
  private static Drive servoDrive(Joint joint, Driven.Servo servo) {
    return Drive.builder()
        .label(servo.label())
        .actuator(servo.servo())
        .transmission(
            Transmission.Between.builder()
                .a(joint.parent())
                .onA(
                    joint
                        .origin()
                        .getTranslation()
                        .plus(servo.onParent().rotateBy(joint.origin().getRotation())))
                .b(joint)
                .onB(servo.onLink())
                .build())
        .build();
  }

  /**
   * A joint's own sensors: its absolute encoder ("arm absolute encoder") and its limit switches
   * ("arm lower limit", "arm upper limit"), each with its wiring.
   */
  static List<Sensor> sensors(Joint joint) {
    List<Sensor> sensors = new ArrayList<>();
    absoluteEncoder(joint).ifPresent(sensors::add);
    for (Driven.LimitSwitch limit : joint.driven().limitSwitches()) {
      sensors.add(limitSwitch(joint, limit));
    }
    return sensors;
  }

  /**
   * A joint's absolute encoder as a sensor, if the joint has one that can be read as its position.
   *
   * <p>On a joint that turns, the encoder turns with the joint. On a joint that slides, it turns
   * with the drum, so its reading means a position only once the joint's motors say the drum's
   * pitch radius. Until they do, the joint has no such sensor, and the robot's definition refuses
   * the encoder ({@code Blocks}).
   */
  private static Optional<Sensor> absoluteEncoder(Joint joint) {
    Optional<Driven.AbsoluteEncoder> encoder = joint.driven().absoluteEncoder();
    if (encoder.isEmpty()) {
      return Optional.empty();
    }
    Optional<Distance> drum = drumOf(joint);
    boolean turnsWithADrum = joint instanceof Joint.Prismatic;
    if (turnsWithADrum && drum.isEmpty()) {
      return Optional.empty();
    }
    double turnsForEachOfTheJoints = encoder.get().ratio() / encoder.get().turnsPerSignal();
    return Optional.of(
        Sensor.Encoder.builder()
            .label(joint.label() + " absolute encoder")
            .terms(
                List.of(
                    Transmission.Term.builder()
                        .joint(joint)
                        .ratio(turnsForEachOfTheJoints)
                        .pitchRadius(drum)
                        .build()))
            .absolute(true)
            .build());
  }

  /** The pitch radius of the drum a sliding joint's motors wind onto. A turning joint has none. */
  private static Optional<Distance> drumOf(Joint joint) {
    return switch (joint) {
      case Joint.Revolute turning -> Optional.empty();
      case Joint.Prismatic sliding -> sliding.driven().motors().flatMap(Driven.Motors::pitchRadius);
    };
  }

  private static Sensor limitSwitch(Joint joint, Driven.LimitSwitch limit) {
    String label = joint.label() + " " + lowerOrUpper(limit.end()) + " limit";
    Sensor.Switch unwired =
        limit
            .position()
            .map(at -> Sensor.Switch.limit(label, joint, limit.end(), inJointUnits(at)))
            .orElseGet(() -> Sensor.Switch.limit(label, joint, limit.end()));
    if (limit.wiring().isEmpty()) {
      return unwired;
    }
    return unwired.on(limit.wiring().get().of(joint));
  }

  /**
   * A joint's limit switch at one end of its travel.
   *
   * @throws IllegalArgumentException if the joint has none at that end
   */
  static Sensor.Switch switchAt(Joint joint, Sensor.Switch.Limit.End end) {
    for (Sensor sensor : sensors(joint)) {
      if (sensor instanceof Sensor.Switch onSwitch
          && onSwitch.kind() instanceof Sensor.Switch.Limit limit
          && limit.end() == end) {
        return onSwitch;
      }
    }
    throw new IllegalArgumentException(
        joint.label() + " has no limit switch at its " + lowerOrUpper(end) + " end");
  }

  /**
   * A joint's devices on the CAN bus: its motor controllers, the leader first, each named for its
   * ID ("pivot motor 32") and carrying the PDH channel that feeds it; then its CANcoder ("pivot
   * CANcoder"). A joint whose motors are on no controllers runs in the simulator alone and has
   * none, and its CANcoder is not listed either, since it is read through those controllers.
   */
  static List<CanDevice> devices(Joint joint) {
    List<CanDevice> devices = new ArrayList<>();
    List<MechanismWiring.Device> controllers =
        joint.driven().motors().map(Driven.Motors::controllers).orElse(List.of());
    for (MechanismWiring.Device controller : controllers) {
      devices.add(
          new CanDevice(
              joint.label() + " motor " + controller.id(),
              controller.controller().kind(),
              controller.bus(),
              controller.id(),
              controller.feed()));
    }
    Optional<MechanismWiring.Cancoder> cancoder =
        joint.driven().absoluteEncoder().flatMap(Driven.AbsoluteEncoder::cancoder);
    if (cancoder.isPresent() && !controllers.isEmpty()) {
      devices.add(
          new CanDevice(
              joint.label() + " CANcoder",
              CanDevice.Kind.CANCODER,
              cancoder.get().bus(),
              cancoder.get().id(),
              Optional.empty()));
    }
    return devices;
  }

  /**
   * A joint's own interlocks, each about the joint as it is now. A block's interlock was written
   * about the joint as it was at that point in the block, before the calls that followed it, so
   * each is made again about the finished joint.
   */
  static List<Interlock> interlocks(Joint joint) {
    List<Interlock> interlocks = new ArrayList<>();
    for (Interlock written : joint.driven().interlocks()) {
      interlocks.add(
          switch (written) {
            case Interlock.RunsOnlyWhen runsOnlyWhen ->
                Interlock.that(joint).runsOnlyWhen(runsOnlyWhen.condition());
            case Interlock.KeepsOut keepsOut ->
                Interlock.that(joint).keepsOut(keepsOut.region()).when(keepsOut.condition());
          });
    }
    return interlocks;
  }

  /** An end of a joint's travel as a sentence names it: "lower" or "upper". */
  static String lowerOrUpper(Sensor.Switch.Limit.End end) {
    return end.name().toLowerCase(Locale.ROOT);
  }

  /**
   * A measure of a joint's position or speed as a number in the joint's units: radians, meters,
   * radians a second or meters a second. Those are the units WPILib keeps every angle, distance and
   * speed in, so the number is the measure's own magnitude in its base unit.
   */
  static double inJointUnits(Measure<?> measure) {
    return measure.baseUnitMagnitude();
  }
}
