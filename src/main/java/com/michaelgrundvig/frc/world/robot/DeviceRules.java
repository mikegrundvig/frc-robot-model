package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.between;
import static org.wpilib.units.Units.Rotations;

import com.michaelgrundvig.frc.world.config.Rule;
import java.util.List;
import java.util.Optional;
import org.wpilib.units.measure.Angle;

/**
 * The rules a device's values keep wherever it's written: a CAN ID on the bus, and an absolute
 * encoder's offset within a turn. The device itself doesn't check them. What holds it does, a
 * joint's motors and absolute encoder or a swerve module's corner, so that a refusal names the
 * joint or the corner: {@code ModuleWiring[back-right].steering: CAN ID 70 must be from 0 to 62}.
 */
final class DeviceRules {
  private DeviceRules() {}

  /** The IDs a CAN bus gives its devices. */
  private static final Rule<Double> ON_THE_BUS = between(0, 62);

  /** How far an absolute encoder's offset may go: a turn either way. */
  private static final Rule<Angle> WITHIN_A_TURN = between(Rotations.of(-1), Rotations.of(1));

  /** A motor controller's CAN ID, on the bus. */
  static Rule<MechanismWiring.Device> controller() {
    return Rule.of(device -> whyNotOnTheBus(device.id()));
  }

  /** Each of a joint's motor controllers' CAN IDs, on the bus: the first that isn't, if any. */
  static Rule<List<MechanismWiring.Device>> controllers() {
    return Rule.of(
        devices ->
            devices.stream()
                .map(device -> whyNotOnTheBus(device.id()))
                .flatMap(Optional::stream)
                .findFirst());
  }

  /** A CANcoder's CAN ID on the bus, and its offset within a turn. */
  static Rule<MechanismWiring.Cancoder> cancoder() {
    return Rule.of(
        cancoder -> whyNotOnTheBus(cancoder.id()).or(() -> whyNotWithinATurn(cancoder.offset())));
  }

  /** A data-port encoder's offset, within a turn. */
  static Rule<MechanismWiring.DataPortEncoder> dataPort() {
    return Rule.of(encoder -> whyNotWithinATurn(encoder.offset()));
  }

  /** A swerve module's absolute encoder, a CANcoder or one on its steering SPARK's data port. */
  static Rule<SwerveWiring.ModuleEncoder> moduleEncoder() {
    return Rule.of(
        encoder ->
            switch (encoder) {
              case MechanismWiring.Cancoder cancoder -> cancoder().reason(cancoder);
              case MechanismWiring.DataPortEncoder dataPort -> dataPort().reason(dataPort);
            });
  }

  /** Why a CAN ID isn't one a device on the bus can have, if it isn't: "CAN ID 70 must be ...". */
  private static Optional<String> whyNotOnTheBus(int id) {
    return ON_THE_BUS.reason((double) id).map(why -> "CAN ID " + why);
  }

  /** Why an encoder's offset is more than a turn, if it is: "offset 1.5 R must be ...". */
  private static Optional<String> whyNotWithinATurn(Angle offset) {
    return WITHIN_A_TURN.reason(offset).map(why -> "offset " + why);
  }
}
