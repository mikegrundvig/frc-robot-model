package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.given;
import static com.michaelgrundvig.frc.world.config.Rules.shown;
import static org.wpilib.units.Units.Amps;

import com.michaelgrundvig.frc.world.config.Checks;
import java.util.Optional;
import org.wpilib.units.measure.Current;

/**
 * The checks a joint makes, under its own name, of the values its devices and gains leave to it:
 * its motor controllers' CAN IDs, its absolute encoder's ID and offset, its control's gains, and
 * its current limits against the breakers that feed its motors. A device or a set of gains doesn't
 * check these as it's made, since only the joint holding it can say which joint a refusal is about:
 * {@code Gains[pivot].kP: -30 must be zero or more}, {@code Motors[pivot].controllers: CAN ID 70
 * must be from 0 to 62}.
 */
final class JointWiringChecks {
  private JointWiringChecks() {}

  /**
   * Checks a joint's devices and gains, each kind under its record's name and the joint's.
   *
   * @param label the joint's name, which its refusals carry
   * @throws com.michaelgrundvig.frc.world.config.ConfigException listing everything wrong
   */
  static void check(String label, Driven driven) {
    driven
        .motors()
        .ifPresent(
            motors ->
                Checks.on(Driven.Motors.class, label)
                    .field("controllers", motors.controllers(), DeviceRules.controllers())
                    .orThrow());
    driven
        .absoluteEncoder()
        .ifPresent(
            encoder ->
                Checks.on(Driven.AbsoluteEncoder.class, label)
                    .field("cancoder", encoder.cancoder(), given(DeviceRules.cancoder()))
                    .field("dataPort", encoder.dataPort(), given(DeviceRules.dataPort()))
                    .orThrow());
    driven.control().ifPresent(control -> control.gains().checkOn(label));
    if (driven.motors().isPresent() && driven.control().isPresent()) {
      Driven.Motors motors = driven.motors().get();
      MechanismControl.CurrentLimits limits = driven.control().get().currentLimits();
      Checks.on(MechanismControl.CurrentLimits.class, label)
          .whyNot("supplyLower", () -> whyOverABreaker(motors, limits))
          .orThrow();
    }
  }

  /**
   * Why a motor's supply limit after its step-down is more than the breaker that feeds it, if it
   * is: a long push then trips the breaker. A supply limit below the step-down's is the limit
   * always, so the lower of the two is what the motor draws for long.
   */
  private static Optional<String> whyOverABreaker(
      Driven.Motors motors, MechanismControl.CurrentLimits limits) {
    Current sustained =
        limits.supply().lt(limits.supplyLower()) ? limits.supply() : limits.supplyLower();
    for (MechanismWiring.Device controller : motors.controllers()) {
      if (controller.feed().isEmpty()) {
        continue;
      }
      Current breaker = controller.feed().get().breaker();
      if (sustained.gt(breaker)) {
        return Optional.of(
            shown(sustained)
                + " is more than the "
                + shown(breaker)
                + " breaker feeding "
                + controller.controller().shown()
                + " at "
                + controller.id()
                + ", which a long push trips: give withSupplyLimit(Amps.of("
                + shown(breaker.in(Amps))
                + ")), or a bigger breaker");
      }
    }
    return Optional.empty();
  }
}
