package com.michaelgrundvig.frc.world.robot;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.Rule;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * The checks on a swerve drivetrain's wiring ({@link SwerveWiring}): each corner's own values,
 * under its corner's name, and the rules across them a joint's motors keep too, one kind of
 * controller and one bus.
 */
final class ModuleChecks {
  private ModuleChecks() {}

  /**
   * One corner's own values: its devices' CAN IDs, its encoder's offset within a turn, and its
   * encoder read as a module reads it, refused under its name, {@code
   * ModuleWiring[back-right].steering: CAN ID 70 must be from 0 to 62}.
   *
   * @param corner its name: "front-left"
   */
  static void ownValues(String corner, SwerveWiring.ModuleWiring module) {
    Checks.on(SwerveWiring.ModuleWiring.class, corner)
        .field("drive", module.drive(), DeviceRules.controller())
        .field("steering", module.steering(), DeviceRules.controller())
        .field("encoder", module.encoder(), DeviceRules.moduleEncoder())
        .field("encoder", module.encoder(), Rule.of(ModuleChecks::whyReadAsAJoints))
        .orThrow();
  }

  /**
   * Why a module's encoder says how a joint's loop would read it, if it does: a module's steering
   * always closes its loop on its encoder, in its own way, so a CANcoder's {@code .remote()},
   * {@code .fused()} or {@code .sync()}, or a data port's {@code .closesLoop()}, would be ignored.
   */
  private static Optional<String> whyReadAsAJoints(SwerveWiring.ModuleEncoder encoder) {
    String said = saidHowItsRead(encoder);
    if (said.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(
        said
            + ", and a module chooses how its steering reads its encoder itself, closing its loop"
            + " on it: leave "
            + said.substring(said.indexOf('.'))
            + " out");
  }

  /** How an encoder was told to be read, as written: "is read .remote()", or "" for nothing. */
  private static String saidHowItsRead(SwerveWiring.ModuleEncoder encoder) {
    return switch (encoder) {
      case MechanismWiring.Cancoder cancoder ->
          switch (cancoder.feedback()) {
            case ROTOR -> "";
            case REMOTE -> "is read .remote()";
            case FUSED -> "is read .fused()";
            case SYNC -> "is read .sync()";
          };
      case MechanismWiring.DataPortEncoder dataPort ->
          dataPort.closedLoop() ? "is told .closesLoop()" : "";
    };
  }

  /**
   * Why a drivetrain's four drives, or its four steerings, aren't driven one way, if they aren't:
   * all four say {@code .foc()} or none does, so that the modules match.
   */
  static Optional<String> whyNotOneCommutation(List<SwerveWiring.ModuleWiring> corners) {
    Optional<String> drives =
        notOneCommutation(
            "drives", corners.stream().map(SwerveWiring.ModuleWiring::drive).toList());
    if (drives.isPresent()) {
      return drives;
    }
    return notOneCommutation(
        "steerings", corners.stream().map(SwerveWiring.ModuleWiring::steering).toList());
  }

  /**
   * Why four controllers aren't driven one way, if they aren't, naming those that say FOC.
   *
   * @param which "drives" or "steerings", as the refusal names them
   * @param devices the four controllers, front left first
   */
  private static Optional<String> notOneCommutation(
      String which, List<MechanismWiring.Device> devices) {
    List<String> withFoc = new ArrayList<>();
    for (int i = 0; i < devices.size(); i++) {
      if (devices.get(i).commutation() == MechanismWiring.Commutation.FOC) {
        withFoc.add(Drivetrain.LABELS.get(i) + "'s");
      }
    }
    if (withFoc.isEmpty() || withFoc.size() == devices.size()) {
      return Optional.empty();
    }
    return Optional.of(
        "a drivetrain's "
            + which
            + " are all driven with FOC or none is: "
            + String.join(" and ", withFoc)
            + " say .foc(): say it on all four or on none");
  }

  /**
   * Why the modules' drives, or their steerings, are not all on one kind of controller, if they are
   * not: the front left's is compared with each other corner's.
   */
  static Optional<String> whyNotOneKind(List<SwerveWiring.ModuleWiring> corners) {
    SwerveWiring.ModuleWiring first = corners.get(0);
    for (int i = 1; i < corners.size(); i++) {
      SwerveWiring.ModuleWiring corner = corners.get(i);
      String label = Drivetrain.LABELS.get(i);
      if (corner.drive().controller() != first.drive().controller()) {
        return Optional.of(notOneKind("drive", first.drive(), label, corner.drive()));
      }
      if (corner.steering().controller() != first.steering().controller()) {
        return Optional.of(notOneKind("steering", first.steering(), label, corner.steering()));
      }
    }
    return Optional.empty();
  }

  /** How two corners' motors on different kinds of controller are refused. */
  private static String notOneKind(
      String which, MechanismWiring.Device frontLeft, String label, MechanismWiring.Device other) {
    return "a drivetrain's "
        + which
        + " motors are on one kind of controller: front-left's is on "
        + frontLeft.controller().shown()
        + ", and "
        + label
        + "'s on "
        + other.controller().shown();
  }

  /**
   * Why the modules' devices are not all on the gyro's bus, if they are not: Phoenix reads a
   * drivetrain's devices together, on one bus.
   */
  static Optional<String> whyNotOneBus(CanBus gyros, List<SwerveWiring.ModuleWiring> corners) {
    for (int i = 0; i < corners.size(); i++) {
      SwerveWiring.ModuleWiring corner = corners.get(i);
      List<CanBus> buses = new ArrayList<>();
      buses.add(corner.drive().bus());
      buses.add(corner.steering().bus());
      if (corner.encoder() instanceof MechanismWiring.Cancoder cancoder) {
        buses.add(cancoder.bus());
      }
      boolean allOnTheGyros = buses.stream().allMatch(gyros::equals);
      if (!allOnTheGyros) {
        return Optional.of(
            "a drivetrain's devices are on one bus, the gyro's, "
                + gyros.shown()
                + ": "
                + Drivetrain.LABELS.get(i)
                + "'s aren't");
      }
    }
    return Optional.empty();
  }

  /** Why a kind of controller cannot run a swerve module yet, if it cannot. */
  static Optional<String> whyItRunsNoModule(List<SwerveWiring.ModuleWiring> corners) {
    SwerveWiring.ModuleWiring first = corners.get(0);
    boolean drivesOnFxs = first.drive().controller() == MechanismWiring.Controller.TALON_FXS;
    boolean steersOnFxs = first.steering().controller() == MechanismWiring.Controller.TALON_FXS;
    if (drivesOnFxs || steersOnFxs) {
      return Optional.of("a TalonFXS doesn't run a swerve module in the template yet");
    }
    return Optional.empty();
  }

  /**
   * Why the modules' absolute encoders do not fit what steers them, if they do not. A TalonFX
   * steers by a CANcoder, so every module has one. A SPARK steers by the encoder on its own data
   * port, so every module's is there.
   */
  static Optional<String> whyEncodersDoNotFit(List<SwerveWiring.ModuleWiring> corners) {
    for (SwerveWiring.ModuleWiring corner : corners) {
      boolean steersByACancoder =
          corner.steering().controller() == MechanismWiring.Controller.TALON_FX;
      boolean hasACancoder = corner.encoder() instanceof MechanismWiring.Cancoder;
      if (!steersByACancoder && hasACancoder) {
        return Optional.of(
            "a SPARK steers by the absolute encoder on its data port, DATA_PORT.offset(...), not"
                + " a CANcoder");
      }
      if (steersByACancoder && !hasACancoder) {
        return Optional.of(
            "a TalonFX steers by a CANcoder: give each module its own, CANCODER.can(bus, id)"
                + ".offset(...)");
      }
    }
    return Optional.empty();
  }

  /**
   * Which CAN IDs two devices of one kind share, if any do: two TalonFXs, two SPARKs, or two
   * CANcoders. Devices of different kinds may share an ID.
   */
  static Optional<String> whyIdsAreShared(List<SwerveWiring.ModuleWiring> corners) {
    Set<String> seen = new HashSet<>();
    List<String> shared = new ArrayList<>();
    for (SwerveWiring.ModuleWiring corner : corners) {
      addIfShared(corner.drive(), seen, shared);
      addIfShared(corner.steering(), seen, shared);
      if (corner.encoder() instanceof MechanismWiring.Cancoder cancoder) {
        addIfShared("CANcoder", cancoder.id(), seen, shared);
      }
    }
    if (shared.isEmpty()) {
      return Optional.empty();
    }
    return Optional.of(String.join("; ", shared));
  }

  private static void addIfShared(
      MechanismWiring.Device device, Set<String> seen, List<String> shared) {
    addIfShared(device.controller().kind().toString(), device.id(), seen, shared);
  }

  /**
   * Notes one device, and adds a problem if another of its kind already has its ID.
   *
   * @param kind what kind of device it is, as the message says it: "TalonFX"
   * @param seen every kind and ID noted so far
   */
  private static void addIfShared(String kind, int id, Set<String> seen, List<String> shared) {
    boolean alreadyUsed = !seen.add(kind + " " + id);
    if (alreadyUsed) {
      shared.add("CAN ID " + id + " is used by two " + kind + "s");
    }
  }
}
