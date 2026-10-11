package com.michaelgrundvig.frc.world.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Every device on a robot's CAN buses, gathered from wherever the definition wires each one. It is
 * the one list that the checks across the robot's wiring read: no two devices of a kind on one bus
 * at one ID, and no PDH channel feeding two devices.
 */
final class RobotDevices {
  private RobotDevices() {}

  /**
   * The devices, each once: the PDH, then the drivetrain's, then each joint's in order, then the
   * CANdis and CANranges that the sensors are on.
   *
   * @param sensors every sensor of the robot, the joints' own included
   */
  static List<CanDevice> of(
      Optional<Pdh> pdh, List<Joint> joints, DrivetrainGeometry drivetrain, List<Sensor> sensors) {
    List<CanDevice> devices = new ArrayList<>();
    pdh.ifPresent(hub -> devices.add(device("the PDH", CanDevice.Kind.PDH, hub.bus(), hub.id())));
    drivetrain.wiring().ifPresent(wiring -> devices.addAll(wiring.devices()));
    joints.forEach(joint -> devices.addAll(joint.devices()));
    for (Sensor sensor : sensors) {
      switch (sensor) {
        case Sensor.Switch onSwitch -> addItsCandi(onSwitch, devices);
        case Sensor.Rangefinder rangefinder -> addItsCanRange(rangefinder, devices);
        case Sensor.Encoder encoder -> {}
      }
    }
    return devices;
  }

  /**
   * Adds the CANdi a switch is wired to, unless it is listed already. A CANdi has two inputs, so
   * two switches can be on one CANdi, which is still one device.
   */
  private static void addItsCandi(Sensor.Switch onSwitch, List<CanDevice> devices) {
    Optional<SensorWiring.SwitchInput> input =
        onSwitch.wiring().map(SensorWiring.Connection::input);
    if (input.isEmpty() || !(input.get() instanceof SensorWiring.Candi candi)) {
      return;
    }
    boolean alreadyListed =
        devices.stream()
            .anyMatch(
                device ->
                    device.kind() == CanDevice.Kind.CANDI
                        && device.bus().equals(candi.bus())
                        && device.id() == candi.id());
    if (!alreadyListed) {
      devices.add(device("CANdi " + candi.id(), CanDevice.Kind.CANDI, candi.bus(), candi.id()));
    }
  }

  /** Adds the CANrange a rangefinder is, if it is wired. */
  private static void addItsCanRange(Sensor.Rangefinder rangefinder, List<CanDevice> devices) {
    if (rangefinder.wiring().isEmpty()) {
      return;
    }
    SensorWiring.CanRange wiring = rangefinder.wiring().get();
    devices.add(device(rangefinder.label(), CanDevice.Kind.CANRANGE, wiring.bus(), wiring.id()));
  }

  /** A device that no PDH channel is said to feed. */
  private static CanDevice device(String name, CanDevice.Kind kind, CanBus bus, int id) {
    return new CanDevice(name, kind, bus, id, Optional.empty());
  }
}
