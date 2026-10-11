package com.michaelgrundvig.frc.world.robot;

import static org.wpilib.units.Units.Seconds;

import java.util.Optional;
import org.wpilib.units.measure.Time;

/**
 * How long homing a joint should take, worked out from the joint's model: how far it may have to
 * go, and how fast the gentle push that homing uses moves it. A homing recipe that gives no timeout
 * of its own is allowed this long, with a margin, before the joint is faulted.
 */
final class HomingTime {
  private HomingTime() {}

  /**
   * How many times over the model's time a joint is allowed. Twice leaves room for friction, and
   * for gravity, to take half the push.
   */
  private static final double TIMES_THE_MODEL = 2;

  /**
   * What is allowed on top of that, in seconds. A joint pushed into a hard stop is only taken to be
   * there once it has been still for a quarter of a second.
   */
  private static final double SECONDS_TO_SPARE = 0.5;

  /**
   * How long a joint takes to cross its travel when pushed at a voltage, in seconds.
   *
   * <p>The ideal kV is the volts for each unit a second of the joint's speed, so a push of some
   * volts moves the joint at volts / kV, and the travel takes travel · kV / volts.
   *
   * @param volts the push
   * @throws IllegalArgumentException if no motors move the joint, or it follows another joint
   */
  static double travelSeconds(RobotMath math, Joint joint, double volts) {
    return farthestToGo(joint) * math.idealGains(joint).kV() / volts;
  }

  /**
   * The longest a recipe may push a joint before it is faulted, in seconds: the timeout the recipe
   * gives, or, when it gives none, twice the model's time and half a second more.
   *
   * @param timeout the recipe's own timeout, if it gives one
   * @param volts the push
   */
  static double secondsAllowed(Optional<Time> timeout, double volts, RobotMath math, Joint joint) {
    if (timeout.isPresent()) {
      return timeout.get().in(Seconds);
    }
    return TIMES_THE_MODEL * travelSeconds(math, joint, volts) + SECONDS_TO_SPARE;
  }

  /**
   * The farthest homing can have to move a joint, in the joint's units. A joint with hard stops may
   * have to cross its whole range. A joint that turns freely reaches anywhere within one turn. A
   * slide with no hard stops is only ever between its named places and where it rests.
   */
  private static double farthestToGo(Joint joint) {
    if (joint.range().isPresent()) {
      return joint.range().get().width();
    }
    if (joint instanceof Joint.Revolute) {
      return 2 * Math.PI;
    }
    return spreadOfItsPlaces(joint);
  }

  /** How far apart the lowest and highest of a joint's named places and its rest position lie. */
  private static double spreadOfItsPlaces(Joint joint) {
    double lowest = joint.restPosition();
    double highest = joint.restPosition();
    for (Position place : joint.positions()) {
      lowest = Math.min(lowest, place.value());
      highest = Math.max(highest, place.value());
    }
    return highest - lowest;
  }
}
