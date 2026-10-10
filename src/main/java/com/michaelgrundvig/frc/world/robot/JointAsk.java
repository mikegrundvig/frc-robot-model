package com.michaelgrundvig.frc.world.robot;

/**
 * What a joint is asked to do in one loop, in the joint's own units (radians or meters). The same
 * type says what the joint is allowed to do once its interlocks have been applied ({@link
 * InterlockEnforcement}): an interlock that holds a joint replaces what was asked with a position
 * to hold, or with neutral.
 */
public sealed interface JointAsk permits JointAsk.ToPosition, JointAsk.AtOutput, JointAsk.Neutral {
  /** A share of 1 is the joint's whole cruise speed. An interlock's hold always moves at this. */
  double FULL_CRUISE = 1;

  /** Neutral, which has nothing to say beyond itself, so one value serves everywhere. */
  Neutral NEUTRAL = new Neutral();

  /**
   * Go to a position along the joint's motion profile.
   *
   * @param position where to go
   * @param cruiseShare the share of the joint's cruise speed to move at, from 0 to 1
   */
  record ToPosition(double position, double cruiseShare) implements JointAsk {}

  /**
   * Run at an output rather than go to a position: a speed, a voltage, or a share of the battery.
   * Every output drives the joint one way or the other by its sign, and each would run the joint at
   * some speed, which is what a keep-out needs to know to stop the joint in time.
   */
  sealed interface AtOutput extends JointAsk permits AtVelocity, AtVoltage, AtDutyCycle {
    /** The output itself. A positive value drives the joint toward the high end of its travel. */
    double value();

    /**
     * The speed this output runs the joint at with nothing loading it, in the joint's units a
     * second.
     *
     * @param freeSpeed the speed a full battery runs the unloaded joint at ({@link
     *     Travel#freeSpeed})
     */
    double unloadedSpeed(double freeSpeed);
  }

  /**
   * Run at a speed, on the motor controller's velocity loop.
   *
   * @param speed the speed, in the joint's units a second
   */
  record AtVelocity(double speed) implements AtOutput {
    @Override
    public double value() {
      return speed;
    }

    @Override
    public double unloadedSpeed(double freeSpeed) {
      return speed;
    }
  }

  /**
   * Put a voltage across the joint's motors.
   *
   * @param volts the voltage
   */
  record AtVoltage(double volts) implements AtOutput {
    @Override
    public double value() {
      return volts;
    }

    @Override
    public double unloadedSpeed(double freeSpeed) {
      return volts / Motor.NOMINAL_VOLTS * freeSpeed;
    }
  }

  /**
   * Put a share of the battery's voltage across the joint's motors.
   *
   * @param share the share, from -1 to 1
   */
  record AtDutyCycle(double share) implements AtOutput {
    @Override
    public double value() {
      return share;
    }

    @Override
    public double unloadedSpeed(double freeSpeed) {
      return share * freeSpeed;
    }
  }

  /** Drive nothing: the joint's motors brake or coast, as its control says. */
  record Neutral() implements JointAsk {}
}
