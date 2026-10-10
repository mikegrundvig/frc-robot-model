package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Rules.notBlank;
import static com.michaelgrundvig.frc.world.config.Rules.positive;

import com.michaelgrundvig.frc.world.config.Checks;
import com.michaelgrundvig.frc.world.config.ConfigRecord;
import io.soabase.recordbuilder.core.RecordBuilder;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.jspecify.annotations.Nullable;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.Measure;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * How a joint is driven, and how its devices are wired: everything its block says beyond its shape
 * and its places. Its motors or servos, its own sensors (an absolute encoder on its axle, limit
 * switches at its ends), their wiring, and how the program drives it. A joint's block writes it a
 * call at a time ({@code .motors(...)}, {@code .control(...)}); {@link #NONE} is a joint nothing
 * drives.
 *
 * <p>It isn't part of the joint's identity: a joint that differs only here is the same joint,
 * re-tuned or re-wired, so a practice robot's, or a test's, still matches everything that names it.
 *
 * <p>Each of its parts checks its own values as it's written; how they fit each other and the joint
 * (an encoder read through motors that aren't wired, soft limits past the stops, a homing recipe
 * the joint can't carry out) the robot's definition checks once the block is whole, so the order of
 * its calls never matters.
 *
 * @param motors its motors, through its gearing; empty if none turn it
 * @param servos its servos, each pushing between its parent and its link
 * @param absoluteEncoder an absolute encoder on its axle, or a sliding joint's drum, if it has one
 * @param limitSwitches the switches it presses at the ends of its travel
 * @param control how the program drives it: what its program needs of a joint with motors, which
 *     refuses one without ({@code RobotDefinition.findings}); the definition checks it fits the
 *     joint, its soft limits, homing and loop
 * @param interlocks what restricts it by where another joint is, how fast one runs, or what a
 *     sensor reads, written in its block ({@code .keepsOut(...).when(...)}, {@code
 *     .runsOnlyWhen(...)}); the robot's definition can list more beside them
 */
@ConfigRecord
public record Driven(
    Optional<Motors> motors,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Servo> servos,
    Optional<AbsoluteEncoder> absoluteEncoder,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<LimitSwitch> limitSwitches,
    Optional<MechanismControl> control,
    @RecordBuilder.Initializer(value = "of", source = List.class) List<Interlock> interlocks) {
  /** A joint nothing drives: a follower, a passive hinge, or one not described yet. */
  public static final Driven NONE = DrivenBuilder.builder().build();

  public Driven {
    servos = List.copyOf(servos);
    limitSwitches = List.copyOf(limitSwitches);
    interlocks = List.copyOf(interlocks);
    List<LimitSwitch> switches = limitSwitches;
    Checks.on(Driven.class).check("limitSwitches", () -> twice(switches)).orThrow();
  }

  /** Starts describing how a joint is driven, one named value at a time. */
  public static DrivenBuilder.DrivenBuilderStage builder() {
    return DrivenBuilder.builder();
  }

  /**
   * Motors turning a joint together through its gearing, and, for one that slides, onto a drum.
   *
   * @param motor the motor, the same for each
   * @param count how many
   * @param gearing from the motors to the joint, or to the drum
   * @param pitchRadius where the belt, chain or rack meets the drum, sprocket or pinion: a sliding
   *     joint's, empty for one that turns
   * @param controllers each motor's controller, the leader first; none for the simulator alone
   */
  @ConfigRecord
  public record Motors(
      Motor motor,
      int count,
      Gearing gearing,
      Optional<Distance> pitchRadius,
      @RecordBuilder.Initializer(value = "of", source = List.class)
          List<MechanismWiring.Device> controllers) {
    public Motors {
      controllers = List.copyOf(controllers);
      List<MechanismWiring.Device> each = controllers;
      Checks.on(Motors.class)
          .field("count", count, positive())
          .check(
              "controllers",
              () ->
                  each.isEmpty() || each.size() == count
                      ? null
                      : "names "
                          + each.size()
                          + " controllers for "
                          + count
                          + " motors: one each, or none for the simulator")
          .check("controllers", () -> mixed(motor, each))
          .orThrow();
    }

    /** Starts describing motors, one named value at a time. */
    public static DrivenMotorsBuilder.MotorStage builder() {
      return DrivenMotorsBuilder.builder();
    }

    /**
     * This many of the motor turning the joint together: {@code .count(4)}. Motors on controllers
     * need not say it, since {@link #on} counts them.
     */
    public Motors count(int count) {
      return DrivenMotorsBuilder.builder(this).count(count).build();
    }

    /**
     * The same motors, driving a sliding joint through a drum, a sprocket or a pinion: {@code
     * .drum(Inches.of(0.75))}.
     *
     * @param pitchRadius where the belt, chain or rack meets it
     */
    public Motors drum(Distance pitchRadius) {
      return DrivenMotorsBuilder.builder(this).pitchRadius(pitchRadius).build();
    }

    /** The same motors through another gearing. */
    public Motors gearing(Gearing gearing) {
      return DrivenMotorsBuilder.builder(this).gearing(gearing).build();
    }

    /**
     * The same motors, each on its controller, the leader first: {@code .on(TALON_FX.can(20),
     * TALON_FX.can(21).clockwise())}. There are as many motors as controllers. With no controllers
     * the motors keep their count and run in the simulator alone.
     */
    public Motors on(MechanismWiring.Device... controllers) {
      return DrivenMotorsBuilder.builder(this)
          .count(controllers.length == 0 ? count : controllers.length)
          .controllers(List.of(controllers))
          .build();
    }

    /** Why a joint's controllers can't be its motors', or null when they can be. */
    private static @Nullable String mixed(Motor motor, List<MechanismWiring.Device> controllers) {
      if (controllers.isEmpty()) {
        return null;
      }
      MechanismWiring.Device leader = controllers.get(0);
      for (MechanismWiring.Device device : controllers) {
        if (device.controller() != leader.controller() || !device.bus().equals(leader.bus())) {
          return "a joint's motors are on one kind of controller, on one bus: its leader is "
              + leader.controller().shown()
              + ", and CAN ID "
              + device.id()
              + " isn't the same";
        }
      }
      if (leader.controller().drives(motor)) {
        return null;
      }
      List<String> could = new ArrayList<>();
      for (MechanismWiring.Controller each : MechanismWiring.Controller.values()) {
        if (each.drives(motor)) {
          could.add(each.shown());
        }
      }
      return "its "
          + motor.name()
          + " is on "
          + leader.controller().shown()
          + ", which doesn't drive one: "
          + String.join(" or ", could)
          + " does";
    }
  }

  /**
   * A servo pushing between the joint's parent and its link: a hood's linear servo.
   *
   * @param label what logs call it: unique among the robot's drives
   * @param servo the servo
   * @param onParent where its body is fixed, from the joint's axle, in the parent's frame
   * @param onLink where its rod meets the link, in the joint's frame
   */
  @ConfigRecord
  public record Servo(
      String label, Actuator.Servo servo, Translation3d onParent, Translation3d onLink) {
    public Servo {
      Checks.on(Servo.class, label).field("label", label, notBlank()).orThrow();
    }
  }

  /**
   * An absolute encoder on the joint, turning with it: it knows where the joint is from the moment
   * the robot starts. On its axle at 1:1, or geared ({@link #ratio}); on a joint that slides, on
   * its drum's shaft, or geared to it. Read by a CANcoder, through the joint's TalonFX; on its
   * SPARK's data port; or on one of Systemcore's ports, by the robot; or by none of them, in the
   * simulator alone.
   *
   * @param ratio its turns for each of the joint's, positive: 1 on the axle, 92/18 on an 18-tooth
   *     gear against a 92-tooth ring, which way it turns being its wiring's. On a joint that
   *     slides, its turns for each of its drum's, the drum its motors' pitch radius names: 1 on the
   *     drum's shaft, 1/12 geared twelve to one down from it. Its travel must stay within one of
   *     its turns, or the joint's position can't be told from it alone
   * @param cancoder its CANcoder, if it's read through one
   * @param dataPort its SPARK's data port, if it's read there
   * @param port the Systemcore port it's on, if the robot reads it there
   */
  @ConfigRecord
  public record AbsoluteEncoder(
      double ratio,
      Optional<MechanismWiring.Cancoder> cancoder,
      Optional<MechanismWiring.DataPortEncoder> dataPort,
      Optional<MechanismWiring.PortEncoder> port) {
    public AbsoluteEncoder {
      int read =
          (cancoder.isPresent() ? 1 : 0)
              + (dataPort.isPresent() ? 1 : 0)
              + (port.isPresent() ? 1 : 0);
      Checks.on(AbsoluteEncoder.class)
          .field(
              "ratio",
              ratio,
              r ->
                  Double.isFinite(r) && r > 0
                      ? null
                      : r
                          + " must be a ratio, finite and positive: which way it turns is its"
                          + " wiring's, after its offset: a CANcoder's .clockwise(), a data"
                          + " port's or a Systemcore port's .inverted()")
          .check(
              "port",
              () ->
                  read > 1
                      ? "names more than one of a CANcoder, a data port and a Systemcore port: it's"
                          + " read one way"
                      : null)
          .orThrow();
    }

    /**
     * An absolute encoder on the joint's axle and unwired: {@code
     * AbsoluteEncoder.of().ratio(0.536).on(cancoder)}.
     */
    public static AbsoluteEncoder of() {
      return DrivenAbsoluteEncoderBuilder.builder().ratio(1).build();
    }

    /** The same encoder, geared: its turns for each of the joint's. */
    public AbsoluteEncoder ratio(double ratio) {
      return DrivenAbsoluteEncoderBuilder.builder(this).ratio(ratio).build();
    }

    /**
     * The same encoder, read another way: by a CANcoder, on the joint's SPARK's data port, or on
     * one of Systemcore's ports. An encoder that was wired the same way before keeps how the
     * joint's loop reads it, so a practice robot that only moved a CANcoder to another ID runs as
     * the competition robot does.
     */
    public AbsoluteEncoder on(MechanismWiring.EncoderWiring wiring) {
      Optional<MechanismWiring.EncoderWiring> before = wiring();
      if (before.isEmpty()) {
        return readBy(wiring);
      }
      return readBy(wiring.keepingLoopOf(before.get()));
    }

    /**
     * The same encoder, its reading offset otherwise, on the wiring it has: a practice robot's
     * magnet, {@code encoder.offset(Rotations.of(0.47))}.
     *
     * @throws IllegalArgumentException if it is not wired
     */
    public AbsoluteEncoder offset(Angle offset) {
      MechanismWiring.EncoderWiring wiring =
          wiring()
              .orElseThrow(
                  () ->
                      new IllegalArgumentException(
                          "an absolute encoder that isn't wired has no offset: the simulator alone"
                              + " reads it. Wire it, on(...)"));
      return readBy(wiring.offset(offset));
    }

    /** What reads it, or none for an encoder only the simulator reads. */
    public Optional<MechanismWiring.EncoderWiring> wiring() {
      if (cancoder.isPresent()) {
        return Optional.of(cancoder.get());
      }
      if (dataPort.isPresent()) {
        return Optional.of(dataPort.get());
      }
      if (port.isPresent()) {
        return Optional.of(port.get());
      }
      return Optional.empty();
    }

    /** The same encoder, read by this wiring and no other. */
    private AbsoluteEncoder readBy(MechanismWiring.EncoderWiring wiring) {
      DrivenAbsoluteEncoderBuilder unwired =
          DrivenAbsoluteEncoderBuilder.builder(this)
              .cancoder(Optional.empty())
              .dataPort(Optional.empty())
              .port(Optional.empty());
      return switch (wiring) {
        case MechanismWiring.Cancoder onCancoder -> unwired.cancoder(onCancoder).build();
        case MechanismWiring.DataPortEncoder onDataPort -> unwired.dataPort(onDataPort).build();
        case MechanismWiring.PortEncoder onPort -> unwired.port(onPort).build();
      };
    }

    /** Whether it's wired at all: a CANcoder, a data port or a Systemcore port. */
    public boolean wired() {
      return wiring().isPresent();
    }

    /**
     * How many of its turns one turn of its signal covers: its port's range for a potentiometer on
     * a Systemcore port, ten for a ten-turn one; 1 for every other encoder. Its sensor in the
     * robot's definition counts in turns of its signal, so a potentiometer reads as an encoder
     * geared that much slower ({@link MechanismWiring.PortEncoder#read}).
     */
    public double turnsPerSignal() {
      return port.map(MechanismWiring.PortEncoder::turnsPerRange).orElse(1.0);
    }

    /** Whether the joint's motor controller reads it: a CANcoder, or a SPARK's data port. */
    public boolean readByController() {
      return cancoder.isPresent() || dataPort.isPresent();
    }

    /**
     * Whether the joint's loop closes on it, as its wiring says, rather than on its motor's own
     * encoder told once where the joint is: a Systemcore port's, in robot code each loop, or a
     * SPARK's data port's, on the SPARK. A CANcoder's wiring says it its own way ({@link
     * MechanismWiring.Feedback}).
     */
    public boolean closedLoop() {
      return port.map(MechanismWiring.PortEncoder::closedLoop).orElse(false)
          || dataPort.map(MechanismWiring.DataPortEncoder::closedLoop).orElse(false);
    }
  }

  /**
   * A switch the joint presses at one end of its travel: {@code
   * LimitSwitch.lower().at(Degrees.of(0)).on(SensorWiring.port(1).normallyClosed())}.
   *
   * @param end which end
   * @param position where along the joint it trips, an angle on a joint that turns and a distance
   *     on one that slides; empty if that isn't known, so it can stop the joint but not home it
   * @param wiring where it's connected, or none for the simulator alone
   */
  @ConfigRecord
  public record LimitSwitch(
      Sensor.Switch.Limit.End end,
      Optional<Measure<?>> position,
      Optional<SensorWiring.LimitWiring> wiring) {
    /**
     * A switch at the lower end of a joint's travel, where it trips and its wiring still to say.
     */
    public static LimitSwitch lower() {
      return DrivenLimitSwitchBuilder.builder().end(Sensor.Switch.Limit.End.LOWER).build();
    }

    /**
     * A switch at the upper end of a joint's travel, where it trips and its wiring still to say.
     */
    public static LimitSwitch upper() {
      return DrivenLimitSwitchBuilder.builder().end(Sensor.Switch.Limit.End.UPPER).build();
    }

    /** The same switch, tripping at an angle of a joint that turns. */
    public LimitSwitch at(Angle position) {
      return DrivenLimitSwitchBuilder.builder(this).position(position).build();
    }

    /** The same switch, tripping at a distance along a joint that slides. */
    public LimitSwitch at(Distance position) {
      return DrivenLimitSwitchBuilder.builder(this).position(position).build();
    }

    /**
     * The same switch, on an input ({@code SensorWiring.port(1).normallyClosed()}, {@code
     * CANDI.can(30).s1().normallyClosed()}) or on one of its joint's own controller's limit pins
     * ({@code Direction.REVERSE.normallyClosed()}), where the controller stops the motor itself.
     */
    public LimitSwitch on(SensorWiring.LimitWiring wiring) {
      return DrivenLimitSwitchBuilder.builder(this).wiring(wiring).build();
    }
  }

  /** The ends with two limit switches, or null when each has one at most. */
  private static @Nullable String twice(List<LimitSwitch> switches) {
    Set<Sensor.Switch.Limit.End> seen = new HashSet<>();
    List<String> twice = new ArrayList<>();
    for (LimitSwitch each : switches) {
      if (!seen.add(each.end())) {
        twice.add(each.end().name().toLowerCase(java.util.Locale.ROOT));
      }
    }
    return twice.isEmpty()
        ? null
        : "two limit switches at its " + String.join(", ", twice) + " end";
  }
}
