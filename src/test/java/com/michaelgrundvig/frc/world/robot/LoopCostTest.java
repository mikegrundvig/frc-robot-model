package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import java.lang.management.ManagementFactory;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.IntSupplier;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

/**
 * What the robot asks of the model every loop makes no objects, as a table: each row is something
 * asked a few thousand times over, and the bytes the asking allocated. A robot's loop that makes
 * objects makes work for the garbage collector, and the collector's pauses are loops that run late.
 */
class LoopCostTest {
  /** Times each question is asked before anything is measured, so that its classes are loaded. */
  private static final int WARM_UP = 2000;

  private static final int MEASURED = 2000;

  /** Where the answers go, so that asking for them is work the compiler cannot leave out. */
  @SuppressWarnings("unused")
  private static volatile int SINK;

  private static final com.sun.management.ThreadMXBean THREADS =
      (com.sun.management.ThreadMXBean) ManagementFactory.getThreadMXBean();

  private static final Joint.Revolute PIVOT = StandardRobot.PIVOT;

  /** A wheel that turns freely: it has no range. */
  private static final Joint.Revolute WHEEL = StandardRobot.lightPivot("wheel", Optional.empty());

  private static Sensor.Switch limitAtZero(Joint joint) {
    return Sensor.Switch.builder()
        .label(joint.label() + " down")
        .part(joint.parent())
        .kind(new Sensor.Switch.Limit(joint, Optional.of(0.0), Sensor.Switch.Limit.End.LOWER))
        .build();
  }

  private static final Sensor.Switch PIVOT_DOWN = limitAtZero(PIVOT);
  private static final Sensor.Switch WHEEL_DOWN = limitAtZero(WHEEL);

  /**
   * The bytes this thread allocates asking a question, a time asked, on average. What a question
   * answers is added up and kept, so that the asking cannot be optimized away.
   */
  private static double bytesEachTime(IntSupplier question) {
    int answers = 0;
    for (int time = 0; time < WARM_UP; time++) {
      answers += question.getAsInt();
    }
    long thread = Thread.currentThread().threadId();
    long before = THREADS.getThreadAllocatedBytes(thread);
    for (int time = 0; time < MEASURED; time++) {
      answers += question.getAsInt();
    }
    long after = THREADS.getThreadAllocatedBytes(thread);
    SINK = answers;
    return (double) (after - before) / MEASURED;
  }

  /** One if a switch's reading is doubted, else none. */
  private static int doubted(Joint joint, Sensor.Switch limit) {
    return SwitchDoubt.of(joint, limit, 0, true) == null ? 0 : 1;
  }

  @ParameterizedTest
  @MethodSource("askedEveryLoop")
  void itMakesNoObjects(IntSupplier question) {
    // The smallest object is 16 bytes, so anything made each time shows as 16 or more.
    assertThat(bytesEachTime(question)).isLessThan(1);
  }

  private static Arguments asks(String label, IntSupplier question) {
    return argumentSet(label, question);
  }

  private static final Sensor.Rangefinder HOPPER =
      Sensor.Rangefinder.of("hopper", Part.CHASSIS, Meters.of(1.3), Degrees.of(27));

  private static final Sensor.Encoder PIVOT_ENCODER =
      MechanismFixtures.encoder("pivot encoder", PIVOT, 1, true);

  /** Hash maps, as the robot keeps them: one finds a key that is the very object it holds. */
  private static final Map<Joint, Integer> BY_JOINT =
      new HashMap<>(Map.of(PIVOT, 1, StandardRobot.ELEVATOR, 2, WHEEL, 3));

  private static final Map<Sensor, Integer> BY_SENSOR =
      new HashMap<>(Map.of(PIVOT_DOWN, 1, HOPPER, 2, PIVOT_ENCODER, 3));

  static Stream<Arguments> askedEveryLoop() {
    return Stream.of(
        asks(
            "whether a limit switch agrees with where its joint is",
            () -> doubted(PIVOT, PIVOT_DOWN)),
        asks("...for a joint that turns freely", () -> doubted(WHEEL, WHEEL_DOWN)),
        asks("the hash of a joint that turns", PIVOT::hashCode),
        asks("the hash of a joint that slides", StandardRobot.ELEVATOR::hashCode),
        asks("the hash of a switch", PIVOT_DOWN::hashCode),
        asks("the hash of a rangefinder", HOPPER::hashCode),
        asks("the hash of an encoder, which hashes its joint", PIVOT_ENCODER::hashCode),
        asks("what a map holds for a joint", () -> BY_JOINT.getOrDefault(PIVOT, 0)),
        asks(
            "what a map holds for a sensor",
            () -> BY_SENSOR.getOrDefault(HOPPER, 0) + BY_SENSOR.getOrDefault(PIVOT_DOWN, 0)));
  }
}
