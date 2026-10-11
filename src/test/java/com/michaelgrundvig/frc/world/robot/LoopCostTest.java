package com.michaelgrundvig.frc.world.robot;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.params.provider.Arguments.argumentSet;

import java.lang.management.ManagementFactory;
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
   * answers is added up and returned with the bytes, so that the asking cannot be optimized away.
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
    assertThat(answers).as("the questions were asked").isNotNegative();
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

  static Stream<Arguments> askedEveryLoop() {
    return Stream.of(
        asks(
            "whether a limit switch agrees with where its joint is",
            () -> doubted(PIVOT, PIVOT_DOWN)),
        asks("...for a joint that turns freely", () -> doubted(WHEEL, WHEEL_DOWN)));
  }
}
