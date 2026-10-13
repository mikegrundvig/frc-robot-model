package com.michaelgrundvig.frc.world.record;

import static com.michaelgrundvig.frc.world.robot.RecordedRobot.ARM;
import static com.michaelgrundvig.frc.world.robot.RecordedRobot.ELEVATOR;
import static com.michaelgrundvig.frc.world.robot.RecordedRobot.PIVOT;
import static com.michaelgrundvig.frc.world.robot.RecordedRobot.ROBOT;
import static com.michaelgrundvig.frc.world.robot.RecordedRobot.ROLLER;
import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.world.robot.RecordedRobot;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * The wording of how a move, a move of the drivetrain and an auto ended. This is the one test of
 * each sentence: every other test reads the facts.
 */
class MoveSentencesTest {
  private static final double NOT_INSIDE = Double.NaN;

  /** One joint's part of a move that went to a place: the pivot's "out" is its place 2. */
  private static MoveOutcome toPlace(
      MoveResult result, int joint, double seconds, double degrees, MoveReason reason) {
    return part(result, MoveTask.GO_TO, MoveProgress.BEGUN, seconds, joint, MoveTarget.PLACE)
        .at(Math.toRadians(degrees))
        .because(reason, Identity.NONE)
        .build();
  }

  private static Part part(
      MoveResult result,
      MoveTask task,
      MoveProgress progress,
      double seconds,
      int joint,
      MoveTarget target) {
    return new Part(result, task, progress, seconds, joint, target);
  }

  /** Builds one joint's part of a move, a value at a time, so a row says only what it is about. */
  private static final class Part {
    private final MoveResult result;
    private final MoveTask task;
    private final MoveProgress progress;
    private final double seconds;
    private final int joint;
    private final MoveTarget target;
    private int configuration = Identity.NONE;
    private double targetNumber = Double.NaN;
    private double position;
    private double velocity;
    private double arrivedSeconds = NOT_INSIDE;
    private MoveReason reason = MoveReason.NONE;
    private int reasonJoint = Identity.NONE;

    Part(
        MoveResult result,
        MoveTask task,
        MoveProgress progress,
        double seconds,
        int joint,
        MoveTarget target) {
      this.result = result;
      this.task = task;
      this.progress = progress;
      this.seconds = seconds;
      this.joint = joint;
      this.target = target;
    }

    Part at(double position) {
      this.position = position;
      return this;
    }

    Part moving(double velocity) {
      this.velocity = velocity;
      return this;
    }

    Part chasing(double targetNumber) {
      this.targetNumber = targetNumber;
      return this;
    }

    Part of(int configuration) {
      this.configuration = configuration;
      return this;
    }

    Part insideAfter(double arrivedSeconds) {
      this.arrivedSeconds = arrivedSeconds;
      return this;
    }

    Part because(MoveReason reason, int reasonJoint) {
      this.reason = reason;
      this.reasonJoint = reasonJoint;
      return this;
    }

    MoveOutcome build() {
      return new MoveOutcome(
          result,
          task,
          progress,
          seconds,
          joint,
          configuration,
          target,
          Identity.NONE,
          targetNumber,
          position,
          velocity,
          arrivedSeconds,
          reason,
          reasonJoint);
    }
  }

  private static String said(String name, MoveOutcome part, String why) {
    return MoveSentences.outcome(ROBOT, name, List.of(part), List.of(), why);
  }

  @Test
  void aMoveOfOneJointSaysHowItEndedHowLongAndWhereItWas() {
    assertThat(
            said(
                "pivot → out", toPlace(MoveResult.ARRIVED, PIVOT, 0.414, 130, MoveReason.NONE), ""))
        .isEqualTo("pivot → out: ARRIVED in 0.41 s");
    assertThat(
            said(
                "pivot → out",
                toPlace(MoveResult.TIMED_OUT, PIVOT, 1.62, 1.5, MoveReason.NONE),
                ""))
        .isEqualTo("pivot → out: TIMED_OUT after 1.62 s, pivot at 1.5°");
    assertThat(
            said(
                "pivot → out",
                toPlace(MoveResult.CANCELED, PIVOT, 0.2, 38.49, MoveReason.ROBOT_DISABLED),
                ""))
        .isEqualTo("pivot → out: CANCELED after 0.20 s, pivot at 38.49°: the robot was disabled");
  }

  /** A joint run at an output, and a joint that was homed, have no place to say they were at. */
  @Test
  void aMoveThatPositionsNoJointSaysNoPlace() {
    MoveOutcome intaking =
        part(MoveResult.CANCELED, MoveTask.RUN, MoveProgress.BEGUN, 0.98, ROLLER, MoveTarget.OUTPUT)
            .build();
    MoveOutcome homing =
        part(MoveResult.CANCELED, MoveTask.HOME, MoveProgress.BEGUN, 0.5, PIVOT, MoveTarget.NONE)
            .build();

    assertThat(said("roller: intaking", intaking, ""))
        .isEqualTo("roller: intaking: CANCELED after 0.98 s");
    assertThat(said("pivot homes", homing, "")).isEqualTo("pivot homes: CANCELED after 0.50 s");
  }

  @Test
  void aMoveThatTimedOutSaysWhatHeldEachJoint() {
    MoveOutcome arm = toPlace(MoveResult.TIMED_OUT, ARM, 2.5, -12, MoveReason.NONE);
    MoveOutcome pivot = toPlace(MoveResult.TIMED_OUT, PIVOT, 2.5, 0.2, MoveReason.NONE);
    List<Holder> held = List.of(Holder.interlock(1), Holder.limitSwitch(0));

    assertThat(MoveSentences.outcome(ROBOT, "arm and pivot → up", List.of(arm, pivot), held, ""))
        .isEqualTo(
            "arm and pivot → up: TIMED_OUT after 2.50 s, arm at -12°, pivot at 0.2°,"
                + " held: arm keeps out of -10° to 30° while elevator at most 0.35 m,"
                + " held: pivot down tripped");
  }

  /** A canceled move says where its joints were, and never what held them. */
  @Test
  void aCanceledMoveDoesNotSayWhatHeldIt() {
    MoveOutcome arm = toPlace(MoveResult.CANCELED, ARM, 0.3, -12, MoveReason.NONE);

    assertThat(
            MoveSentences.outcome(
                ROBOT, "arm → up", List.of(arm), List.of(Holder.interlock(1)), ""))
        .isEqualTo("arm → up: CANCELED after 0.30 s, arm at -12°");
  }

  @Test
  void aMoveOfSeveralJointsSaysWhenEachArrived() {
    MoveOutcome elevator =
        part(
                MoveResult.ARRIVED,
                MoveTask.GO_TO,
                MoveProgress.BEGUN,
                0.72,
                ELEVATOR,
                MoveTarget.PLACE)
            .of(0)
            .insideAfter(0.72)
            .build();
    MoveOutcome arm =
        part(MoveResult.ARRIVED, MoveTask.GO_TO, MoveProgress.BEGUN, 0.72, ARM, MoveTarget.PLACE)
            .of(0)
            .insideAfter(0.41)
            .build();

    assertThat(elevator.isOfAConfiguration()).isTrue();
    assertThat(
            MoveSentences.outcome(
                ROBOT, "elevator and arm → stowed", List.of(elevator, arm), List.of(), ""))
        .isEqualTo("elevator and arm → stowed: ARRIVED in 0.72 s (elevator 0.72 s, arm 0.41 s)");
  }

  @Test
  void aFollowSaysWhatItsJointWasChasing() {
    MoveOutcome position =
        part(MoveResult.CANCELED, MoveTask.FOLLOW, MoveProgress.BEGUN, 1, PIVOT, MoveTarget.NUMBER)
            .at(Math.toRadians(62))
            .chasing(Math.toRadians(70))
            .build();
    MoveOutcome speed =
        part(MoveResult.CANCELED, MoveTask.FOLLOW, MoveProgress.BEGUN, 1, ROLLER, MoveTarget.SPEED)
            .moving(2990 * 2 * Math.PI / 60)
            .chasing(3000 * 2 * Math.PI / 60)
            .build();
    MoveOutcome nothingYetOnAPivot =
        part(
                MoveResult.CANCELED,
                MoveTask.FOLLOW,
                MoveProgress.NOT_BEGUN,
                1,
                PIVOT,
                MoveTarget.NONE)
            .at(Math.toRadians(62))
            .build();
    MoveOutcome nothingYetOnARoller =
        part(
                MoveResult.CANCELED,
                MoveTask.FOLLOW,
                MoveProgress.NOT_BEGUN,
                1,
                ROLLER,
                MoveTarget.NONE)
            .moving(0)
            .build();

    assertThat(said("pivot follows a target", position, ""))
        .isEqualTo("pivot follows a target: CANCELED after 1.00 s, pivot at 62°, its target 70°");
    assertThat(said("roller follows a target", speed, ""))
        .isEqualTo(
            "roller follows a target: CANCELED after 1.00 s, roller at 2990 rpm,"
                + " its target 3000 rpm");
    assertThat(said("pivot follows a target", nothingYetOnAPivot, ""))
        .isEqualTo("pivot follows a target: CANCELED after 1.00 s, pivot at 62°");
    assertThat(said("roller follows a target", nothingYetOnARoller, ""))
        .isEqualTo("roller follows a target: CANCELED after 1.00 s, roller at 0 rpm");
  }

  /** A reason from the closed set is said from the facts; any other is the text beside them. */
  @Test
  void aReasonIsSaidAfterTheRest() {
    assertThat(
            refused(
                MoveTask.GO_TO, MoveProgress.NOT_BEGUN, MoveReason.GATE_FALSE, "clear is false"))
        .isEqualTo("pivot → out: REFUSED: clear is false");
    assertThat(refused(MoveTask.GO_TO, MoveProgress.NOT_BEGUN, MoveReason.JOINT_NOT_ANSWERING, ""))
        .isEqualTo("pivot → out: REFUSED: pivot isn't answering: its motor controllers are silent");
    assertThat(refused(MoveTask.GO_TO, MoveProgress.BEGUN, MoveReason.JOINT_NOT_ANSWERING, ""))
        .isEqualTo(
            "pivot → out: REFUSED: can't go on: pivot isn't answering: its motor controllers are"
                + " silent");
    assertThat(refused(MoveTask.GO_TO, MoveProgress.BEGUN, MoveReason.JOINT_FAULTED, "it slipped"))
        .isEqualTo("pivot → out: REFUSED: can't go on: it slipped");
    assertThat(refused(MoveTask.HOME, MoveProgress.BEGUN, MoveReason.JOINT_FAULTED, "no stop"))
        .isEqualTo("pivot → out: REFUSED: no stop");
    assertThat(refused(MoveTask.GO_TO, MoveProgress.BEGUN, MoveReason.POSITION_LOST, ""))
        .isEqualTo(
            "pivot → out: REFUSED: can't go on: pivot's position stopped being a number: check its"
                + " encoder and its motor controller's settings");
    assertThat(refused(MoveTask.GO_TO, MoveProgress.NOT_BEGUN, MoveReason.NO_DRIVER, ""))
        .isEqualTo("pivot → out: REFUSED: " + MoveSentences.NO_DRIVER);
  }

  @Test
  void aSilentJointIsSaidTheSameWayWhereverItIsSaid() {
    assertThat(MoveSentences.notAnswering(RecordedRobot.joint(PIVOT)))
        .isEqualTo("pivot isn't answering: its motor controllers are silent");
  }

  /** A record names a sensor by where it is among every sensor, which is the model's own order. */
  @Test
  void everySensorIsTheListTheRobotsMathReads() {
    assertThat(ROBOT.everySensor())
        .containsExactly(RecordedRobot.PIVOT_DOWN, RecordedRobot.CLAW_BEAM)
        .isEqualTo(ROBOT.math().sensors());
  }

  private static String refused(
      MoveTask task, MoveProgress progress, MoveReason reason, String why) {
    MoveOutcome part =
        part(MoveResult.REFUSED, task, progress, 0, PIVOT, MoveTarget.PLACE)
            .because(reason, PIVOT)
            .build();
    return said("pivot → out", part, why);
  }

  /** A move that moves no joint the robot drives has one part, which is no joint's. */
  @Test
  void aMoveOfNoJointsSaysOnlyThatItWasRefused() {
    MoveOutcome none =
        part(
                MoveResult.REFUSED,
                MoveTask.GO_TO,
                MoveProgress.NOT_BEGUN,
                0,
                Identity.NONE,
                MoveTarget.NONE)
            .because(MoveReason.REFUSED_AS_BUILT, Identity.NONE)
            .build();

    assertThat(said("climber → up", none, "climber has no mechanism"))
        .isEqualTo("climber → up: REFUSED: climber has no mechanism");
  }

  @Test
  void aMoveOfTheDrivetrainSaysWhereTheRobotGotTo() {
    assertThat(drive(MoveResult.ARRIVED, DriveMoveTask.TURN_TO, 0.84, MoveReason.NONE, ""))
        .isEqualTo("drive → there: ARRIVED in 0.84 s");
    assertThat(drive(MoveResult.TIMED_OUT, DriveMoveTask.TURN_TO, 0.2, MoveReason.NONE, ""))
        .isEqualTo("drive → there: TIMED_OUT after 0.20 s, heading at 14.0°");
    assertThat(drive(MoveResult.TIMED_OUT, DriveMoveTask.TO_POSE, 0.2, MoveReason.NONE, ""))
        .isEqualTo("drive → there: TIMED_OUT after 0.20 s, at (0.31, 0.12) facing 14.0°");
    assertThat(
            drive(
                MoveResult.TIMED_OUT,
                DriveMoveTask.RUN_UNTIL,
                1,
                MoveReason.CONDITION_NEVER_HELD,
                "bumped is false"))
        .isEqualTo("drive → there: TIMED_OUT after 1.00 s: bumped is false");
    assertThat(
            drive(MoveResult.CANCELED, DriveMoveTask.TO_PLACE, 5.98, MoveReason.ROBOT_DISABLED, ""))
        .isEqualTo("drive → there: CANCELED after 5.98 s: the robot was disabled");
    assertThat(
            drive(
                MoveResult.REFUSED,
                DriveMoveTask.TO_PLACE,
                0,
                MoveReason.PATH_NOT_PLANNED,
                "alliance unknown"))
        .isEqualTo("drive → there: REFUSED: alliance unknown");
    assertThat(
            drive(MoveResult.REFUSED, DriveMoveTask.WITH_THE_STICKS, 0, MoveReason.NO_DRIVER, ""))
        .isEqualTo("drive → there: REFUSED: " + MoveSentences.NO_DRIVER);
  }

  private static String drive(
      MoveResult result, DriveMoveTask task, double seconds, MoveReason reason, String why) {
    DriveOutcome outcome =
        new DriveOutcome(result, task, seconds, 0.314, 0.123, Math.toRadians(14), reason);
    return MoveSentences.driveOutcome("drive → there", outcome, why);
  }

  @Test
  void anAutoSaysHowItsRunWent() {
    assertThat(auto(AutoOutcome.Result.RUNNING, 0, "")).isEqualTo("Left Double: running");
    assertThat(auto(AutoOutcome.Result.COMPLETED, 9.8, ""))
        .isEqualTo("Left Double: COMPLETED in 9.80 s");
    assertThat(auto(AutoOutcome.Result.CUT_OFF, 15, ""))
        .isEqualTo("Left Double: CUT OFF after 15.00 s, as autonomous ended");
    assertThat(auto(AutoOutcome.Result.CANCELED, 3.25, ""))
        .isEqualTo("Left Double: CANCELED after 3.25 s");
    assertThat(auto(AutoOutcome.Result.FAILED, 0, "it couldn't be built: no path"))
        .isEqualTo("Left Double: FAILED after 0.00 s: it couldn't be built: no path");
  }

  private static String auto(AutoOutcome.Result result, double seconds, String why) {
    return MoveSentences.autoOutcome("Left Double", new AutoOutcome(result, seconds), why);
  }
}
