package com.michaelgrundvig.frc.world.robot;

import static com.michaelgrundvig.frc.world.config.Finding.Scope.ROBOT;
import static com.michaelgrundvig.frc.world.config.Finding.Scope.SIMULATOR;
import static org.assertj.core.api.Assertions.assertThat;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Hertz;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;

import com.michaelgrundvig.frc.world.config.Finding;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation3d;

/**
 * A robot states only what it knows: what it leaves out, a value it has no data for or one only the
 * simulator reads, takes a stand-in and is said once at startup, as a note on the robot or in the
 * simulator. One row a value left out, from the fixture's robot, which states them all.
 */
class LeftOutValuesTest {
  /** A camera stating everything, at the front of the fixture's robot. */
  private static final CameraConfig STATED =
      CameraConfigBuilder.builder(ConfigFixtures.camera("front", new Translation3d(0.3, 0, 0.5)))
          .mountTolerance(CameraConfig.MountTolerance.MEASURED)
          .build();

  @Test
  void eachValueLeftOutIsNotedWhereItsRead() {
    record Row(
        String left, RobotDefinition robot, String subject, String says, Finding.Scope scope) {}
    RobotDefinition stated = ConfigFixtures.definition(List.of(STATED));
    CameraConfig.ImageSensor bare = CameraConfig.ImageSensor.of(1280, 800, Hertz.of(30));
    List<Row> rows =
        List.of(
            new Row(
                "the bumpers' thickness",
                withBody(
                    stated, BodyBuilder.builder(stated.body()).bumperThickness(Optional.empty())),
                "test",
                "the bumpers' thickness isn't given: they're taken as 3 in, the usual bumper",
                ROBOT),
            new Row(
                "the bumpers' height",
                withBody(
                    stated,
                    BodyBuilder.builder(stated.body())
                        .bumperBottom(Optional.empty())
                        .bumperTop(Optional.empty())),
                "test",
                "the bumpers' height isn't given: they're drawn from 2.5 in to 5.75 in up: the"
                    + " season's bumper zone isn't checked against them",
                ROBOT),
            new Row(
                "the backup IMU",
                RobotDefinitionBuilder.builder(stated).backupImu(Optional.empty()).build(),
                "test",
                "the robot has no backup IMU: with the gyro lost, its heading rests on the wheels"
                    + " alone",
                ROBOT),
            new Row(
                "a camera's roll, noise scale, latency and checked mount",
                withCamera(
                    stated,
                    CameraConfigBuilder.builder(STATED)
                        .roll(Optional.empty())
                        .noiseScale(Optional.empty())
                        .sensor(
                            bare.fieldOfView(Degrees.of(80), CameraConfig.Across.DIAGONAL)
                                .sightRange(Meters.of(7)))
                        .mountTolerance(CameraConfig.MountTolerance.UNCHECKED)
                        .build()),
                "front",
                "camera front: its roll is taken as 0°, level, its noise scale as 1, a typical"
                    + " camera's, its latency as 30 ms, its software's typical one, its mount as"
                    + " unchecked, as far as 25 mm and 3° off",
                ROBOT),
            new Row(
                "a camera's field of view and sight range",
                withCamera(
                    stated,
                    CameraConfigBuilder.builder(STATED)
                        .sensor(bare.latency(Milliseconds.of(30)))
                        .build()),
                "front",
                "camera front: its field of view is taken as 80° across the diagonal, its sight"
                    + " range as 7 m: what the simulator's camera sees",
                SIMULATOR),
            new Row(
                "a camera's latency, field of view and sight range, in one note",
                withCamera(stated, CameraConfigBuilder.builder(STATED).sensor(bare).build()),
                "front",
                "camera front: its latency as 30 ms, its software's typical one: the localizer"
                    + " places and weighs its images by them: the mount procedure gives its roll"
                    + " and checks its mount (docs/calibration.md), and its Vision log its latency"
                    + " and noise. In the simulator alone, its field of view is taken as 80° across"
                    + " the diagonal, its sight range as 7 m: what the simulator's camera sees, and"
                    + " the views' drawing of it, rest on them",
                ROBOT),
            new Row(
                "a joint's place",
                RobotDefinitionBuilder.builder(stated)
                    .joints(List.of(Joint.roller("intake", Part.CHASSIS)))
                    .build(),
                "intake",
                "intake isn't placed on the chassis: it's taken at the chassis' origin",
                ROBOT));

    assertThat(notes(stated))
        .as("the robot that states them all")
        .noneMatch(note -> note.message().contains("isn't given"))
        .noneMatch(note -> note.message().contains("backup IMU"))
        .noneMatch(note -> note.subject().equals("front"));
    for (Row row : rows) {
      boolean aboutTheCamera = row.subject().equals("front");
      if (aboutTheCamera) {
        assertThat(notes(row.robot()))
            .as(row.left() + ": one note for the camera")
            .filteredOn(note -> note.subject().equals("front"))
            .hasSize(1);
      }
      assertThat(notes(row.robot()))
          .as(row.left())
          .anySatisfy(
              note -> {
                assertThat(note.subject()).isEqualTo(row.subject());
                assertThat(note.message()).startsWith(row.says());
                assertThat(note.scope()).isEqualTo(row.scope());
              });
    }
  }

  /**
   * A field of view given beside a lens that sees otherwise is doubted, and the lens's is used: the
   * lens is its one owner.
   */
  @Test
  void aFieldOfViewThatDisagreesWithItsLensIsDoubted() {
    CameraConfig.Software.Lens lens =
        CameraConfig.Software.Lens.builder()
            .width(1280)
            .height(800)
            .fx(905.2)
            .fy(905.6)
            .cx(641.3)
            .cy(398.8)
            .distortion(List.of())
            .build();
    CameraConfig datasheet =
        CameraConfigBuilder.builder(STATED)
            .sensor(STATED.sensor().fieldOfView(Degrees.of(90), CameraConfig.Across.DIAGONAL))
            .software(CameraConfig.Software.visor().lens(lens))
            .build();

    assertThat(MechanismFixtures.findings(ConfigFixtures.definition(List.of(datasheet))))
        .filteredOn(finding -> finding.level() == Finding.Level.DOUBT)
        .extracting(Finding::subject, Finding::message)
        .containsExactly(
            org.assertj.core.groups.Tuple.tuple(
                "front",
                "camera front's sensor gives a field of view of 90.0° across the diagonal, but its"
                    + " lens calibration sees 79.6°: the lens's is used. Leave the sensor's out, or"
                    + " check which is right"));
  }

  private static RobotDefinition withBody(RobotDefinition robot, BodyBuilder body) {
    return RobotDefinitionBuilder.builder(robot).body(body.build()).build();
  }

  private static RobotDefinition withCamera(RobotDefinition robot, CameraConfig camera) {
    return robot.with(camera);
  }

  private static List<Finding> notes(RobotDefinition robot) {
    return MechanismFixtures.findings(robot).stream()
        .filter(finding -> finding.level() == Finding.Level.NOTE)
        .toList();
  }
}
