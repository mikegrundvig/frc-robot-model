# FRC robot model

What an FRC robot and its field are, written once as data, for everything that needs to know them:
the robot program, a simulator, localization, and the tools around them. It's a Java library on
WPILib's math and units, and nothing else of a robot program: no HAL, no NetworkTables, no clock
and no state, so a robot, a simulator and a replay all read the same model.

It's unreleased: there are no versions yet, and anything may change.

## What's in it

| Package (`com.michaelgrundvig.frc.world`) | Holds |
|---|---|
| `robot` | The robot's definition: its body, drivetrain, joints and what drives them, sensors, cameras and IMUs, and the rules between them (interlocks, conditions, named places and outputs); and what a joint's definition means for a program, as functions of plain values (where it is driven, what its interlocks allow it each loop, how far its encoder and motor may disagree, which limits its controller reads). |
| `field` | The field: its tags, the places and zones code names, headings, starts, terrain, the elements that stand on it, and a venue's measured layout. |
| `physical` | What both are made of: mass and inertia, shapes and materials. |
| `config` | The validation kit every record is built and checked with. A definition that can't work is refused as it's built, saying why; one that's doubtful says so. |
| `path` | Timed paths for a drivetrain: keyframes at poses or the field's places, and the trajectories planned through them or along a line under the limits they're given. |
| `game` | A season's game: what its manual allows a robot to be, checked against its definition, and its field (`Rebuilt` for 2026, `Reefscape`, `Crescendo`), written once when the game is revealed. A new season's is added on kickoff day, in a new version of the model. |
| `bridges` | Writers for other tools, such as AdvantageScope's robot assets. |

A joint, as a definition writes it:

```java
static final Joint.Revolute PIVOT =
    Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
        .range(Range.degrees(0, 135))
        .rest(PIVOT_IN)
        .positions(PIVOT_IN, PIVOT_OUT)
        .motors(Motor.KRAKEN_X60.through(Gearing.of(25)).on(TALON_FX.can(32)))
        .control(MechanismControl.position(30));

static final Joint.Revolute ROLLER =
    Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35))
        .outputs(INTAKING)
        .motors(Motor.KRAKEN_X60.through(Gearing.of(3)).on(TALON_FX.can(31)))
        // The roller runs only once the pivot is out of the frame.
        .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
        .control(MechanismControl.velocity(0.02));
```

That's `src/test/java/com/michaelgrundvig/frc/world/examples/ExampleJoints.java`, which the build
compiles and the Javadoc shows.

## What stands on a field

A field has two kinds of structure, and a thing is one or the other. What a robot drives over is
terrain: a bump, a ramp, a platform, a low plate. What a robot runs into is an element: a hub, a
tower, a trench's column, a speaker built into the wall. An element is made of upright blocks, each
an outline seen from above standing from one height to another, and it carries the AprilTags that
hang on it.

A season's field writes what stands on blue's half, and the model adds red's:

```java
static List<Element> elements() {
  Polygon2d hub =
      Polygon2d.rectangle(new Pose2d(HUB_X, WIDTH / 2, Rotation2d.ZERO), HUB_SIZE, HUB_SIZE);
  return List.of(
      Element.standing("Hub", hub, HUB_HEIGHT, STRUCTURE, 18, 19, 20, 21, 24, 25, 26, 27),
      trench("Right trench", 0, TRENCH_OPENING, BUMP_EDGE, 17, 28));
}

FieldModel.builder(layout, SYMMETRY).surfaces(surfaces()).elements(elements()).build();
```

The field then has "Hub (blue)" and "Hub (red)", the second carrying the partners of the first's
tags. Something across the center line, such as 2025's barge, is written once with `shared`. As the
field is built it refuses an element whose tag isn't in the layout or doesn't hang on it, a block
wholly off the field, and a shared element that isn't its own image.

Whatever reads an element asks it, and works nothing out again: `footprint()` is what it covers seen
from above, `heightMeters()` how tall it is, and `inTheWayOf(lowest, highest)` the outlines of what
something between two heights runs into, so a robot's bumpers meet a trench's column and not the
bar it drives under. The walls around the field are the field's own `outline()`, not elements. None
of it needs the simulator: a path that must go around a hub reads the same footprints.

Elements also block sight. `field.hides(...)` says whether anything stands between two points, in
3D: the line is cut against each block's outline seen from above, and then held against the
block's heights over that stretch, so a line passes under a trench's bar and over a low panel.
`field.hidesOnAFace(eye, point, outOfTheFace)` asks it for a point on a face, such as a corner of
an AprilTag from a camera. A tag hangs on its own element, a few millimeters proud of the face or
sunk into it as the layout has it, so that line ends 3 cm in front of the face. The tag's own
element then hides it from behind and never from in front, and a view along the face doesn't
flicker. A simulated camera asks this of every tag before it takes an image. Terrain doesn't block
sight: nothing a robot drives over is tall enough to matter.

A block stands straight up. What slopes or leans (a subwoofer's inclined panels, a reef's angled
branches, a stage's chains) is left out, and each season's field says what it left out. Elements
are nominal geometry from the manual: a venue's file surveys tags, not elements, just as it doesn't
survey terrain.

`docs-processor/` is an annotation processor that goes with it. As a project compiles, it writes
each configuration record's Javadoc into the jar, so a tool can show what a value means in the
words the code already has.

## Using it

It is published to the FRC libraries' Maven repository,
[mikegrundvig.github.io/frc-maven](https://mikegrundvig.github.io/frc-maven/), as
`com.michaelgrundvig.frc:frc-robot-model`, with its processor as
`com.michaelgrundvig.frc:frc-docs-processor`. Versions are dated releases (`2026.10.21`), shared by
all the FRC libraries; they are pre-release, with no promise of compatibility from one to the next.

```groovy
// build.gradle
repositories {
    exclusiveContent {
        forRepository {
            maven { url = "https://mikegrundvig.github.io/frc-maven/" }
        }
        filter { includeGroup "com.michaelgrundvig.frc" }
    }
}
dependencies {
    implementation "com.michaelgrundvig.frc:frc-robot-model:2026.10.21"
    annotationProcessor "com.michaelgrundvig.frc:frc-docs-processor:2026.10.21"
}
```

WPILib's libraries (its math, units, utilities and field layouts, with EJML, QuickBuffers and
avaje-jsonb, which come with them) are compiled against and never brought along: a robot program
has them already, from GradleRIO, at its own version. A build that isn't a robot program adds them
itself, as this one's tests do.

## Developing it with the libraries that read it

Localization, Pitwall, the runtime and the robot template read the model by its published version.
To try a change to the model in one of them before it's published, give that build
`-PlocalLibs=<folder>`, a folder holding this repository's clone as `model/` (and any of
`localization/`, `pitwall/`, `runtime/` beside it): it builds what it finds there from source, in
place of the published versions. Without it, nothing local is used.

## Publishing it

`./gradlew publishToFrcMaven` runs every check, then writes this release (the catalog's `frc-libs`
version) into `../frc-maven`, a clone of [frc-maven](https://github.com/mikegrundvig/frc-maven)
beside this one (`-PfrcMavenDir=<folder>` names another). That repository's `release.sh` publishes
all the libraries in order and commits; its README says how a release goes.

## Building it

Java 25, which Gradle downloads if it's missing.

| Command | Does |
|---|---|
| `./gradlew ci` | Everything CI checks: formatting, static analysis (Error Prone, NullAway), the tests, coverage, the Javadoc. Under a minute. |
| `./gradlew test --tests '*InterlockTest'` | Some of the tests. |
| `./gradlew pitest` | Mutation testing: changes the compiled code a little at a time and runs the tests that cover each change, to find lines the tests run without checking. About six minutes; the report is `build/reports/pitest/index.html`. |
| `./gradlew pitest -PpitClasses='com.michaelgrundvig.frc.world.robot.Interlock*'` | The same for one class, in well under a minute. |

## License

[BSD 3-Clause](LICENSE).
