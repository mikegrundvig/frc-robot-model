# FRC robot model

What an FRC robot and its field are, written once as data, for everything that needs to know them:
the robot program, a simulator, localization, and the tools around them. It's a Java library on
WPILib's math and units, and nothing else of a robot program: no HAL, no NetworkTables, no clock
and no state, so a robot, a simulator and a replay all read the same model.

It's unreleased: there are no versions yet, and anything may change.

## What's in it

| Package (`com.michaelgrundvig.frc.world`) | Holds |
|---|---|
| `robot` | The robot's definition: its body, drivetrain, joints and what drives them, sensors, cameras and IMUs, and the rules between them (interlocks, conditions, named places and outputs). |
| `field` | The field: its tags, the places and zones code names, headings, starts, terrain, and a venue's measured layout. |
| `physical` | What both are made of: mass and inertia, shapes and materials. |
| `config` | The validation kit every record is built and checked with. A definition that can't work is refused as it's built, saying why; one that's doubtful says so. |
| `bridges` | Writers for other tools, such as AdvantageScope's robot assets. |

A joint, as a definition writes it:

```java
static final Joint.Revolute PIVOT =
    Joint.pivot("pivot", Part.CHASSIS, new Translation3d(0.25, 0, 0.32), Joint.PITCH_DOWN)
        .range(Range.degrees(0, 135))
        .rest(PIVOT_IN)
        .positions(PIVOT_IN, PIVOT_OUT)
        .motor(Motor.KRAKEN_X60, Gearing.of(25), TALON_FX.can(32))
        .control(MechanismControl.position(30));

static final Joint.Revolute ROLLER =
    Joint.roller("roller", PIVOT, new Translation3d(0, 0, 0.35))
        .outputs(INTAKING)
        .motor(Motor.KRAKEN_X60, Gearing.of(3), TALON_FX.can(31))
        // The roller runs only once the pivot is out of the frame.
        .runsOnlyWhen(PIVOT.atLeast(Degrees.of(110)))
        .control(MechanismControl.velocity(0.02));
```

That's `src/test/java/com/michaelgrundvig/frc/world/examples/ExampleJoints.java`, which the build
compiles and the Javadoc shows.

`docs-processor/` is an annotation processor that goes with it. As a project compiles, it writes
each configuration record's Javadoc into the jar, so a tool can show what a value means in the
words the code already has.

## Using it

It isn't published anywhere. A build takes it as source, as a git submodule built as part of that
build (a Gradle composite build), so there is no version to bump:

```
git submodule add https://github.com/mikegrundvig/frc-robot-model.git model
```

```groovy
// settings.gradle
includeBuild("model")

// build.gradle
dependencies {
    implementation "com.michaelgrundvig.frc:frc-robot-model"
    annotationProcessor "com.michaelgrundvig.frc:docs-processor"
}
```

A change to the model is made in that folder and committed there, then the build that uses it
commits the submodule's new commit.

## Building it

Java 25, which Gradle downloads if it's missing.

| Command | Does |
|---|---|
| `./gradlew ci` | Everything CI checks: formatting, static analysis (Error Prone, NullAway), the tests, coverage, the Javadoc. Under a minute. |
| `./gradlew test --tests '*InterlockTest'` | Some of the tests. |
| `./gradlew pitest` | Mutation testing: changes the compiled code a little at a time and runs the tests that cover each change, to find lines the tests run without checking. About six minutes; the report is `build/reports/pitest/index.html`. |
| `./gradlew pitest -PpitClasses='com.michaelgrundvig.frc.world.robot.Interlock*'` | The same for one class, in well under a minute. |
