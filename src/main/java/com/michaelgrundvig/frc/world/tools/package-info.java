/**
 * What the build's tool writers read of the robot program: its robots, through the {@link
 * com.michaelgrundvig.frc.world.tools.RobotCatalog} the program implements, each a {@link
 * com.michaelgrundvig.frc.world.tools.RobotDescription}, and the releases of the tools they write
 * for ({@link com.michaelgrundvig.frc.world.tools.AdvantageScopeRelease}).
 *
 * <p>The writers themselves are the build's, not the robot's: each tool's Gradle plugin (the
 * runtime's {@code build-logic/}) runs its writer on the compiled program, which finds the robots
 * through the catalog the program names, builds their configuration records, and writes files.
 * Nothing on the robot runs them, so they're not in the model's jar. What's here reads the robot's
 * definition, and the field and the robot's definition never see it (the model's {@code
 * ArchitectureTest}).
 */
@NullMarked
package com.michaelgrundvig.frc.world.tools;

import org.jspecify.annotations.NullMarked;
