/**
 * The world model: what the field and the robot are, described once, and read by everything that
 * needs to know them: the robot program, localization, the simulator, and the views.
 *
 * <ul>
 *   <li>{@code config}: the validation kit every configuration record is built with, by name, and
 *       checked with.
 *   <li>{@code physical}: what the field and the robot are both made of: mass and inertia, shapes
 *       and materials, and the plain geometry they're written in, where WPILib's math has none.
 *   <li>{@code field}: the field, its tags, the places and zones code refers to by name, its
 *       terrain, and the elements that stand on it.
 *   <li>{@code robot}: the robot's definition, its body, drivetrain, cameras, and IMUs, with what a
 *       joint's definition means for a program: where it is driven, how far its encoder and motor
 *       may disagree, which switches its controller reads.
 *   <li>{@code path}: the timed paths a drivetrain follows, through keyframes at poses or the
 *       field's places, under the limits they're given.
 *   <li>{@code game}: a season's game: what its manual allows a robot to be, checked against its
 *       definition, and its field, written once when the game is revealed.
 * </ul>
 *
 * <p>The field and the robot don't know about each other, and the kit and the physical values know
 * about neither; paths read the field, and a season's game reads the field, the physical values and
 * the robot, and neither is read by them (the model's {@code ArchitectureTest}). None of it reads a
 * clock or holds state, so the robot, the simulator's parallel runs, and a replay all read the same
 * model (the robot program's): what a joint's definition means is worked out here from plain
 * values, and what to remember between loops is the robot program's.
 */
@NullMarked
package com.michaelgrundvig.frc.world;

import org.jspecify.annotations.NullMarked;
