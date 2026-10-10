/**
 * Timed paths: the keyframes a path passes through ({@link
 * com.michaelgrundvig.frc.world.path.Keyframe}, at a pose or at one of the field's places), and the
 * trajectories planned through them or along a line, under what a move may ask of the robot ({@link
 * com.michaelgrundvig.frc.world.path.Trajectories}).
 *
 * <p>A plan is a function of where the robot is, how it's moving and the limits it's given, in
 * field coordinates and the seconds between samples the caller says; it reads no clock and holds no
 * state. Following one, and finding a keyframe's place for the alliance the robot is on, are the
 * robot program's.
 */
@NullMarked
package com.michaelgrundvig.frc.world.path;

import org.jspecify.annotations.NullMarked;
