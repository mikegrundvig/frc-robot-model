/**
 * A season's game: what its manual allows a robot to be ({@link
 * com.michaelgrundvig.frc.world.game.SeasonLimits}), and its field, written once when the game is
 * revealed ({@link com.michaelgrundvig.frc.world.game.Rebuilt} for 2026, {@link
 * com.michaelgrundvig.frc.world.game.Reefscape} for 2025, {@link
 * com.michaelgrundvig.frc.world.game.Crescendo} for 2024).
 *
 * <p>The limits are how big around a robot may be, how tall at the start, how far it may reach,
 * where its bumpers go, and how much it may weigh, checked against a robot's definition. A season's
 * field is its tags, places, zones, terrain, starts and limits, as plain data about a published
 * game. A {@link com.michaelgrundvig.frc.world.game.Game} bundles the field with the layouts a
 * venue may be measured against, for the robot program to be started with. A new season's field is
 * added here on kickoff day, in a new version of the model. A program that needs it sooner writes a
 * {@code Game} of its own.
 *
 * <p>This package reads the field, the physical values, the robot's body and definition, and the
 * validation kit. None of them reads it, and it reads no clock and holds no state: a field is the
 * same every time it is made. The robot's definition doesn't know the game.
 */
@NullMarked
package com.michaelgrundvig.frc.world.game;

import org.jspecify.annotations.NullMarked;
