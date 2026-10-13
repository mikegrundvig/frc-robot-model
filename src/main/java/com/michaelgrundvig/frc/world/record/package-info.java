/**
 * The record of what a robot did, as facts. Each type here is one thing the robot program logs
 * about its joints, its moves, its gates and its controls: an enum, or a record of enums and
 * numbers, which AdvantageKit logs as a struct. No type here holds text. A thing with a name is
 * referred to by where it is in one of the robot definition's own lists, which every reader of the
 * definition can look up, and each field's Javadoc says which list.
 *
 * <p>A sentence is made from the facts, by one function for each type, wherever a person reads one:
 * the console, Pitwall, a test's failure message. {@link
 * com.michaelgrundvig.frc.world.record.MoveSentences} says how a move, the drivetrain's move or an
 * auto ended; {@link com.michaelgrundvig.frc.world.record.JointSentences} says where a joint
 * stands, what holds it, where it is and who owns it; {@link
 * com.michaelgrundvig.frc.world.record.ControlSentences} says a gate's terms and a binding. The
 * wording lives in those three classes and nowhere else.
 *
 * <p>Some of what a sentence says is a team's own text and cannot be a fact: a command's name, a
 * gate's name, what an exception said. That text is logged as a string of its own beside the
 * record, and the function that makes the sentence is handed it.
 */
@NullMarked
package com.michaelgrundvig.frc.world.record;

import org.jspecify.annotations.NullMarked;
