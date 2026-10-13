package com.michaelgrundvig.frc.world.record;

import com.michaelgrundvig.frc.world.robot.Joint;
import com.michaelgrundvig.frc.world.robot.Position;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Where a joint that holds positions is, among its named places: at one within its band, between
 * two, below the lowest or above the highest; and for a joint with no places, nothing but its
 * position as a number. Every answer it can give is made once, as it is built, so asking where the
 * joint is in a loop makes no new object, and the same place gives the very same record.
 */
public final class JointPlaces {
  /** How near a place the joint must be to be at it, in its units: its band to arrive. */
  private final double enter;

  /** The value of each of the joint's named positions, lowest first, in the joint's units. */
  private final double[] values;

  private final JointPlace[] atPlace;
  private final JointPlace[] betweenPlaces;
  private final JointPlace belowPlaces;
  private final JointPlace abovePlaces;

  /**
   * @param joint the joint, whose places are its named positions
   * @param enter how near one it must be to be at it, in its units
   */
  public JointPlaces(Joint joint, double enter) {
    this.enter = enter;
    List<Position> declared = joint.positions();
    // The places are gone through lowest first. A record names a place by where it is in the
    // joint's own list, so each place's index there is kept beside it.
    List<Integer> lowestFirst = new ArrayList<>();
    for (int index = 0; index < declared.size(); index++) {
      lowestFirst.add(index);
    }
    lowestFirst.sort(Comparator.comparingDouble(index -> declared.get(index).value()));

    int count = lowestFirst.size();
    values = new double[count];
    atPlace = new JointPlace[count];
    betweenPlaces = new JointPlace[Math.max(0, count - 1)];
    for (int rank = 0; rank < count; rank++) {
      int place = lowestFirst.get(rank);
      values[rank] = declared.get(place).value();
      atPlace[rank] = JointPlace.by(JointPlace.Where.AT, place);
      if (rank > 0) {
        betweenPlaces[rank - 1] = JointPlace.between(lowestFirst.get(rank - 1), place);
      }
    }
    boolean hasPlaces = count > 0;
    belowPlaces =
        hasPlaces
            ? JointPlace.by(JointPlace.Where.BELOW, lowestFirst.getFirst())
            : JointPlace.UNNAMED;
    abovePlaces =
        hasPlaces
            ? JointPlace.by(JointPlace.Where.ABOVE, lowestFirst.getLast())
            : JointPlace.UNNAMED;
  }

  /**
   * Where the joint is among its places, at a position.
   *
   * @param position where the joint is, in its units: radians, or meters
   */
  public JointPlace at(double position) {
    if (values.length == 0) {
      return JointPlace.UNNAMED;
    }
    for (int rank = 0; rank < values.length; rank++) {
      if (Math.abs(position - values[rank]) <= enter) {
        return atPlace[rank];
      }
    }
    if (position < values[0]) {
      return belowPlaces;
    }
    for (int rank = 1; rank < values.length; rank++) {
      if (position < values[rank]) {
        return betweenPlaces[rank - 1];
      }
    }
    return abovePlaces;
  }
}
