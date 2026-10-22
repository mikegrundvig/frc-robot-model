package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import java.util.HashSet;
import java.util.Set;

/**
 * What a field has to get right before anything uses it: every place can be found for both
 * alliances, every zone and surface is on the field, and no two things of a kind share a name. A
 * {@link FieldModel} runs these as it is built, so a wrong tag number is refused as the program
 * starts, and in the tests before that.
 */
final class FieldChecks {
  private FieldChecks() {}

  /**
   * Refuses a field that gets any of it wrong.
   *
   * @throws IllegalArgumentException naming the first thing that is wrong
   */
  static void require(FieldModel field) {
    requirePlacesCanBeFound(field);
    requireZonesOnTheField(field);
    requireSurfacesOnTheField(field);
  }

  /**
   * Every start and every other place has a name of its own, and every place can be found for both
   * alliances: a place found from a tag needs the tag to be in the layout, and to have a partner on
   * the red alliance's half. A start is written as a pose, so it can always be found.
   */
  private static void requirePlacesCanBeFound(FieldModel field) {
    Set<String> names = new HashSet<>();
    for (Start start : field.starts()) {
      requireNewName(names, start.name());
    }
    for (Place place : field.places()) {
      requireNewName(names, place.name());
      for (Alliance alliance : Alliance.values()) {
        try {
          place.on(field, alliance);
        } catch (IllegalArgumentException e) {
          throw new IllegalArgumentException(place.name() + ": " + e.getMessage(), e);
        }
      }
    }
  }

  private static void requireNewName(Set<String> names, String name) {
    boolean nameIsNew = names.add(name);
    if (!nameIsNew) {
      throw new IllegalArgumentException("two places are named " + name);
    }
  }

  /**
   * Every zone has a name of its own, and the middle of each of its areas, as written for the blue
   * alliance, is on the field. Only the middle is judged: a zone may reach past a wall.
   */
  private static void requireZonesOnTheField(FieldModel field) {
    Set<String> names = new HashSet<>();
    for (Zone zone : field.zones()) {
      boolean nameIsNew = names.add(zone.name());
      if (!nameIsNew) {
        throw new IllegalArgumentException("two zones are named " + zone.name());
      }
      for (Polygon2d area : zone.blueAreas()) {
        if (!field.contains(area.centroid())) {
          throw new IllegalArgumentException(zone.name() + ": an area is off the field");
        }
      }
    }
  }

  /**
   * Every surface, on both halves, has a name of its own, and is on the field: the middle of its
   * footprint, and both ends of each of its crossings.
   */
  private static void requireSurfacesOnTheField(FieldModel field) {
    Set<String> names = new HashSet<>();
    for (Surface surface : field.terrain().surfaces()) {
      boolean nameIsNew = names.add(surface.name());
      if (!nameIsNew) {
        throw new IllegalArgumentException("two surfaces are named " + surface.name());
      }
      if (!isOnTheField(surface, field)) {
        throw new IllegalArgumentException(surface.name() + ": it runs off the field");
      }
    }
  }

  private static boolean isOnTheField(Surface surface, FieldModel field) {
    if (!field.contains(surface.footprint().centroid())) {
      return false;
    }
    for (Crossing crossing : surface.crossings()) {
      if (!field.contains(crossing.start()) || !field.contains(crossing.end())) {
        return false;
      }
    }
    return true;
  }
}
