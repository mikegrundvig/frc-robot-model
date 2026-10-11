package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;

/**
 * The field's raised ground, on both alliances' halves: its {@link Surface}s, and the floor
 * everywhere else. The simulator rides the robot over it, odometry tells a tilt the ground explains
 * from one it doesn't, and a crossing of a crest becomes a landmark.
 */
public final class Terrain {
  /** A field with nothing to drive over. */
  public static final Terrain FLAT = new Terrain(List.of());

  private final List<Surface> surfaces;

  public Terrain(List<? extends Surface> surfaces) {
    this.surfaces = List.copyOf(surfaces);
  }

  /**
   * The terrain of a whole field, from the surfaces written for the blue alliance's half. Each
   * surface is followed by its image on the red alliance's half, and the two are named for the half
   * each is on, not for whose it is: "Left bump (blue)" and "Left bump (red)".
   *
   * @param blueSurfaces the raised ground on the blue alliance's half
   * @param symmetry how the red alliance's half relates to the blue alliance's
   * @param fieldLength the field's length, along x, in meters
   * @param fieldWidth the field's width, along y, in meters
   */
  static Terrain onBothHalves(
      List<? extends Surface> blueSurfaces,
      Symmetry symmetry,
      double fieldLength,
      double fieldWidth) {
    List<Surface> onBothHalves = new ArrayList<>();
    for (Surface surface : blueSurfaces) {
      Surface image = symmetry.apply(surface, fieldLength, fieldWidth);
      onBothHalves.add(withNameEnding(surface, " (blue)"));
      onBothHalves.add(withNameEnding(image, " (red)"));
    }
    return new Terrain(onBothHalves);
  }

  /**
   * A surface with something added to the end of its name, and to its crossings' names. A bump's, a
   * ramp's and a platform's crossings are named from the surface, so they follow by themselves. A
   * mesh's crossings are given with their own names, so each is renamed here.
   */
  private static Surface withNameEnding(Surface surface, String ending) {
    String name = surface.name() + ending;
    return switch (surface) {
      case Bump bump ->
          new Bump(
              name,
              bump.crestStart(),
              bump.crestEnd(),
              bump.heightMeters(),
              bump.depthMeters(),
              bump.material());
      case Ramp ramp ->
          new Ramp(
              name,
              ramp.foot(),
              ramp.top(),
              ramp.widthMeters(),
              ramp.heightMeters(),
              ramp.material());
      case Platform platform ->
          new Platform(
              name, platform.top(), platform.heightMeters(), platform.sides(), platform.material());
      case MeshSurface mesh -> {
        List<Crossing> crossings = new ArrayList<>();
        for (Crossing crossing : mesh.crossings()) {
          crossings.add(
              new Crossing(
                  crossing.name() + ending, crossing.start(), crossing.end(), crossing.kind()));
        }
        yield new MeshSurface(name, mesh.mesh(), crossings, mesh.material());
      }
    };
  }

  public List<Surface> surfaces() {
    return surfaces;
  }

  /** Every surface's crossings: the lines that place a robot crossing them. */
  public List<Crossing> crossings() {
    List<Crossing> crossings = new ArrayList<>();
    for (Surface surface : surfaces) {
      crossings.addAll(surface.crossings());
    }
    return List.copyOf(crossings);
  }

  /** How high the ground is at a point: the highest surface there, or the floor. */
  public double heightAt(Translation2d point) {
    double height = 0;
    for (Surface surface : surfaces) {
      height = Math.max(height, surface.heightAt(point));
    }
    return height;
  }

  /**
   * Which way the ground faces at a point, as a unit vector: the way the highest surface there
   * faces, or straight up where the ground is the floor.
   */
  public Vector3 normalAt(Translation2d point) {
    double highest = 0;
    Vector3 normal = Vector3.UP;
    for (Surface surface : surfaces) {
      double height = surface.heightAt(point);
      if (height > highest) {
        highest = height;
        normal = surface.normalAt(point);
      }
    }
    return normal;
  }

  /**
   * The surface whose footprint is at or nearest a point: the one to explain a tilt there.
   *
   * @param marginMeters how far outside a footprint still counts, for a point that may be off
   */
  public Optional<Surface> under(Translation2d point, double marginMeters) {
    Optional<Surface> nearest = Optional.empty();
    double nearestDistance = Double.POSITIVE_INFINITY;
    for (Surface surface : surfaces) {
      if (!surface.covers(point, marginMeters)) {
        continue;
      }
      double distance = surface.footprint().distance(point);
      if (distance < nearestDistance) {
        nearest = Optional.of(surface);
        nearestDistance = distance;
      }
    }
    return nearest;
  }
}
