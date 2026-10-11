package com.michaelgrundvig.frc.world.field;

import com.michaelgrundvig.frc.world.physical.Polygon2d;
import com.michaelgrundvig.frc.world.physical.TriangleMesh;
import com.michaelgrundvig.frc.world.physical.Vector3;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.math.geometry.Translation2d;

/**
 * Which of a mesh's triangles are over each part of the field, so that finding the ground at a
 * point looks at a few triangles instead of all of them.
 *
 * <p>The mesh's footprint is covered by a grid of square cells, laid out along the field's x and y
 * axes. Each cell lists every triangle whose bounding box touches it, which is every triangle that
 * could be over a point in the cell, and a few that are not. The cells are sized so that each lists
 * about {@link #TRIANGLES_PER_CELL} triangles.
 */
final class GroundGrid {
  /** About how many triangles stand over each cell. */
  private static final int TRIANGLES_PER_CELL = 4;

  /** The grid's corner with the least x and the least y, in the field frame, in meters. */
  private final double minX;

  private final double minY;

  /** The length of a cell's side, in meters. */
  private final double cellMeters;

  /** How many cells the grid has along x. */
  private final int columns;

  /** How many cells the grid has along y. */
  private final int rows;

  /**
   * For each cell, the numbers of the triangles over it. The cell in a column and a row is at
   * {@code column * rows + row}.
   */
  private final int[][] cells;

  /**
   * @param mesh the mesh the triangles are numbered in
   * @param ground the numbers of the triangles to look up: the ones that can be stood on
   * @param footprint the outline of those triangles seen from above, which the grid covers
   */
  GroundGrid(TriangleMesh mesh, List<Integer> ground, Polygon2d footprint) {
    double maxX = Double.NEGATIVE_INFINITY;
    double maxY = Double.NEGATIVE_INFINITY;
    double lowX = Double.POSITIVE_INFINITY;
    double lowY = Double.POSITIVE_INFINITY;
    for (Translation2d corner : footprint.corners()) {
      lowX = Math.min(lowX, corner.getX());
      lowY = Math.min(lowY, corner.getY());
      maxX = Math.max(maxX, corner.getX());
      maxY = Math.max(maxY, corner.getY());
    }
    minX = lowX;
    minY = lowY;

    // Cells of this size, covering the footprint's bounding box, number about as many as wanted.
    int cellsWanted = Math.max(1, ground.size() / TRIANGLES_PER_CELL);
    double boundsArea = (maxX - minX) * (maxY - minY);
    cellMeters = Math.sqrt(boundsArea / cellsWanted);
    columns = Math.max(1, (int) Math.ceil((maxX - minX) / cellMeters));
    rows = Math.max(1, (int) Math.ceil((maxY - minY) / cellMeters));

    List<List<Integer>> listed = new ArrayList<>();
    for (int cell = 0; cell < columns * rows; cell++) {
      listed.add(new ArrayList<>());
    }
    for (int number : ground) {
      listInEveryCellItsBoundsTouch(mesh, number, listed);
    }
    cells = new int[listed.size()][];
    for (int cell = 0; cell < listed.size(); cell++) {
      cells[cell] = listed.get(cell).stream().mapToInt(Integer::intValue).toArray();
    }
  }

  /** Adds a triangle to the list of every cell that its bounding box, seen from above, touches. */
  private void listInEveryCellItsBoundsTouch(
      TriangleMesh mesh, int number, List<List<Integer>> listed) {
    double fromX = Double.POSITIVE_INFINITY;
    double fromY = Double.POSITIVE_INFINITY;
    double toX = Double.NEGATIVE_INFINITY;
    double toY = Double.NEGATIVE_INFINITY;
    for (Vector3 corner : mesh.corners(mesh.triangles().get(number))) {
      fromX = Math.min(fromX, corner.x());
      fromY = Math.min(fromY, corner.y());
      toX = Math.max(toX, corner.x());
      toY = Math.max(toY, corner.y());
    }
    for (int column = column(fromX); column <= column(toX); column++) {
      for (int row = row(fromY); row <= row(toY); row++) {
        listed.get(column * rows + row).add(number);
      }
    }
  }

  /**
   * The numbers of the triangles that may be over a point: those listed in the point's cell. A
   * point off the grid gets the nearest cell's triangles, none of which is over it.
   */
  int[] trianglesNear(Translation2d point) {
    return cells[column(point.getX()) * rows + row(point.getY())];
  }

  /** The column an x is in, held to the grid's first and last columns. */
  private int column(double x) {
    return Math.clamp((long) Math.floor((x - minX) / cellMeters), 0, columns - 1);
  }

  /** The row a y is in, held to the grid's first and last rows. */
  private int row(double y) {
    return Math.clamp((long) Math.floor((y - minY) / cellMeters), 0, rows - 1);
  }
}
