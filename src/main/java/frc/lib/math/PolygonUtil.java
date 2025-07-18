package frc.lib.math;

import edu.wpi.first.math.geometry.Translation2d;

public class PolygonUtil {
  /**
   * Check if a point lies inside a polygon using ray casting algorithm with boundary box
   * optimization
   *
   * @param point The point to check
   * @param polygon Array of polygon vertices
   * @return if the point is inside the polygon
   */
  public static boolean isInPolygon(Translation2d point, Translation2d[] polygon) {
    // Quick rejection using bounding box check
    if (!isInBoundingBox(point, polygon)) {
      return false;
    }

    var inside = false;
    var j = polygon.length - 1;
    var x = point.getX();
    var y = point.getY();

    for (var i = 0; i < polygon.length; i++) {
      double xi = polygon[i].getX();
      double yi = polygon[i].getY();
      double xj = polygon[j].getX();
      double yj = polygon[j].getY();

      // Check if point is exactly on the edge
      if (isOnLine(point, polygon[i], polygon[j])) {
        return true;
      }

      if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
        inside = !inside;
      }
      j = i;
    }

    return inside;
  }

  /** Check if point is within the bounding box of the polygon */
  private static boolean isInBoundingBox(Translation2d point, Translation2d[] polygon) {
    var minX = polygon[0].getX();
    var maxX = minX;
    var minY = polygon[0].getY();
    var maxY = minY;

    // Find bounding box
    for (Translation2d vertex : polygon) {
      minX = Math.min(minX, vertex.getX());
      maxX = Math.max(maxX, vertex.getX());
      minY = Math.min(minY, vertex.getY());
      maxY = Math.max(maxY, vertex.getY());
    }

    // Check if point is within bounding box
    return point.getX() >= minX
        && point.getX() <= maxX
        && point.getY() >= minY
        && point.getY() <= maxY;
  }

  /** Check if point lies on a line segment */
  private static boolean isOnLine(Translation2d point, Translation2d start, Translation2d end) {
    final var EPSILON = 1e-10;

    var lineLength = start.getDistance(end);
    var d1 = point.getDistance(start);
    var d2 = point.getDistance(end);

    return Math.abs(d1 + d2 - lineLength) < EPSILON;
  }
}
