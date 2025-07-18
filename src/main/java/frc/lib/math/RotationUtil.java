package frc.lib.math;

public class RotationUtil {
  public static boolean isRotationEnterSpecificAngleArea(
      double startAngleRad,
      double endAngleRad,
      double minSpecificAngleRad,
      double maxSpecificAngleRad) {

    // Handle the wrapped range case
    boolean isWrappedRange = minSpecificAngleRad > maxSpecificAngleRad;

    // Check if rotation passes either boundary
    if (isRotationPassSpecificAngle(startAngleRad, endAngleRad, minSpecificAngleRad, isWrappedRange)
        || isRotationPassSpecificAngle(
            startAngleRad, endAngleRad, maxSpecificAngleRad, isWrappedRange)) {
      return true;
    }

    if (isWrappedRange) {
      // For wrapped range, angle is in range if it's >= min OR <= max
      if ((startAngleRad >= minSpecificAngleRad || startAngleRad <= maxSpecificAngleRad)
          || (endAngleRad >= minSpecificAngleRad || endAngleRad <= maxSpecificAngleRad)) {
        return true;
      }
    } else {
      // For normal range
      if ((startAngleRad >= minSpecificAngleRad && startAngleRad <= maxSpecificAngleRad)
          || (endAngleRad >= minSpecificAngleRad && endAngleRad <= maxSpecificAngleRad)) {
        return true;
      }
    }

    return false;
  }

  public static boolean isRotationPassSpecificAngle(
      double startAngleRad, double endAngleRad, double specificAngleRad, boolean isWrappedRange) {

    if (isWrappedRange) {
      return (startAngleRad < specificAngleRad && endAngleRad > specificAngleRad)
          || (startAngleRad > specificAngleRad && endAngleRad < specificAngleRad);
    } else {
      return (startAngleRad < specificAngleRad && endAngleRad >= specificAngleRad)
          || (startAngleRad > specificAngleRad && endAngleRad <= specificAngleRad);
    }
  }
}
