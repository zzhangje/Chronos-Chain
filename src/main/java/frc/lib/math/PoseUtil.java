package frc.lib.math;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.Map;

public class PoseUtil {
  public static Pose3d[] concat(Pose3d[] array1, Pose3d[] array2) {
    Pose3d[] result = new Pose3d[array1.length + array2.length];
    System.arraycopy(array1, 0, result, 0, array1.length);
    System.arraycopy(array2, 0, result, array1.length, array2.length);
    return result;
  }

  public static <T> Pose3d[] tolist(Map<T, Pose3d> pose) {
    return pose.values().toArray(new Pose3d[0]);
  }

  public static Pose3d[] repeat(Pose3d[] array, int times) {
    if (times <= 0) {
      return new Pose3d[0];
    }
    Pose3d[] result = new Pose3d[array.length * times];
    for (int i = 0; i < times; i++) {
      System.arraycopy(array, 0, result, i * array.length, array.length);
    }
    return result;
  }
}
