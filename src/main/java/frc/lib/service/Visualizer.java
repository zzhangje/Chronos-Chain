package frc.lib.service;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.interfaces.VirtualSubsystem;
import frc.lib.math.TransformTree;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Visualizer service for managing and visualizing components in a robot's transform tree. This
 * service allows registration of components with their transforms and provides methods to retrieve
 * their poses and transforms.
 */
public class Visualizer extends VirtualSubsystem {
  public static final String BASE_FRAME = "robot";
  private final TransformTree transformTree = new TransformTree();
  private final Map<Integer, String> indexToPath = new HashMap<>();
  private final Map<String, String> nameToPath = new HashMap<>();

  public Visualizer() {
    nameToPath.put(BASE_FRAME, BASE_FRAME);
    transformTree.setRootPose(new Pose3d());
  }

  /**
   * Registers a node for visualization
   *
   * @param parentName Name of the parent node
   * @param childName Name of the child node
   * @param visualizationId Unique ID for the visualization
   * @param transformSupplier Supplier for the transform relative to its parent
   */
  public void registerVisualizedComponent(
      String parentName,
      String childName,
      int visualizationId,
      Supplier<Transform3d> transformSupplier) {
    var path = nameToPath.get(parentName) + "/" + childName;
    transformTree.registerComponent(path, transformSupplier);
    nameToPath.put(childName, path);
    if (visualizationId >= 0) {
      indexToPath.put(visualizationId, path);
    }
  }

  /**
   * Registers a node for visualization with a fixed transform
   *
   * @param parentName Name of the parent node
   * @param childName Name of the child node
   * @param visualizationId Unique ID for the visualization
   * @param transform Constant transform relative to its parent
   */
  public void registerVisualizedComponent(
      String parentName, String childName, int visualizationId, Transform3d transform) {
    var path = nameToPath.get(parentName) + "/" + childName;
    transformTree.registerComponent(path, () -> transform);
    nameToPath.put(childName, path);
    if (visualizationId >= 0) {
      indexToPath.put(visualizationId, path);
    }
  }

  /**
   * Registers a node without visualization
   *
   * @param parentName Name of the parent node
   * @param childName Name of the child node
   * @param transformSupplier Supplier for the transform relative to its parent
   */
  @Deprecated
  public void registerUnvisualizedComponent(
      String parentName, String childName, Supplier<Transform3d> transformSupplier) {
    var path = nameToPath.get(parentName) + "/" + childName;
    transformTree.registerComponent(path, transformSupplier);
    nameToPath.put(childName, path);
  }

  /**
   * Returns the pose of a component by its ID
   *
   * @param componentId Unique ID of the component
   * @return Pose3d of the component in the world frame
   */
  public Pose3d getComponentPose(int componentId) {
    return transformTree.getNodePose(indexToPath.get(componentId));
  }

  /**
   * Returns the transform of a component relative to its parent
   *
   * @param componentId Unique ID of the component
   * @return Transform3d of the component relative to its parent
   */
  public Transform3d getComponentTransform(int componentId) {
    return transformTree.getNodePose(indexToPath.get(componentId)).minus(new Pose3d());
  }

  /** Updates all visualized poses */
  @Override
  public void periodic() {
    transformTree.update();

    var maxId = indexToPath.keySet().stream().max(Integer::compare).get();
    var poses = new Pose3d[maxId + 1];

    for (int id = 0; id < maxId + 1; id++) {
      if (indexToPath.get(id) == null) {
        poses[id] = new Pose3d(1e9, 1e9, 1e9, new Rotation3d());
      } else {
        poses[id] = transformTree.getNodePose(indexToPath.get(id));
      }
    }

    Logger.recordOutput("Visualization/Components", poses);
  }

  public void print() {
    transformTree.print();
  }
}
