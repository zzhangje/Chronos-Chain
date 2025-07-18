package frc.lib.math;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class TransformTree {
  protected static class Node {
    Supplier<Transform3d> transformSupplier;
    Pose3d cachedPose = new Pose3d();
    Map<String, Node> children = new HashMap<>();

    public Node() {}

    public Node(Supplier<Transform3d> transformSupplier) {
      this.transformSupplier = transformSupplier;
    }

    public Transform3d getTransform() {
      return transformSupplier.get();
    }
  }

  protected final Node root = new Node();
  protected Pose3d rootPose = new Pose3d();

  /**
   * Creates a new TransformTree with an optional root name
   *
   * @param rootName The name of the root node, can be null
   */
  public TransformTree() {}

  /**
   * Sets the root pose of the transform tree
   *
   * @param pose The new root pose
   */
  public void setRootPose(Pose3d pose) {
    rootPose = pose;
  }

  /**
   * Registers a transform in the tree with a supplier
   *
   * @param path The path to the transform (e.g. "robot/arm/wrist")
   * @param transformSupplier Supplier for the transform relative to its parent
   */
  public void registerComponent(String path, Supplier<Transform3d> transformSupplier) {
    String[] parts = path.split("/");
    Node current = root;

    for (String part : parts) {
      current = current.children.computeIfAbsent(part, k -> new Node());
    }

    current.transformSupplier = transformSupplier;
  }

  /**
   * Registers a static transform in the tree
   *
   * @param path The path to the transform
   * @param transform The static transform relative to its parent
   */
  public void registerComponent(String path, Transform3d transform) {
    registerComponent(path, () -> transform);
  }

  /**
   * Gets the pose of a specific node in the tree
   *
   * @param path The path to the node
   * @return The absolute pose of the node, or null if not found
   */
  public Pose3d getNodePose(String path) {
    String[] parts = path.split("/");
    Node current = root;

    for (String part : parts) {
      current = current.children.get(part);
      if (current == null) {
        return null; // Node not found
      }
    }

    return current.cachedPose;
  }

  /** Updates all transforms in the tree by clearing the cache */
  public void update() {
    updateNode(root, rootPose);
  }

  private void updateNode(Node node, Pose3d parentPose) {
    if (node.transformSupplier != null) {
      Transform3d transform = node.getTransform();
      node.cachedPose = parentPose.plus(transform);
    } else {
      node.cachedPose = parentPose; // No transform, just use parent pose
    }
    for (Node child : node.children.values()) {
      updateNode(child, node.cachedPose);
    }
  }

  /** Prints the structure of the transform tree for debugging */
  public void print() {
    System.out.println(root);
    printNode(root, "");
  }

  private void printNode(Node node, String indent) {
    for (Map.Entry<String, Node> entry : node.children.entrySet()) {
      System.out.println(
          indent
              + "|--- "
              + entry.getKey()
              + (entry.getValue().transformSupplier == null ? " ?" : " #"));
      printNode(entry.getValue(), indent + "|   ");
    }
  }
}
