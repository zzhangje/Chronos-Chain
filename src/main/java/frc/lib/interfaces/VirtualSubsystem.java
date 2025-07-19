package frc.lib.interfaces;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

public abstract class VirtualSubsystem implements Subsystem {
  // Thread-safe list implementation to prevent ConcurrentModificationException
  private static final List<VirtualSubsystem> subsystems = new CopyOnWriteArrayList<>();

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  public static void periodicAll() {
    // CopyOnWriteArrayList is already thread-safe for iteration
    for (VirtualSubsystem subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  public abstract void periodic();

  public final Command run() {
    return Commands.runOnce(
        () -> {
          if (!subsystems.contains(this)) {
            subsystems.add(this);
          }
          onRun();
        },
        this // Requires this subsystem
        );
  }

  public final Command stop() {
    return Commands.runOnce(
        () -> {
          subsystems.remove(this);
          onStop();
        },
        this // Requires this subsystem
        );
  }

  /** Called when the subsystem is started. Override this method to perform any initialization. */
  protected void onRun() {}

  /**
   * Called when the subsystem is stopped. Override this method to perform any cleanup or
   * finalization.
   */
  protected void onStop() {}

  /** Clears all registered subsystems (for testing purposes) */
  public static void clearAllSubsystems() {
    subsystems.clear();
  }
}
