package frc.lib.interfaces;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
  private static final List<VirtualSubsystem> subsystems = new ArrayList<>();

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  public static void periodicAll() {
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
        });
  }

  public final Command stop() {
    return Commands.runOnce(
        () -> {
          if (subsystems.contains(this)) {
            subsystems.remove(this);
          }
          onStop();
        });
  }

  /** Called when the subsystem is started. Override this method to perform any initialization. */
  protected void onRun() {}

  /**
   * Called when the subsystem is stopped. Override this method to perform any cleanup or
   * finalization.
   */
  protected void onStop() {}
}
