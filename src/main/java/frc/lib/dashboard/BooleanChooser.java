package frc.lib.dashboard;

import edu.wpi.first.networktables.*;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

/** A boolean chooser for the dashboard that allows switching between true/false. */
public class BooleanChooser extends LoggedNetworkInput implements BooleanSupplier {
  private boolean value;

  private final BooleanPublisher valuePublisher;
  private final BooleanSubscriber valueInput;

  public BooleanChooser(String name) {
    this(name, false);
  }

  public BooleanChooser(String name, boolean defaultValue) {
    this.value = defaultValue;

    var table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    valuePublisher = table.getBooleanTopic(name).publish();
    valueInput = table.getBooleanTopic(name).subscribe(defaultValue, PubSubOption.periodic(0.1));
    Logger.registerDashboardInput(this);

    valuePublisher.set(value);
  }

  /** Returns the selected boolean value. */
  @Override
  public boolean getAsBoolean() {
    return value;
  }

  /** Returns the selected boolean value (alternative to getAsBoolean). */
  public boolean get() {
    return getAsBoolean();
  }

  /** Sets the current boolean value. */
  public void set(boolean newValue) {
    if (value != newValue) {
      value = newValue;
      valuePublisher.set(value);
    }
  }

  @Override
  public void periodic() {
    value = valueInput.get();
    valuePublisher.set(value);
  }
}
