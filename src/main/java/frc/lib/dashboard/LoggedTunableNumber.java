package frc.lib.dashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** A class representing a tunable number whose value can be adjusted via NetworkTables. */
public class LoggedTunableNumber implements DoubleSupplier {
  private static final String ROOT_TABLE_KEY = "/Tuning";
  private static final Map<String, Boolean> activeGroups = new HashMap<>();
  private static final Map<String, List<LoggedTunableNumber>> groupInstances = new HashMap<>();

  private final String fullKey;
  private final String groupName;
  private boolean hasDefault = false;
  private double defaultValue;
  private LoggedNetworkNumber dashboardNumber;
  private boolean isActive;
  private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

  /**
   * Sets whether a specific group is active. When a group becomes active, its tunable numbers are
   * registered in NetworkTables. When inactive, they are removed or ignored.
   *
   * @param group The name of the group to activate or deactivate
   * @param active True to activate, false to deactivate
   */
  public static synchronized void setGroupActive(String group, boolean active) {
    boolean validActive = active; // && Config.IS_LIVE_DEBUG.getAsBoolean();
    if (activeGroups.getOrDefault(group, false) != validActive) {
      activeGroups.put(group, validActive);
      List<LoggedTunableNumber> instances = groupInstances.get(group);
      if (instances != null) {
        for (LoggedTunableNumber instance : instances) {
          instance.updateActivation();
        }
      }
    }
  }

  /**
   * Checks whether a specific group is active.
   *
   * @param group The name of the group to check
   * @return True if the group is active, false otherwise
   */
  public static boolean isGroupActive(String group) {
    return activeGroups.getOrDefault(group, false);
  }

  /**
   * Constructs a tunable number without a default value.
   *
   * @param group The group this tunable belongs to
   * @param key The unique identifier for this tunable within the group
   */
  public LoggedTunableNumber(String group, String key) {
    this.fullKey = ROOT_TABLE_KEY + "/" + key;
    this.groupName = group;
    registerInstance(group);
  }

  /**
   * Constructs a tunable number with a default value.
   *
   * @param group The group this tunable belongs to
   * @param key The unique identifier for this tunable within the group
   * @param defaultValue Default value if tuning is disabled or value hasn't been set
   */
  public LoggedTunableNumber(String group, String key, double defaultValue) {
    this(group, key);
    initDefault(defaultValue);
  }

  /**
   * Convenience constructor using the default group.
   *
   * @param key The unique identifier for this tunable
   */
  public LoggedTunableNumber(String key) {
    this.groupName = key.split("/")[0];
    this.fullKey = ROOT_TABLE_KEY + "/" + key;
    registerInstance(this.groupName);
  }

  /**
   * Convenience constructor using the default group with a default value.
   *
   * @param key The unique identifier for this tunable
   * @param defaultValue Default value if tuning is disabled or value hasn't been set
   */
  public LoggedTunableNumber(String key, double defaultValue) {
    this(key);
    initDefault(defaultValue);
  }

  /**
   * Initializes the default value and activates the tunable number if applicable.
   *
   * @param defaultValue Default value if tuning is disabled or value hasn't been set
   */
  public void initDefault(double defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      updateActivation(); // Attempt to register with NetworkTables if group is active
    }
  }

  /**
   * Registers this instance with its group so activation changes can be tracked.
   *
   * @param group The group this tunable belongs to
   */
  private void registerInstance(String group) {
    synchronized (LoggedTunableNumber.class) {
      groupInstances.computeIfAbsent(group, k -> new ArrayList<>()).add(this);
      this.isActive = isGroupActive(group);
      updateActivation();
    }
  }

  /**
   * Updates whether this tunable number should be active based on its group state.
   * Activates/deactivates the NetworkTables entry accordingly.
   */
  private void updateActivation() {
    if (!hasDefault) return;

    boolean shouldBeActive = isGroupActive(getGroupName());
    if (this.isActive != shouldBeActive) {
      this.isActive = shouldBeActive;
      if (this.isActive) {
        this.dashboardNumber = new LoggedNetworkNumber(fullKey, defaultValue);
      } else {
        this.dashboardNumber = null;
      }
    }
  }

  /**
   * Extracts the group name from the full NetworkTables key.
   *
   * @return Group name as a String
   */
  private String getGroupName() {
    return groupName;
  }

  /**
   * Gets the current value of the tunable number. If active and tunable, reads from NetworkTables;
   * otherwise returns the default.
   *
   * @return The current value
   */
  public double get() {
    if (!hasDefault) {
      return 0.0;
    } else if (this.isActive && this.dashboardNumber != null) {
      return dashboardNumber.get();
    }
    return defaultValue;
  }

  /**
   * Checks if the value has changed since the last call for a given ID.
   *
   * @param id Unique ID to track changes per context (e.g., subsystem or command)
   * @return True if the value has changed
   */
  public boolean hasChanged(int id) {
    double currentValue = get();
    Double lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  /**
   * Executes an action if any of the provided tunable numbers have changed since last use of the
   * given ID.
   *
   * @param id Unique ID to track changes
   * @param action Action to run when any value has changed
   * @param tunableNumbers Array of tunable numbers to monitor
   */
  public static void ifChanged(
      int id, Consumer<double[]> action, LoggedTunableNumber... tunableNumbers) {
    if (Arrays.stream(tunableNumbers).anyMatch(tunableNumber -> tunableNumber.hasChanged(id))) {
      action.accept(Arrays.stream(tunableNumbers).mapToDouble(LoggedTunableNumber::get).toArray());
    }
  }

  /**
   * Overloaded version of ifChanged that runs a Runnable instead of a Consumer<double[]>.
   *
   * @param id Unique ID to track changes
   * @param action Action to run when any value has changed
   * @param tunableNumbers Array of tunable numbers to monitor
   */
  public static void ifChanged(int id, Runnable action, LoggedTunableNumber... tunableNumbers) {
    ifChanged(id, values -> action.run(), tunableNumbers);
  }

  /**
   * Implementation of DoubleSupplier.getAsDouble(). Simply delegates to the get() method.
   *
   * @return Current value of the tunable number
   */
  @Override
  public double getAsDouble() {
    return get();
  }
}
