package frc.lib.service;

import frc.lib.dashboard.BooleanChooser;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.interfaces.VirtualSubsystem;
import java.util.HashMap;
import java.util.Map;

public class TunableManager extends VirtualSubsystem {
  private final String name;
  private final Map<String, BooleanChooser> tunables = new HashMap<>();

  public TunableManager(String name) {
    this.name = name;
  }

  public void register(String group) {
    this.tunables.put(group, new BooleanChooser(name + "/" + group, false));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (Map.Entry<String, BooleanChooser> entry : tunables.entrySet()) {
      String group = entry.getKey();
      BooleanChooser chooser = entry.getValue();
      LoggedTunableNumber.setGroupActive(group, chooser.getAsBoolean());
    }
  }

  @Override
  public void onStop() {
    for (Map.Entry<String, BooleanChooser> entry : tunables.entrySet()) {
      String group = entry.getKey();
      LoggedTunableNumber.setGroupActive(group, false);
    }
  }
}
