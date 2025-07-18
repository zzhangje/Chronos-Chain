package frc.lib.interfaces.sensor.digital;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1StateValue;
import com.ctre.phoenix6.signals.S2StateValue;
import frc.lib.interfaces.CanId;
import frc.lib.utils.Phoenix6Helper;

public class BiDigitalInputCandi implements BiDigitalInput {
  private final CANdi candi;

  private final StatusSignal<S1StateValue> s1;
  private final StatusSignal<S2StateValue> s2;

  public BiDigitalInputCandi(String name, CanId device, CANdiConfiguration config) {
    candi = new CANdi(device.id(), device.bus());
    Phoenix6Helper.checkErrorAndRetry(
        "[" + name + "] config", () -> candi.getConfigurator().apply(config), 5);

    s1 = candi.getS1State();
    s2 = candi.getS2State();
  }

  @Override
  public boolean getValue1() {
    s1.refresh();
    return s1.getValue().equals(S1StateValue.High);
  }

  @Override
  public boolean getValue2() {
    s2.refresh();
    return s2.getValue().equals(S2StateValue.High);
  }

  @Override
  public DigitalInput getDigitalIO1() {
    return new DigitalInputSim(
        () -> {
          s1.refresh();
          return s1.getValue().equals(S1StateValue.High);
        });
  }

  @Override
  public DigitalInput getDigitalIO2() {
    return new DigitalInputSim(
        () -> {
          s2.refresh();
          return s2.getValue().equals(S2StateValue.High);
        });
  }
}
