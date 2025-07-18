package frc.lib.utils;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class Phoenix6Helper {
  public static void checkErrorAndRetry(String deviceName, Supplier<StatusCode> configFunction) {
    checkErrorAndRetry(deviceName, configFunction, 5);
  }

  public static void checkErrorAndRetry(
      String deviceName, Supplier<StatusCode> configFunction, int numTries) {
    StatusCode code = configFunction.get();

    var tries = 0;
    while (code != StatusCode.OK && tries < numTries) {
      DriverStation.reportWarning(
          deviceName + " -> Retrying phoenix 6 device config " + code.getName(), false);
      code = configFunction.get();
      tries++;
    }

    if (code != StatusCode.OK) {
      DriverStation.reportError(
          deviceName + " -> Failed to config phoenix 6 device after " + numTries + " attempts",
          false);
    }
  }
}
