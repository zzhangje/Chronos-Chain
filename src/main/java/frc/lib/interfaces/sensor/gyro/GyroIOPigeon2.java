package frc.lib.interfaces.sensor.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.interfaces.CanId;
import frc.lib.utils.Phoenix6Helper;
import lombok.Getter;

public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;

  @Getter private final StatusSignal<Angle> yaw;
  private final StatusSignal<AngularVelocity> yawVelocity;

  public GyroIOPigeon2(CanId id) {
    pigeon = new Pigeon2(id.id(), id.bus());
    yaw = pigeon.getYaw();
    yawVelocity = pigeon.getAngularVelocityZWorld();

    Phoenix6Helper.checkErrorAndRetry(
        "Gyro config", () -> pigeon.getConfigurator().apply(new Pigeon2Configuration()));

    Phoenix6Helper.checkErrorAndRetry("Gyro zero", () -> pigeon.getConfigurator().setYaw(0.0));

    Phoenix6Helper.checkErrorAndRetry(
        "Gyro set yaw vel signal update frequency",
        () -> BaseStatusSignal.setUpdateFrequencyForAll(100., yaw, yawVelocity));

    Phoenix6Helper.checkErrorAndRetry(
        "Gyro optimize CAN utilization", pigeon::optimizeBusUtilization);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).isOK();

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
  }
}
