package frc.lib.interfaces.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class GenericRollerIOSim implements GenericRollerIO {
  static final TrapezoidProfile.Constraints UNLIMITED =
      new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

  private final DCMotorSim sim;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(0.0, 0.0, 0.0, UNLIMITED, Constants.LOOP_PERIOD_SEC);
  private final SlewRateLimiter voltageLimiter = new SlewRateLimiter(2.5);

  private double appliedVoltageVolt = 0.0;

  public GenericRollerIOSim(DCMotor motor, double reduction, double moiJKgMeter2) {
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, moiJKgMeter2, reduction), motor);
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    if (DriverStation.isDisabled()) {
      setVoltage(voltageLimiter.calculate(appliedVoltageVolt));
    } else {
      voltageLimiter.reset(appliedVoltageVolt);
    }

    sim.update(Constants.LOOP_PERIOD_SEC);

    inputs.connected = true;

    inputs.velRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.outputVoltageVolt = appliedVoltageVolt;
    inputs.supplyCurrentAmp = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setPdf(double kp, double kd, double kf) {
    pid.setPID(kp, 0.0, kd);
  }

  @Override
  public void setVoltage(double voltageVolt) {
    appliedVoltageVolt = MathUtil.clamp(voltageVolt, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltageVolt);
  }

  @Override
  public void setVel(double velRadPerSec) {
    pid.setConstraints(UNLIMITED);
    setVoltage(pid.calculate(sim.getAngularVelocityRadPerSec(), velRadPerSec));
  }

  @Override
  public void setVel(double velRadPerSec, double accelRadPerSec2) {
    pid.setConstraints(new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, accelRadPerSec2));
    setVoltage(pid.calculate(sim.getAngularVelocityRadPerSec(), velRadPerSec));
  }

  // TODO
  @Override
  public void setCurrent(double currentAmp) {
    setVoltage(currentAmp);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
