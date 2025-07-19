package frc.lib.interfaces.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class GenericElevatorIOSim implements GenericElevatorIO {
  private final ElevatorSim sim;
  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private double appliedVoltageVolt = 0.0;

  private TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(0.0, 0.0));
  private TrapezoidProfile.State lastSetpoint = new TrapezoidProfile.State(0.0, 0.0);
  private boolean hasProfileInit = false;
  private boolean needResetProfile = false;

  private double lastGoalPositionMeter = 0.0;

  public GenericElevatorIOSim(
      DCMotor motor,
      double reduction,
      double massKg,
      double meterPerRotation,
      double minHeightMeter,
      double maxHeightMeter,
      double startingHeightMeter) {
    sim =
        new ElevatorSim(
            motor,
            reduction,
            massKg,
            meterPerRotation / Math.PI / 2.0,
            minHeightMeter,
            maxHeightMeter,
            false,
            startingHeightMeter);
    sim.setState(startingHeightMeter, 0.0);
  }

  @Override
  public void updateInputs(GenericElevatorIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(Constants.LOOP_PERIOD_SEC);

    inputs.connected = true;

    inputs.velMeterPerSec = sim.getVelocityMetersPerSecond();
    inputs.positionMeter = sim.getPositionMeters();
    inputs.outputVoltageVolt = appliedVoltageVolt;
    inputs.supplyCurrentAmp = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setPosition(double positionMeter, double feedforward) {
    // needResetProfile = true;
    // setVoltage(pid.calculate(sim.getPositionMeters(), positionMeter));
    sim.setState(positionMeter, 0.0);
  }

  @Override
  public void setPosition(
      double positionMeter, double velMeterPerSec, double accelMeterPerSec2, double feedforward) {
    sim.setState(positionMeter, 0.0);
    // needResetProfile = lastGoalPositionMeter != positionMeter;

    // if (!hasProfileInit) {
    //   profile =
    //       new TrapezoidProfile(new TrapezoidProfile.Constraints(velMeterPerSec,
    // accelMeterPerSec2));
    //   lastSetpoint =
    //       new TrapezoidProfile.State(sim.getPositionMeters(), sim.getVelocityMetersPerSecond());
    //   lastGoalPositionMeter = positionMeter;

    //   hasProfileInit = true;

    //   return;
    // } else if (needResetProfile) {
    //   profile =
    //       new TrapezoidProfile(new TrapezoidProfile.Constraints(velMeterPerSec,
    // accelMeterPerSec2));
    //   lastSetpoint =
    //       new TrapezoidProfile.State(sim.getPositionMeters(), sim.getVelocityMetersPerSecond());
    //   lastGoalPositionMeter = positionMeter;
    // }

    // var setpoint =
    //     profile.calculate(
    //         Constants.LOOP_PERIOD_SEC,
    //         lastSetpoint,
    //         new TrapezoidProfile.State(positionMeter, 0.0));

    // if (EqualsUtil.epsilonEquals(sim.getPositionMeters(), positionMeter, 0.03)) {
    //   setpoint = new TrapezoidProfile.State(positionMeter, 0.0);
    //   setVoltage(0.0);
    // } else {
    //   setVoltage(pid.calculate(sim.getPositionMeters(), setpoint.position));
    // }

    // lastGoalPositionMeter = positionMeter;
    // lastSetpoint = setpoint;
  }

  public void setVoltage(double voltageVolt) {
    needResetProfile = true;
    appliedVoltageVolt = MathUtil.clamp(voltageVolt, -12.0, 12.0);
    sim.setInputVoltage(appliedVoltageVolt);
  }

  @Override
  public void setPdf(double kp, double kd, double kf, double kg) {
    needResetProfile = pid.getP() != kp || pid.getD() != kd;
    pid.setPID(kp, 0.0, kd);
  }

  @Override
  public void setCurrent(double currentAmp) {
    needResetProfile = true;
    setVoltage(currentAmp);
  }

  @Override
  public void stop() {
    needResetProfile = true;
    setVoltage(0.0);
  }
}
