package frc.lib.interfaces.motor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class GenericArmIOSim implements GenericArmIO {
  private final SingleJointedArmSim sim;
  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private double appliedVoltageVolt = 0.0;
  private double offset = 0.0;

  private TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(0.0, 0.0));
  private TrapezoidProfile.State lastSetpoint = new TrapezoidProfile.State(0.0, 0.0);
  private boolean hasProfileInit = false;
  private boolean needResetProfile = false;

  private double lastGoalPositionRad = 0.0;

  public GenericArmIOSim(
      DCMotor motor,
      double reduction,
      double moiJKgMeter2,
      double armLengthMeter,
      double minAngleRad,
      double maxAngleRad,
      double startingAngleRad) {
    sim =
        new SingleJointedArmSim(
            motor,
            reduction,
            moiJKgMeter2,
            armLengthMeter,
            minAngleRad,
            maxAngleRad,
            false,
            startingAngleRad);
  }

  public void setVoltage() {}

  @Override
  public void updateInputs(GenericArmIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }
    sim.update(Constants.LOOP_PERIOD_SEC);

    inputs.connected = true;

    inputs.velRadPerSec = sim.getVelocityRadPerSec();
    inputs.positionRad = sim.getAngleRads();
    inputs.outputVoltageVolt = appliedVoltageVolt;
    inputs.supplyCurrentAmp = Math.abs(sim.getCurrentDrawAmps());
  }

  @Override
  public void setPosition(double positionRad, double feedforward) {
    sim.setState(positionRad, 0.0);
    // needResetProfile = true;
    // setVoltage(pid.calculate(sim.getAngleRads(), positionRad + offset));
  }

  @Override
  public void setPosition(
      double positionRad, double velRadPerSec, double accelRadPerSec2, double feedforward) {
    sim.setState(positionRad, 0.0);
    // setPosition(positionRad, feedforward);

    // needResetProfile = lastGoalPositionRad != positionRad;

    // if (!hasProfileInit) {
    //   profile =
    //       new TrapezoidProfile(new TrapezoidProfile.Constraints(velRadPerSec, accelRadPerSec2));
    //   lastSetpoint = new TrapezoidProfile.State(sim.getAngleRads(), sim.getVelocityRadPerSec());
    //   lastGoalPositionRad = positionRad;

    //   hasProfileInit = true;

    //   return;
    // } else if (needResetProfile) {
    //   profile =
    //       new TrapezoidProfile(new TrapezoidProfile.Constraints(velRadPerSec, accelRadPerSec2));
    //   lastSetpoint = new TrapezoidProfile.State(sim.getAngleRads(), sim.getVelocityRadPerSec());
    //   lastGoalPositionRad = positionRad;
    // }

    // var setpoint =
    //     profile.calculate(
    //         Constants.LOOP_PERIOD_SEC, lastSetpoint, new TrapezoidProfile.State(positionRad,
    // 0.0));

    // if (EqualsUtil.epsilonEquals(sim.getAngleRads(), positionRad, Units.degreesToRadians(3.0))) {
    //   setpoint = new TrapezoidProfile.State(positionRad, 0.0);
    //   setVoltage(0.0);
    // } else {
    //   setVoltage(pid.calculate(sim.getAngleRads(), setpoint.position));
    // }

    // lastGoalPositionRad = positionRad;
    // lastSetpoint = setpoint;
  }

  @Override
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

  @Override
  public void home(double position) {
    needResetProfile = true;
    offset = position - sim.getAngleRads();
  }
}
