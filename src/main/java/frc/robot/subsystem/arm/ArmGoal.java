package frc.robot.subsystem.arm;

import edu.wpi.first.math.util.Units;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.Constants.DebugGroup;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;

public class ArmGoal {
  @RequiredArgsConstructor
  public enum EndEffectorGoal {
    IDLE(new LoggedTunableNumber(DebugGroup.ARM, "EndEffector/Goal/IdleVoltageVolt", 0.0)),
    HOLDING(new LoggedTunableNumber(DebugGroup.ARM, "EndEffector/Goal/HoldingVoltageVolt", -12.0)),
    CORAL_COLLECT(
        new LoggedTunableNumber(DebugGroup.ARM, "EndEffector/Goal/CoralCollectVoltageVolt", -12.0)),
    ALGAE_COLLECT(
        new LoggedTunableNumber(DebugGroup.ARM, "EndEffector/Goal/AlgaeCollectVoltageVolt", -12.0)),
    EJECT(new LoggedTunableNumber(DebugGroup.ARM, "EndEffector/Goal/EjectVoltageVolt", 1.0)),
    ALGAE_SCORE(
        new LoggedTunableNumber(DebugGroup.ARM, "EndEffector/Goal/AlgaeScoreVoltageVolt", 13.0)),
    CORAL_SCORE(
        new LoggedTunableNumber(DebugGroup.ARM, "EndEffector/Goal/CoralScoreVoltageVolt", 1.0));

    private final DoubleSupplier voltage;

    public double getVoltageVolt() {
      return voltage.getAsDouble();
    }
  }

  @RequiredArgsConstructor
  public enum ArmSubsystemGoal {
    START(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/StartHeightMeter", 0.0),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/StartPositionDegree", 90.0)),
    IDLE(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/IdleHeightMeter", 0.0),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/IdlePositionDegree", 90.0)),
    ALGAE_IDLE(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/AlgaeIdleHeightMeter", 0.2),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/AlgaeIdlePositionDegree", 90.0)),
    CORAL_GROUND_PICK(
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Shoulder/Goal/CoralGroundPickHeightMeter", 0.82),
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Elbow/Goal/CoralGroundPickPositionDegree", -90.0)),
    CORAL_IDLE(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/CoralIdleHeightMeter", 0.82),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/CoralIdlePositionDegree", 90.0)),
    CORAL_L1_PRESCORE(
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Shoulder/Goal/CoralL1PrescoreHeightMeter", 0.0),
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Elbow/Goal/CoralL1PrescorePositionDegree", 47.0)),
    CORAL_L2_PRESCORE(
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Shoulder/Goal/CoralL2PrescoreHeightMeter", 0.4),
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Elbow/Goal/CoralL2PrescorePositionDegree", 45.0)),
    CORAL_L3_PRESCORE(
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Shoulder/Goal/CoralL3PrescoreHeightMeter", 0.8),
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Elbow/Goal/CoralL3PrescorePositionDegree", 45.0)),
    CORAL_L4_PRESCORE(
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Shoulder/Goal/CoralL4PrescoreHeightMeter", 1.4),
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Elbow/Goal/CoralL4PrescorePositionDegree", 45.0)),
    CORAL_L1_SCORE(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/CoralL1ScoreHeightMeter", 0.0),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/CoralL1ScorePositionDegree", 55.0)),
    CORAL_L2_SCORE(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/CoralL2ScoreHeightMeter", 0.32),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/CoralL2ScorePositionDegree", 0.0)),
    CORAL_L3_SCORE(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/CoralL3ScoreHeightMeter", 0.72),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/CoralL3ScorePositionDegree", 0.0)),
    CORAL_L4_SCORE(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/CoralL4ScoreHeightMeter", 1.4),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/CoralL4ScorePositionDegree", 0.0)),
    ALGAE_GROUND_PICK(
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Shoulder/Goal/AlgaeGroundPickHeightMeter", 0.15),
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Elbow/Goal/AlgaeGroundPickPositionDegree", 180.0)),
    ALGAE_LOW_PICK(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/AlgaeLowPickHeightMeter", 0.63),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/AlgaeLowPickPositionDegree", 0.0)),
    ALGAE_HIGH_PICK(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/AlgaeHighPickHeightMeter", 1.1),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/AlgaeHighPickPositionDegree", 0.0)),
    ALGAE_PROCESSOR_SCORE(
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Shoulder/Goal/AlgaeProcessorScoreHeightMeter", 0.15),
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Elbow/Goal/AlgaeProcessorScorePositionDegree", 0.0)),
    ALGAE_NET_SCORE(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/AlgaeNetScoreHeightMeter", 1.5),
        new LoggedTunableNumber(
            DebugGroup.ARM, "Arm/Elbow/Goal/AlgaeNetScorePositionDegree", 115.0)),
    CLIMB(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/ClimbHeightMeter", 0.0),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/ClimbPositionDegree", 90.0)),
    HOME(
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Shoulder/Goal/HomeHeightMeter", 0.0),
        new LoggedTunableNumber(DebugGroup.ARM, "Arm/Elbow/Goal/HomePositionDegree", 90.0));

    private final DoubleSupplier shoulderHeightMeterSupplier;
    // should in range [-90, +90]
    private final DoubleSupplier elbowPositionDegreeSupplier;

    // This is used to determine the elbow position based on the side of the robot
    private BooleanSupplier isLeftSupplier = () -> false;

    public ArmSubsystemGoal setIsLeft(BooleanSupplier isLeftSupplier) {
      this.isLeftSupplier = isLeftSupplier;
      return this;
    }

    public Boolean getIsLeft() {
      return isLeftSupplier.getAsBoolean();
    }

    public double getShoulderHeightMeter() {
      return shoulderHeightMeterSupplier.getAsDouble();
    }

    public double getElbowPositionRad() {
      return isLeftSupplier.getAsBoolean()
          ? Units.degreesToRadians(elbowPositionDegreeSupplier.getAsDouble())
          : Math.PI - Units.degreesToRadians(elbowPositionDegreeSupplier.getAsDouble());
    }
  }
}
