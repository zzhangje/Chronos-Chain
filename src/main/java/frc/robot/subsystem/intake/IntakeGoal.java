package frc.robot.subsystem.intake;

import edu.wpi.first.math.util.Units;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class IntakeGoal {
  public enum IntakePivotGoal {
    DOWN(
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Pivot/DownAngleRadians", 130.0)),
    IDLE(
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Pivot/IdleAngleRadians", 30.0)),
    REGRASP(
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Pivot/RegraspAngleRadians", 0.0)),
    DODGE(
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Pivot/DodgeAngleRadians", 50.0)),
    TROUGH(
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Pivot/TroughAngleRadians", 30.0));

    private final DoubleSupplier degreeSupplier;

    IntakePivotGoal(DoubleSupplier degreeSupplier) {
      this.degreeSupplier = degreeSupplier;
    }

    public double getAngleRadians() {
      return Units.degreesToRadians(degreeSupplier.getAsDouble());
    }
  }

  public enum IntakeRollerGoal {
    INJECT(
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Rolling/InjectVoltageVolts", -6.0),
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Centering/InjectVoltageVolts", -12.0)),
    STANDBY(
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Rolling/StandByVoltageVolts", -1.0),
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Centering/StandByVoltageVolts", -2.0)),
    TROUGH(
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Rolling/TroughVoltageVolts", 12.0),
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Centering/TroughVoltageVolts", 0.0)),
    EJECT(
        new LoggedTunableNumber(
            Constants.DebugGroup.INTAKE, "Intake/Rolling/EjectVoltageVolts", 12.0),
        () -> 0.0),
    IDLE(() -> 0.0, () -> 0.0);

    private final DoubleSupplier rollingVolts;
    private final DoubleSupplier centeringVolts;

    IntakeRollerGoal(DoubleSupplier rollingVolts, DoubleSupplier centeringVolts) {
      this.rollingVolts = rollingVolts;
      this.centeringVolts = centeringVolts;
    }

    public double getRollingVolts() {
      return rollingVolts.getAsDouble();
    }

    public double getCenteringVoltage() {
      return centeringVolts.getAsDouble();
    }
  }
}
