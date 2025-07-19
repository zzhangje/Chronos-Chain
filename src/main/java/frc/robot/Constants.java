package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.dashboard.LoggedTunableNumber;
import frc.lib.interfaces.CanId;
import java.util.function.BooleanSupplier;

public class Constants {
  public static final double LOOP_PERIOD_SEC = 0.02;

  public static BooleanSupplier IS_LIVE_DEBUG = () -> true;

  public static final Mode MODE =
      RobotBase.isReal() ? Mode.REAL : RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY;

  public static final boolean ENABLE_SINGLE_TAG_POSE = false;

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public final class Misc {
    public static final Transform3d intake_T_coral =
        new Transform3d(0.225, -0.15, 0.07, new Rotation3d(0.0, 0.0, Math.PI / 2));
    public static final Transform3d ee_T_coral =
        new Transform3d(0.225, -0.15, 0.5, new Rotation3d(0.0, 0.0, Math.PI / 2));
    public static final Transform3d ee_T_algae =
        new Transform3d(0.225, 0.0, 0.65, new Rotation3d());

    public static final Transform2d leftSide =
        new Transform2d(0.0, -0.2169, new Rotation2d(+Math.PI / 2.0));
    public static final Transform2d rightSide =
        new Transform2d(0.0, +0.2169, new Rotation2d(-Math.PI / 2.0));

    public static final LoggedTunableNumber omegaCOGHeightScaleFactor =
        new LoggedTunableNumber("Swerve", "TeleopController/OmegaCOGHeightScaleFactor", 0.1);
    public static final LoggedTunableNumber accelCOGHeightScaleFactor =
        new LoggedTunableNumber("Swerve", "TeleopController/AccelCOGHeightScaleFactor", 0.1);
  }

  public final class AscopeAssets {

    public static final int CHASSIS = 0;
    public static final int ELEVATOR_1 = 1;
    public static final int ELEVATOR_2 = 2;
    public static final int CARRIAGE = 3;
    public static final int ARM = 4;
    public static final int INTAKE_BASE = 5;
    public static final int INTAKE = 6;
    public static final int CLIMBER_BASE = 7;
    public static final int CLIMBER = 8;
    public static final int ALGAE = 9;
    public static final int CORAL = 10;
    public static final int CORAL_INTAKE = 11;
  }

  public final class DebugGroup {
    public static final String SWERVE = "Swerve";
    public static final String CLIMBER = "Climber";
    public static final String INTAKE = "Intake";
    public static final String ARM = "Arm";
    public static final String ODOMETRY = "Odometry";
    public static final String AUTO = "Auto";
  }

  public final class Ports {
    public static final class Can {
      public static final String RIO_BUS = "";
      public static final String CHASSIS_CANIVORE_BUS = "chassis";
      public static final String SUPERSTRUCTURE_CANIVORE_BUS = "super";

      // Led
      public static final CanId CANDLE = new CanId(0, RIO_BUS);

      // Climber
      public static final CanId CLIMBER_ARM = new CanId(6, SUPERSTRUCTURE_CANIVORE_BUS);
      public static final CanId CLIMBER_ROLLER = new CanId(7, SUPERSTRUCTURE_CANIVORE_BUS);

      // IMU
      public static final CanId CHASSIS_PIGEON = new CanId(0, CHASSIS_CANIVORE_BUS);

      // Swerve
      public static final CanId FL_DRIVE_MOTOR = new CanId(1, CHASSIS_CANIVORE_BUS);
      public static final CanId FL_STEER_MOTOR = new CanId(2, CHASSIS_CANIVORE_BUS);
      public static final CanId FL_STEER_SENSOR = new CanId(3, CHASSIS_CANIVORE_BUS);

      public static final CanId BL_DRIVE_MOTOR = new CanId(4, CHASSIS_CANIVORE_BUS);
      public static final CanId BL_STEER_MOTOR = new CanId(5, CHASSIS_CANIVORE_BUS);
      public static final CanId BL_STEER_SENSOR = new CanId(6, CHASSIS_CANIVORE_BUS);

      public static final CanId BR_DRIVE_MOTOR = new CanId(7, CHASSIS_CANIVORE_BUS);
      public static final CanId BR_STEER_MOTOR = new CanId(8, CHASSIS_CANIVORE_BUS);
      public static final CanId BR_STEER_SENSOR = new CanId(9, CHASSIS_CANIVORE_BUS);

      public static final CanId FR_DRIVE_MOTOR = new CanId(10, CHASSIS_CANIVORE_BUS);
      public static final CanId FR_STEER_MOTOR = new CanId(11, CHASSIS_CANIVORE_BUS);
      public static final CanId FR_STEER_SENSOR = new CanId(12, CHASSIS_CANIVORE_BUS);

      // Ground Intake
      public static final CanId GROUND_INTAKE_ROLLER = new CanId(13, CHASSIS_CANIVORE_BUS);
      public static final CanId GROUND_INTAKE_PIVOT = new CanId(14, CHASSIS_CANIVORE_BUS);
      public static final CanId GROUND_INTAKE_CENTERING = new CanId(15, CHASSIS_CANIVORE_BUS);

      // End Effector
      public static final CanId END_EFFECTOR = new CanId(0, SUPERSTRUCTURE_CANIVORE_BUS);
      public static final CanId END_EFFECTOR_CANDI = new CanId(1, SUPERSTRUCTURE_CANIVORE_BUS);

      // Arm
      public static final CanId ARM_ELBOW = new CanId(2, SUPERSTRUCTURE_CANIVORE_BUS);
      public static final CanId ARM_ELBOW_CANCODER = new CanId(3, SUPERSTRUCTURE_CANIVORE_BUS);
      public static final CanId ARM_SHOULDER_MASTER = new CanId(4, SUPERSTRUCTURE_CANIVORE_BUS);
      public static final CanId ARM_SHOULDER_SLAVE = new CanId(5, SUPERSTRUCTURE_CANIVORE_BUS);
    }

    public static final class Digital {
      public static final int GROUND_INTAKE_BEAM_BREAK = 9;
      public static final int CLIMBER_BEAM_BREAK = 2;
    }

    public static final class Joystick {
      public static final int DRIVER = 0;
    }
  }
}
