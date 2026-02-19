package frc.frc_java9485.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class RobotConsts {
    public static final RobotModes SIM_MODE = RobotModes.SIM;
    public static final RobotModes CURRENT_ROBOT_MODE =
        RobotBase.isReal() ? RobotModes.REAL : RobotModes.SIM;

    public static final String LOGS_PATH = "/home/lvuser/logs";
    public static final String SPARK_ODOMETRY_THREAD_PATH = "Spark Odometry Thread";

    public static enum RobotModes {
      SIM,
      REAL
    }

    public static boolean isSimulation() {
      return CURRENT_ROBOT_MODE == RobotModes.SIM;
    }
  }
