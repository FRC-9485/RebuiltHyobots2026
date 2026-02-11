package frc.frc_java9485.constants.mechanisms;



import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.frc_java9485.loggers.LoggedPathPlannerPIDConstants;

public final class DriveConsts {
  public static final String ACTIVE_TRACJECTORY_LOG_ENTRY = "Swerve/Auto/Active Trajectory";
  public static final String TRAJECTORY_SETPOINT_LOG_ENTRY = "Swerve/Auto/Trajectory Setpoint";

  public static final Matrix<N4, N1> STATE_STD_DEVS = VecBuilder.fill(1, 1, 1, 0.1);
  public static final Matrix<N4, N1> VISION_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.5, 0);

  public static final int CANCODER_MODULE1_ID = 10;
  public static final int CANCODER_MODULE2_ID = 11;
  public static final int CANCODER_MODULE3_ID = 12;
  public static final int CANCODER_MODULE4_ID = 13;

  private static final double SIM_ROTATION_kP = 0.01001;
  private static final double SIM_ROTATION_kI = 0.00001;
  private static final double SIM_ROTATION_kD = 0.00007948059087543;

  private static final double SIM_TRANSLATION_kP = 0.014;
  private static final double SIM_TRANSLATION_kI = 0.00019;
  private static final double SIM_TRANSLATION_kD = 0.00075;

  private static final double REAL_TRANSLATION_kP = 0.256;
  private static final double REAL_TRANSLATION_kI = 0.000;
  private static final double REAL_TRANSLATION_kD = 0.000;

  private static final double REAL_ROTATION_kP = 0.03;
  private static final double REAL_ROTATION_kI = 0.000;
  private static final double REAL_ROTATION_kD = 0.01;

  public static LoggedPathPlannerPIDConstants REAL_TRANSLATION_PID =
    new LoggedPathPlannerPIDConstants("PathPlanner/Tuning/REAL Translation", REAL_TRANSLATION_kP, REAL_TRANSLATION_kI, REAL_TRANSLATION_kD);

  public static LoggedPathPlannerPIDConstants REAL_ROTATION_PID =
    new LoggedPathPlannerPIDConstants("PathPlanner/Tuning/REAL Rotation", REAL_ROTATION_kP, REAL_ROTATION_kI, REAL_ROTATION_kD);

  public static LoggedPathPlannerPIDConstants SIM_TRANSLATION_PID =
    new LoggedPathPlannerPIDConstants("PathPlanner/Tuning/SIM Translation", SIM_TRANSLATION_kP, SIM_TRANSLATION_kI, SIM_TRANSLATION_kD);

  public static LoggedPathPlannerPIDConstants SIM_ROTATION_PID =
    new LoggedPathPlannerPIDConstants("PathPlanner/Tuning/SIM Rotation", SIM_ROTATION_kP, SIM_ROTATION_kI, SIM_ROTATION_kD);


  public static final int ODOMETRY_FREQUENCY = 100; //HZ

  public static final double MAX_SPEED = 2.0;
  public static final boolean FIELD_ORIENTED = true;

  public static final Translation2d[] MODULES_TRANSLATIONS =
      new Translation2d[] {
        new Translation2d(0.356, 0.305),
        new Translation2d(0.356, -0.305),
        new Translation2d(-0.356, 0.305),
        new Translation2d(-0.356, -0.305)
      };
}
