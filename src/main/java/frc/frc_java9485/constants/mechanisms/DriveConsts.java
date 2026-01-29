package frc.frc_java9485.constants.mechanisms;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;

public final class DriveConsts {
    public static final String ACTIVE_TRACJECTORY_LOG_ENTRY = "Swerve/Auto/Active Trajectory";
    public static final String TRAJECTORY_SETPOINT_LOG_ENTRY = "Swerve/Auto/Trajectory Setpoint";

    public static final Matrix<N4, N1> STATE_STD_DEVS = VecBuilder.fill(1, 1, 1, 0.1);
    public static final Matrix<N4, N1> VISION_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.5, 0);

    public static final int CANCODER_MODULE1_ID = 10;
    public static final int CANCODER_MODULE2_ID = 11;
    public static final int CANCODER_MODULE3_ID = 12;
    public static final int CANCODER_MODULE4_ID = 13;

    public static final double AUTO_TRANSLATION_kP = 0.01;
    public static final double AUTO_TRANSLATION_kI = 0;
    public static final double AUTO_TRANSLATION_kD = 0.001;

    public static final double AUTO_ROTATION_kP = 0.08;
    public static final double AUTO_ROTATION_kI = 0;
    public static final double AUTO_ROTATION_kD = 0.0;

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
