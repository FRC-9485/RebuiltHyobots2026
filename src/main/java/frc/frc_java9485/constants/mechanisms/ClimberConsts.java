package frc.frc_java9485.constants.mechanisms;

import edu.wpi.first.math.controller.PIDController;

public class ClimberConsts {
    public static final int LEFT_ID = 67; // ALTERAR
    public static final int RIGHT_ID = 66; // ALTERAR

    public static final double SETPOINT_LEFT_L1 = 0.0;
    public static final double SETPOINT_RIGHT_L1 = 0.0;

    public static final double SETPOINT_LEFT_L2 = 0.0;
    public static final double SETPOINT_RIGHT_L2 = 0.0;

    public static final double SETPOINT_LEFT_L3 = 0.0;
    public static final double SETPOINT_RIGHT_L3 = 0.0;

    private static final double LEFT_kP = 0.0;
    private static final double LEFT_kI = 0.0;
    private static final double LEFT_kD = 0.0;

    private static final double RIGHT_kP = 0.0;
    private static final double RIGHT_kI = 0.0;
    private static final double RIGHT_kD = 0.0;

    public static final PIDController LEFT_CONTROLLER = new PIDController(LEFT_kP, LEFT_kI, LEFT_kD);
    public static final PIDController RIGHT_CONTROLLER = new PIDController(RIGHT_kP, RIGHT_kI, RIGHT_kD);
}
