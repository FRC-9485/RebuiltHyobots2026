package frc.frc_java9485.constants.mechanisms;

import edu.wpi.first.math.controller.PIDController;

public class TurretConsts {
    public static final int TURN_ID = 72; // ALTERAR, ajeitar torreta
    public static final int CONTROL_ID = 71; // ALTERAR
    public static final int SHOOTER_ID = 70; // ALTERAR

    public static final int RPM_SETPOINT = 0; // ALTERAR
    public static final int SHOOTER_SPEED = 0; // ALTERAR

    private static final double TURN_kP = 0.0;
    private static final double TURN_kI = 0.0;
    private static final double TURN_kD = 0.0;

    private static final double CONTROL_kP = 0.0;
    private static final double CONTROL_kI = 0.0;
    private static final double CONTROL_kD = 0.0;

    public static final PIDController TURN_PID = new PIDController(TURN_kP, TURN_kI, TURN_kD);
    public static final PIDController CONTROL_PID = new PIDController(CONTROL_kP, CONTROL_kI, CONTROL_kD);
}
