package frc.frc_java9485.constants.mechanisms;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class IntakeConsts {
    public static final int PIVOT_ID = 10;
    public static final int CATCH_BALL_ID = 9;

    public static final boolean ENCODER_INVERTED = true;
    public static final int ENCODER_CHANNEL = 9;

    public static final double SETPOINT_UP = 359.0;
    public static final double SETPOINT_DOWN = 117.0;

    public static final double STOPPED_FUEL_SPEED = 0;
    public static final double COLLECT_FUEL_SPEED = 0.7;

    public static final ControlConstants PIVOT_CONTROL_CONSTANTS = new ControlConstants()
      .withPID(0.085, 0.03, 0.012)
      .withTolerance(2)
      .withProfile(500, 300)
      .withContinuous(-180, 180)
      .withFeedforward(0.008, 0);

    public static final TunableControlConstants PIVOT_CONSTANTS =
        new TunableControlConstants("Throw Intake Consts",  PIVOT_CONTROL_CONSTANTS);

    static {
      SmartDashboard.putData("PID", PIVOT_CONSTANTS.getProfiledPIDController());
    }
}
