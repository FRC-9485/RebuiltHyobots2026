package frc.frc_java9485.constants.mechanisms;

import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class IntakeConsts {
    public static final int PIVOT_ID = 10;
    public static final int CATCH_BALL_ID = 9;

    public static final int ENCODER_A_CHANNEL = 8;
    public static final int ENCODER_B_CHANNEL = 9;
    public static final boolean ENCODER_INVERTED = true;
    public static final double ENCODER_DISTANCE_PER_PULSE = 360.0/2048.0;

    public static final double SETPOINT_UP = 113.00;
    public static final double SETPOINT_DOWN = 10;

    public static final ControlConstants PIVOT_CONTROL_CONSTANTS = new ControlConstants()
      .withPID(0.07, 0, 0.007)
      .withTolerance(3)
      .withProfile(10, 5)
      .withFeedforward(1.65, 0.1)
      .withPhysical(0.0, 0.0);

    public static final TunableControlConstants PIVOT_CONSTANTS =
        new TunableControlConstants("Throw Intake Consts",  PIVOT_CONTROL_CONSTANTS);
}
