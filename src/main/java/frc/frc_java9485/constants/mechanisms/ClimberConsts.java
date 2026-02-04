package frc.frc_java9485.constants.mechanisms;

import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class ClimberConsts {
    public static final int LEFT_ID = 67; // ALTERAR
    public static final int RIGHT_ID = 66; // ALTERAR

    public static final double SETPOINT_LEFT_L1 = 0.0;
    public static final double SETPOINT_RIGHT_L1 = 0.0;

    public static final double SETPOINT_LEFT_L2 = 0.0;
    public static final double SETPOINT_RIGHT_L2 = 0.0;

    public static final double SETPOINT_LEFT_L3 = 0.0;
    public static final double SETPOINT_RIGHT_L3 = 0.0;

    private static final ControlConstants LEFT_CONSTANTS = new ControlConstants()
      .withPID(0, 0, 0)
      .withProfile(0, 0)
      .withPhysical(0, 0)
      .withTolerance(0);

    private static final ControlConstants RIGHT_CONSTANTS = new ControlConstants()
      .withPID(0, 0, 0)
      .withProfile(0, 0)
      .withPhysical(0, 0)
      .withTolerance(0);

    public static final TunableControlConstants LEFT_CLIMBER_CONTROL_CONSTANTS =
        new TunableControlConstants("Left Climber PID", LEFT_CONSTANTS);

    public static final TunableControlConstants RIGHT_CLIMBER_CONTROL_CONSTANTS =
        new TunableControlConstants("Right Climber PID", RIGHT_CONSTANTS);
}
