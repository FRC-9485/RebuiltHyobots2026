package frc.frc_java9485.constants.mechanisms;

import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class TurretConsts {
    public static final int TURN_ID = 72; // ALTERAR, ajeitar torreta
    public static final int CONTROL_ID = 71; // ALTERAR
    public static final int SHOOTER_ID = 70; // ALTERAR

    public static final int BACKSPIN_RPM_SETPOINT = 0; // ALTERAR
    public static final int SHOOTER_SPEED = 0; // ALTERAR


    private static final ControlConstants TURN_CONSTANTS = new ControlConstants()
      .withPID(0, 0, 0)
      .withFeedforward(0, 0)
      .withPhysical(0, 0)
      .withProfile(0, 0)
      .withTolerance(0);

    private static final ControlConstants BACKSPIN_CONSTANTS = new ControlConstants()
      .withPID(0, 0, 0)
      .withFeedforward(0, 0)
      .withPhysical(0, 0)
      .withProfile(0, 0)
      .withTolerance(0);

    public static TunableControlConstants TUNNABLE_TURN_CONSTANTS = new TunableControlConstants("Turn PID", TURN_CONSTANTS);
    public static TunableControlConstants TUNNABLE_BACKSPIN_CONSTANTS = new TunableControlConstants("Backspin PID", BACKSPIN_CONSTANTS);
}
