package frc.frc_java9485.constants.mechanisms;

import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class HoodConsts {
    public static final int HOOD_MOTOR_ID = 15; // ALTERAR

    public static final double MAX_POSITION = 40.0; // ALTERAR
    public static final double MIN_POSITION = 0.0; // ALTERAR

    public static final int HOOD_CURRENT_LIMIT = 30;

    private static final ControlConstants CONTROL_CONSTANTS = new ControlConstants()
    .withPID(0.1, 0, 0)
    .withProfile(300, 200)
    .withTolerance(1.5)
    .withFeedforward(0.01, 0);

    public static final TunableControlConstants TUNABLE_CONSTANTS =
        new TunableControlConstants("Estêvão Barros Reis (electric boss) loves jhonny from lipetral, Linhares, ES, Brazil", CONTROL_CONSTANTS);
}
