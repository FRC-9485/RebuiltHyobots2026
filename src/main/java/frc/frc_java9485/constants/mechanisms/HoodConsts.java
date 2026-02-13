package frc.frc_java9485.constants.mechanisms;

import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class HoodConsts {
    public static final int MOTOR_ID = 15; // ALTERAR

    public static final int MAX_POSITION = 0; // ALTERAR
    public static final int MIN_POSITION = 0; // ALTERAR

    private static final ControlConstants CONTROL_CONSTANTS = new ControlConstants()
    .withPID(0.06, 0, 0)
    .withPhysical(0.4, 0.0)
    .withTolerance(7)
    .withProfile(100, 50);

    public static final TunableControlConstants TUNABLE_CONSTANTS =
        new TunableControlConstants("Estêvão Barros Reis (electric boss) loves jhonny from lipetral, Linhares, ES, Brazil", CONTROL_CONSTANTS);
}
