package frc.frc_java9485.constants.mechanisms;

import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class HoodConsts {
    public static final class Motor{
        public static final int HOOD_MOTOR_ID = 15; // ALTERAR

        public static final int HOOD_CURRENT_LIMIT = 30;
    }

    public static final class Setpoint{
        public static final double MAX_POSITION = 3.5;
        public static final double MIN_POSITION = 0.0;
    }

    public static final class PID {
        private static final ControlConstants CONTROL_CONSTANTS = new ControlConstants()
        .withPID(0.15, 0, 0)
        .withProfile(300, 200)
        .withContinuous(-180, 180)
        .withTolerance(0.02);
    
        public static final TunableControlConstants TUNABLE_CONSTANTS =
            new TunableControlConstants("hood constants", CONTROL_CONSTANTS);
    }
}
