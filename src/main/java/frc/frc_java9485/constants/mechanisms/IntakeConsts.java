package frc.frc_java9485.constants.mechanisms;

import frc.frc_java9485.utils.TunableControls.ControlConstants;
import frc.frc_java9485.utils.TunableControls.TunableControlConstants;

public class IntakeConsts {
    public static final class Motors{
      public static final int PIVOT_ID = 10;
      public static final int CATCH_BALL_ID = 9;
    }

    public static final class Encoder{
      public static final boolean ENCODER_INVERTED = true;
      public static final int ENCODER_CHANNEL = 9;
    }

    public static final class Setpoint{
      public static final double SETPOINT_UP = 250.00;
      public static final double SETPOINT_MIDDLE = 200;
      public static final double SETPOINT_DOWN = 16;
  
      public static final double STOPPED_FUEL_SPEED = 0;
      public static final double COLLECT_FUEL_SPEED = 0.7;
    }

    public static final class PID{
      public static final ControlConstants PIVOT_CONTROL_CONSTANTS = new ControlConstants()
        .withPID(0.035, 0.007, 0.0)
        .withTolerance(4)
        .withProfile(1000, 700)
        .withContinuous(-180, 180)
        .withPhysical(0.05, 0)
        .withFeedforward(0.015, 0);
  
      public static final TunableControlConstants PIVOT_CONSTANTS =
          new TunableControlConstants("Throw Intake Consts",  PIVOT_CONTROL_CONSTANTS);
    }
}
