package frc.robot.subsystems.mechanism.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.climber.ClimberSubsystem.ClimberStates;

public interface ClimberIO {
    public Command climbRobot(ClimberStates state);

    @AutoLog
    public static class ClimberInputs {
        double leftPosition = 0;
        double rightPosition = 0;
        double leftPIDOut = 0;
        double rightPIDOut = 0;
        ClimberStates currentState = ClimberStates.DEFAULT;
    }

    public void updateInputs(ClimberInputs inputs);
}
