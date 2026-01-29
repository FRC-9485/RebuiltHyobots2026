package frc.frc_java9485.utils;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.CatchBall;
import frc.robot.subsystems.mechanism.intake.Intake;

public class RegisterNamedCommands {

    Intake intake;

    public static RegisterNamedCommands mInstance = null;

    public static RegisterNamedCommands getInstance(){
        if(mInstance == null){
            mInstance = new RegisterNamedCommands();
        }
        return mInstance;
    }

    private RegisterNamedCommands(){
        intake = Intake.getInstance();
    }

    public void configureAllCommands(Intake intake){
        configureIntakeCommands(intake);
    }

    private void configureIntakeCommands(Intake intake){
        NamedCommands.registerCommand("CATCH FUELS", new CatchBall(0.3).until(() -> Timer.getFPGATimestamp() > 20));
    }
}
