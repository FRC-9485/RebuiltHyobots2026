package frc.frc_java9485.utils;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.CatchBall;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructure.Actions;
import frc.robot.subsystems.mechanism.intake.Intake;

public class RegisterNamedCommands {
    public static RegisterNamedCommands mInstance = null;

    public static RegisterNamedCommands getInstance(){
        if(mInstance == null){
            mInstance = new RegisterNamedCommands();
        }
        return mInstance;
    }

    private RegisterNamedCommands(){}

    public void configureRealCommands(Intake intake, SuperStructure superStructure){
        configureIntakeCommands(intake);
    }

    public void configureSimCommands(Intake intke, SuperStructure superStructure) {
        configureIntakeSimCommands(superStructure);
        configureTurretSimCommands(superStructure);
    }

    private void configureIntakeCommands(Intake intake){
        NamedCommands.registerCommand("CATCH FUELS", new CatchBall(0.3).until(() -> Timer.getFPGATimestamp() > 20));
    }

    private void configureIntakeSimCommands(SuperStructure superStructure) {
        NamedCommands.registerCommand("coleta", superStructure.setActionSim(Actions.CATCH_FUEL));
        NamedCommands.registerCommand("parar coleta", superStructure.setActionSim(Actions.CLOSE_INTAKE));
    }

    private void configureTurretSimCommands(SuperStructure superStructure) {
        NamedCommands.registerCommand("shootar", superStructure.setActionSim(Actions.SHOOT_FUEL));
    }
}
