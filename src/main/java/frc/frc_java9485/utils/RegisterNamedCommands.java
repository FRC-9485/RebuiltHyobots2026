package frc.frc_java9485.utils;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.mechanism.CatchBall;
import frc.robot.commands.mechanism.ShotFuel;
import frc.robot.commands.mechanism.TurnToShotCenter;
import frc.robot.commands.mechanism.TurnToShotRight;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructure.Actions;
import frc.robot.subsystems.mechanism.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.mechanism.index.IndexSubsystem;
import frc.robot.subsystems.mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretSubsystem;

public class RegisterNamedCommands {
    public static RegisterNamedCommands mInstance = null;

    public static RegisterNamedCommands getInstance(){
        if(mInstance == null){
            mInstance = new RegisterNamedCommands();
        }
        return mInstance;
    }

    private RegisterNamedCommands(){}

    public void configureRealCommands(IntakeSubsystem intake, SuperStructure superStructure, ConveyorSubsystem conveyorSubsystem,
     TurretSubsystem turretSubsystem, IndexSubsystem indexSubsystem){
        configureIntakeCommands(intake, superStructure, conveyorSubsystem);
        configureConveyorCommands(superStructure, conveyorSubsystem);
        configureTurretCommands(turretSubsystem, indexSubsystem);
    }

    public void configureSimCommands(IntakeSubsystem intke, SuperStructure superStructure) {
        configureIntakeSimCommands(superStructure);
        configureTurretSimCommands(superStructure);
    }

    private void configureIntakeCommands(IntakeSubsystem intake, SuperStructure superStructure, ConveyorSubsystem conveyorSubsystem){
        NamedCommands.registerCommand("coleta", new CatchBall(0.7).withTimeout(5));
        NamedCommands.registerCommand("down intake", Commands.run(() -> superStructure.alternActions(Actions.OPEN_INTAKE), superStructure)
        .onlyIf(() -> conveyorSubsystem.conveyorInLimit())
        .until(() -> intake.atSetpoint()));
    }

    private void configureConveyorCommands(SuperStructure superStructure, ConveyorSubsystem conveyorSubsystem){
        NamedCommands.registerCommand("open conveyor", Commands.run(() -> superStructure.alternActions(Actions.OPEN_CONVEYOR), superStructure)
        .until(() -> conveyorSubsystem.conveyorInLimit()));
    }

    private void configureTurretCommands(TurretSubsystem turretSubsystem, IndexSubsystem indexSubsystem){
        NamedCommands.registerCommand("turn to right", new TurnToShotRight(turretSubsystem));
        NamedCommands.registerCommand("shot fuel right", new ShotFuel(indexSubsystem, turretSubsystem, 2820));

        NamedCommands.registerCommand("turn to center", new TurnToShotCenter(turretSubsystem));
        NamedCommands.registerCommand("shot fuel center", new ShotFuel(indexSubsystem, turretSubsystem, 3000));
    }

    private void configureIntakeSimCommands(SuperStructure superStructure) {
        NamedCommands.registerCommand("coleta", superStructure.alternActionsSim(Actions.CATCH_FUEL));
        NamedCommands.registerCommand("parar coleta", superStructure.alternActionsSim(Actions.CLOSE_INTAKE));
    }

    private void configureTurretSimCommands(SuperStructure superStructure) {
        NamedCommands.registerCommand("shootar", superStructure.alternActionsSim(Actions.SHOOT_FUEL));
    }
}
