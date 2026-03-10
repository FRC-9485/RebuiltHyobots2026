package frc.robot.commands.mechanism;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretSubsystem;

public class TurnToShotCenter extends Command{

    private final TurretSubsystem turretSubsystem;

    private final Timer timer;

    public TurnToShotCenter(TurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;
        this.timer = new Timer();

        addRequirements(turretSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        turretSubsystem.turnToMapSetpoint(0);
        turretSubsystem.turnHoodFromSetpoint(0.5);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3);
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.turnToMapSetpoint(0);
        turretSubsystem.turnHoodFromSetpoint(0);
    }
}
