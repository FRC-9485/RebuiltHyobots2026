package frc.robot.commands.mechanism;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.index.IndexSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretSubsystem;

public class ShotFuel extends Command{

    private final IndexSubsystem indexSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final double rpm;

    private final Timer timer;

    public ShotFuel(IndexSubsystem indexSubsystem, TurretSubsystem turretSubsystem, double rpm){
        this.turretSubsystem = turretSubsystem;
        this.indexSubsystem = indexSubsystem;
        this.timer = new Timer();
        this.rpm = rpm;

        addRequirements(turretSubsystem, indexSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        indexSubsystem.turnOn();
        turretSubsystem.shotWithRPM(rpm);

        if (timer.hasElapsed(3)) {
            turretSubsystem.turnOnFuelToTurret(-0.4);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(6);
    }

    @Override
    public void end(boolean interrupted) {
        indexSubsystem.stopIndex();
        turretSubsystem.shotWithRPM(0);
    }
}
