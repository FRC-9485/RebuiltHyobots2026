package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.intake.Intake;

public class CatchBall extends Command{

    Intake intake;
    double speed;

    public CatchBall(double speed){
        this.intake = Intake.getInstance();
        this.speed = speed;

        addRequirements(intake);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intake.catchFuel(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.catchFuel(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
