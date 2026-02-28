package frc.robot.commands.mechanism;

import static frc.frc_java9485.constants.mechanisms.TurretConsts.MAX_TURN_POSITION;
import static frc.frc_java9485.constants.mechanisms.TurretConsts.MIN_TURN_POSITION;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretSubsystem;

public class TurnTurretToSetpoint extends Command{

    private final TurretSubsystem turretSubsystem;

    private double setpoint;

    public TurnTurretToSetpoint(double setpoint, TurretSubsystem turretSubsystem){
        this.turretSubsystem = turretSubsystem;
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        if(setpoint >= MAX_TURN_POSITION){
            setpoint = MAX_TURN_POSITION;
        } else if(setpoint <= MIN_TURN_POSITION){
            setpoint = MIN_TURN_POSITION;
        }
    }

    @Override
    public void execute() {
        turretSubsystem.setSetpoint(setpoint);
        double output = turretSubsystem.turretController.calculate(turretSubsystem.getTurretPosition());

        turretSubsystem.setSpeed(output);
    }

    @Override
    public boolean isFinished() {
        return turretSubsystem.turretOnSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setSpeed(0);
    }
}
