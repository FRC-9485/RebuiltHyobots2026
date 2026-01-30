package frc.robot.commands.swerveUtils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;
import swervelib.simulation.ironmaple.simulation.drivesims.GyroSimulation;

public class ResetSimGyro extends Command {

    Swerve swerveSubsystem;
    GyroSimulation gyro;

  public ResetSimGyro() {
    this.swerveSubsystem = Swerve.getInstance();
    this.gyro = swerveSubsystem.getGyroSimulation();
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    gyro.setRotation(new Rotation2d());
}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return gyro.getGyroReading().getDegrees() == 0;
  }
}
