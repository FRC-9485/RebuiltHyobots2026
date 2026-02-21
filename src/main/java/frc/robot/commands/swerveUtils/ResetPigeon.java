package frc.robot.commands.swerveUtils;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ResetPigeon extends Command {

  Pigeon2 pigeon2;
  SwerveSubsystem swerveSubsystem;

  public ResetPigeon() {
    this.swerveSubsystem = SwerveSubsystem.getInstance();
    this.pigeon2 = swerveSubsystem.getPigeon();
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("resetando o pigeon");
  }

  @Override
  public void execute() {
    pigeon2.reset();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return pigeon2.getYaw().getValueAsDouble() == 0;
  }
}
