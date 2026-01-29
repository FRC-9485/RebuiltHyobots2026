package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.autonomous.AutoChooser;
import frc.frc_java9485.constants.mechanisms.DriveConsts;
import frc.frc_java9485.joystick.driver.DriverJoystick;
import frc.frc_java9485.joystick.mechanism.MechanismJoystick;
import frc.frc_java9485.utils.RegisterNamedCommands;
import frc.robot.commands.CatchBall;
import frc.robot.commands.swerveUtils.ResetPigeon;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructure.Actions;
import frc.robot.subsystems.mechanism.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {
  private final Intake intake;
  private final Swerve swerveSubsystem;
  private final SuperStructure superStructure;

  private final DriverJoystick driverJoystick;
  private final MechanismJoystick mechanismJoystick;

  private final AutoChooser autoChooser;
  private final RegisterNamedCommands namedCommands;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    swerveSubsystem = Swerve.getInstance();
    intake = Intake.getInstance();
    superStructure = SuperStructure.getInstance();

    driverJoystick = DriverJoystick.getInstance();
    mechanismJoystick = MechanismJoystick.getInstance();

    autoChooser = new AutoChooser("Autonomous Chooser", "path");
    namedCommands = RegisterNamedCommands.getInstance();

    swerveSubsystem.setDefaultCommand(
        swerveSubsystem.driveCommand(
            () -> driverJoystick.getLeftY(),
            () -> driverJoystick.getLeftX(),
            () -> driverJoystick.getRightX(),
            DriveConsts.FIELD_ORIENTED));

    configureBindings();
    configureAutonomousCommands();
  }

  private void configureAutonomousCommands(){
    namedCommands.configureAllCommands(intake);
  }

  private void configureBindings() {
    driverJoystick.getLeftBack().onTrue(new ResetPigeon());

    mechanismJoystick.a().whileTrue(superStructure.setAction(Actions.CATCH_FUEL));
    mechanismJoystick.b().whileTrue(superStructure.setAction(Actions.CLOSE_INTAKE));

    mechanismJoystick.x().whileTrue(new CatchBall(0.3));
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.getAutonomousCommand(autoChooser.getSelectedOption(), true);
  }
}
