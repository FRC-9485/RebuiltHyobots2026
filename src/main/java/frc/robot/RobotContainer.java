package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.frc_java9485.autonomous.AutoChooser;
import frc.frc_java9485.constants.mechanisms.DriveConsts;
import frc.frc_java9485.joystick.driver.DriverJoystick;
import frc.frc_java9485.joystick.mechanism.MechanismJoystick;
import frc.frc_java9485.utils.RegisterNamedCommands;
import frc.robot.commands.CatchBall;
import frc.robot.commands.swerveUtils.ResetSimGyro;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructure.Actions;
import frc.robot.subsystems.mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.frc_java9485.constants.RobotConsts.*;

public class RobotContainer {
  private final IntakeSubsystem intake;
  private final SwerveSubsystem swerveSubsystem;
  private final TurretSubsystem turret;

  private SuperStructure superStructure;
  // private SuperStructureSim superStructureSim;

  private final DriverJoystick driverJoystick;
  private final MechanismJoystick mechanismJoystick;

  private final AutoChooser autoChooser;
  private final RegisterNamedCommands namedCommands;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    swerveSubsystem = SwerveSubsystem.getInstance();
    intake = IntakeSubsystem.getInstance();
    turret = new TurretSubsystem(new TurretIO() {}, swerveSubsystem::getRobotRelativeSpeeds, swerveSubsystem::getPose2d);
    superStructure = new SuperStructure(turret);

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

    if (isSimulation()) {
      // superStructureSim = new SuperStructureSim();
      configureSimBindings();
    } else {
      configureBindings();
    }
    configureAutonomousCommands();
  }

  private void configureAutonomousCommands(){
    if (isSimulation()) {
      namedCommands.configureSimCommands(intake, superStructure);
    } else {
      namedCommands.configureRealCommands(intake, superStructure);
    }
  }

  private void configureBindings() {
    driverJoystick.a().onTrue(Commands.runOnce(() -> superStructure.setAction(Actions.SHOOT_FUEL), superStructure))
    .onFalse(Commands.runOnce(() -> superStructure.setAction(Actions.LOCK_TURRET), superStructure));

    driverJoystick.x().whileTrue(new CatchBall(0.7));

    mechanismJoystick.rightTrigger().onTrue(Commands.runOnce(() -> superStructure.setAction(Actions.SHOOT_FUEL), superStructure));
    mechanismJoystick.rightTrigger().onFalse(Commands.runOnce(() -> superStructure.setAction(Actions.LOCK_TURRET), superStructure));

    mechanismJoystick.x().whileTrue(new CatchBall(0.7));

    mechanismJoystick.a().onTrue(Commands.runOnce(() -> superStructure.setAction(Actions.CATCH_FUEL), superStructure));
    mechanismJoystick.b().onTrue(Commands.runOnce(() -> superStructure.setAction(Actions.CLOSE_INTAKE), superStructure));
  }

  private void configureSimBindings() {
    driverJoystick.getLeftBack().onTrue(new ResetSimGyro());

    // driverJoystick.a().whileTrue(superStructureSim.setSimAction(SimAction.CATCH_FUEL));
    // driverJoystick.a().whileTrue(superStructureSim.setSimAction(SimAction.CLOSE_INTAKE));
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.getAutonomousCommand(autoChooser.getSelectedOption(), true);
  }
}
