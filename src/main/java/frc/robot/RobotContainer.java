package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.frc_java9485.autonomous.AutoChooser;
import frc.frc_java9485.constants.mechanisms.DriveConsts;
import frc.frc_java9485.joystick.driver.DriverJoystick;
import frc.frc_java9485.joystick.mechanism.MechanismJoystick;
import frc.frc_java9485.utils.RegisterNamedCommands;
import frc.robot.commands.swerveUtils.ResetPigeon;
import frc.robot.commands.swerveUtils.ResetSimGyro;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructure.Actions;
import frc.robot.subsystems.mechanism.index.IndexSubsystem;
import frc.robot.subsystems.mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static frc.frc_java9485.constants.RobotConsts.*;

public class RobotContainer {
  private final IndexSubsystem index;
  private final IntakeSubsystem intake;
  private final TurretSubsystem turret;
  private final SwerveSubsystem swerveSubsystem;

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
    index = IndexSubsystem.getInstance();
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

    turret.setDefaultCommand(turret.normalTurretCommand(
      () -> mechanismJoystick.getLeftX(),
      () -> mechanismJoystick.getRightY(),
      () -> mechanismJoystick.getRightTrigger(),
      () -> mechanismJoystick.getRightBumper(),
      () -> mechanismJoystick.y().getAsBoolean()));

      index.setDefaultCommand(index.turnOnCommand(
      () -> mechanismJoystick.getRightTrigger(),
      () -> mechanismJoystick.y().getAsBoolean()
    ));

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
    //drive
    driverJoystick.getLeftBack().onTrue(new ResetPigeon());

    //mecanismos
    mechanismJoystick.leftBumper().onTrue(Commands.runOnce(() -> superStructure.alternActions(Actions.OPEN_INTAKE), superStructure));

    mechanismJoystick.leftTrigger().onTrue(Commands.run(() -> superStructure.alternActions(Actions.CATCH_FUEL), superStructure))
    .onFalse(Commands.runOnce(() -> superStructure.alternActions(Actions.STOP_CATCH), superStructure));

    mechanismJoystick.x().onTrue(Commands.runOnce(() -> superStructure.alternActions(Actions.CLOSE_INTAKE), superStructure));

    mechanismJoystick.a().onTrue(Commands.runOnce(() -> superStructure.alternActions(Actions.OPEN_CONVEYOR), superStructure))
    .onFalse(Commands.runOnce(() -> superStructure.alternActions(Actions.LOCK_CONVEYOR), superStructure));

    mechanismJoystick.b().onTrue(Commands.runOnce(() -> superStructure.alternActions(Actions.CLOSE_CONVEYOR), superStructure))
    .onFalse(Commands.runOnce(() -> superStructure.alternActions(Actions.LOCK_CONVEYOR), superStructure));
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
