package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.autonomous.AutoChooser;
import frc.frc_java9485.constants.mechanisms.DriveConsts;
import frc.frc_java9485.joystick.driver.DriverJoystick;
import frc.frc_java9485.joystick.mechanism.MechanismJoystick;
import frc.frc_java9485.utils.RegisterNamedCommands;
import frc.robot.commands.CatchBall;
import frc.robot.commands.swerveUtils.ResetSimGyro;
// import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructureSim;
import frc.robot.subsystems.mechanism.SuperStructureSim.SimAction;
import frc.robot.subsystems.mechanism.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;

import static frc.frc_java9485.constants.RobotConsts.*;

public class RobotContainer {
  private final Intake intake;
  private final Swerve swerveSubsystem;
  // private final Turret turret;

  // private SuperStructure superStructure;
  private SuperStructureSim superStructureSim;

  private final DriverJoystick driverJoystick;
  private final MechanismJoystick mechanismJoystick;

  private final AutoChooser autoChooser;
  private final RegisterNamedCommands namedCommands;

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    swerveSubsystem = Swerve.getInstance();
    intake = Intake.getInstance();
    // turret = new Turret(new TurretIO() {},swerveSubsystem::getRobotRelativeSpeeds, swerveSubsystem::getPose2d);
    // superStructure = new SuperStructure();

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
      superStructureSim = new SuperStructureSim();
      configureSimBindings();
    } else {
      // superStructure = new SuperStructure();
      configureBindings();
    }
    configureAutonomousCommands();
  }

  private void configureAutonomousCommands(){
    if (isSimulation()) {
      // namedCommands.configureSimCommands(intake, superStructure);
    } else {
      // namedCommands.configureRealCommands(intake, superStructure);
    }
  }

  private void configureBindings() {
    // mechanismJoystick.a().onTrue(superStructure.setAction(Actions.CATCH_FUEL));
    mechanismJoystick.x().whileTrue(new CatchBall(0.7));
    // mechanismJoystick.b().onTrue(superStructure.setAction(Actions.CLOSE_INTAKE));
  }

  private void configureSimBindings() {
    driverJoystick.getLeftBack().onTrue(new ResetSimGyro());

    driverJoystick.a().whileTrue(superStructureSim.setSimAction(SimAction.CATCH_FUEL));
    driverJoystick.a().whileTrue(superStructureSim.setSimAction(SimAction.CLOSE_INTAKE));
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.getAutonomousCommand(autoChooser.getSelectedOption(), true);
  }
}
