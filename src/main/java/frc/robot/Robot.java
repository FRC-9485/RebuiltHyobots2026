package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static frc.frc_java9485.constants.FieldConsts.FieldMeansureds.*;
import static frc.frc_java9485.constants.FieldConsts.HubMeansured.*;
import static frc.frc_java9485.constants.FieldConsts.SimulationPoses.*;
import frc.frc_java9485.constants.RobotConsts;
import frc.frc_java9485.utils.Elastic;
import frc.frc_java9485.utils.HubTracker;
import frc.frc_java9485.utils.Elastic.Notification;
import frc.frc_java9485.utils.Elastic.Notification.NotificationLevel;
import frc.frc_java9485.utils.Simulation;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private double currentMatchTime;
  private boolean runnedAutonomous;

  private final Timer timer;
  private final PowerDistribution powerDistribution;

  private final Optional<Time> shiftTime = HubTracker.timeRemainingInCurrentShift();

  private Simulation simulator;
  private final SwerveSubsystem swerve;
  private final RobotContainer m_robotContainer;

  public Robot() {
    switch (RobotConsts.CURRENT_ROBOT_MODE) {
      case REAL:
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter(RobotConsts.LOGS_PATH));
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        simulator = Simulation.getInstance();
        break;
    }

    Logger.registerURCL(URCL.startExternal());
    Logger.start();

    m_robotContainer = new RobotContainer();
    swerve = SwerveSubsystem.getInstance();

    timer = new Timer();
    powerDistribution = new PowerDistribution();

    currentMatchTime = 0.00;
    runnedAutonomous = false;

    if(shiftTime.isPresent()){
      SmartDashboard.putNumber("time do shift", shiftTime.get().in(Seconds));
    }
  }

  @Override
  public void robotInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("hub is active", HubTracker.isActive());
    SmartDashboard.putNumber("batery voltage", powerDistribution.getVoltage());

    if (powerDistribution.getVoltage() <= 11.4
        && timer.advanceIfElapsed(10)
        && !DriverStation.isFMSAttached()) {
      String desc = String.format("Bateria com %.2f Volts", powerDistribution.getVoltage());
      Elastic.sendNotification(
          new Notification(NotificationLevel.WARNING, "BATERIA BAIXA!!", desc));
    }

    currentMatchTime = DriverStation.getMatchTime();
    SmartDashboard.putNumber("Match Time", currentMatchTime);
  }

  @Override
  public void autonomousInit() {
    Elastic.sendNotification(
        new Notification(NotificationLevel.INFO, "Inicio do Autonomous!!", ""));

    runnedAutonomous = true;
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Inicio do teleop!!", ""));
      if (!runnedAutonomous) {
        var alliancePosition = DriverStation.getRawAllianceStation();
        switch (alliancePosition) {
          case Blue1:
          System.out.println("resetando...");
            swerve.resetOdometry(BLUE_LEFT_START_POSE);
            break;
          case Blue2:
            swerve.resetOdometry(BLUE_CENTER_START_POSE);
            break;
          case Blue3:
            swerve.resetOdometry(BLUE_RIGHT_START_POSE);
            break;

          case Red1:
            swerve.resetOdometry(RED_LEFT_START_POSE);
            break;
          case Red2:
            swerve.resetOdometry(RED_CENTER_START_POSE);
            break;
          case Red3:
            swerve.resetOdometry(RED_RIGHT_START_POSE);
            break;

          case Unknown:
            swerve.resetOdometry(FIELD_CENTER_POSE);
            break;
        }
      }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if(HubTracker.isActive()){
      Elastic.sendNotification(new Notification(NotificationLevel.INFO,
                                          "Hub is active",
                                    "the hub is active"));
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void simulationPeriodic() {
    simulator.updateArena();
  }
}
