package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static frc.frc_java9485.constants.RobotConsts.*;

import org.littletonrobotics.junction.Logger;

import frc.frc_java9485.constants.mechanisms.ConveyorConsts;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretIO;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretSim;
import frc.robot.subsystems.swerve.Swerve;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class SuperStructureSim extends SubsystemBase{
  private final TurretSim turretSim;
  private IntakeSimulation intakeSimulation;

  private static SuperStructureSim m_instance;

  private SimAction currentAction = SimAction.SECURITY;

  public SuperStructureSim() {
    turretSim =
        new TurretSim(new TurretIO() {}, Swerve.getInstance()::getRobotRelativeSpeeds, Swerve.getInstance()::getPose2d);


    if (CURRENT_ROBOT_MODE == RobotModes.SIM) {
      intakeSimulation = IntakeSimulation.InTheFrameIntake("Fuel",
                                                           Swerve.getInstance().getSimulation(),
                                                           Meters.of(0.36),
                                                           IntakeSide.BACK,
                                                           ConveyorConsts.MAX_FUELS);
    }
  }

  @Override
  public void periodic() {
  }

  public enum SimAction {
    SHOOT_FUEL,
    CATCH_FUEL,
    CLOSE_INTAKE,
    LOCK_CONVEYOR,
    AUTOMATIC_HOOD,
    MANUAL_HOOD,
    LOCK_HOOD,
    CLIMBER_L1,
    AUTOMATIC_TURRET,
    MANUAL_TURRET,
    LOCK_TURRET,
    SECURITY
  }

  public Command setSimAction(SimAction action) {
    return new Command() {
          public void execute() {
              currentAction = action;
          }

      public boolean isFinished() {
          return currentAction == action;
      }
    };
  }

  public void simulationPeriodic() {
      switch (currentAction) {
        case SHOOT_FUEL:

          break;

          case CATCH_FUEL:
            intakeSimulation.startIntake();
            break;

          case CLOSE_INTAKE:
            intakeSimulation.stopIntake();
            break;

          case LOCK_HOOD:
            break;

          case CLIMBER_L1:
            break;

          case LOCK_TURRET:
            break;

          default:
            break;
    }
    Logger.recordOutput("Mechanism/Intake/Fuel Quantity", intakeSimulation.getGamePiecesAmount());
  }

  public SimAction getAction() {
    return currentAction;
  }
}
