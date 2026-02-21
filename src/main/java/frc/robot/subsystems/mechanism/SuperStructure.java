package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static frc.frc_java9485.constants.RobotConsts.*;
import static frc.frc_java9485.constants.mechanisms.IntakeConsts.*;

import org.littletonrobotics.junction.Logger;

import frc.frc_java9485.constants.mechanisms.ConveyorConsts;
import frc.robot.subsystems.mechanism.conveyor.ConveyorSubsystem;
import frc.robot.subsystems.mechanism.index.IndexSubsystem;
import frc.robot.subsystems.mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretSubsystem;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretSubsystem.TurretGoal;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class SuperStructure extends SubsystemBase{
  private final IntakeSubsystem intake;
  private final ConveyorSubsystem conveyor;
  private final TurretSubsystem turret;
  private final IndexSubsystem index;
  // private final TurretSimTeste turretSim;

  private Actions currentAction = Actions.SECURITY;
  private static SuperStructure m_instance;

  private IntakeSimulation intakeSimulation;

  public SuperStructure(TurretSubsystem turret) {
    intake = IntakeSubsystem.getInstance();
    conveyor = ConveyorSubsystem.getInstance();
    index = IndexSubsystem.getInstance();
    this.turret = turret;

    if (isSimulation()) {
      intakeSimulation = IntakeSimulation.InTheFrameIntake("Fuel",
                                                           SwerveSubsystem.getInstance().getSimulation(),
                                                           Meters.of(0.36),
                                                           IntakeSide.BACK,
                                                           ConveyorConsts.MAX_FUELS);
    }
  }

  @Override
  public void periodic() {
    executeAction(getAction());
  }

  public enum Actions {
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

  public void executeAction(Actions actions){
      switch (actions) {
        case SHOOT_FUEL:
            index.turnOn();
            turret.setGoal(TurretGoal.SCORING);
            break;

        case CATCH_FUEL:
            intake.enablePivot(SETPOINT_DOWN);
            // intake.catchFuel(COLLECT_FUEL_SPEED);
          break;

        case CLOSE_INTAKE:
            // intake.catchFuel(STOPPED_FUEL_SPEED);
            intake.enablePivot(SETPOINT_UP);
          break;

        case LOCK_CONVEYOR:
          break;

        case LOCK_HOOD:
          break;

        case CLIMBER_L1:
          break;

        case LOCK_TURRET:
            turret.setGoal(TurretGoal.OFF);
            index.stopIndex();
          break;

        default:
          break;
      }
  }

  public Actions setAction(Actions actions) {
      if(currentAction != actions){
        currentAction = actions;
      }
      switch (actions) {
        case SHOOT_FUEL:
            currentAction = Actions.SHOOT_FUEL;
          break;

        case CATCH_FUEL:
            currentAction = Actions.CATCH_FUEL;
          break;

        case CLOSE_INTAKE:
            currentAction = Actions.CLOSE_INTAKE;
          break;

        case LOCK_CONVEYOR:
            currentAction = Actions.LOCK_CONVEYOR;
          break;

        case LOCK_HOOD:
            currentAction = Actions.LOCK_HOOD;
          break;

        case CLIMBER_L1:
            currentAction = Actions.CLIMBER_L1;
          break;

        case LOCK_TURRET:
            currentAction = Actions.LOCK_TURRET;
          break;

        default:
          break;
      }
      return currentAction;
  }

  public Command setActionSim(Actions action) {
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

  public Actions getAction() {
    return currentAction;
  }
}
