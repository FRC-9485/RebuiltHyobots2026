package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static frc.frc_java9485.constants.RobotConsts.*;
import static frc.frc_java9485.constants.mechanisms.IntakeConsts.*;

import org.littletonrobotics.junction.Logger;

import frc.frc_java9485.constants.mechanisms.ConveyorConsts;
import frc.robot.subsystems.mechanism.conveyor.Conveyor;
import frc.robot.subsystems.mechanism.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class SuperStructure extends SubsystemBase{
  private final Intake intake;
  private final Conveyor conveyor;

  private Actions currentAction = Actions.SECURITY;
  private static SuperStructure m_instance;

  private IntakeSimulation intakeSimulation;

  public static SuperStructure getInstance() {
    if (m_instance == null) m_instance = new SuperStructure();
    return m_instance;
  }

  private SuperStructure() {
    intake = Intake.getInstance();
    conveyor = Conveyor.getInstance();

    if (CURRENT_ROBOT_MODE == RobotModes.SIM) {
      intakeSimulation = IntakeSimulation.InTheFrameIntake("Fuel",
                                                           Swerve.getInstance().getSimulation(),
                                                           Meters.of(0.32779),
                                                           IntakeSide.BACK,
                                                           ConveyorConsts.MAX_FUELS);
    }
  }

  @Override
  public void periodic() {
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

  public Command setAction(Actions actions) {
    return run(() ->{
      switch (actions) {
        case SHOOT_FUEL:
          break;

        case CATCH_FUEL:
            intake.enablePivot(SETPOINT_DOWN);
            intake.catchFuel(COLLECT_FUEL_SPEED);
          break;

        case CLOSE_INTAKE:
          intake.enablePivot(SETPOINT_UP);
          break;

        case LOCK_CONVEYOR:
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
    });
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
