package frc.robot.subsystems.mechanism;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.frc_java9485.constants.RobotConsts.*;
import frc.frc_java9485.constants.RobotConsts.RobotModes;
import frc.frc_java9485.constants.mechanisms.ConveyorConsts;

import static frc.frc_java9485.constants.mechanisms.IntakeConsts.*;
import frc.robot.subsystems.mechanism.conveyor.Conveyor;
import frc.robot.subsystems.mechanism.intake.Intake;
import frc.robot.subsystems.mechanism.shooter.hood.Hood;
import frc.robot.subsystems.mechanism.shooter.turret.Turret;
import frc.robot.subsystems.swerve.Swerve;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class SuperStructure extends SubsystemBase implements SuperStructureIO {
  private final Hood hood;
  private final Turret turret;
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
    hood = Hood.getInstance();
    turret = Turret.getInstance();
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

  public enum Actions {
    SHOOT_FUEL,
    CATCH_FUEL,
    CLOSE_INTAKE,
    RUN_CONVEYOR,
    STOP_CONVEYOR,
    LOCK_CONVEYOR,
    AUTOMATIC_HOOD,
    MANUAL_HOOD,
    LOCK_HOOD,
    CLIMBER_L1,
    CLIMBER_L2,
    CLIMBER_L3,
    AUTOMATIC_TURRET,
    MANUAL_TURRET,
    LOCK_TURRET,
    SECURITY
  }

  @Override
  public Command setAction(Actions actions) {
    return run(() ->{
      switch (actions) {
        case SHOOT_FUEL:
          break;

        case CATCH_FUEL:
        // intake.catchBall(0.3);
        intake.enablePivot(SETPOINT_DOWN);
        // intakeInputs = IntakeConsts.SETPOINT_DOWN;
          break;

        case CLOSE_INTAKE:
        // intake.catchBall(0);
        intake.enablePivot(SETPOINT_UP);
        // intakeInputs = IntakeConsts.SETPOINT_UP;
          break;

        case RUN_CONVEYOR:
          break;

        case STOP_CONVEYOR:
          break;

        case LOCK_CONVEYOR:
          break;

        case LOCK_HOOD:
          break;

        case CLIMBER_L1:
          break;

        case CLIMBER_L2:
          break;

        case CLIMBER_L3:
          break;

        case LOCK_TURRET:
          break;

        default:
          break;
      }
    });
  }

  @Override
  public Command setActionSim(Actions action) {
    return new Command() {
      @Override
          public void execute() {
              currentAction = action;
          }

      @Override
      public boolean isFinished() {
          return currentAction == action;
      }
    };
  }

  @Override
  public void simulationPeriodic() {
      switch (currentAction) {
        case SHOOT_FUEL:
            System.out.println("shootando");
            break;

          case CATCH_FUEL:
            intakeSimulation.startIntake();
            break;

          case CLOSE_INTAKE:
            intakeSimulation.stopIntake();
            break;

          case RUN_CONVEYOR:
            break;

          case STOP_CONVEYOR:
            break;

          case LOCK_HOOD:
            break;

          case CLIMBER_L1:
            break;

          case CLIMBER_L2:
            break;

          case CLIMBER_L3:
            break;

          case LOCK_TURRET:
            break;

          default:
            break;
    }
    Logger.recordOutput("Mechanism/Intake/Fuel Quantity", intakeSimulation.getGamePiecesAmount());
  }

  @Override
  public Actions getAction() {
    return currentAction;
  }
}
