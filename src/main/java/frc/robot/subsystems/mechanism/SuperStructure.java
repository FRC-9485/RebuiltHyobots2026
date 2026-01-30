package frc.robot.subsystems.mechanism;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.RobotConsts;
import frc.frc_java9485.constants.RobotConsts.RobotModes;
import frc.frc_java9485.constants.mechanisms.IntakeConsts;
import frc.robot.subsystems.mechanism.intake.Intake;
import frc.robot.subsystems.mechanism.shooter.hood.Hood;
import frc.robot.subsystems.mechanism.shooter.turret.Turret;
import frc.robot.subsystems.swerve.Swerve;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class SuperStructure extends SubsystemBase implements SuperStructureIO {
  private Hood hood;
  private Turret turret;
  private Intake intake;

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

    if (RobotConsts.CURRENT_ROBOT_MODE == RobotModes.SIM) {
      intakeSimulation = IntakeSimulation.InTheFrameIntake("Fuel", Swerve.getInstance().getSimulation(),
                                                           Meters.of(0.348978601), IntakeSide.BACK, 40);
    }
  }

  public enum Actions {
    SHOOT_FUEL,
    CATCH_FUEL,
    CLOSE_INTAKE,
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
        intake.enablePivot(IntakeConsts.SETPOINT_DOWN);
        // intakeInputs = IntakeConsts.SETPOINT_DOWN;
        break;
        case CLOSE_INTAKE:
        // intake.catchBall(0);
        intake.enablePivot(IntakeConsts.SETPOINT_UP);
        // intakeInputs = IntakeConsts.SETPOINT_UP;
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
    return run(() -> {
      currentAction = action;
    });
  }

  @Override
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
