package frc.robot.subsystems.mechanism;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.mechanisms.IntakeConsts;
import frc.robot.subsystems.mechanism.hood.Hood;
import frc.robot.subsystems.mechanism.intake.Intake;
import frc.robot.subsystems.mechanism.turret.Turret;

public class SuperStructure extends SubsystemBase implements SuperStructureIO {
  private Hood hood;
  private Turret turret;
  private Intake intake;

  private Actions currentAction = Actions.NORMAL;
  private static SuperStructure m_instance;

  public static SuperStructure getInstance() {
    if (m_instance == null) m_instance = new SuperStructure();
    return m_instance;
  }

  private SuperStructure() {
    hood = Hood.getInstance();
    turret = Turret.getInstance();
    intake = Intake.getInstance();
  }

  public enum Actions {
    SHOOT_FUEL,
    CATCH_FUEL,
    CLOSE_INTAKE,
    LOCK_HOOD,
    CLIMBER_L1,
    CLIMBER_L2,
    CLIMBER_L3,
    LOCK_TURRET,
    NORMAL
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
  public Actions getAction() {
    return currentAction;
  }
}
