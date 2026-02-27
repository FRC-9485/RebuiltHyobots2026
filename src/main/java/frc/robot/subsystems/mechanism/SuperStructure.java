package frc.robot.subsystems.mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meters;
import static frc.frc_java9485.constants.RobotConsts.*;
import static frc.frc_java9485.constants.mechanisms.IntakeConsts.*;

import org.littletonrobotics.junction.Logger;

import frc.frc_java9485.constants.mechanisms.ConveyorConsts;
import frc.robot.subsystems.led.LedSubsystem;
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
  private final LedSubsystem ledSubsystem;
  // private final TurretSimTeste turretSim;

  private Actions currentAction = Actions.SECURITY;

  private IntakeSimulation intakeSimulation;

  public SuperStructure(TurretSubsystem turret) {
    intake = IntakeSubsystem.getInstance();
    conveyor = ConveyorSubsystem.getInstance();
    index = IndexSubsystem.getInstance();
    ledSubsystem = LedSubsystem.getInstance();
    this.turret = turret;

    if (isSimulation()) {
      intakeSimulation = IntakeSimulation.InTheFrameIntake("Fuel",
                                                           SwerveSubsystem.getInstance().getSimulation(),
                                                           Meters.of(0.36),
                                                           IntakeSide.BACK,
                                                           ConveyorConsts.MAX_FUELS);
    }

    turret.setGoal(TurretGoal.MANUAL);
  }

  @Override
  public void periodic() {
    executeAction(getAction());
    // setLedActions(getAction());
  }

  // private void setLedActions(Actions action){
  //   switch (action) {
  //     case SHOOT_FUEL:
  //         ledSubsystem.setSolidColor(Color.kRed);
  //       break;

  //     case LOCK_TURRET:
  //         ledSubsystem.setSolidColor(Color.kGreen);
  //       break;

  //     default:
  //       break;
  //   }
  // }

  public enum Actions {
    SHOOT_FUEL,
    OPEN_INTAKE,
    CATCH_FUEL,
    CLOSE_INTAKE,
    STOP_CATCH,
    LOCK_CONVEYOR,
    AUTOMATIC_HOOD,
    MANUAL_HOOD,
    OPEN_CONVEYOR,
    CLOSE_CONVEYOR,
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
            break;

        case OPEN_INTAKE:
            intake.enablePivot(SETPOINT_DOWN);
          break;

        case CLOSE_INTAKE:
            intake.enablePivot(SETPOINT_UP);
          break;

        case STOP_CATCH:
          intake.catchFuel(STOPPED_FUEL_SPEED);
          break;

        case CATCH_FUEL:
            intake.catchFuel(COLLECT_FUEL_SPEED);
         break;

        case LOCK_CONVEYOR:
          conveyor.runConveyor(0);
          break;

        case OPEN_CONVEYOR:
            conveyor.runConveyor(0.8);
          break;

        case CLOSE_CONVEYOR:
            conveyor.runConveyor(-0.8);
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
  }

  public Actions alternActions(Actions actions) {
      if(currentAction != actions){
        currentAction = actions;
      }
      switch (actions) {
        case SHOOT_FUEL:
            currentAction = Actions.SHOOT_FUEL;
          break;

        case STOP_CATCH:
            currentAction = Actions.STOP_CATCH;
          break;

        case OPEN_INTAKE:
        if(!conveyor.conveyorInLimit()){
            currentAction = Actions.OPEN_CONVEYOR;
          } else {
            currentAction = Actions.OPEN_INTAKE;
          }
          break;

        case CATCH_FUEL:
          currentAction = Actions.CATCH_FUEL;
          break;

        case CLOSE_INTAKE:
        if(!conveyor.conveyorInLimit()){
          currentAction = Actions.OPEN_INTAKE;
          } else {
            currentAction = Actions.CLOSE_INTAKE;
          }
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

  public Command alternActionsSim(Actions action) {
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

          case OPEN_INTAKE:
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
