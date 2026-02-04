package frc.robot.subsystems.mechanism.shooter.turret;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.frc_java9485.constants.mechanisms.TurretConsts.*;
import frc.frc_java9485.motors.spark.SparkMaxMotor;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;

public class Turret extends SubsystemBase implements TurretIO {
  private static Turret m_instance;

  private final SparkMaxMotor turn;
  private final SparkMaxMotor backSpin;
  private final SparkMaxMotor shooter;

  private final RelativeEncoder turnEncoder;
  private final RelativeEncoder controlEncoder;

  private final TunableProfiledController turnController;
  private final TunableProfiledController backspinnController;

  private TurretInputsAutoLogged inputs;

  public double speed = 0;

  public static Turret getInstance() {
    if (m_instance == null) m_instance = new Turret();
    return m_instance;
  }

  private Turret() {
    turn = new SparkMaxMotor(TURN_ID, "Turn Turret");
    backSpin = new SparkMaxMotor(CONTROL_ID, "Backspin");
    shooter = new SparkMaxMotor(SHOOTER_ID, "Shooter");

    turnEncoder = turn.getEncoder();
    controlEncoder = backSpin.getEncoder();

    turnController = new TunableProfiledController(TUNNABLE_TURN_CONSTANTS);
    backspinnController = new TunableProfiledController(TUNNABLE_BACKSPIN_CONSTANTS);

    inputs = new TurretInputsAutoLogged();
  }

  @Override
  public void periodic() {
    updateInputs(inputs);
    Logger.processInputs("Mechanisms/Turret", inputs);
  }

  @Override
  public void setAngulation(double angle) {
    // fazer logica
  }

  @Override
  public double getAnglulation() {
    return turnEncoder.getPosition();
  }

  @Override
  public void shootFuel(double speed) {
    // fazer logica
    this.speed = speed;
  }

  @Override
  public double getControlPosition() {
    return controlEncoder.getVelocity();
  }

  @Override
  public double getSpeed() {
    return speed;
  }

  @Override
  public boolean turnInSetpoint() {
    return turnController.atGoal();
  }

  @Override
  public void updateInputs(TurretInputs inputs) {
      inputs.angulation = getAnglulation();
      inputs.controlPosition = getControlPosition();
      inputs.inSetpoint = turnInSetpoint();
      inputs.speed = getSpeed();
  }
}
