package frc.robot.subsystems.mechanism.shooter.turret;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.frc_java9485.constants.mechanisms.TurretConsts.*;

import frc.frc_java9485.motors.spark.SparkFlexMotor;
import frc.frc_java9485.motors.spark.SparkMaxMotor;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;

public class Turret extends SubsystemBase implements TurretIO {
  private static Turret m_instance;

  private final SparkMaxMotor turnTurret;
  private final SparkFlexMotor leftShooterMotor;
  private final SparkFlexMotor rightShooterMotor;

  private final RelativeEncoder turnEncoder;

  private final TunableProfiledController turnController;

  private TurretInputsAutoLogged inputs;

  public double speed = 0;

  public static Turret getInstance() {
    if (m_instance == null) m_instance = new Turret();
    return m_instance;
  }

  private Turret() {
    turnTurret = new SparkMaxMotor(TURN_ID, "Turn Turret");
    leftShooterMotor = new SparkFlexMotor(LEFT_SHOOTER, "left shooter");
    rightShooterMotor = new SparkFlexMotor(RIGHT_SHOOTER, "right shooter");

    turnEncoder = turnTurret.getEncoder();

    turnController = new TunableProfiledController(TUNNABLE_TURN_CONSTANTS);

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
    return 0;
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
