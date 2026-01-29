package frc.robot.subsystems.mechanism.turret;

import org.littletonrobotics.junction.Logger;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.mechanisms.TurretConsts;
import frc.frc_java9485.motors.spark.SparkMaxMotor;

public class Turret extends SubsystemBase implements TurretIO {
  private static Turret m_instance;

  private final SparkMaxMotor turnMotor;
  private final SparkMaxMotor controlMotor;
  private final SparkMaxMotor shooterMotor;

  private final RelativeEncoder turnEncoder;
  private final RelativeEncoder controlEncoder;

  private final PIDController kTURN_CONTROLLER;
  private final PIDController kRPM_CONTROLLER;

  private TurretInputsAutoLogged inputs;

  public double speed = 0;

  public static Turret getInstance() {
    if (m_instance == null) m_instance = new Turret();
    return m_instance;
  }

  private Turret() {
    turnMotor = new SparkMaxMotor(TurretConsts.TURN_ID, "girar torreta");
    controlMotor = new SparkMaxMotor(TurretConsts.CONTROL_ID, "controlar shooter");
    shooterMotor = new SparkMaxMotor(TurretConsts.SHOOTER_ID, "motor shooter");

    turnEncoder = turnMotor.getEncoder();
    controlEncoder = controlMotor.getEncoder();

    kTURN_CONTROLLER = TurretConsts.TURN_PID;
    kRPM_CONTROLLER = TurretConsts.CONTROL_PID;

    kRPM_CONTROLLER.setSetpoint(TurretConsts.RPM_SETPOINT);

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
  public boolean inAtSetpoint() {
    return kTURN_CONTROLLER.atSetpoint();
  }

  @Override
  public void updateInputs(TurretInputs inputs) {
      inputs.angulation = getAnglulation();
      inputs.controlPosition = getControlPosition();
      inputs.inSetpoint = inAtSetpoint();
      inputs.speed = getSpeed();
  }
}
