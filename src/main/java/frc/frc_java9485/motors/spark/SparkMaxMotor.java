package frc.frc_java9485.motors.spark;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.frc_java9485.motors.io.MotorIO;

public class SparkMaxMotor implements MotorIO{

  private SparkMax motor;
  private SparkMaxConfig config;
  private int id;
  private boolean isFollower;
  private String name;

  private double speed = 0;
  private double porcentage = 0;
  private double position = 0;
  private IdleMode currentIdleMode;

  public SparkMaxMotor(int id, String name) {
    this(id, name, false);
  }

  public SparkMaxMotor(int id, String name, boolean isFollower) {
    this.id = id;
    this.isFollower = isFollower;
    this.name = name;
    this.motor = new SparkMax(id, SparkMax.MotorType.kBrushless);
    this.config = new SparkMaxConfig();

    cleanStickFaults();
  }

  @Override
  public void setSpeed(double speeds) {
    if (speeds != speed) {
      motor.set(speeds);
      this.speed = speeds;
    }
  }

  @Override
  public void setPorcentage(double porcentage) {
    if (porcentage != this.porcentage) {
      this.porcentage = porcentage;
      motor.set(porcentage);
    }
  }

  @Override
  public double getPosition() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public double getRate() {
    return motor.getEncoder().getVelocity();
  }

  @Override
  public void setSetpoint(double setpoint) {
    if (setpoint != getPosition()) {
      motor.getClosedLoopController().setSetpoint(setpoint, ControlType.kPosition);
    }
  }

  @Override
  public void setRampRate(double ramp) {
    config.closedLoopRampRate(ramp).openLoopRampRate(ramp);
  }

  @Override
  public double getVoltage() {
    return motor.getBusVoltage();
  }

  public SparkMax getSpark() {
    return motor;
  }

  @Override
  public RelativeEncoder getEncoder() {
    return motor.getAlternateEncoder();
  }

  @Override
  public void followMotor(int id) {
    config.follow(id);
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getTemperature() {
    return motor.getMotorTemperature();
  }

  @Override
  public void setIdleMode(IdleMode idleMode) {
    this.currentIdleMode = idleMode;
    config.idleMode(idleMode);
  }

  @Override
  public double getCurrent(){
    return motor.getOutputCurrent();
  }

  @Override
  public void setInvert() {
      config.inverted(true);
  }

  @Override
  public IdleMode getIdleMode() {
      return currentIdleMode;
  }

  @Override
  public void resetPositionByEncoder(double posisition) {
      motor.getEncoder().setPosition(posisition);
  }

  private void configureSparkMax(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
      Timer.delay(Milliseconds.of(5).in(Seconds));
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  @Override
  public void setCurrentLimit(int current) {
      config.smartCurrentLimit(current);
  }

  @Override
  public void cleanStickFaults() {
      configureSparkMax(motor::clearFaults);
  }

  @Override
  public void burnFlash() {
    if (!DriverStation.isDisabled()) {
      throw new RuntimeException("Config updates cannot be applied while the robot is Enabled!");
    }
    configureSparkMax(() -> {
      return motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    });
  }
}
