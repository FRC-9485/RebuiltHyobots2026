package frc.frc_java9485.motors.spark;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.frc_java9485.motors.io.SparkMotorIO;

public class SparkFlexMotor implements SparkMotorIO {

  private SparkFlex motor;
  private SparkFlexConfig config;
  private int id;
  private boolean inverted;
  private String name;

  private double speed = 0;
  private double porcentage = 0;
  private double position = 0;

  private IdleMode currentIdleMode;

  public SparkFlexMotor(int id, String name) {
    this.id = id;
    this.name = name;
    this.motor = new SparkFlex(id, SparkFlex.MotorType.kBrushless);
    this.config = new SparkFlexConfig();

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

  public SparkFlex getSpark() {
    return motor;
  }

  @Override
  public RelativeEncoder getEncoder() {
    return motor.getEncoder();
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

  private void configureSparkFLEX(Supplier<REVLibError> config) {
    for (int i = 0; i < maximumRetries; i++) {
      if (config.get() == REVLibError.kOk) {
        return;
      }
      Timer.delay(Milliseconds.of(5).in(Seconds));
    }
    DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
  }

  @Override
  public void cleanStickFaults() {
    configureSparkFLEX(motor::clearFaults);
  }

  @Override
  public void burnFlash() {
    if (!DriverStation.isDisabled()) {
      throw new RuntimeException("Config updates cannot be applied while the robot is Enabled!");
    }
    configureSparkFLEX(() -> {
      return motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    });
  }
}
