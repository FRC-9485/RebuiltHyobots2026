package frc.frc_java9485.motors.spark;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.frc_java9485.motors.io.SparkIO;
import frc.frc_java9485.motors.io.SparkInputsAutoLogged;
import frc.frc_java9485.utils.TunableControls.ControlConstants;

public class SparkMaxBrushed implements SparkIO{
  private final SparkMax motor;
  private final SparkMaxConfig config;

  private double speed = 0;
  private double porcentage = 0;
  private IdleMode currentIdleMode;

  private final String name;
  private final String key;

  public SparkMaxBrushed(int id, String name) {
    this.motor = new SparkMax(id, SparkMax.MotorType.kBrushed);
    this.config = new SparkMaxConfig();

    this.name = name;
    key = "Motors/Spark Max/Brushed/";

    cleanStickFaults();
  }

  @Override
  public void updateInputs(SparkInputsAutoLogged inputs) {
    inputs.id = motor.getDeviceId();
    inputs.currentRPM = RPM.of(getRate());
    inputs.currentAmps = Amps.of(getCurrent());
    inputs.currentVoltage = Volts.of(getVoltage());
    inputs.currentPosition = Rotations.of(getPosition());
    inputs.currentTemperature = Celsius.of(getTemperature());

    Logger.processInputs(key + name, inputs);
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
  public double getCurrent() {
      return motor.getOutputCurrent();
  }

  @Override
  public void setInvert(boolean invert) {
      config.inverted(true);
  }

   @Override
  public IdleMode getIdleMode() {
      return currentIdleMode;
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
  public void cleanStickFaults() {
      configureSparkMax(motor::clearFaults);
  }

  @Override
  public void setCurrentLimit(int current) {
      config.smartCurrentLimit(current);
  }


  @Override
  public void setForwardSoftLimit(double limit) {
    config.softLimit.forwardSoftLimit(limit);
  }

  @Override
  public void setReverseSoftLimit(double limit) {
    config.softLimit.reverseSoftLimit(limit);
  }

  @Override
  public void enableForwardSoftLimit(boolean enable) {
    config.softLimit.forwardSoftLimitEnabled(enable);
  }

  @Override
  public void enableReverseSoftLimit(boolean enable) {
    config.softLimit.reverseSoftLimitEnabled(enable);
  }

  @Override
  public void setPositionConversionFactor(double factor) {
    config.encoder.positionConversionFactor(factor);
  }

  @Override
  public void setVelocityConversionFactor(double factor) {
    config.encoder.velocityConversionFactor(factor);
  }

  @Override
  public void setClosedLoopFeedbackSensor(FeedbackSensor feedbackSensor) {
    config.closedLoop.feedbackSensor(feedbackSensor);
  }

  @Override
  public void setClosedLoopPID(double kP, double kI, double kD) {
    config.closedLoop.pid(kP, kI, kD);
  }

  @Override
  public void setClosedLoopPhysical(double kS, double kG) {
    config.closedLoop.feedForward.kS(kS);
    config.closedLoop.feedForward.kG(kG);
  }

  @Override
  public void setClosedLoopFeedForward(double kA, double kV) {
    config.closedLoop.feedForward.kA(kA);
    config.closedLoop.feedForward.kV(kV);
  }

  @Override
  public void setClosedLoopControlConstants(ControlConstants constants) {
      PIDController pid = constants.getPIDController();
      ElevatorFeedforward ff = constants.getElevatorFeedforward();

      config.closedLoop.pid(pid.getP(), pid.getI(), pid.getD());
      config.closedLoop.feedForward.kS(ff.getKs());
      config.closedLoop.feedForward.kV(ff.getKv());
      config.closedLoop.feedForward.kA(ff.getKa());
      config.closedLoop.feedForward.kG(ff.getKg());
  }

  @Override
  public void setClosedLoopPID(PIDController pid) {
      config.closedLoop.pid(pid.getP(), pid.getI(), pid.getD());
  }

  @Override
  public void resetConfigToDefault(Motor motor) {
    switch (motor) {
      case NEO_1:
        config.apply(SparkMaxConfig.Presets.REV_NEO);
        break;
      case NEO_2:
        config.apply(SparkMaxConfig.Presets.REV_NEO_2);
        break;
      case NEO_550:
        config.apply(SparkMaxConfig.Presets.REV_NEO_550);
        break;
      case NEO_VORTEX:
        throw new RuntimeException("Cannot configure a Spark Max to NEO Vortex default config");
    }
  }

  @Override
  public SparkClosedLoopController getClosedLoopController() {
      return motor.getClosedLoopController();
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

  @Override
  public void resetPositionByEncoder(double posisition) {
      motor.getEncoder().setPosition(posisition);
  }
}
