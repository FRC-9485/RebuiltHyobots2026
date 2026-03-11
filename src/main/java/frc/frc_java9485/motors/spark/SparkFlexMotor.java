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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import static frc.frc_java9485.constants.LoggerConstants.*;
import frc.frc_java9485.motors.io.SparkIO;
import frc.frc_java9485.motors.io.SparkInputsAutoLogged;
import frc.frc_java9485.utils.TunableControls.ControlConstants;

public class SparkFlexMotor implements SparkIO {
  private final SparkFlex motor;
  private final SparkFlexConfig config;

  private double speed = 0;
  private double porcentage = 0;

  private IdleMode currentIdleMode;

  private final String name;

  public SparkFlexMotor(int id, String name) {
    this.config = new SparkFlexConfig();
    this.motor = new SparkFlex(id, MotorType.kBrushless);

    this.name = name;

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

    Logger.processInputs(SPARK_FLEX_KEY + name, inputs);
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
    if (setpoint != getRate()) {
      motor.getClosedLoopController().setSetpoint(setpoint, ControlType.kVelocity);
    }
  }

  public void setRPM(double RPM){
    motor.set(RPM/6000);
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
  public void setInvert(boolean invert) {
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
  public void setCurrentLimit(int current) {
      config.smartCurrentLimit(current);
  }

  @Override
  public void cleanStickFaults() {
    configureSparkFLEX(motor::clearFaults);
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
  public void setClosedLoopPID(double kP, double kI, double kD) {
    config.closedLoop.pid(kP, kI, kD);
  }

  @Override
  public void setClosedLoopFeedbackSensor(FeedbackSensor feedbackSensor) {
    config.closedLoop.feedbackSensor(feedbackSensor);
  }

  @Override
  public void setClosedLoopFeedForward(double kA, double kV) {
    config.closedLoop.feedForward.kA(kA);
    config.closedLoop.feedForward.kV(kV);
  }

  @Override
    public void setClosedLoopPhysical(double kS, double kG) {
      config.closedLoop.feedForward.kS(kS);
      config.closedLoop.feedForward.kG(kG);
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
        throw new RuntimeException("Cannot configure a Spark Flex to NEO 1.0 default config");
      case NEO_2:
        throw new RuntimeException("Cannot configure a Spark Flex to NEO 2.0 default config");
      case NEO_550:
        throw new RuntimeException("Cannot configure a Spark Flex to NEO 500 default config");
      case NEO_VORTEX:
        config.apply(SparkFlexConfig.Presets.REV_Vortex);
        break;
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
    configureSparkFLEX(() -> {
      return motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    });
  }

  @Override
  public void resetPositionByEncoder(double posisition) {
      motor.getEncoder().setPosition(posisition);
  }
}
