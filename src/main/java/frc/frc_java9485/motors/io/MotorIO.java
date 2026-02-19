package frc.frc_java9485.motors.io;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {
    final int maximumRetries = 5;

    public void setSpeed(double speeds);
    public void setPorcentage(double porcentage);
    public void setSetpoint(double setpoint);
    public void setRampRate(double ramp);
    public void setIdleMode(IdleMode idleMode);
    public void followMotor(int id);
    public void setVoltage(double voltage);
    public void setVoltage(Voltage voltage);
    public void setInvert();
    public void cleanStickFaults();
    public void burnFlash();
    public void setCurrentLimit(int current);

    public double getPosition();
    public double getRate();
    public double getVoltage();
    public double getTemperature();
    public double getCurrent();

    public RelativeEncoder getEncoder();
    public IdleMode getIdleMode();
}
