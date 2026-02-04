package frc.frc_java9485.motors.spark;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;

public interface SparkMotorIO {
    public void setSpeed(double speeds);
    public void setPorcentage(double porcentage);
    public double getPosition();
    public double getRate();
    public void setSetpoint(double setpoint);
    public void setRampRate(double ramp);
    public double getVoltage();
    public RelativeEncoder getEncoder();
    public void followMotor(int id);
    public void setVoltage(double voltage);
    public void setVoltage(Voltage voltage);
    public double getTemperature();
    public void setIdleMode(IdleMode idleMode);
}
