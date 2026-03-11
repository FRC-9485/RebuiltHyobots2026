package frc.frc_java9485.motors.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.frc_java9485.utils.TunableControls.ControlConstants;

public interface SparkIO {

    @AutoLog
    public static class SparkInputs{
        public int id = 0;
        public Voltage currentVoltage = Volts.of(0);
        public Angle currentPosition = Rotations.of(0);
        public Current currentAmps = Amps.of(0);
        public AngularVelocity currentRPM = RPM.of(0);
        public Temperature currentTemperature = Celsius.of(0);
    }

    final int maximumRetries = 5;

    public void setSpeed(double speeds);
    public void setPorcentage(double porcentage);
    public void setSetpoint(double setpoint);
    public void setRampRate(double ramp);
    public void setIdleMode(IdleMode idleMode);
    public void followMotor(int id);
    public void setVoltage(double voltage);
    public void setVoltage(Voltage voltage);
    public void setInvert(boolean invert);
    public void cleanStickFaults();
    public void setCurrentLimit(int current);
    public void resetPositionByEncoder(double posisition);

    public double getPosition();
    public double getRate();
    public double getVoltage();
    public double getTemperature();
    public double getCurrent();

    public IdleMode getIdleMode();
    public RelativeEncoder getEncoder();

    public void updateInputs(SparkInputsAutoLogged inputs);

    public void setForwardSoftLimit(double limit);
    public void setReverseSoftLimit(double limit);
    public void enableForwardSoftLimit(boolean enable);
    public void enableReverseSoftLimit(boolean enable);

    public void setVelocityConversionFactor(double factor);
    public void setPositionConversionFactor(double factor);

    public void setClosedLoopPID(double kP, double kI, double kD);
    public void setClosedLoopFeedbackSensor(FeedbackSensor sensor);
    public void setClosedLoopFeedForward(double kA, double kV);
    public void setClosedLoopPhysical(double kS, double kG);

    public void setClosedLoopPID(PIDController pid);
    public void setClosedLoopControlConstants(ControlConstants constants);

    public SparkClosedLoopController getClosedLoopController();

    public void resetConfigToDefault(Motor motor);
    public void burnFlash();

    public enum Motor {
        NEO_1,
        NEO_2,
        NEO_550,
        NEO_VORTEX
    }
}
