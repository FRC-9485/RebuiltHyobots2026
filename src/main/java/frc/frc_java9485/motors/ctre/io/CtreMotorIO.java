package frc.frc_java9485.motors.ctre.io;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.frc_java9485.utils.TunableControls.ControlConstants;

public interface CtreMotorIO {
    @AutoLog
    public static class CtreMotorInputs {
        public int id = 0;
        public double speed = 0.0;
        public boolean inverted = false;
        public Voltage currentVotlage = Volts.of(0);
        public Angle currentPosition = Rotations.of(0);
        public Current currentAmps = Amps.of(0);
        public AngularVelocity currentRPM = RPM.of(0);
        public Temperature currentTemperature = Celsius.of(0);
    }

    public void updateInputs(CtreMotorInputsAutoLogged inputs);

    public void stopMotor();
    public void setSpeed(double speed);
    public void setVelocity(double vel);
    public void setPosition(double pos);
    public void setFollow(int otherId);

    public void setInverted(boolean inverted);
    public void cleanStickFaults();

    public void setPID(PIDController pid);
    public void setPID(double kP, double kI, double kD);
    public void setControlConstats(ControlConstants constants);
    public void setPIDF(double kP, double kI, double kD, double kF);
    public void setPIDF(double kP, double kI, double kD, double kF, double iZone);

    public void burnFlash();

    public double getTemperature();
}
