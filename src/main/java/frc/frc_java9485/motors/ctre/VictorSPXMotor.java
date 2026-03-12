package frc.frc_java9485.motors.ctre;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import frc.frc_java9485.motors.ctre.io.CtreMotorIO;
import frc.frc_java9485.motors.ctre.io.CtreMotorInputsAutoLogged;
import frc.frc_java9485.utils.TunableControls.ControlConstants;

import static frc.frc_java9485.constants.LoggerConstants.*;

public class VictorSPXMotor implements CtreMotorIO{
    private final VictorSPX motor;
    private final SlotConfiguration slotConfig;
    private final VictorSPXConfiguration config;

    private double pos = 0;
    private double vel = 0;
    private double speed = 0;
    private boolean inverted = false;

    private final String name;

    public VictorSPXMotor(int id, String name) {
        motor = new VictorSPX(id);
        this.name = name;

        config = new VictorSPXConfiguration();
        slotConfig = config.slot0;

        motor.clearStickyFaults();
    }

    @Override
    public void setSpeed(double speed) {
        if (this.speed != speed) {
            motor.set(VictorSPXControlMode.PercentOutput, speed);
            this.speed = speed;
        }
    }

    @Override
    public void stopMotor() {
        motor.set(VictorSPXControlMode.Disabled, 0);
    }

    @Override
    public void setPosition(double pos) {
        if (this.pos != pos) {
            motor.set(VictorSPXControlMode.Position, pos);
            this.pos = pos;
        }
    }

    @Override
    public void setVelocity(double vel) {
        if (this.vel != vel) {
            motor.set(VictorSPXControlMode.Velocity, vel);
            this.vel = vel;
        }
    }

    @Override
    public void setFollow(int otherId) {
        motor.set(VictorSPXControlMode.Follower, otherId);
    }

    @Override
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
        this.inverted = inverted;
    }

    @Override
    public void cleanStickFaults() {
        motor.clearStickyFaults();
    }

    @Override
    public void setPIDF(double kP, double kI, double kD, double kF, double iZone) {
        slotConfig.kP = kP;
        slotConfig.kD = kD;
        slotConfig.kI = kI;
        slotConfig.kF = kF;
        slotConfig.integralZone = iZone;
    }

    @Override
    public void setPIDF(double kP, double kI, double kD, double kF) {
        setPIDF(kP, kI, kD, kF, Double.POSITIVE_INFINITY);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        setPIDF(kP, kI, kD, 0, Double.POSITIVE_INFINITY);
    }

    @Override
    public void setPID(PIDController pid) {
        setPIDF(pid.getP(), pid.getI(), pid.getD(), 0, pid.getIZone());
    }

    @Override
    public void setControlConstats(ControlConstants constants) {
        PIDController pid = constants.getPIDController();
        setPIDF(pid.getP(), pid.getI(), pid.getD(), 0, pid.getIZone());
    }

    @Override
    public void burnFlash() {
        motor.configAllSettings(config);
    }

    @Override
    public double getTemperature() {
        return motor.getTemperature();
    }

    @Override
    public void updateInputs(CtreMotorInputsAutoLogged inputs) {
        inputs.id = motor.getBaseID();
        inputs.speed = this.speed;
        inputs.inverted = this.inverted;
        inputs.currentTemperature = Celsius.of(getTemperature());
        inputs.currentVotlage = Volts.of(motor.getMotorOutputVoltage());

        Logger.processInputs(CTRE_VICTOR_SPX_KEY + name, inputs);
    }
}
