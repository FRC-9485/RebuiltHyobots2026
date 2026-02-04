package frc.robot.subsystems.mechanism.conveyor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.frc_java9485.constants.mechanisms.ConveyorConsts.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.frc_java9485.motors.spark.SparkMaxMotor;
import frc.frc_java9485.sensor.DigitalSensor;

public class Conveyor extends SubsystemBase implements ConveyorIO{
    private static Conveyor m_instance;

    private final SparkMaxMotor conveyor;

    private final DigitalSensor homeSensor;
    private final DigitalSensor limitSensor;

    public static Conveyor getInstance() {
        if (m_instance == null) m_instance = new Conveyor();
        return m_instance;
    }

    private Conveyor() {
        conveyor = new SparkMaxMotor(MOTOR_ID, "Conveyor");

        homeSensor = new DigitalSensor(HOME_SENSOR_ID, INVERT_HOME);
        limitSensor = new DigitalSensor(LIMIT_SENSOR_ID, INVERT_LIMIT);
    }

    @Override
    public void runConveyor(double speed) {
        conveyor.setSpeed(speed >= MAX_SPEED ? MAX_SPEED : speed);
    }

    @Override
    public void stopConveyor() {
        conveyor.setSpeed(0);
    }

    @Override
    public boolean conveyorIsInHome() {
        return homeSensor.isDetected();
    }

    @Override
    public boolean conveyorInLimit() {
        return limitSensor.isDetected();
    }

    @Override
    public void setConveyorIdleMode(IdleMode idleMode) {
        conveyor.setIdleMode(idleMode);
    }
}
