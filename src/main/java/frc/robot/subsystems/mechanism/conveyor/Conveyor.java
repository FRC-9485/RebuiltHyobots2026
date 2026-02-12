package frc.robot.subsystems.mechanism.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;
import static frc.frc_java9485.constants.mechanisms.ConveyorConsts.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.frc_java9485.motors.spark.SparkMaxBrushed;
import frc.frc_java9485.sensor.DigitalSensor;

public class Conveyor extends SubsystemBase implements ConveyorIO{
    private static Conveyor m_instance;

    private final SparkMaxBrushed conveyor;

    private final DigitalSensor homeSensor;
    private final DigitalSensor limitSensor;

    private final ConveyorInputsAutoLogged conveyorInputs;
    private final ConveyorInputs inputs;

    private IdleMode currentIdleMode = IdleMode.kBrake;

    public static Conveyor getInstance() {
        if (m_instance == null) m_instance = new Conveyor();
        return m_instance;
    }

    private Conveyor() {
        conveyor = new SparkMaxBrushed(MOTOR_ID, "Conveyor Motor");

        homeSensor = new DigitalSensor(HOME_SENSOR_ID, INVERT_HOME);
        limitSensor = new DigitalSensor(LIMIT_SENSOR_ID, INVERT_LIMIT);

        inputs = new ConveyorInputs();
        conveyorInputs = new ConveyorInputsAutoLogged();
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("coveyor inputs", conveyorInputs);
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
    public Command setConveyorIdleMode(IdleMode idleMode) {
        return runOnce(() ->{
            currentIdleMode = idleMode;
            conveyor.setIdleMode(idleMode);
        });
    }

    @Override
    public void updateInputs(ConveyorInputs inputs) {
        inputs.atHome = conveyorIsInHome();
        inputs.atLimit = conveyorInLimit();
        inputs.isLocked = currentIdleMode == IdleMode.kBrake;
        inputs.speed = conveyor.getRate();
        inputs.voltage = Volts.of(conveyor.getVoltage());
    }
}
