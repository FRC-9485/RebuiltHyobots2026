package frc.robot.subsystems.mechanism.conveyor;

import static frc.frc_java9485.constants.mechanisms.ConveyorConsts.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.frc_java9485.sensor.DigitalSensor;

public class ConveyorSubsystem extends SubsystemBase implements ConveyorIO{
    private static ConveyorSubsystem m_instance;

    private final VictorSPX conveyor;

    private final DigitalSensor homeSensor;
    private final DigitalSensor limitSensor;

    private final ConveyorInputsAutoLogged conveyorInputs;
    private final ConveyorInputs inputs;

    public static ConveyorSubsystem getInstance() {
        if (m_instance == null) m_instance = new ConveyorSubsystem();
        return m_instance;
    }

    private ConveyorSubsystem() {
        conveyor = new VictorSPX(CONVEYOR_MOTOR_ID);

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
        if(Math.abs(speed) > MAX_SPEED){
            speed = MAX_SPEED;
        }

        conveyor.set(VictorSPXControlMode.Velocity, speed);
    }

    @Override
    public void stopConveyor() {
        conveyor.set(VictorSPXControlMode.Velocity, 0);
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
    public void updateInputs(ConveyorInputs inputs) {
        inputs.atHome = conveyorIsInHome();
        inputs.atLimit = conveyorInLimit();
    }
}
