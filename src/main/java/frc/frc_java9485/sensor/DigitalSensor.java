package frc.frc_java9485.sensor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import static frc.frc_java9485.constants.LoggerConstants.*;

public class DigitalSensor implements SensorIO{

    private final DigitalInput sensor;
    private final boolean inverted;
    private final String identification;

    private final SensorInputsAutoLogged sensorInputsAutoLogged;

    public DigitalSensor(int input, boolean inverted, String identification){
        sensor = new DigitalInput(input);
        this.inverted = inverted;
        this.identification = identification;
        this.sensorInputsAutoLogged = new SensorInputsAutoLogged();
    }

    public DigitalSensor(int input, String identification){
        this(input, false, identification);
    }

    @Override
    public void processInput(SensorInputsAutoLogged sensorInputs) {
        sensorInputsAutoLogged.detected = isDetected();
        sensorInputsAutoLogged.inverted = inverted;     
        
        Logger.processInputs(DIGITAL_SENSOR_KEY + identification, sensorInputs);
    }

    @Override
    public boolean isDetected() {
        boolean isDetected = isInverted() ? !sensor.get() : sensor.get();

        return isDetected;
    }

    @Override
    public boolean isInverted() {
        return inverted;
    }

    @Override
    public void updateInputs(SensorInputs inputs) {
        inputs.detected = isDetected();
        inputs.inverted = isInverted();
    }
}
