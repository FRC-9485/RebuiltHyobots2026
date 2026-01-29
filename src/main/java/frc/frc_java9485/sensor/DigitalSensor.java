package frc.frc_java9485.sensor;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalSensor implements SensorIO{

    private final DigitalInput sensor;
    private final boolean inverted;

    public DigitalSensor(int input, boolean inverted){
        sensor = new DigitalInput(input);
        this.inverted = inverted;
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
