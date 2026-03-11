package frc.frc_java9485.sensor;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {

    @AutoLog
    public static class SensorInputs{
        public boolean detected = false;
        public boolean inverted = false;
    }

    public void processInput(SensorInputsAutoLogged sensorInputs);

    public boolean isDetected();

    public boolean isInverted();

    public void updateInputs(SensorInputs inputs);
}
