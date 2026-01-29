package frc.frc_java9485.sensor;

public interface SensorIO {

    public static final class SensorInputs{
        public boolean detected = false;
        public boolean inverted = false;
    }

    public boolean isDetected();

    public boolean isInverted();

    public void updateInputs(SensorInputs inputs);
}
