package frc.robot.subsystems.mechanism.index;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface IndexIO {

    @AutoLog
    public static class IndexInputs{
        public double indexSpeed = 0;
        public boolean isCollecting = false;
        public double current = 0;
        public Voltage voltage = Volts.of(0);
    }

    public void updateInputs(IndexInputs indexInputs);
    public void turnOn();
    public void stopIndex();

    public boolean isCollecting();
}
