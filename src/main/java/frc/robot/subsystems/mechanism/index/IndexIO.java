package frc.robot.subsystems.mechanism.index;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

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
    public Command turnOnCommand(DoubleSupplier speed, BooleanSupplier invert);

    public boolean isCollecting();
}
