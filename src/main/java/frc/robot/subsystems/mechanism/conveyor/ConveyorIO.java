package frc.robot.subsystems.mechanism.conveyor;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

public interface ConveyorIO {

    @AutoLog
    public static class ConveyorInputs{
       public double speed = 0;
       public boolean atHome = false;
       public boolean atLimit = false;
       public Voltage voltage = Volts.of(0);
       public boolean isLocked = false;
    }

    public void runConveyor(double speed);
    public void stopConveyor();

    public boolean conveyorIsInHome();
    public boolean conveyorInLimit();

    public Command setConveyorIdleMode(IdleMode idleMode);
    public void updateInputs(ConveyorInputs inputs);
}
