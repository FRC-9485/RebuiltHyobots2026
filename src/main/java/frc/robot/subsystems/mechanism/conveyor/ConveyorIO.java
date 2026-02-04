package frc.robot.subsystems.mechanism.conveyor;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public interface ConveyorIO {
    public void runConveyor(double speed);
    public void stopConveyor();

    public boolean conveyorIsInHome();
    public boolean conveyorInLimit();

    public void setConveyorIdleMode(IdleMode idleMode);

}
