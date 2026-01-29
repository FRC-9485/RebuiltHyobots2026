package frc.robot.subsystems.mechanism.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Voltage;

public interface IntakeIO {
  @AutoLog
  public class IntakeInputs{
    public double catchFuelSpeed = 0;
    public boolean isColecting = false;
    public double pivotAngle = 0;
    public double pivotSetpoint = 0;
    public Voltage pivotVolts = Volts.of(0);
  }

  public void catchFuel(double speed);

  public double getCatchFuelSpeed();

  public void enablePivot(double setpoint);

  public void updateInputs(IntakeInputs inputs);

  public boolean isColecting();

  public double getPivotVoltage();
}
