package frc.robot.subsystems.mechanism.turret;

import org.littletonrobotics.junction.AutoLog;


public interface TurretIO {

  @AutoLog
  public static class TurretInputs{
    public double angulation = 0;
    public boolean inSetpoint = false;
    public double speed = 0;
    public double controlPosition = 0;
  }

  public void setAngulation(double angle);

  public double getAnglulation();

  public void shootFuel(double speed);

  public double getControlPosition();

  public double getSpeed();

  public boolean inAtSetpoint();

  public void updateInputs(TurretInputs inputs);
}
