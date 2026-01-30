package frc.robot.subsystems.mechanism.hood;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.RelativeEncoder;

public interface HoodIO {

  @AutoLog
  public static class HoodInputs{
    public double position = 0;
  }

  public void setPosition(double position);

  public double getPosition();

  public void updateInputs(HoodInputs inputs);

  public RelativeEncoder getEncoder();
}
