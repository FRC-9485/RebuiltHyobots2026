package frc.robot.subsystems.led;


import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public interface LedIO {

  public void setRainbow();

  public Command runPattern();

  public LEDPattern getActualPattern();

  public void setSolidColor(Color color);

  public void setPattern(LEDPattern pattern);

  public void setRGBColor(int index, int r, int g, int b);
}
