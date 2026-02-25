package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.frc_java9485.constants.mechanisms.LedConsts.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase implements LedIO {
  private static LedSubsystem mInstance = null;
  private LEDPattern pattern;

  private final AddressableLEDBuffer buffer;
  private final AddressableLED addressableLED;

  private LedSubsystem() {
    this.addressableLED = new AddressableLED(LED_ID);
    this.buffer = new AddressableLEDBuffer(LED_BUFFER);
    this.pattern = LEDPattern.solid(Color.kRed);

    this.addressableLED.setLength(this.buffer.getLength());

    this.pattern.applyTo(buffer);
    this.addressableLED.setData(buffer);
    this.addressableLED.start();
  }

  public static LedSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new LedSubsystem();
    }
    return mInstance;
  }

  @Override
  public void periodic() {
    if (this.pattern != null) {
      pattern.applyTo(buffer);
      addressableLED.setData(buffer);
    }
  }

  @Override
  public LEDPattern getActualPattern() {
    return pattern;
  }

  @Override
  public Command runPattern() {
    return run(
        () -> {
          if (this.pattern != null) {
            this.pattern.applyTo(buffer);
            this.addressableLED.setData(buffer);
          }
        });
  }

  @Override
  public void setPattern(LEDPattern pattern) {
    this.pattern = pattern;
  }

  @Override
  public void setSolidColor(Color color) {
    setPattern(LEDPattern.solid(color));
  }

  @Override
  public void setRGBColor(int index, int r, int g, int b) {
    for (int i = 0; i <= index; i++) {
      this.buffer.setRGB(index, r, g, b);
      this.addressableLED.setData(buffer);
    }
  }

  @Override
  public void setRainbow() {
    Distance ledSpace = Meters.of(1.0 / 60.0);
    LEDPattern rainbowPattern = LEDPattern.rainbow(255, 128);
    LEDPattern scrollingRainbowPattern =
        rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpace);

    scrollingRainbowPattern.applyTo(buffer);
    addressableLED.setData(buffer);
  }
}
