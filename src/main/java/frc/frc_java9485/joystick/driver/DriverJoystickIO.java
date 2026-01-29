package frc.frc_java9485.joystick.driver;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverJoystickIO {
  double getLeftY();

  double getLeftX();

  double getRightX();

  double getRightY();

  boolean isTurboMode();

  boolean isLowMode();

  double getPerfomanceByAlliance(double speed);

  Trigger a();

  Trigger b();

  Trigger y();

  Trigger x();

  Trigger rightBumper();

  Trigger leftBumper();

  Trigger getLeftBack();
}
