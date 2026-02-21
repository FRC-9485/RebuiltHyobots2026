package frc.frc_java9485.joystick.mechanism;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.frc_java9485.constants.JoystickConsts;

public class MechanismJoystick implements MechanismJoystickIO {
  private final CommandXboxController joystick;
  private static MechanismJoystick mInstnace;

  public static MechanismJoystick getInstance() {
    if (mInstnace == null) {
      mInstnace = new MechanismJoystick();
    }
    return mInstnace;
  }

  private MechanismJoystick() {
    joystick = new CommandXboxController(JoystickConsts.MECHANISM_PORT);
  }

  @Override
  public Trigger a() {
    return joystick.a();
  }

  @Override
  public Trigger b() {
    return joystick.b();
  }

  @Override
  public Trigger x() {
    return joystick.x();
  }

  @Override
  public Trigger y() {
    return joystick.y();
  }

  @Override
  public Trigger rightTrigger() {
    return joystick.rightTrigger();
  }

  @Override
  public Trigger letTrigger() {
    return joystick.leftTrigger();
  }

  @Override
  public Trigger backRight() {
    return joystick.button(8);
  }

  @Override
  public Trigger backLeft() {
    return joystick.button(7);
  }
}
