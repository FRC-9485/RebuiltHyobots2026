package frc.frc_java9485.joystick.mechanism;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface MechanismJoystickIO {

  public Trigger a(); // travar

  public Trigger b();

  public Trigger x();

  public Trigger y();

  public Trigger rightTrigger(); // jogar

  public Trigger letTrigger(); // pegar

  public Trigger backRight(); // climber

  public Trigger backLeft(); // climber
}
