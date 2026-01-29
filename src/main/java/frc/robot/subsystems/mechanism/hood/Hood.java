package frc.robot.subsystems.mechanism.hood;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.motors.spark.SparkMaxMotor;

public class Hood extends SubsystemBase implements HoodIO {
  private static Hood m_instance;

  private final SparkMaxMotor motor;
  private final RelativeEncoder encoder;

  private final HoodInputsAutoLogged inputs;

  public static Hood getInstance() {
    if (m_instance == null) m_instance = new Hood();
    return m_instance;
  }

  private Hood() {
    motor = new SparkMaxMotor(0, "Hood");
    encoder = motor.getEncoder();

    inputs = new HoodInputsAutoLogged();
  }

  @Override
  public void periodic() {
      updateInputs(inputs);
      Logger.processInputs("Mechanisms/Hood", inputs);
  }

  @Override
  public void setPosition(double position) {
    // fazer logica
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public void updateInputs(HoodInputs inputs) {
      inputs.position = getPosition();
  }
}
