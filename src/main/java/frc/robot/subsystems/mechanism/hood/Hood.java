package frc.robot.subsystems.mechanism.hood;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.mechanisms.HoodConsts;
import frc.frc_java9485.motors.spark.SparkMaxMotor;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;

public class Hood extends SubsystemBase implements HoodIO {
  private static Hood m_instance;

  private final SparkMaxMotor hood;
  private final RelativeEncoder encoder;

  private final HoodInputsAutoLogged inputs;
  private final TunableProfiledController controller;

  public static Hood getInstance() {
    if (m_instance == null) m_instance = new Hood();
    return m_instance;
  }

  private Hood() {
    hood = new SparkMaxMotor(0, "Hood");
    encoder = hood.getEncoder();

    inputs = new HoodInputsAutoLogged();
    controller = new TunableProfiledController(HoodConsts.TUNABLE_CONSTANTS);
  }

  @Override
  public void periodic() {
      updateInputs(inputs);
      Logger.processInputs("Mechanisms/Hood", inputs);
  }

  @Override
  public void setPosition(double position) {
    controller.setGoal(position);
    double output = controller.calculate(getPosition());
    
    output = MathUtil.clamp(output, -5, 5);

    hood.setVoltage(output);
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public RelativeEncoder getEncoder() {
      return encoder;
  }

  @Override
  public void updateInputs(HoodInputs inputs) {
      inputs.position = getPosition();
  }
}
