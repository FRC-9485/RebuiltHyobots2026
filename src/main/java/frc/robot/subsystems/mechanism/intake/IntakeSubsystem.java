package frc.robot.subsystems.mechanism.intake;

import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.frc_java9485.constants.mechanisms.IntakeConsts.*;
import frc.frc_java9485.motors.spark.SparkMaxMotor;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;

public class IntakeSubsystem extends SubsystemBase implements IntakeIO {
  private static IntakeSubsystem m_instance;

  private final SparkMaxMotor pivot;
  private final SparkMaxMotor catchBall;

  private final DutyCycleEncoder pivotEncoder;

  private final TunableProfiledController controller;

  private double catchFuelSpeed = 0;
  private double pivotSetpoint = 0;
  private boolean isCollecting = false;
  private Voltage pivotVolts = Volts.of(0);

  private final IntakeInputsAutoLogged inputs;

  public static IntakeSubsystem getInstance() {
    if (m_instance == null) m_instance = new IntakeSubsystem();
    return m_instance;
  }

  private IntakeSubsystem() {
    pivot = new SparkMaxMotor(PIVOT_ID, "Pivot");
    catchBall = new SparkMaxMotor(CATCH_BALL_ID, "Catch Fuel");

    controller = new TunableProfiledController(PIVOT_CONSTANTS);

    pivotEncoder = new DutyCycleEncoder(ENCODER_CHANNEL);
    pivotEncoder.setInverted(ENCODER_INVERTED);

    inputs = new IntakeInputsAutoLogged();

    configureIntakeMotor();
  }

  private void configureIntakeMotor(){
    pivot.setCurrentLimit(30);
    pivot.burnFlash();
  }

  @Override
  public void periodic() {
    updateInputs(inputs);
    Logger.processInputs("Mechanism/Intake", inputs);

    // System.out.println("Angulo: " + pivotEncoder.get() * 360.0);
    // System.out.println("Setpoint: " + pivotSetpoint);
    // System.out.println("Voltagem: " + pivot.getVoltage() + "\n");
  }

  @Override
  public void catchFuel(double speed) {
    this.catchFuelSpeed = speed;
    catchBall.setSpeed(speed);
  }

  @Override
  public void enablePivot(double setpoint) {
    controller.setGoal(setpoint);
    double angle = pivotEncoder.get() * 360.0;

    double output = controller.calculate(angle);

    pivot.setVoltage(output);
  }

  @Override
  public double getCatchFuelSpeed() {
    return catchFuelSpeed;
  }

  @Override
  public boolean isColecting() {
      return isCollecting; //mudar lógica
  }

  @Override
  public double getPivotVoltage() {
      return pivotVolts.in(Volts);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.isColecting = isColecting();
    inputs.catchFuelSpeed = getCatchFuelSpeed();
    inputs.pivotVolts = pivotVolts;
    inputs.pivotAngle = pivotEncoder.get()*360.0;
    inputs.pivotSetpoint = pivotSetpoint;
  }
}
