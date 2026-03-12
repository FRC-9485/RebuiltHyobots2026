package frc.robot.subsystems.mechanism.intake;

import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.frc_java9485.constants.mechanisms.IntakeConsts.*;

import frc.frc_java9485.motors.rev.SparkMaxMotor;
import frc.frc_java9485.motors.rev.io.SparkInputsAutoLogged;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;

public class IntakeSubsystem extends SubsystemBase implements IntakeIO {
  private static IntakeSubsystem m_instance;

  private final SparkMaxMotor pivot;
  private final SparkMaxMotor catchBall;

  private final DutyCycleEncoder pivotEncoder;

  private final TunableProfiledController controller;

  private double pivotSetpoint = 0;

  private final IntakeInputsAutoLogged inputs;
  private final SparkInputsAutoLogged pivotInputs;
  private final SparkInputsAutoLogged catchBallInputs;

  public static IntakeSubsystem getInstance() {
    if (m_instance == null) m_instance = new IntakeSubsystem();
    return m_instance;
  }

  private IntakeSubsystem() {
    pivot = new SparkMaxMotor(PIVOT_ID, "Pivot motor");
    catchBall = new SparkMaxMotor(CATCH_BALL_ID, "Catch Fuel motor");

    controller = new TunableProfiledController(PIVOT_CONSTANTS);

    pivotEncoder = new DutyCycleEncoder(ENCODER_CHANNEL);
    pivotEncoder.setInverted(ENCODER_INVERTED);

    inputs = new IntakeInputsAutoLogged();
    pivotInputs = new SparkInputsAutoLogged();
    catchBallInputs = new SparkInputsAutoLogged();

    configureIntakeMotor();
  }

  private void configureIntakeMotor(){
    pivot.setCurrentLimit(30);
    pivot.burnFlash();
  }

  @Override
  public void periodic() {
    updateInputs(inputs);
    Logger.processInputs("Mechanism/Intake inputs", inputs);

    pivot.updateInputs(pivotInputs);
    catchBall.updateInputs(catchBallInputs);
  }

  @Override
  public void catchFuel(double speed) {
    catchBall.setSpeed(speed);
  }

  @Override
  public void enablePivot(double setpoint) {
    this.pivotSetpoint = setpoint;
    double angle = pivotEncoder.get() * 360.0;
    controller.setGoal(setpoint);

    double output = controller.calculate(angle);

    pivot.setVoltage(-output);
  }

  @Override
  public double getCatchFuelSpeed() {
    return catchBall.getRPM();
  }

  @Override
  public boolean isColecting() {
    return getCatchFuelSpeed() > 0.1;
  }

  @Override
  public double getPivotVoltage() {
      return pivot.getVoltage();
  }

  @Override
  public boolean atSetpoint() {
      return controller.atGoal();
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.isColecting = isColecting();
    inputs.catchFuelSpeed = getCatchFuelSpeed();
    inputs.pivotVolts = Volts.of(getPivotVoltage());
    inputs.pivotAngle = pivotEncoder.get()*360.0;
    inputs.pivotSetpoint = pivotSetpoint;
  }
}
