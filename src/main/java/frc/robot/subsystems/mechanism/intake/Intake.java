package frc.robot.subsystems.mechanism.intake;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.mechanisms.IntakeConsts;
import frc.frc_java9485.motors.spark.SparkFlexMotor;
import frc.frc_java9485.motors.spark.SparkMaxMotor;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;

public class Intake extends SubsystemBase implements IntakeIO {
  private static Intake m_instance;

  private final SparkMaxMotor pivot;
  private final SparkFlexMotor catchBall;

  private final Encoder pivotEncoder;

  private final TunableProfiledController controller;

  private double pivotAngle = 0;
  private double catchFuelSpeed = 0;
  private double pivotSetpoint = 0;
  private boolean isCollecting = false;
  private Voltage pivotVolts = Volts.of(0);

  private final IntakeInputsAutoLogged inputs;

  public static Intake getInstance() {
    if (m_instance == null) m_instance = new Intake();
    return m_instance;
  }

  private Intake() {
    pivot = new SparkMaxMotor(IntakeConsts.PIVOT_ID, "Pivot");
    catchBall = new SparkFlexMotor(IntakeConsts.CATCH_BALL_ID, "Catch Fuel");

    controller = new TunableProfiledController(IntakeConsts.PIVOT_CONSTANTS);

    pivotEncoder = new Encoder(IntakeConsts.ENCODER_A_CHANNEL,
                               IntakeConsts.ENCODER_B_CHANNEL);
    pivotEncoder.setDistancePerPulse(IntakeConsts.ENCODER_DISTANCE_PER_PULSE);
    pivotEncoder.setReverseDirection(IntakeConsts.ENCODER_INVERTED);
    pivotEncoder.reset();

    inputs = new IntakeInputsAutoLogged();
  }

  @Override
  public void periodic() {
    updateInputs(inputs);
    Logger.processInputs("Mechanism/Intake", inputs);
  }

  @Override
  public void catchFuel(double speed) {
    this.catchFuelSpeed = speed;
    catchBall.setSpeed(speed);
  }

  @Override
  public void enablePivot(double setpoint) {
    this.pivotSetpoint = setpoint;

    pivotAngle = pivotEncoder.getDistance();

    controller.setGoal(setpoint);

    Voltage ff = Volts.of(controller.calculate(pivotAngle));

    pivot.setVoltage(ff);
    // this.setpoint = setpoint;

    // double clampedngle =
    //   MathUtil.clamp(setpoint, IntakeConsts.SETPOINT_DOWN, IntakeConsts.SETPOINT_UP);
    // controller.setGoal(clampedngle);

    // pivotAngle = pivotEncoder.getDistance();

    // Voltage ff = Volts.of(controller.calculateFeedforward());
    // volts = Volts.of(controller.calculate(pivotAngle));

    // Voltage allVoltage;
    // if(!controller.atGoal()){
    //   allVoltage = volts.plus(ff);
    // } else{
    //   allVoltage = ff;
    // }

    // double voltsInDouble = allVoltage.in(Volts);
    // voltsInDouble = MathUtil.clamp(voltsInDouble, -12.0, 12.0);

    // pivot.setVoltage(voltsInDouble);
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
    inputs.pivotAngle = pivotEncoder.getDistance();
    inputs.pivotSetpoint = pivotSetpoint;
  }
}
