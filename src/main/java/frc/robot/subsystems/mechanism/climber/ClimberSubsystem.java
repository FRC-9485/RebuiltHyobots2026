package frc.robot.subsystems.mechanism.climber;


import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.mechanisms.ClimberConsts;
import frc.frc_java9485.motors.spark.SparkMaxMotor;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;

public class ClimberSubsystem extends SubsystemBase implements ClimberIO {
    private static ClimberSubsystem m_instance;
    private final ClimberInputs inputs;

    private final SparkMaxMotor left;
    private final SparkMaxMotor right;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final TunableProfiledController leftController;
    private final TunableProfiledController rightController;

    private double leftOutput;
    private double rightOutput;
    private double leftPosition;
    private double rightPosition;
    private ClimberStates currentState;

    public static ClimberSubsystem getInstance() {
        if (m_instance == null) m_instance = new ClimberSubsystem();
        return m_instance;
    }

    private ClimberSubsystem() {
        inputs = new ClimberInputsAutoLogged();
        currentState = ClimberStates.DEFAULT;

        left = new SparkMaxMotor(ClimberConsts.LEFT_ID, "Left Climber");
        right = new SparkMaxMotor(ClimberConsts.RIGHT_ID, "Right Climber");

        leftController = new TunableProfiledController(ClimberConsts.LEFT_CLIMBER_CONTROL_CONSTANTS);
        rightController = new TunableProfiledController(ClimberConsts.RIGHT_CLIMBER_CONTROL_CONSTANTS);

        leftEncoder = left.getEncoder();
        rightEncoder = right.getEncoder();
    }

    public enum ClimberStates {
        DEFAULT,
        L1,
        L2,
        L3
    }

    @Override
    public Command climbRobot(ClimberStates state) {
        // fazer logica
        return run(() -> {
            switch (state) {
                case L1:
                    leftController.setGoal(ClimberConsts.SETPOINT_LEFT_L1);
                    rightController.setGoal(ClimberConsts.SETPOINT_RIGHT_L1);
                    break;
                case L2:
                    leftController.setGoal(ClimberConsts.SETPOINT_LEFT_L2);
                    rightController.setGoal(ClimberConsts.SETPOINT_RIGHT_L2);
                    break;
                case L3:
                    leftController.setGoal(ClimberConsts.SETPOINT_LEFT_L3);
                    rightController.setGoal(ClimberConsts.SETPOINT_RIGHT_L3);
                    break;
                default:
                    leftController.setGoal(0.0);
                    rightController.setGoal(0.0);
                    break;
            }
        });
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        // Logger.processInputs("Mechanisms/Climber", inputs);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.currentState = currentState;
        inputs.leftPIDOut = leftOutput;
        inputs.rightPIDOut = rightOutput;
        inputs.leftPosition = leftPosition;
        inputs.rightPosition = rightPosition;
    }
}
