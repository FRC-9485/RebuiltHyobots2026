package frc.robot.subsystems.mechanism.conveyor;

import static frc.frc_java9485.constants.mechanisms.ConveyorConsts.CONVEYOR_MOTOR_ID;
import static frc.frc_java9485.constants.mechanisms.ConveyorConsts.HOME_SENSOR_ID;
import static frc.frc_java9485.constants.mechanisms.ConveyorConsts.INVERT_HOME;
import static frc.frc_java9485.constants.mechanisms.ConveyorConsts.INVERT_LIMIT;
import static frc.frc_java9485.constants.mechanisms.ConveyorConsts.LIMIT_SENSOR_ID;
import static frc.frc_java9485.constants.mechanisms.ConveyorConsts.MAX_SPEED;

import frc.frc_java9485.sensor.DigitalSensor;
import frc.frc_java9485.sensor.SensorInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConveyorSubsystem extends SubsystemBase implements ConveyorIO{
    private static ConveyorSubsystem m_instance;

    private final VictorSPX conveyor;

    private final DigitalSensor homeSensor;
    private final DigitalSensor limitSensor;

    private final SensorInputsAutoLogged homeAutoLogged;
    private final SensorInputsAutoLogged limitAutoLogged;

    private final ConveyorInputsAutoLogged conveyorInputs;

    public static ConveyorSubsystem getInstance() {
        if (m_instance == null) m_instance = new ConveyorSubsystem();
        return m_instance;
    }

    private ConveyorSubsystem() {
        conveyor = new VictorSPX(CONVEYOR_MOTOR_ID);

        homeSensor = new DigitalSensor(HOME_SENSOR_ID, INVERT_HOME, "home sensor");
        limitSensor = new DigitalSensor(LIMIT_SENSOR_ID, INVERT_LIMIT, "limit sensor");

        homeAutoLogged = new SensorInputsAutoLogged();
        limitAutoLogged = new SensorInputsAutoLogged();

        conveyorInputs = new ConveyorInputsAutoLogged();
    }

    @Override
    public void periodic() {
        updateInputs(conveyorInputs);
        Logger.processInputs("Mechanism/coveyor inputs", conveyorInputs);

        homeSensor.processInput(homeAutoLogged);
        limitSensor.processInput(limitAutoLogged);
    }

    @Override
    public void runConveyor(double speed) {
        speed = MathUtil.clamp(speed, -MAX_SPEED, MAX_SPEED);

        // Bloqueia movimento para trás se já está no home
        if (speed < 0 && conveyorIsInHome()) {
            stopConveyor();
            return;
        }

        // Bloqueia movimento para frente se já está no limit
        if (speed > 0 && conveyorInLimit()) {
            stopConveyor();
            return;
        }

        conveyor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    @Override
    public void stopConveyor() {
        conveyor.set(VictorSPXControlMode.PercentOutput, 0);
    }

    @Override
    public boolean conveyorIsInHome() {
        return homeSensor.isDetected();
    }

    @Override
    public boolean conveyorInLimit() {
        return limitSensor.isDetected();
    }

    @Override
    public void updateInputs(ConveyorInputs inputs) {
        inputs.atHome = conveyorIsInHome();
        inputs.atLimit = conveyorInLimit();
    }
}
