package frc.robot.subsystems.mechanism.index;

import static edu.wpi.first.units.Units.Volts;
import static frc.frc_java9485.constants.mechanisms.IndexConsts.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.motors.rev.SparkMaxMotor;
import frc.frc_java9485.motors.rev.io.SparkInputsAutoLogged;

public class IndexSubsystem extends SubsystemBase implements IndexIO{

    private final SparkMaxMotor index;
    private boolean isActive;
    private final IndexInputsAutoLogged inputs;

    private final SparkInputsAutoLogged motorInputs;

    private static IndexSubsystem mInstance = null;

    public static IndexSubsystem getInstance(){
        if (mInstance == null) {
            mInstance = new IndexSubsystem();
        }
        return mInstance;
    }

    private IndexSubsystem(){
        this.index = new SparkMaxMotor(INDEX_ID, "index motor");
        inputs = new IndexInputsAutoLogged();
        motorInputs = new SparkInputsAutoLogged();

        isActive = false;

        configureIndexMotor();
    }

    @Override
    public void updateInputs(IndexInputs indexInputs) {
        indexInputs.current = index.getCurrent();
        indexInputs.indexSpeed = index.getRPM();
        indexInputs.isCollecting = isCollecting();
        indexInputs.voltage = Volts.of(index.getVoltage());
    }

    private void configureIndexMotor(){
        index.setCurrentLimit(INDEX_CURRENT_LIMIT);
        index.burnFlash();
    }

    @Override
    public void periodic() {
        if(isActive){
            index.setSpeed(-MAX_SPEED);
        }

        updateInputs(inputs);
        Logger.processInputs("Mechanism/index inputs", inputs);

        index.updateInputs(motorInputs);
    }

    @Override
    public void turnOn(){
        isActive = true;
    }

    @Override
    public Command turnOnCommand(DoubleSupplier speed){
        return run(() -> {
            index.setSpeed(-speed.getAsDouble() * 0.6);
        });
    }

    @Override
    public boolean isCollecting(){
        return Math.abs(index.getRPM()) > 0.01;
    }

    @Override
    public void stopIndex(){
        index.setSpeed(0);
        isActive = false;
    }
}
