package frc.robot.subsystems.mechanism.index;

import static edu.wpi.first.units.Units.Volts;
import static frc.frc_java9485.constants.mechanisms.IndexConsts.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.motors.spark.SparkMaxMotor;

public class IndexSubsystem extends SubsystemBase implements IndexIO{

    private final SparkMaxMotor index;
    private boolean isActive;
    private final IndexInputsAutoLogged inputs;

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

        isActive = false;

        configureIndexMotor();
    }

    @Override
    public void updateInputs(IndexInputs indexInputs) {
        indexInputs.current = index.getCurrent();
        indexInputs.indexSpeed = index.getRate();
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
        Logger.processInputs("index inputs", inputs);
    }

    @Override
    public void turnOn(){
        isActive = true;
    }

    @Override
    public boolean isCollecting(){
        return Math.abs(index.getRate()) > 0.01;
    }

    @Override
    public void stopIndex(){
        index.setSpeed(0);
        index.setVoltage(0);
        isActive = false;
    }
}
