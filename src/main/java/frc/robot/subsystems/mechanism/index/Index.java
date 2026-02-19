package frc.robot.subsystems.mechanism.index;

import static edu.wpi.first.units.Units.Volts;
import static frc.frc_java9485.constants.mechanisms.IndexConsts.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.motors.spark.SparkMaxMotor;

public class Index extends SubsystemBase implements IndexIO{

    private final SparkMaxMotor index;
    private boolean isActive;
    private final IndexInputsAutoLogged inputs;

    private static Index mInstance = null;

    public static Index getInstance(){
        if (mInstance == null) {
            mInstance = new Index();
        }
        return mInstance;
    }

    private Index(){
        this.index = new SparkMaxMotor(INDEX_ID, "index motor");
        inputs = new IndexInputsAutoLogged();
        
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
            index.setSpeed(MAX_SPEED);
        }

        updateInputs(inputs);
        Logger.processInputs("index inputs", inputs);
    }

    @Override
    public Command turnOn(){
        return runOnce(() ->{
            isActive = true;
        });
    }

    @Override
    public boolean isCollecting(){
        return Math.abs(index.getRate()) > 0.01;
    }

    @Override
    public Command stopIndex(){
        return runOnce(() ->{
            isActive = false;
        });
    }
}
