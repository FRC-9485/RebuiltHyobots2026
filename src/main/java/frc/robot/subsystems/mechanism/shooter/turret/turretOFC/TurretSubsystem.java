package frc.robot.subsystems.mechanism.shooter.turret.turretOFC;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Volts;

import static frc.frc_java9485.constants.FieldConsts.*;
import static frc.frc_java9485.constants.mechanisms.TurretConsts.*;
import static frc.frc_java9485.constants.mechanisms.HoodConsts.*;

import frc.frc_java9485.motors.rev.SparkFlexMotor;
import frc.frc_java9485.motors.rev.SparkMaxMotor;
import frc.frc_java9485.motors.rev.io.SparkInputsAutoLogged;
import frc.frc_java9485.utils.TunableControls.TunablePIDController;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;
import frc.robot.subsystems.vision.LimelightHelpers;

import static frc.frc_java9485.constants.RobotConsts.*;

public class TurretSubsystem extends SubsystemBase implements TurretIO{

    private final SparkFlexMotor right_motor;
    private final SparkFlexMotor left_motor;

    private final SparkMaxMotor turn_turret;
    private final SparkMaxMotor indexer;
    private final SparkMaxMotor hoodMotor;

    private final TunableProfiledController manual_turret_controller;
    private final TunableProfiledController automatic_turret_controller;
    private final TunableProfiledController hoodController;
    private final TunablePIDController flyWheelController;

    @AutoLogOutput
    private TurretGoal goal = TurretGoal.MANUAL;

    private final InterpolatingDoubleTreeMap map;

    private SparkMaxSim turnTurretSim;

    @AutoLogOutput
    private double hoodSetpoint = 0;
    private double turretSetpoint = 0;
    private double shooterSetpoint = 0;
    private boolean automatic = false;

    private final SparkInputsAutoLogged hoodInputs;
    private final SparkInputsAutoLogged turnTurretInputs;
    private final SparkInputsAutoLogged fuelToTurretInputs;

    private final SparkInputsAutoLogged leftMotorInputs;
    private final SparkInputsAutoLogged rightMotorInputs;

    private final TurretIOinputsAutoLogged turretInputs;

    @AutoLogOutput
    private Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? HUB_BLUE
                : HUB_RED;

    public TurretSubsystem(){
        this.right_motor = new SparkFlexMotor(RIGHT_SHOOTER, "right shooter");
        this.left_motor = new SparkFlexMotor(LEFT_SHOOTER, "left shooter motor");
        this.hoodMotor = new SparkMaxMotor(HOOD_MOTOR_ID, "hood motor");

        this.turn_turret = new SparkMaxMotor(TURN_TURRET, "turn turret");
        this.indexer = new SparkMaxMotor(FUEL_TO_TURRET, "catch fuel to turret");

        this.hoodMotor.resetPositionByEncoder(MIN_POSITION);
        this.turn_turret.resetPositionByEncoder(0);

        this.manual_turret_controller = new TunableProfiledController(TURRET_TUNABLE);
        this.hoodController = new TunableProfiledController(TUNABLE_CONSTANTS);
        this.flyWheelController = new TunablePIDController(SHOOTER_CONSTANTS);
        this.automatic_turret_controller = new TunableProfiledController(AUTOMATIC_TURRET_CONTROL);

        this.map = new InterpolatingDoubleTreeMap();
        this.putValues();

        this.hoodInputs = new SparkInputsAutoLogged();
        this.turnTurretInputs = new SparkInputsAutoLogged();
        this.fuelToTurretInputs = new SparkInputsAutoLogged();

        this.leftMotorInputs = new SparkInputsAutoLogged();
        this.rightMotorInputs = new SparkInputsAutoLogged();

        this.turretInputs = new TurretIOinputsAutoLogged();

        if (isSimulation()) {
            this.turnTurretSim = new SparkMaxSim(turn_turret.getSpark(), DCMotor.getNeo550(1));

            turnTurretSim.enable();
        }
        configureShooter();

        SmartDashboard.putBoolean("isAutomatic", automatic);
    }

    @Override
    public double getDegreesPerRotationTurret(){
        return (360 / 66.8) * turn_turret.getPosition();
    }

    @Override
    public void updateInputs(TurretIOinputsAutoLogged turretIOinputsAutoLogged) {
        turretIOinputsAutoLogged.automatic = automatic;
        turretIOinputsAutoLogged.hoodSetpoint = hoodSetpoint;
        turretIOinputsAutoLogged.inbuxing = inbuxing();
        turretIOinputsAutoLogged.turretAtSetpoint = manual_turret_controller.atGoal();
        turretIOinputsAutoLogged.turretSetpoint = manual_turret_controller.getSetpoint().position;
    }

    @Override
    public void shotWithRPM(double RPM){
        shooterSetpoint = -RPM;
    }

    @Override
    public boolean inbuxing(){
        return indexer.getCurrent() > 30;
    }

    @Override
    public void turnOnFuelToTurret(double speed) {
        indexer.setSpeed(speed);
    }

    private void configureShooter(){
        right_motor.setCurrentLimit(SHOOTER_CURRENT_LIMIT);
        right_motor.setIdleMode(IdleMode.kBrake);
        right_motor.setInverted(false);
        right_motor.burnFlash();

        left_motor.setCurrentLimit(SHOOTER_CURRENT_LIMIT);
        left_motor.setIdleMode(IdleMode.kBrake);
        left_motor.setInverted(false);
        left_motor.burnFlash();

        hoodMotor.setCurrentLimit(HOOD_CURRENT_LIMIT);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        hoodMotor.burnFlash();

        indexer.setCurrentLimit(FUEL_TO_TURRET_CURRENT_LIMIT);
        indexer.setIdleMode(IdleMode.kCoast);
        indexer.burnFlash();
    }

    private void putValues(){
        map.put(7.276, MAX_TURN_POSITION); // esquerda
        map.put(0.871, MIN_TURN_POSITION); // direita
        map.put(3.990, 0.0); //central
    }

    @Override
    public double getMapValue(Pose2d pose2d){
        return map.get(pose2d.getY());
    }

    @Override
    public void turnToMapSetpoint(double setpoint){
        if(!automatic){
            if(setpoint > MAX_TURN_POSITION){
                setpoint = MAX_TURN_POSITION;
            } else if(setpoint < MIN_TURN_POSITION){
                setpoint = MIN_TURN_POSITION;
            }

            this.turretSetpoint = setpoint;

            manual_turret_controller.setGoal(setpoint, 0.1);

            double output = manual_turret_controller.calculate(turn_turret.getPosition());

            turn_turret.setSpeed(output);

            if(manual_turret_controller.atGoal()) return;
        }
    }

    @Override
    public void turnHoodFromSetpoint(double setpoint, double motor, BooleanSupplier tag){
        if(!automatic){
            if(setpoint > MAX_POSITION){
                setpoint = MAX_POSITION;
            } else if(setpoint < MIN_POSITION){
                setpoint = MIN_POSITION;
            }

            this.hoodSetpoint = setpoint;
            hoodController.setGoal(setpoint, 0.1);

            double output = hoodController.calculate(hoodMotor.getPosition());

            if(tag.getAsBoolean()){
                indexer.setSpeed(-0.5);
            }
            hoodMotor.setSpeed(output);
            shooterSetpoint = -motor;
        }
    }

    @Override
    public void turnHoodFromSetpoint(double setpoint, double motor){
        if(!automatic){
            if(setpoint > MAX_POSITION){
                setpoint = MAX_POSITION;
            } else if(setpoint < MIN_POSITION){
                setpoint = MIN_POSITION;
            }

            this.hoodSetpoint = setpoint;

            hoodController.setGoal(setpoint, 0.1);
            double output = hoodController.calculate(hoodMotor.getPosition());

            hoodMotor.setSpeed(output);
            shooterSetpoint = -motor;

            if(flyWheelController.atSetpoint() && motor > 0){
                indexer.setSpeed(-0.4);
            }
        }
    }

    @Override
    public void turnHoodFromSetpoint(double setpoint){
        if(!automatic){
            if(setpoint > MAX_POSITION){
                setpoint = MAX_POSITION;
            } else if(setpoint < MIN_POSITION){
                setpoint = MIN_POSITION;
            }

            this.hoodSetpoint = setpoint;
            hoodController.setGoal(setpoint, 0.1);

            double output = hoodController.calculate(hoodMotor.getPosition());
            hoodMotor.setSpeed(output);
        }
    }

    @Override
    public void shootWithForce(double speed){
        right_motor.setSpeed(-0.8);
    }

    @Override
    public boolean turretOnSetpoint(){
        return manual_turret_controller.atGoal();
    }

    @Override
    public void setSetpoint(double setpoint){
        manual_turret_controller.setGoal(setpoint);
    }

    @Override
    public void automatic(){
        hoodSetpoint = hoodMotor.getPosition();
        turretSetpoint = turn_turret.getPosition();

        if(hoodSetpoint == hoodMotor.getPosition() && turretSetpoint == turn_turret.getPosition()){
            automatic = !automatic;
        }
    }

    private void proccesInput(){
        Logger.processInputs("Mechanism/turret inputs", turnTurretInputs);
        updateInputs(turretInputs);

        hoodMotor.updateInputs(hoodInputs);
        turn_turret.updateInputs(turnTurretInputs);
        indexer.updateInputs(fuelToTurretInputs);

        left_motor.updateInputs(leftMotorInputs);
        right_motor.updateInputs(rightMotorInputs);
        flyWheelController.setSetpoint(shooterSetpoint);
    }

    @Override
    public void periodic() {
        proccesInput();
        flyWheelController.setSetpoint(shooterSetpoint);
        double shooterOutput = flyWheelController.calculate(-right_motor.getRPM());

        System.out.println("RPM: " + left_motor.getRPM());
        System.out.println("position: " + turn_turret.getPosition());

        if (shooterSetpoint == 0) {
            left_motor.setRPM(0);
            right_motor.setRPM(0);
        } else {
            left_motor.setRPM(shooterOutput);
            right_motor.setRPM(-shooterOutput);
        }

        // if(shooterSetpoint == 0){
        //     leftController.setSetpoint(0, ControlType.kVelocity);
        //     rightController.setSetpoint(0, ControlType.kVelocity);
        // } else {
        //     leftController.setSetpoint(-shooterSetpoint, ControlType.kVelocity);
        //     rightController.setSetpoint(shooterSetpoint, ControlType.kVelocity);
        // }

        // System.out.println("RPM: " + left_motor.getRate());
        // System.out.println("setpoint: " + leftController.getSetpoint());
    }

    @Override
    public double regulateTa(){
        double valorA = LimelightHelpers.getTA("");

        if(valorA > 1){
            return 0.5;
        } else if(valorA < 1.0 && valorA > 0.8){
            return 0.52;
        } else if(valorA < 0.8 && valorA > 0.6){
            return 0.54;
        } else if(valorA < 0.6 && valorA > 0.4){
            return 0.56;
        } else if(valorA < 0.4 && valorA > 0.2){
            return 0.58;
        } else {
            return 0.61;
        }
    }

    @Override
    public boolean isAutomatic(){
        return automatic;
    }

    @Override
    public Command normalTurretCommand(DoubleSupplier turret, DoubleSupplier hood, DoubleSupplier shooter, BooleanSupplier acel,
                                       BooleanSupplier spellFuels){
        return run(() ->{
            if(goal == TurretGoal.MANUAL && automatic == false){
                double hoodInput = hood.getAsDouble() * 0.1;
                double turretInput = turret.getAsDouble();
                double shooterInput = shooter.getAsDouble();

                if(hoodInput < 0){
                    hoodSetpoint = (hoodSetpoint + 0.205694851156169075657563) + hoodInput;
                } else if(hoodInput > 0){
                    hoodSetpoint = (hoodSetpoint - 0.205694851156169075657563) + hoodInput;
                }

                if (hoodSetpoint >= MAX_POSITION) {
                    hoodSetpoint = MAX_POSITION;
                } else if (hoodSetpoint <= MIN_POSITION) {
                    hoodSetpoint = MIN_POSITION;
                    hoodMotor.resetPositionByEncoder(0);
                }

                hoodController.setGoal(hoodSetpoint);
                double hoodPID = hoodController.calculate(hoodMotor.getPosition());

                if(turretInput > 0){
                    turretSetpoint = (turretSetpoint + 0.0190) + turretInput * 0.6;
                } else if(turretInput < 0){
                    turretSetpoint = (turretSetpoint - 0.0190) + turretInput * 0.6;
                }

                if (turretSetpoint >= MAX_TURN_POSITION && turretInput > 0) {
                    turretSetpoint = MAX_TURN_POSITION;
                } else if (turretSetpoint <= MIN_TURN_POSITION && turretInput < 0) {
                    turretSetpoint = MIN_TURN_POSITION;
                }

                manual_turret_controller.setGoal(turretSetpoint);
                double turretOutput = manual_turret_controller.calculate(turn_turret.getPosition());

                turn_turret.setSpeed(turretOutput);

                hoodMotor.setSpeed(hoodPID);
                indexer.setSpeed(-0.4 * shooterInput);

                shooterSetpoint = 6000 * -shooterInput * 0.4;

                if(acel.getAsBoolean()){
                    if(shooterInput < 0.6){
                        shooterSetpoint = -3600;
                    } else {
                        shooterSetpoint = (-shooterInput * 0.8) * 6000;
                    }
                } else {
                     shooterSetpoint = (-shooterInput * 0.8) * 6000;
                }

                if(spellFuels.getAsBoolean()){
                    indexer.setSpeed(0.6);
                } else {
                    indexer.setSpeed(-0.6 * shooterInput);
                }

            } else if(automatic){
                turretSetpoint = 0.0;
                automatic_turret_controller.setGoal(turretSetpoint);

                double outptut = automatic_turret_controller.calculate(LimelightHelpers.getTX(""));
                Voltage volts = Volts.of(-outptut);

                if(turn_turret.getPosition() >= MAX_TURN_POSITION && outptut < 0 ||
                   turn_turret.getPosition() <= MIN_TURN_POSITION && outptut > 0){
                    turn_turret.setVoltage(0);
                } else {
                    turn_turret.setVoltage(volts);
                }

                hoodSetpoint = LimelightHelpers.getTY("");

                hoodController.setGoal(-hoodSetpoint);

                double hoodOutput = hoodController.calculate(hoodMotor.getPosition());

                Voltage voltageHood = Volts.of(hoodOutput);
                hoodMotor.setVoltage(voltageHood);

                if(hoodOutput > 0 && hoodMotor.getPosition()>= MAX_POSITION ||
                    hoodOutput < 0 && hoodMotor.getPosition()<= MIN_POSITION){
                    hoodMotor.setVoltage(0);
                } else {
                    hoodMotor.setVoltage(voltageHood);
                }

                double shoot = shooter.getAsDouble();

                if(shoot <= 0.1){
                    shooterSetpoint = 0;
                } else {
                    shooterSetpoint = 6000 * ((-shoot * 0.4));
                }

                if(acel.getAsBoolean()){
                    if(shoot < 0.6){
                        shooterSetpoint = 6000 * ((-0.6 * 0.4) * regulateTa());
                    } else {
                        shooterSetpoint = 6000 * ((-shoot * 0.4) * regulateTa());
                    }
                } else {
                    shooterSetpoint = 6000 * ((-shoot * 0.4));
                }

                flyWheelController.setSetpoint(shooterSetpoint);
                double shooterOutput = flyWheelController.calculate(-right_motor.getRPM());

                if (shooterSetpoint == 0) {
                    left_motor.setRPM(0);
                    right_motor.setRPM(0);
                } else {
                    left_motor.setRPM(shooterOutput);
                    right_motor.setRPM(-shooterOutput);
                }


                indexer.setSpeed(-0.4 * shoot);

                if(spellFuels.getAsBoolean() || inbuxing()){
                    indexer.setSpeed(0.4);
                } else {
                    indexer.setSpeed(-0.4 * shoot);
                }
            } else {
                turn_turret.setVoltage(0);
                indexer.setVoltage(0);
                left_motor.setVoltage(0);
                right_motor.setVoltage(0);
            }
        });
    }

    public enum TurretGoal{
        MANUAL,
        OFF
    }

    @Override
    public void turnOfAllComponents(){
        turn_turret.setVoltage(0);
        hoodMotor.setVoltage(0);
        right_motor.setRPM(0);
    }
}
