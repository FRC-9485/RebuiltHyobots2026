package frc.robot.subsystems.mechanism.shooter.turret.turretOFC;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import static frc.frc_java9485.constants.FieldConsts.*;
import static frc.frc_java9485.constants.mechanisms.TurretConsts.*;
import static frc.frc_java9485.constants.mechanisms.HoodConsts.*;

import frc.frc_java9485.motors.spark.SparkFlexMotor;
import frc.frc_java9485.motors.spark.SparkMaxMotor;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;

import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretCalculator.ShotData;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretIO.TurretIOInputs;

import static frc.frc_java9485.constants.RobotConsts.*;

public class TurretSubsystem extends SubsystemBase{

    private final TurretIO io;
    private final Supplier<ChassisSpeeds> chassisSpeed;
    private final Supplier<Pose2d> pose;

    private final SparkFlexMotor right_motor;

    private final SparkMaxMotor turn_turret;
    private final SparkMaxMotor fuel_to_turret;
    private final SparkMaxMotor hoodMotor;

    public final TunableProfiledController turretController;
    private final TunableProfiledController hoodController;
    private final TunableProfiledController flyWheelController;
    private final TurretIOInputsAutoLogged inputs;

    @AutoLogOutput
    private TurretGoal goal = TurretGoal.MANUAL;

    private final InterpolatingDoubleTreeMap map;

    private SparkMaxSim turnTurretSim;

    @AutoLogOutput
    private double hoodSetpoint = 0;
    private double turretSetpoint = 0;
    private boolean automatic = false;

    @AutoLogOutput
    private Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? HUB_BLUE
                : HUB_RED;

    public TurretSubsystem(TurretIO io,Supplier<ChassisSpeeds> chassisSpeed, Supplier<Pose2d> pose){
        this.chassisSpeed = chassisSpeed;
        this.pose = pose;
        this.io = io;

        this.setTarget(HUB_BLUE);

        this.right_motor = new SparkFlexMotor(RIGHT_SHOOTER, "right shooter");
        this.hoodMotor = new SparkMaxMotor(HOOD_MOTOR_ID, "hood motor");

        this.turn_turret = new SparkMaxMotor(TURN_TURRET, "turn turret");
        this.fuel_to_turret = new SparkMaxMotor(FUEL_TO_TURRET, "catch fuel to turret");

        this.hoodMotor.resetPositionByEncoder(MIN_POSITION);
        this.turn_turret.resetPositionByEncoder(0);

        this.turretController = new TunableProfiledController(TURRET_TUNABLE);
        this.hoodController = new TunableProfiledController(TUNABLE_CONSTANTS);
        this.flyWheelController = new TunableProfiledController(SHOOTER_CONSTANTS);

        this.map = new InterpolatingDoubleTreeMap();
        this.putValues();

        this.inputs = new TurretIOInputsAutoLogged();

        if (isSimulation()) {
            this.turnTurretSim = new SparkMaxSim(turn_turret.getSpark(), DCMotor.getNeo550(1));

            turnTurretSim.enable();
        }

        configureShooter();
    }

    public void setSpeed(double speed){
        turn_turret.setSpeed(speed);
    }

    public double getTurretPosition(){
        return turn_turret.getPosition();
    }

    private void configureShooter(){
        right_motor.setCurrentLimit(SHOOTER_CURRENT_LIMIT);
        right_motor.burnFlash();

        hoodMotor.setCurrentLimit(HOOD_CURRENT_LIMIT);
        hoodMotor.setIdleMode(IdleMode.kBrake);
        hoodMotor.burnFlash();

        fuel_to_turret.setCurrentLimit(FUEL_TO_TURRET_CURRENT_LIMIT);
        fuel_to_turret.burnFlash();
    }

    private void updateInputs(TurretIOInputs inputs){
        inputs.turnPosition = Rotations.of(turn_turret.getPosition());
    }

    public void setTarget(Translation3d target) {
        currentTarget = target;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            Translation2d flipped = FlippingUtil.flipFieldPosition(target.toTranslation2d());
            currentTarget = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
        }
    }

    private void putValues(){
        map.put(7.276, MAX_TURN_POSITION); // esquerda
        map.put(0.871, MIN_TURN_POSITION); // direita
        map.put(3.990, 0.0); //central
    }

    public double getMapValue(Pose2d pose2d){
        return map.get(pose2d.getY());
    }

    public void turnToMapSetpoint(double setpoint){
        automatic = true;

            if(setpoint > MAX_TURN_POSITION){
                setpoint = MAX_TURN_POSITION;
            } else if(setpoint < MIN_TURN_POSITION){
                setpoint = MIN_TURN_POSITION;
            }

            this.turretSetpoint = setpoint;

            turretController.setGoal(setpoint, 0.1);

            double output = turretController.calculate(turn_turret.getPosition());

            turn_turret.setSpeed(output);

        if(turretController.atGoal()) return;
    }

    public void turnHoodFromSetpoint(double setpoint){
          if(setpoint > MAX_POSITION){
                setpoint = MAX_POSITION;
            } else if(setpoint < MIN_POSITION){
                setpoint = MIN_POSITION;
            }

            this.hoodSetpoint = setpoint;

            hoodController.setGoal(setpoint, 0.1);

            double output = hoodController.calculate(hoodMotor.getPosition());

            hoodMotor.setSpeed(output);

        if(hoodController.atGoal()) return;
    }

    public boolean turretOnSetpoint(){
        return turretController.atGoal();
    }

    public void setSetpoint(double setpoint){
        turretController.setGoal(setpoint);
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("turret inputs", inputs);

        System.out.println("setpoint: " + turretSetpoint);
        System.out.println("Pose: " + pose.get().getY());

        if (goal == TurretGoal.SCORING || goal == TurretGoal.PASSING) {

        }

         if (goal == TurretGoal.PASSING) {
            setTarget(getPassingTarget(pose.get()));
        }

        if(goal == TurretGoal.OFF){
            turnOfAllComponents();
            fuel_to_turret.setSpeed(0);
        }
    }

    @Override
    public void simulationPeriodic() {}

    private Translation3d getPassingTarget(Pose2d pose) {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        boolean onBlueLeftSide = this.pose.get().getMeasureY().gt(FIELD_WIDTH.div(2));

        return isBlue == onBlueLeftSide ? PASSING_SPOT_LEFT : PASSING_SPOT_RIGHT;
    }

    public void setGoal(TurretGoal goal){
            this.goal = goal;
            switch (goal) {
                case SCORING:
                    setTarget(HUB_BLUE);
                break;

                case OFF:
                    turnOfAllComponents();
                break;

                case PASSING:
                    setTarget(getPassingTarget(pose.get()));
                break;

                case MANUAL:

                break;

                case SECURITY:

                break;
            }
    }

    public Command normalTurretCommand(DoubleSupplier turret, DoubleSupplier hood, DoubleSupplier shooter, BooleanSupplier acel,
                                       BooleanSupplier spellFuels){
        return run(() ->{
            if(goal == TurretGoal.MANUAL){
                double hoodInput = hood.getAsDouble() * 0.1;
                double turretInput = turret.getAsDouble();
                double shooterInput = shooter.getAsDouble();

                // faz o setpoint - hood
                if(hoodInput < 0){
                    hoodSetpoint = (hoodSetpoint + 0.205694851156169075657563) + hoodInput;
                } else if(hoodInput > 0){
                    hoodSetpoint = (hoodSetpoint - 0.205694851156169075657563) + hoodInput;
                }

                //limita o setpoint - hood
                if (hoodSetpoint >= MAX_POSITION) {
                    hoodSetpoint = MAX_POSITION;
                } else if (hoodSetpoint <= MIN_POSITION) {
                    hoodSetpoint = MIN_POSITION;
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

                    turretController.setGoal(turretSetpoint);
                    double turretOutput = turretController.calculate(turn_turret.getPosition());

                    turn_turret.setSpeed(turretOutput);

                hoodMotor.setSpeed(hoodPID);
                fuel_to_turret.setSpeed(-0.4 * shooterInput);
                right_motor.setSpeed(-shooterInput * 0.6);

                if(acel.getAsBoolean()){
                    if(shooterInput < 0.4){
                        right_motor.setSpeed(-0.4);
                    } else {
                        right_motor.setSpeed(-shooterInput * 0.6);
                    }
                } else {
                    right_motor.setSpeed(-shooterInput * 0.6);
                }

                if(spellFuels.getAsBoolean()){
                    fuel_to_turret.setSpeed(0.4);
                } else {
                    fuel_to_turret.setSpeed(-0.4 * shooterInput);
                }

            } else {
                turn_turret.setSpeed(0);
                hoodMotor.setSpeed(0);
                right_motor.setSpeed(0);
                fuel_to_turret.setSpeed(0);
            }

        });
    }

    public Command calculateShot(Pose2d robotPose, DoubleSupplier turret) {
        return run(() ->{

        ChassisSpeeds fieldSpeeds = chassisSpeed.get();
        double input = turret.getAsDouble();

        ShotData calculatedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(robotPose,
        fieldSpeeds,
        currentTarget,
        LOOKAHEAD_ITERATIONS);

        Angle azimuthAngle =
        TurretCalculator.calculateAzimuthAngle(robotPose, calculatedShot.target(), inputs.turnPosition);
        AngularVelocity azimuthVelocity = RadiansPerSecond.of(fieldSpeeds.omegaRadiansPerSecond);

        // turretController.setGoal(azimuthAngle.in(Rotations), azimuthVelocity.in(RadiansPerSecond));
        hoodController.setGoal(calculatedShot.getHoodAngle().in(Rotations));
        flyWheelController.setGoal(TurretCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), FLY_WHEEL_RADIUS).in(RPM));

        // Voltage turretVoltage = Volts.of(turretController.calculate(turn_turret.getPosition()));
        Voltage hoodVoltage = Volts.of(hoodController.calculate(hoodMotor.getPosition()));
        Voltage flyWheelVoltage = Volts.of(flyWheelController.calculate(RPM.of(right_motor.getRate()).in(RPM)));

        turn_turret.setVoltage(input * 0.24);
        hoodMotor.setVoltage(hoodVoltage);

        // right_motor.setVoltage(flyWheelVoltage.times(-1));

        System.out.println("turret goal: " + turretController.getGoal());
        System.out.println("\nturret atual: " + turn_turret.getPosition());

        Logger.recordOutput("Turret/Shot", calculatedShot);
    });
    }

    public enum TurretGoal{
        SCORING,
        SECURITY,
        PASSING,
        MANUAL,
        OFF
    }

    private void turnOfAllComponents(){
        turn_turret.setVoltage(0);
        hoodMotor.setVoltage(0);
        // fuel_to_turret.setVoltage(0);
        right_motor.setVoltage(0);

        // turretController.setGoal(turn_turret.getPosition());
        // turn_turret.setVoltage(Volts.of(turretController.calculate(turn_turret.getPosition())));

        // hoodController.setGoal(hoodMotor.getPosition());
        // hoodMotor.setVoltage(Volts.of(hoodController.calculate(hoodMotor.getTemperature())));

        // fuel_to_turret.setSpeed(0);

        // flyWheelController.setGoal(left_motor.getPosition());
        // left_motor.setVoltage(Volts.of(left_motor.getPosition()));
        // right_motor.setVoltage(Volts.of(-right_motor.getPosition()));
    }
}
