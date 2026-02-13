package frc.robot.subsystems.mechanism.shooter.turret.turretOFC;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.frc_java9485.constants.FieldConsts.*;
import static frc.frc_java9485.constants.mechanisms.TurretConsts.*;

import frc.frc_java9485.constants.mechanisms.HoodConsts;
import frc.frc_java9485.motors.spark.SparkFlexMotor;
import frc.frc_java9485.motors.spark.SparkMaxMotor;
import frc.frc_java9485.utils.TunableControls.TunableProfiledController;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretCalculator.ShotData;
import frc.robot.subsystems.mechanism.shooter.turret.turretOFC.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase{
    
    private final TurretIO io;
    private final Supplier<ChassisSpeeds> chassisSpeed;
    private final Supplier<Pose2d> pose;

    private final SparkFlexMotor left_motor;
    private final SparkFlexMotor right_motor;

    private final SparkMaxMotor turn_turret;
    private final SparkMaxMotor fuel_to_turret;
    private final SparkMaxMotor hoodMotor;

    private final TunableProfiledController turretController;
    private final TurretIOInputsAutoLogged inputs;

    private final RelativeEncoder encoder;

    private boolean isActive = false;

    private Translation3d currentTarget;

    public Turret(TurretIO io,Supplier<ChassisSpeeds> chassisSpeed, Supplier<Pose2d> pose){
        this.chassisSpeed = chassisSpeed;
        this.pose = pose;
        this.io = io;

        this.left_motor = new SparkFlexMotor(LEFT_SHOOTER, "left shooter");
        this.right_motor = new SparkFlexMotor(RIGHT_SHOOTER, "right shooter");
        this.hoodMotor = new SparkMaxMotor(HoodConsts.MOTOR_ID, "hood motor");

        this.turn_turret = new SparkMaxMotor(TURN_TURRET, "turn turret");
        this.fuel_to_turret = new SparkMaxMotor(FUEL_TO_TURRET, "catch fuel to turret");
        this.encoder = turn_turret.getEncoder();

        this.turretController = new TunableProfiledController(TURRET_TUNABLE);
        this.inputs = new TurretIOInputsAutoLogged();

        configureShooter();
    }

    private void configureShooter(){
        right_motor.setInvert();
        right_motor.followMotor(LEFT_SHOOTER);
        right_motor.burnFlash();

        hoodMotor.setInvert();
        hoodMotor.burnFlash();
    }

    private void updateInputs(TurretIOInputs inputs){
        inputs.turnPosition = encoder.getPosition();
    }

    public Command setTarget(Translation3d target) {
        return this.runOnce(() -> {
            currentTarget = target;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                Translation2d flipped = FlippingUtil.flipFieldPosition(target.toTranslation2d());
                currentTarget = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
            }
        });
    }

    public Command stop(){
        return runOnce(() ->{
            turn_turret.setIdleMode(IdleMode.kBrake);
            isActive = false;
        });
    }

    public Command start(){
        return runOnce(() ->{
            isActive = true;
        });
    }

    @Override
    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("turret inputs", inputs);

        calculateShot();
    }
    
    private void calculateShot(){
        if(isActive){
            currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? HUB_BLUE
                : HUB_RED;

            Pose2d robot = pose.get();
            ChassisSpeeds fieldSpeeds = chassisSpeed.get();

            ShotData calculatedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                        robot, fieldSpeeds, currentTarget, LOOKAHEAD_ITERATIONS);
            Angle azimuthAngle = TurretCalculator.calculateAzimuthAngle(robot, calculatedShot.target());
            AngularVelocity azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);

            turretController.setGoal(azimuthAngle.in(Radians), azimuthVelocity.in(RadiansPerSecond));

            Voltage turnVoltage = Volts.of(turretController.calculate(Radians.of(inputs.turnPosition).in(Radians)));

            turn_turret.setVoltage(turnVoltage);
        }
    }
}
