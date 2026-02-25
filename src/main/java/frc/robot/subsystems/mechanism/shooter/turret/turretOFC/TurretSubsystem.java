package frc.robot.subsystems.mechanism.shooter.turret.turretOFC;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private final TunableProfiledController turretController;
    private final TunableProfiledController hoodController;
    private final TunableProfiledController flyWheelController;
    private final TurretIOInputsAutoLogged inputs;

    @AutoLogOutput
    private TurretGoal goal = TurretGoal.OFF;

    private SparkMaxSim turnTurretSim;

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

        this.inputs = new TurretIOInputsAutoLogged();

        if (isSimulation()) {
            this.turnTurretSim = new SparkMaxSim(turn_turret.getSpark(), DCMotor.getNeo550(1));

            turnTurretSim.enable();
        }

        configureShooter();
    }

    private void configureShooter(){
        right_motor.setCurrentLimit(SHOOTER_CURRENT_LIMIT);
        right_motor.burnFlash();

        hoodMotor.setCurrentLimit(HOOD_CURRENT_LIMIT);
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

    @Override
    public void periodic() {
        Logger.processInputs("turret inputs", inputs);
        updateInputs(inputs);

        if (goal == TurretGoal.SCORING || goal == TurretGoal.PASSING) {
            calculateShot(pose.get());
            // fuel_to_turret.setSpeed(-0.6);
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

                case SECURITY:

                break;
            }
    }

    private void calculateShot(Pose2d robotPose) {
        ChassisSpeeds fieldSpeeds = chassisSpeed.get();

        ShotData calculatedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(robotPose,
                                                                                          fieldSpeeds,
                                                                                          currentTarget,
                                                                                          LOOKAHEAD_ITERATIONS);

        Angle azimuthAngle =
                TurretCalculator.calculateAzimuthAngle(robotPose, calculatedShot.target(), inputs.turnPosition);
        AngularVelocity azimuthVelocity = RadiansPerSecond.of(fieldSpeeds.omegaRadiansPerSecond);

        turretController.setGoal(azimuthAngle.in(Rotations), azimuthVelocity.in(RadiansPerSecond));
        hoodController.setGoal(calculatedShot.getHoodAngle().in(Rotations));
        flyWheelController.setGoal(TurretCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), FLY_WHEEL_RADIUS).in(RPM));

        Voltage turretVoltage = Volts.of(turretController.calculate(turn_turret.getPosition()));
        Voltage hoodVoltage = Volts.of(hoodController.calculate(hoodMotor.getPosition()));
        Voltage flyWheelVoltage = Volts.of(flyWheelController.calculate(RPM.of(right_motor.getRate()).in(RPM)));

        turn_turret.setVoltage(turretVoltage);
        hoodMotor.setVoltage(hoodVoltage);

        right_motor.setVoltage(flyWheelVoltage.times(-1));

        System.out.println("turret goal: " + turretController.getGoal());
        System.out.println("\nturret atual: " + turn_turret.getPosition());

        Logger.recordOutput("Turret/Shot", calculatedShot);
    }

    public enum TurretGoal{
        SCORING,
        SECURITY,
        PASSING,
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
