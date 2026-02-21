package frc.robot.subsystems.mechanism.shooter.turret.turretOFC;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
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

    private final SparkFlexMotor left_motor;
    private final SparkFlexMotor right_motor;

    private final SparkMaxMotor turn_turret;
    private final SparkMaxMotor fuel_to_turret;
    private final SparkMaxMotor hoodMotor;

    private final TunableProfiledController turretController;
    private final TunableProfiledController hoodController;
    private final TurretIOInputsAutoLogged inputs;

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

        this.left_motor = new SparkFlexMotor(LEFT_SHOOTER, "left shooter");
        this.right_motor = new SparkFlexMotor(RIGHT_SHOOTER, "right shooter");
        this.hoodMotor = new SparkMaxMotor(HOOD_MOTOR_ID, "hood motor");
        this.hoodMotor.resetPositionByEncoder(MIN_POSITION);

        this.turn_turret = new SparkMaxMotor(TURN_TURRET, "turn turret");
        this.fuel_to_turret = new SparkMaxMotor(FUEL_TO_TURRET, "catch fuel to turret");

        this.turretController = new TunableProfiledController(TURRET_TUNABLE);
        this.hoodController = new TunableProfiledController(TUNABLE_CONSTANTS);

        this.inputs = new TurretIOInputsAutoLogged();

        if (isSimulation()) {
            this.turnTurretSim = new SparkMaxSim(turn_turret.getSpark(), DCMotor.getNeo550(1));

            turnTurretSim.enable();
        }

        configureShooter();
    }

    private void configureShooter(){
        // right_motor.followMotor(LEFT_SHOOTER);
        right_motor.setCurrentLimit(SHOOTER_CURRENT_LIMIT);
        right_motor.burnFlash();

        left_motor.setCurrentLimit(SHOOTER_CURRENT_LIMIT);
        left_motor.burnFlash();

        hoodMotor.setInvert();
        hoodMotor.setCurrentLimit(HOOD_CURRENT_LIMIT);
        hoodMotor.burnFlash();

        fuel_to_turret.setCurrentLimit(FUEL_TO_TURRET_CURRENT_LIMIT);
        fuel_to_turret.burnFlash();
    }

    private void updateInputs(TurretIOInputs inputs){
        inputs.turnPosition = isSimulation() ? turnTurretSim.getPosition() : turn_turret.getPosition();
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
        updateInputs(inputs);
        Logger.processInputs("turret inputs", inputs);
        System.out.println("turret Goal:" + goal.toString());
        System.out.println("hood: " + hoodMotor.getPosition());

         if (goal == TurretGoal.SCORING || goal == TurretGoal.PASSING) {
            fuel_to_turret.setSpeed(-0.8);
            left_motor.setSpeed(0.9);
            right_motor.setSpeed(-0.9);
            calculateShot();
        }

         if (goal == TurretGoal.PASSING) {
            setTarget(getPassingTarget(pose.get()));
        }

        if(goal == TurretGoal.OFF){
            turnOfAllComponents();
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
                    calculateShot();
                break;

                case PASSING:
                    setTarget(getPassingTarget(pose.get()));
                break;

                case SECURITY:

                break;
            }
    }

    private void calculateShot(){
        ChassisSpeeds fieldSpeeds = chassisSpeed.get();

        ShotData calculatedShot = TurretCalculator.iterativeMovingShotFromMap(
                pose.get(), fieldSpeeds, currentTarget, LOOKAHEAD_ITERATIONS);
        Angle azimuthAngle =
                TurretCalculator.calculateAzimuthAngle(pose.get(), calculatedShot.getTarget(), Radians.of(inputs.turnPosition));

        AngularVelocity azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);

        turretController.setGoal(-azimuthAngle.in(Radians));
        hoodController.setGoal(calculatedShot.getHoodAngle().in(Degrees));

        Voltage turnVoltage = Volts.of(turretController.calculate(turn_turret.getPosition()));
        Voltage hoodVoltage = Volts.of(hoodController.calculate(hoodMotor.getPosition()));

        hoodMotor.setVoltage(hoodVoltage);
        turn_turret.setVoltage(turnVoltage);

        Logger.recordOutput("Turret/Pose 3d", new Pose3d(
            new Translation3d(
                Inches.of(0),
                Inches.of(0),
                Inches.of(32.5)
            ),
            new Rotation3d(
                Degrees.of(0),
                Degrees.of(0),
                Degrees.of(turn_turret.getPosition())
            )
        ));

        if (!isSimulation()) {
        } else {
            turnTurretSim.setAppliedOutput(turnVoltage.in(Volts));
            System.out.println("volts: " + turnVoltage.in(Volts));

            turnTurretSim.iterate(azimuthVelocity.in(RadiansPerSecond), 12.0, 0.02);

            Logger.recordOutput("turretpose",
                new Pose3d(
                    Inches.of(8), //x
                Inches.of(0), //y
                Inches.of(35.2), //z
                new Rotation3d(
                    Degrees.of(0), //yaw
                    Degrees.of(0), //pitch
                    Degrees.of((turnTurretSim.getPosition() * 360.0) - 90.0)
                    )) //roll
                );
        }

    }

    public enum TurretGoal{
        SCORING,
        SECURITY,
        PASSING,
        OFF
    }

    private void turnOfAllComponents(){
        turn_turret.setSpeed(0);
        hoodMotor.setSpeed(0);
        fuel_to_turret.setSpeed(0);
        left_motor.setSpeed(0);
        right_motor.setSpeed(0);
    }
}
