package frc.robot.subsystems.swerve;

import static frc.frc_java9485.constants.RobotConsts.CURRENT_ROBOT_MODE;
import static frc.frc_java9485.constants.mechanisms.DriveConsts.*;

import java.io.File;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.frc_java9485.constants.ComponentsConsts.*;
import static frc.frc_java9485.constants.FieldConsts.ALLIANCE_ZONE;
import static frc.frc_java9485.constants.FieldConsts.FIELD_LENGTH;

import frc.frc_java9485.constants.RobotConsts.RobotModes;
import frc.frc_java9485.motors.io.SparkOdometryThread;
import frc.frc_java9485.utils.MathUtils;
import frc.robot.subsystems.swerve.IO.GyroIOInputsAutoLogged;
import frc.robot.subsystems.swerve.IO.PigeonIO;
import frc.robot.subsystems.swerve.IO.SwerveIO;
import frc.robot.subsystems.swerve.IO.SwerveInputsAutoLogged;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionInstances;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.GyroSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase implements SwerveIO {
  public static final Lock odometryLock = new ReentrantLock();

  private final SwerveDrive swerveDrive;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator3d poseEstimator;

  private final SwerveInputsAutoLogged swerveInputs;
  private final GyroIOInputsAutoLogged pigeonInputs;

  private final Pigeon2 pigeon;
  private final PigeonIO pigeonIO;

  private final CANcoder[] encoders; // FL FR BL BR

  private SwerveDriveSimulation driveSimulator;

  private Vision limelight;
  private Vision raspberry;

  private SwerveModule[] modules;
  private SwerveModuleState states[];

  private static Swerve mInstance;

  private final boolean isSimulation;

  public static Swerve getInstance() {
    if (mInstance == null) {
      mInstance = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
    }
    return mInstance;
  }

  private Swerve(File directory) {
    try {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
      swerveDrive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED);
      swerveDrive.setMotorIdleMode(false);

      if (CURRENT_ROBOT_MODE == RobotModes.SIM) {
        isSimulation = true;
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);

        driveSimulator = swerveDrive.getMapleSimDrive().get();
        driveSimulator.setEnabled(true);

        driveSimulator.config.gyroSimulationFactory = COTS.ofPigeon2();

        resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
      } else {
        isSimulation = false;
      }

      encoders =
          new CANcoder[] {
            new CANcoder(CANCODER_MODULE1_ID), // FL
            new CANcoder(CANCODER_MODULE2_ID), // FR
            new CANcoder(CANCODER_MODULE3_ID), // BR
            new CANcoder(CANCODER_MODULE4_ID)  // BL
          };

      pigeon = new Pigeon2(PIGEON2);
      pigeonIO = new PigeonIO();

      swerveInputs = new SwerveInputsAutoLogged();
      pigeonInputs = new GyroIOInputsAutoLogged();

      setupPathPlanner();
      SparkOdometryThread.getInstance().start();

      kinematics = new SwerveDriveKinematics(MODULES_TRANSLATIONS);
      poseEstimator =
          new SwerveDrivePoseEstimator3d(kinematics, getHeading3d(), swerveDrive.getModulePositions(),
                                        new Pose3d(), STATE_STD_DEVS, VISION_STD_DEVS);

      limelight = VisionInstances.getLimelightInstance();
      raspberry = VisionInstances.getRaspberryInstance();
        } catch (Exception e) {
      throw new RuntimeException("Erro criando Swerve!!!!\n", e);
    }
  }

  @Override
  public void periodic() {
    if (poseEstimator != null) {
      // raspberry.estimatePose(this::addVisionMeasurement);
      limelight.estimatePose(this::addVisionMeasurement);
      poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getHeading3d(), swerveDrive.getModulePositions());

      if (isSimulation) {
        poseEstimator.addVisionMeasurement(new Pose3d(driveSimulator.getSimulatedDriveTrainPose()), Timer.getFPGATimestamp());
      } else if (!isSimulation) {
        odometryLock.lock();
        pigeonIO.updateInputs(pigeonInputs);
        odometryLock.unlock();
        Logger.processInputs("Swerve/Odometry", pigeonInputs);
    }

    swerveDrive.updateOdometry();
    updateInputs(swerveInputs);
    Logger.processInputs("Swerve", swerveInputs);
    }
  }

  @Override
  public Pose3d getPose3d() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition().toPose2d();
  }

  @Override
  public Rotation2d getHeading2d() {
    return isSimulation ?
      driveSimulator.getGyroSimulation().getGyroReading() :
      Rotation2d.fromDegrees(MathUtils.scope0To360(pigeon.getYaw().getValueAsDouble()));
  }

  @Override
  public Rotation3d getHeading3d() {
    return isSimulation ?
      new Rotation3d(driveSimulator.getGyroSimulation().getGyroReading()) :
      pigeon.getRotation3d();
  }

  @Override
  public void resetOdometry(Pose3d pose) {
    if (poseEstimator != null) {
      poseEstimator.resetPose(pose);
    }
    if (isSimulation) {
      driveSimulator.setSimulationWorldPose(pose.toPose2d());
    }
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    if (poseEstimator != null) {
      poseEstimator.resetPose(new Pose3d(pose));
    }
    if (isSimulation) {
      driveSimulator.setSimulationWorldPose(pose);
    }
  }

  @Override
  public SwerveDriveSimulation getSimulation() {
      return driveSimulator;
  }

  @Override
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return isSimulation ?
      driveSimulator.getDriveTrainSimulatedChassisSpeedsRobotRelative() :
      swerveDrive.getRobotVelocity();
  }

  @Override
  public void driveFieldOriented(ChassisSpeeds speed) {
    swerveDrive.driveFieldOriented(speed);
  }

  @Override
  public Pigeon2 getPigeon() {
    return pigeon;
  }

  @Override
  public GyroSimulation getGyroSimulation() {
    return driveSimulator.getGyroSimulation();
  }

  @Override
  public void drive(Translation2d translation2d, double rotation, boolean fieldOriented) {
    swerveDrive.drive(translation2d, rotation, fieldOriented, false);
  }

  @Override
  public void lock() {
    swerveDrive.lockPose();
  }

  @Override
  public void addVisionMeasurement(Pose3d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  @Override
  public void addVisionMeasurement(Pose3d visionMeasurement, double timestampSeconds, Matrix<N4, N1> stdDevs) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }

  @Override
  public Command getAutonomousCommand(String path, boolean altern) {
    if (altern) {
      return AutoBuilder.buildAuto(path);
    }
    return new PathPlannerAuto(path);
  }

  @Override
  public Command driveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier omega, boolean fieldOriented) {
    return run(
        () -> {
          double Xcontroller = Math.pow(X.getAsDouble(), 3);
          double Ycontroller = Math.pow(Y.getAsDouble(), 3);
          double rotation = omega.getAsDouble();
          double td = 0.02;

          ChassisSpeeds speeds =
              fieldOriented
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      Xcontroller * swerveDrive.getMaximumChassisVelocity(),
                      Ycontroller * swerveDrive.getMaximumChassisVelocity(),
                      rotation * swerveDrive.getMaximumChassisAngularVelocity(),
                      isSimulation ?
                        getGyroSimulation().getGyroReading() :
                        pigeon.getRotation2d())
                  : new ChassisSpeeds(
                      Xcontroller * swerveDrive.getMaximumChassisVelocity(),
                      Ycontroller * swerveDrive.getMaximumChassisVelocity(),
                      rotation * swerveDrive.getMaximumChassisAngularVelocity());

          ChassisSpeeds discretize = ChassisSpeeds.discretize(speeds, td);
          states = swerveDrive.kinematics.toSwerveModuleStates(discretize);
          SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED);
          modules = swerveDrive.getModules();

          for (int i = 0; i < states.length; i++) {
            modules[i].setDesiredState(states[i], true, true);
          }
        });
  }



  private void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose2d,
          this::resetOdometry,
          this::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> driveFieldOriented(speeds),
          CURRENT_ROBOT_MODE == RobotModes.SIM ?
            new PPHolonomicDriveController(SIM_TRANSLATION_PID.getPIDConsants(), SIM_ROTATION_PID.getPIDConsants()) :
            new PPHolonomicDriveController(REAL_TRANSLATION_PID.getPIDConsants(), REAL_ROTATION_PID.getPIDConsants()),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);

      PathPlannerLogging.setLogActivePathCallback(
          (activePath) -> {
            Logger.recordOutput(
                ACTIVE_TRACJECTORY_LOG_ENTRY,
                activePath.toArray(new Pose2d[activePath.size()]));
          });

      PathPlannerLogging.setLogTargetPoseCallback(
          (targetPose) -> {
            Logger.recordOutput(TRAJECTORY_SETPOINT_LOG_ENTRY, targetPose);
          });

    } catch (Exception e) {
      System.out.println(e.getMessage());
    }
  }

  @Override
  public boolean inAllianceZone() {
        Pose2d pose = getPose2d();
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                        && pose.getMeasureX().lt(ALLIANCE_ZONE)
                || DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                        && pose.getMeasureX().gt(FIELD_LENGTH.minus(ALLIANCE_ZONE));
    }

  @Override
  public void updateInputs(SwerveInputs inputs) {
    inputs.currentPose2d = getPose2d();
    inputs.currentPose3d = getPose3d();

    inputs.moduleStates = DriverStation.isDisabled() ? new SwerveModuleState[] {} : states;
    if (encoders.length == 3) {
      byte i = 0;
      double[] cancoderPos = new double[] {};
      for (CANcoder encoder : encoders) {
        i++;
        cancoderPos[i] = encoder.getPosition().getValueAsDouble();
      }
      inputs.currentCanCodersPosition = cancoderPos;
      inputs.chassisSpeeds = getRobotRelativeSpeeds();
    }
  }
}
