package frc.robot.subsystems.swerve.IO;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.Command;

import swervelib.simulation.ironmaple.simulation.drivesims.GyroSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  public static class SwerveInputs{
    public Pose2d currentPose2d = new Pose2d();
    public Pose3d currentPose3d = new Pose3d();
    public SwerveModuleState[] moduleStates = {};
    public double[] currentCanCodersPosition = {0, 0, 0, 0};
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  }

  public Pose2d getPose2d();
  public Pose3d getPose3d();

  public Rotation2d getHeading2d();
  public Rotation3d getHeading3d();

  public void resetOdometry(Pose2d pose);
  public void resetOdometry(Pose3d pose);

  public void addVisionMeasurement(Pose3d visionMeasurement, double timestampSeconds);
  public void addVisionMeasurement(Pose3d visionMeasurement, double timestampSeconds, Matrix<N4, N1> stdDevs);

  public Pigeon2 getPigeon();
  public GyroSimulation getGyroSimulation();

  public SwerveDriveSimulation getSimulation();

  public ChassisSpeeds getRobotRelativeSpeeds();

  public void driveFieldOriented(ChassisSpeeds speed);


  public Command getAutonomousCommand(String path, boolean altern);

  public Command driveCommand(
      DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, boolean fieldOriented);

  public void updateInputs(SwerveInputs inputs);

  public void drive(Translation2d translation2d, double rotation, boolean fieldOriented);

  public void lock();
}
