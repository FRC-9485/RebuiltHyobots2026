package frc.robot.commands.swerveUtils;

import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.frc_java9485.constants.VisionConsts;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionInstances;

public class AlignWithTag extends Command {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem limelight;

    private Optional<Pose3d> tagPose;

    private final Transform2d transform2d;
    private final PIDController x, y, omega;
    private final AprilTagFieldLayout fieldLayout;

    public AlignWithTag() {
        swerve = SwerveSubsystem.getInstance();
        limelight = VisionInstances.getLimelightInstance();

        fieldLayout = VisionConsts.APRIL_TAG_FIELD_LAYOUT;
        transform2d = new Transform2d(0.4, 0.4, Rotation2d.fromRadians(0.4));


        x = new PIDController(2.2, 0, 0);
        y = new PIDController(2.2, 0, 0);
        omega = new PIDController(4.0, 0, 0.002);

        addRequirements(swerve, limelight);
    }

    @Override
    public void initialize() {
        x.setTolerance(0.1);
        y.setTolerance(0.4);
        omega.setTolerance(Math.toRadians(1));

        omega.enableContinuousInput(-Math.PI, -Math.PI);
        tagPose = fieldLayout.getTagPose(limelight.getBestTarget().fiducialId);
    }

    @Override
    public void execute() {
        if (limelight.hasTargets()) {
            if (tagPose == null) return;

            Pose2d current = swerve.getPose2d();
            Pose2d target = tagPose.get().toPose2d().transformBy(transform2d);

            double xOut = x.calculate(current.getX(), target.getX());
            double yOut = y.calculate(current.getY(), target.getY());
            double rotation = omega.calculate(current.getRotation().getRadians(), target.getRotation().getRadians());

            swerve.drive(new Translation2d(xOut, yOut), rotation, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.lock();
    }

    @Override
    public boolean isFinished() {
        return x.atSetpoint() && y.atSetpoint() && omega.atSetpoint();
    }
}
