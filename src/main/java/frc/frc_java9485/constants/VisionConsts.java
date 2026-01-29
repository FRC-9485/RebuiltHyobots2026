package frc.frc_java9485.constants;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.frc_java9485.constants.RobotConsts.RobotModes;

public class VisionConsts {
    public static final String RASPBERRY_CAMERA_NAME = "raspphoto";
    public static final Transform3d RASPBERRY_ROBOT_TO_CAMERA = new Transform3d(new Translation3d(), new Rotation3d());

    public static final String LIMELIGHT_CAMERA_NAME = "limelightphoto";
    public static final Transform3d LIMELIGHT_ROBOT_TO_CAMERA = new Transform3d(new Translation3d(), new Rotation3d());

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Matrix<N4, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(2, 2, 2, 0);
    public static final Matrix<N4, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1, 0);

    public static SimCameraProperties LIMELIGHT_CAMERA_PROPS;
    public static SimCameraProperties RASPBERRY_CAMERA_PROPS;

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose3d pose, double timestamp, Matrix<N4, N1> estimationStdDevs);
    }

    public enum Cooprocessor {
        RASPBERRY,
        LIMELIGHT
    }

    static {
        if (RobotConsts.CURRENT_ROBOT_MODE == RobotModes.SIM) {
            LIMELIGHT_CAMERA_PROPS = new SimCameraProperties();
            RASPBERRY_CAMERA_PROPS = new SimCameraProperties();

            LIMELIGHT_CAMERA_PROPS.setFPS(60);
            LIMELIGHT_CAMERA_PROPS.setAvgLatencyMs(35);
            LIMELIGHT_CAMERA_PROPS.setLatencyStdDevMs(10);
            LIMELIGHT_CAMERA_PROPS.setCalibError(0.2, 0.2);
            LIMELIGHT_CAMERA_PROPS.setCalibration(640, 480, Rotation2d.fromDegrees(82));

            RASPBERRY_CAMERA_PROPS .setFPS(50);
            RASPBERRY_CAMERA_PROPS.setAvgLatencyMs(35);
            RASPBERRY_CAMERA_PROPS.setLatencyStdDevMs(5);
            RASPBERRY_CAMERA_PROPS.setCalibError(0.5, 0.5);
            RASPBERRY_CAMERA_PROPS.setCalibration(1920, 1080, Rotation2d.fromDegrees(110));
        }
    }
}
