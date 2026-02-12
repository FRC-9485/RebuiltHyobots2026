package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.RobotConsts;
import frc.frc_java9485.constants.VisionConsts;
import frc.frc_java9485.constants.RobotConsts.RobotModes;
import frc.frc_java9485.constants.VisionConsts.Cooprocessor;
import frc.frc_java9485.constants.VisionConsts.EstimateConsumer;
import frc.robot.subsystems.swerve.Swerve;

public class Vision extends SubsystemBase implements VisionIO {
    private Matrix<N4, N1> curStdDevs;
    private List<PhotonPipelineResult> currentResults;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public Vision(Cooprocessor cooprocessor) {
        poseEstimator = new PhotonPoseEstimator(VisionConsts.APRIL_TAG_FIELD_LAYOUT,
                                                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO,
                                                new Transform3d(0.1, 0.1, 0.1, Rotation3d.kZero));

        switch (cooprocessor) {
            case RASPBERRY:
                camera = new PhotonCamera(VisionConsts.RASPBERRY_CAMERA_NAME);
                poseEstimator.setRobotToCameraTransform(VisionConsts.RASPBERRY_ROBOT_TO_CAMERA);

                if (RobotConsts.CURRENT_ROBOT_MODE == RobotModes.SIM) {
                    cameraSim = new PhotonCameraSim(camera, VisionConsts.RASPBERRY_CAMERA_PROPS);
                    visionSim = new VisionSystemSim(VisionConsts.RASPBERRY_CAMERA_NAME);

                    visionSim.addAprilTags(VisionConsts.APRIL_TAG_FIELD_LAYOUT);
                    visionSim.addCamera(cameraSim, VisionConsts.RASPBERRY_ROBOT_TO_CAMERA);
                    cameraSim.enableRawStream(true);
                    cameraSim.enableProcessedStream(true);
                    cameraSim.enableDrawWireframe(true);
                }
                break;
            case LIMELIGHT:
                camera = new PhotonCamera(VisionConsts.LIMELIGHT_CAMERA_NAME);
                poseEstimator.setRobotToCameraTransform(VisionConsts.LIMELIGHT_ROBOT_TO_CAMERA);

                if (RobotConsts.CURRENT_ROBOT_MODE == RobotModes.SIM) {
                    visionSim = new VisionSystemSim(VisionConsts.LIMELIGHT_CAMERA_NAME);
                    cameraSim = new PhotonCameraSim(camera, VisionConsts.LIMELIGHT_CAMERA_PROPS);
                    visionSim.addCamera(cameraSim, VisionConsts.LIMELIGHT_ROBOT_TO_CAMERA);

                    visionSim.addAprilTags(VisionConsts.APRIL_TAG_FIELD_LAYOUT);

                    cameraSim.enableRawStream(true);
                    cameraSim.enableProcessedStream(true);
                    cameraSim.enableDrawWireframe(true);
                }
                break;
            default:
                camera = new PhotonCamera("");
                break;
        }
    }

    @Override
    public void estimatePose(EstimateConsumer estimateConsumer) {
        Optional<EstimatedRobotPose> estPose = Optional.empty();
        var currentResults = getCurrentResults();
        for(var result : currentResults) {
            estPose = poseEstimator.estimateCoprocMultiTagPose(result);

            if (estPose.isEmpty()) {
                estPose = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            updateEstimationStdDevs(estPose, result.getTargets());

            if (RobotConsts.CURRENT_ROBOT_MODE == RobotModes.SIM) {
                estPose.ifPresentOrElse(est -> {
                    getSimDebugField().getObject("VisionEstimation").
                    setPose(est.estimatedPose.toPose2d());
                }, () -> {
                    getSimDebugField().getObject("VisionEstimation").setPoses();
                });
            }

            estPose.ifPresent(est -> {
                var estStdDevs = getEstimationStdDevs();
                estimateConsumer.accept(est.estimatedPose, est.timestampSeconds, estStdDevs);
            });
        }
    }

    @Override
    public void periodic() {
        currentResults = camera.getAllUnreadResults();
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null) visionSim.update(Swerve.getInstance().getPose3d());
    }

    @Override
    public boolean hasTargets() {
        return currentResults.get(0).hasTargets();
    }

    @Override
    public double getTX() {
        return getBestTarget().yaw;
    }

    @Override
    public double getTY() {
        return getBestTarget().pitch;
    }

    @Override
    public double getTA() {
        return getBestTarget().area;
    }

    @Override
    public double getTRotation() {
        return getBestTarget().skew;
    }

    @Override
    public PhotonTrackedTarget getBestTarget() {
        return currentResults.get(0).getBestTarget();
    }

    @Override
    public List<PhotonTrackedTarget> getTargets() {
        return currentResults.get(0).getTargets();
    }

    @Override
    public Matrix<N4, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    private Field2d getSimDebugField() {
        return visionSim.getDebugField();
    }

    private List<PhotonPipelineResult> getCurrentResults() {
        if (currentResults != null && !currentResults.isEmpty()) return currentResults;
        return new ArrayList<PhotonPipelineResult>();
    }

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = VisionConsts.SINGLE_TAG_STD_DEVS;
        } else {
            var estStdDevs = VisionConsts.SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose
                    .get()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose
                    .getTranslation());
            }

            if (numTags == 0) {
                curStdDevs = VisionConsts.SINGLE_TAG_STD_DEVS;
            } else {
                avgDist /= numTags;
                if (numTags > 1) estStdDevs = VisionConsts.MULTI_TAG_STD_DEVS;
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
}
