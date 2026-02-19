package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.frc_java9485.constants.RobotConsts;
import frc.frc_java9485.constants.RobotConsts.RobotModes;
import frc.frc_java9485.constants.VisionConsts.Cooprocessor;
import frc.frc_java9485.constants.VisionConsts.EstimateConsumer;
import frc.robot.subsystems.swerve.Swerve;
import static frc.frc_java9485.constants.VisionConsts.*;

public class Vision extends SubsystemBase implements VisionIO {
    private Matrix<N4, N1> curStdDevs;
    private List<PhotonPipelineResult> currentResults;

    private final PhotonCamera camera;

    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private Cooprocessor currentCooprocessor;

    public Vision(Cooprocessor cooprocessor) {
        switch (cooprocessor) {
            case RASPBERRY:
                camera = new PhotonCamera(RASPBERRY_CAMERA_NAME);

                if (RobotConsts.CURRENT_ROBOT_MODE == RobotModes.SIM) {
                    cameraSim = new PhotonCameraSim(camera, RASPBERRY_CAMERA_PROPS);
                    visionSim = new VisionSystemSim(RASPBERRY_CAMERA_NAME);

                    visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
                    visionSim.addCamera(cameraSim, RASPBERRY_ROBOT_TO_CAMERA);
                    cameraSim.enableRawStream(true);
                    cameraSim.enableProcessedStream(true);
                    cameraSim.enableDrawWireframe(true);
                }
                break;
            case LIMELIGHT:
                camera = new PhotonCamera(LIMELIGHT_CAMERA_NAME);

                if (RobotConsts.CURRENT_ROBOT_MODE == RobotModes.SIM) {
                    visionSim = new VisionSystemSim(LIMELIGHT_CAMERA_NAME);
                    cameraSim = new PhotonCameraSim(camera, LIMELIGHT_CAMERA_PROPS);
                    visionSim.addCamera(cameraSim, LIMELIGHT_ROBOT_TO_CAMERA);

                    visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);

                    cameraSim.enableRawStream(true);
                    cameraSim.enableProcessedStream(true);
                    cameraSim.enableDrawWireframe(true);
                }
                break;
            default:
                camera = new PhotonCamera("");
                break;
        }

        currentCooprocessor = cooprocessor;
    }

    @Override
    public void estimatePose(EstimateConsumer estimateConsumer) {
        Optional<EstimatedRobotPose> estPose = Optional.empty();
        estPose.ifPresent(est -> {
            updateEstimationStdDevs(estPose, getTargets());
            estimateConsumer.accept(est.estimatedPose, est.timestampSeconds, curStdDevs);
        });
    }

    @Override
    public void periodic() {
        currentResults = camera.getAllUnreadResults();
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null) visionSim.update(Swerve.getInstance().getPose3d());

        switch (currentCooprocessor) {
            case LIMELIGHT:
                SmartDashboard.putData("PhotonLimelight/Debug Field", getSimDebugField());
                break;
            case RASPBERRY:
                SmartDashboard.putData("PhotonRaspberry/Debug Field", getSimDebugField());
                break;
        }
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

    @Override
    public List<PhotonPipelineResult> getCurrentResults() {
        if (currentResults != null && !currentResults.isEmpty()) return currentResults;
        return new ArrayList<PhotonPipelineResult>();
    }

    private Field2d getSimDebugField() {
        return visionSim.getDebugField();
    }

    private void updateEstimationStdDevs(
        Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            curStdDevs = SINGLE_TAG_STD_DEVS;
        } else {
            var estStdDevs = SINGLE_TAG_STD_DEVS;
            int numTags = 0;
            double avgDist = 0;

            for (var tgt : targets) {
                var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist += tagPose
                    .get()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose
                    .getTranslation());
            }

            if (numTags == 0) {
                curStdDevs = SINGLE_TAG_STD_DEVS;
            } else {
                avgDist /= numTags;
                if (numTags > 1) estStdDevs = MULTI_TAG_STD_DEVS;
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }
}
