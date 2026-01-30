package frc.robot.subsystems.vision;



import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import frc.frc_java9485.constants.VisionConsts.EstimateConsumer;

public interface VisionIO {
    public void estimatePose(EstimateConsumer estimateConsumer);
    public boolean hasTargets();
    public PhotonTrackedTarget getBestTarget();
    public List<PhotonTrackedTarget> getTargets();
    public Matrix<N4, N1> getEstimationStdDevs();

    public double getTX();
    public double getTY();
    public double getTA();
    public double getTRotation();
}
