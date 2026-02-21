package frc.robot.subsystems.vision;

import frc.frc_java9485.constants.VisionConsts.Cooprocessor;

public class VisionInstances {
    private static VisionSubsystem limelight;
    private static VisionSubsystem raspberry;

    public static VisionSubsystem getLimelightInstance() {
        if (limelight == null) limelight = new VisionSubsystem(Cooprocessor.LIMELIGHT);
        return limelight;
    }

    public static VisionSubsystem getRaspberryInstance() {
        if (raspberry == null) raspberry = new VisionSubsystem(Cooprocessor.RASPBERRY);
        return raspberry;
    }
}
