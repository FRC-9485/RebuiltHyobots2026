package frc.robot.subsystems.vision;

import frc.frc_java9485.constants.VisionConsts.Cooprocessor;

public class VisionInstances {
    private static Vision limelight;
    private static Vision raspberry;

    public static Vision getLimelightInstance() {
        if (limelight == null) limelight = new Vision(Cooprocessor.LIMELIGHT);
        return limelight;
    }

    public static Vision getRaspberryInstance() {
        if (raspberry == null) raspberry = new Vision(Cooprocessor.RASPBERRY);
        return raspberry;
    }
}
