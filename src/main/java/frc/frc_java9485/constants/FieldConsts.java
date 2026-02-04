package frc.frc_java9485.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.frc_java9485.utils.AllianceFlip;

public final class FieldConsts {
    public static final double FIELD_WIDTH_METERS = 8.21; // X
    public static final double FIELD_LENGTH_METERS = 16.54; // Y

    public static final double FUEL_DIAMETER = 0.15; // CM -> M
    public static final double FUEL_RADIUS = FUEL_DIAMETER / 2; // CM -> M
    public static final double FUEL_SPACING = 0.15; // CM -> M

    public static final Pose2d BLUE_LEFT_START_POSE =
        new Pose2d(3.52, 7.44, Rotation2d.fromDegrees(0));

    public static final Pose2d BLUE_CENTER_START_POSE =
        new Pose2d(3.14, 3.96, Rotation2d.fromDegrees(0));

    public static final Pose2d BLUE_RIGHT_START_POSE =
        new Pose2d(3.54, 0.66, Rotation2d.fromDegrees(0));

    public static final Pose2d RED_LEFT_START_POSE =
        AllianceFlip.flipPose2dToRed(BLUE_LEFT_START_POSE);

    public static final Pose2d RED_CENTER_START_POSE =
        AllianceFlip.flipPose2dToRed(BLUE_CENTER_START_POSE);

    public static final Pose2d RED_RIGHT_START_POSE =
        AllianceFlip.flipPose2dToRed(BLUE_RIGHT_START_POSE);

    public static final Pose2d FIELD_CENTER_POSE =
        new Pose2d(FIELD_LENGTH_METERS / 2, FIELD_WIDTH_METERS / 2, Rotation2d.fromDegrees(0));

    public static final Pose2d BLUE_OUTPOST_MIN =
        new Pose2d(0.368, 0.369, Rotation2d.fromDegrees(0));

    public static final Pose2d BLUE_OUTPOST_MAX =
        new Pose2d(0.368, 1.127, Rotation2d.fromDegrees(0));
  }
