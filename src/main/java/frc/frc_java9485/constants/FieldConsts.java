package frc.frc_java9485.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
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

    public static final Distance FIELD_LENGTH = Inches.of(650.12);
    public static final Distance FIELD_WIDTH = Inches.of(316.64);

    public static final Translation3d HUB_BLUE =
                new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
    public static final Translation3d HUB_RED =
                new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));

    public static final Distance FUNNEL_RADIUS = Inches.of(24);
    public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

    public static final Translation3d PASSING_SPOT_LEFT = new Translation3d(
            Inches.of(90), FIELD_WIDTH.div(2).plus(Inches.of(85)), Inches.zero());
    public static final Translation3d PASSING_SPOT_CENTER =
            new Translation3d(Inches.of(90), FIELD_WIDTH.div(2), Inches.zero());
    public static final Translation3d PASSING_SPOT_RIGHT = new Translation3d(
            Inches.of(90), FIELD_WIDTH.div(2).minus(Inches.of(85)), Inches.zero());

    public static final Distance ALLIANCE_ZONE = Inches.of(156.06);
}
