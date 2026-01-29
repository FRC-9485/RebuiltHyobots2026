package frc.frc_java9485.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.frc_java9485.constants.FieldConsts;

public class AllianceFlip {
  private AllianceFlip() {}

  public static Pose2d flipPose2dToRed(Pose2d bluePose) {
    return new Pose2d(
        FieldConsts.FIELD_LENGTH_METERS - bluePose.getX(),
        bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg));
  }

  public static Pose2d flipPose2dToRedAndNormalize(Pose2d bluePose) {
    return new Pose2d(
        FieldConsts.FIELD_LENGTH_METERS - bluePose.getX(),
        bluePose.getY(),
        bluePose.getRotation().plus(Rotation2d.k180deg).minus(Rotation2d.fromDegrees(360)));
  }

  public static Translation2d flipTranslation2dToRed(Translation2d blueTranslation) {
    return new Translation2d(
        FieldConsts.FIELD_LENGTH_METERS - blueTranslation.getX(), blueTranslation.getY());
  }
}
