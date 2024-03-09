package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Field;

public class AllianceHelpers {
  public static Pose3d getAllianceSpeakerPose3d() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return Field.k_blueSpeakerPose;
    } else {
      return Field.k_redSpeakerPose;
    }
  }

  public static Pose2d getAllianceSpeakerPose2d() {
    return getAllianceSpeakerPose3d().toPose2d();
  }
}
