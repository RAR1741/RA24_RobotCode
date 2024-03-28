package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.ApolloConstants.Field;

public class AllianceHelpers {
  public static Pose3d getAllianceSpeakerPose3d() {
    if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
      return Field.k_blueSpeakerPose;
    }
    return Field.k_redSpeakerPose;
  }

  public static Pose2d getAllianceSpeakerPose2d() {
    return getAllianceSpeakerPose3d().toPose2d();
  }

  public static Pose2d getAlliancePassPose2d() {
    if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
      return Field.k_bluePassPose;
    }
    return Field.k_redPassPose;
  }

  public static Rotation2d getAllianceAmpRotation() {
    if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
      return new Rotation2d(-Math.PI / 2);
    }
    return new Rotation2d(Math.PI / 2);
  }

  public static Rotation2d allianceForwardRotation() {
    if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
      return new Rotation2d(Math.PI);
    }
    return new Rotation2d(0.0);
  }

  public static Rotation2d getAlliancePassRotation() {
    return new Rotation2d(Math.toRadians(12.0));
  }
}
