package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class AllianceHelpers {
  private static SwerveDrive m_swerve = SwerveDrive.getInstance();

  public static Pose3d getAllianceSpeakerPose3d() {
    if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
      return RobotConstants.config.field().k_blueSpeakerPose;
    }
    return RobotConstants.config.field().k_redSpeakerPose;
  }

  public static Pose2d getAllianceSpeakerPose2d() {
    return getAllianceSpeakerPose3d().toPose2d();
  }

  public static Rotation2d getAllianceSpeakerRotationTarget() {
    return new Rotation2d(
        m_swerve.getPose().getTranslation().getX()
            - getAllianceSpeakerPose2d().getTranslation().getX(),
        m_swerve.getPose().getTranslation().getY()
            - getAllianceSpeakerPose2d().getTranslation().getY());
  }

  public static Pose2d getAlliancePassPose2d() {
    if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
      return RobotConstants.config.field().k_bluePassPose;
    }
    return RobotConstants.config.field().k_redPassPose;
  }

  public static Rotation2d getAllianceAmpRotation() {
    return new Rotation2d(-Math.PI / 2);
  }

  public static Rotation2d allianceForwardRotation() {
    if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
      return new Rotation2d(Math.PI);
    }
    return new Rotation2d(0.0);
  }

  public static Rotation2d getAlliancePassRotation() {
    double passAngle = RobotConstants.config.field().k_passAngle;

    if (!DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() == Alliance.Blue) {
      return Rotation2d.fromDegrees(-passAngle);
    }
    return Rotation2d.fromDegrees(180.0 + passAngle);
  }
}
