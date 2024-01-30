package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class Robot {
    public static final double k_width = 30; // Inches
    public static final double k_length = 30; // Inches

    public static final double k_bumperStart = 1; // Inches
    public static final double k_bumperHeight = 5; // Inches
  }

  public static class Auto {
    public static final double k_maxSpeed = 1; // 1 meters per second
    public static final double k_maxAcceleration = 0.5;
  }

  public class SwerveDrive {
    // Drivetrain wheel offsets
    public static final double k_xDistance = 0.762; // 30 inches Forward/Backward
    public static final double k_yDistance = 0.762; // in meters! Side-to-Side

    public static final double k_xCenterDistance = k_xDistance / 2.0;
    public static final double k_yCenterDistance = k_yDistance / 2.0;

    public static final double k_maxSpeed = 3.0; // 3 meters per second
    public static final double k_maxAngularSpeed = Math.PI; // 1/2 rotation per second
    public static final double k_slowScaler = 0.2; // 20% reduction in speed
    public static final double k_boostScaler = 2.0; // 200% increase in speed

    public static final double k_wheelRadiusIn = 2.0; // 2 inches
    public static final double k_driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double k_turnGearRatio = 7.0 / 150.0;

    // Drivetrain drive motor constants
    public class Drive {
      public static final int k_FLMotorId = 6;
      public static final int k_FRMotorId = 8;
      public static final int k_BRMotorId = 10;
      public static final int k_BLMotorId = 12;

      public static final double k_P = 0.0;
      public static final double k_I = 0.0;
      public static final double k_D = 0.0;
      public static final double k_IZone = 0.0;
      public static final double k_FF = 3.25;
    }

    // Drivetrain (turn) constants
    public class Turn {
      // Drivetrain turning offset constants
      public static final double k_FLOffset = 0.157329;
      public static final double k_FROffset = 0.345974;
      public static final double k_BROffset = 0.823048;
      public static final double k_BLOffset = 0.329460;

      public static final int k_FLAbsID = 0;
      public static final int k_FRAbsID = 1;
      public static final int k_BRAbsID = 2;
      public static final int k_BLAbsID = 3;

      public static final int k_FLMotorId = 5;
      public static final int k_FRMotorId = 7;
      public static final int k_BRMotorId = 9;
      public static final int k_BLMotorId = 11;

      // TODO: Tweak these as necessary
      public static final double k_turningP = 0.1;
      public static final double k_turningI = 0.0;
      public static final double k_turningD = 0.0;
      public static final double k_turningIZone = 0;
      public static final double k_turningFF = 0;

      public static final int k_TurningMinOutput = -1;
      public static final int k_TurningMaxOutput = 1;
    }

    public class AutoAim {
      public static final double k_P = 1;
      public static final double k_I = 0;
      public static final double k_D = 0;
    }
  }

  public class Intake {
    public static final int k_pivotMotorID = 98;
    public static final int k_intakeMotorID = 99;

    public static final int k_pivotEncoderId = 0;
    public static final double k_pivotEncoderOffset = 0.0;

    // TODO: get pivot angles
    public static final double k_groundPivotAngle = 0.0;
    public static final double k_sourcePivotAngle = 0.0;
    public static final double k_ampPivotAngle = 0.0;
    public static final double k_stowPivotAngle = 0.0;

    // TODO: get intake speeds
    public static double k_intakeSpeed = 0.0;
    public static double k_ejectSpeed = 0.0;
    public static double k_feedShooterSpeed = 0.0;

    public static final double k_pivotMotorP = 0.0;
    public static final double k_pivotMotorI = 0.0;
    public static final double k_pivotMotorD = 0.0;
  }

  public static class Field {
    // All dimensions from Figure 5-16 in the manual
    public static final double k_lowGoalX = 22.75; // Inches
    public static final double k_lowGoalHeight = 34; // Inches

    public static final double k_highGoalX = 39.75; // Inches
    public static final double k_highGoalHeight = 46; // Inches

    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);

    public static final double k_autoAimThreshold = 1; // in meters

    // TODO: Make sure the robot uses the same coordinate system
    public static final Pose2d k_redSpeakerPose = new Pose2d(16.579342, 5.547868, new Rotation2d(0));
    public static final Pose2d k_blueSpeakerPose = new Pose2d(-0.0381, 5.547868, new Rotation2d(0));
  }

  public static class Limelight {
    public static class AprilTags {
      public static final int k_redCenterSpeaker = 4;
      public static final int k_blueCenterSpeaker = 7;
    }
  }
}
