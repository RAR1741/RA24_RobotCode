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
    public static final double k_maxSpeed = 3.0; // 1 meters per second
    public static final double k_maxAcceleration = 3.0;

    public class PIDConstants {
      public class Translation {
        public static final double k_P = 0.5;
        public static final double k_I = 0.0;
        public static final double k_D = 0.0;
      }

      public class Rotation {
        public static final double k_P = 0.7;
        public static final double k_I = 0.0;
        public static final double k_D = 0.0;
      }
    }
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
      public static final double k_FLOffset = 0.156013;
      public static final double k_FROffset = 0.156604;
      public static final double k_BROffset = 0.820736;
      public static final double k_BLOffset = 0.161767;

      public static final int k_FLAbsID = 0;
      public static final int k_FRAbsID = 1;
      public static final int k_BRAbsID = 2;
      public static final int k_BLAbsID = 3;

      public static final int k_FLMotorId = 5;
      public static final int k_FRMotorId = 7;
      public static final int k_BRMotorId = 9;
      public static final int k_BLMotorId = 11;

      public static final double k_turningP = 0.1;
      public static final double k_turningI = 0.0;
      public static final double k_turningD = 0.0;
      public static final double k_turningIZone = 0.0;
      public static final double k_turningFF = 0.0;

      public static final double k_TurningMinOutput = -1.0;
      public static final double k_TurningMaxOutput = 1.0;
    }

    public class AutoAim {
      public static final double k_P = 1.0;
      public static final double k_I = 0.0;
      public static final double k_D = 0.0;
    }
  }

  public class Intake {
    // TODO: Get intake motor IDs
    public static final int k_pivotMotorId = 13;
    public static final int k_intakeMotorId = 14;

    public static final double k_pivotMotorMaxOutput = 0.2;

    public static final int k_pivotEncoderId = 4;

    // TODO: get pivot angles
    public static final double k_stowPivotAngle = 267.0;
    public static final double k_groundPivotAngle = 41.0;
    public static final double k_sourcePivotAngle = 180.0;
    // public static final double k_sourcePivotAngle = k_stowPivotAngle;
    public static final double k_ampPivotAngle = k_stowPivotAngle;

    // TODO: get intake speeds
    public static final double k_intakeSpeed = 0.7;
    public static final double k_ejectSpeed = -0.45;
    public static final double k_feedShooterSpeed = -0.5;

    // TODO: get intake pivot PID
    public static final double k_pivotMotorP = 0.035;
    public static final double k_pivotMotorI = 0.0;
    public static final double k_pivotMotorD = 0.0;
    public static final double k_pivotMotorIZone = 0.0;

    // TODO Get values
    public static final double k_length = 14.319626; // In inches
    public static final double k_mass = 5.8967; // In kg

    public static final double k_pivotHeight = 5.75;

    public static final double k_minAngle = 0.0;
    public static final double k_maxAngle = 0.0;
    public static final double k_startingAngle = 0.0;

    public static final double k_distanceFromCenter = 12.5;
  }

  public static class Simulation {
    // TODO Get values
    public static final double k_width = 150; // Inches
    public static final double k_height = 100; // Inches
  }

  public class Shooter {
    public static final int k_pivotMotorId = 15;
    public static final int k_topMotorId = 16;
    public static final int k_bottomMotorId = 17;

    // TODO: Get shooter motor PID
    public static final double k_shooterMotorP = 0.0;
    public static final double k_shooterMotorI = 0.0;
    public static final double k_shooterMotorD = 0.0;

    // TODO: Check these
    public static final double k_shooterMinOutput = 0.0;
    public static final double k_shooterMaxOutput = 1.0;

    public static final int k_pivotEncoderId = 5;

    // TODO: get shooter pivot PID
    public static final double k_pivotMotorP = 0.0;
    public static final double k_pivotMotorI = 0.0;
    public static final double k_pivotMotorD = 0.0;
    public static final double k_pivotMotorIZone = 0.0;

    // TODO: get shooter pivot setpoint angles
    public static final double k_lowPivotAngle = 0.0;
    public static final double k_ampPivotAngle = 0.0;
    public static final double k_speakerPivotAngle = 0.0;

    // TODO get values
    public static final double k_length = 11.94335; // in inches
    public static final double k_mass = 5.44311; // in kg

    public static final double k_pivotHeight = 4.0;

    public static final double k_minAngle = 18.0;
    public static final double k_maxAngle = 63.75;
    public static final double k_startingAngle = 0.0;

    public static final double k_distanceFromCenter = 4.0008;
  }

  public static class Climber {
    public static final int k_leftMotorID = 18;
    public static final int k_rightMotorID = 19;

    public static final double k_velocity = 0.0;

    public class Setpoints {
      // TODO: get climber setpoints
      public static final double k_fullyExtended = 0.0;
      public static final double k_fullyRetracted = 0.0;

    }
  }

  public static class Field {
    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);

    public static final double k_ampBottom = 26;
    public static final double k_ampTop = 44;

    public static final double k_speakerBottom = 78;
    public static final double k_speakerTop = 82.875;
    public static final double k_speakerAngle = 14.0;

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
