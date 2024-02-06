package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;

public final class Constants {
  public static class Robot {
    public static final double k_width = 30; // Inches
    public static final double k_length = 30; // Inches

    public static final double k_bumperStart = 1; // Inches
    public static final double k_bumperHeight = 5; // Inches
  }

  public static class Auto {
    public static final double k_maxSpeed = Config.getData("autoMaxSpeed", 3.0); // 1 meters per second
    public static final double k_maxAcceleration = Config.getData("autoMaxAcceleration", 3.0);
  }

  public class SwerveDrive {
    // Drivetrain wheel offsets
    public static final double k_xDistance = 0.762; // 30 inches Forward/Backward
    public static final double k_yDistance = 0.762; // in meters! Side-to-Side

    public static final double k_xCenterDistance = k_xDistance / 2.0;
    public static final double k_yCenterDistance = k_yDistance / 2.0;

    public static final double k_maxSpeed = Config.getData("driveMaxSpeed", 3.0); // 3 meters per second
    public static final double k_maxAngularSpeed = Config.getData("maxAngularSpeed", Math.PI); // 1/2 rotation per second
    public static final double k_slowScaler = Config.getData("slowScaler", 0.2); // 20% reduction in speed
    public static final double k_boostScaler = Config.getData("boostScaler", 2.0); // 200% increase in speed

    public static final double k_wheelRadiusIn = 2.0; // 2 inches
    public static final double k_driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double k_turnGearRatio = 7.0 / 150.0;

    // Drivetrain drive motor constants
    public class Drive {
      public static final int k_FLMotorId = 6;
      public static final int k_FRMotorId = 8;
      public static final int k_BRMotorId = 10;
      public static final int k_BLMotorId = 12;

      public static final double k_P = Config.getData("driveP", 1.0);
      public static final double k_I = Config.getData("driveI", 0.0);
      public static final double k_D = Config.getData("driveD", 0.0);
      public static final double k_IZone = Config.getData("driveIZone", 0.0);
      public static final double k_FF = Config.getData("driveFF", 3.25);
    }

    // Drivetrain (turn) constants
    public class Turn {
      // Drivetrain turning offset constants
      public static final double k_FLOffset = Config.getData("FLModuleOffset", 0.0);
      public static final double k_FROffset = Config.getData("FRModuleOffset", 0.0);
      public static final double k_BROffset = Config.getData("BRModuleOffset", 0.0);
      public static final double k_BLOffset = Config.getData("BLModuleOffset", 0.0);

      public static final int k_FLAbsID = 0;
      public static final int k_FRAbsID = 1;
      public static final int k_BRAbsID = 2;
      public static final int k_BLAbsID = 3;

      public static final int k_FLMotorId = 5;
      public static final int k_FRMotorId = 7;
      public static final int k_BRMotorId = 9;
      public static final int k_BLMotorId = 11;

      // TODO: Tweak these as necessary
      public static final double k_turningP = Config.getData("turnP", 0.1);
      public static final double k_turningI = Config.getData("turnI", 0.0);
      public static final double k_turningD = Config.getData("turnD", 0.0);
      public static final double k_turningIZone = Config.getData("turnIZone", 0.0);
      public static final double k_turningFF = Config.getData("turnFF", 0.0);

      public static final double k_TurningMinOutput = Config.getData("turningMinOutput", -1.0);
      public static final double k_TurningMaxOutput = Config.getData("turningMaxOutput", 1.0);
    }

    public class AutoAim {
      public static final double k_P = Config.getData("autoAimP", 1.0);
      public static final double k_I = Config.getData("autoAimI", 0.0);
      public static final double k_D = Config.getData("autoAimD", 0.0);
    }
  }

  public class Intake {
    // TODO: Get intake motor id's
    public static final int k_pivotMotorID = 13;
    public static final int k_intakeMotorID = 14;

    public static final int k_pivotEncoderID = 4;
    public static final double k_pivotEncoderOffset = Config.getData("pivotEncoderOffset", 0);

    // TODO: get pivot angles
    public static final double k_groundPivotAngle = Config.getData("groundPivotAngle", 0.0);
    public static final double k_sourcePivotAngle = Config.getData("sourcePivotAngle", 0.0);
    public static final double k_ampPivotAngle = Config.getData("intakeAmpPivotAngle", 0.0);
    public static final double k_stowPivotAngle = Config.getData("stowPivotAngle", 0.0);

    // TODO: get intake speeds
    public static final double k_intakeSpeed = Config.getData("intakeSpeed", 0);
    public static final double k_ejectSpeed = Config.getData("ejectSpeed", 0);
    public static final double k_feedShooterSpeed = Config.getData("feedShooterSpeed", 0.0);

    // TODO: get intake pivot PID
    public static final double k_pivotMotorP = Config.getData("intakePivotMotorP", 1.0);
    public static final double k_pivotMotorI = Config.getData("intakePivotMotorI", 0.0);
    public static final double k_pivotMotorD = Config.getData("intakePivotMotorD", 0.0);
  }

  public class Shooter {
    public static final int k_pivotMotorID = 15;
    public static final int k_topMotorID = 16;
    public static final int k_bottomMotorID = 17;

    // TODO: Get shooter motor PID
    public static final double k_shooterMotorP = Config.getData("shooterMotorP", 1.0);
    public static final double k_shooterMotorI = Config.getData("shooterMotorI", 0.0);
    public static final double k_shooterMotorD = Config.getData("shooterMotorD", 0.0);

    // TODO: Check these
    public static final double k_shooterMinOutput = Config.getData("shooterMinOutput", 0.0);
    public static final double k_shooterMaxOutput = Config.getData("shooterMaxOutput", 1.0);

    public static final int k_pivotEncoderID = 5;

    // TODO: get shooter pivot PID
    public static final double k_pivotMotorP = Config.getData("shooterPivotMotorP", 1.0);
    public static final double k_pivotMotorI = Config.getData("shooterPivotMotorI", 0.0);
    public static final double k_pivotMotorD = Config.getData("shooterPivotMotorD", 0.0);

    // TODO: get shooter pivot setpoint angles
    public static final double k_lowPivotAngle = Config.getData("lowPivotAngle", 0.0);
    public static final double k_ampPivotAngle = Config.getData("shooterAmpPivotAngle", 0.0);
    public static final double k_speakerPivotAngle = Config.getData("speakerPivotAngle", 0.0);
  }

  public static class Climber {
    public static final int k_motorID = 18;

    public static final double k_velocity = Config.getData("climberVelocity", 0.0);
  }

  public static class Field {
    // All dimensions from Figure 5-16 in the manual
    public static final double k_lowGoalX = 22.75; // Inches
    public static final double k_lowGoalHeight = 34; // Inches

    public static final double k_highGoalX = 39.75; // Inches
    public static final double k_highGoalHeight = 46; // Inches

    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);

    public static final double k_autoAimThreshold = Config.getData("autoAimThreshold", 1.0); // in meters

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
