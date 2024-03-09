package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class Robot {
    public static final double k_width = 30; // Inches
    public static final double k_length = 30; // Inches

    public static final double k_bumperStart = 1; // Inches
    public static final double k_bumperHeight = 5; // Inches
  }

  public static class Auto {
    // Meters per second2
    // Needs to be more than the max robot speed, to allow for turning
    public static final double k_maxModuleSpeed = 5.0;
    public static final double k_maxAcceleration = 1.0; // NOT USED

    public class Timing {
      public static final double k_shootFeedTime = 0.5; // seconds
      public static final double k_shootRevTime = 0.75; // seconds
      public static final double k_intakeBounceTime = 0.2; // seconds
    }

    // TODO: Add left and right subwoofer starting poses
    public static final Pose2d k_redCenterPose2d = new Pose2d(15.19, 5.50,
        new Rotation2d(Units.degreesToRadians(180.0)));

    public static final Pose2d k_blueCenterPose2d = new Pose2d(1.27, 5.50,
        new Rotation2d(0));
  }

  public static class Vision {
    // Increase these numbers to trust your model's state estimates less.
    public static final double k_positionStdDevX = 0.1;
    public static final double k_positionStdDevY = 0.1;
    public static final double k_positionStdDevTheta = 10.0;

    // TODO: try this: 0.003, 0.003, 0.0002

    // Increase these numbers to trust global measurements from vision less.
    public static final double k_visionStdDevX = 4.0;
    public static final double k_visionStdDevY = 4.0;
    public static final double k_visionStdDevTheta = 400.0;

    public class Rotation {
      public static final double k_P = 5.0;
      public static final double k_I = 0.0;
      public static final double k_D = 0.0;
    }
  }

  public class SwerveDrive {
    // Drivetrain wheel offsets
    public static final double k_xDistance = 0.762; // 30 inches Forward/Backward
    public static final double k_yDistance = 0.762; // in meters! Side-to-Side

    public static final double k_xCenterDistance = k_xDistance / 2.0;
    public static final double k_yCenterDistance = k_yDistance / 2.0;

    // Max speeds
    public static final double k_maxSpeed = 2.5; // Meters per second
    public static final double k_maxBoostSpeed = 5.0; // Meters per second
    public static final double k_maxAngularSpeed = Math.PI * 2.0; // Radians per second

    // Max acceleration
    public static final double k_maxLinearAcceleration = 12.0; // Meters per second squared
    public static final double k_maxAngularAcceleration = Math.PI * 8.0; // Radians per second squared

    public static final double k_slowScaler = 0.2; // % reduction in speed

    public static final double k_wheelRadiusIn = 2.0; // inches
    public static final double k_driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public static final double k_turnGearRatio = 7.0 / 150.0;

    // Drivetrain drive motor constants
    public class Drive {
      public static final int k_FLMotorId = 6;
      public static final int k_FRMotorId = 8;
      public static final int k_BRMotorId = 10;
      public static final int k_BLMotorId = 12;

      public static final int k_currentLimit = 40;

      public static final double k_P = 0.00079049;
      public static final double k_I = 0.0;
      public static final double k_D = 0.0;
      public static final double k_IZone = 0.0;

      public static final double k_FFS = 0.21465;
      public static final double k_FFV = 2.3914;
      public static final double k_FFA = 0.36664;
    }

    // Drivetrain (turn) constants
    public class Turn {
      // Drivetrain turning offset constants
      public static final double k_FLOffset = 0.159;
      public static final double k_FROffset = 0.163;
      public static final double k_BROffset = 0.832;
      public static final double k_BLOffset = 0.157;

      public static final int k_FLAbsId = 0;
      public static final int k_FRAbsId = 1;
      public static final int k_BRAbsId = 2;
      public static final int k_BLAbsId = 3;

      public static final int k_currentLimit = 25;

      public static final int k_FLMotorId = 5;
      public static final int k_FRMotorId = 7;
      public static final int k_BRMotorId = 9;
      public static final int k_BLMotorId = 11;

      public static final double k_P = 5.6906;
      public static final double k_I = 0.0;
      public static final double k_D = 0.1976;
      public static final double k_IZone = 0.0;

      public static final double k_FFS = 0.29745;
      public static final double k_FFV = 0.43892; // Not used
      public static final double k_FFA = 0.048573; // Not used

      public static final double k_TurningMinOutput = -1.0;
      public static final double k_TurningMaxOutput = 1.0;
    }
  }

  public class AutoAim {
    public class Translation {
      public static final double k_P = 10.0;
      public static final double k_I = 0.0;
      public static final double k_D = 0.0;
    }

    public class Rotation {
      public static final double k_P = 3.0;
      public static final double k_I = 0.0;
      public static final double k_D = 0.0;
    }

    public static final double k_autoAimDistanceThreshold = 1; // in meters
    public static final double k_autoAimAngleTolerance = Units.degreesToRadians(3.0);

    // Max speed
    public static final double k_maxSpeed = 3.0; // Meters per second
    public static final double k_maxLinearAcceleration = 1.5; // Meters per second squared

    // Max rotation
    public static final double k_maxAngularSpeed = Math.PI * 2 * 1.5; // Radians per second (540 degrees per second)
    public static final double k_maxAngularAcceleration = Math.PI * 2 * 2; // Radians per second squared (720 degrees
                                                                           // per second squared)

    // Use these values as overrides in DriveTrajectoryTask
    // These are the current PathPlanner values
    public static final PathConstraints k_pathConstraints = new PathConstraints(
        k_maxSpeed,
        k_maxLinearAcceleration,
        k_maxAngularSpeed,
        k_maxAngularAcceleration);

    public static final PIDController translationPIDController = new PIDController(
        Translation.k_P,
        Translation.k_I,
        Translation.k_D);

    public static final ProfiledPIDController rotationPIDController = new ProfiledPIDController(
        Rotation.k_P,
        Rotation.k_I,
        Rotation.k_D,
        new Constraints(k_maxSpeed, k_maxLinearAcceleration));

    public static final HolonomicDriveController k_autoTargetController = new HolonomicDriveController(
        translationPIDController,
        translationPIDController,
        rotationPIDController);
  }

  public class Intake {
    public static final int k_pivotMotorId = 13;
    public static final int k_intakeMotorId = 14;

    public static final double k_pivotMotorMaxOutput = 0.2;

    public static final int k_pivotEncoderId = 4;

    public static final double k_stowPivotAngle = 267.0;
    public static final double k_groundPivotAngle = 41.0;
    public static final double k_sourcePivotAngle = 180.0;
    // public static final double k_sourcePivotAngle = k_stowPivotAngle;
    public static final double k_ejectPivotAngle = 110.0;
    public static final double k_ampPivotAngle = 181.0;

    public static final double k_intakeSpeed = 0.4;
    public static final double k_ejectSpeed = -0.525;
    public static final double k_feedShooterSpeed = -0.5;

    public static final double k_pivotMotorP = 0.035;
    public static final double k_pivotMotorI = 0.0;
    public static final double k_pivotMotorD = 0.0;
    public static final double k_pivotMotorIZone = 0.0;

    public static final double k_pivotMotorKS = 0.0; // volts
    public static final double k_pivotMotorKG = 0.03; // volts
    public static final double k_pivotMotorKV = 0.244; // volts*sec / rad
    public static final double k_pivotMotorKA = 0.0; // 0.02; // volts*sec^2 / rad

    public static final double k_length = 14.319626; // In inches
    public static final double k_mass = 5.8967; // In kg

    public static final double k_pivotHeight = 5.75;

    public static final double k_minAngle = 0.0;
    public static final double k_maxAngle = 0.0;
    public static final double k_startingAngle = 0.0;

    public static final double k_distanceFromCenter = 12.5;
    public static final int k_sensorThreshold = 1000; // 1,000 is "in", 2000 is max

    public static final double k_pivotOffset = 74.5;

    public static double k_maxVelocity = 690.0;
    public static double k_maxAcceleration = 1380.0;
  }

  public static class Simulation {
    public static final double k_width = 150.0; // Inches
    public static final double k_height = 100.0; // Inches
  }

  public class Shooter {
    public static final double k_maxRPM = 6000.0; // but that's just a theory

    public static final int k_pivotMotorId = 15;
    public static final int k_topMotorId = 16;
    public static final int k_bottomMotorId = 17;

    public static final double k_shooterMotorP = 0.00075;
    public static final double k_shooterMotorI = 0.0000008;
    public static final double k_shooterMotorD = 0.0;
    public static final double k_shooterMotorFF = 0.00015;

    public static final double k_shooterMinOutput = 0.0;
    public static final double k_shooterMaxOutput = 1.0;

    public static final int k_pivotEncoderId = 5;

    public static final double k_pivotMotorP = 0.4;
    public static final double k_pivotMotorI = 0.0;
    public static final double k_pivotMotorD = 0.0;
    public static final double k_pivotMotorIZone = 0.0;

    public static final double k_ampPivotAngle = 0.0; // TODO: get amp pivot angle
    public static final double k_wingPivotAngle = 26.0;
    public static final double k_subwooferPivotAngle = 62.0;
    public static final double k_podiumPivotAngle = 46.0;

    public static final double k_length = 11.94335; // in inches
    public static final double k_mass = 5.44311; // in kg

    public static final double k_pivotHeight = 4.0;

    public static final double k_minAngle = 20.0;
    public static final double k_maxAngle = 65.0;
    public static final double k_startingAngle = 0.0;

    public static final double k_distanceFromCenter = 4.0008;

    // Offset, -90, since that's where we want to start
    public static final double k_absPivotOffset = 208.587 - 90;

    static double threadsPerInch = 10.0;
    static double helices = 2.0;
    public static final double k_rotationsPerInch = threadsPerInch / helices;

    static double maxPivotExtensionInches = 15.5;
    public static final double k_relRotationsToMaxExtension = maxPivotExtensionInches * k_rotationsPerInch;

    // Shooter angle lookup table, V2
    // Rotations from plate at 90 degrees (aka straight up)
    // Plate, Rel
    // 20 deg, 33.162384
    // 45 deg, 20.878754
    // 60 deg, 12.488831
    // 90 deg, 0.0
  }

  public static class Climber {
    public static final int k_leftMotorID = 19;
    public static final int k_rightMotorID = 18;

    public static final double k_raiseSpeed = 600.0; // RPM
    public static final double k_lowerSpeed = -600.0; // RPM

    public static final double k_gearRatio = 1.0 / 16.0;

    public static final double k_P = 0.001;
    public static final double k_I = 0.0;
    public static final double k_D = 0.0;

    public static final double k_minOutput = -0.5;
    public static final double k_maxOutput = 0.5;
  }

  public static class Field {
    public static final double k_width = Units.feetToMeters(54.0);
    public static final double k_length = Units.feetToMeters(27.0);

    public static final double k_ampBottom = 26;
    public static final double k_ampTop = 44;

    public static final double k_speakerBottom = 78;
    public static final double k_speakerTop = 82.875;
    public static final double k_speakerAngle = 14.0;

    // TODO: Maybe get these from AprilTags?
    private static final double speakerHeight = 2.032; // Meters
    public static final Pose3d k_redSpeakerPose = new Pose3d(16.579342, 5.547868, speakerHeight, new Rotation3d());
    public static final Pose3d k_blueSpeakerPose = new Pose3d(-0.0381, 5.547868, speakerHeight, new Rotation3d());
  }

  public static class Limelight {
    public static class AprilTags {
      public static final int k_redCenterSpeaker = 4;
      public static final int k_blueCenterSpeaker = 7;
    }
  }
}
