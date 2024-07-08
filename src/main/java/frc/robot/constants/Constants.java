package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
  public RobotConstants Robot = new RobotConstants();
  public ClimberConstants Climber = new ClimberConstants();
  public IntakeConstants Intake = new IntakeConstants();
  public SimulationConstants Simulation = new SimulationConstants();
  public FieldConstants Field = new FieldConstants();
  public AutoConstants Auto = new AutoConstants();
  public VisionConstants Vision = new VisionConstants();
  public SwerveDriveConstants SwerveDrive = new SwerveDriveConstants();
  public AutoAimConstants AutoAim = new AutoAimConstants();
  public ShooterConstants Shooter = new ShooterConstants();

  public static class RobotConstants {
    public double k_width = 30.0; // Inches
    public double k_length = 30.0; // Inches

    public double k_bumperStart = 1.0; // Inches
    public double k_bumperHeight = 5.0; // Inches
  }

  public static class ClimberConstants {
    public int k_leftMotorID = 19;
    public int k_rightMotorID = 18;

    public double k_raiseSpeed = 1200.0; // RPM
    public double k_lowerSpeed = -1200.0; // RPM

    public double k_gearRatio = 1.0 / 16.0;

    public double k_P = 0.001;
    public double k_I = 0.0;
    public double k_D = 0.0;

    public double k_minOutput = -0.5;
    public double k_maxOutput = 0.5;
  }

  public static class IntakeConstants {
    public int k_pivotMotorId = 13;
    public int k_intakeMotorId = 14;

    public double k_pivotMotorMaxOutput = 0.2;

    public int k_pivotEncoderId = 4;

    public double k_stowPivotAngle = 256.5;
    public double k_groundPivotAngle = 40.4;
    public double k_sourcePivotAngle = 180.0;
    public double k_ejectPivotAngle = 100.0;
    public double k_ampPivotAngle = 100.0;

    public double k_intakeSpeed = 0.8;
    public double k_ejectSpeed = -0.525;
    public double k_feedShooterSpeed = -1.0;

    public double k_pivotMotorP = 0.045;
    public double k_pivotMotorI = 0.0;
    public double k_pivotMotorD = 0.0;
    public double k_pivotMotorIZone = 0.0;

    public double k_pivotMotorKS = 0.0; // volts
    public double k_pivotMotorKG = 0.03; // volts
    public double k_pivotMotorKV = 0.244; // volts*sec / rad
    public double k_pivotMotorKA = 0.0; // 0.02; // volts*sec^2 / rad

    public double k_length = 14.319626; // In inches
    public double k_mass = 5.8967; // In kg

    public double k_pivotHeight = 5.75;

    public double k_minAngle = 0.0;
    public double k_maxAngle = 0.0;
    public double k_startingAngle = 0.0;

    public double k_distanceFromCenter = 12.5;
    public int k_sensorThreshold = 1000; // 1,000 is "in", 2000 is max

    public double k_pivotOffset = 127.5;

    public double k_maxVelocity = 690.0;
    public double k_maxAcceleration = 1380.0;
  }

  public static class SimulationConstants {
    public double k_width = 150.0; // Inches
    public double k_height = 100.0; // Inches
  }

  public static class FieldConstants {
    public double k_width = Units.feetToMeters(54.0);
    public double k_length = Units.feetToMeters(27.0);

    public double k_ampBottom = 26;
    public double k_ampTop = 44;

    public double k_speakerBottom = 78;
    public double k_speakerTop = 82.875;
    public double k_speakerAngle = 14.0;

    // TODO: Maybe get these from AprilTags?
    private double speakerHeight = 2.032; // Meters
    public Pose3d k_redSpeakerPose = new Pose3d(16.426942, 5.547868, speakerHeight, new Rotation3d());
    public Pose3d k_blueSpeakerPose = new Pose3d(0.1524, 5.547868, speakerHeight, new Rotation3d());

    public Pose2d k_redPassPose = new Pose2d(14.71, 6.0, new Rotation2d());
    public Pose2d k_bluePassPose = new Pose2d(1.75, 6.0, new Rotation2d());
    public double k_passAngle = 23.75;
  }

  public static class ShooterConstants {
    public double k_maxRPM; // but that's just a theory
    public double k_passRPM;
    public double k_ampSpeed = 800.0;
    public double k_trapSpeed = 6000.0;

    public double k_demoBigRPM = 6000.0;
    public double k_demoLittleRPM = 2000.0;

    public int k_shooterSpeedTolerance = 100; // 1,000 is "in", 2000 is max

    public int k_pivotMotorId = 15;
    public int k_topMotorId = 16;
    public int k_bottomMotorId = 17;

    public ShootConstants ShootPID = new ShootConstants();

    public class ShootConstants {
      public double k_shooterMotorP;
      public double k_shooterMotorI;
      public double k_shooterMotorD;
      public double k_shooterMotorFF;
    }

    public AmpConstants AmpPID = new AmpConstants();

    public class AmpConstants {
      public double k_shooterMotorP;
      public double k_shooterMotorI;
      public double k_shooterMotorD;
      public double k_shooterMotorFF;
    }

    public TrapConstants TrapPID = new TrapConstants();

    public class TrapConstants {
      public double k_shooterMotorP = 0.0009300;
      public double k_shooterMotorI = 0.00000008;
      public double k_shooterMotorD = 0.0001000;
      public double k_shooterMotorFF = 0.00010;
    }

    public double k_shooterMinOutput;
    public double k_shooterMaxOutput = 1.0;

    public int k_pivotEncoderId = 5;

    public double k_pivotMotorP;
    public double k_pivotMotorI;
    public double k_pivotMotorD;
    public double k_pivotMotorIZone;

    public double k_ampPivotAngle = 65.0;
    public double k_wingPivotAngle = 26.0;
    public double k_subwooferPivotAngle = 62.0;
    public double k_podiumPivotAngle = 46.0;
    public double k_passPivotAngle = 50.0;
    public double k_trapPivotAngle = 45.0;

    public double k_length = 11.94335; // in inches
    public double k_mass = 5.44311; // in kg

    public double k_pivotHeight = 4.0;

    public double k_minAngle;
    public double k_maxAngle = 65.0;
    public double k_startingAngle = 0.0;

    public double k_distanceFromCenter = 4.0008;

    // Offset, -90, since that's where we want to start
    public double k_absPivotOffset;

    public double k_initialPivotOffset;

    public double threadsPerInch = 8.0;
    public double helices = 1.0;
    public double k_rotationsPerInch = threadsPerInch / helices;

    public double maxPivotExtensionInches = 12.5643;
    public double k_relRotationsToMaxExtension = maxPivotExtensionInches * k_rotationsPerInch;

    // Shooter angle lookup table, V2
    // Rotations from plate at 90 degrees (aka straight up)
    // Plate, Rel
    // 20 deg, 33.162384
    // 45 deg, 20.878754
    // 60 deg, 12.488831
    // 90 deg, 0.0
  }

  public static class AutoAimConstants {
    public TranslationConstants Translation = new TranslationConstants();

    public class TranslationConstants {
      public double k_P = 3.0;
      public double k_I = 0.0;
      public double k_D = 0.0;
    }

    public RotationConstants Rotation = new RotationConstants();

    public class RotationConstants {
      public double k_P = 3.0;
      public double k_I = 0.0;
      public double k_D = 0.0;
    }

    public double k_autoAimDistanceThreshold = 1; // in meters
    public double k_autoAimOmegaRPSThreshold = Math.PI / 8; // in radians
    public double k_autoAimAngleTolerance = Units.degreesToRadians(3.0);

    // Max speed
    public double k_maxSpeed = 3.0; // Meters per second
    public double k_maxLinearAcceleration = 1.5; // Meters per second^2

    // Max rotation
    public double k_maxAngularSpeed = Math.PI * 2 * 1.5; // Radians per second (540 degrees per second)
    public double k_maxAngularAcceleration = Math.PI * 2 * 2; // Radians per second^2 (720 degrees per second^2)

    // Use these values as overrides in DriveTrajectoryTask
    // These are the current PathPlanner values
    // public static PathConstraints k_pathConstraints = new PathConstraints(
    // k_maxSpeed,
    // k_maxLinearAcceleration,
    // k_maxAngularSpeed,
    // k_maxAngularAcceleration);

    // public static PIDController translationPIDController = new
    // PIDController(
    // Translation.k_P,
    // Translation.k_I,
    // Translation.k_D);

    // public static ProfiledPIDController rotationPIDController = new
    // ProfiledPIDController(
    // Rotation.k_P,
    // Rotation.k_I,
    // Rotation.k_D,
    // new Constraints(k_maxSpeed, k_maxLinearAcceleration));

    // public static HolonomicDriveController k_autoTargetController = new
    // HolonomicDriveController(
    // translationPIDController,
    // translationPIDController,
    // rotationPIDController);
  }

  public static class SwerveDriveConstants {
    // Drivetrain wheel offsets
    public double k_xDistance = 0.762; // 30 inches Forward/Backward
    public double k_yDistance = 0.762; // in meters! Side-to-Side

    public double k_xCenterDistance = k_xDistance / 2.0;
    public double k_yCenterDistance = k_yDistance / 2.0;

    // Max speeds
    public double k_maxSpeed = 2.5; // Meters per second
    public double k_maxBoostSpeed; // Meters per second
    public double k_maxAngularSpeed = Math.PI * 2.0; // Radians per second

    public double k_maxDemoSpeed = k_maxSpeed / 2.0;
    public double k_maxDemoAngularSpeed = k_maxAngularSpeed / 2.0;
    public double k_maxDemoBoostSpeed = 4.5;

    // Max acceleration
    public double k_maxLinearAcceleration = 12.0; // Meters per second^2
    public double k_maxAngularAcceleration = Math.PI * 8.0; // Radians per second^2

    public double k_slowScaler; // % reduction in speed

    public double k_wheelRadiusIn; // inches
    public double k_driveGearRatio;
    public double k_turnGearRatio = 7.0 / 150.0;

    public DriveConstants Drive = new DriveConstants();

    // Drivetrain drive motor constants
    public class DriveConstants {
      public int k_FLMotorId = 6;
      public int k_FRMotorId = 8;
      public int k_BRMotorId = 10;
      public int k_BLMotorId = 12;

      public int k_currentLimit = 40;

      public double k_P;
      public double k_I = 0.0;
      public double k_D = 0.0;
      public double k_IZone = 0.0;

      public double k_FFS;
      public double k_FFV;
      public double k_FFA;
    }

    public TurnConstants Turn = new TurnConstants();

    // Drivetrain (turn) constants
    public class TurnConstants {
      // Drivetrain turning offset constants
      public double k_FLOffset;
      public double k_FROffset;
      public double k_BROffset;
      public double k_BLOffset;

      public int k_FLAbsId = 0;
      public int k_FRAbsId = 1;
      public int k_BRAbsId = 2;
      public int k_BLAbsId = 3;

      public int k_currentLimit = 25;

      public int k_FLMotorId = 5;
      public int k_FRMotorId = 7;
      public int k_BRMotorId = 9;
      public int k_BLMotorId = 11;

      public double k_P = 5.6906;
      public double k_I = 0.0;
      public double k_D = 0.1976;
      public double k_IZone = 0.0;

      // We only use FF and are too scared to delete the others
      public double k_FF = 0.0;

      public double k_FFS = 0.29745;
      public double k_FFV = 0.43892; // Not used (fear)
      public double k_FFA = 0.048573; // Not used (fear)

      public double k_TurningMinOutput = -1.0;
      public double k_TurningMaxOutput = 1.0;
    }
  }

  public static class VisionConstants {
    // Increase these numbers to trust your model's state estimates less.
    public double k_positionStdDevX = 0.1;
    public double k_positionStdDevY = 0.1;
    public double k_positionStdDevTheta = 10.0;

    // TODO: try this: 0.003, 0.003, 0.0002

    // Increase these numbers to trust global measurements from vision less.
    public double k_visionStdDevX = 0.0;
    public double k_visionStdDevY = 0.0;
    public double k_visionStdDevTheta = 0.0;

    public RotationConstants Rotation = new RotationConstants();

    public class RotationConstants {
      public double k_P;
      public double k_I = 0.0;
      public double k_D = 0.02;
    }

    public frc.robot.constants.VisionConstants teleopVisionConstants = new frc.robot.constants.VisionConstants(1, 6.0,
        32.0, 10.0);
    public frc.robot.constants.VisionConstants defaultAutoVisionConstants = new frc.robot.constants.VisionConstants(2,
        4.0, 32.0, 10.0);
    public frc.robot.constants.VisionConstants fourNoteVisionConstants = new frc.robot.constants.VisionConstants(2, 4.0,
        32.0, 10.0);
    public frc.robot.constants.VisionConstants threeNoteVisionConstants = new frc.robot.constants.VisionConstants(2,
        6.0, 16.0, 0.25);
    public frc.robot.constants.VisionConstants funnyNoteVisionConstants = new frc.robot.constants.VisionConstants(1,
        10.0, 100.0, 0.25);
  }

  public static class AutoConstants {
    // Needs to be more than the max robot speed, to allow for turning
    public double k_maxModuleSpeed; // Meters per second
    public double k_maxAcceleration = 1.0; // NOT USED

    public TimingConstants Timing = new TimingConstants();

    public class TimingConstants {
      public double k_shootFeedTime = 0.2; // seconds
      public double k_shootRevTime = 0.75; // seconds
      public double k_intakeDeployTime = 0.1; // seconds
      public double k_intakeBounceTime; // seconds
    }

    public Pose2d k_redCenterPose2d = new Pose2d(15.19, 5.50, new Rotation2d(Units.degreesToRadians(180.0)));
    public Pose2d k_blueCenterPose2d = new Pose2d(1.27, 5.50, new Rotation2d(0));
  }

  public LEDsConstants LEDs = new LEDsConstants();

  public static class LEDsConstants {
    public int k_PWMId = 1;

    public DriveConstants Drive = new DriveConstants();

    public class DriveConstants {
      public int k_start = 0;
      public int k_length = 150;
    }

    public ShooterConstants Shooter = new ShooterConstants();

    public class ShooterConstants {
      public int k_start = Drive.k_start + Drive.k_length;
      public int k_length = 150;
    }

    public int k_totalLength = Drive.k_length + Shooter.k_length;
  }
}
