package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
// import frc.robot.constants.Constants.*;

public final class AmadeusConstants {
  // private ClimberConstants m_climber = new ClimberConstants();
  // private IntakeConstants m_intake = new IntakeConstants();
  // private SimulationConstants m_sim = new SimulationConstants();
  // private FieldConstants m_field = new FieldConstants();
  // private Constants.RobotConstants m_robot = new Constants.RobotConstants();

  // public ClimberConstants climber() {
  //   return m_climber;
  // }

  // public IntakeConstants intake() {
  //   return m_intake;
  // }

  // public SimulationConstants simulation() {
  //   return m_sim;
  // }

  // public FieldConstants field() {
  //   return m_field;
  // }

  // public Constants.RobotConstants robot() {
  //   return m_robot;
  // }

  public AutoConstants Auto = new AutoConstants();
  public class AutoConstants {
    // Needs to be more than the max robot speed, to allow for turning
    public final double k_maxModuleSpeed = 5.0; // Meters per second
    public final double k_maxAcceleration = 1.0; // NOT USED

    public TimingConstants Timing = new TimingConstants();
    public class TimingConstants {
      public final double k_shootFeedTime = 0.2; // seconds
      public final double k_shootRevTime = 0.75; // seconds
      public final double k_intakeDeployTime = 0.1; // seconds
      public final double k_intakeBounceTime = 0.2; // seconds
    }

    // TODO: Add left and right subwoofer starting poses
    public final Pose2d k_redCenterPose2d = new Pose2d(15.19, 5.50, new Rotation2d(Units.degreesToRadians(180.0)));

    public final Pose2d k_blueCenterPose2d = new Pose2d(1.27, 5.50, new Rotation2d(0));
  }

  public VisionConstants Vision = new VisionConstants();
  public class VisionConstants {
    // Increase these numbers to trust your model's state estimates less.
    public final double k_positionStdDevX = 0.1;
    public final double k_positionStdDevY = 0.1;
    public final double k_positionStdDevTheta = 10.0;

    // TODO: try this: 0.003, 0.003, 0.0002

    // Increase these numbers to trust global measurements from vision less.
    public final double k_visionStdDevX = 0.0;
    public final double k_visionStdDevY = 0.0;
    public final double k_visionStdDevTheta = 0.0;

    public RotationConstants Rotation = new RotationConstants();
    public class RotationConstants {
      public final double k_P = 7.0;
      public final double k_I = 0.0;
      public final double k_D = 0.02;
    }
  }

  public SwerveDriveConstants SwerveDrive = new SwerveDriveConstants();
  public class SwerveDriveConstants {
    // Drivetrain wheel offsets
    public final double k_xDistance = 0.762; // 30 inches Forward/Backward
    public final double k_yDistance = 0.762; // in meters! Side-to-Side

    public final double k_xCenterDistance = k_xDistance / 2.0;
    public final double k_yCenterDistance = k_yDistance / 2.0;

    // Max speeds
    public final double k_maxSpeed = 2.5; // Meters per second
    public final double k_maxBoostSpeed = 5.0; // Meters per second
    public final double k_maxAngularSpeed = Math.PI * 2.0; // Radians per second

    // Max acceleration
    public final double k_maxLinearAcceleration = 12.0; // Meters per second^2
    public final double k_maxAngularAcceleration = Math.PI * 8.0; // Radians per second^2

    public final double k_slowScaler = 0.2; // % reduction in speed

    public final double k_wheelRadiusIn = 2.0; // inches
    public final double k_driveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    public final double k_turnGearRatio = 7.0 / 150.0;

    public DriveConstants Drive = new DriveConstants();
    // Drivetrain drive motor constants
    public class DriveConstants {
      public final int k_FLMotorId = 6;
      public final int k_FRMotorId = 8;
      public final int k_BRMotorId = 10;
      public final int k_BLMotorId = 12;

      public final int k_currentLimit = 40;

      public final double k_P = 0.00079049;
      public final double k_I = 0.0;
      public final double k_D = 0.0;
      public final double k_IZone = 0.0;

      public final double k_FFS = 0.21465;
      public final double k_FFV = 2.3914;
      public final double k_FFA = 0.36664;
    }

    public TurnConstants Turn = new TurnConstants();
    // Drivetrain (turn) constants
    public class TurnConstants {
      // Drivetrain turning offset constants
      public final double k_FLOffset = 0.159;
      public final double k_FROffset = 0.163;
      public final double k_BROffset = 0.832;
      public final double k_BLOffset = 0.157;

      public final int k_FLAbsId = 0;
      public final int k_FRAbsId = 1;
      public final int k_BRAbsId = 2;
      public final int k_BLAbsId = 3;

      public final int k_currentLimit = 25;

      public final int k_FLMotorId = 5;
      public final int k_FRMotorId = 7;
      public final int k_BRMotorId = 9;
      public final int k_BLMotorId = 11;

      public final double k_P = 5.6906;
      public final double k_I = 0.0;
      public final double k_D = 0.1976;
      public final double k_IZone = 0.0;

      public final double k_FFS = 0.29745;
      public final double k_FFV = 0.43892; // Not used
      public final double k_FFA = 0.048573; // Not used

      public final double k_TurningMinOutput = -1.0;
      public final double k_TurningMaxOutput = 1.0;
    }
  }

  public AutoAimConstants AutoAim = new AutoAimConstants();
  public class AutoAimConstants {
    public TranslationConstants Translation = new TranslationConstants();
    public class TranslationConstants {
      public final double k_P = 3.0;
      public final double k_I = 0.0;
      public final double k_D = 0.0;
    }

    public RotationConstants Rotation = new RotationConstants();
    public class RotationConstants {
      public final double k_P = 3.0;
      public final double k_I = 0.0;
      public final double k_D = 0.0;
    }

    public final double k_autoAimDistanceThreshold = 1; // in meters
    public final double k_autoAimOmegaRPSThreshold = Math.PI / 8; // in radians
    public final double k_autoAimAngleTolerance = Units.degreesToRadians(3.0);

    // Max speed
    public final double k_maxSpeed = 3.0; // Meters per second
    public final double k_maxLinearAcceleration = 1.5; // Meters per second^2

    // Max rotation
    public final double k_maxAngularSpeed = Math.PI * 2 * 1.5; // Radians per second (540 degrees per second)
    public final double k_maxAngularAcceleration = Math.PI * 2 * 2; // Radians per second^2 (720 degrees per second^2)

    // Use these values as overrides in DriveTrajectoryTask
    // These are the current PathPlanner values
    // public static final PathConstraints k_pathConstraints = new PathConstraints(
    // k_maxSpeed,
    // k_maxLinearAcceleration,
    // k_maxAngularSpeed,
    // k_maxAngularAcceleration);

    // public static final PIDController translationPIDController = new
    // PIDController(
    // Translation.k_P,
    // Translation.k_I,
    // Translation.k_D);

    // public static final ProfiledPIDController rotationPIDController = new
    // ProfiledPIDController(
    // Rotation.k_P,
    // Rotation.k_I,
    // Rotation.k_D,
    // new Constraints(k_maxSpeed, k_maxLinearAcceleration));

    // public static final HolonomicDriveController k_autoTargetController = new
    // HolonomicDriveController(
    // translationPIDController,
    // translationPIDController,
    // rotationPIDController);
  }

  public IntakeConstants Intake = new IntakeConstants();
  public class IntakeConstants {
    public final int k_pivotMotorId = 13;
    public final int k_intakeMotorId = 14;

    public final double k_pivotMotorMaxOutput = 0.2;

    public final int k_pivotEncoderId = 4;

    public final double k_stowPivotAngle = 263.0;
    public final double k_groundPivotAngle = 41.0;
    public final double k_sourcePivotAngle = 180.0;
    // public final double k_sourcePivotAngle = k_stowPivotAngle;
    public final double k_ejectPivotAngle = 110.0;
    public final double k_ampPivotAngle = 100.0;

    public final double k_intakeSpeed = 0.4;
    public final double k_ejectSpeed = -0.525;
    public final double k_feedShooterSpeed = -1.0;

    public final double k_pivotMotorP = 0.035;
    public final double k_pivotMotorI = 0.0;
    public final double k_pivotMotorD = 0.0;
    public final double k_pivotMotorIZone = 0.0;

    public final double k_pivotMotorKS = 0.0; // volts
    public final double k_pivotMotorKG = 0.03; // volts
    public final double k_pivotMotorKV = 0.244; // volts*sec / rad
    public final double k_pivotMotorKA = 0.0; // 0.02; // volts*sec^2 / rad

    public final double k_length = 14.319626; // In inches
    public final double k_mass = 5.8967; // In kg

    public final double k_pivotHeight = 5.75;

    public final double k_minAngle = 0.0;
    public final double k_maxAngle = 0.0;
    public final double k_startingAngle = 0.0;

    public final double k_distanceFromCenter = 12.5;
    public final int k_sensorThreshold = 1000; // 1,000 is "in", 2000 is max

    public final double k_pivotOffset = 74.5;

    public double k_maxVelocity = 690.0;
    public double k_maxAcceleration = 1380.0;
  }

  public ShooterConstants Shooter = new ShooterConstants();
  public class ShooterConstants {
    public final double k_maxRPM = 6000.0; // but that's just a theory
    public final double k_passRPM = 4000.0;

    public final int k_shooterSpeedTolerance = 100; // 1,000 is "in", 2000 is max

    public final int k_pivotMotorId = 15;
    public final int k_topMotorId = 16;
    public final int k_bottomMotorId = 17;

    public final double k_shooterMotorP = 0.0009300;
    public final double k_shooterMotorI = 0.00000008;
    public final double k_shooterMotorD = 0.0001000;
    public final double k_shooterMotorFF = 0.00015;

    public final double k_shooterMinOutput = 0.0;
    public final double k_shooterMaxOutput = 1.0;

    public final int k_pivotEncoderId = 5;

    public final double k_pivotMotorP = 2.0;
    public final double k_pivotMotorI = 0.0;
    public final double k_pivotMotorD = 0.001;
    public final double k_pivotMotorIZone = 0.0;

    public final double k_ampPivotAngle = 0.0; // TODO: get amp pivot angle
    public final double k_wingPivotAngle = 26.0;
    public final double k_subwooferPivotAngle = 62.0;
    public final double k_podiumPivotAngle = 46.0;
    public final double k_passPivotAngle = 50.0;

    public final double k_length = 11.94335; // in inches
    public final double k_mass = 5.44311; // in kg

    public final double k_pivotHeight = 4.0;

    public final double k_minAngle = 20.0;
    public final double k_maxAngle = 65.0;
    public final double k_startingAngle = 0.0;

    public final double k_distanceFromCenter = 4.0008;

    // Offset, -90, since that's where we want to start
    public final double k_absPivotOffset = 208.587 - 90;

    public final double k_initalPivotOffset = -1.75;

    public final double threadsPerInch = 8.0;
    public final double helices = 1.0;
    public final double k_rotationsPerInch = threadsPerInch / helices;

    public final double maxPivotExtensionInches = 12.5643;
    public final double k_relRotationsToMaxExtension = maxPivotExtensionInches * k_rotationsPerInch;

    // Shooter angle lookup table, V2
    // Rotations from plate at 90 degrees (aka straight up)
    // Plate, Rel
    // 20 deg, 33.162384
    // 45 deg, 20.878754
    // 60 deg, 12.488831
    // 90 deg, 0.0
  }

  public ClimberConstants Climber = new ClimberConstants();
  public class ClimberConstants {
    public final int k_leftMotorID = 19;
    public final int k_rightMotorID = 18;

    public final double k_raiseSpeed = 600.0; // RPM
    public final double k_lowerSpeed = -600.0; // RPM

    public final double k_gearRatio = 1.0 / 16.0;

    public final double k_P = 0.001;
    public final double k_I = 0.0;
    public final double k_D = 0.0;

    public final double k_minOutput = -0.5;
    public final double k_maxOutput = 0.5;
  }
}
