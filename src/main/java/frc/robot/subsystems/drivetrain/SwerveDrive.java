package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Subsystem;

public class SwerveDrive extends Subsystem {
  private static SwerveDrive m_swerve = null;

  private final SwerveModule[] m_modules = {
    new SwerveModule(Constants.SwerveDrive.Drive.k_FLMotorId, Constants.SwerveDrive.Turn.k_FLMotorId,
      Constants.SwerveDrive.Turn.k_FLOffset, "FL"), // 0
    new SwerveModule(Constants.SwerveDrive.Drive.k_FRMotorId, Constants.SwerveDrive.Turn.k_FRMotorId,
      Constants.SwerveDrive.Turn.k_FROffset, "FR"), // 1
    new SwerveModule(Constants.SwerveDrive.Drive.k_BLMotorId, Constants.SwerveDrive.Turn.k_BLMotorId,
      Constants.SwerveDrive.Turn.k_BLOffset, "BL"), // 2
    new SwerveModule(Constants.SwerveDrive.Drive.k_BRMotorId, Constants.SwerveDrive.Turn.k_BRMotorId,
      Constants.SwerveDrive.Turn.k_BROffset, "BR") // 3
  };

  // Robot "forward" is +x
  // Robot "left" is +y
  // Robot "clockwise" is -z
  private final Translation2d[] m_moduleLocations = {
    new Translation2d(Constants.SwerveDrive.k_xCenterDistance, Constants.SwerveDrive.k_yCenterDistance),
    new Translation2d(Constants.SwerveDrive.k_xCenterDistance, -Constants.SwerveDrive.k_yCenterDistance),
    new Translation2d(-Constants.SwerveDrive.k_xCenterDistance, Constants.SwerveDrive.k_yCenterDistance),
    new Translation2d(-Constants.SwerveDrive.k_xCenterDistance, -Constants.SwerveDrive.k_yCenterDistance)
  };

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final Limelight m_limelight = new Limelight("limelight");

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_moduleLocations[Module.FRONT_LEFT],
      m_moduleLocations[Module.FRONT_RIGHT],
      m_moduleLocations[Module.BACK_LEFT],
      m_moduleLocations[Module.BACK_RIGHT]);

  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    m_kinematics,
    m_gyro.getRotation2d(),
    new SwerveModulePosition[] {
      m_modules[Module.FRONT_LEFT].getPosition(),
      m_modules[Module.FRONT_RIGHT].getPosition(),
      m_modules[Module.BACK_LEFT].getPosition(),
      m_modules[Module.BACK_RIGHT].getPosition()
    },
    new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

  private SwerveDrive() {
    reset();
  }

  public static SwerveDrive getInstance() {
    if (m_swerve == null) {
      m_swerve = new SwerveDrive();
    }
    return m_swerve;
  }

  /**
   * Toggles between NeutralModeValue.Brake and NeutralModeValue.Coast
   *
   * @param isBrake true enables Brake mode, false enables Coast mode
   */
  public void setBrakeMode(boolean isBrake) {
    for (SwerveModule module : m_modules) {
      module.getDriveMotor().setNeutralMode(
          isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);

    }
  }

  public void setPose(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getRotation2d(),
        new SwerveModulePosition[] {
          m_modules[Module.FRONT_LEFT].getPosition(),
          m_modules[Module.FRONT_RIGHT].getPosition(),
          m_modules[Module.BACK_LEFT].getPosition(),
          m_modules[Module.BACK_RIGHT].getPosition()
        },
        pose);
  }

  public double calculateAutoAimAngle() {
    double bot_x = m_poseEstimator.getEstimatedPosition().getX();
    double bot_y = m_poseEstimator.getEstimatedPosition().getY();
    double speaker_x = Constants.Field.k_redSpeakerPose.getX();
    double speaker_y = Constants.Field.k_redSpeakerPose.getY();

    double x = Constants.Field.k_width-bot_x;
    double distance = Math.sqrt(Math.pow(speaker_x - bot_x,2) + Math.pow(speaker_y - bot_y,2));

    return Math.toDegrees(Math.acos(x/distance)); //Theta
    // inverse cosine(x/d)
  }

  public void resetPose() {
    resetGyro();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void resetOdometry(Pose2d pose) {
    for (SwerveModule module : m_modules) {
      module.resetDriveEncoder();
    }

    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_modules[Module.FRONT_LEFT].getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_modules[Module.FRONT_RIGHT].getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_modules[Module.BACK_LEFT].getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_modules[Module.BACK_LEFT].getTurnPosition()))
        },
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    setPose(pose);
  }

  public Rotation2d getRotation2d() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }

  public void clearTurnPIDAccumulation() {
    for (SwerveModule module : m_modules) {
      module.clearTurnPIDAccumulation();
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    double maxBoostSpeed = Constants.SwerveDrive.k_maxSpeed * Constants.SwerveDrive.k_boostScaler;
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxBoostSpeed);

    for(int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void pointModules(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // Zero out the speed component of each swerve module state
    for (SwerveModuleState moduleState : swerveModuleStates) {
      moduleState.speedMetersPerSecond = 0.0;
    }

    for(int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void pointInwards() {
    m_modules[Module.FRONT_LEFT].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_modules[Module.FRONT_RIGHT].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_modules[Module.BACK_LEFT].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_modules[Module.BACK_RIGHT].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void reset() {
    setBrakeMode(false);
    resetPose();
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void setGyroAngleAdjustment(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  public void resetGyro() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
  }

  @Override
  public void periodic() {
    for (SwerveModule module : m_modules) {
      module.periodic();
    }
  }

  @Override
  public void stop() {
    setBrakeMode(true);
    drive(0.0,0.0,0.0,true);
  }

  @Override
  public void writePeriodicOutputs() {
  }

  private double[] getCurrentStates() {
    double[] currentStates = {
        m_modules[Module.FRONT_LEFT].getTurnPosition() * 360, m_modules[Module.FRONT_LEFT].getDriveVelocity(),
        m_modules[Module.FRONT_RIGHT].getTurnPosition() * 360, m_modules[Module.FRONT_RIGHT].getDriveVelocity(),
        m_modules[Module.BACK_LEFT].getTurnPosition() * 360, m_modules[Module.BACK_LEFT].getDriveVelocity(),
        m_modules[Module.FRONT_RIGHT].getTurnPosition() * 360, m_modules[Module.BACK_RIGHT].getDriveVelocity()
    };

    return currentStates;
  }

  private double[] getDesiredStates() {
    double[] desiredStates = {
        m_modules[Module.FRONT_LEFT].getDesiredState().angle.getDegrees(), m_modules[Module.FRONT_LEFT].getDesiredState().speedMetersPerSecond,
        m_modules[Module.FRONT_RIGHT].getDesiredState().angle.getDegrees(), m_modules[Module.FRONT_RIGHT].getDesiredState().speedMetersPerSecond,
        m_modules[Module.BACK_LEFT].getDesiredState().angle.getDegrees(), m_modules[Module.FRONT_LEFT].getDesiredState().speedMetersPerSecond,
        m_modules[Module.BACK_RIGHT].getDesiredState().angle.getDegrees(), m_modules[Module.BACK_RIGHT].getDesiredState().speedMetersPerSecond
    };

    return desiredStates;
  }

  @Override
  public void outputTelemetry() {
    double currentTime = Timer.getFPGATimestamp();

    if(m_limelight.seesAprilTag()) {
      m_poseEstimator.addVisionMeasurement(m_limelight.getBotpose2D(), currentTime);
    }

    m_poseEstimator.updateWithTime(
        currentTime,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_modules[Module.FRONT_LEFT].getPosition(),
            m_modules[Module.FRONT_RIGHT].getPosition(),
            m_modules[Module.BACK_LEFT].getPosition(),
            m_modules[Module.BACK_RIGHT].getPosition()
        }
    );

    for (SwerveModule module : m_modules) {
      module.outputTelemetry();
    }

    SmartDashboard.putNumberArray("SwerveDrive/CurrentStates", getCurrentStates());
    SmartDashboard.putNumberArray("SwerveDrive/DesiredStates", getDesiredStates());

    SmartDashboard.putNumber("SwerveDrive/Gyro/AngleDegrees", m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("SwerveDrive/Gyro/Pitch", m_gyro.getPitch());
    SmartDashboard.putNumberArray("SwerveDrive/Pose",
        new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
  }

  public interface Module {
    int FRONT_LEFT = 0;
    int FRONT_RIGHT = 1;
    int BACK_LEFT = 2;
    int BACK_RIGHT = 3;
  }
}
