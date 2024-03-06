package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.AprilTagLocations;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;

public class SwerveDrive extends SwerveSysId {
  private static SwerveDrive m_swerve = null;

  private static final SwerveModule[] m_modules = {
      new SwerveModule(Constants.SwerveDrive.Drive.k_FLMotorId, Constants.SwerveDrive.Turn.k_FLMotorId,
          Constants.SwerveDrive.Turn.k_FLAbsId,
          Constants.SwerveDrive.Turn.k_FLOffset, "FL"), // 0
      new SwerveModule(Constants.SwerveDrive.Drive.k_FRMotorId, Constants.SwerveDrive.Turn.k_FRMotorId,
          Constants.SwerveDrive.Turn.k_FRAbsId,
          Constants.SwerveDrive.Turn.k_FROffset, "FR"), // 1
      new SwerveModule(Constants.SwerveDrive.Drive.k_BRMotorId, Constants.SwerveDrive.Turn.k_BRMotorId,
          Constants.SwerveDrive.Turn.k_BRAbsId,
          Constants.SwerveDrive.Turn.k_BROffset, "BR"), // 2
      new SwerveModule(Constants.SwerveDrive.Drive.k_BLMotorId, Constants.SwerveDrive.Turn.k_BLMotorId,
          Constants.SwerveDrive.Turn.k_BLAbsId,
          Constants.SwerveDrive.Turn.k_BLOffset, "BL") // 3
  };

  // Robot "forward" is +x
  // Robot "left" is +y
  // Robot "clockwise" is -z
  private final Translation2d[] m_moduleLocations = {
      new Translation2d(Constants.SwerveDrive.k_xCenterDistance, Constants.SwerveDrive.k_yCenterDistance),
      new Translation2d(Constants.SwerveDrive.k_xCenterDistance, -Constants.SwerveDrive.k_yCenterDistance),
      new Translation2d(-Constants.SwerveDrive.k_xCenterDistance, -Constants.SwerveDrive.k_yCenterDistance),
      new Translation2d(-Constants.SwerveDrive.k_xCenterDistance, Constants.SwerveDrive.k_yCenterDistance)
  };

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final Limelight m_limelightOne = new Limelight("limelight-one");
  private final Limelight m_limelightTwo = new Limelight("limelight-two");
  private final Limelight m_limelightThree = new Limelight("limelight-three");

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_moduleLocations[Module.FRONT_LEFT],
      m_moduleLocations[Module.FRONT_RIGHT],
      m_moduleLocations[Module.BACK_RIGHT],
      m_moduleLocations[Module.BACK_LEFT]);

  // TODO: we might be able to have a better default pose here
  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_modules[Module.FRONT_LEFT].getPosition(),
          m_modules[Module.FRONT_RIGHT].getPosition(),
          m_modules[Module.BACK_RIGHT].getPosition(),
          m_modules[Module.BACK_LEFT].getPosition()
      },
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

  private boolean m_hasSetPose = false;

  private SwerveDrive() {
    super(m_modules, "SwerveDrive");

    resetTurnOffsets();
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
      module.getDriveMotor().setIdleMode(
          isBrake ? IdleMode.kBrake : IdleMode.kCoast);
    }
  }

  /**
   * @param degreeMode If <code>true</code>, return result in degrees; otherwise,
   *                   return in radians
   */
  public double calculateAutoAimAngle(boolean degreeMode, int tagID) {
    double botX = m_poseEstimator.getEstimatedPosition().getX();
    double botY = m_poseEstimator.getEstimatedPosition().getY();
    double targetX = AprilTagLocations.getPosition(tagID).getX();
    double targetY = AprilTagLocations.getPosition(tagID).getY();

    double x = targetX - botX;
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(targetY - botY, 2));

    double theta = Math.acos(x / distance);

    putNumber("AutoAimAngle", Units.radiansToDegrees(theta));

    return degreeMode ? Units.radiansToDegrees(theta) : theta;
  }

  public void resetPose() {
    resetGyro();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)), false);
  }

  public void resetOdometry(Pose2d pose) {
    resetOdometry(pose, true);
  }

  public void resetOdometry(Pose2d pose, boolean shouldSetFlag) {
    m_hasSetPose |= shouldSetFlag;

    // setAllianceGyroAngleAdjustment();

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
                Rotation2d.fromRotations(m_modules[Module.BACK_RIGHT].getTurnPosition())),
            new SwerveModulePosition(0.0,
                Rotation2d.fromRotations(m_modules[Module.BACK_LEFT].getTurnPosition()))
        },
        pose);

    // setPose(pose);
  }

  // Only call this in simulation
  public void setPose(Pose2d pose) {
    if (RobotBase.isReal()) {
      throw new UnsupportedOperationException("Only call setPose() from sim");
    }
    // m_hasSetPose = true;

    m_poseEstimator.resetPosition(
        pose.getRotation(), // Maybe should be getRotation2d() (it uses pose estimator)?
        new SwerveModulePosition[] {
            m_modules[Module.FRONT_LEFT].getPosition(),
            m_modules[Module.FRONT_RIGHT].getPosition(),
            m_modules[Module.BACK_RIGHT].getPosition(),
            m_modules[Module.BACK_LEFT].getPosition()
        },
        pose);
  }

  public void clearTurnPIDAccumulation() {
    for (SwerveModule module : m_modules) {
      module.clearTurnPIDAccumulation();
    }
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrive.k_maxBoostSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrive.k_maxBoostSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  /////
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean lockHeading) {
    if (lockHeading) {
      rot = correct();
    }

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrive.k_maxBoostSpeed);

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  private Rotation2d m_oldRotation;

  public void updateFormerGyroPosition(boolean hasUpdated) {
    if (!hasUpdated) {
      m_oldRotation = m_gyro.getRotation2d();
    }
  }

  private double correct() {
    // TODO: use the PID controllers in Constants.AutoAim to correct the heading
    // (Bro Jordan can't spell 💀)
    ProfiledPIDController pid = new ProfiledPIDController(1, 0, 0, new Constraints(10, 1));
    return pid.calculate(m_gyro.getRotation2d().getDegrees(), m_oldRotation.getDegrees());
  }
  /////

  public void pointModules(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // Zero out the speed component of each swerve module state
    for (SwerveModuleState moduleState : swerveModuleStates) {
      moduleState.speedMetersPerSecond = 0.0;
    }

    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  public void resetTurnOffsets() {
    for (SwerveModule module : m_modules) {
      module.resetTurnConfig();
    }
  }

  public AHRS getGyro() {
    return m_gyro;
  }

  public void setGyroAngleAdjustment(double angle) {
    m_gyro.setAngleAdjustment(angle);
  }

  public void setAllianceGyroAngleAdjustment() {
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      m_gyro.setAngleAdjustment(180);
    } else {
      m_gyro.setAngleAdjustment(0);
    }
  }

  public void resetGyro() {
    m_gyro.reset();
    // setGyroAngleAdjustment(0);
    setAllianceGyroAngleAdjustment();
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();
    LimelightHelpers.PoseEstimate LL1Pose = m_limelightOne.getPoseEstimation();
    LimelightHelpers.PoseEstimate LL2Pose = m_limelightTwo.getPoseEstimation();
    LimelightHelpers.PoseEstimate LL3Pose = m_limelightThree.getPoseEstimation();

    // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,99999)); //TODO: Check this, Limelight docs use these values

    if(LL1Pose.tagCount > 0) {
      m_poseEstimator.addVisionMeasurement(LL1Pose.pose, LL1Pose.timestampSeconds);
    }

    if(LL2Pose.tagCount > 0) {
      m_poseEstimator.addVisionMeasurement(LL2Pose.pose, LL2Pose.timestampSeconds);
    }

    if(LL3Pose.tagCount > 0) {
      m_poseEstimator.addVisionMeasurement(LL3Pose.pose, LL3Pose.timestampSeconds);
    }

    if (RobotBase.isReal()) {
      m_poseEstimator.updateWithTime(
          currentTime,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_modules[Module.FRONT_LEFT].getPosition(),
              m_modules[Module.FRONT_RIGHT].getPosition(),
              m_modules[Module.BACK_RIGHT].getPosition(),
              m_modules[Module.BACK_LEFT].getPosition()
          });
    } else {
      setPose(new Pose2d(
          Preferences.getDouble("SwerveDrive/x", 0),
          Preferences.getDouble("SwerveDrive/y", 0),
          new Rotation2d(Math.toRadians(Preferences.getDouble("SwerveDrive/rot", 0)))));
    }

    for (SwerveModule module : m_modules) {
      module.periodic();
    }
  }

  @Override
  public void stop() {
    setBrakeMode(true);
    drive(0.0, 0.0, 0.0, true);
  }

  @Override
  public void writePeriodicOutputs() {
  }

  @Override
  public void reset() {
    setBrakeMode(false);
    resetPose();
  }

  public interface Module {
    int FRONT_LEFT = 0;
    int FRONT_RIGHT = 1;
    int BACK_RIGHT = 2;
    int BACK_LEFT = 3;
  }

  // Logged
  @AutoLogOutput
  public Rotation2d getRotation2d() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }

  @AutoLogOutput
  private SwerveModuleState[] getCurrentStates() {
    SwerveModuleState[] currentStates = {
        m_modules[Module.FRONT_LEFT].getState(),
        m_modules[Module.FRONT_RIGHT].getState(),
        m_modules[Module.BACK_RIGHT].getState(),
        m_modules[Module.BACK_LEFT].getState()
    };

    return currentStates;
  }

  @AutoLogOutput
  private SwerveModuleState[] getDesiredStates() {
    SwerveModuleState[] desiredStates = {
        m_modules[Module.FRONT_LEFT].getDesiredState(),
        m_modules[Module.FRONT_RIGHT].getDesiredState(),
        m_modules[Module.BACK_RIGHT].getDesiredState(),
        m_modules[Module.BACK_LEFT].getDesiredState()
    };

    return desiredStates;
  }

  @AutoLogOutput
  public boolean hasSetPose() {
    return m_hasSetPose;
  }

  @AutoLogOutput
  private double getGyroYaw() {
    return m_gyro.getRotation2d().getDegrees();
  }

  @AutoLogOutput
  private double getGyroPitch() {
    return m_gyro.getPitch();
  }

  @AutoLogOutput
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }
}
