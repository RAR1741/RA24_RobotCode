package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.AllianceHelpers;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Limelight;

public class SwerveDrive extends SwerveSysId {
  private static SwerveDrive m_swerve = null;

  private Rotation2d m_rotationTarget;

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
  public final Limelight m_limelightLeft = new Limelight("limelight-left");
  public final Limelight m_limelightRight = new Limelight("limelight-right");
  public final Limelight m_limelightShooter = new Limelight("limelight-shooter");

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
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      createStateStdDevs(
          Constants.Vision.k_positionStdDevX,
          Constants.Vision.k_positionStdDevY,
          Constants.Vision.k_positionStdDevTheta),
      createVisionMeasurementStdDevs(
          Constants.Vision.k_visionStdDevX,
          Constants.Vision.k_visionStdDevY,
          Constants.Vision.k_visionStdDevTheta));

  private final PIDController k_rotController = new PIDController(
      Constants.AutoAim.Rotation.k_P,
      Constants.AutoAim.Rotation.k_I,
      Constants.AutoAim.Rotation.k_D);

  private boolean m_hasSetPose = false;

  private SwerveDrive() {
    super(m_modules, "SwerveDrive");

    k_rotController.enableContinuousInput(-Math.PI, Math.PI);

    m_rotationTarget = m_gyro.getRotation2d();

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
  public double calculateSpeakerAutoAimAngle(boolean degreeMode) {
    double botX = m_poseEstimator.getEstimatedPosition().getX();
    double botY = m_poseEstimator.getEstimatedPosition().getY();
    double targetX = Constants.Auto.k_blueCenterPose2d.getX(); // TODO: Work on red
    double targetY = Constants.Auto.k_blueCenterPose2d.getY();

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

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        m_modules[Module.FRONT_LEFT].getState(),
        m_modules[Module.FRONT_RIGHT].getState(),
        m_modules[Module.BACK_RIGHT].getState(),
        m_modules[Module.BACK_LEFT].getState());
  }

  public void driveLockedHeading(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean autoAim) {
    double rotationFeedback = 0.0;
    double rotationFF = rot;

    if (autoAim) {
      // m_rotationTarget = new Rotation2d(calculateSpeakerAutoAimAngle(false));
      m_rotationTarget = new Rotation2d(
          getPose().getTranslation().getX() - Constants.Field.k_blueSpeakerPose.getTranslation().getX(),
          getPose().getTranslation().getY() - Constants.Field.k_blueSpeakerPose.getTranslation().getY());
    }

    if (Math.abs(rot) > 0.03 * Constants.SwerveDrive.k_maxAngularSpeed) {
      m_rotationTarget = m_gyro.getRotation2d().plus(new Rotation2d(rot * (1.0 / 50.0)));
    } else {
      rotationFeedback = k_rotController.calculate(
          m_gyro.getRotation2d().getRadians(),
          m_rotationTarget.getRadians());
    }
    Logger.recordOutput("SwerveDrive/HeadingLock/RotFeedback", rotationFeedback);
    Logger.recordOutput("SwerveDrive/HeadingLock/RotFeedforward", rotationFF);
    Logger.recordOutput("SwerveDrive/HeadingLock/Target", m_rotationTarget);
    // rotationFF = rot; // k_rotController.getSetpoint().velocity;

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotationFeedback + rotationFF,
                m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rotationFeedback + rotationFF));

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
    m_rotationTarget = new Rotation2d(0.0); // jitter go brr
    // setGyroAngleAdjustment(0);
    setAllianceGyroAngleAdjustment();
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();
    LimelightHelpers.PoseEstimate leftPose = filteredLLPoseEstimate(m_limelightLeft.getPoseEstimation());
    LimelightHelpers.PoseEstimate rightPose = filteredLLPoseEstimate(m_limelightRight.getPoseEstimation());
    LimelightHelpers.PoseEstimate shooterPose = filteredLLPoseEstimate(m_limelightShooter.getPoseEstimation());

    // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,99999));
    // TODO: Check this, Limelight docs use these values

    if (leftPose.invalid == false) {
      m_poseEstimator.addVisionMeasurement(leftPose.pose, leftPose.timestampSeconds);
    }

    if (rightPose.invalid == false) {
      m_poseEstimator.addVisionMeasurement(rightPose.pose, rightPose.timestampSeconds);
    }

    if (shooterPose.invalid == false) {
      m_poseEstimator.addVisionMeasurement(shooterPose.pose, shooterPose.timestampSeconds);
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

  public Vector<N3> createStateStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  int tagThreshold = 1;
  double poseDistanceDiffThreshold = 10.0; // meters
  double robotSpeedThreshold = 1.0; // meters per second
  double maxTargetDistance = 4.0; // meters
  double maxLatency = 500; // milliseconds

  public PoseEstimate filteredLLPoseEstimate(PoseEstimate poseEstimate) {
    // Only use the pose if we can see a certain number of tags
    if (poseEstimate.tagCount < tagThreshold) {
      poseEstimate.invalid = true;
      return poseEstimate;
    }

    // Only use the pose if it's within a certain distance of our current pose
    if (poseEstimate.pose.getTranslation().getDistance(
        m_poseEstimator.getEstimatedPosition().getTranslation()) >= poseDistanceDiffThreshold) {
      poseEstimate.invalid = true;
      return poseEstimate;
    }

    // Only use the pose if the target is within a certain distance
    if (poseEstimate.avgTagDist >= maxTargetDistance) {
      poseEstimate.invalid = true;
      return poseEstimate;
    }

    // Only use the pose if we're not moving too fast (robot go weeeeeeee-Andrew)
    ChassisSpeeds chassisSpeeds = getChassisSpeeds();
    double robotSpeed = Math.sqrt(
        Math.pow(chassisSpeeds.vxMetersPerSecond, 2) +
            Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
    if (robotSpeed >= robotSpeedThreshold) {
      poseEstimate.invalid = true;
      return poseEstimate;
    }

    // TODO: add max rotation rate limiting

    // Only use the pose if the latency (ms) is within a certain threshold
    if (poseEstimate.latency >= maxLatency) {
      poseEstimate.invalid = true;
      return poseEstimate;
    }

    // Only use the pose if it's legally on the field
    if (poseEstimate.pose.getY() < 0 || poseEstimate.pose.getY() > Constants.Field.k_length) {
      poseEstimate.invalid = true;
      return poseEstimate;
    }
    if (poseEstimate.pose.getX() < 0 || poseEstimate.pose.getX() > Constants.Field.k_width) {
      poseEstimate.invalid = true;
      return poseEstimate;
    }

    return poseEstimate;
  }

  @AutoLogOutput
  public Pose2d getLLLeftCurrentPose() {
    return m_limelightLeft.getPoseEstimation().pose;
  }

  @AutoLogOutput
  public Pose2d getLLRightCurrentPose() {
    return m_limelightRight.getPoseEstimation().pose;
  }

  @AutoLogOutput
  public Pose2d getLLShooterCurrentPose() {
    return m_limelightShooter.getPoseEstimation().pose;
  }

  @AutoLogOutput
  public double getDistanceFromSpeaker() {
    Pose2d pose = getPose();
    Pose3d speakerPose = AllianceHelpers.getAllianceSpeakerPose3d();

    return Math.sqrt(Math.pow(pose.getX() - speakerPose.getX(), 2) +
        Math.pow(pose.getY() - speakerPose.getY(), 2));
  }
}
