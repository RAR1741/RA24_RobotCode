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
import frc.robot.AprilTagLocations;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.constants.ApolloConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.Limelight;

public class SwerveDrive extends SwerveSysId {
  private static SwerveDrive m_swerve = null;

  private Rotation2d m_rotationTarget;
  // private Pose2d m_accelerometerPose;

  private static final SwerveModule[] m_modules = {
      new SwerveModule(ApolloConstants.SwerveDrive.Drive.k_FLMotorId, ApolloConstants.SwerveDrive.Turn.k_FLMotorId,
          ApolloConstants.SwerveDrive.Turn.k_FLAbsId,
          ApolloConstants.SwerveDrive.Turn.k_FLOffset, "FL"), // 0
      new SwerveModule(ApolloConstants.SwerveDrive.Drive.k_FRMotorId, ApolloConstants.SwerveDrive.Turn.k_FRMotorId,
          ApolloConstants.SwerveDrive.Turn.k_FRAbsId,
          ApolloConstants.SwerveDrive.Turn.k_FROffset, "FR"), // 1
      new SwerveModule(ApolloConstants.SwerveDrive.Drive.k_BRMotorId, ApolloConstants.SwerveDrive.Turn.k_BRMotorId,
          ApolloConstants.SwerveDrive.Turn.k_BRAbsId,
          ApolloConstants.SwerveDrive.Turn.k_BROffset, "BR"), // 2
      new SwerveModule(ApolloConstants.SwerveDrive.Drive.k_BLMotorId, ApolloConstants.SwerveDrive.Turn.k_BLMotorId,
          ApolloConstants.SwerveDrive.Turn.k_BLAbsId,
          ApolloConstants.SwerveDrive.Turn.k_BLOffset, "BL") // 3
  };

  // Robot "forward" is +x
  // Robot "left" is +y
  // Robot "clockwise" is -z
  private final Translation2d[] m_moduleLocations = {
      new Translation2d(ApolloConstants.SwerveDrive.k_xCenterDistance, ApolloConstants.SwerveDrive.k_yCenterDistance),
      new Translation2d(ApolloConstants.SwerveDrive.k_xCenterDistance, -ApolloConstants.SwerveDrive.k_yCenterDistance),
      new Translation2d(-ApolloConstants.SwerveDrive.k_xCenterDistance, -ApolloConstants.SwerveDrive.k_yCenterDistance),
      new Translation2d(-ApolloConstants.SwerveDrive.k_xCenterDistance, ApolloConstants.SwerveDrive.k_yCenterDistance)
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
          ApolloConstants.Vision.k_positionStdDevX,
          ApolloConstants.Vision.k_positionStdDevY,
          ApolloConstants.Vision.k_positionStdDevTheta),
      createVisionMeasurementStdDevs(
          ApolloConstants.Vision.k_visionStdDevX,
          ApolloConstants.Vision.k_visionStdDevY,
          ApolloConstants.Vision.k_visionStdDevTheta));

  private final PIDController k_rotController = new PIDController(
      ApolloConstants.Vision.Rotation.k_P,
      ApolloConstants.Vision.Rotation.k_I,
      ApolloConstants.Vision.Rotation.k_D);

  enum VisionInstance {
    SHOOTER, LEFT, RIGHT;
  }

  public VisionConstants m_visionConstants = ApolloConstants.Vision.defaultAutoVisionConstants;

  private boolean m_hasSetPose = false;

  private SwerveDrive() {
    super(m_modules, "SwerveDrive");

    k_rotController.enableContinuousInput(-Math.PI, Math.PI);

    resetRotationTarget();
    // m_accelerometerPose = new Pose2d();

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
    double targetX = ApolloConstants.Auto.k_blueCenterPose2d.getX(); // TODO: Work on red
    double targetY = ApolloConstants.Auto.k_blueCenterPose2d.getY();

    double x = targetX - botX;
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(targetY - botY, 2));

    double theta = Math.acos(x / distance);

    putNumber("AutoAimAngle", Units.radiansToDegrees(theta));

    return degreeMode ? Units.radiansToDegrees(theta) : theta;
  }

  public double calculateAmpAutoAimAngle(boolean degreeMode) {
    double botX = m_poseEstimator.getEstimatedPosition().getX();
    double botY = m_poseEstimator.getEstimatedPosition().getY();
    double targetX = AprilTagLocations.Blue.k_ampTag6.getX(); // TODO: Work on red
    double targetY = AprilTagLocations.Blue.k_ampTag6.getY();

    double x = targetX - botX;
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(targetY - botY, 2));

    double theta = Math.acos(x / distance);

    putNumber("AutoAimAngle", Units.radiansToDegrees(theta));

    return degreeMode ? Units.radiansToDegrees(theta) : theta;
  }

  public void resetPose() {
    resetGyro();
    resetOdometry(new Pose2d(0, 0, new Rotation2d(0)), false, true);
  }

  public void resetOdometry(Pose2d pose) {
    resetOdometry(pose, true, false);
  }

  public void resetOdometry(Pose2d pose, boolean shouldSetFlag, boolean shouldResetEncoders) {
    m_hasSetPose |= shouldSetFlag;

    // setAllianceGyroAngleAdjustment();

    if (shouldResetEncoders) {
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
    } else {
      m_poseEstimator.resetPosition(
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              new SwerveModulePosition(m_modules[Module.FRONT_LEFT].getDrivePosition(),
                  Rotation2d.fromRotations(m_modules[Module.FRONT_LEFT].getTurnPosition())),
              new SwerveModulePosition(m_modules[Module.FRONT_RIGHT].getDrivePosition(),
                  Rotation2d.fromRotations(m_modules[Module.FRONT_RIGHT].getTurnPosition())),
              new SwerveModulePosition(m_modules[Module.BACK_RIGHT].getDrivePosition(),
                  Rotation2d.fromRotations(m_modules[Module.BACK_RIGHT].getTurnPosition())),
              new SwerveModulePosition(m_modules[Module.BACK_LEFT].getDrivePosition(),
                  Rotation2d.fromRotations(m_modules[Module.BACK_LEFT].getTurnPosition()))
          },
          pose);
    }

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

  public void pointForward() {
    for (SwerveModule module : m_modules) {
      module.pointForward();
    }
  }

  public void drive(ChassisSpeeds speeds) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ApolloConstants.SwerveDrive.k_maxBoostSpeed);

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

  public void driveLockedHeading(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean speakerAim,
      boolean ampAim, boolean passAim) {
    double rotationFeedback = 0.0;
    double rotationFF = rot;
    Rotation2d currentRotation;

    if (speakerAim) {
      m_rotationTarget = AllianceHelpers.getAllianceSpeakerRotationTarget();
      currentRotation = getPose().getRotation();
    } else if (ampAim) {
      m_rotationTarget = AllianceHelpers.getAllianceAmpRotation();
      currentRotation = getPose().getRotation();
    } else if (passAim) {
      m_rotationTarget = AllianceHelpers.getAlliancePassRotation();
      currentRotation = getPose().getRotation();
    } else {
      currentRotation = m_gyro.getRotation2d();
    }

    if (Math.abs(rot) > 0.03 * ApolloConstants.SwerveDrive.k_maxAngularSpeed) {
      m_rotationTarget = currentRotation.plus(new Rotation2d(rot * (1.0 / 50.0)));
    } else {
      rotationFeedback = k_rotController.calculate(
          currentRotation.getRadians(),
          m_rotationTarget.getRadians());
    }
    Logger.recordOutput("SwerveDrive/HeadingLock/RotFeedback", rotationFeedback);
    Logger.recordOutput("SwerveDrive/HeadingLock/RotFeedforward", rotationFF);
    Logger.recordOutput("SwerveDrive/HeadingLock/Target", m_rotationTarget);

    Logger.recordOutput("SwerveDrive/HeadingLock/AimingToSpeaker", speakerAim);
    Logger.recordOutput("SwerveDrive/HeadingLock/AimingToAmp", ampAim);
    Logger.recordOutput("SwerveDrive/HeadingLock/AimingToPass", passAim);

    drive(xSpeed, ySpeed, rotationFeedback + rotationFF, fieldRelative);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ApolloConstants.SwerveDrive.k_maxBoostSpeed);

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
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      m_rotationTarget = new Rotation2d(Math.PI); // jitter go brr
    } else {
      m_rotationTarget = new Rotation2d(0);
    }
    setAllianceGyroAngleAdjustment();
    // resetAccelerometerPose();
  }

  // set rotation target
  public void resetRotationTarget() {
    m_rotationTarget = m_gyro.getRotation2d();
  }

  public SwerveModule[] getModules() {
    return m_modules;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SwerveDrive/Limelights/Left/seesAprilTag", m_limelightLeft.seesAprilTag());
    Logger.recordOutput("SwerveDrive/Limelights/Right/seesAprilTag", m_limelightRight.seesAprilTag());
    Logger.recordOutput("SwerveDrive/Limelights/Shooter/seesAprilTag", m_limelightShooter.seesAprilTag());

    updateVisionPoseWithStdDev(m_limelightLeft.getPoseEstimation(), VisionInstance.LEFT);
    updateVisionPoseWithStdDev(m_limelightRight.getPoseEstimation(), VisionInstance.RIGHT);
    updateVisionPoseWithStdDev(m_limelightShooter.getPoseEstimation(), VisionInstance.SHOOTER);

    // clampPose(getPose());

    if (RobotBase.isReal()) {
      m_poseEstimator.updateWithTime(
          Timer.getFPGATimestamp(),
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

  public Vector<N3> createStateStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  public Vector<N3> createVisionMeasurementStdDevs(double x, double y, double theta) {
    return VecBuilder.fill(x, y, Units.degreesToRadians(theta));
  }

  private double xyStdDevCoefficient = 0.005;
  private double thetaStdDevCoefficient = 0.01;
  private double[] stdDevFactors = {
      0.6,
      2.0,
      2.0
  }; // shooter, left, right
  private boolean useVisionRotation = true;
  // private int minTagCount = m_visionConstants.minTagCount; // 1;
  // private double maxAvgDistance = m_visionConstants.maxAvgDistance; // 10.0;
  // private double autoStdDevScale = m_visionConstants.autoStdDevScale; // 16.0;

  public void updateVisionPoseWithStdDev(PoseEstimate poseEstimate, VisionInstance instanceIndex) {
    // Add observation to list
    double avgDistance = poseEstimate.avgTagDist;
    // double avgArea = poseEstimate.avgTagArea;

    if (poseEstimate.tagCount < m_visionConstants.minTagCount) {
      return;
    }

    if (avgDistance >= m_visionConstants.maxAvgDistance) {
      return;
    }

    if (poseEstimate.pose.getX() < 0 || poseEstimate.pose.getX() > RobotConstants.config.field().k_width) {
      return;
    }

    if (poseEstimate.pose.getY() < 0 || poseEstimate.pose.getY() > RobotConstants.config.field().k_length) {
      return;
    }

    if (getChassisSpeeds().vxMetersPerSecond > m_visionConstants.autoTranslationMax
        || getChassisSpeeds().vyMetersPerSecond > m_visionConstants.autoTranslationMax) {
      return;
    }

    double xyStdDev = xyStdDevCoefficient
        * Math.pow(avgDistance, 2.0)
        / poseEstimate.tagCount
        * stdDevFactors[instanceIndex.ordinal()]
        * (DriverStation.isAutonomous() ? m_visionConstants.autoStdDevScale : 1.0);

    double thetaStdDev = useVisionRotation
        ? thetaStdDevCoefficient
            * Math.pow(avgDistance, 2.0)
            / poseEstimate.tagCount
            * stdDevFactors[instanceIndex.ordinal()]
            * (DriverStation.isAutonomous() ? m_visionConstants.autoStdDevScale : 1.0)
        : Double.POSITIVE_INFINITY;

    m_poseEstimator.addVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds,
        createVisionMeasurementStdDevs(xyStdDev, xyStdDev, thetaStdDev));
  }

  // private Pose2d clampPose(Pose2d pose) {
  // return new Pose2d(MathUtil.clamp(pose.getX(), 0, Field.k_width),
  // MathUtil.clamp(pose.getY(), 0, Field.k_length),
  // getPose().getRotation());
  // }

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
  public boolean isAimedAtTarget() {
    double rotationError = Math.abs(m_rotationTarget.minus(getRotation2d()).getDegrees());
    Logger.recordOutput("Auto/AutoTarget/rotationError", rotationError);

    boolean isAtAimedAtTarget = rotationError < ApolloConstants.AutoAim.k_autoAimAngleTolerance;
    boolean isAtOmega = Math.abs(
        getChassisSpeeds().omegaRadiansPerSecond) < ApolloConstants.AutoAim.k_autoAimOmegaRPSThreshold;

    // RobotTelemetry.print(isAtAimedAtTarget + "|" + isAtOmega);

    return isAtAimedAtTarget && isAtOmega;
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

  public void setRotationTarget(Rotation2d target) {
    m_rotationTarget = target;
  }

  @AutoLogOutput
  public Rotation2d getRotationTarget() {
    return m_rotationTarget;
  }

  @AutoLogOutput
  public double getNavXTimestamp() {
    return (double) m_gyro.getLastSensorTimestamp();
  }

  @AutoLogOutput
  public double getAccelerometerVelocityX() {
    return m_gyro.getVelocityX();
  }

  @AutoLogOutput
  public double getAccelerometerVelocityY() {
    return m_gyro.getVelocityY();
  }

  @AutoLogOutput
  public double getAccelerometerVelocityZ() {
    return m_gyro.getVelocityZ();
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
