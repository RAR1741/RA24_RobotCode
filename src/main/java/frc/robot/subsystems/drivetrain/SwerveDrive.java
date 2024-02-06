package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Subsystem;

public class SwerveDrive extends Subsystem {
  private static SwerveDrive m_swerve = null;

  private final SwerveModule[] m_modules = {
      new SwerveModule(Constants.SwerveDrive.Drive.k_FLMotorId, Constants.SwerveDrive.Turn.k_FLMotorId,
          Constants.SwerveDrive.Turn.k_FLAbsID,
          Constants.SwerveDrive.Turn.k_FLOffset, "FL"), // 0
      new SwerveModule(Constants.SwerveDrive.Drive.k_FRMotorId, Constants.SwerveDrive.Turn.k_FRMotorId,
          Constants.SwerveDrive.Turn.k_FRAbsID,
          Constants.SwerveDrive.Turn.k_FROffset, "FR"), // 1
      new SwerveModule(Constants.SwerveDrive.Drive.k_BLMotorId, Constants.SwerveDrive.Turn.k_BLMotorId,
          Constants.SwerveDrive.Turn.k_BLAbsID,
          Constants.SwerveDrive.Turn.k_BLOffset, "BL"), // 2
      new SwerveModule(Constants.SwerveDrive.Drive.k_BRMotorId, Constants.SwerveDrive.Turn.k_BRMotorId,
          Constants.SwerveDrive.Turn.k_BRAbsID,
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
  private final Limelight m_limelightOne = new Limelight("limelight-one");
  private final Limelight m_limelightTwo = new Limelight("limelight-two");

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

  // Mutable holder for unit-safe SysID values (to avoid reallocation)
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  private final SysIdRoutine m_sysIdRoutine;

  private SwerveDrive() {
    super("SwerveDrive");

    reset();

    m_sysIdRoutine = new SysIdRoutine( // i hate this constructor
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
            // Tell SysId how to plumb the driving voltage to the motors.
            (Measure<Voltage> volts) -> {
              for (SwerveModule module: m_modules) {
                module.sysidDrive(volts.in(Volts));
              }
            },
            // Tell SysId how to record a frame of data for each motor on the mechanism
            // being characterized.
            log -> {
              // Record a frame for the front left
              log.motor("drive-frontleft")
                  .voltage(m_appliedVoltage.mut_replace(
                      m_modules[Module.FRONT_LEFT].getDriveMotor().getAppliedOutput() * RobotController.getBatteryVoltage(),
                      Volts))
                  .linearPosition(m_distance.mut_replace(m_modules[Module.FRONT_LEFT].getDrivePosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_modules[Module.FRONT_LEFT].getDriveVelocity(), MetersPerSecond));

              // Record a frame for the front left
              log.motor("drive-frontright")
                  .voltage(m_appliedVoltage.mut_replace(
                      m_modules[Module.FRONT_RIGHT].getDriveMotor().getAppliedOutput() * RobotController.getBatteryVoltage(),
                      Volts))
                  .linearPosition(m_distance.mut_replace(m_modules[Module.FRONT_RIGHT].getDrivePosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_modules[Module.FRONT_RIGHT].getDriveVelocity(), MetersPerSecond));

              // Record a frame for the back left
              log.motor("drive-backleft")
                  .voltage(m_appliedVoltage.mut_replace(
                      m_modules[Module.BACK_LEFT].getDriveMotor().getAppliedOutput() * RobotController.getBatteryVoltage(),
                      Volts))
                  .linearPosition(m_distance.mut_replace(m_modules[Module.BACK_LEFT].getDrivePosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_modules[Module.BACK_LEFT].getDriveVelocity(), MetersPerSecond));

              // Record a frame for the front left
              log.motor("drive-backright")
                  .voltage(m_appliedVoltage.mut_replace(
                      m_modules[Module.BACK_RIGHT].getDriveMotor().getAppliedOutput() * RobotController.getBatteryVoltage(),
                      Volts))
                  .linearPosition(m_distance.mut_replace(m_modules[Module.BACK_RIGHT].getDrivePosition(), Meters))
                  .linearVelocity(
                      m_velocity.mut_replace(m_modules[Module.BACK_RIGHT].getDriveVelocity(), MetersPerSecond));
            },
            // Tell SysId to make generated commands require this subsystem, suffix test
            // state in WPILog with this subsystem's name ("drive")
            this));
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

  /**
   * @param degreeMode If <code>true</code>, return result in degrees; otherwise,
   *                   return in radians
   */
  public double calculateAutoAimAngle(boolean degreeMode) {
    double bot_x = m_poseEstimator.getEstimatedPosition().getX();
    double bot_y = m_poseEstimator.getEstimatedPosition().getY();
    double speaker_x = Constants.Field.k_redSpeakerPose.getX();
    double speaker_y = Constants.Field.k_redSpeakerPose.getY();

    double x = speaker_x - bot_x;
    double distance = Math.sqrt(Math.pow(x, 2) + Math.pow(speaker_y - bot_y, 2));

    double theta = Math.acos(x / distance);

    putNumber("AutoAimAngle", Math.toDegrees(theta));
    // System.out.println(theta);

    return degreeMode ? Units.radiansToDegrees(theta) : theta;
    // arccosine(x/d)
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

  public void pointInwards() {
    m_modules[Module.FRONT_LEFT].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_modules[Module.FRONT_RIGHT].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_modules[Module.BACK_LEFT].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_modules[Module.BACK_RIGHT].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
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
        m_modules[Module.FRONT_LEFT].getDesiredState().angle.getDegrees(),
        m_modules[Module.FRONT_LEFT].getDesiredState().speedMetersPerSecond,
        m_modules[Module.FRONT_RIGHT].getDesiredState().angle.getDegrees(),
        m_modules[Module.FRONT_RIGHT].getDesiredState().speedMetersPerSecond,
        m_modules[Module.BACK_LEFT].getDesiredState().angle.getDegrees(),
        m_modules[Module.FRONT_LEFT].getDesiredState().speedMetersPerSecond,
        m_modules[Module.BACK_RIGHT].getDesiredState().angle.getDegrees(),
        m_modules[Module.BACK_RIGHT].getDesiredState().speedMetersPerSecond
    };

    return desiredStates;
  }

  @Override
  public void outputTelemetry() {
    double currentTime = Timer.getFPGATimestamp();

    if (m_limelightOne.seesAprilTag()) {
      m_poseEstimator.addVisionMeasurement(m_limelightOne.getBotpose2D(), currentTime);
    }

    if (m_limelightTwo.seesAprilTag()) {
      m_poseEstimator.addVisionMeasurement(m_limelightTwo.getBotpose2D(), currentTime);
    }

    if (RobotBase.isReal()) {
      m_poseEstimator.updateWithTime(
          currentTime,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
              m_modules[Module.FRONT_LEFT].getPosition(),
              m_modules[Module.FRONT_RIGHT].getPosition(),
              m_modules[Module.BACK_LEFT].getPosition(),
              m_modules[Module.BACK_RIGHT].getPosition()
          });
    } else {
      setPose(new Pose2d(
          Preferences.getDouble("SwerveDrive/x", 0),
          Preferences.getDouble("SwerveDrive/y", 0),
          new Rotation2d(Math.toRadians(Preferences.getDouble("SwerveDrive/rot", 0)))));
    }

    for (SwerveModule module : m_modules) {
      module.outputTelemetry();
    }

    putNumberArray("CurrentStates", getCurrentStates());
    putNumberArray("DesiredStates", getDesiredStates());

    putNumber("Gyro/AngleDegrees", m_gyro.getRotation2d().getDegrees());
    putNumber("Gyro/Pitch", m_gyro.getPitch());
    putNumberArray("Pose",
        new double[] { getPose().getX(), getPose().getY(), getPose().getRotation().getDegrees() });
  }

  public interface Module {
    int FRONT_LEFT = 0;
    int FRONT_RIGHT = 1;
    int BACK_LEFT = 2;
    int BACK_RIGHT = 3;
  }
}
