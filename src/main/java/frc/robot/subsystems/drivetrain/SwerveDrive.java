package frc.robot.subsystems.drivetrain;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class SwerveDrive {
  private static SwerveDrive m_swerve = null;

  private final ArrayList<SwerveModule> m_modules = new ArrayList<>(Arrays.asList(
      new SwerveModule(Constants.Drivetrain.Drive.k_FLMotorId, Constants.Drivetrain.Turn.k_FLMotorId,
          Constants.Drivetrain.Turn.k_FLOffset, "FL"), // 0
      new SwerveModule(Constants.Drivetrain.Drive.k_FRMotorId, Constants.Drivetrain.Turn.k_FRMotorId,
          Constants.Drivetrain.Turn.k_FROffset, "FR"), // 1
      new SwerveModule(Constants.Drivetrain.Drive.k_BLMotorId, Constants.Drivetrain.Turn.k_BLMotorId,
          Constants.Drivetrain.Turn.k_BLOffset, "BL"), // 2
      new SwerveModule(Constants.Drivetrain.Drive.k_BRMotorId, Constants.Drivetrain.Turn.k_BRMotorId,
          Constants.Drivetrain.Turn.k_BROffset, "BR") // 3
  ));

  // Robot "forward" is +x
  // Robot "left" is +y
  // Robot "clockwise" is -z
  private final ArrayList<Translation2d> m_moduleLocations = new ArrayList<>(Arrays.asList(
    new Translation2d(Constants.Drivetrain.k_xCenterDistance,
      Constants.Drivetrain.k_yCenterDistance),
    new Translation2d(Constants.Drivetrain.k_xCenterDistance,
        -Constants.Drivetrain.k_yCenterDistance),
    new Translation2d(-Constants.Drivetrain.k_xCenterDistance,
        Constants.Drivetrain.k_yCenterDistance),
    new Translation2d(-Constants.Drivetrain.k_xCenterDistance,
        -Constants.Drivetrain.k_yCenterDistance)
  ));

  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    m_moduleLocations.get(Module.FRONT_LEFT),
    m_moduleLocations.get(Module.FRONT_RIGHT),
    m_moduleLocations.get(Module.BACK_LEFT),
    m_moduleLocations.get(Module.BACK_RIGHT));

  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
    m_kinematics,
    m_gyro.getRotation2d(),
    new SwerveModulePosition[] {
      m_modules.get(Module.FRONT_LEFT).getPosition(),
      m_modules.get(Module.FRONT_RIGHT).getPosition(),
      m_modules.get(Module.BACK_LEFT).getPosition(),
      m_modules.get(Module.BACK_RIGHT).getPosition()
    },
    new Pose2d(0, 0, Rotation2d.fromDegrees(0))
  );

  private SwerveDrive() {
    reset();
  }

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
        m_modules.get(Module.FRONT_LEFT).getPosition(),
        m_modules.get(Module.FRONT_RIGHT).getPosition(),
        m_modules.get(Module.BACK_LEFT).getPosition(),
        m_modules.get(Module.BACK_RIGHT).getPosition()
      },
      pose);
  }

  public void resetPose() {
    resetGyro();
    resetOdometry(new Pose2d(0,0,new Rotation2d(0)));
  }

  public void resetOdometry(Pose2d pose) {
    for (SwerveModule module : m_modules) {
      module.resetDriveEncoder();
    }
    
    m_poseEstimator.resetPosition(
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
        new SwerveModulePosition(0.0,
          Rotation2d.fromRotations(m_modules.get(Module.FRONT_LEFT).getTurnPosition())),
        new SwerveModulePosition(0.0,
          Rotation2d.fromRotations(m_modules.get(Module.FRONT_RIGHT).getTurnPosition())),
        new SwerveModulePosition(0.0,
          Rotation2d.fromRotations(m_modules.get(Module.BACK_LEFT).getTurnPosition())),
        new SwerveModulePosition(0.0,
          Rotation2d.fromRotations(m_modules.get(Module.BACK_RIGHT).getTurnPosition()))
      },
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public Rotation2d getRotation2d() {
    return m_gyro.getRotation2d();
  }

  public void reset() {
    setBrakeMode(false);
    resetPose();
  }

  public void resetGyro() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(0);
  }

  public static SwerveDrive getInstance() {
    if (m_swerve == null) {
      m_swerve = new SwerveDrive();
    }
    return m_swerve;
  }

  public interface Module {
    int FRONT_LEFT = 0;
    int FRONT_RIGHT = 1;
    int BACK_LEFT = 2;
    int BACK_RIGHT = 3;
  }
}
