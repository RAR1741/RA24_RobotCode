package frc.robot.subsystems.drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningRelEncoder;
  private final AbsoluteEncoder m_turningAbsEncoder;
  private final SparkPIDController m_turningPIDController;
  private final SparkPIDController m_drivePIDController;

  // private final double k_turnFeedForwardS = 1.0;
  // private final double k_turnFeedForwardV = 0.5;
  // private final double k_turnFeedForwardA = 0.0;
  // private final SimpleMotorFeedforward m_turnFeedforward = new
  // SimpleMotorFeedforward(k_turnFeedForwardS,
  // k_turnFeedForwardV, k_turnFeedForwardA);

  // private final double k_driveFeedForwardS = 1.0;
  // private final double k_driveFeedForwardV = 0.5;
  // private final double k_driveFeedForwardA = 0.0;
  // private final SimpleMotorFeedforward m_driveFeedforward = new
  // SimpleMotorFeedforward(k_driveFeedForwardS,
  // k_driveFeedForwardV, k_driveFeedForwardA);

  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private final double m_turningOffset;
  private final String m_moduleName;
  private final String m_smartDashboardKey;

  private static class PeriodicIO {
    SwerveModuleState desiredState;
  }

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, double turningOffset, String moduleName) {
    m_turningOffset = turningOffset;
    m_moduleName = moduleName;

    m_smartDashboardKey = "SwerveDrive/" + m_moduleName + "/";

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(Units.inchesToMeters(Constants.SwerveDrive.k_wheelRadiusIn * 2 * Math.PI)
        / Constants.SwerveDrive.k_driveGearRatio);
    m_driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(
        (Constants.SwerveDrive.k_wheelRadiusIn * 2 * Math.PI) / Constants.SwerveDrive.k_driveGearRatio) / 60);

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setIdleMode(IdleMode.kCoast);

    m_turningAbsEncoder = m_turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    // TODO: use m_turningAbsEncoder.getPosition() to set the offset for the
    // m_turningRelEncoder

    m_turningRelEncoder = m_turningMotor.getEncoder();
    m_turningMotor.setInverted(true);
    m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_turningRelEncoder.setPositionConversionFactor(Constants.SwerveDrive.k_turnGearRatio * 2 * Math.PI);
    m_turningRelEncoder.setVelocityConversionFactor(Constants.SwerveDrive.k_turnGearRatio * 2 * Math.PI / 60);

    m_turningPIDController = m_turningMotor.getPIDController();
    m_turningPIDController.setP(Constants.SwerveDrive.Turn.k_turningP);
    m_turningPIDController.setI(Constants.SwerveDrive.Turn.k_turningI);
    m_turningPIDController.setD(Constants.SwerveDrive.Turn.k_turningD);
    m_turningPIDController.setIZone(Constants.SwerveDrive.Turn.k_turningIZone);
    m_turningPIDController.setFF(Constants.SwerveDrive.Turn.k_turningFF);
    m_turningPIDController.setOutputRange(
        Constants.SwerveDrive.Turn.k_TurningMinOutput, Constants.SwerveDrive.Turn.k_TurningMinOutput);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setP(Constants.SwerveDrive.Drive.k_P);
    m_drivePIDController.setI(Constants.SwerveDrive.Drive.k_I);
    m_drivePIDController.setD(Constants.SwerveDrive.Drive.k_D);
    m_drivePIDController.setIZone(Constants.SwerveDrive.Drive.k_IZone);
    m_drivePIDController.setFF(Constants.SwerveDrive.Drive.k_FF);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), Rotation2d.fromRadians(getTurnPosition()));
  }

  public double getTurnPosition() {
    return Helpers.modRadians(m_turningRelEncoder.getPosition() - m_turningOffset);
  }

  public SwerveModulePosition getPosition() {
    double drivePosition = m_driveEncoder.getPosition();

    return new SwerveModulePosition(
        drivePosition, Rotation2d.fromRadians(getTurnPosition()));
  }

  public CANSparkMax getDriveMotor() {
    return m_driveMotor;
  }

  public double getDriveVelocity() {
    // In revs per minute
    double velocity = m_driveEncoder.getVelocity();

    // Convert to revs per second
    velocity = velocity / 60;

    // Convert to in per second
    velocity *= ((2.0 * Constants.SwerveDrive.k_wheelRadiusIn * Math.PI) / Constants.SwerveDrive.k_driveGearRatio);

    // Convert to m per second
    velocity = Units.inchesToMeters(velocity);

    return velocity;
  }

  public void clearTurnPIDAccumulation() {
    m_turningPIDController.setIAccum(0); // TODO: Make sure this works
  }

  public void resetDriveEncoder() {
    m_driveEncoder.setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurnPosition()));
    m_periodicIO.desiredState = desiredState;

    // If the turn position changes, set turn motor accordingly
    // if (m_turningEncoder.getPosition() !=
    // m_periodicIO.desiredState.angle.getRotations()) {
    // m_turnPIDController.setReference(m_periodicIO.desiredState.angle.getRotations(),
    // ControlType.kPosition);
    // }

    m_drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningPIDController.setReference(m_periodicIO.desiredState.angle.getRadians(), ControlType.kPosition);

    SmartDashboard.putNumber(m_smartDashboardKey + "DriveTargetVelocity",
        m_periodicIO.desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnTargetAngleRadians",
        m_periodicIO.desiredState.angle.getRadians());
  }

  public SwerveModuleState getDesiredState() {
    return m_periodicIO.desiredState;
  }

  public void periodic() {
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorPos", m_driveEncoder.getPosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorVelocity", getDriveVelocity());
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnMotorPosition", getTurnPosition());
  }
}