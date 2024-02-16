package frc.robot.subsystems.drivetrain;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;

public class SwerveModule {
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningRelEncoder;
  private final DutyCycleEncoder m_turningAbsEncoder;
  private final SparkPIDController m_turningPIDController;
  private final SparkPIDController m_drivePIDController;

  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private final double m_turningOffset;
  private final String m_moduleName;
  private final String m_smartDashboardKey;

  private static class PeriodicIO {
    SwerveModuleState desiredState = new SwerveModuleState();
    boolean shouldChangeState = false;
  }

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningAbsoluteID, double turningOffset,
      String moduleName) {
    m_turningOffset = turningOffset;
    m_moduleName = moduleName;

    m_smartDashboardKey = "SwerveDrive/" + m_moduleName + "/";

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder
        .setPositionConversionFactor(Units.inchesToMeters(Constants.SwerveDrive.k_wheelRadiusIn * 2.0 * Math.PI)
            / Constants.SwerveDrive.k_driveGearRatio);
    m_driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(
        (Constants.SwerveDrive.k_wheelRadiusIn * 2.0 * Math.PI) / Constants.SwerveDrive.k_driveGearRatio) / 60.0);

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    // m_turningMotor.burnFlash();
    m_turningMotor.setIdleMode(IdleMode.kCoast);

    m_turningAbsEncoder = new DutyCycleEncoder(turningAbsoluteID);

    m_turningAbsEncoder.setDistancePerRotation(Constants.SwerveDrive.k_turnGearRatio);

    m_turningRelEncoder = m_turningMotor.getEncoder();

    m_turningMotor.setInverted(true);
    m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_turningRelEncoder.setPositionConversionFactor(Constants.SwerveDrive.k_turnGearRatio * 2 * Math.PI);
    m_turningRelEncoder.setVelocityConversionFactor(Constants.SwerveDrive.k_turnGearRatio * 2 * Math.PI / 60);

    m_turningRelEncoder.setPosition(
        Helpers.modRadians(Units.rotationsToRadians(m_turningAbsEncoder.get() - m_turningOffset)));

    m_turningPIDController = m_turningMotor.getPIDController();
    m_turningPIDController.setP(Constants.SwerveDrive.Turn.k_turningP);
    m_turningPIDController.setI(Constants.SwerveDrive.Turn.k_turningI);
    m_turningPIDController.setD(Constants.SwerveDrive.Turn.k_turningD);
    m_turningPIDController.setIZone(Constants.SwerveDrive.Turn.k_turningIZone);
    m_turningPIDController.setFF(Constants.SwerveDrive.Turn.k_turningFF);
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(0.0);
    m_turningPIDController.setPositionPIDWrappingMaxInput(2.0 * Math.PI);
    m_turningPIDController.setOutputRange(
        Constants.SwerveDrive.Turn.k_TurningMinOutput,
        Constants.SwerveDrive.Turn.k_TurningMaxOutput);

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
    return Helpers.modRadians(m_turningRelEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    double drivePosition = m_driveEncoder.getPosition();

    return new SwerveModulePosition(
        drivePosition, Rotation2d.fromRadians(getTurnPosition()));
  }

  public CANSparkMax getDriveMotor() {
    return m_driveMotor;
  }

  // returns m/s
  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  // returns meters
  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  public void clearTurnPIDAccumulation() {
    m_turningPIDController.setIAccum(0);
  }

  public void resetDriveEncoder() {
    m_driveEncoder.setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurnPosition()));
    desiredState.angle = new Rotation2d(Helpers.modRadians(desiredState.angle.getRadians()));
    m_periodicIO.shouldChangeState = !desiredState.equals(m_periodicIO.desiredState);
    m_periodicIO.desiredState = desiredState;
  }

  // plumb voltage into drive motor and set turn motor to 0deg
  public void sysidDrive(double volts) {
    m_turningPIDController.setReference(0, ControlType.kPosition);

    m_driveMotor.setVoltage(volts);
  }

  public SwerveModuleState getDesiredState() {
    return m_periodicIO.desiredState;
  }

  public void periodic() {
    if (m_periodicIO.shouldChangeState) {
      m_drivePIDController.setReference(m_periodicIO.desiredState.speedMetersPerSecond, ControlType.kVelocity);
      m_turningPIDController.setReference(m_periodicIO.desiredState.angle.getRadians(), ControlType.kPosition);
      m_periodicIO.shouldChangeState = false;
    }
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnAbsPosition", Helpers.modRotations(m_turningAbsEncoder.get()));
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorPos", m_driveEncoder.getPosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorVelocity", getDriveVelocity());
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnMotorPosition", getTurnPosition());

    SmartDashboard.putNumber(m_smartDashboardKey + "TurnVoltage", Helpers.getVoltage(m_turningMotor));
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveTargetVelocity",
        m_periodicIO.desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnTargetAngleRadians",
        m_periodicIO.desiredState.angle.getRadians());
  }
}
