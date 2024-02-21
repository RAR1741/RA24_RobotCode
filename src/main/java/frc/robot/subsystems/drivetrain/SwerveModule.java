package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private final CANSparkMax m_turnMotor;
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningRelEncoder;
  private final DutyCycleEncoder m_turningAbsEncoder;
  private final SimpleMotorFeedforward m_drivingFeedForward;
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

    m_drivingFeedForward = new SimpleMotorFeedforward(
        Constants.SwerveDrive.Drive.k_FFS,
        Constants.SwerveDrive.Drive.k_FFV,
        Constants.SwerveDrive.Drive.k_FFA);

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder
        .setPositionConversionFactor(Units.inchesToMeters(Constants.SwerveDrive.k_wheelRadiusIn * 2.0 * Math.PI)
            / Constants.SwerveDrive.k_driveGearRatio);
    m_driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(
        (Constants.SwerveDrive.k_wheelRadiusIn * 2.0 * Math.PI) / Constants.SwerveDrive.k_driveGearRatio) / 60.0);
    m_driveMotor.setSmartCurrentLimit(Constants.SwerveDrive.Drive.k_currentLimit);

    m_turnMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turnMotor.restoreFactoryDefaults();
    m_turnMotor.setIdleMode(IdleMode.kCoast);
    m_turnMotor.setInverted(true);
    m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_turnMotor.setSmartCurrentLimit(Constants.SwerveDrive.Turn.k_currentLimit);

    m_turningAbsEncoder = new DutyCycleEncoder(turningAbsoluteID);

    m_turningRelEncoder = m_turnMotor.getEncoder();
    m_turningRelEncoder.setPositionConversionFactor(Constants.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI);
    m_turningRelEncoder.setVelocityConversionFactor(Constants.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI / 60.0);
    m_turningRelEncoder.setPosition(
        Helpers.modRadians(Units.rotationsToRadians(m_turningAbsEncoder.getAbsolutePosition() - m_turningOffset)));

    m_turningPIDController = m_turnMotor.getPIDController();
    m_turningPIDController.setP(Constants.SwerveDrive.Turn.k_P);
    m_turningPIDController.setI(Constants.SwerveDrive.Turn.k_I);
    m_turningPIDController.setD(Constants.SwerveDrive.Turn.k_D);
    m_turningPIDController.setIZone(Constants.SwerveDrive.Turn.k_IZone);
    m_turningPIDController.setFF(Constants.SwerveDrive.Turn.k_FFS);

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
    // m_drivePIDController.setFF(Constants.SwerveDrive.Drive.k_FF);

    // m_driveMotor.burnFlash();
    // m_turnMotor.burnFlash();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), Rotation2d.fromRadians(getTurnPosition()));
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Position")
  public double getTurnPosition() {
    return Helpers.modRadians(m_turningRelEncoder.getPosition());
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/absPosition")
  public double getTurnAbsPosition() {
    return Helpers.modRotations(m_turningAbsEncoder.getAbsolutePosition() - m_turningOffset);
  }

  public SwerveModulePosition getPosition() {
    double drivePosition = m_driveEncoder.getPosition();

    return new SwerveModulePosition(
        drivePosition, Rotation2d.fromRadians(getTurnPosition()));
  }

  public CANSparkMax getDriveMotor() {
    return m_driveMotor;
  }

  public CANSparkMax getTurnMotor() {
    return m_turnMotor;
  }

  public double getTurnVelocity() {
    return m_turningRelEncoder.getVelocity();
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

  // Pass voltage into drive motor and set turn motor to 0 deg
  public void sysidDrive(double volts) {
    m_turningPIDController.setReference(0, ControlType.kPosition);

    m_driveMotor.setVoltage(volts);
  }

  // Pass voltage into turn motor and set drive motor to 0 volts⚡
  public void sysidTurn(double volts) {
    m_drivePIDController.setReference(0, ControlType.kVoltage);

    m_turnMotor.setVoltage(volts);
  }

  public SwerveModuleState getDesiredState() {
    return m_periodicIO.desiredState;
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Voltage")
  public double getTurnMotorVoltage() {
    return Helpers.getVoltage(m_turnMotor);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Current")
  public double getTurnMotorCurrent() {
    return m_turnMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Voltage")
  public double getDriveMotorVoltage() {
    return Helpers.getVoltage(m_driveMotor);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Current")
  public double getDriveMotorCurrent() {
    return m_driveMotor.getOutputCurrent();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Abs/Frequency")
  public int getAsbEncoderFrequency() {
    return m_turningAbsEncoder.getFrequency();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Abs/isConnected")
  public boolean getAsbEncoderIsConnected() {
    return m_turningAbsEncoder.isConnected();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Abs/getPosition")
  public double getAsbEncoderPosition() {
    return m_turningAbsEncoder.getAbsolutePosition() - m_turningOffset;
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Temperature")
  public double getDriveTemp() {
    return m_driveMotor.getMotorTemperature();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Temperature")
  public double getTurnTemp() {
    return m_turnMotor.getMotorTemperature();
  }

  public void periodic() {
    if (m_periodicIO.shouldChangeState) {
      double feedforward = m_drivingFeedForward.calculate(m_periodicIO.desiredState.speedMetersPerSecond);

      m_drivePIDController.setReference(m_periodicIO.desiredState.speedMetersPerSecond, ControlType.kVelocity, 0,
          feedforward);
      m_turningPIDController.setReference(m_periodicIO.desiredState.angle.getRadians(), ControlType.kPosition);
      m_periodicIO.shouldChangeState = false;
    }
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnAbsPosition",
        Helpers.modRotations(m_turningAbsEncoder.getAbsolutePosition()));
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorPos", m_driveEncoder.getPosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorVelocity", getDriveVelocity());
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnMotorPosition", getTurnPosition());

    SmartDashboard.putNumber(m_smartDashboardKey + "TurnVoltage", Helpers.getVoltage(m_turnMotor));
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveTargetVelocity",
        m_periodicIO.desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnTargetAngleRadians",
        m_periodicIO.desiredState.angle.getRadians());
  }
}
