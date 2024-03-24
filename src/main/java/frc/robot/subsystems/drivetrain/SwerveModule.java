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
import frc.robot.Helpers;
import frc.robot.constants.ApolloConstants;

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
  @SuppressWarnings("unused")
  private final String m_moduleName;

  private static class PeriodicIO {
    SwerveModuleState desiredState = new SwerveModuleState();
    boolean shouldChangeState = false;
  }

  private boolean m_moduleDisabled = false;

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int turningAbsoluteID, double turningOffset,
      String moduleName) {
    m_turningOffset = turningOffset;
    m_moduleName = moduleName;

    m_drivingFeedForward = new SimpleMotorFeedforward(
        ApolloConstants.SwerveDrive.Drive.k_FFS,
        ApolloConstants.SwerveDrive.Drive.k_FFV,
        ApolloConstants.SwerveDrive.Drive.k_FFA);

    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_driveMotor.restoreFactoryDefaults();
    m_driveMotor.setIdleMode(IdleMode.kCoast);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder
        .setPositionConversionFactor(Units.inchesToMeters(ApolloConstants.SwerveDrive.k_wheelRadiusIn * 2.0 * Math.PI)
            / ApolloConstants.SwerveDrive.k_driveGearRatio);
    m_driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(
        (ApolloConstants.SwerveDrive.k_wheelRadiusIn * 2.0 * Math.PI) / ApolloConstants.SwerveDrive.k_driveGearRatio)
        / 60.0);
    m_driveMotor.setSmartCurrentLimit(ApolloConstants.SwerveDrive.Drive.k_currentLimit);

    m_turnMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turnMotor.restoreFactoryDefaults();
    m_turnMotor.setIdleMode(IdleMode.kCoast);
    m_turnMotor.setInverted(true);
    m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    m_turnMotor.setSmartCurrentLimit(ApolloConstants.SwerveDrive.Turn.k_currentLimit);

    m_turningAbsEncoder = new DutyCycleEncoder(turningAbsoluteID);

    m_turningRelEncoder = m_turnMotor.getEncoder();
    m_turningRelEncoder.setPositionConversionFactor(ApolloConstants.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI);
    m_turningRelEncoder.setVelocityConversionFactor(ApolloConstants.SwerveDrive.k_turnGearRatio * 2.0 * Math.PI / 60.0);
    resetTurnConfig();

    m_turningPIDController = m_turnMotor.getPIDController();
    m_turningPIDController.setP(ApolloConstants.SwerveDrive.Turn.k_P);
    m_turningPIDController.setI(ApolloConstants.SwerveDrive.Turn.k_I);
    m_turningPIDController.setD(ApolloConstants.SwerveDrive.Turn.k_D);
    m_turningPIDController.setIZone(ApolloConstants.SwerveDrive.Turn.k_IZone);
    m_turningPIDController.setFF(ApolloConstants.SwerveDrive.Turn.k_FFS);

    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(0.0);
    m_turningPIDController.setPositionPIDWrappingMaxInput(2.0 * Math.PI);
    m_turningPIDController.setOutputRange(
        ApolloConstants.SwerveDrive.Turn.k_TurningMinOutput,
        ApolloConstants.SwerveDrive.Turn.k_TurningMaxOutput);

    m_drivePIDController = m_driveMotor.getPIDController();
    m_drivePIDController.setP(ApolloConstants.SwerveDrive.Drive.k_P);
    m_drivePIDController.setI(ApolloConstants.SwerveDrive.Drive.k_I);
    m_drivePIDController.setD(ApolloConstants.SwerveDrive.Drive.k_D);
    m_drivePIDController.setIZone(ApolloConstants.SwerveDrive.Drive.k_IZone);
    // m_drivePIDController.setFF(Constants.SwerveDrive.Drive.k_FF);

    m_driveMotor.burnFlash();
    m_turnMotor.burnFlash();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), Rotation2d.fromRadians(getTurnPosition()));
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

  public void clearTurnPIDAccumulation() {
    m_turningPIDController.setIAccum(0);
  }

  public void resetDriveEncoder() {
    m_driveEncoder.setPosition(0.0);
  }

  public void resetTurnConfig() {
    if (getAsbEncoderIsConnected()) {
      m_turningRelEncoder.setPosition(
          Helpers.modRadians(Units.rotationsToRadians(m_turningAbsEncoder.getAbsolutePosition() - m_turningOffset)));
      m_moduleDisabled = false;
    } else {
      m_moduleDisabled = true;
    }

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

  public void pointForward() {
    m_periodicIO.desiredState.speedMetersPerSecond = 0.0;
    m_periodicIO.desiredState.angle = new Rotation2d(0.0);
    m_periodicIO.desiredState = SwerveModuleState.optimize(m_periodicIO.desiredState,
        Rotation2d.fromRadians(getTurnPosition()));
    m_periodicIO.shouldChangeState = true;
  }

  public void periodic() {
    if (m_periodicIO.shouldChangeState) {
      if (!m_moduleDisabled) {
        double feedforward = m_drivingFeedForward.calculate(getDriveTargetVelocity());

        m_drivePIDController.setReference(getDriveTargetVelocity(), ControlType.kVelocity, 0, feedforward);
        m_turningPIDController.setReference(getTurnTargetAngleRadians(), ControlType.kPosition);
      } else {
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_turnMotor.setIdleMode(IdleMode.kCoast);

        m_drivePIDController.setReference(0, ControlType.kVoltage);
        m_turningPIDController.setReference(0, ControlType.kVoltage);
      }

      m_periodicIO.shouldChangeState = false;
    }
  }

  // Logged
  @AutoLogOutput
  private double getTurnTargetAngleRadians() {
    return m_periodicIO.desiredState.angle.getRadians();
  }

  @AutoLogOutput
  private double getDriveTargetVelocity() {
    return m_periodicIO.desiredState.speedMetersPerSecond;
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

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Abs/getTurnPosition")
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

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Velocity")
  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Drive/Position")
  public double getDrivePosition() {
    return m_driveEncoder.getPosition();
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Position")
  public double getTurnPosition() {
    return Helpers.modRadians(m_turningRelEncoder.getPosition());
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/absPosition")
  public double getTurnAbsPosition() {
    return Helpers.modRotations(m_turningAbsEncoder.getAbsolutePosition() - m_turningOffset);
  }

  @AutoLogOutput(key = "SwerveDrive/Modules/{m_moduleName}/Turn/Velocity")
  public double getTurnVelocity() {
    return m_turningRelEncoder.getVelocity();
  }
}
