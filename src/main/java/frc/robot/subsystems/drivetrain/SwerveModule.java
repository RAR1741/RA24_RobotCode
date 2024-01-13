package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Helpers;

public class SwerveModule {
  private final TalonFX m_driveMotor;

  private final CANSparkMax m_turningMotor;
  private final AbsoluteEncoder m_turningEncoder;

  private final PhoenixPIDController m_drivePIDController;
  private final SparkPIDController m_turningPIDController;

  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private final double m_turningOffset;
  private final String m_moduleName;
  private final String m_smartDashboardKey;
  
  private static final double k_wheelRadiusIn = 2; // 2 inches
  private static final double k_driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);

  private static final double k_turningP = 20.0;
  private static final double k_turningI = 0.1;
  private static final double k_turningD = 0.01;
  
  private static final double k_turnFeedForwardS = 1;
  private static final double k_turnFeedForwardV = 0.5;
  private static final double k_turnFeedForwardA = 0.0;

  private static final double k_driveP = 0.86853;
  private static final double k_driveI = 0.0;
  private static final double k_driveD = 0.0;

  private static final double k_driveFeedForwardS = 0.25043;
  private static final double k_driveFeedForwardV = 3.0125;
  private static final double k_driveFeedForwardA = 0.38005;

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
      k_driveFeedForwardS,
      k_driveFeedForwardV,
      k_driveFeedForwardA);

  // TODO: Gains are for example purposes only - must be determined for your own
  // robot!
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(k_turnFeedForwardS,
      k_turnFeedForwardV, k_turnFeedForwardA);

  private static class PeriodicIO {
    double turnMotorVoltage = 0.0;
    double driveMotorVoltage = 0.0;
    SwerveModuleState desiredState;
  }

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, double turningOffset, String moduleName) {
    m_turningOffset = turningOffset;
    m_moduleName = moduleName;

    m_smartDashboardKey = "SwerveDrive/" + m_moduleName + "/";

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_driveMotor.getConfigurator().apply(new TalonFXConfiguration());

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setIdleMode(IdleMode.kBrake);
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_turningMotor.setInverted(true);

    m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    m_turningPIDController = m_turningMotor.getPIDController();
    m_drivePIDController = new PhoenixPIDController(k_driveP, k_driveI, k_driveD);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveVelocity(), Rotation2d.fromRotations(getTurnPosition()));
  }

  public double getTurnPosition() {
    return Helpers.modRotations(m_turningEncoder.getPosition() - m_turningOffset);
  }

  public SwerveModulePosition getPosition() {
    double drivePosition = m_driveMotor.getPosition().getValueAsDouble();
    drivePosition *= ((2 * k_wheelRadiusIn * Math.PI) / k_driveGearRatio); // Convert to inches
    drivePosition = Units.inchesToMeters(drivePosition);

    return new SwerveModulePosition(
        drivePosition, Rotation2d.fromRotations(getTurnPosition()));
  }

  public TalonFX getDriveMotor() {
    return m_driveMotor;
  }

  public double getDriveVelocity() {
    // In revs per second
    double velocity = m_driveMotor.getVelocity().getValue(); 

    // Convert to in per second
    velocity *= ((2.0 * k_wheelRadiusIn * Math.PI) / k_driveGearRatio);

    // Convert to m per second
    velocity = Units.inchesToMeters(velocity);

    return velocity;
  }

  public void clearTurnPIDAccumulation() {
    
  }

  public void resetDriveEncoder() {
    m_driveMotor.setPosition(0.0, 50);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(getTurnPosition()));
    m_periodicIO.desiredState = desiredState;
    // Calculate the drive output from the drive PID controller.
    double driveOutput = m_drivePIDController.calculate(getDriveVelocity(), m_periodicIO.desiredState.speedMetersPerSecond, );
    double driveFeedforward = m_driveFeedforward.calculate(m_periodicIO.desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    double turnTarget = m_periodicIO.desiredState.angle.getRotations();
    double turnOutput = m_turningPIDController.calculate(getTurnPosition(), turnTarget);
    double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    boolean turnAtGoal = m_turningPIDController.atGoal();

    SmartDashboard.putNumber(m_smartDashboardKey + "TurnTarget", turnTarget);
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnOutput", turnOutput + turnFeedforward);
    SmartDashboard.putBoolean(m_smartDashboardKey + "TurnAtGoal", turnAtGoal);
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveTargetVelocity", m_periodicIO.desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveOutput", driveOutput + driveFeedforward);

    m_periodicIO.turnMotorVoltage = turnOutput + turnFeedforward;
    m_periodicIO.driveMotorVoltage = driveOutput + driveFeedforward;
  }

  public SwerveModuleState getDesiredState() {
    return m_periodicIO.desiredState;
  }

  public void periodic() {
    m_turningMotor.setVoltage(m_periodicIO.turnMotorVoltage);
    m_driveMotor.setVoltage(m_periodicIO.driveMotorVoltage);
  }

  public void outputTelemetry() {
    SmartDashboard.putString(m_smartDashboardKey + "DriveMotorPositionUnits", m_driveMotor.getPosition().getUnits());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorPos", m_driveMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorVelocity", getDriveVelocity());
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnMotorPosition", getTurnPosition());
  }
}