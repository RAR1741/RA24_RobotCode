package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private VelocityVoltage m_driveVoltage;

  private final CANSparkMax m_turningMotor;
  private final AbsoluteEncoder m_turningEncoder;
  private final SparkPIDController m_turningPIDController;

  private final PeriodicIO m_periodicIO = new PeriodicIO();

  private final double m_turningOffset;
  private final String m_moduleName;
  private final String m_smartDashboardKey;

  // TODO: Gains are for example purposes only - must be determined for your own
  // robot!
  // private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(Constants.SwerveDrive.Turn.k_turnFeedForwardS,
  //     Constants.SwerveDrive.Turn.k_turnFeedForwardV, Constants.SwerveDrive.Turn.k_turnFeedForwardA);

  private static class PeriodicIO {
    SwerveModuleState desiredState;
  }

  public SwerveModule(int driveMotorChannel, int turningMotorChannel, double turningOffset, String moduleName) {
    m_turningOffset = turningOffset;
    m_moduleName = moduleName;

    m_smartDashboardKey = "SwerveDrive/" + m_moduleName + "/";

    m_driveMotor = new TalonFX(driveMotorChannel);
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    m_driveVoltage = new VelocityVoltage(0);
    m_driveVoltage.Slot = 0;

    talonConfig.Slot0.kP = Constants.SwerveDrive.Drive.k_driveP;
    talonConfig.Slot0.kI = Constants.SwerveDrive.Drive.k_driveI;
    talonConfig.Slot0.kD = Constants.SwerveDrive.Drive.k_driveD;

    talonConfig.Slot0.kS = Constants.SwerveDrive.Drive.k_driveFeedForwardS;
    talonConfig.Slot0.kV = Constants.SwerveDrive.Drive.k_driveFeedForwardV;
    talonConfig.Slot0.kA = Constants.SwerveDrive.Drive.k_driveFeedForwardA;

    m_driveMotor.getConfigurator().apply(talonConfig);

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setIdleMode(IdleMode.kBrake);
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_turningMotor.setInverted(true);
    m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    m_turningPIDController = m_turningMotor.getPIDController(); // TODO: Set PID Values

    m_turningPIDController.setP(Constants.SwerveDrive.Turn.k_turningP);
    m_turningPIDController.setI(Constants.SwerveDrive.Turn.k_turningI);
    m_turningPIDController.setD(Constants.SwerveDrive.Turn.k_turningD);
    m_turningPIDController.setIZone(Constants.SwerveDrive.Turn.k_turningIZone);
    m_turningPIDController.setFF(Constants.SwerveDrive.Turn.k_turningFF);

    m_turningPIDController.setOutputRange(
      Constants.SwerveDrive.Turn.k_TurningMinOutput, Constants.SwerveDrive.Turn.k_TurningMinOutput);
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
    drivePosition *= ((2 * Constants.SwerveDrive.k_wheelRadiusIn * Math.PI) / Constants.SwerveDrive.k_driveGearRatio); // Convert to inches
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
    velocity *= ((2.0 * Constants.SwerveDrive.k_wheelRadiusIn * Math.PI) / Constants.SwerveDrive.k_driveGearRatio);

    // Convert to m per second
    velocity = Units.inchesToMeters(velocity);

    return velocity;
  }

  public void clearTurnPIDAccumulation() {
    m_turningPIDController.setIAccum(0); //TODO: Make sure this works
  }

  public void resetDriveEncoder() {
    m_driveMotor.setPosition(0.0, 50);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(getTurnPosition()));
    m_periodicIO.desiredState = desiredState;

    // If the velocity changes, set drive motor accordingly.
    if(m_driveVoltage.Velocity != m_periodicIO.desiredState.speedMetersPerSecond * Constants.SwerveDrive.k_driveGearRatio * Units.inchesToMeters(Constants.SwerveDrive.k_wheelRadiusIn)) {
      m_driveVoltage = new VelocityVoltage(m_periodicIO.desiredState.speedMetersPerSecond);
      m_driveVoltage.Slot = 0;
      m_driveMotor.setControl(m_driveVoltage);
    }

    // Calculate the turning motor output from the turning PID controller.
    // double turnTarget = m_periodicIO.desiredState.angle.getRotations();
    // double turnOutput = m_turningPIDController.calculate(getTurnPosition(), turnTarget);
    // double turnFeedforward = m_turnFeedforward.calculate(m_periodicIO.desiredState.speedMetersPerSecond);
    // boolean turnAtGoal = m_turningPIDController.atGoal();

    m_turningPIDController.setReference(m_periodicIO.desiredState.angle.getRotations(), ControlType.kPosition);

    // SmartDashboard.putNumber(m_smartDashboardKey + "TurnTarget", turnTarget);
    // SmartDashboard.putNumber(m_smartDashboardKey + "TurnOutput", turnOutput + turnFeedforward);
    // SmartDashboard.putBoolean(m_smartDashboardKey + "TurnAtGoal", turnAtGoal);
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveTargetVelocity", m_periodicIO.desiredState.speedMetersPerSecond);
  }

  public SwerveModuleState getDesiredState() {
    return m_periodicIO.desiredState;
  }

  public void periodic() {
  }

  public void outputTelemetry() {
    SmartDashboard.putString(m_smartDashboardKey + "DriveMotorPositionUnits", m_driveMotor.getPosition().getUnits());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorPos", m_driveMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorVelocity", getDriveVelocity());
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnMotorPosition", getTurnPosition());
  }
}