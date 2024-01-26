package frc.robot.subsystems.drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;

public class SwerveModule {
  // private final TalonFX m_driveMotor;
  // private VelocityVoltage m_driveVoltage;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;
  private final RelativeEncoder m_driveEncoder;
  private final AbsoluteEncoder m_turningEncoder;
  private final SparkPIDController m_turnPIDController;
  private final SparkPIDController m_drivePIDController;

  
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
    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_driveEncoder = m_driveMotor.getEncoder();
    m_driveEncoder.setPositionConversionFactor(Units.inchesToMeters(Constants.SwerveDrive.k_wheelRadiusIn * 2 * Math.PI) / Constants.SwerveDrive.k_driveGearRatio);
    m_driveEncoder.setVelocityConversionFactor(Units.inchesToMeters((Constants.SwerveDrive.k_wheelRadiusIn * 2 * Math.PI) / Constants.SwerveDrive.k_driveGearRatio) / 60);

    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setIdleMode(IdleMode.kBrake);
    m_turningEncoder = m_turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_turningMotor.setInverted(true);
    m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);

    m_turnPIDController = m_turningMotor.getPIDController();
    m_turnPIDController.setP(Constants.SwerveDrive.Turn.k_turningP);
    m_turnPIDController.setI(Constants.SwerveDrive.Turn.k_turningI);
    m_turnPIDController.setD(Constants.SwerveDrive.Turn.k_turningD);
    m_turnPIDController.setIZone(Constants.SwerveDrive.Turn.k_turningIZone);
    m_turnPIDController.setFF(Constants.SwerveDrive.Turn.k_turningFF);
    m_turnPIDController.setOutputRange(
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
        getDriveVelocity(), Rotation2d.fromRotations(getTurnPosition()));
  }

  public double getTurnPosition() {
    return Helpers.modRotations(m_turningEncoder.getPosition() - m_turningOffset);
  }

  public SwerveModulePosition getPosition() {
    double drivePosition = m_driveEncoder.getPosition();
    drivePosition *= ((2 * Constants.SwerveDrive.k_wheelRadiusIn * Math.PI) / Constants.SwerveDrive.k_driveGearRatio); // Convert to inches
    drivePosition = Units.inchesToMeters(drivePosition);

    return new SwerveModulePosition(
        drivePosition, Rotation2d.fromRotations(getTurnPosition()));
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
    m_turnPIDController.setIAccum(0); //TODO: Make sure this works
  }

  public void resetDriveEncoder() {
    m_driveEncoder.setPosition(0.0);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(getTurnPosition()));
    m_periodicIO.desiredState = desiredState;

    // If the velocity changes, set drive motor accordingly.
    // if(m_driveEncoder.getVelocity() != m_periodicIO.desiredState.speedMetersPerSecond * Constants.SwerveDrive.k_driveGearRatio * Units.inchesToMeters(Constants.SwerveDrive.k_wheelRadiusIn)) {
    //   m_drivePIDController.setReference(m_periodicIO.desiredState.speedMetersPerSecond, ControlType.kVelocity);
    // }

    // Calculate the turning motor output from the turning PID controller.
    // double turnTarget = m_periodicIO.desiredState.angle.getRotations();
    // double turnOutput = m_turningPIDController.calculate(getTurnPosition(), turnTarget);
    // double turnFeedforward = m_turnFeedforward.calculate(m_periodicIO.desiredState.speedMetersPerSecond);
    // boolean turnAtGoal = m_turningPIDController.atGoal();

    // If the turn position changes, set turn motor accordingly
    // if(m_turningEncoder.getPosition() != m_periodicIO.desiredState.angle.getRotations()) {
    //   m_turnPIDController.setReference(m_periodicIO.desiredState.angle.getRotations(), ControlType.kPosition);
    // }

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
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorPos", m_driveEncoder.getPosition());
    SmartDashboard.putNumber(m_smartDashboardKey + "DriveMotorVelocity", getDriveVelocity());
    SmartDashboard.putNumber(m_smartDashboardKey + "TurnMotorPosition", getTurnPosition());
  }
}