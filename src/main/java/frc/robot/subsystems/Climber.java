package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Helpers;
import frc.robot.constants.RobotConstants;
import frc.robot.wrappers.RARSparkMax;

public class Climber extends Subsystem {
  private static Climber m_climber;
  private PeriodicIO m_periodicIO;

  public static Climber getInstance() {
    if (m_climber == null) {
      m_climber = new Climber();
    }
    return m_climber;
  }

  private RARSparkMax m_leftMotor;
  private RARSparkMax m_rightMotor;

  private SparkPIDController m_leftPID;
  private SparkPIDController m_rightPID;

  private RelativeEncoder m_leftEncoder;
  private RelativeEncoder m_rightEncoder;

  private Climber() {
    super("Climber");

    m_periodicIO = new PeriodicIO();

    m_leftMotor = new RARSparkMax(RobotConstants.config.climber().k_leftMotorID, MotorType.kBrushless);
    m_leftMotor.restoreFactoryDefaults();
    m_leftMotor.setSmartCurrentLimit(40);
    m_leftMotor.setIdleMode(IdleMode.kBrake);

    m_rightMotor = new RARSparkMax(RobotConstants.config.climber().k_rightMotorID, MotorType.kBrushless);
    m_rightMotor.restoreFactoryDefaults();
    m_rightMotor.setSmartCurrentLimit(40);
    m_rightMotor.setIdleMode(IdleMode.kBrake);

    m_leftPID = m_leftMotor.getPIDController();
    m_leftPID.setP(RobotConstants.config.climber().k_P);
    m_leftPID.setI(RobotConstants.config.climber().k_I);
    m_leftPID.setD(RobotConstants.config.climber().k_D);
    m_leftPID.setOutputRange(
        RobotConstants.config.climber().k_minOutput,
        RobotConstants.config.climber().k_maxOutput);

    m_rightPID = m_rightMotor.getPIDController();
    m_rightPID.setP(RobotConstants.config.climber().k_P);
    m_rightPID.setI(RobotConstants.config.climber().k_I);
    m_rightPID.setD(RobotConstants.config.climber().k_D);
    m_rightPID.setOutputRange(
        RobotConstants.config.climber().k_minOutput,
        RobotConstants.config.climber().k_maxOutput);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setPositionConversionFactor(RobotConstants.config.climber().k_gearRatio);
    m_leftEncoder.setVelocityConversionFactor(RobotConstants.config.climber().k_gearRatio);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setPositionConversionFactor(RobotConstants.config.climber().k_gearRatio);
    m_rightEncoder.setVelocityConversionFactor(RobotConstants.config.climber().k_gearRatio);

    m_leftMotor.setIdleMode(RARSparkMax.IdleMode.kBrake);
    m_rightMotor.setIdleMode(RARSparkMax.IdleMode.kBrake);

    m_leftMotor.setInverted(true);
    m_rightMotor.setInverted(false);

    m_leftMotor.burnFlash();
    m_rightMotor.burnFlash();
  }

  private static class PeriodicIO {
    double climber_right_speed = 0.0; // RPM of spool (Spark Max default unit)
    double climber_left_speed = 0.0; // RPM of spool (Spark Max default unit)
    double manual_speed = 0.0; // only used in TEST
  }

  @Override
  public void periodic() {
  }

  @Override
  public void writePeriodicOutputs() {
    if (!(Preferences.getString("TestMode", "NONE").equals("CLIMBER") && DriverStation.isTest())) {
      m_leftPID.setReference(m_periodicIO.climber_left_speed, ControlType.kVelocity);
      m_rightPID.setReference(m_periodicIO.climber_right_speed, ControlType.kVelocity);
    } else {
      m_leftMotor.set(m_periodicIO.manual_speed);
      m_rightMotor.set(m_periodicIO.manual_speed);
    }
  }

  @Override
  public void stop() {
    stopClimber();
  }

  @Override
  public void reset() {
  }

  public void manualControl(double positive, double negative, double limit) {
    m_periodicIO.manual_speed = (positive - negative) * limit;
  }

  public void setBrakeMode() {
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.setIdleMode(IdleMode.kCoast);
  }

  public void raise() {
    m_periodicIO.climber_left_speed = RobotConstants.config.climber().k_raiseSpeed;
    m_periodicIO.climber_right_speed = RobotConstants.config.climber().k_raiseSpeed;
  }

  public void lower() {
    m_periodicIO.climber_left_speed = RobotConstants.config.climber().k_lowerSpeed;
    m_periodicIO.climber_right_speed = RobotConstants.config.climber().k_lowerSpeed;
  }

  public void tiltLeft() {
    m_periodicIO.climber_left_speed = RobotConstants.config.climber().k_lowerSpeed;
    m_periodicIO.climber_right_speed = 0.0;
  }

  public void tiltRight() {
    m_periodicIO.climber_left_speed = 0.0;
    m_periodicIO.climber_right_speed = RobotConstants.config.climber().k_lowerSpeed;
  }

  public void stopClimber() {
    m_periodicIO.climber_left_speed = 0.0;
    m_periodicIO.climber_right_speed = 0.0;
  }

  // Logged
  @AutoLogOutput
  public double getLeftMotorSpeedSetpoint() {
    return m_periodicIO.climber_left_speed;
  }

  @AutoLogOutput
  public double getRightMotorSpeedSetpoint() {
    return m_periodicIO.climber_right_speed;
  }

  @AutoLogOutput
  public double getLeftMotorSpeed() {
    return m_leftEncoder.getVelocity();
  }

  @AutoLogOutput
  public double getRightMotorSpeed() {
    return m_rightEncoder.getVelocity();
  }

  @AutoLogOutput
  public double getLeftMotorVoltage() {
    return Helpers.getVoltage(m_leftMotor);
  }

  @AutoLogOutput
  public double getRightMotorVoltage() {
    return Helpers.getVoltage(m_rightMotor);
  }

  @AutoLogOutput
  public double getLeftMotorCurrent() {
    return m_leftMotor.getOutputCurrent();
  }

  @AutoLogOutput
  public double getRightMotorCurrent() {
    return m_rightMotor.getOutputCurrent();
  }
}
