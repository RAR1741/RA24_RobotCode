package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.simulation.ShooterSim;
import frc.robot.simulation.SimMaster;

public class Shooter extends Subsystem {
  private static Shooter m_shooter;
  private static ShooterSim m_sim;

  private CANSparkFlex m_topShooterMotor;
  private CANSparkFlex m_bottomShooterMotor;
  private CANSparkFlex m_pivotMotor;

  private RelativeEncoder m_topMotorEncoder;
  private RelativeEncoder m_bottomMotorEncoder;
  private DutyCycleEncoder m_pivotAbsEncoder = new DutyCycleEncoder(Constants.Shooter.k_pivotEncoderId);

  private SparkPIDController m_topShooterMotorPID;
  private SparkPIDController m_bottomShooterMotorPID;
  private final SparkPIDController m_pivotMotorPID;

  private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(1000); // TODO Double-check this value

  private PeriodicIO m_periodicIO;

  private Shooter() {
    super("Shooter");

    m_sim = SimMaster.getInstance().getShooterSim();

    m_topShooterMotor = new CANSparkFlex(Constants.Shooter.k_topMotorId, MotorType.kBrushless);

    m_topShooterMotor.restoreFactoryDefaults();
    m_topShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    m_topShooterMotor.setInverted(true);

    m_bottomShooterMotor = new CANSparkFlex(Constants.Shooter.k_bottomMotorId, MotorType.kBrushless);
    m_bottomShooterMotor.restoreFactoryDefaults();
    m_bottomShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    m_bottomShooterMotor.setInverted(false);

    m_pivotMotor = new CANSparkFlex(Constants.Shooter.k_pivotMotorId, MotorType.kBrushless);
    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_pivotMotor.setSmartCurrentLimit(5); // TODO: Double check this

    m_topMotorEncoder = m_topShooterMotor.getEncoder();
    m_bottomMotorEncoder = m_bottomShooterMotor.getEncoder();

    m_topShooterMotorPID = m_topShooterMotor.getPIDController();
    m_topShooterMotorPID.setP(Constants.Shooter.k_shooterMotorP);
    m_topShooterMotorPID.setI(Constants.Shooter.k_shooterMotorI);
    m_topShooterMotorPID.setD(Constants.Shooter.k_shooterMotorD);
    m_topShooterMotorPID.setOutputRange(Constants.Shooter.k_shooterMinOutput, Constants.Shooter.k_shooterMaxOutput);

    m_bottomShooterMotorPID = m_bottomShooterMotor.getPIDController();
    m_bottomShooterMotorPID.setP(Constants.Shooter.k_shooterMotorP);
    m_bottomShooterMotorPID.setI(Constants.Shooter.k_shooterMotorI);
    m_bottomShooterMotorPID.setD(Constants.Shooter.k_shooterMotorD);
    m_bottomShooterMotorPID.setOutputRange(Constants.Shooter.k_shooterMinOutput, Constants.Shooter.k_shooterMaxOutput);

    m_pivotMotorPID = m_pivotMotor.getPIDController();
    m_pivotMotorPID.setP(Constants.Shooter.k_pivotMotorP);
    m_pivotMotorPID.setI(Constants.Shooter.k_pivotMotorI);
    m_pivotMotorPID.setD(Constants.Shooter.k_pivotMotorD);
    m_pivotMotorPID.setIZone(Constants.Shooter.k_pivotMotorIZone);

    m_periodicIO = new PeriodicIO();
  }

  public static Shooter getInstance() {
    if (m_shooter == null) {
      m_shooter = new Shooter();
    }

    return m_shooter;
  }

  @Override
  public void periodic() {
    if (!(Preferences.getString("Test Mode", "NONE").contains("SHOOTER_") && DriverStation.isTest())) {
      // m_periodicIO.pivot_voltage =
      // m_pivotMotorPID.calculate(getCurrentPivotAngle(), m_periodicIO.pivot_angle);

      m_pivotMotorPID.setReference(m_periodicIO.pivot_angle, ControlType.kPosition);

      // if (m_pivotAbsEncoder.get() == 0.0) {
      // m_periodicIO.pivot_voltage = 0.0;
      // }

      m_sim.updateAngle(getCurrentPivotAngle());
    }
  }

  @Override
  public void stop() {
    // m_periodicIO.pivot_voltage = 0.0;

    stopShooter();
  }

  @Override
  public void writePeriodicOutputs() {
    if (!(Preferences.getString("Test Mode", "NONE").contains("SHOOTER_") && DriverStation.isTest())) {
      double limited_speed = m_speedLimiter.calculate(m_periodicIO.shooter_rpm);
      m_topShooterMotorPID.setReference(limited_speed, ControlType.kVelocity);
      m_bottomShooterMotorPID.setReference(limited_speed, ControlType.kVelocity);
    } else {
      if (Preferences.getString("Test Mode", "NONE").equals("SHOOTER_PIVOT")) {
        m_pivotMotor.set(m_periodicIO.pivot_speed);
      }
      if (Preferences.getString("Test Mode", "NONE").equals("SHOOTER_SHOOT")) {
        m_topShooterMotor.set(m_periodicIO.shoot_speed);
        m_bottomShooterMotor.set(m_periodicIO.shoot_speed);
      }
    }

    // m_pivotMotor.setVoltage(m_periodicIO.pivot_voltage);
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed", m_periodicIO.shooter_rpm);
    putNumber("TopMotorSpeed", m_topMotorEncoder.getVelocity());
    putNumber("BottomMotorSpeed", m_bottomMotorEncoder.getVelocity());
  }

  @Override
  public void reset() {
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  public void setAngle(double angle) {
    m_periodicIO.pivot_angle = angle;
  }

  public void setAngle(ShooterPivotTarget target) {
    switch (target) {
      case LOW:
        m_periodicIO.pivot_angle = Constants.Shooter.k_lowPivotAngle;
        break;
      case AMP:
        m_periodicIO.pivot_angle = Constants.Shooter.k_ampPivotAngle;
        break;
      case SPEAKER:
        m_periodicIO.pivot_angle = Constants.Shooter.k_speakerPivotAngle;
        break;
      default:
        break;
    }
  }

  // TODO Get rid of this and make it part of the actual functionality
  public void setSimPosition(double a) {
    m_sim.updateAngle(a);
  }

  public void setSpeed(double rpm) {
    m_periodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() {
    m_periodicIO.shooter_rpm = 0.0;
  }

  public void manualPivotControl(double positive, double negative, double limit) {
    m_periodicIO.pivot_speed = (positive - negative) * limit;
  }

  public void manualShootControl(double positive, double negative, double limit) {
    m_periodicIO.shoot_speed = (positive - negative) * limit;
  }

  public double getCurrentPivotAngle() {
    return Units.rotationsToDegrees(Helpers.modRotations(m_pivotAbsEncoder.get()));
  }

  public double getAngleFromTarget(ShooterPivotTarget target) {
    switch (target) {
      case LOW:
        return Constants.Shooter.k_lowPivotAngle;
      case AMP:
        return Constants.Shooter.k_ampPivotAngle;
      case SPEAKER:
        return Constants.Shooter.k_speakerPivotAngle;

      default:
        return Constants.Shooter.k_lowPivotAngle;
    }
  }

  public boolean isAtTarget(ShooterPivotTarget target) {
    if (target == ShooterPivotTarget.NONE) {
      return true;
    }

    double current_angle = getCurrentPivotAngle();
    double target_angle = getAngleFromTarget(target);

    return current_angle <= target_angle + 2 && current_angle >= target_angle - 2;
  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;

    double pivot_angle = 60.0;

    double pivot_speed = 0.0;
    double shoot_speed = 0.0;

    // double pivot_voltage = 0.0;
  }

  public enum ShooterPivotTarget {
    LOW,
    AMP,
    SPEAKER,
    NONE
  }
}
