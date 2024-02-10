package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.simulation.ShooterSim;
import frc.robot.simulation.SimMaster;

public class Shooter extends Subsystem {
  private static Shooter m_shooter;
  private static ShooterSim m_shooterSim;

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
    m_shooterSim = SimMaster.getInstance().getShooterSim();

    m_topShooterMotor = new CANSparkFlex(Constants.Shooter.k_topMotorId, MotorType.kBrushless);
    m_topShooterMotor.restoreFactoryDefaults();
    m_topShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    m_topShooterMotor.setInverted(false);

    m_bottomShooterMotor = new CANSparkFlex(Constants.Shooter.k_bottomMotorId, MotorType.kBrushless);
    m_bottomShooterMotor.restoreFactoryDefaults();
    m_bottomShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    m_bottomShooterMotor.setInverted(true);

    m_pivotMotor = new CANSparkFlex(Constants.Shooter.k_pivotMotorId, MotorType.kBrushless);
    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_pivotMotor.setSmartCurrentLimit(10); // TODO: Double check this

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
    // m_periodicIO.pivot_voltage =
    // m_pivotMotorPID.calculate(getCurrentPivotAngle(), m_periodicIO.pivot_angle);

    m_pivotMotorPID.setReference(m_periodicIO.pivot_angle, ControlType.kPosition);

    // if (m_pivotAbsEncoder.get() == 0.0) {
    // m_periodicIO.pivot_voltage = 0.0;
    // }
  }

  @Override
  public void stop() {
    // m_periodicIO.pivot_voltage = 0.0;

    stopShooter();
  }

  @Override
  public void writePeriodicOutputs() {
    double limited_speed = m_speedLimiter.calculate(m_periodicIO.shooter_rpm);
    m_topShooterMotorPID.setReference(limited_speed, ControlType.kVelocity);
    m_bottomShooterMotorPID.setReference(limited_speed, ControlType.kVelocity);

    // m_pivotMotor.setVoltage(m_periodicIO.pivot_voltage);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Shooter/Speed", m_periodicIO.shooter_rpm);
    SmartDashboard.putNumber("Shooter/TopMotorSpeed", m_topMotorEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter/BottomMotorSpeed", m_bottomMotorEncoder.getVelocity());
  }

  public void setAngle(double angle) {
    m_periodicIO.pivot_angle = angle;
  }

  public void setAngle(ShooterPivotTarget target) {
    switch (target) {
      case SHOOTER_LOW:
        m_periodicIO.pivot_angle = Constants.Shooter.k_lowPivotAngle;
        break;
      case SHOOTER_AMP:
        m_periodicIO.pivot_angle = Constants.Shooter.k_ampPivotAngle;
        break;
      case SHOOTER_SPEAKER:
        m_periodicIO.pivot_angle = Constants.Shooter.k_speakerPivotAngle;
        break;
      default:
        break;
    }
  }

  // TODO Get rid of this and make it part of the actual functionality
  public void setSimPosition(double a) {
    m_shooterSim.updateIntakePosition(a);
  }

  public void setSpeed(double rpm) {
    m_periodicIO.shooter_rpm = rpm;
  }

  public void stopShooter() {
    m_periodicIO.shooter_rpm = 0.0;
  }

  public double getCurrentPivotAngle() {
    return Units.rotationsToDegrees(Helpers.modRotations(m_pivotAbsEncoder.get()));
  }

  public double getAngleFromTarget(ShooterPivotTarget target) {
    switch (target) {
      case SHOOTER_LOW:
        return Constants.Shooter.k_lowPivotAngle;
      case SHOOTER_AMP:
        return Constants.Shooter.k_ampPivotAngle;
      case SHOOTER_SPEAKER:
        return Constants.Shooter.k_speakerPivotAngle;

      default:
        return Constants.Shooter.k_lowPivotAngle;
    }
  }

  public boolean isAtTarget(ShooterPivotTarget target) {
    if (target == ShooterPivotTarget.SHOOTER_NONE) {
      return true;
    }

    double current_angle = getCurrentPivotAngle();
    double target_angle = getAngleFromTarget(target);

    return current_angle <= target_angle + 2 && current_angle >= target_angle - 2;
  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;

    double pivot_angle = 60.0;
    // double pivot_voltage = 0.0;
  }

  public enum ShooterPivotTarget {
    SHOOTER_LOW, SHOOTER_AMP, SHOOTER_SPEAKER, SHOOTER_NONE
  }
}
