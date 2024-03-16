package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.AllianceHelpers;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.REVThroughBoreEncoder;
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
  private REVThroughBoreEncoder m_pivotAbsEncoder = new REVThroughBoreEncoder(Constants.Shooter.k_pivotEncoderId);

  private SparkPIDController m_topShooterMotorPID;
  private SparkPIDController m_bottomShooterMotorPID;
  private SparkPIDController m_pivotMotorPID;

  private SlewRateLimiter m_speedLimiter = new SlewRateLimiter(4000);

  private PeriodicIO m_periodicIO;
  private boolean m_hasSetPivotRelEncoder = false;

  private int m_cycles = 0;

  private Shooter() {
    super("Shooter");

    m_sim = SimMaster.getInstance().getShooterSim();

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
    m_pivotMotor.setSmartCurrentLimit(60);
    m_pivotMotor.setInverted(true);

    m_topMotorEncoder = m_topShooterMotor.getEncoder();
    m_bottomMotorEncoder = m_bottomShooterMotor.getEncoder();

    m_topShooterMotorPID = m_topShooterMotor.getPIDController();
    m_topShooterMotorPID.setP(Constants.Shooter.k_shooterMotorP);
    m_topShooterMotorPID.setI(Constants.Shooter.k_shooterMotorI);
    m_topShooterMotorPID.setD(Constants.Shooter.k_shooterMotorD);
    m_topShooterMotorPID.setFF(Constants.Shooter.k_shooterMotorFF);
    m_topShooterMotorPID.setOutputRange(Constants.Shooter.k_shooterMinOutput, Constants.Shooter.k_shooterMaxOutput);

    m_bottomShooterMotorPID = m_bottomShooterMotor.getPIDController();
    m_bottomShooterMotorPID.setP(Constants.Shooter.k_shooterMotorP);
    m_bottomShooterMotorPID.setI(Constants.Shooter.k_shooterMotorI);
    m_bottomShooterMotorPID.setD(Constants.Shooter.k_shooterMotorD);
    m_bottomShooterMotorPID.setFF(Constants.Shooter.k_shooterMotorFF);
    m_bottomShooterMotorPID.setOutputRange(Constants.Shooter.k_shooterMinOutput, Constants.Shooter.k_shooterMaxOutput);

    m_pivotMotorPID = m_pivotMotor.getPIDController();
    m_pivotMotorPID.setP(Constants.Shooter.k_pivotMotorP);
    m_pivotMotorPID.setI(Constants.Shooter.k_pivotMotorI);
    m_pivotMotorPID.setD(Constants.Shooter.k_pivotMotorD);
    m_pivotMotorPID.setIZone(Constants.Shooter.k_pivotMotorIZone);

    m_periodicIO = new PeriodicIO();

    m_topShooterMotor.burnFlash();
    m_bottomShooterMotor.burnFlash();
  }

  public static Shooter getInstance() {
    if (m_shooter == null) {
      m_shooter = new Shooter();
    }

    return m_shooter;
  }

  @Override
  public void periodic() {
    // If we haven't set the pivot relative encoder yet, and
    // the abs encoder is connected:
    if (!m_hasSetPivotRelEncoder && getIsPivotAsbConnected()) {
      setPivotAbsOffset();
    }

    m_cycles++;

    if (!m_bottomShooterMotor.getInverted()) {
      m_bottomShooterMotor.setInverted(true);
    }

    Logger.recordOutput("Shooter/PivotTargetAngle", m_periodicIO.pivot_angle);
  }

  @Override
  public void stop() {
    stopShooter();
  }

  @Override
  public void writePeriodicOutputs() {
    m_periodicIO.pivot_angle = MathUtil.clamp(
        m_periodicIO.pivot_angle,
        Constants.Shooter.k_minAngle,
        Constants.Shooter.k_maxAngle);

    if (!(Preferences.getString("Test Mode", "NONE").contains("SHOOTER_") && DriverStation.isTest())) {
      // TeleOp mode
      double limited_speed = m_speedLimiter.calculate(m_periodicIO.shooter_rpm);
      m_topShooterMotorPID.setReference(limited_speed, ControlType.kVelocity);
      m_bottomShooterMotorPID.setReference(limited_speed, ControlType.kVelocity);

      double targetPivot = MathUtil.clamp(
          m_periodicIO.pivot_angle + m_periodicIO.manualPivotOffset,
          Constants.Shooter.k_minAngle,
          Constants.Shooter.k_maxAngle);

      double pivotRelRotations = targetAngleToRelRotations(targetPivot);

      if (m_cycles % 10 == 0) {
        setPivotAbsOffset();
      }

      m_pivotMotorPID.setReference(pivotRelRotations, ControlType.kPosition);
    } else {
      // Test mode
      if (Preferences.getString("Test Mode", "NONE").equals("SHOOTER_PIVOT")) {
        m_pivotMotor.set(m_periodicIO.pivot_speed);
      }
      if (Preferences.getString("Test Mode", "NONE").equals("SHOOTER_SHOOT")) {
        m_topShooterMotor.set(m_periodicIO.shoot_speed);
        m_bottomShooterMotor.set(m_periodicIO.shoot_speed);
      }
    }

    // If the pivot absolute encoder isn't connected
    if (!m_pivotAbsEncoder.isConnected()) {
      DriverStation.reportWarning("THE SHOOTER PIVOT IS BROKEN", false);
    }

    m_sim.updateAngle(getPivotAngle());
  }

  @Override
  public void reset() {
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  public void setPivotAbsOffset() {
    RelativeEncoder m_pivotRelEncoder = m_pivotMotor.getEncoder();
    double pivotAngle = getPivotAngle();
    double offset = targetAngleToRelRotations(pivotAngle);

    m_pivotRelEncoder.setPosition(offset);
    m_hasSetPivotRelEncoder = true;
  }

  public double targetAngleToRelRotations(double angle) {
    angle = Units.degreesToRadians(angle);
    double a = 7.90594;
    double theta = Math.atan(3.153 / 7.25);
    double distanceInches = Math
        .sqrt(Math.pow(a, 2.0) + Math.pow(7, 2.0) - (2.0 * a * 7.0 * Math.cos(theta + angle)));

    // Result in relative encoder rotations
    return Constants.Shooter.k_relRotationsToMaxExtension - (distanceInches * Constants.Shooter.k_rotationsPerInch);
  }

  public void changePivotByAngle(double alpha) {
    m_periodicIO.manualPivotOffset += alpha;
  }

  public void setAngle(double angle) {
    m_periodicIO.pivot_angle = angle;
  }

  public void setAngle(ShooterPivotTarget target) {
    m_periodicIO.pivot_angle = getAngleFromTarget(target);
  }

  public void pivotAbsAngleToRel(double angle) {
    MathUtil.interpolate(angle, angle, angle);
  }

  public void setSpeed(double rpm) {
    m_periodicIO.shooter_rpm = rpm;
  }

  public void setSpeed(ShooterSpeedTarget target) {
    switch (target) {
      case MAX:
        m_periodicIO.shooter_rpm = Constants.Shooter.k_maxRPM;
        break;
      case HALF:
        m_periodicIO.shooter_rpm = Constants.Shooter.k_maxRPM / 2;
        break;
      case QUARTER:
        m_periodicIO.shooter_rpm = Constants.Shooter.k_maxRPM / 4;
        break;
      case OFF:
        stopShooter();
        break;
      default:
        stopShooter();
        break;
    }
  }

  public double getSpeakerAutoAimAngle(Pose2d currentPose) {
    // diffPose.getX() needs to be the distance from the robot to the speaker
    double distanceToTarget = currentPose.minus(AllianceHelpers.getAllianceSpeakerPose2d()).getTranslation().getNorm();

    // Use the distance between the robot and the speaker to calculate the angle to
    // aim the shooter at
    Pose2d[] aimingPoses = {
        new Pose2d(1.485, 60.0, new Rotation2d()), // Speaker (1.36)
        new Pose2d(1.686, 56.0, new Rotation2d()), // Speaker Corner (1.47)
        new Pose2d(2.860, 39.5, new Rotation2d()), // Podium (2.74)
        new Pose2d(3.702, 34.1, new Rotation2d()), // Stage Left Close ()
        new Pose2d(3.972, 31.7, new Rotation2d()), // Stage Left ()
        new Pose2d(5.123, 29.5, new Rotation2d()), // Wing shot Close (5.59)
    };

    // Find the upper and lower bounds of the aimingPoses array using the
    // distanceToTarget
    Pose2d upperBound = aimingPoses[aimingPoses.length - 1];
    Pose2d lowerBound = aimingPoses[0];

    for (int i = 0; i < aimingPoses.length; i++) {
      if (distanceToTarget > aimingPoses[i].getX()) {
        lowerBound = aimingPoses[i];
      }
      if (distanceToTarget < aimingPoses[i].getX()) {
        upperBound = aimingPoses[i];
        break;
      }
    }

    // Clamped between [0, 1] (should be a %)
    double t = (distanceToTarget - lowerBound.getX()) / (upperBound.getX() - lowerBound.getX());

    return MathUtil.interpolate(lowerBound.getY(), upperBound.getY(), t); // may the java gods have mercy on us
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

  public double getAngleFromTarget(ShooterPivotTarget target) {
    switch (target) {
      case MAX:
        m_periodicIO.manualPivotOffset = Constants.Shooter.k_initalPivotOffset; // :(
        return Constants.Shooter.k_maxAngle;
      case MIN:
        m_periodicIO.manualPivotOffset = Constants.Shooter.k_initalPivotOffset;
        return Constants.Shooter.k_minAngle;
      case AMP:
        return Constants.Shooter.k_ampPivotAngle;
      case PODIUM:
        return Constants.Shooter.k_podiumPivotAngle;
      case SUBWOOFER:
        return Constants.Shooter.k_subwooferPivotAngle;
      default:
        return m_periodicIO.pivot_angle;
    }
  }

  private static class PeriodicIO {
    double shooter_rpm = 0.0;

    double pivot_angle = 60.0;

    double pivot_speed = 0.0;
    double shoot_speed = 0.0;

    double manualPivotOffset = Constants.Shooter.k_initalPivotOffset;
  }

  public enum ShooterSpeedTarget {
    MAX,
    HALF,
    QUARTER,
    OFF
  }

  public enum ShooterPivotTarget {
    MAX,
    MIN,
    AMP,
    SUBWOOFER,
    PODIUM,
    NONE
  }

  // Logged
  @AutoLogOutput
  public boolean isAtTarget() {
    if (m_periodicIO.pivot_angle == getAngleFromTarget(ShooterPivotTarget.NONE)) {
      return true;
    }

    double angle = getPivotAngle();
    double target_angle = m_periodicIO.pivot_angle;

    return angle <= target_angle + 2 && angle >= target_angle - 2;
  }

  @AutoLogOutput
  public double getPivotAngle() {
    return Units.rotationsToDegrees(Helpers.modRotations(
        m_pivotAbsEncoder.getAbsolutePosition() - Units.degreesToRotations(Constants.Shooter.k_absPivotOffset)));
  }

  @AutoLogOutput
  public boolean isAtSpeed() {
    return Math
        .abs(m_topMotorEncoder.getVelocity() - m_periodicIO.shooter_rpm) <= Constants.Shooter.k_shooterSpeedTolerance
        && Math.abs(
            m_bottomMotorEncoder.getVelocity() - m_periodicIO.shooter_rpm) <= Constants.Shooter.k_shooterSpeedTolerance;
  }

  @AutoLogOutput
  public boolean getIsPivotAsbConnected() {
    return m_pivotAbsEncoder.isConnected();
  }

  @AutoLogOutput
  public double getPivotRelRotations() {
    return m_pivotMotor.getEncoder().getPosition();
  }

  @AutoLogOutput
  public int getPivotAbsFrequency() {
    return m_pivotAbsEncoder.getFrequency();
  }

  @AutoLogOutput
  public double getPivotRelTarget() {
    return targetAngleToRelRotations(m_periodicIO.pivot_angle);
  }

  @AutoLogOutput
  public double getPivotMotorCurrent() {
    return m_pivotMotor.getOutputCurrent();
  }

  @AutoLogOutput
  private double getPivotVoltage() {
    return Helpers.getVoltage(m_pivotMotor);
  }

  @AutoLogOutput
  private double getPivotSpeed() {
    return m_periodicIO.pivot_speed;
  }

  @AutoLogOutput
  public double getTopMotorSpeed() {
    return m_topMotorEncoder.getVelocity();
  }

  @AutoLogOutput
  public double getBottomMotorSpeed() {
    return m_bottomMotorEncoder.getVelocity();
  }

  @AutoLogOutput
  public double getShooterTargetSpeed() {
    return m_periodicIO.shooter_rpm;
  }

  @AutoLogOutput
  public double getManualPivotOffset() {
    return m_periodicIO.manualPivotOffset;
  }
}
