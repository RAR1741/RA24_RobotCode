package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.simulation.IntakeSim;
import frc.robot.simulation.SimMaster;

public class Intake extends Subsystem {
  private static Intake m_intake;
  private static IntakeSim m_sim;

  private CANSparkMax m_pivotMotor;
  private CANSparkMax m_intakeMotor;

  private final DutyCycleEncoder m_pivotAbsEncoder = new DutyCycleEncoder(Constants.Intake.k_pivotEncoderId);

  private final PIDController m_pivotMotorPID;

  private final double k_pivotThreshold = 2.0;
  private final double k_intakeSpeedThreshold = 0.1;

  private PeriodicIO m_periodicIO;

  private final I2C.Port k_colorSensorPort = I2C.Port.kMXP;

  private ColorSensorV3 m_colorSensor;

  private Intake() {
    super("Intake");

    m_sim = SimMaster.getInstance().getIntakeSim();

    // Pivot motor setup
    m_pivotMotor = new CANSparkMax(Constants.Intake.k_pivotMotorId, MotorType.kBrushless);
    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_pivotMotor.setSmartCurrentLimit(10); // TODO: Double check this
    m_pivotMotor.setInverted(true);

    // Intake motor setup
    m_intakeMotor = new CANSparkMax(Constants.Intake.k_intakeMotorId, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_intakeMotor.setInverted(true);

    // Pivot PID
    m_pivotMotorPID = new PIDController(
        Constants.Intake.k_pivotMotorP,
        Constants.Intake.k_pivotMotorI,
        Constants.Intake.k_pivotMotorD);

    m_periodicIO = new PeriodicIO();

    m_colorSensor = new ColorSensorV3(k_colorSensorPort);
  }

  public boolean doesHaveRing() {
    return m_colorSensor.getProximity() >= Constants.Intake.k_sensorThreshold;
  }

  public static Intake getInstance() {
    if (m_intake == null) {
      m_intake = new Intake();
    }

    return m_intake;
  }

  @Override
  public void periodic() {
    if (!DriverStation.isTest()) {
      double target_pivot_angle = getAngleFromTarget(m_periodicIO.pivot_target);
      m_periodicIO.pivot_voltage = m_pivotMotorPID.calculate(getPivotAngle(), target_pivot_angle);

      m_periodicIO.intake_speed = getSpeedFromState(m_periodicIO.intake_state);
      putString("IntakeState", m_periodicIO.intake_state.toString());
    }

    m_sim.updateAngle(getPivotAngle());
  }

  @Override
  public void stop() {
    stopIntake();
  }

  @Override
  public void writePeriodicOutputs() {
    m_pivotMotor.set(m_periodicIO.pivot_voltage);
    m_intakeMotor.set(m_periodicIO.intake_speed);
  }

  @Override
  public void outputTelemetry() {
    putNumber("IntakeSpeed_PERIODIC", m_periodicIO.intake_speed);
    putNumber("IntakePower", Helpers.getVoltage(m_intakeMotor));
    putNumber("IntakeSpeed", getSpeedFromState(m_periodicIO.intake_state));
    putBoolean("IntakeHasRing", doesHaveRing());

    putString("PivotTarget", m_periodicIO.pivot_target.toString());
    putNumber("PivotAbsPos", m_pivotAbsEncoder.getAbsolutePosition());
    putNumber("PivotSetpoint", getAngleFromTarget(m_periodicIO.pivot_target));
    putNumber("PivotSpeed", m_periodicIO.pivot_voltage);
    putNumber("PivotPower", Helpers.getVoltage(m_pivotMotor));
    putNumber("PivotRelAngle", getPivotAngle());
    putBoolean("PivotAtTarget", isAtPivotTarget(m_periodicIO.pivot_target));
  }

  @Override
  public void reset() {
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  private double getAngleFromTarget(IntakePivotTarget target) {
    switch (target) {
      case GROUND:
        return Constants.Intake.k_groundPivotAngle;
      case PIVOT:
        return Constants.Intake.k_sourcePivotAngle;
      case AMP:
        return Constants.Intake.k_ampPivotAngle;
      case STOW:
        return Constants.Intake.k_stowPivotAngle;
      default:
        return Constants.Intake.k_stowPivotAngle;
    }
  }

  private double getSpeedFromState(IntakeState state) {
    switch (state) {
      case INTAKE:
        return Constants.Intake.k_intakeSpeed;
      case EJECT:
        return Constants.Intake.k_ejectSpeed;
      case FEED_SHOOTER:
        return Constants.Intake.k_feedShooterSpeed;
      default:
        return 0.0;
    }
  }

  public void stopIntake() {
    m_periodicIO.intake_speed = 0.0;
    m_periodicIO.pivot_voltage = 0.0;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  public void setPivotTarget(IntakePivotTarget target) {
    m_periodicIO.pivot_target = target;
  }

  public double getPivotAngle() {
    return Units.rotationsToDegrees(m_pivotAbsEncoder.getAbsolutePosition());
  }

  public double getCurrentSpeed() {
    return m_intakeMotor.getEncoder().getVelocity();
  }

  public boolean isAtPivotTarget(IntakePivotTarget target) {
    if (target == IntakePivotTarget.NONE) {
      return true;
    }

    double current_angle = getPivotAngle();
    double target_angle = getAngleFromTarget(target);

    return Math.abs(target_angle - current_angle) <= k_pivotThreshold;
  }

  public boolean isAtState(IntakeState state) {
    if (state == IntakeState.NONE) {
      return true;
    }

    double current_speed = getCurrentSpeed();
    double target_speed = getSpeedFromState(state);

    return Math.abs(target_speed - current_speed) <= k_intakeSpeedThreshold;
  }

  public void manualPivotControl(double positive, double negative, double limit) {
    m_periodicIO.pivot_voltage = (positive - negative) * limit;
  }

  public void manualIntakeControl(double positive, double negative, double limit) {
    m_periodicIO.intake_speed = (positive - negative) * limit;
  }

  private static class PeriodicIO {
    double pivot_voltage = 0.0;
    IntakePivotTarget pivot_target = IntakePivotTarget.STOW;
    IntakeState intake_state = IntakeState.NONE;

    // double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  }

  public enum IntakePivotTarget {
    NONE,
    GROUND,
    PIVOT,
    AMP,
    STOW
  }

  public enum IntakeState {
    NONE,
    INTAKE,
    EJECT,
    PULSE,
    FEED_SHOOTER
  }
}
