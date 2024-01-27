package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Helpers;

/**
 * 2 Neo
 *
 * Limit Switch
 * Through-bore Encoder
 */

public class Intake extends Subsystem {
  private static Intake m_manipulation;

  private CANSparkMax m_pivotMotor;
  private CANSparkMax m_intakeMotor;

  private final DutyCycleEncoder m_pivotMotorEncoder = new DutyCycleEncoder(Constants.Intake.k_pivotEncoderId);

  private final PIDController m_pivotPID = new PIDController(
    Constants.Intake.k_pivotMotorP, Constants.Intake.k_pivotMotorI, Constants.Intake.k_pivotMotorD);

  private PeriodicIO m_periodicIO;

  private Intake() {
    m_pivotMotor = new CANSparkMax(Constants.Intake.k_pivotMotorID, MotorType.kBrushless);
    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_pivotMotor.setSmartCurrentLimit(10); // TODO: Double check this

    m_intakeMotor = new CANSparkMax(Constants.Intake.k_intakeMotorID, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_periodicIO = new PeriodicIO();
  }

  public static Intake getInstance() {
    if (m_manipulation == null) {
      m_manipulation = new Intake();
    }

    return m_manipulation;
  }

  @Override
  public void periodic() {
    double pivot_angle = getAngleFromTarget(m_periodicIO.pivot_target);
    m_periodicIO.intake_pivot_voltage = m_pivotPID.calculate(getCurrentPivotAngle(), pivot_angle);

    if (m_pivotMotorEncoder.get() == 0.0) {
      m_periodicIO.intake_pivot_voltage = 0.0;
    }

    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);
  }

  @Override
  public void stop() {

  }

  @Override
  public void writePeriodicOutputs() {

  }

  @Override
  public void outputTelemetry() {

  }

  private double getAngleFromTarget(PivotTarget target) {
    switch (target) {
      case GROUND: return Constants.Intake.k_groundPivotAngle;
      case SOURCE: return Constants.Intake.k_sourcePivotAngle;
      case AMP: return Constants.Intake.k_ampPivotAngle;
      case STOW: return Constants.Intake.k_stowPivotAngle;

      default: return 180.0;
    }
  }

  private double intakeStateToSpeed(IntakeState state) {

  }

  public void goToGround() {
    m_periodicIO.pivot_target = PivotTarget.GROUND;
  }

  public void goToSource() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
  }

  public void goToAmp() {
    m_periodicIO.pivot_target = PivotTarget.SOURCE;
  }

  public void goToStow() {
    m_periodicIO.pivot_target = PivotTarget.STOW;
  }

  // Intake helper functions
  public void intake() {
    m_periodicIO.intake_state = IntakeState.INTAKE;
  }

  public void eject() {
    m_periodicIO.intake_state = IntakeState.EJECT;
  }

  public void pulse() {
    m_periodicIO.intake_state = IntakeState.PULSE;
  }

  public void feedShooter() {
    m_periodicIO.intake_state = IntakeState.FEED_SHOOTER;
  }

  public void stopIntake() {
    m_periodicIO.intake_state = IntakeState.NONE;
    m_periodicIO.intake_speed = 0.0;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  public void setPivotTarget(PivotTarget target) {
    m_periodicIO.pivot_target = target;
  }

  private double getCurrentPivotAngle() {
    double value = m_pivotMotorEncoder.getAbsolutePosition() - Constants.Intake.k_pivotEncoderOffset + 0.5;

    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  private static class PeriodicIO {
    PivotTarget pivot_target = PivotTarget.STOW;
    IntakeState intake_state = IntakeState.NONE;

    double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  }

  public enum PivotTarget {
    NONE, GROUND, SOURCE, AMP, STOW
  }

  public enum IntakeState {
    NONE, INTAKE, EJECT, PULSE, FEED_SHOOTER
  }
}
