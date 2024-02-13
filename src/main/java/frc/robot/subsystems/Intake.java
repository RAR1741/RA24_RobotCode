package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;

public class Intake extends Subsystem {
  private static Intake m_intake;

  private CANSparkMax m_pivotMotor;
  private CANSparkMax m_intakeMotor;

  private final DutyCycleEncoder m_pivotMotorEncoder = new DutyCycleEncoder(Constants.Intake.k_pivotEncoderID);

  private final SparkPIDController m_pivotMotorPID;

  private PeriodicIO m_periodicIO;

  private Intake() {
    super("Intake");

    m_pivotMotor = new CANSparkMax(Constants.Intake.k_pivotMotorID, MotorType.kBrushless);
    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_pivotMotor.setSmartCurrentLimit(5); // TODO: Double check this

    m_intakeMotor = new CANSparkMax(Constants.Intake.k_intakeMotorID, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_pivotMotorPID = m_pivotMotor.getPIDController();
    m_pivotMotorPID.setP(Constants.Intake.k_pivotMotorP);
    m_pivotMotorPID.setI(Constants.Intake.k_pivotMotorI);
    m_pivotMotorPID.setD(Constants.Intake.k_pivotMotorD);
    m_pivotMotorPID.setIZone(Constants.Intake.k_pivotMotorIZone);

    m_periodicIO = new PeriodicIO();
  }

  public static Intake getInstance() {
    if (m_intake == null) {
      m_intake = new Intake();
    }

    return m_intake;
  }

  @Override
  public void periodic() {
    if(!(Preferences.getString("Test Mode", null).equals("INTAKE_TEST_MODE") && DriverStation.isTest())) {
      double pivot_angle = getAngleFromTarget(m_periodicIO.pivot_target);
      m_pivotMotorPID.setReference(pivot_angle, ControlType.kPosition);

      m_periodicIO.intake_speed = getSpeedFromState(m_periodicIO.intake_state);
      SmartDashboard.putString("Intake/CurrentState", m_periodicIO.intake_state.toString());
    }
  }

  @Override
  public void stop() {
    stopIntake();
  }

  @Override
  public void writePeriodicOutputs() {
    m_intakeMotor.set(m_periodicIO.intake_speed);
  }

  @Override
  public void outputTelemetry() {
    putNumber("Speed", getSpeedFromState(m_periodicIO.intake_state));
    putNumber("CurrentPivotAngle", getCurrentPivotAngle());
    putNumber("CurrentSetpoint", getAngleFromTarget(m_periodicIO.pivot_target));
  }

  @Override
  public void reset() {
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  private double getAngleFromTarget(IntakePivotTarget target) {
    switch (target) {
      case INTAKE_PIVOT_GROUND: 
        return Constants.Intake.k_groundPivotAngle;
      case INTAKE_PIVOT_SOURCE: 
        return Constants.Intake.k_sourcePivotAngle;
      case INTAKE_PIVOT_AMP: 
        return Constants.Intake.k_ampPivotAngle;
      case INTAKE_PIVOT_STOW: 
        return Constants.Intake.k_stowPivotAngle;
      default: 
        return 180.0;
    }
  }

  private double getSpeedFromState(IntakeState state) {
    switch (state) {
      case INTAKE_STATE_INTAKE: 
        return Constants.Intake.k_intakeSpeed;
      case INTAKE_STATE_EJECT: 
        return Constants.Intake.k_ejectSpeed;
      case INTAKE_STATE_FEED_SHOOTER: 
        return Constants.Intake.k_feedShooterSpeed;
      case INTAKE_STATE_PULSE: {
        if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) { // TODO: check if this is what we want
          return Constants.Intake.k_intakeSpeed;
        }

        return 0.0;
      }
      default: 
        return 0.0;
    }
  }

  public void stopIntake() {
    m_periodicIO.intake_speed = 0.0;
    m_periodicIO.intake_state = IntakeState.INTAKE_STATE_NONE;
  }

  public void setState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  public void setPivotTarget(IntakePivotTarget target) {
    m_periodicIO.pivot_target = target;
  }

  public double getCurrentPivotAngle() {
    double value = m_pivotMotorEncoder.getAbsolutePosition() - Constants.Intake.k_pivotEncoderOffset + 0.5;

    return Units.rotationsToDegrees(Helpers.modRotations(value));
  }

  public double getCurrentSpeed() {
    return m_intakeMotor.getEncoder().getVelocity();
  }

  public boolean isAtPivotTarget(IntakePivotTarget target) {
    if (target == IntakePivotTarget.INTAKE_PIVOT_NONE) {
      return true;
    }

    double current_angle = getCurrentPivotAngle();
    double target_angle = getAngleFromTarget(target);

    return current_angle <= target_angle+2 && current_angle >= target_angle-2;
  }

  public boolean isAtState(IntakeState state) {
    if (state == IntakeState.INTAKE_STATE_NONE) {
      return true;
    }

    double current_speed = getCurrentSpeed();
    double target_speed = getSpeedFromState(state);

    return current_speed <= target_speed+0.1 && current_speed >= target_speed-0.1;
  }

  public void manualPivotControl(double positive, double negative, double limit) {
    m_pivotMotor.set((positive + negative) * limit);
  }

  private static class PeriodicIO {
    IntakePivotTarget pivot_target = IntakePivotTarget.INTAKE_PIVOT_STOW;
    IntakeState intake_state = IntakeState.INTAKE_STATE_NONE;

    // double intake_pivot_voltage = 0.0;
    double intake_speed = 0.0;
  }

  public enum IntakePivotTarget {
    INTAKE_PIVOT_NONE,
    INTAKE_PIVOT_GROUND,
    INTAKE_PIVOT_SOURCE,
    INTAKE_PIVOT_AMP,
    INTAKE_PIVOT_STOW
  }

  public enum IntakeState {
    INTAKE_STATE_NONE,
    INTAKE_STATE_INTAKE,
    INTAKE_STATE_EJECT,
    INTAKE_STATE_PULSE,
    INTAKE_STATE_FEED_SHOOTER
  }
}
