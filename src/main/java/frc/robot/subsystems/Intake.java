package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.simulation.IntakeSim;
import frc.robot.simulation.SimMaster;

public class Intake extends Subsystem {
  private static Intake m_intake;
  private static IntakeSim m_sim;

  private CANSparkMax m_pivotMotor;
  private CANSparkMax m_intakeMotor;

  private final DutyCycleEncoder m_pivotMotorEncoder = new DutyCycleEncoder(Constants.Intake.k_pivotEncoderId);

  private final SparkPIDController m_pivotMotorPID;

  private PeriodicIO m_periodicIO;

  private Intake() {
    super("Intake");

    m_sim = SimMaster.getInstance().getIntakeSim();

    m_pivotMotor = new CANSparkMax(Constants.Intake.k_pivotMotorId, MotorType.kBrushless);
    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_pivotMotor.setSmartCurrentLimit(5); // TODO: Double check this
    m_pivotMotor.setInverted(true);

    m_intakeMotor = new CANSparkMax(Constants.Intake.k_intakeMotorId, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_pivotMotorPID = m_pivotMotor.getPIDController();
    m_pivotMotorPID.setP(Constants.Intake.k_pivotMotorP);
    m_pivotMotorPID.setI(Constants.Intake.k_pivotMotorI);
    m_pivotMotorPID.setD(Constants.Intake.k_pivotMotorD);
    m_pivotMotorPID.setIZone(Constants.Intake.k_pivotMotorIZone);

    m_pivotMotor.setIdleMode(IdleMode.kBrake);

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
    if(!(Preferences.getString("Test Mode", "NONE").equals("INTAKE_PIVOT") && DriverStation.isTest())) {
      double pivot_angle = getAngleFromTarget(m_periodicIO.pivot_target);
      m_pivotMotorPID.setReference(pivot_angle, ControlType.kPosition);

      m_periodicIO.intake_speed = getSpeedFromState(m_periodicIO.intake_state);
      SmartDashboard.putString("Intake/CurrentState", m_periodicIO.intake_state.toString());
    }

    m_sim.updateAngle(getCurrentPivotAngle());
  }

  @Override
  public void stop() {
    stopIntake();
  }

  @Override
  public void writePeriodicOutputs() {
    m_pivotMotor.set(m_periodicIO.pivot_speed);
  }

  @Override
  public void outputTelemetry() {
    putNumber("IntakeSpeed", getSpeedFromState(m_periodicIO.intake_state));
    putNumber("CurrentPivotAngle", getCurrentPivotAngle());
    putNumber("CurrentSetpoint", getAngleFromTarget(m_periodicIO.pivot_target));
    putNumber("PivotSpeed", m_periodicIO.pivot_speed);
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
    double value = m_pivotMotorEncoder.getAbsolutePosition() - Constants.Intake.k_pivotEncoderOffset;

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

    return current_angle <= target_angle + 2 && current_angle >= target_angle - 2;
  }

  public boolean isAtState(IntakeState state) {
    if (state == IntakeState.INTAKE_STATE_NONE) {
      return true;
    }

    double current_speed = getCurrentSpeed();
    double target_speed = getSpeedFromState(state);

    return current_speed <= target_speed + 0.1 && current_speed >= target_speed - 0.1;
  }

  public void manualPivotControl(double positive, double negative, double limit) {
    m_periodicIO.pivot_speed = (positive - negative) * limit;
  }

  private static class PeriodicIO {
    double pivot_speed = 0.0;
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
