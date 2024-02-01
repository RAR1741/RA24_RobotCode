package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.simulation.IntakeSim;

public class Intake extends Subsystem {
  private static Intake m_intake;
  private static IntakeSim m_intakeSim;

  private CANSparkMax m_pivotMotor;
  private CANSparkMax m_intakeMotor;

  private final DutyCycleEncoder m_pivotMotorEncoder = new DutyCycleEncoder(Constants.Intake.k_pivotEncoderId);

  private final PIDController m_pivotPID = new PIDController(
    Constants.Intake.k_pivotMotorP, Constants.Intake.k_pivotMotorI, Constants.Intake.k_pivotMotorD);

  private PeriodicIO m_periodicIO;

  private Intake() {
    m_intakeSim = IntakeSim.getInstance();

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
    if (m_intake == null) {
      m_intake = new Intake();
    }

    return m_intake;
  }

  @Override
  public void periodic() {
    double pivot_angle = getAngleFromTarget(m_periodicIO.pivot_target);
    m_periodicIO.intake_pivot_voltage = m_pivotPID.calculate(getCurrentPivotAngle(), pivot_angle);

    if (m_pivotMotorEncoder.get() == 0.0) {
      m_periodicIO.intake_pivot_voltage = 0.0;
    }

    m_periodicIO.intake_speed = intakeStateToSpeed(m_periodicIO.intake_state);
    SmartDashboard.putString("Intake/CurrentState", m_periodicIO.intake_state.toString());
  }

  @Override
  public void stop() {
    m_periodicIO.intake_pivot_voltage = 0.0;

    stopIntake();
  }

  @Override
  public void writePeriodicOutputs() {
    m_pivotMotor.setVoltage(m_periodicIO.intake_pivot_voltage);
    m_intakeMotor.set(m_periodicIO.intake_speed);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Intake/Speed", intakeStateToSpeed(m_periodicIO.intake_state));
    SmartDashboard.putNumber("Intake/CurrentSetpoint", getAngleFromTarget(m_periodicIO.pivot_target));
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
    switch (state) {
      case INTAKE: return Constants.Intake.k_intakeSpeed;
      case EJECT: return Constants.Intake.k_ejectSpeed;
      case FEED_SHOOTER: return Constants.Intake.k_feedShooterSpeed;

      case PULSE: {
        if (Timer.getFPGATimestamp() % 1.0 < (1.0 / 45.0)) { // TODO: check if this is what we want
          return Constants.Intake.k_intakeSpeed;
        }

        return 0.0;
      }

      default: return 0.0;
    }
  }

  public void stopIntake() {
    m_periodicIO.intake_speed = 0.0;
    m_periodicIO.intake_state = IntakeState.NONE;
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

  public void setSimPosition(double a) {
    m_intakeSim.updateIntakePosition(a);
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
