package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import frc.robot.Helpers;
import frc.robot.constants.RobotConstants;
import frc.robot.simulation.IntakeSim;
import frc.robot.simulation.SimMaster;
import frc.robot.wrappers.RARSparkMax;
import frc.robot.wrappers.REVThroughBoreEncoder;

public class Intake extends Subsystem {
  private static Intake m_intake;
  private static IntakeSim m_sim;

  private RARSparkMax m_pivotMotor;
  private RARSparkMax m_intakeMotor;

  private final REVThroughBoreEncoder m_pivotAbsEncoder = new REVThroughBoreEncoder(
      RobotConstants.config.intake().k_pivotEncoderId);

  private final ProfiledPIDController m_pivotMotorPID;
  private final ArmFeedforward m_pivotFeedForward;

  private final double k_pivotThreshold = 3.0;
  private final double k_intakeSpeedThreshold = 0.1;

  private PeriodicIO m_periodicIO;

  private final I2C.Port k_colorSensorPort = I2C.Port.kMXP;
  // private final DigitalInput m_noteTOF1 = new DigitalInput(6);
  // private final DigitalInput m_noteTOF2 = new DigitalInput(7);

  private final double m_intakeAutoDetectCurrent = 40; // Amps
  private final int m_cycleThreshold = 10;
  private double minCurrent = 999.0;
  private int m_cycles = 0;

  private ColorSensorV3 m_colorSensor;

  private Intake() {
    super("Intake");

    m_sim = SimMaster.getInstance().getIntakeSim();

    // Pivot motor setup
    m_pivotMotor = new RARSparkMax(RobotConstants.config.intake().k_pivotMotorId, MotorType.kBrushless);
    m_pivotMotor.restoreFactoryDefaults();
    m_pivotMotor.setIdleMode(RARSparkMax.IdleMode.kCoast);
    m_pivotMotor.setSmartCurrentLimit(20);
    m_pivotMotor.setInverted(true);

    // Intake motor setup
    m_intakeMotor = new RARSparkMax(RobotConstants.config.intake().k_intakeMotorId, MotorType.kBrushless);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setIdleMode(RARSparkMax.IdleMode.kCoast);
    m_intakeMotor.setInverted(true);

    // Pivot PID
    m_pivotMotorPID = new ProfiledPIDController(
        RobotConstants.config.intake().k_pivotMotorP,
        RobotConstants.config.intake().k_pivotMotorI,
        RobotConstants.config.intake().k_pivotMotorD,
        new TrapezoidProfile.Constraints(
            RobotConstants.config.intake().k_maxVelocity,
            RobotConstants.config.intake().k_maxAcceleration));

    // Pivot Feedforward
    m_pivotFeedForward = new ArmFeedforward(
        RobotConstants.config.intake().k_pivotMotorKS,
        RobotConstants.config.intake().k_pivotMotorKG,
        RobotConstants.config.intake().k_pivotMotorKV,
        RobotConstants.config.intake().k_pivotMotorKA);

    m_periodicIO = new PeriodicIO();

    m_colorSensor = new ColorSensorV3(k_colorSensorPort);

    m_intakeMotor.burnFlash();
    m_pivotMotor.burnFlash();
  }

  public static Intake getInstance() {
    if (m_intake == null) {
      m_intake = new Intake();
    }

    return m_intake;
  }

  @Override
  public void periodic() {
    checkAutoTasks();

    if (!DriverStation.isTest()) {
      double target_pivot_angle = getAngleFromTarget(m_periodicIO.pivot_target);

      double pidCalc = m_pivotMotorPID.calculate(getPivotAngle(), target_pivot_angle);
      double ffCalc = m_pivotFeedForward.calculate(Math.toRadians(getPivotReferenceToHorizontal()),
          Math.toRadians(m_pivotMotorPID.getSetpoint().velocity));

      m_periodicIO.pivot_voltage = pidCalc + ffCalc;

      m_periodicIO.intake_speed = getSpeedFromState(m_periodicIO.intake_state);
      putString("IntakeState", m_periodicIO.intake_state.toString());
    }

    double currentCurrent = getIntakeCurrent();
    if (currentCurrent < minCurrent) {
      minCurrent = currentCurrent;
    }

    // FRANKENCODE
    m_cycles++;
    if (m_cycles % m_cycleThreshold == 0) {
      m_cycles = 0;
      minCurrent = 999.0;
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
  public void reset() {
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  public void stopIntake() {
    m_periodicIO.intake_speed = 0.0;
    m_periodicIO.pivot_voltage = 0.0;
    m_periodicIO.intake_state = IntakeState.NONE;
  }

  public void setIntakeState(IntakeState state) {
    m_periodicIO.intake_state = state;
  }

  public void setPivotTarget(IntakePivotTarget target) {
    m_periodicIO.pivot_target = target;
  }

  public IntakePivotTarget getPivotTarget() {
    return m_periodicIO.pivot_target;
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

    double intake_speed = 0.0;
  }

  public enum IntakePivotTarget {
    NONE,
    GROUND,
    EJECT,
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

  private boolean m_override = false;

  public void overrideAutoFlip(boolean override) {
    m_override = override;
  }

  public boolean isAtStow() {
    return getPivotTarget() == IntakePivotTarget.STOW && isAtPivotTarget();
  }

  public boolean isAtGround() {
    return m_periodicIO.pivot_target == IntakePivotTarget.GROUND && isAtPivotTarget();
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private void checkAutoTasks() {
    // If the intake is set to GROUND, and the intake has a note, and the pivot is
    // close to it's target
    // Stop the intake and go to the SOURCE position
    if (isAtGround() && isHoldingNote() && !m_override) {
      setPivotTarget(IntakePivotTarget.STOW);
      setIntakeState(IntakeState.NONE);
      // m_leds.setColor(Color.kGreen);
    }
  }

  // Logged
  @AutoLogOutput
  public double getVelocityError() {
    return m_pivotMotorPID.getVelocityError();
  }

  @AutoLogOutput
  public double getPivotReferenceToHorizontal() {
    return getPivotAngle() - RobotConstants.config.intake().k_pivotOffset;
  }

  @AutoLogOutput
  private String getPivotTargetAsString() {
    return m_periodicIO.pivot_target.toString();
  }

  @AutoLogOutput
  public double getPivotAngle() {
    return Units.rotationsToDegrees(m_pivotAbsEncoder.getAbsolutePosition());
  }

  @AutoLogOutput
  public double getAbsoluteAngle() {
    return m_pivotAbsEncoder.getAbsolutePosition();
  }

  @AutoLogOutput
  public double getCurrentSpeed() {
    return m_intakeMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput
  public boolean isAtPivotTarget() {
    if (m_periodicIO.pivot_target == IntakePivotTarget.NONE) {
      return true;
    }

    double current_angle = getPivotAngle();
    double target_angle = getAngleFromTarget(m_periodicIO.pivot_target);

    return Math.abs(target_angle - current_angle) <= k_pivotThreshold;
  }

  @AutoLogOutput
  private double getPivotVoltage() {
    return Helpers.getVoltage(m_pivotMotor);
  }

  @AutoLogOutput
  private double getTargetPivotVoltage() {
    return m_periodicIO.pivot_voltage;
  }

  @AutoLogOutput
  private double getIntakeVoltage() {
    return Helpers.getVoltage(m_intakeMotor);
  }

  @AutoLogOutput
  private double getPivotCurrent() {
    return m_pivotMotor.getOutputCurrent();
  }

  @AutoLogOutput
  private double getIntakeCurrent() {
    return m_intakeMotor.getOutputCurrent();
  }

  @AutoLogOutput
  public double getAngleFromTarget(IntakePivotTarget target) {
    switch (target) {
      case GROUND:
        return RobotConstants.config.intake().k_groundPivotAngle;
      case PIVOT:
        return RobotConstants.config.intake().k_sourcePivotAngle;
      case EJECT:
        return RobotConstants.config.intake().k_ejectPivotAngle;
      case AMP:
        return RobotConstants.config.intake().k_ampPivotAngle;
      case STOW:
        return RobotConstants.config.intake().k_stowPivotAngle;
      default:
        return RobotConstants.config.intake().k_stowPivotAngle;
    }
  }

  public double getSpeedFromState(IntakeState state) {
    switch (state) {
      case INTAKE:
        return RobotConstants.config.intake().k_intakeSpeed;
      case EJECT:
        return RobotConstants.config.intake().k_ejectSpeed;
      case FEED_SHOOTER:
        return RobotConstants.config.intake().k_feedShooterSpeed;
      default:
        return 0.0;
    }
  }

  @AutoLogOutput
  public boolean isHoldingNote() {
    boolean hasHeldHighCurrent = ((minCurrent > m_intakeAutoDetectCurrent) && (m_cycles % m_cycleThreshold == 0)
        && (minCurrent < 120));
    return getColorSensor() || getTOFOne() || hasHeldHighCurrent;
  }

  @AutoLogOutput
  public boolean getColorSensor() {
    if (isColorSensorConnected()) {
      return m_colorSensor.getProximity() >= RobotConstants.config.intake().k_sensorThreshold;
    }
    return false;
  }

  @AutoLogOutput
  public boolean getTOFOne() {
    // return !m_noteTOF1.get();
    return false; // TODO: FIX PLEASE THANKS PLEASE GOD I HOPE 🙏🙏🙏🙏
  }

  @AutoLogOutput
  public boolean isColorSensorConnected() {
    return m_colorSensor.isConnected();
  }

  @AutoLogOutput
  public int getIntakeProximity() {
    return m_colorSensor.getProximity();
  }
}
