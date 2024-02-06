package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * 1 neo
 */

public class Climber extends Subsystem {
  private static Climber m_climber;

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;

  private PeriodicIO m_periodicIO;

  private Climber() {
    super("Climber");

    m_motor = new CANSparkMax(Constants.Climber.k_motorID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPositionConversionFactor(1/16); // TODO: Fix this to account for whinch circumference (approx. 3.5in)
  }

  public static Climber getInstance() {
    if (m_climber == null) {
      m_climber = new Climber();
    }

    return m_climber;
  }

  @Override
  public void periodic() {
    if (isLimited()) {
      m_periodicIO.power = Constants.Climber.k_velocity;
    }
  }

  @Override
  public void stop() {
    m_periodicIO.power = 0.0;
  }

  @Override
  public void writePeriodicOutputs() {
    putString("Direction", m_periodicIO.target_direction.toString());
    putBoolean("IsFullyRaised", m_periodicIO.fully_raised);
    putNumber("Power", m_periodicIO.power);
  }

  @Override
  public void outputTelemetry() {
  }

  @Override
  public void reset() {
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  public boolean isLimited() {
    return isAtBottom() || isAtTop();
  }

  public boolean isAtBottom() {
    return false;
  }

  public boolean isAtTop() {
    return false;
  }

  private static class PeriodicIO {
    boolean fully_raised = false;
    DirectionTarget target_direction = DirectionTarget.UP;
    double power = 0.0;
  }

  public enum DirectionTarget {
    UP, DOWN
  }
}
