package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
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

  public boolean isHoming() {
    return m_periodicIO.climberState == ClimberState.REHOME || m_periodicIO.climberState == ClimberState.HOMING_RETRACT;
  }

  @Override
  public void periodic() { //home
    // if(isHoming()) {
    //   switch(m_periodicIO.climberState) {
    //     case REHOME:
      
    //     case HOMING_RETRACT:
    //     default:
    //       break;
    //   }
    // } else { // do normal (boring) subsystem stuff
      
    // }

    m_periodicIO.speed = 0.01;
  }

  @Override
  public void stop() {
    m_periodicIO.speed = 0.0;
  }

  @Override
  public void writePeriodicOutputs() {
    m_motor.set(m_periodicIO.speed);
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putNumber("Climber/Speed", m_periodicIO.speed);
    SmartDashboard.putString("Climber/State", m_periodicIO.climberState.toString());
  }

  private static class PeriodicIO {
    ClimberState climberState = ClimberState.HAS_NOT_HOMED;
    double speed = 0.0; // m/s
  }

  private enum ClimberState {
    FULLY_EXTENDED,
    FULLY_RETRACTED,
    GOING_UP,
    GOING_DOWN,
    HAS_NOT_HOMED,
    REHOME,
    HOMING_RETRACT
  }
}
