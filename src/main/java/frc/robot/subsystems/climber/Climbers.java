package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class Climbers extends Subsystem {
  private static Climbers m_instance;
  private Climber[] m_climbers;

  public Climbers() {
  }

  public static Climbers getInstance() {
    if (m_instance == null) {
        m_instance = new Climbers();
    }
    
    return m_instance;
  }

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'periodic'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public void writePeriodicOutputs() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'writePeriodicOutputs'");
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
  }
}

class Climber {
  private static Climber m_climber;

  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;

  private PeriodicIO m_periodicIO;

  public Climber() {
    m_motor = new CANSparkMax(Constants.Climber.k_motorID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_encoder = m_motor.getEncoder();
    m_encoder.setPositionConversionFactor(1 / 16); // TODO: Fix this to account for whinch circumference (approx.
                                                   // 3.5in)
  }

  public boolean isHoming() {
    return m_periodicIO.climberState == ClimberState.REHOME
        || m_periodicIO.climberState == ClimberState.HOMING_RETRACT;
  }

  public void periodic() { // home
    // if(isHoming()) {
    // switch(m_periodicIO.climberState) {
    // case REHOME:

    // case HOMING_RETRACT:
    // default:
    // break;
    // }
    // } else { // do normal (boring) subsystem stuff

    // }

    m_periodicIO.speed = 0.01;
  }

  public void stop() {
    m_periodicIO.speed = 0.0;
  }

  public void writePeriodicOutputs() {
    m_motor.set(m_periodicIO.speed);
  }

  public void outputTelemetry() {
    SmartDashboard.putNumber("Climber/Speed", m_periodicIO.speed);
    SmartDashboard.putString("Climber/State", m_periodicIO.climberState.toString());
  }

  private static class PeriodicIO {
    ClimberState climberState = ClimberState.HAS_NOT_HOMED;
    double speed = 0.0; // m/s
  }

  public enum ClimberState {
    FULLY_EXTENDED,
    FULLY_RETRACTED,
    GOING_UP,
    GOING_DOWN,
    HAS_NOT_HOMED,
    REHOME,
    HOMING_RETRACT
  }
}
