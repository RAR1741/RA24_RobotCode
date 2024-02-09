package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class Climbers extends Subsystem {
  private static Climbers m_instance;
  private Climber[] m_climbers = {
      new Climber(Constants.Climber.k_leftMotorID), // LEFT, 0
      new Climber(Constants.Climber.k_rightMotorID) // RIGHT, 1
  };

  public static Climbers getInstance() {
    if (m_instance == null) {
      m_instance = new Climbers();
    }

    return m_instance;
  }

  @Override
  public void periodic() {
    for(Climber climber : m_climbers) {
      climber.periodic();
    }
  }

  @Override
  public void stop() {
    for(Climber climber : m_climbers) {
      climber.stop();
    }
  }

  @Override
  public void writePeriodicOutputs() {
    for(Climber climber : m_climbers) {
      climber.writePeriodicOutputs();
    }
  }

  @Override
  public void outputTelemetry() {
    for(Climber climber : m_climbers) {
      climber.outputTelemetry();
    }
  }

  class Climber {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;
    private SparkPIDController m_pidController;
    private PeriodicIO m_periodicIO;

    Climber(int motorID) {
      m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
      m_motor.restoreFactoryDefaults();
      m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

      m_encoder = m_motor.getEncoder();
      m_encoder.setPositionConversionFactor(1 / 16); // TODO: Fix this to account for whinch circumference (approx.
                                                     // 3.5in)
      
      m_pidController = m_motor.getPIDController();
    }

    public void setState(ClimberState state) { // TODO: Don't hardcode climber speeds
      m_periodicIO.climberState = state;
      
      switch (m_periodicIO.climberState) {
        case FULLY_EXTENDED:
          m_periodicIO.setpoint = Constants.Climber.Setpoints.k_fullyExtended;
          break;
        case FULLY_RETRACTED:
          m_periodicIO.setpoint = Constants.Climber.Setpoints.k_fullyRetracted;
          break;
        case REHOME:
          m_periodicIO.setpoint = 5.0;
          break;
        case HOMING_RETRACT:
          m_periodicIO.speed = 0.1;
          break;
        default:
          break;
      }
    }

    public void periodic() { 
      switch (m_periodicIO.climberState) {
        case GOING_UP:
          if(m_encoder.getPosition() <= Constants.Climber.Setpoints.k_fullyExtended) {
            setState(ClimberState.FULLY_EXTENDED);
          }
          break;
        case GOING_DOWN:
          if(m_encoder.getPosition() <= 0) { //TODO: Check for limit switch
            setState(ClimberState.FULLY_RETRACTED);
          }
          break;
        case REHOME:
          if(m_encoder.getPosition() >= m_periodicIO.setpoint) {
            setState(ClimberState.HOMING_RETRACT);
          }
          break;
        case HOMING_RETRACT:
          if(true) { //TODO: Check for limit switch
            setState(ClimberState.FULLY_RETRACTED);
          }
          break;
        default:
          m_periodicIO.speed = 0.0;
          break;
      }
    }

    public boolean isHoming() {
      return m_periodicIO.climberState == ClimberState.REHOME
          || m_periodicIO.climberState == ClimberState.HOMING_RETRACT;
    }

    public void stop() {
      m_periodicIO.speed = 0.0;
    }

    public void writePeriodicOutputs() {
      if(!isHoming()) {
        m_pidController.setReference(m_periodicIO.setpoint, ControlType.kPosition);
      } else {
        m_pidController.setReference(m_periodicIO.speed, ControlType.kVelocity);
      }
    }

    public void outputTelemetry() {
      SmartDashboard.putNumber("Climber/Speed", m_periodicIO.speed);
      SmartDashboard.putString("Climber/State", m_periodicIO.climberState.toString());
    }

    private static class PeriodicIO {
      ClimberState climberState = ClimberState.REHOME;
      double setpoint = 0.0; // rotations
      double speed = 0.0; // m/s
    }

    public enum ClimberState {
      FULLY_EXTENDED,
      FULLY_RETRACTED,
      GOING_UP,
      GOING_DOWN,
      REHOME,
      HOMING_RETRACT
    }
  }
}
