package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
    for(Climber i : m_climbers) {
      i.periodic();
    }
  }

  @Override
  public void stop() {
    for(Climber i : m_climbers) {
      i.stop();
    }
  }

  @Override
  public void writePeriodicOutputs() {
    for(Climber i : m_climbers) {
      i.writePeriodicOutputs();
    }
  }

  @Override
  public void outputTelemetry() {
    for(Climber i : m_climbers) {
      i.outputTelemetry();
    }
  }

  class Climber {
    private CANSparkMax m_motor;
    private RelativeEncoder m_encoder;
    private PeriodicIO m_periodicIO;

    Climber(int motorID) {
      m_motor = new CANSparkMax(motorID, MotorType.kBrushless);
      m_motor.restoreFactoryDefaults();
      m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

      m_encoder = m_motor.getEncoder();
      m_encoder.setPositionConversionFactor(1 / 16); // TODO: Fix this to account for whinch circumference (approx.
                                                     // 3.5in)
    }

    public void periodic() { // home
      switch (m_periodicIO.climberState) {
        case FULLY_EXTENDED:
          //can switch to going_down or rehome
          break;
        case FULLY_RETRACTED:
          //can switch to going_up or rehome
          break;
        case GOING_UP:
          //can switch to going_down, fully_retracted, or rehome
          break;
        case GOING_DOWN:
          //can switch to going_up, fully_extended, or rehome
          break;
        case REHOME:
          //can switch to homing_retract
          break;
        case HOMING_RETRACT:
          //can switch to fully_retracted
          break;
      }

      m_periodicIO.speed = 0.01;
    }

    public boolean isHoming() {
      return m_periodicIO.climberState == ClimberState.REHOME
          || m_periodicIO.climberState == ClimberState.HOMING_RETRACT;
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
      ClimberState climberState = ClimberState.REHOME;
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
