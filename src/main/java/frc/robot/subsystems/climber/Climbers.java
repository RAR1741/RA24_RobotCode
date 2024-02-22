package frc.robot.subsystems.climber;

import frc.robot.Constants;
import frc.robot.subsystems.Subsystem;

public class Climbers extends Subsystem {
  public Climbers() {
    super("Climbers");
  }

  private static Climbers m_instance;
  private Climber[] m_climbers = {
      new Climber(Constants.Climber.k_leftMotorID),
      new Climber(Constants.Climber.k_rightMotorID)
  };

  public void manualControl(double positive, double negative, double limit) {
    for(Climber climber : m_climbers) {
      climber.setManualSpeed((positive - negative) * limit);
    }
  }

  public static Climbers getInstance() {
    if (m_instance == null) {
      m_instance = new Climbers();
    }

    return m_instance;
  }

  @Override
  public void periodic() {
    for (Climber climber : m_climbers) {
      climber.periodic();
    }
  }

  @Override
  public void stop() {
    for (Climber climber : m_climbers) {
      climber.stop();
    }
  }

  @Override
  public void writePeriodicOutputs() {
    for (Climber climber : m_climbers) {
      climber.writePeriodicOutputs();
    }
  }

  @Override
  public void outputTelemetry() {
    for (Climber climber : m_climbers) {
      climber.outputTelemetry();
    }
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }
}
