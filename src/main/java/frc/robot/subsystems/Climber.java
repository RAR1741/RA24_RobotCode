package frc.robot.subsystems;

/**
 * 1 neo
 */

public class Climber extends Subsystem {
  private static Climber m_climber;

  private Climber() {

  }

  public static Climber getInstance() {
    if (m_climber == null) {
      m_climber = new Climber();
    }

    return m_climber;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void stop() {

  }

  @Override
  public void writePeriodicOutputs() {

  }

  @Override
  public void outputTelemetry() {

  }

}
