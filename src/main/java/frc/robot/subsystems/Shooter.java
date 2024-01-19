package frc.robot.subsystems;

public class Shooter extends Subsystem {
  private static Shooter m_shooter;

  private Shooter() {

  }

  public static Shooter getInstance() {
    if (m_shooter == null) {
      m_shooter = new Shooter();
    }

    return m_shooter;
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
