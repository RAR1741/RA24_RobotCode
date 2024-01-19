package frc.robot.subsystems;

public class Intake extends Subsystem {
  private static Intake m_intake;

  private Intake() {

  }

  public static Intake getInstance() {
    if (m_intake == null) {
      m_intake = new Intake();
    }

    return m_intake;
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
