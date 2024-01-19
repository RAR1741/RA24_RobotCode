package frc.robot.subsystems;

public class Manipulation extends Subsystem {
  private static Manipulation m_manipulation;

  private Manipulation() {

  }

  public static Manipulation getInstance() {
    if (m_manipulation == null) {
      m_manipulation = new Manipulation();
    }

    return m_manipulation;
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
