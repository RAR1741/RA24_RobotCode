package frc.robot.subsystems;

public abstract class Subsystem {
  public void writeToLog() {
  }

  public void zeroSensors() {
  }

  public void reloadConfig() {
  }

  public abstract void periodic();

  public abstract void stop();

  public abstract void writePeriodicOutputs();

  public abstract void outputTelemetry();
}
