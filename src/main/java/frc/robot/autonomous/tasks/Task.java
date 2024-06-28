package frc.robot.autonomous.tasks;

import org.littletonrobotics.junction.Logger;

public abstract class Task {
  public boolean m_isFinished = false;

  public abstract void prepare();

  public abstract void update();

  public void updateSim() {
  }

  public abstract boolean isFinished();

  public void done() {
  };

  public void log(boolean isRunning) {
    Logger.recordOutput("Auto/Tasks/" + this.getClass().getSimpleName(), isRunning);
  }
}
