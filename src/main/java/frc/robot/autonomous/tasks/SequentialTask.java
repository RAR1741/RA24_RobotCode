package frc.robot.autonomous.tasks;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This task is for running tasks sequentially inside
 * an automode, for use when you want to run things
 * sequentially in a parallel task
 */
public class SequentialTask extends Task {
  private int m_index = 0;
  private Task[] m_tasks;
  private Task m_currentTask;

  public SequentialTask(Task... tasks) {
    m_tasks = tasks;
    m_currentTask = m_tasks[m_index];
  }

  @Override
  public void start() {
    m_currentTask = m_tasks[m_index];

    m_currentTask.start();
  }

  @Override
  public void update() {
    if (m_currentTask.isFinished()) {
      m_currentTask.done();

      if (++m_index >= m_tasks.length) {
        m_isFinished = true;
        System.out.println("Sequential task finished!");
        return;
      }

      m_currentTask = m_tasks[m_index];

      m_currentTask.start();
    } else {
      m_currentTask.update();
      m_currentTask.updateSim();
    }
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }

  @Override
  public void done() {
    DriverStation.reportWarning("Sequential task done", false);
  }

  public Task[] getTasks() {
    return m_tasks;
  }
}
