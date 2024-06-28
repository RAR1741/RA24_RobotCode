package frc.robot.autonomous.tasks;

import frc.robot.RobotTelemetry;

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
  public void prepare() {
    m_currentTask = m_tasks[m_index];

    m_currentTask.prepare();
  }

  @Override
  public void update() {
    if (m_currentTask.isFinished()) {
      m_currentTask.done();

      if (++m_index >= m_tasks.length) {
        m_isFinished = true;
        RobotTelemetry.print("Sequential task finished!");
        return;
      }

      m_currentTask = m_tasks[m_index];

      m_currentTask.prepare();
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
    RobotTelemetry.print("Sequential task done");
  }

  public Task[] getTasks() {
    return m_tasks;
  }
}
