package frc.robot.autonomous.tasks;

/**
 * This task is for running tasks sequentially inside
 * an automode, for use when you want to run things
 * sequentially in a parallel task
 */
public class SequentialTask extends Task {
  private int m_index = 0;
  private Task[] m_tasks;
  private Task m_currentTask;
  private boolean m_finished = false;

  public SequentialTask(Task... tasks) {
    this.m_tasks = tasks;
    m_currentTask = m_tasks[m_index];
  }

  @Override
  public void start() {
    m_currentTask = m_tasks[m_index];

    m_currentTask.start();
  }

  @Override
  public void update() {
    if(m_currentTask.isFinished()) {
      m_currentTask.done();

      if(++m_index >= m_tasks.length) {
        m_finished = true;
        System.out.println("Sequential task finished!");
        return;
      }

      m_currentTask = m_tasks[m_index];

      m_currentTask.start();
    } else {
      m_currentTask.update();
    }
  }

  @Override
  public boolean isFinished() {
    return m_finished;
  }
}