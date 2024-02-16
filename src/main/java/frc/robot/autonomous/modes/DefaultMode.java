package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.BrakeTask;
import frc.robot.autonomous.tasks.DriveForwardTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;

public class DefaultMode extends AutoModeBase {
  public void queueTasks() {
    queueTask(new ParallelTask(
        new PointForwardTask(),
        new WaitTask(0.5)));

    // queueTask(new ParallelTask(new DriveForwardTask(0.5, 1.0)));
    queueTask(new DriveForwardTask(2, 0.1));

    queueTask(new BrakeTask(false));
  }
}
