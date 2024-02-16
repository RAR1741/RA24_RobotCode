package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.SequentialTask;
import frc.robot.autonomous.tasks.WaitTask;

public class TestMode2 extends AutoModeBase {
  @Override
  public void queueTasks() {
    // queueTask(new ParallelTask(new DriveTrajectoryTask("pleasework")));
    // queueTask(new DriveTrajectoryTask("pleasegodihope"));
    queueTask(new SequentialTask(
        new DriveTrajectoryTask("pleasework"),
        new DriveTrajectoryTask("pleasegodihope")));
    // queueTask(new PointForwardTask());
    queueTask(new WaitTask(10));
  }
}
