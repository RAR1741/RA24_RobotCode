package frc.robot.autonomous.modes;

import frc.robot.autonomous.tasks.DriveTrajectoryTask;
import frc.robot.autonomous.tasks.ParallelTask;
import frc.robot.autonomous.tasks.PointForwardTask;
import frc.robot.autonomous.tasks.WaitTask;

public class TestMode extends AutoModeBase {
    @Override
    public void queueTasks() {
        queueTask(new ParallelTask(new DriveTrajectoryTask("pleasegodihope")));
        // queueTask(new DriveTrajectoryTask("pleasework"));
        queueTask(new PointForwardTask());
        queueTask(new WaitTask(10));
    }
}
